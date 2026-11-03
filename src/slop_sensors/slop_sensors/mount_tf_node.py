#!/usr/bin/env python3
"""Montage-TFs base_footprint → Sensor, Neigung (roll/pitch) automatisch aus der Schwerkraft der Sensor-IMUs.

    ros2 run slop_sensors mount_tf --ros-args -p mounts_file:=.../mounts.yaml -p sensors:="['livox','camera']"
    (normalerweise über sensors.launch.py)

Ablauf:
  1. Sofort: TFs mit den Werten aus mounts.yaml senden (damit alles gleich angezeigt wird).
  2. Für jeden Sensor mit `gravity: true` und `imu_topic`: `sample_time` s lang die Beschleunigung mitteln.
     Bewegt sich der Roboter dabei, wird neu gemessen. Dann roll/pitch aus der Schwerkraft setzen,
     x/y/z/yaw bleiben aus mounts.yaml (yaw ist aus der Schwerkraft nicht bestimmbar).
  3. Neu messen: ros2 service call /mount_tf/recalibrate std_srvs/srv/Trigger
  4. Laufend: slop_slam/mount_calib schickt aus LIO bessere Montagen auf ~/update (RealSense komplett per ICP,
     Lidar-yaw aus Geradeausfahrt). Die werden sofort übernommen und in calib_file gespeichert.
     Beim nächsten Start lädt mount_tf calib_file (falls vorhanden) statt der Werte aus mounts.yaml;
     roll/pitch misst es trotzdem neu aus der Schwerkraft.

Annahme: Der Roboter steht beim Messen still und WAAGRECHT (base_footprint z senkrecht).
Liegt die IMU in einem anderen Frame als der Montage-Frame (RealSense: camera_imu_optical_frame unter
camera_link), wird ihre Messung über die TF des Treibers in den Montage-Frame gedreht.

Topics:
  /tf_static              tf2_msgs/TFMessage   Pub  base_footprint → <frame> für alle Sensoren
  <imu_topic>             sensor_msgs/Imu      Sub  je Sensor aus mounts.yaml
  ~/update                geometry_msgs/TransformStamped  Sub  neue Montage base → Sensor (von mount_calib)
  ~/markers               visualization_msgs/MarkerArray  Pub  Anzeige in RViz (5 Hz):
                            orange = Schwerkraft live gemessen (im Sensorframe, geglättet)
                            grün   = senkrecht nach unten laut base_footprint
                            Text   = Status, gemessenes roll/pitch, Abweichung live
                          Liegen orange und grün übereinander, passt die Montage (Roboter waagrecht, in Ruhe).
Services:
  ~/recalibrate           std_srvs/Trigger     Neigung aller Sensoren neu messen
"""
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import Point, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from slop_sensors.gravity import StillWindow, quat_from_rpy, roll_pitch_from_up, rotate, rpy_from_quat


class Mount:
    """Ein Sensor aus mounts.yaml."""

    def __init__(self, name, cfg):
        self.name = name
        self.frame = cfg['frame']
        self.xyz = tuple(float(cfg.get(k, 0.0)) for k in ('x', 'y', 'z'))
        self.yaml_rpy = tuple(float(cfg.get(k, 0.0)) for k in ('roll', 'pitch', 'yaw'))
        self.rpy = self.yaml_rpy
        self.imu_topic = cfg.get('imu_topic', '')
        self.gravity = bool(cfg.get('gravity', False)) and bool(self.imu_topic)
        self.window = None
        self.done = not self.gravity
        self.start_time = None
        self.warned = False
        self.live = None          # geglättete Schwerkraftrichtung (nach oben) im Montage-Frame
        self.rms = None
        self.status = 'messe …' if self.gravity else 'aus mounts.yaml'
        self.source = 'mounts.yaml'   # Herkunft von x/y/z/yaw
        self.lio_updates = 0
        self.imu_count = 0            # empfangene IMU-Nachrichten (für das Ausdünnen nach der Messung)
        self.imu_rot = None           # Drehung IMU-Frame → Montage-Frame (statisch, einmal nachgeschlagen)

    def load_calib(self, c):
        """Gespeicherte LIO-Kalibrierung übernehmen: nur die Drehung (roll/pitch misst die Schwerkraft danach
        neu), die Position bleibt aus mounts.yaml (nachgemessen ist sie genauer als aus LIO/ICP)."""
        self.yaml_rpy = tuple(float(c[k]) for k in ('roll', 'pitch', 'yaw'))
        self.rpy = self.yaml_rpy
        self.source = 'Kalibrierdatei'


class MountTfNode(Node):

    def __init__(self):
        super().__init__('mount_tf')
        for name, default in [
            ('mounts_file', ''),
            ('sensors', ['']),           # Einträge aus mounts.yaml, leer = alle
            ('base_frame', 'base_footprint'),
            ('gravity', True),           # false = nur mounts.yaml, keine Messung
            ('sample_time', 2.0),        # s mitteln
            ('max_rms_deg', 2.0),        # Streuung der Richtung, darüber gilt der Sensor als bewegt
            ('wait_warn', 10.0),         # s ohne Ergebnis → Warnung (IMU fehlt? Roboter bewegt?)
            ('markers', True),           # Anzeige in RViz auf ~/markers
            ('arrow_length', 0.3),       # m
            ('live_smoothing', 0.05),    # Tiefpass der Live-Richtung je IMU-Messung (0..1, kleiner = ruhiger)
            ('calib_file', ''),          # gespeicherte LIO-Kalibrierung (slop_slam/mount_calib), '' = keine
            ('live_every', 10),          # nach der Messung nur jede n-te IMU-Nachricht auswerten (nur für die Anzeige)
        ]:
            self.declare_parameter(name, default)

        with open(self._p('mounts_file')) as f:
            cfg = yaml.safe_load(f)
        wanted = [s for s in self._p('sensors') if s]
        self.mounts = [Mount(n, c) for n, c in cfg.items() if not wanted or n in wanted]
        if not self._p('gravity'):
            for m in self.mounts:
                m.gravity, m.done = False, True
        calib_file = os.path.expanduser(self._p('calib_file'))
        if calib_file and os.path.exists(calib_file):
            with open(calib_file) as f:
                calib = yaml.safe_load(f) or {}
            for m in self.mounts:
                if m.frame in calib:
                    m.load_calib(calib[m.frame])
                    self.get_logger().info(f'{m.name}: Montage aus {calib_file}')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_all()
        for m in self.mounts:
            src = 'Schwerkraft (' + m.imu_topic + ')' if m.gravity else 'mounts.yaml'
            self.get_logger().info(f'{m.name}: {self._p("base_frame")} → {m.frame}, roll/pitch aus {src}')

        for m in self.mounts:
            if m.gravity:
                # roh empfangen: 200 Hz je IMU in Python zu entpacken kostet viel CPU, entpackt wird nur, was gebraucht wird
                self.create_subscription(Imu, m.imu_topic, lambda data, m=m: self.imu_raw(m, data),
                                         qos_profile_sensor_data, raw=True)
        self.live_every = max(1, int(self._p('live_every')))
        self.live_k = self._p('live_smoothing')
        self.create_subscription(TransformStamped, '~/update', self.update_callback, 10)
        self.create_service(Trigger, '~/recalibrate', self.recalibrate_callback)
        self.create_timer(1.0, self.check_waiting)
        if self._p('markers'):
            self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
            self.create_timer(0.2, self.publish_markers)
        self.recalibrate()

    def _p(self, name):
        return self.get_parameter(name).value

    def recalibrate(self):
        now = self.get_clock().now()
        for m in self.mounts:
            if m.gravity:
                m.window = StillWindow(self._p('sample_time'), self._p('max_rms_deg'))
                m.done = False
                m.start_time = now
                m.warned = False
                m.status = 'messe …'

    def update_callback(self, msg):
        m = next((m for m in self.mounts if m.frame == msg.child_frame_id), None)
        if m is None or msg.header.frame_id != self._p('base_frame'):
            return
        t, r = msg.transform.translation, msg.transform.rotation
        m.xyz = (t.x, t.y, t.z)
        m.rpy = rpy_from_quat((r.x, r.y, r.z, r.w))
        m.yaml_rpy = (m.yaml_rpy[0], m.yaml_rpy[1], m.rpy[2])   # yaw für spätere Schwerkraftmessungen behalten
        m.done = True
        m.lio_updates += 1
        m.source = 'LIO'
        m.status = f'aus LIO ({m.lio_updates}×)'
        self.publish_all()

    def recalibrate_callback(self, request, response):
        self.recalibrate()
        response.success = True
        response.message = 'Messung gestartet, Roboter ' + f'{self._p("sample_time"):g} s still halten'
        return response

    def imu_raw(self, m, data):
        # Während der Schwerkraftmessung jede Nachricht, danach nur jede live_every-te (Anzeige)
        m.imu_count += 1
        if m.done and m.imu_count % self.live_every:
            return
        self.imu_callback(m, deserialize_message(data, Imu))

    def imu_callback(self, m, msg):
        a = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        # Messung in den Montage-Frame drehen, falls die IMU einen eigenen Frame hat (statisch → einmal nachschlagen)
        imu_frame = msg.header.frame_id or m.frame
        if imu_frame != m.frame:
            if m.imu_rot is None:
                try:
                    r = self.tf_buffer.lookup_transform(m.frame, imu_frame, Time()).transform.rotation
                except TransformException:
                    return   # Treiber-TF noch nicht da
                m.imu_rot = (r.x, r.y, r.z, r.w)
            a = rotate(m.imu_rot, a)

        # Live-Richtung für die Anzeige (Betrag egal: Livox misst in g, RealSense in m/s²).
        # Glättung je ausgewerteter Nachricht so, dass sie pro Sekunde gleich stark bleibt wie ohne Ausdünnen.
        n = math.sqrt(sum(c * c for c in a))
        if n > 1e-6:
            u = tuple(c / n for c in a)
            k = self.live_k if not m.done else 1 - (1 - self.live_k) ** self.live_every
            m.live = u if m.live is None else tuple((1 - k) * lv + k * uv for lv, uv in zip(m.live, u))

        if m.done:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        result = m.window.add(stamp, a)
        if result is None:
            return
        still, mean, rms = result
        m.rms = rms
        if not still:
            m.status = f'bewegt ({rms:.1f}°), messe neu …'
            self.get_logger().warn(f'{m.name}: bewegt (Streuung {rms:.1f}°), messe neu', throttle_duration_sec=5.0)
            return
        roll, pitch = roll_pitch_from_up(*mean)
        m.rpy = (roll, pitch, m.yaml_rpy[2])
        m.done = True
        m.status = 'aus Schwerkraft'
        self.publish_all()
        self.get_logger().info(
            f'{m.name}: roll {math.degrees(roll):+.1f}°, pitch {math.degrees(pitch):+.1f}° aus Schwerkraft '
            f'(Streuung {rms:.2f}°; mounts.yaml: roll {math.degrees(m.yaml_rpy[0]):+.1f}°, '
            f'pitch {math.degrees(m.yaml_rpy[1]):+.1f}°), yaw {math.degrees(m.rpy[2]):+.1f}° aus mounts.yaml')

    def check_waiting(self):
        now = self.get_clock().now()
        for m in self.mounts:
            if m.done or m.warned:
                continue
            if now - m.start_time > Duration(seconds=self._p('wait_warn')):
                self.get_logger().warn(f'{m.name}: nach {self._p("wait_warn"):g} s noch keine ruhige Messung auf '
                                       f'{m.imu_topic} – IMU aus oder Roboter bewegt? Bis dahin gilt mounts.yaml.')
                m.warned = True
                if m.live is None:
                    m.status = f'keine Daten auf {m.imu_topic}, mounts.yaml'

    def publish_markers(self):
        base = self._p('base_frame')
        length = self._p('arrow_length')
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def marker(ns, mid, frame, mtype):
            mk = Marker()
            mk.header.stamp, mk.header.frame_id = stamp, frame
            mk.ns, mk.id, mk.type, mk.action = ns, mid, mtype, Marker.ADD
            mk.pose.orientation.w = 1.0
            mk.lifetime = Duration(seconds=1.0).to_msg()   # verschwindet, wenn die Node nicht mehr läuft
            mk.frame_locked = True
            return mk

        def arrow(ns, mid, frame, start, end, rgb):
            mk = marker(ns, mid, frame, Marker.ARROW)
            mk.points = [Point(x=start[0], y=start[1], z=start[2]), Point(x=end[0], y=end[1], z=end[2])]
            mk.scale.x, mk.scale.y, mk.scale.z = 0.015, 0.035, 0.05   # Schaft, Spitze Breite, Spitze Länge
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = rgb[0], rgb[1], rgb[2], 0.9
            return mk

        for i, m in enumerate(self.mounts):
            ns = f'gravity/{m.name}'
            # grün: senkrecht nach unten laut Roboter, am Sensor angesetzt
            x, y, z = m.xyz
            arr.markers.append(arrow(ns, 0, base, (x, y, z), (x, y, z - length), (0.2, 0.9, 0.2)))

            lines = [f'{m.name}: {m.status}',
                     f'roll {math.degrees(m.rpy[0]):+.1f}°  pitch {math.degrees(m.rpy[1]):+.1f}°'
                     f'  yaw {math.degrees(m.rpy[2]):+.1f}°',
                     f'x {m.xyz[0]:+.3f}  y {m.xyz[1]:+.3f}  z {m.xyz[2]:+.3f} m  ({m.source})']
            if m.live is not None:
                # orange: gemessene Schwerkraft (nach unten) im Sensorframe
                down = tuple(-c * length for c in m.live)
                arr.markers.append(arrow(ns, 1, m.frame, (0.0, 0.0, 0.0), down, (1.0, 0.55, 0.0)))
                # Abweichung live: gemessene Richtung, mit der aktuellen Montage in base gedreht, gegen base -z
                up_base = rotate(quat_from_rpy(*m.rpy), m.live)
                dev = math.degrees(math.acos(max(-1.0, min(1.0, up_base[2]))))
                lines.append(f'Abweichung live {dev:.1f}°')
            text = marker(ns, 2, base, Marker.TEXT_VIEW_FACING)
            text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, z + 0.15 + 0.22 * i
            text.scale.z = 0.04
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = '\n'.join(lines)
            arr.markers.append(text)
        self.marker_pub.publish(arr)

    def publish_all(self):
        # Alle in EINER Nachricht: /tf_static hält nur die letzte Nachricht pro Publisher
        stamp = self.get_clock().now().to_msg()
        msgs = []
        for m in self.mounts:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._p('base_frame')
            t.child_frame_id = m.frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = m.xyz
            q = quat_from_rpy(*m.rpy)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            msgs.append(t)
        self.broadcaster.sendTransform(msgs)


def main(args=None):
    rclpy.init(args=args)
    node = MountTfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise          # beim Beenden (Strg+C) wirft rclpy manchmal noch beim Abholen einer Nachricht
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
