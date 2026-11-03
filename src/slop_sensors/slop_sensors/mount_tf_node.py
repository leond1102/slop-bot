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

Annahme: Der Roboter steht beim Messen still und WAAGRECHT (base_footprint z senkrecht).
Liegt die IMU in einem anderen Frame als der Montage-Frame (RealSense: camera_imu_optical_frame unter
camera_link), wird ihre Messung über die TF des Treibers in den Montage-Frame gedreht.

Topics:
  /tf_static              tf2_msgs/TFMessage   Pub  base_footprint → <frame> für alle Sensoren
  <imu_topic>             sensor_msgs/Imu      Sub  je Sensor aus mounts.yaml
Services:
  ~/recalibrate           std_srvs/Trigger     Neigung aller Sensoren neu messen
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from slop_sensors.gravity import StillWindow, quat_from_rpy, roll_pitch_from_up, rotate


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
        ]:
            self.declare_parameter(name, default)

        with open(self._p('mounts_file')) as f:
            cfg = yaml.safe_load(f)
        wanted = [s for s in self._p('sensors') if s]
        self.mounts = [Mount(n, c) for n, c in cfg.items() if not wanted or n in wanted]
        if not self._p('gravity'):
            for m in self.mounts:
                m.gravity, m.done = False, True

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_all()
        for m in self.mounts:
            src = 'Schwerkraft (' + m.imu_topic + ')' if m.gravity else 'mounts.yaml'
            self.get_logger().info(f'{m.name}: {self._p("base_frame")} → {m.frame}, roll/pitch aus {src}')

        for m in self.mounts:
            if m.gravity:
                self.create_subscription(Imu, m.imu_topic, lambda msg, m=m: self.imu_callback(m, msg),
                                         qos_profile_sensor_data)
        self.create_service(Trigger, '~/recalibrate', self.recalibrate_callback)
        self.create_timer(1.0, self.check_waiting)
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

    def recalibrate_callback(self, request, response):
        self.recalibrate()
        response.success = True
        response.message = 'Messung gestartet, Roboter ' + f'{self._p("sample_time"):g} s still halten'
        return response

    def imu_callback(self, m, msg):
        if m.done:
            return
        a = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        # Messung in den Montage-Frame drehen, falls die IMU einen eigenen Frame hat
        imu_frame = msg.header.frame_id or m.frame
        if imu_frame != m.frame:
            try:
                t = self.tf_buffer.lookup_transform(m.frame, imu_frame, Time())
            except TransformException:
                return   # Treiber-TF noch nicht da
            r = t.transform.rotation
            a = rotate((r.x, r.y, r.z, r.w), a)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        result = m.window.add(stamp, a)
        if result is None:
            return
        still, mean, rms = result
        if not still:
            self.get_logger().warn(f'{m.name}: bewegt (Streuung {rms:.1f}°), messe neu', throttle_duration_sec=5.0)
            return
        roll, pitch = roll_pitch_from_up(*mean)
        m.rpy = (roll, pitch, m.yaml_rpy[2])
        m.done = True
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
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
