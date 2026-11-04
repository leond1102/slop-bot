#!/usr/bin/env python3
"""FAST-LIO in den TF-Baum des Roboters einhängen (REP 105: map → odom → base_footprint).

Keine Radodometrie: Die Roboterpose kommt allein aus LIO.

FAST-LIO rechnet die Pose seiner IMU (`body`) im Startframe `camera_init`. Diese Node macht daraus die
Pose von base_footprint im Frame `map` (map = Roboterlage beim Start) und veröffentlicht:

  TF   map → camera_init       statisch, damit /cloud_registered, /lio/map und /path in map liegen
  TF   map → odom              statisch, Identität (nur eine Posenquelle, also keine Drift zwischen beiden)
  TF   odom → base_footprint   LIO-Pose zur Scanzeit, dazwischen `predict_rate`-mal je Sekunde bis jetzt
                               vorausgerechnet (siehe unten)
  /lio/odom  nav_msgs/Odometry  Pose von base_footprint in map (Scanzeit), Geschwindigkeit in base_footprint

Verzögerung: Eine LIO-Pose ist beim Eintreffen schon ~0,1–0,2 s alt (Scan sammeln, rechnen). Ohne Vorausrechnung
hinkte alles in odom/map so weit hinterher: TF-Abfragen für „jetzt“ (Kamerabilder, RViz) mussten auf die
nächste LIO-Pose warten. Deshalb schreibt diese Node die letzte Pose mit der mittleren Geschwindigkeit der letzten
`velocity_window` s fort (konstant im Roboterframe, Kreisbogen), höchstens `max_predict` s weit. Hängt LIO länger,
bleibt die TF stehen (veraltet sichtbar statt falsch). Jede neue LIO-Pose ersetzt die Schätzung.

Die Montage base_footprint → livox_frame kommt aus /tf_static (slop_sensors/mount_tf). IMU und Punktwolke
des Livox liegen um extrinsic_T versetzt (wie in der FAST-LIO-Config).
"""
import collections

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformException

from slop_slam.transforms import Transform, body_twist, predict


def to_transform(tf):
    r, t = tf.transform.rotation, tf.transform.translation
    return Transform((r.x, r.y, r.z, r.w), (t.x, t.y, t.z))


def fill(msg_tf, T):
    msg_tf.translation.x, msg_tf.translation.y, msg_tf.translation.z = T.t
    msg_tf.rotation.x, msg_tf.rotation.y, msg_tf.rotation.z, msg_tf.rotation.w = T.q


def sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LioTfNode(Node):

    def __init__(self):
        super().__init__('lio_tf')
        for name, default in [
            ('lio_odom_topic', '/Odometry'),
            ('lio_world_frame', 'camera_init'),
            ('map_frame', 'map'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_footprint'),
            ('lidar_frame', 'livox_frame'),
            ('extrinsic_T', [-0.011, -0.02329, 0.04412]),   # Lidar im IMU-Frame, wie FAST-LIO mapping.extrinsic_T
            ('freeze_map_after', 10.0),   # s: danach bleibt map → camera_init fest, spätere Montage-Korrekturen
                                          # (mount_tf/recalibrate, mount_calib) verschieben den Roboter in map,
                                          # nicht die Karte
            ('predict_rate', 30.0),       # Hz, vorausgerechnete odom → base_footprint; 0 = nur LIO-Posen (verzögert)
            ('max_predict', 0.5),         # s, höchstens so weit über die letzte LIO-Pose hinaus
            ('velocity_window', 0.5),     # s, Geschwindigkeit über diese Spanne gemittelt (LIO-Rauschen ~mm)
        ]:
            self.declare_parameter(name, default)

        ext = [float(v) for v in self._p('extrinsic_T')]
        self.T_lidar_imu = Transform(t=(-ext[0], -ext[1], -ext[2]))   # IMU-Ursprung im Lidar-Frame (R = I)

        # Gebraucht wird nur die Montage (statisch): nur /tf_static lesen. Ein TransformListener läse auch /tf
        # (die eigenen 30 Hz, FAST-LIO) und kostete in Python CPU für nichts.
        self.tf_buffer = Buffer()
        self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_pub = TransformBroadcaster(self)
        self.static_pub = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/lio/odom', 10)

        self.T_map_ci = None
        self.first_time = None
        self.frozen = False
        self.history = collections.deque(maxlen=50)   # (Zeit [s], T_map_base) der letzten LIO-Posen
        self.last_lio_wall = None

        self.create_subscription(Odometry, self._p('lio_odom_topic'), self.lio_callback, 10)
        if self._p('predict_rate') > 0:
            self.create_timer(1.0 / self._p('predict_rate'), self.predict_step)
        self.get_logger().info(f'Warte auf {self._p("lio_odom_topic")} und TF '
                               f'{self._p("base_frame")} → {self._p("lidar_frame")}')

    def _p(self, name):
        return self.get_parameter(name).value

    def tf_static_callback(self, msg):
        for t in msg.transforms:
            self.tf_buffer.set_transform_static(t, 'lio_tf')

    def send_static(self):
        """map → camera_init und map → odom in EINER Nachricht (/tf_static hält nur die letzte je Publisher)."""
        out = []
        for child, T in ((self._p('lio_world_frame'), self.T_map_ci), (self._p('odom_frame'), Transform())):
            st = TransformStamped()
            st.header.stamp = self.get_clock().now().to_msg()
            st.header.frame_id = self._p('map_frame')
            st.child_frame_id = child
            fill(st.transform, T)
            out.append(st)
        self.static_pub.sendTransform(out)

    def odom_tf(self, stamp, T):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self._p('odom_frame'), self._p('base_frame')
        fill(t.transform, T)
        return t

    def velocity_reference(self):
        """Ältere Pose (≥ velocity_window vor der neuesten) für die Geschwindigkeit, None = noch zu kurz."""
        t1 = self.history[-1][0]
        return next(((t, T) for t, T in reversed(self.history) if t1 - t >= self._p('velocity_window')), None)

    def lio_callback(self, msg):
        try:
            T_base_lidar = to_transform(self.tf_buffer.lookup_transform(self._p('base_frame'), self._p('lidar_frame'),
                                                                        Time()))
        except TransformException:
            self.get_logger().warn('Montage-TF fehlt (läuft slop_sensors?)', throttle_duration_sec=5.0)
            return
        T_base_imu = T_base_lidar * self.T_lidar_imu
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        T_ci_body = Transform((o.x, o.y, o.z, o.w), (p.x, p.y, p.z))
        now = self.get_clock().now()
        if self.first_time is None:
            self.first_time = now
        if not self.frozen and now - self.first_time > Duration(seconds=self._p('freeze_map_after')):
            self.frozen = True
            self.get_logger().info('map festgelegt (map → camera_init bleibt jetzt fest)')
        T_map_ci = self.T_map_ci if self.frozen else T_base_imu
        T_map_base = T_map_ci * T_ci_body * T_base_imu.inverse()

        # Statische TFs nur neu senden, wenn sich die Montage geändert hat (vor dem Einfrieren)
        if self.T_map_ci is None or not self.T_map_ci.close_to(T_map_ci, 1e-5):
            self.T_map_ci = T_map_ci
            self.send_static()

        stamp = msg.header.stamp
        t = sec(stamp)
        if self.history and t <= self.history[-1][0]:
            return                                   # doppelt oder rückwärts (Neustart von FAST-LIO?)
        self.history.append((t, T_map_base))
        self.last_lio_wall = now
        latency = sec(now.to_msg()) - t
        if not -0.05 < latency <= self._p('max_predict'):
            self.get_logger().warn(f'LIO-Pose kommt {latency:+.2f} s nach ihrem Zeitstempel an (Lidar-Uhr nicht '
                                   'synchron oder LIO überlastet?) – Vorausrechnung aus', throttle_duration_sec=10.0)
        self.tf_pub.sendTransform(self.odom_tf(stamp, T_map_base))

        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, self._p('map_frame'), self._p('base_frame')
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = T_map_base.t
        (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
         od.pose.pose.orientation.z, od.pose.pose.orientation.w) = T_map_base.q
        od.pose.covariance = msg.pose.covariance
        ref = self.velocity_reference()
        if ref is not None:
            v, w = body_twist(ref[1], ref[0], T_map_base, t)
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = v
            od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = w
        self.odom_pub.publish(od)

    def predict_step(self):
        if not self.history:
            return
        now = self.get_clock().now()
        t_now = sec(now.to_msg())
        t_b, T_b = self.history[-1]
        horizon = t_now - t_b
        if not 0.0 < horizon <= self._p('max_predict'):
            if horizon > self._p('max_predict') and now - self.last_lio_wall > Duration(seconds=1.0):
                self.get_logger().warn('Keine LIO-Pose seit über 1 s – odom → base_footprint steht',
                                       throttle_duration_sec=10.0)
            return
        ref = self.velocity_reference()
        T = predict(ref[1], ref[0], T_b, t_b, t_now) if ref is not None else T_b
        self.tf_pub.sendTransform(self.odom_tf(now.to_msg(), T))


def main(args=None):
    rclpy.init(args=args)
    node = LioTfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
