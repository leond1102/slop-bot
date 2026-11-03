#!/usr/bin/env python3
"""FAST-LIO in den TF-Baum des Roboters einhängen (REP 105: map → odom → base_footprint).

FAST-LIO rechnet die Pose seiner IMU (`body`) im Startframe `camera_init`. Diese Node macht daraus die
Pose von base_footprint im Frame `map` (map = Roboterlage beim Start) und veröffentlicht:

  TF   map → camera_init       statisch, damit /cloud_registered, /Laser_map und /path in map liegen
  TF   map → odom              wenn die Radodometrie läuft (slop_base: odom → base_footprint):
                               korrigiert deren Drift, der Roboter steht in map dort, wo LIO ihn sieht
  TF   map → odom = Identität  wenn KEINE Radodometrie läuft, dazu selbst odom → base_footprint (aus LIO)
       + odom → base_footprint
  /lio/odom  nav_msgs/Odometry  Pose von base_footprint in map

Welcher Fall gilt, erkennt die Node selbst: Kommt auf /tf ein odom → base_footprint, das nicht von ihr
stammt, läuft die Radodometrie (wheel_timeout s lang gemerkt).

Die Montage base_footprint → livox_frame kommt aus der TF (slop_sensors/mount_tf). IMU und Punktwolke
des Livox liegen um extrinsic_T versetzt (wie in der FAST-LIO-Config).
"""
import collections

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformException, TransformListener

from slop_slam.transforms import Transform


def to_transform(tf):
    r, t = tf.transform.rotation, tf.transform.translation
    return Transform((r.x, r.y, r.z, r.w), (t.x, t.y, t.z))


def fill(msg_tf, T):
    msg_tf.translation.x, msg_tf.translation.y, msg_tf.translation.z = T.t
    msg_tf.rotation.x, msg_tf.rotation.y, msg_tf.rotation.z, msg_tf.rotation.w = T.q


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
            ('wheel_timeout', 0.5),                          # s ohne fremdes odom → base_footprint = keine Radodometrie
        ]:
            self.declare_parameter(name, default)

        ext = [float(v) for v in self._p('extrinsic_T')]
        self.T_lidar_imu = Transform(t=(-ext[0], -ext[1], -ext[2]))   # IMU-Ursprung im Lidar-Frame (R = I)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.static_pub = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/lio/odom', 10)

        self.T_map_ci = None
        self.own_stamps = collections.deque(maxlen=50)   # eigene odom → base_footprint, um fremde zu erkennen
        self.last_wheel = None
        self.mode = None

        self.create_subscription(TFMessage, '/tf', self.tf_callback, 100)
        self.create_subscription(Odometry, self._p('lio_odom_topic'), self.lio_callback, 10)
        self.get_logger().info(f'Warte auf {self._p("lio_odom_topic")} und TF '
                               f'{self._p("base_frame")} → {self._p("lidar_frame")}')

    def _p(self, name):
        return self.get_parameter(name).value

    def tf_callback(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == self._p('odom_frame') and t.child_frame_id == self._p('base_frame'):
                stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
                if stamp not in self.own_stamps:
                    self.last_wheel = self.get_clock().now()

    def wheel_odom_running(self):
        return (self.last_wheel is not None
                and self.get_clock().now() - self.last_wheel < Duration(seconds=self._p('wheel_timeout')))

    def lio_callback(self, msg):
        base, odom, mp = self._p('base_frame'), self._p('odom_frame'), self._p('map_frame')
        try:
            T_base_lidar = to_transform(self.tf_buffer.lookup_transform(base, self._p('lidar_frame'), Time()))
        except TransformException:
            self.get_logger().warn('Montage-TF fehlt (läuft slop_sensors?)', throttle_duration_sec=5.0)
            return
        T_base_imu = T_base_lidar * self.T_lidar_imu
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        T_ci_body = Transform((o.x, o.y, o.z, o.w), (p.x, p.y, p.z))
        T_map_ci = T_base_imu
        T_map_base = T_map_ci * T_ci_body * T_base_imu.inverse()

        # map → camera_init nur neu senden, wenn sich die Montage geändert hat (z. B. nach der Schwerkraftmessung)
        if self.T_map_ci is None or not self.T_map_ci.close_to(T_map_ci, 1e-5):
            self.T_map_ci = T_map_ci
            st = TransformStamped()
            st.header.stamp = msg.header.stamp
            st.header.frame_id = mp
            st.child_frame_id = self._p('lio_world_frame')
            fill(st.transform, T_map_ci)
            self.static_pub.sendTransform(st)

        stamp = msg.header.stamp
        out = []
        if self.wheel_odom_running():
            mode = 'Radodometrie läuft: LIO korrigiert sie über map → odom'
            try:
                T_odom_base = to_transform(self.tf_buffer.lookup_transform(odom, base, Time.from_msg(stamp),
                                                                           Duration(seconds=0.05)))
            except TransformException:
                try:
                    T_odom_base = to_transform(self.tf_buffer.lookup_transform(odom, base, Time()))
                except TransformException:
                    return
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, mp, odom
            fill(t.transform, T_map_base * T_odom_base.inverse())
            out.append(t)
        else:
            mode = 'keine Radodometrie: LIO liefert odom → base_footprint selbst'
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, mp, odom
            fill(t.transform, Transform())
            t2 = TransformStamped()
            t2.header.stamp, t2.header.frame_id, t2.child_frame_id = stamp, odom, base
            fill(t2.transform, T_map_base)
            self.own_stamps.append((stamp.sec, stamp.nanosec))
            out += [t, t2]
        self.tf_pub.sendTransform(out)
        if mode != self.mode:
            self.get_logger().info(mode)
            self.mode = mode

        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, mp, base
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = T_map_base.t
        (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
         od.pose.pose.orientation.z, od.pose.pose.orientation.w) = T_map_base.q
        od.pose.covariance = msg.pose.covariance
        self.odom_pub.publish(od)


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
