#!/usr/bin/env python3
"""Brücke FAST-LIO → TF-Baum und EKF.

  /Odometry   nav_msgs/Odometry  Sub  FAST-LIO: Pose des IMU-Frames in camera_init (Welt beim Start, schwerkraftausgerichtet)
  /lio/odom   nav_msgs/Odometry  Pub  Pose von base_footprint in map, Geschwindigkeit im Roboterframe (aus den Posen
                                       gerechnet, FAST-LIO liefert keine). Eingang odom1 des EKF (nur die Geschwindigkeit).
  TF          map → odom         Pose aus LIO relativ zur stetigen EKF-Odometrie: map_T_odom = map_T_base · odom_T_base⁻¹
  TF          map → camera_init  Welt von FAST-LIO in map (damit /cloud_registered mit clouds:=true in RViz erscheint)
  /lio/crop_boxes  MarkerArray   Pub  die aus den Scans geschnittenen Quader (Parameter crop_boxes, im Lidar-Frame) für RViz

map = Lage von base_footprint beim ersten LIO-Wert (Position und Gier, Roll/Pitch bleiben schwerkraftausgerichtet).
Warum map → odom und nicht odom → base_footprint: Die EKF-Odometrie (Rad + Gyro + LIO-Geschwindigkeit) bleibt stetig, Sprünge
oder Drift der LIO-Pose landen in map → odom. In den EKF geht deshalb nur die LIO-Geschwindigkeit (Roboterframe), nie die
Pose in map: eine Pose in map bräuchte map → odom, und das wird aus dem EKF-Ergebnis gebildet (Zirkelbezug).

Braucht das TF base_footprint → livox_frame (robot_state_publisher, URDF) und odom → base_footprint (EKF).
Ohne den EKF läuft /lio/odom trotzdem, nur map → odom fehlt dann.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from slop_lio.crop import center_and_size
from slop_lio.lio_math import (body_twist, inverse, planar_inverse, planar_map_to_odom, to_matrix, to_pose,
                               with_translation)


def tf_to_matrix(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)


class LioBridge(Node):
    def __init__(self):
        super().__init__('lio_bridge')
        for name, default in [
            ('lio_topic', '/Odometry'),
            ('out_topic', '/lio/odom'),
            ('map_frame', 'map'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_footprint'),
            ('sensor_frame', 'livox_frame'),          # Lidar-Frame im URDF
            ('imu_to_lidar_xyz', [-0.011, -0.02329, 0.04412]),   # Lidar im IMU-Frame (extrinsic_T von FAST-LIO)
            ('crop_boxes', Parameter.Type.DOUBLE_ARRAY),   # xmin,xmax,ymin,ymax,zmin,zmax je Quader (von lio.launch.py)
            ('publish_map_odom', True),
            ('camera_init_frame', 'camera_init'),     # Welt-Frame von FAST-LIO ('' = nicht senden)
            ('tf_future_s', 0.1),                     # s: map → odom in die Zukunft datieren (Pose ist ~0,1 s alt)
            ('twist_stddev', 0.04),                   # m/s, Geschwindigkeit aus Posendifferenzen (10 Hz)
            ('yaw_rate_stddev', 0.02),                # rad/s
            ('max_speed', 0.6),                       # m/s: schneller als der Roboter kann → Geschwindigkeit verwerfen
            ('max_yaw_rate', 2.0),                    # rad/s
        ]:
            self.declare_parameter(name, default)
        self.imu_to_lidar = with_translation(self._p('imu_to_lidar_xyz'))
        self.align = None                  # map_T_camera_init
        self.sensor_to_base = None         # livox_frame ← base_footprint (statisch)
        self.prev = None                   # (Stempel [s], map_T_base) der letzten Pose
        self.warned = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, self._p('out_topic'), 10)
        self.create_subscription(Odometry, self._p('lio_topic'), self.callback, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/lio/crop_boxes',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.crop_markers = self._crop_markers()
        if self.crop_markers.markers:
            self.marker_pub.publish(self.crop_markers)
            self.create_timer(2.0, lambda: self.marker_pub.publish(self.crop_markers))   # RViz, das später startet
        self.get_logger().info(f'{self._p("lio_topic")} → {self._p("out_topic")}, TF {self._p("map_frame")} → '
                               f'{self._p("odom_frame")}; map = Startlage von {self._p("base_frame")}')

    def _p(self, name):
        return self.get_parameter(name).value

    def _crop_markers(self):
        """Die geschnittenen Quader als durchsichtige Würfel im Lidar-Frame (0 = Körper, 1 = hinter dem Lidar)."""
        values = list(self.get_parameter('crop_boxes').get_parameter_value().double_array_value)
        out = MarkerArray()
        colors = ((1.0, 0.5, 0.0), (1.0, 0.0, 0.0))
        for i in range(len(values) // 6):
            center, size = center_and_size(values[6 * i:6 * i + 6])
            m = Marker()
            m.header.frame_id = self._p('sensor_frame')
            m.ns, m.id, m.type, m.action = 'crop', i, Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = center
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = size
            m.color.r, m.color.g, m.color.b = colors[i % len(colors)]
            m.color.a = 0.25
            out.markers.append(m)
        return out

    def _warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def _sensor_to_base(self):
        """Pose von base_footprint im Lidar-Frame (statisch, aus dem URDF); None, solange das TF fehlt."""
        if self.sensor_to_base is None:
            try:
                tf = self.tf_buffer.lookup_transform(self._p('sensor_frame'), self._p('base_frame'), Time())
                self.sensor_to_base = tf_to_matrix(tf)
            except Exception:
                self._warn_once('urdf', f'Warte auf TF {self._p("base_frame")} → {self._p("sensor_frame")} '
                                        '(robot_state_publisher, bringup.launch.py)')
        return self.sensor_to_base

    def callback(self, msg):
        sensor_to_base = self._sensor_to_base()
        if sensor_to_base is None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        camera_init_to_base = to_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w) @ self.imu_to_lidar @ sensor_to_base
        if self.align is None:
            self.align = planar_inverse(camera_init_to_base)
            self.get_logger().info('map festgelegt: Startlage von base_footprint in der LIO-Welt')
        map_to_base = self.align @ camera_init_to_base
        stamp = Time.from_msg(msg.header.stamp)
        t = stamp.nanoseconds * 1e-9

        vx = vy = wz = 0.0
        twist_ok = False
        if self.prev is not None:
            dt = t - self.prev[0]
            if 0.02 < dt < 0.5:
                vx, vy, wz = body_twist(self.prev[1], map_to_base, dt)
                twist_ok = math.hypot(vx, vy) <= self._p('max_speed') and abs(wz) <= self._p('max_yaw_rate')
                if not twist_ok:
                    self.get_logger().warn(f'LIO-Geschwindigkeit unplausibel (v {math.hypot(vx, vy):.2f} m/s, '
                                           f'w {wz:.2f} rad/s) – nicht in den EKF übernommen', throttle_duration_sec=2.0)
        self.prev = (t, map_to_base)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._p('map_frame')
        out.child_frame_id = self._p('base_frame')
        x, y, z, qx, qy, qz, qw = to_pose(map_to_base)
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = x, y, z
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = qx, qy
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = qz, qw
        for i, v in ((0, 0.01), (7, 0.01), (14, 1e6), (21, 1e6), (28, 1e6), (35, 0.02)):
            out.pose.covariance[i] = v
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.angular.z = vx, 0.0, wz
        big = 1e6
        s, sw = self._p('twist_stddev') ** 2, self._p('yaw_rate_stddev') ** 2
        for i, v in ((0, s if twist_ok else big), (7, big), (14, big), (21, big), (28, big),
                     (35, sw if twist_ok else big)):
            out.twist.covariance[i] = v
        self.pub.publish(out)

        if self._p('camera_init_frame'):
            self._send_tf(self.align, self._p('map_frame'), self._p('camera_init_frame'))
        if self._p('publish_map_odom'):
            self._publish_map_to_odom(map_to_base, stamp)

    def _publish_map_to_odom(self, map_to_base, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(self._p('odom_frame'), self._p('base_frame'), stamp,
                                                 timeout=Duration(seconds=0.05))
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(self._p('odom_frame'), self._p('base_frame'), Time())
            except Exception:
                self._warn_once('ekf', f'Kein TF {self._p("odom_frame")} → {self._p("base_frame")} (EKF läuft nicht?) – '
                                       f'{self._p("map_frame")} → {self._p("odom_frame")} wird nicht gesendet')
                return
        self._send_tf(planar_map_to_odom(map_to_base, tf_to_matrix(tf)), self._p('map_frame'), self._p('odom_frame'))

    def _send_tf(self, matrix, parent, child):
        x, y, z, qx, qy, qz, qw = to_pose(matrix)
        out = TransformStamped()
        out.header.stamp = (self.get_clock().now() + Duration(seconds=self._p('tf_future_s'))).to_msg()
        out.header.frame_id = parent
        out.child_frame_id = child
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = x, y, z
        out.transform.rotation.x, out.transform.rotation.y = qx, qy
        out.transform.rotation.z, out.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(out)


def main(args=None):
    rclpy.init(args=args)
    node = LioBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:        # zweites SIGINT vom Launch während des Aufräumens
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
