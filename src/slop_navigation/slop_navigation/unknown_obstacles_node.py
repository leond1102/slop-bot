#!/usr/bin/env python3
"""Dynamisches Sicherheitsfeld: gibt nur die Scanpunkte weiter, die NICHT auf bekannter Struktur liegen.

  /scan                   sensor_msgs/LaserScan            Sub  (base_footprint)
  /map                    nav_msgs/OccupancyGrid           Sub  gespeicherte Karte (transient local)
  /experience/known       nav_msgs/OccupancyGrid           Sub  Erfahrung: 100 fest, 50 vielleicht, 0 frei, -1 ?
  /relocalize/score       std_msgs/Float32                 Sub  Lokalisierung sicher?
  /amcl_pose              geometry_msgs/PoseWithCovStamped Sub  Lokalisierung sicher?
  /safety/unknown_points  sensor_msgs/PointCloud2          Pub  unbekannte Hindernisse (Scan-Frame, Scan-Zeitstempel)

Der zweite Collision Monitor (collision_monitor_dynamic) sieht nur diese Punkte und die Kamera-Hindernisse und hat die
größeren Zonen; der erste (collision_monitor) sieht alles und hat nur sehr kleine Zonen. Wände passiert der Roboter so eng,
Menschen und Stühle mit Abstand.

Im Zweifel unbekannt: Ohne Karte, ohne TF, bei unsicherer Lokalisierung oder außerhalb der Karte werden ALLE Punkte
weitergegeben – der Roboter fährt dann so vorsichtig wie bei lauter Menschen, nie mutiger.
"""
import array
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

from slop_navigation.safety_core import is_known, known_mask, outside_body, scan_points, transform


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class UnknownObstacles(Node):
    def __init__(self):
        super().__init__('unknown_obstacles')
        for name, default in [
            ('map_frame', 'map'),
            ('max_range', 3.0),             # m: weiter weg liegt keine Zone
            ('known_dilate', 0.10),         # m: so weit um bekannte Struktur gilt ein Punkt noch als bekannt
            ('z', 0.5),                     # m: Höhe der Ausgabepunkte (Collision Monitor filtert nach Höhe)
            # Footprint (base_footprint) und Zuschlag: Punkte darin sind Reste des eigenen Körpers
            ('body_x_min', -0.40), ('body_x_max', 0.10), ('body_half_width', 0.16), ('body_margin', 0.03),
            # Lokalisierung muss sicher sein, sonst gilt alles als unbekannt
            ('min_score', 0.60), ('score_timeout', 3.0), ('max_pos_std', 0.30), ('max_yaw_std_deg', 10.0),
            ('report_period', 60.0),
        ]:
            self.declare_parameter(name, default)
        self.map_msg = None
        self.exp_msg = None
        self.known = None
        self.score = None
        self.amcl_cov = None
        self.mode = None
        self.stats = {'scans': 0, 'punkte': 0, 'bekannt': 0, 'alles unbekannt': 0}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(OccupancyGrid, '/experience/known', self.on_experience, latched)
        self.create_subscription(Float32, '/relocalize/score', lambda m: setattr(self, 'score', (m.data, time.monotonic())), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, '/safety/unknown_points', qos_profile_sensor_data)
        self.create_timer(self._p('report_period'), self.report)
        self.get_logger().info('Dynamisches Sicherheitsfeld bereit: /scan → /safety/unknown_points '
                               '(bis Karte und Lokalisierung da sind, gilt alles als unbekannt)')

    def _p(self, name):
        return self.get_parameter(name).value

    # ---------------------------------------------------------------- Karten
    def on_map(self, msg):
        self.map_msg = msg
        self.rebuild()

    def on_experience(self, msg):
        self.exp_msg = msg
        self.rebuild()

    def rebuild(self):
        m = self.map_msg
        if m is None:
            return
        e = self.exp_msg
        if e is not None and (e.info.width != m.info.width or e.info.height != m.info.height):
            e = None                                     # andere Karte: Erfahrung nicht verwenden
        cells = int(round(self._p('known_dilate') / m.info.resolution))
        self.known = known_mask(m.info.width, m.info.height, m.data, e.data if e is not None else None, cells)

    def on_amcl(self, msg):
        c = msg.pose.covariance
        self.amcl_cov = (c[0], c[7], c[35])

    # ---------------------------------------------------------------- Scan
    def reason_all_unknown(self):
        if self.known is None:
            return 'keine Karte'
        now = time.monotonic()
        if self.score is None or now - self.score[1] > self._p('score_timeout') or self.score[0] < self._p('min_score'):
            return 'Lokalisierung unsicher (relocalize)'
        if self.amcl_cov is not None:
            sx, sy, syaw = (math.sqrt(max(v, 0.0)) for v in self.amcl_cov)
            if max(sx, sy) > self._p('max_pos_std') or math.degrees(syaw) > self._p('max_yaw_std_deg'):
                return 'Lokalisierung unsicher (AMCL)'
        return None

    def on_scan(self, msg):
        x, y = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                           self._p('max_range'))
        keep = outside_body(x, y, self._p('body_x_min'), self._p('body_x_max'), self._p('body_half_width'),
                            self._p('body_margin'))
        x, y = x[keep], y[keep]

        reason = self.reason_all_unknown()
        if reason is None:
            try:
                tf = self.tf_buffer.lookup_transform(self._p('map_frame'), msg.header.frame_id,
                                                     Time.from_msg(msg.header.stamp))
            except Exception:
                reason = 'kein TF map → Scan'
        if reason is None:
            t = tf.transform.translation
            mx, my = transform(x, y, t.x, t.y, yaw_of(tf.transform.rotation))
            i = self.map_msg.info
            known = is_known(self.known, i.origin.position.x, i.origin.position.y, i.resolution, mx, my)
            self.stats['bekannt'] += int(known.sum())
            x, y = x[~known], y[~known]
        else:
            self.stats['alles unbekannt'] += 1
        self.stats['scans'] += 1
        self.stats['punkte'] += int(x.size)

        mode = reason or 'Karte genutzt'
        if mode != self.mode:
            # getrennte Aufrufe: rclpy erlaubt an einer Stelle nicht abwechselnd info und warn
            if reason is None:
                self.get_logger().info('Sicherheitsfeld: Karte genutzt – bekannte Struktur bekommt die kleinen Zonen')
            else:
                self.get_logger().warn(f'Sicherheitsfeld: {mode} – alle Punkte gelten als unbekannt (vorsichtig)')
            self.mode = mode
        self.publish(msg.header, x, y)

    def publish(self, header, x, y):
        pts = np.zeros((x.size, 3), dtype=np.float32)
        pts[:, 0], pts[:, 1], pts[:, 2] = x, y, self._p('z')
        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = 1, int(x.size)
        cloud.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                        for k, n in enumerate('xyz')]
        cloud.is_bigendian, cloud.is_dense = False, True
        cloud.point_step, cloud.row_step = 12, 12 * int(x.size)
        cloud.data = array.array('B', pts.tobytes())
        self.pub.publish(cloud)

    def report(self):
        s = self.stats
        if s['scans']:
            total = s['punkte'] + s['bekannt']
            self.get_logger().info(
                f'letzte {self._p("report_period"):g} s: {s["scans"]} Scans, {total} Punkte im Umkreis, davon '
                f'{s["bekannt"]} bekannte Struktur, {s["punkte"]} unbekannt; {s["alles unbekannt"]} Scans ohne Karte')
        self.stats = dict.fromkeys(s, 0)


def main(args=None):
    rclpy.init(args=args)
    node = UnknownObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
