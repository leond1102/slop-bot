#!/usr/bin/env python3
"""Erfahrungskarte: lernt beim Fahren, welche Zellen der Karte fest (Wand, Tisch), vielleicht belegt (Stuhl) oder frei
sind, und gibt „vielleicht“-Zellen als weiche Kosten an den Planer (Nav2 Keepout-Filter in der globalen Costmap).

  /map                    nav_msgs/OccupancyGrid           Sub  Raster (map_server, transient local)
  /scan                   sensor_msgs/LaserScan            Sub  (base_footprint)
  /relocalize/score       std_msgs/Float32                 Sub  Lernsperre: Pose muss zur Karte passen
  /amcl_pose              geometry_msgs/PoseWithCovStamped Sub  Lernsperre: AMCL muss sich sicher sein
  /odom                   nav_msgs/Odometry                Sub  Lernsperre: nicht beim schnellen Drehen (Scan verzerrt)
  /experience/persistence nav_msgs/OccupancyGrid           Pub  Anzeige: Beständigkeit in % (weiß frei … schwarz fest)
  /experience/mask        nav_msgs/OccupancyGrid           Pub  weiche Kosten für den Keepout-Filter
  /experience/filter_info nav2_msgs/CostmapFilterInfo      Pub  nur mit soft_costs: schaltet den Filter ein
  /experience/known       nav_msgs/OccupancyGrid           Pub  Sicherheitsfeld + relocalize: 100 fest, 50 vielleicht, 0 frei, -1 ?
  Service ~/save          std_srvs/Trigger                     Zähler sofort speichern

Ablauf je Scan (höchstens scan_rate Hz):
  1. Pose map → base_footprint zum Scan-Zeitpunkt (TF aus AMCL + EKF)
  2. Lernsperre prüfen: relocalize-Score ≥ min_score, AMCL-Unsicherheit klein, Drehrate klein. Sonst nichts tun.
  3. Strahlverfolgung: Zellen bis zum Treffer frei, Trefferzelle belegt
  4. Sitzung: immer merken (für „zuletzt frei gesehen → keine Kosten“)
  5. Langzeit: nur, wenn sich der Roboter seit dem letzten Mal bewegt hat (min_travel / min_turn) oder idle_period s
     vergangen sind. Sonst zählte ein stehender Roboter jeden Menschen neben sich hundertfach.
Die Zähler liegen in <karte>.experience.npz neben der Karte (alle save_period s und beim Beenden gespeichert).
"""
import array
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from slop_navigation.experience_core import ExperienceGrid, OldFormat


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


class Experience(Node):
    def __init__(self):
        super().__init__('experience')
        for name, default in [
            ('map_frame', 'map'), ('base_frame', 'base_footprint'),
            ('file', ''),                   # Zähler (.npz); '' = nicht speichern
            # Klassen
            ('min_seen', 5.0),              # so viele Beobachtungen, bevor eine Zelle eine Meinung hat
            ('p_free', 0.15),               # Beständigkeit darunter: frei
            ('p_fixed', 0.80),              # ab hier: fest
            ('max_cost', 60),               # Maskenwert für „fast fest“ (× 2,54 = Costmap-Kosten; < 100, nie verboten)
            ('half_life_days', 7.0),        # Vergessen: nach so vielen Tagen zählt eine Beobachtung halb (≤ 0 = nie)
            ('soft_costs', False),          # Maske an den Planer geben (sonst nur lernen und anzeigen)
            ('live_clear', True),           # in dieser Sitzung zuletzt frei gesehene Zellen ohne Kosten
            # Strahlverfolgung
            ('max_range', 6.0),             # m: Treffer weiter weg zählen nicht (Winkelfehler)
            ('inf_free_range', 4.0),        # m: inf (kein Echo) räumt so weit frei
            ('hit_protect', 2),             # Zellen um einen Treffer, die derselbe Scan nicht freiräumt (Wände!)
            ('scan_rate', 2.0),             # Hz: so oft wird ein Scan ausgewertet
            # Lernsperre
            ('min_score', 0.75),            # relocalize: Anteil Scanpunkte auf Wänden
            ('score_timeout', 3.0),         # s: älterer Score gilt nicht
            ('max_pos_std', 0.20),          # m: AMCL-Unsicherheit x/y
            ('max_yaw_std_deg', 6.0),       # °: AMCL-Unsicherheit Gier
            ('max_turn_rate', 0.3),         # rad/s: beim schnelleren Drehen ist der Scan verzerrt
            # Langzeit-Zählung
            ('min_travel', 0.20),           # m seit der letzten Zählung
            ('min_turn_deg', 20.0),         # ° seit der letzten Zählung
            ('idle_period', 60.0),          # s: im Stand höchstens so oft zählen
            # Ausgabe
            ('publish_period', 3.0),        # s: Maske und Anzeige, nur bei Änderung (der Filter loggt jede neue Maske)
            ('save_period', 60.0),          # s
            ('report_period', 60.0),        # s: Zusammenfassung im Log
        ]:
            self.declare_parameter(name, default)

        self.grid = None
        self.map_info = None
        self.score = None               # (Wert, monotone Zeit)
        self.amcl_cov = None
        self.turn_rate = 0.0
        self.last_scan_t = 0.0
        self.last_integrate = None      # (x, y, yaw, monotone Zeit)
        self.session_t0 = time.monotonic()
        self.dirty = False
        self.last_persist = None
        self.last_mask = None
        self.counts = {'gezählt': 0, 'nur Sitzung': 0, 'gesperrt: Score': 0, 'gesperrt: AMCL': 0,
                       'gesperrt: Drehen': 0, 'gesperrt: TF': 0}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Float32, '/relocalize/score', self.on_score, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.persist_pub = self.create_publisher(OccupancyGrid, '/experience/persistence', latched)
        self.mask_pub = self.create_publisher(OccupancyGrid, '/experience/mask', latched)
        self.known_pub = self.create_publisher(OccupancyGrid, '/experience/known', latched)   # Sicherheitsfeld, relocalize
        self.info_pub = self.create_publisher(CostmapFilterInfo, '/experience/filter_info', latched)
        self.create_service(Trigger, '~/save', self.on_save)
        self.create_timer(self._p('publish_period'), self.publish)
        self.create_timer(self._p('save_period'), self.save)
        self.create_timer(self._p('report_period'), self.report)

        if self._p('soft_costs'):
            info = CostmapFilterInfo()
            info.header.frame_id = self._p('map_frame')
            info.type = 0                                  # Keepout
            info.filter_mask_topic = '/experience/mask'
            info.base, info.multiplier = 0.0, 1.0
            self.info_pub.publish(info)
        self.get_logger().info(
            f'Erfahrungskarte bereit: {"weiche Kosten AN" if self._p("soft_costs") else "nur lernen und anzeigen"}, '
            f'lernt bei relocalize ≥ {self._p("min_score"):.0%}, Halbwertszeit {self._p("half_life_days"):g} Tage, '
            f'Datei {self._p("file") or "(keine)"}')

    def _p(self, name):
        return self.get_parameter(name).value

    # ---------------------------------------------------------------- Eingänge
    def on_map(self, msg):
        i = msg.info
        args = (i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y)
        if self.grid is not None and self.grid.same_grid(*args):
            return
        if self.grid is not None:
            self.save()
        self.map_info = i
        self.grid = self.load(*args) or ExperienceGrid(*args)
        s = self.grid.summary(self._p('min_seen'), self._p('p_free'), self._p('p_fixed'))
        self.get_logger().info(f'Karte {i.width}×{i.height}: fest {s["fest"]}, vielleicht {s["vielleicht"]}, '
                               f'frei {s["frei"]} Zellen')
        self.dirty = True

    def load(self, *args):
        path = self._p('file')
        if not path or not os.path.exists(path):
            return None
        try:
            g = ExperienceGrid.load(path)
        except OldFormat as e:
            backup = f'{path}.{time.strftime("%Y%m%d-%H%M%S")}.v1.alt'
            self.get_logger().warn(f'{path}: {e} (hielt Wände oft für „vielleicht“) – beginne neu, alte Datei: {backup}')
            os.replace(path, backup)
            return None
        except (OSError, ValueError, KeyError) as e:
            self.get_logger().error(f'{path} unlesbar ({e}) – beginne neu, alte Datei bleibt als .kaputt')
            os.replace(path, path + '.kaputt')
            return None
        if not g.same_grid(*args):
            backup = f'{path}.{time.strftime("%Y%m%d-%H%M%S")}.alt'
            self.get_logger().warn(f'{path} gehört zu einem anderen Raster – beginne neu, alte Datei: {backup}')
            os.replace(path, backup)
            return None
        days = (time.time() - g.stamp) / 86400.0
        g.decay_to(time.time(), self._half_life_s())
        self.get_logger().info(f'Erfahrung geladen ({days:.1f} Tage alt)')
        return g

    def on_score(self, msg):
        self.score = (msg.data, time.monotonic())

    def on_amcl(self, msg):
        c = msg.pose.covariance
        self.amcl_cov = (c[0], c[7], c[35])

    def on_odom(self, msg):
        self.turn_rate = msg.twist.twist.angular.z

    def _half_life_s(self):
        return self._p('half_life_days') * 86400.0

    # ---------------------------------------------------------------- Lernen
    def lock_reason(self):
        """Grund, warum gerade nicht gelernt wird; None = Lokalisierung ist sicher."""
        now = time.monotonic()
        if self.score is None or now - self.score[1] > self._p('score_timeout') or self.score[0] < self._p('min_score'):
            return 'gesperrt: Score'
        if self.amcl_cov is not None:
            sx, sy, syaw = (math.sqrt(max(v, 0.0)) for v in self.amcl_cov)
            if max(sx, sy) > self._p('max_pos_std') or math.degrees(syaw) > self._p('max_yaw_std_deg'):
                return 'gesperrt: AMCL'
        if abs(self.turn_rate) > self._p('max_turn_rate'):
            return 'gesperrt: Drehen'
        return None

    def on_scan(self, msg):
        if self.grid is None:
            return
        now = time.monotonic()
        if now - self.last_scan_t < 1.0 / self._p('scan_rate'):
            return
        self.last_scan_t = now
        reason = self.lock_reason()
        if reason:
            self.counts[reason] += 1
            return
        try:
            tf = self.tf_buffer.lookup_transform(self._p('map_frame'), msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            self.counts['gesperrt: TF'] += 1
            return
        t = tf.transform.translation
        x, y, yaw = t.x, t.y, yaw_of(tf.transform.rotation)

        free, hit = self.grid.ray_cells(x, y, yaw, msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                        msg.range_max, self._p('max_range'), self._p('inf_free_range'),
                                        hit_protect=self._p('hit_protect'))
        self.grid.observe_session(free, hit, now - self.session_t0)
        self.dirty = True

        last = self.last_integrate
        due = (last is None or math.hypot(x - last[0], y - last[1]) >= self._p('min_travel')
               or math.degrees(angle_diff(yaw, last[2])) >= self._p('min_turn_deg')
               or now - last[3] >= self._p('idle_period'))
        if due:
            self.grid.decay_to(time.time(), self._half_life_s())
            self.grid.integrate(free, hit)
            self.last_integrate = (x, y, yaw, now)
            self.counts['gezählt'] += 1
        else:
            self.counts['nur Sitzung'] += 1

    # ---------------------------------------------------------------- Ausgabe
    def _grid_msg(self, data):
        msg = OccupancyGrid()
        msg.header.frame_id = self._p('map_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_info
        msg.data = array.array('b', data.astype(np.int8).tobytes())   # viel schneller als tolist()
        return msg

    def publish(self):
        """Anzeige und Maske senden, aber nur bei Änderung: Der Keepout-Filter loggt jede neue Maske als Warnung."""
        if self.grid is None or not self.dirty:
            return
        self.dirty = False
        a = (self._p('min_seen'), self._p('p_free'), self._p('p_fixed'))
        persist = self.grid.persistence_grid(a[0])
        if self.last_persist is None or not np.array_equal(persist, self.last_persist):
            self.last_persist = persist
            self.persist_pub.publish(self._grid_msg(persist))
            self.known_pub.publish(self._grid_msg(self.grid.known_grid(*a)))
        if self._p('soft_costs'):
            mask = self.grid.mask(*a, self._p('max_cost'), live=self._p('live_clear'))
            if self.last_mask is None or not np.array_equal(mask, self.last_mask):
                self.last_mask = mask
                self.mask_pub.publish(self._grid_msg(mask))

    def save(self):
        path = self._p('file')
        if self.grid is None or not path:
            return
        try:
            self.grid.save(path)
        except OSError as e:
            self.get_logger().warn(f'Erfahrung nicht gespeichert ({path}): {e}', throttle_duration_sec=60.0)

    def on_save(self, request, response):
        self.save()
        response.success = self.grid is not None and bool(self._p('file'))
        response.message = f'gespeichert: {self._p("file")}' if response.success else 'noch keine Karte oder keine Datei'
        return response

    def report(self):
        if self.grid is None:
            return
        s = self.grid.summary(self._p('min_seen'), self._p('p_free'), self._p('p_fixed'))
        c = ', '.join(f'{k} {v}' for k, v in self.counts.items() if v)
        self.get_logger().info(f'Scans letzte {self._p("report_period"):g} s: {c or "keine"} | Zellen: fest {s["fest"]}, '
                               f'vielleicht {s["vielleicht"]}, frei {s["frei"]}')
        self.counts = dict.fromkeys(self.counts, 0)

    def shutdown(self):
        self.save()


def main(args=None):
    rclpy.init(args=args)
    node = Experience()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
