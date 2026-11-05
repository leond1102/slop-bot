#!/usr/bin/env python3
"""Relokalisierung für AMCL: erkennt, wenn der Scan nicht mehr zur Karte passt, sucht die Pose in der ganzen Karte und
setzt sie (/initialpose). Merkt sich außerdem die letzte gute Pose für den nächsten Start.

  /map           nav_msgs/OccupancyGrid           Sub  (map_server, transient local)
  /scan          sensor_msgs/LaserScan            Sub
  /amcl_pose     geometry_msgs/PoseWithCovStamped Sub  zum Speichern der letzten Pose
  /initialpose   geometry_msgs/PoseWithCovStamped Pub  gefundene Pose an AMCL
  /relocalize/score  std_msgs/Float32             Pub  Anteil der Scanpunkte auf einer Wand (0..1), check_period
  Service relocalize  std_srvs/Trigger                 sofort in der ganzen Karte suchen

Start: sobald AMCL läuft (TF map → base_footprint da), in dieser Reihenfolge prüfen, bis eine Pose passt (≥ min_score):
  1. die Pose, mit der AMCL gestartet ist (localization.launch.py x/y/yaw, Standard 0/0/0)
  2. die zuletzt gespeicherte Pose (pose_file), fein nachgesucht
  3. die ganze Karte (relocalize_core.global_search); gesetzt wird nur ein eindeutiges Ergebnis
Betrieb: Liegt der Anteil länger als bad_duration s unter bad_score, wird die ganze Karte durchsucht. Liegt das eindeutige
Ergebnis woanders als die aktuelle Pose, wird es gesetzt; danach cooldown s Pause.
Die letzte Pose wird alle save_period s gespeichert, wenn sie passt (score ≥ min_score).
"""
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from slop_navigation.relocalize_core import Candidate, MapModel, decide, global_search, refine, scan_points


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Relocalize(Node):
    def __init__(self):
        super().__init__('relocalize')
        for name, default in [
            ('map_frame', 'map'), ('base_frame', 'base_footprint'),
            ('pose_file', ''),              # letzte Pose (YAML); '' = nicht speichern
            ('tolerance', 0.10),            # m: so nah an einer Wand gilt ein Scanpunkt als Treffer
            ('min_score', 0.80),            # ab hier gilt eine Pose als passend
            ('min_margin', 0.08),           # Abstand zum besten Kandidaten an einem anderen Ort
            ('bad_score', 0.60),            # darunter gilt die aktuelle Pose als falsch ...
            ('bad_duration', 3.0),          # ... wenn das so viele Sekunden anhält
            ('check_period', 1.0),          # s zwischen zwei Prüfungen
            ('cooldown', 10.0),             # s nach einer Relokalisierung keine automatische Suche
            ('save_period', 2.0),           # s
            ('max_points', 120),            # Scanpunkte für die Bewertung
            ('relocalize_at_start', True),
            ('auto', True),                 # im Betrieb automatisch suchen
        ]:
            self.declare_parameter(name, default)
        self.model = None
        self.scan = None
        self.amcl_pose = None
        self.started = False
        self.bad_since = None
        self.last_reloc = 0.0
        self.last_save = 0.0
        self.last_score = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: setattr(self, 'amcl_pose', m), latched)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.score_pub = self.create_publisher(Float32, '/relocalize/score', 10)
        self.create_service(Trigger, 'relocalize', self.on_service)
        self.create_timer(self._p('check_period'), self.check)
        self.get_logger().info(f'Relokalisierung bereit (Pose passt ab {self._p("min_score"):.0%} Scanpunkte auf Wänden; '
                               f'unter {self._p("bad_score"):.0%} für {self._p("bad_duration"):g} s wird gesucht)')

    def _p(self, name):
        return self.get_parameter(name).value

    # ---------------------------------------------------------------- Eingänge
    def on_map(self, msg):
        info = msg.info
        self.model = MapModel(msg.data, info.width, info.height, info.resolution, info.origin.position.x,
                              info.origin.position.y, tolerance=self._p('tolerance'))
        self.get_logger().info(f'Karte {info.width}×{info.height} Zellen à {info.resolution:.2f} m geladen')

    def points(self):
        s = self.scan
        return scan_points(s.ranges, s.angle_min, s.angle_increment, max(0.2, s.range_min), min(15.0, s.range_max),
                           self._p('max_points'))

    def current_pose(self):
        """Pose von base_frame in map (TF aus AMCL + EKF); None, solange AMCL nicht läuft."""
        try:
            tf = self.tf_buffer.lookup_transform(self._p('map_frame'), self._p('base_frame'), Time())
        except Exception:
            return None
        t = tf.transform.translation
        return Candidate(0.0, t.x, t.y, yaw_of(tf.transform.rotation))

    # ---------------------------------------------------------------- Ablauf
    def check(self):
        if self.model is None or self.scan is None:
            return
        pose = self.current_pose()
        if pose is None or self.pose_pub.get_subscription_count() == 0:
            return                      # AMCL noch nicht aktiv
        r, a = self.points()
        score = self.model.score(r, a, pose.x, pose.y, pose.yaw)
        self.last_score = score
        self.score_pub.publish(Float32(data=score))
        now = time.monotonic()

        if not self.started:
            self.started = True
            if self._p('relocalize_at_start'):
                self.startup(pose, score, r, a)
            return

        if score >= self._p('min_score'):
            self.bad_since = None
            if self._p('pose_file') and now - self.last_save > self._p('save_period'):
                self.last_save = now
                self.save_pose(pose)
            return
        if not self._p('auto') or score >= self._p('bad_score') or now - self.last_reloc < self._p('cooldown'):
            self.bad_since = None if score >= self._p('bad_score') else self.bad_since
            return
        if self.bad_since is None:
            self.bad_since = now
            self.get_logger().warn(f'Scan passt schlecht zur Karte ({score:.0%}) – suche, wenn das '
                                   f'{self._p("bad_duration"):g} s anhält')
        elif now - self.bad_since >= self._p('bad_duration'):
            self.bad_since = None
            self.search(r, a, reason=f'Scan passt seit {self._p("bad_duration"):g} s nur zu {score:.0%}', current=pose)

    def startup(self, pose, score, r, a):
        if score >= self._p('min_score'):
            self.get_logger().info(f'Startpose passt ({score:.0%} auf Wänden) – keine Suche nötig')
            return
        saved = self.load_pose()
        if saved is not None:
            cand = refine(self.model, r, a, saved, xy=0.3, yaw_deg=10.0)
            if cand.score >= self._p('min_score'):
                self.set_pose(cand, f'letzte gespeicherte Pose passt ({cand.score:.0%})')
                return
            self.get_logger().info(f'Letzte gespeicherte Pose passt nicht ({cand.score:.0%})')
        self.search(r, a, reason=f'Startpose passt nur zu {score:.0%}')

    def search(self, r, a, reason, current=None):
        self.get_logger().info(f'{reason} – durchsuche die ganze Karte …')
        stamp = self.scan.header.stamp
        t0 = time.monotonic()
        best, why = decide(global_search(self.model, r, a), self._p('min_score'), self._p('min_margin'))
        self.last_reloc = time.monotonic()
        took = self.last_reloc - t0
        if best is None:
            self.get_logger().warn(f'Keine eindeutige Pose gefunden ({why}, {took:.1f} s). Pose bleibt; in RViz mit '
                                   '"2D Pose Estimate" setzen oder ein Stück fahren und erneut versuchen')
            return False, why
        if current is not None:
            dist, dyaw = best.distance_to(current)
            if dist < 0.3 and dyaw < 15.0:
                self.get_logger().info(f'Beste Pose liegt an der aktuellen Stelle ({why}) – nichts zu ändern '
                                       '(Umgebung verändert? Menschen, Möbel)')
                return True, 'aktuelle Pose ist die beste'
        self.set_pose(best, f'{why}, Suche {took:.1f} s', stamp)
        return True, f'Pose gesetzt: x {best.x:.2f}, y {best.y:.2f}, Gier {math.degrees(best.yaw):.0f}° ({why})'

    def set_pose(self, cand, why, stamp=None):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._p('map_frame')
        # Stempel des Scans: AMCL rechnet die Bewegung seitdem über die Odometrie nach
        msg.header.stamp = stamp if stamp is not None else self.scan.header.stamp
        msg.pose.pose.position.x, msg.pose.pose.position.y = cand.x, cand.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(cand.yaw / 2), math.cos(cand.yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.1 ** 2
        msg.pose.covariance[35] = math.radians(5.0) ** 2
        self.pose_pub.publish(msg)
        self.last_reloc = time.monotonic()      # AMCL braucht einen Moment; so lange nicht erneut suchen
        self.bad_since = None
        self.get_logger().info(f'Pose gesetzt: x {cand.x:.2f} m, y {cand.y:.2f} m, Gier {math.degrees(cand.yaw):.0f}° ({why})')

    def on_service(self, request, response):
        if self.model is None or self.scan is None:
            response.success, response.message = False, 'Karte oder Scan fehlt noch'
            return response
        r, a = self.points()
        response.success, response.message = self.search(r, a, reason='Suche angefordert', current=self.current_pose())
        return response

    # ---------------------------------------------------------------- letzte Pose
    def save_pose(self, pose):
        path = self._p('pose_file')
        data = {'x': round(pose.x, 3), 'y': round(pose.y, 3), 'yaw': round(pose.yaw, 4),
                'score': round(self.last_score or 0.0, 3), 'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
        try:
            tmp = path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                yaml.safe_dump(data, f)
            os.replace(tmp, path)
        except OSError as e:
            self.get_logger().warn(f'Letzte Pose nicht gespeichert ({path}): {e}', throttle_duration_sec=60.0)

    def load_pose(self):
        path = self._p('pose_file')
        if not path or not os.path.exists(path):
            return None
        try:
            with open(path, encoding='utf-8') as f:
                d = yaml.safe_load(f)
            return Candidate(0.0, float(d['x']), float(d['y']), float(d['yaw']))
        except (OSError, KeyError, TypeError, ValueError) as e:
            self.get_logger().warn(f'Letzte Pose unlesbar ({path}): {e}')
            return None

    def shutdown(self):
        pose = self.current_pose()
        if pose is not None and self._p('pose_file') and (self.last_score or 0.0) >= self._p('min_score'):
            self.save_pose(pose)


def main(args=None):
    rclpy.init(args=args)
    node = Relocalize()
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
