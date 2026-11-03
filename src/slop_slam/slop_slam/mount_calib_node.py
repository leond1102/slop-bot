#!/usr/bin/env python3
"""Montage von Lidar und RealSense mit LIO laufend nachführen.

RealSense (volle Lage x/y/z/roll/pitch/yaw):
  Alle `period` s wird das Tiefenbild (ausgedünnt) mit der aktuellen Montage in map gelegt und per ICP
  (Punkt-zu-Ebene) an die Lidar-Punkte der letzten `map_window` s (/cloud_registered_body) angepasst. Die
  Korrektur wird in eine neue Montage base_footprint → camera_link umgerechnet und geglättet übernommen.
  Nur wenn: Roboter dreht langsam, ICP passt gut (fitness, rmse), Szene hat Struktur in alle Richtungen
  (nicht nur eine Wand/Boden), Korrektur nicht unplausibel groß.

Lidar (yaw, also die Blickrichtung):
  Ein Differentialantrieb fährt nur entlang seiner x-Achse. Fährt der Roboter laut Radodometrie (/odom)
  geradeaus, zeigt die von FAST-LIO gemessene Verschiebung im IMU-Frame, wie der Lidar um die Senkrechte
  verdreht montiert ist. roll/pitch kommen weiter aus der Schwerkraft (mount_tf), x/y/z aus mounts.yaml.
  Braucht laufenden Antrieb (sonst unbekannt, ob vor- oder rückwärts).

Ergebnisse gehen an mount_tf (/mount_tf/update), das die TF sendet und in RViz anzeigt, und werden in
`calib_file` gespeichert. mount_tf lädt die Datei beim nächsten Start.

Topics:
  /cloud_registered_body          sensor_msgs/PointCloud2  Sub  volle Lidar-Scans im IMU-Frame (FAST-LIO)
  /Odometry                       nav_msgs/Odometry        Sub  IMU-Pose (FAST-LIO)
  /odom                           nav_msgs/Odometry        Sub  Radodometrie (slop_base), nur für den Lidar-yaw
  /camera/depth/image_rect_raw    sensor_msgs/Image        Sub
  /camera/depth/camera_info       sensor_msgs/CameraInfo   Sub
  /mount_tf/update                geometry_msgs/TransformStamped  Pub  neue Montage base → Sensor
"""
import collections
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from slop_slam.calib_math import (blend, camera_mount_from_icp, circular_mean, inv, matrix_to_quat, matrix_to_rpy,
                                  pose, quat_to_matrix, rotation_angle, rpy_to_matrix, wrap, yaw_from_motion)


def tf_to_T(tf):
    r, t = tf.transform.rotation, tf.transform.translation
    return pose(quat_to_matrix((r.x, r.y, r.z, r.w)), [t.x, t.y, t.z])


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    xyz = np.empty((n, 3), np.float64)
    for i, k in enumerate('xyz'):
        xyz[:, i] = raw[:, off[k]:off[k] + 4].copy().view(np.float32).ravel()
    return xyz[np.isfinite(xyz).all(axis=1)]


def transform_points(T, p):
    return p @ T[:3, :3].T + T[:3, 3]


def rot_z_np(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class MountCalibNode(Node):

    def __init__(self):
        super().__init__('mount_calib')
        for name, default in [
            ('map_frame', 'map'),
            ('base_frame', 'base_footprint'),
            ('lidar_frame', 'livox_frame'),
            ('camera_frame', 'camera_link'),
            ('lio_world_frame', 'camera_init'),
            ('calib_file', os.path.expanduser('~/slop-bot_ws/calib/mounts_lio.yaml')),
            ('cloud_topic', '/cloud_registered_body'),   # volle Scans; /cloud_registered ist auf 30 cm ausgedünnt
            # RealSense per ICP
            ('camera', True),
            ('period', 2.0),               # s zwischen zwei ICP-Läufen, bis die Kamera einmal gut passt
            ('period_tracking', 10.0),     # s danach (Montage ändert sich kaum; ICP belegt kurz mehrere Kerne)
            ('map_window', 3.0),           # s Lidar-Scans als Referenz
            ('voxel', 0.03),               # m
            ('depth_stride', 2),           # jedes n-te Pixel (bei 424×240)
            ('depth_min', 0.3), ('depth_max', 3.0),   # m (weiter weg: Tiefenfehler der D435i zu groß)
            ('icp_max_dist', [0.25, 0.10, 0.05]),     # m, grob → fein
            ('min_points', 1500),
            ('min_fitness', 0.3),          # Anteil Kamerapunkte mit Partner < 5 cm (D435i rauscht in 3 m um
                                           # mehrere cm, gut ausgerichtet gemessen: ~0,3–0,4)
            ('max_rmse', 0.035),           # m (gemessen gut ausgerichtet: ~3,0 cm, Tiefenrauschen der D435i)
            ('min_normal_spread', 0.04),   # kleinster Eigenwert von Σnnᵀ/N: 0 = nur eine Ebene (unbestimmt)
            ('max_step_t', 0.3), ('max_step_deg', 15.0),   # größere Sprünge gelten als Fehlanpassung
            ('max_turn_rate', 0.15),       # rad/s, schneller drehen = Zeitversatz zu groß, kein ICP
            ('camera_blend', 0.3),         # Anteil, mit dem ein ICP-Ergebnis übernommen wird
            # Position aus ICP ist nur auf ~5–10 cm bestimmt (Tiefenfehler, ein Blickwinkel), die Drehung auf
            # ~1–2°. Daher standardmäßig nur die Drehung nachführen, Position aus mounts.yaml (nachmessen!).
            ('camera_translation', False),
            ('translation_warn', 0.15),    # m: ICP-Position weicht so weit von mounts.yaml ab → Warnung
            # Grobsuche vor dem ICP, solange es noch kein gutes Ergebnis gab (mounts.yaml ist nur geschätzt)
            # zweistufig: grob (±0,6 m in 0,2-m-Schritten, ±40° in 10°), dann fein um den Besten (±0,15 m / ±7,5°)
            ('coarse_xyz', 0.6), ('coarse_xyz_step', 0.2), ('coarse_yaw_deg', 40.0), ('coarse_yaw_step_deg', 10.0),
            ('fine_xyz', 0.15), ('fine_xyz_step', 0.05), ('fine_yaw_deg', 7.5), ('fine_yaw_step_deg', 2.5),
            ('coarse_inlier_dist', 0.05),  # m
            ('coarse_points', 1500),
            # Lidar-yaw aus Geradeausfahrt
            ('lidar_yaw', True),
            ('yaw_window', 3.0),           # s
            ('yaw_min_dist', 0.04),        # m Fahrt je Abschnitt
            ('yaw_max_turn_deg', 3.0),     # Drehung je Abschnitt (LIO)
            ('yaw_max_wheel_w', 0.05),     # rad/s laut Radodometrie
            ('yaw_min_wheel_v', 0.004),    # m/s laut Radodometrie
            ('yaw_min_total', 0.15),       # m Fahrt gesamt, bevor yaw übernommen wird
            ('yaw_blend', 0.3),
        ]:
            self.declare_parameter(name, default)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20))
        # TF in eigener Node mit eigenem Thread: kommt auch an, während diese Node rechnet (ICP)
        self.tf_node = rclpy.create_node('mount_calib_tf', use_global_arguments=False)   # sonst benennt -r __node:= sie mit um
        self.tf_listener = TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
        self.update_pub = self.create_publisher(TransformStamped, '/mount_tf/update', 10)

        self.scans = collections.deque(maxlen=60)  # PointCloud2, erst beim ICP in map umgerechnet
        self.depth = None
        self.depth_buf = collections.deque(maxlen=30)   # ~1 s Tiefenbilder: LIO-TF hinkt 0,1–0,3 s hinterher
        self.info = None
        self.lio = collections.deque()             # (zeit, R, p) IMU in camera_init
        self.base_poses = collections.deque(maxlen=100)   # (zeit, T_map_base) aus /lio/odom
        self.wheel = collections.deque()           # (zeit, v, w)
        self.yaw_samples = collections.deque(maxlen=30)
        self.last_save = None
        self.o3d = None
        self.camera_ok = False     # gab es schon ein gutes ICP-Ergebnis? Sonst erst grob suchen
        self.last_depth_stamp = None   # jedes Tiefenbild nur einmal verwenden
        self.stats = collections.Counter()   # Bilanz der Kamera-Durchläufe, alle 30 s ausgegeben
        self.camera_misses = 0     # ICP-Fehlschläge in Folge (fitness), nach 10 wieder grob suchen
        calib = self._p('calib_file')
        if os.path.exists(calib):
            with open(calib) as f:
                self.camera_ok = self._p('camera_frame') in (yaml.safe_load(f) or {})
            if self.camera_ok:
                self.get_logger().info(f'Kamera-Montage aus {calib} als Start, keine Grobsuche')

        self.create_subscription(PointCloud2, self._p('cloud_topic'), self.scan_cb, 10)
        self.create_subscription(Odometry, '/Odometry', self.lio_cb, 50)
        self.create_subscription(Odometry, '/odom', self.wheel_cb, 50)
        self.create_subscription(Odometry, '/lio/odom', self.base_cb, 50)
        if self._p('camera'):
            self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.info_cb, qos_profile_sensor_data)
            self.camera_timer = self.create_timer(self._p('period'), self.camera_step)
            self.create_timer(30.0, self.report)
        if self._p('lidar_yaw'):
            self.create_timer(0.5, self.lidar_yaw_step)
        self.get_logger().info('Montage nachführen: RealSense per ICP an die LIO-Karte'
                               + (', Lidar-yaw aus Geradeausfahrt' if self._p('lidar_yaw') else ''))

    def _p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    # ---------------------------------------------------------------- Eingänge
    def scan_cb(self, msg):
        # nur sammeln, nicht auf TF warten (blockiert sonst die Node); umgerechnet wird beim ICP
        self.scans.append(msg)

    def scans_in_map(self, t_ref):
        """Scans der letzten map_window s vor t_ref in map. Pose je Scan aus /Odometry mit gleichem Zeitstempel
        (FAST-LIO stempelt Scan und Pose mit derselben Zeit), map → camera_init ist statisch (lio_tf)."""
        try:
            T_map_ci = tf_to_T(self.tf_buffer.lookup_transform(self._p('map_frame'), self._p('lio_world_frame'), Time()))
        except TransformException:
            return []
        poses = {round(t, 3): pose(R, p) for t, R, p in self.lio}
        out = []
        for msg in self.scans:
            t = self._sec(msg.header.stamp)
            T_ci_body = poses.get(round(t, 3))
            if T_ci_body is None or not (t_ref - self._p('map_window') <= t <= t_ref + 0.5):
                continue
            out.append(transform_points(T_map_ci @ T_ci_body, cloud_xyz(msg)))
        return out

    def base_pose_at(self, t):
        """Roboterpose in map zur Zeit t, zwischen zwei /lio/odom-Nachrichten interpoliert (None = nicht abgedeckt)."""
        prev = None
        for tb, T in self.base_poses:
            if tb >= t:
                if prev is None:
                    return None
                ta, Ta = prev
                return blend(Ta, T, (t - ta) / (tb - ta) if tb > ta else 0.0)
            prev = (tb, T)
        return None

    def depth_cb(self, msg):
        self.depth_buf.append(msg)

    def info_cb(self, msg):
        self.info = msg

    def lio_cb(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        t = self._sec(msg.header.stamp)
        self.lio.append((t, quat_to_matrix((o.x, o.y, o.z, o.w)), np.array([p.x, p.y, p.z])))
        while self.lio and t - self.lio[0][0] > 10.0:
            self.lio.popleft()

    def base_cb(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self.base_poses.append((self._sec(msg.header.stamp),
                                pose(quat_to_matrix((o.x, o.y, o.z, o.w)), [p.x, p.y, p.z])))

    def wheel_cb(self, msg):
        t = self._sec(msg.header.stamp)
        self.wheel.append((t, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
        while self.wheel and t - self.wheel[0][0] > 10.0:
            self.wheel.popleft()

    def turn_rate(self):
        """Drehrate laut LIO über die letzten ~0,5 s [rad/s]."""
        if len(self.lio) < 2:
            return None
        t1, R1, _ = self.lio[-1]
        for t0, R0, _ in reversed(self.lio):
            if t1 - t0 >= 0.5:
                return rotation_angle(R0.T @ R1) / (t1 - t0)
        return None

    # ---------------------------------------------------------------- RealSense per ICP
    def depth_points(self):
        d, k = self.depth, self.info
        if d.encoding not in ('16UC1', 'mono16'):
            return None
        img = np.frombuffer(bytes(d.data), dtype=np.uint16).reshape(d.height, d.width)
        s = self._p('depth_stride')
        img = img[::s, ::s].astype(np.float64) * 0.001
        v, u = np.mgrid[0:d.height:s, 0:d.width:s]
        ok = (img > self._p('depth_min')) & (img < self._p('depth_max'))
        z = img[ok]
        fx, fy, cx, cy = k.k[0], k.k[4], k.k[2], k.k[5]
        return np.column_stack(((u[ok] - cx) * z / fx, (v[ok] - cy) * z / fy, z))

    def camera_step(self):
        if not self.depth_buf or self.info is None or not self.scans:
            self.stats['wartet auf Daten'] += 1
            return
        mp, base, cam = self._p('map_frame'), self._p('base_frame'), self._p('camera_frame')
        # neuestes noch nicht benutztes Tiefenbild, für das es schon eine LIO-Pose gibt (LIO hinkt ~0,1–0,3 s nach)
        self.depth, T_map_base = None, None
        for msg in reversed(self.depth_buf):
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if self.last_depth_stamp is not None and key <= self.last_depth_stamp:
                break
            T_map_base = self.base_pose_at(self._sec(msg.header.stamp))
            if T_map_base is not None:
                self.depth = msg
                self.last_depth_stamp = key
                break
        if self.depth is None:
            self.stats['keine LIO-Pose zum Tiefenbild'] += 1
            return
        rate = self.turn_rate()
        if rate is None or rate > self._p('max_turn_rate'):
            self.stats['dreht zu schnell'] += 1
            return
        try:   # nur feste Teile aus der TF: Montage und RealSense-intern
            T_base_cam = tf_to_T(self.tf_buffer.lookup_transform(base, cam, Time()))
            T_base_depth = tf_to_T(self.tf_buffer.lookup_transform(base, self.depth.header.frame_id, Time()))
        except TransformException as e:
            self.get_logger().info(f'Montage-TF fehlt: {e}', throttle_duration_sec=10.0)
            self.stats['TF fehlt'] += 1
            return
        T_map_depth = T_map_base @ T_base_depth
        pts = self.depth_points()
        if pts is None or len(pts) < self._p('min_points'):
            self.stats['zu wenig Tiefenpunkte'] += 1
            return

        if self.o3d is None:
            import open3d   # erst hier, Import dauert ~1 s
            self.o3d = open3d
        o3d = self.o3d
        vox = self._p('voxel')
        src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(transform_points(T_map_depth, pts)))
        src = src.voxel_down_sample(vox)
        scans = self.scans_in_map(self._sec(self.depth.header.stamp))
        if not scans:
            self.stats['keine Lidar-Scans mit TF'] += 1
            return
        tgt = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack(scans)))
        tgt = tgt.voxel_down_sample(vox)
        if len(src.points) < self._p('min_points') or len(tgt.points) < self._p('min_points'):
            self.stats['zu wenig Punkte'] += 1
            return
        tgt.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=4 * vox, max_nn=30))

        dT = np.eye(4)
        if not self.camera_ok:
            dT = self.coarse_search(np.asarray(src.points), np.asarray(tgt.points), T_map_base, T_base_cam)
        res = None
        for dist in self._p('icp_max_dist'):
            res = o3d.pipelines.registration.registration_icp(
                src, tgt, dist, dT, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
            dT = res.transformation

        # Bestimmtheit: Normalen der gepaarten Zielpunkte müssen in alle Richtungen zeigen
        spread = 0.0
        corr = np.asarray(res.correspondence_set)
        if len(corr) > 0:
            n = np.asarray(tgt.normals)[corr[:, 1]]
            spread = float(np.linalg.eigvalsh(n.T @ n / len(n))[0])
        T_icp = camera_mount_from_icp(T_map_base, T_base_cam, dT)
        step_t = float(np.linalg.norm(T_icp[:3, 3] - T_base_cam[:3, 3]))
        step_deg = math.degrees(rotation_angle(T_base_cam[:3, :3].T @ T_icp[:3, :3]))
        info = (f'fitness {res.fitness:.2f}, rmse {res.inlier_rmse * 100:.1f} cm, Struktur {spread:.3f}, '
                f'Korrektur {step_t * 100:.1f} cm / {step_deg:.1f}°')
        reasons = []
        if res.fitness < self._p('min_fitness'):
            reasons.append('fitness')
        if res.inlier_rmse > self._p('max_rmse'):
            reasons.append('rmse')
        if spread < self._p('min_normal_spread'):
            reasons.append('zu wenig Struktur (nur Wand/Boden?)')
        # Sprunggrenze erst, wenn die Kamera schon einmal gut passte (davor darf die Grobsuche weit springen)
        if self.camera_ok and (step_t > self._p('max_step_t') or step_deg > self._p('max_step_deg')):
            reasons.append('Sprung zu groß')
        if reasons:
            self.stats['verworfen: ' + ', '.join(reasons)] += 1
            self.get_logger().info(f'Kamera-ICP verworfen ({", ".join(reasons)}): {info}', throttle_duration_sec=10.0)
            if 'fitness' in reasons:
                self.camera_misses += 1
                if self.camera_ok and self.camera_misses >= 10:
                    self.get_logger().warn('Kamera passt 10× nicht zur Lidar-Karte (umgebaut?) – suche wieder grob')
                    self.camera_ok = False
                    self.camera_misses = 0
                    self.set_period(self._p('period'))
            return
        self.camera_misses = 0
        self.stats['übernommen'] += 1

        icp_xyz = T_icp[:3, 3].copy()
        if not self._p('camera_translation'):
            T_icp[:3, 3] = T_base_cam[:3, 3]      # nur die Drehung übernehmen
            off = float(np.linalg.norm(icp_xyz - T_base_cam[:3, 3]))
            if off > self._p('translation_warn'):
                self.get_logger().warn(
                    f'Kamera: ICP sieht die Kamera {off * 100:.0f} cm von der Position in mounts.yaml entfernt '
                    f'(ICP: x {icp_xyz[0]:+.2f} y {icp_xyz[1]:+.2f} z {icp_xyz[2]:+.2f} m) – mounts.yaml nachmessen',
                    throttle_duration_sec=60.0)
        # erstes gutes Ergebnis ganz übernehmen (Platzhalter ersetzen), danach geglättet nachführen
        T_new = blend(T_base_cam, T_icp, self._p('camera_blend') if self.camera_ok else 1.0)
        if not self.camera_ok or self.camera_timer.timer_period_ns != int(self._p('period_tracking') * 1e9):
            self.set_period(self._p('period_tracking'))
        self.camera_ok = True
        self.send(cam, T_new)
        r, p_, y = (math.degrees(a) for a in matrix_to_rpy(T_new[:3, :3]))
        x, yy, z = T_new[:3, 3]
        self.get_logger().info(f'Kamera: {info} → x {x:+.3f} y {yy:+.3f} z {z:+.3f} m, '
                               f'roll {r:+.1f}° pitch {p_:+.1f}° yaw {y:+.1f}°', throttle_duration_sec=10.0)

    def coarse_search(self, src_map, tgt_map, T_map_base, T_base_cam):
        """Kameramontage im Raster um die aktuelle Schätzung variieren (x/y/z und yaw um die Senkrechte),
        Anteil der Kamerapunkte mit Lidar-Partner < coarse_inlier_dist zählen. Rückgabe: Startkorrektur dT in map."""
        from scipy.spatial import cKDTree
        tree = cKDTree(tgt_map)
        rng = np.random.default_rng(0)
        k = min(self._p('coarse_points'), len(src_map))
        pts = src_map[rng.choice(len(src_map), k, replace=False)]
        # Punkte in base (mit der aktuellen Montage), dort variieren: Drehung um die Senkrechte durch die Kamera
        T_base_map = inv(T_map_base)
        p_base = transform_points(T_base_map, pts)
        c = T_base_cam[:3, 3]
        d_in = self._p('coarse_inlier_dist')

        def score(yaw, off):
            q = (p_base - c) @ rot_z_np(yaw).T + c + off
            dist, _ = tree.query(transform_points(T_map_base, q), distance_upper_bound=d_in)
            return float(np.mean(np.isfinite(dist)))

        best = (-1.0, 0.0, np.zeros(3))
        for stage in ('coarse', 'fine'):
            r, st = self._p(f'{stage}_xyz'), self._p(f'{stage}_xyz_step')
            ya, ys = math.radians(self._p(f'{stage}_yaw_deg')), math.radians(self._p(f'{stage}_yaw_step_deg'))
            offs = np.arange(-r, r + 1e-9, st)
            yaw0, off0 = best[1], best[2]
            for yaw in np.arange(-ya, ya + 1e-9, ys) + yaw0:
                for dx in offs:
                    for dy in offs:
                        for dz in offs:
                            off = off0 + (dx, dy, dz)
                            sc = score(yaw, off)
                            if sc > best[0]:
                                best = (sc, yaw, off)
        score, yaw, off = best
        # Variation als Pose in base: p' = Rz (p - c) + c + off  → Korrektur in map
        D_base = pose(rot_z_np(yaw), c - rot_z_np(yaw) @ c + off)
        self.get_logger().info(f'Kamera-Grobsuche: bester Treffer {score:.0%} bei Verschiebung '
                               f'{off[0]:+.2f} {off[1]:+.2f} {off[2]:+.2f} m, yaw {math.degrees(yaw):+.0f}°')
        return T_map_base @ D_base @ T_base_map

    def set_period(self, period):
        """Kamera-Timer neu stellen (schnell bis zum ersten Treffer, danach period_tracking)."""
        if self.camera_timer.timer_period_ns == int(period * 1e9):
            return
        self.camera_timer.cancel()
        self.destroy_timer(self.camera_timer)
        self.camera_timer = self.create_timer(period, self.camera_step)
        self.get_logger().info(f'Kamera-ICP jetzt alle {period:g} s')

    def report(self):
        if self.stats:
            self.get_logger().info('Kamera, letzte 30 s: ' + ', '.join(f'{k} {v}×' for k, v in self.stats.most_common()))
            self.stats.clear()

    # ---------------------------------------------------------------- Lidar-yaw aus Geradeausfahrt
    def lidar_yaw_step(self):
        win = self._p('yaw_window')
        if len(self.lio) < 2 or not self.wheel:
            return
        t1, R1, p1 = self.lio[-1]
        start = next(((t, R, p) for t, R, p in reversed(self.lio) if t1 - t >= win), None)
        if start is None:
            return
        t0, R0, p0 = start
        wheel = [(v, w) for t, v, w in self.wheel if t0 <= t <= t1]
        if len(wheel) < 5:
            return
        vs = [v for v, _ in wheel]
        straight = all(abs(w) < self._p('yaw_max_wheel_w') for _, w in wheel)
        moving = all(abs(v) > self._p('yaw_min_wheel_v') for v in vs) and (all(v > 0 for v in vs) or all(v < 0 for v in vs))
        turn = math.degrees(rotation_angle(R0.T @ R1))
        d_body = R0.T @ (p1 - p0)
        dist = float(np.linalg.norm(d_body))
        if not (straight and moving) or turn > self._p('yaw_max_turn_deg') or dist < self._p('yaw_min_dist'):
            return
        # Abschnitt verbraucht: nicht doppelt zählen
        while len(self.lio) > 1 and self.lio[0][0] < t1:
            self.lio.popleft()
        try:
            T_base_lidar = tf_to_T(self.tf_buffer.lookup_transform(self._p('base_frame'), self._p('lidar_frame'),
                                                                   Time()))
        except TransformException:
            return
        roll, pitch, yaw_old = matrix_to_rpy(T_base_lidar[:3, :3])
        sign = 1.0 if vs[0] > 0 else -1.0
        self.yaw_samples.append((yaw_from_motion(sign * d_body, roll, pitch), dist))
        total = sum(w for _, w in self.yaw_samples)
        mean = circular_mean([a for a, _ in self.yaw_samples], [w for _, w in self.yaw_samples])
        spread = math.degrees(math.sqrt(sum(w * wrap(a - mean) ** 2 for a, w in self.yaw_samples) / total))
        if total < self._p('yaw_min_total'):
            self.get_logger().info(f'Lidar-yaw: {math.degrees(mean):+.1f}° aus {total * 100:.0f} cm Fahrt '
                                   f'(Streuung {spread:.1f}°), sammle noch bis {self._p("yaw_min_total") * 100:.0f} cm')
            return
        yaw_new = yaw_old + self._p('yaw_blend') * wrap(mean - yaw_old)
        T_new = pose(rpy_to_matrix(roll, pitch, yaw_new), T_base_lidar[:3, 3])
        self.send(self._p('lidar_frame'), T_new)
        self.get_logger().info(f'Lidar-yaw: gemessen {math.degrees(mean):+.1f}° (Streuung {spread:.1f}°, '
                               f'{total * 100:.0f} cm Fahrt) → Montage {math.degrees(yaw_new):+.1f}°')

    # ---------------------------------------------------------------- Ausgabe
    def send(self, child, T):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._p('base_frame')
        msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (float(c) for c in T[:3, 3])
        q = matrix_to_quat(T[:3, :3])
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        self.update_pub.publish(msg)
        self.save(child, T)

    def save(self, child, T):
        """Aktuelle Montagen in calib_file schreiben (höchstens alle 10 s), mount_tf lädt sie beim Start."""
        now = self.get_clock().now()
        if self.last_save is not None and now - self.last_save < Duration(seconds=10.0):
            return
        self.last_save = now
        path = self._p('calib_file')
        data = {}
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        for frame in (self._p('lidar_frame'), self._p('camera_frame')):
            try:
                Tf = T if frame == child else tf_to_T(self.tf_buffer.lookup_transform(self._p('base_frame'), frame, Time()))
            except TransformException:
                continue
            r, p, y = matrix_to_rpy(Tf[:3, :3])
            data[frame] = {k: round(float(v), 5) for k, v in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
                                                                 (*Tf[:3, 3], r, p, y))}
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            f.write('# Von slop_slam/mount_calib aus LIO nachgeführt, von slop_sensors/mount_tf beim Start geladen.\n'
                    '# Löschen = wieder mit mounts.yaml anfangen. Frame → x/y/z [m], roll/pitch/yaw [rad]\n')
            yaml.safe_dump(data, f, sort_keys=False)


def main(args=None):
    rclpy.init(args=args)
    node = MountCalibNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
