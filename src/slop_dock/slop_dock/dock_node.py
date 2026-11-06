#!/usr/bin/env python3
"""Dockingstation: ArUco-Marker mit der RealSense erkennen, in der Karte vermerken und mittig davor anhalten.

  /camera/color/image_raw, /camera/color/camera_info   Sub  Farbbild der RealSense (Marker ID/Wörterbuch/Größe: Parameter).
                                                       Das Farbbild wird NUR abonniert, solange es gebraucht wird (Suche/
                                                       Feinfahren, „Dock speichern“/„Andocken“ prüft kurz, oder jemand hört
                                                       dock/image ab) – im Leerlauf kostet der Knoten fast nichts.
  /scan                                                Sub  nur als Wächter beim Feinfahren (Hindernis direkt vor dem Roboter)
  /safety/stop                                         Sub  Notaus/Hand-Stopp bricht das Andocken ab
  dock/cmd_vel            geometry_msgs/Twist          Pub  Feinfahren → twist_mux (Eingang „dock“, Priorität 40)
  dock/marker             geometry_msgs/PoseStamped    Pub  erkannter Marker in base_footprint (x zeigt aus dem Marker heraus)
  dock/image              sensor_msgs/Image            Pub  DEBUG-Bild mit Overlay (alle erkannten Marker mit ID, verworfene
                                                       Kandidaten, Pose, Hinweis auf falsches Wörterbuch); nur, wenn jemand
                                                       zuhört: rqt_image_view oder die Weboberfläche
  dock/status             std_msgs/String              Pub  Text zum Stand (transient local)
  /dock/save, /dock/start, /dock/cancel                std_srvs/Trigger

Ablauf `/dock/start`: (1) Nav2 fährt zu einem Punkt `approach_distance` vor dem gespeicherten Marker, Blick zur Wand
(entfällt, wenn der Marker schon in Sichtweite ist), (2) Feinfahren mit der Kamera: auf die Markernormale ausrichten und
bei `stop_x` (Marker-Abstand von der Antriebsachse) anhalten. Nach jedem Schritt prüft der Knoten, ob der Marker noch zu
sehen ist (`marker_timeout`) und ob direkt vor dem Roboter nichts liegt. Fehler stoppen sofort.
Das Feinfahren umgeht die Collision Monitore von Nav2 (Tempo deshalb höchstens `v_max`).
"""
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from slop_dock import dock_core as dc


cv2.setNumThreads(1)                       # OpenCV sonst mit allen Kernen: stört LIO und Nav2


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockNode(Node):

    def __init__(self):
        super().__init__('dock')
        for name, default in [
            ('dock_file', ''),                 # maps/<karte>.dock.yaml (setzt die Launch)
            ('marker_id', 1), ('marker_size', 0.10), ('dictionary', 'ARUCO_ORIGINAL'),
            ('image_topic', '/camera/color/image_raw'), ('info_topic', '/camera/color/camera_info'),
            ('base_frame', 'base_footprint'), ('map_frame', 'map'),
            ('max_rate', 10.0),                # Hz höchstens Erkennung (Bilder dazwischen werden verworfen)
            ('idle_detection', False),         # true: Bilder auch im Leerlauf auswerten (Dauerlast, nur zum Testen)
            ('probe_period', 1.5),             # s: bei „nichts gefunden“ im Debug-Bild andere Wörterbücher probieren
            ('max_tilt', 0.2),                 # Marker muss senkrecht hängen (z-Anteil der Normalen)
            ('approach_distance', 1.1),        # m vor dem Marker (Antriebsachse), Nav2-Ziel
            ('use_nav2', True),
            ('nav2_timeout', 240.0),
            ('stop_x', 0.48),                  # m: Marker-Abstand von der Antriebsachse am Ziel (Kamera sitzt 8 cm weiter vorn)
            ('v_max', 0.05), ('v_min', 0.02), ('w_max', 0.3),
            ('tol_x', 0.03), ('tol_y', 0.03), ('tol_yaw_deg', 6.0),       # lockere Grenzen
            ('marker_timeout', 1.0),           # s ohne Marker beim Feinfahren → Abbruch
            ('servo_timeout', 90.0),
            ('settle_time', 1.0),              # s stehen und mitteln, bevor „fertig“ oder „zu ungenau“ gilt
            ('wait_marker', 4.0),              # s Suchzeit nach Nav2, bevor es als „nicht gesehen“ gilt
            ('guard_scan', True),              # Lidar-Wächter beim Feinfahren
            ('guard_x_min', 0.10), ('guard_x_max', 0.20), ('guard_half_width', 0.19),
        ]:
            self.declare_parameter(name, default)
        self.detector = dc.MarkerDetector(self._p('dictionary'), self._p('marker_id'), self._p('marker_size'))
        self.servo = dc.Servo(stop=self._p('stop_x'), v_max=self._p('v_max'), v_min=self._p('v_min'),
                              w_max=self._p('w_max'), tol_x=self._p('tol_x'), tol_y=self._p('tol_y'),
                              tol_yaw=math.radians(self._p('tol_yaw_deg')))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.K = self.D = None
        self.last_det = 0.0                    # monotone Zeit der letzten Erkennungsrechnung (Ratenbegrenzung)
        self.marker = None                     # (x, y, n, stamp_mono, dist) geglättet, in base_footprint
        self.scan = None                       # (monotone Zeit, Punkte in base_footprint)
        self.state, self.state_since, self.message = 'bereit', time.monotonic(), ''
        self.goal_handle = None
        self.estop = False
        self.zero_ticks = 0
        self.samples = []                      # Marker-Messungen beim Prüfen am Ziel
        self.image_sub = None                  # nur bei Bedarf (siehe manage)
        self.image_last_wanted = 0.0
        self.waiters = []                      # [Event, Ergebnis] – Dienste, die auf die nächste Marker-Erkennung warten
        self.probe_t = 0.0
        self.probe_hint = []
        self.rejected_n = 0
        self.stats = {'frames': 0, 'seen': 0}

        self.info_sub = self.create_subscription(CameraInfo, self._p('info_topic'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/safety/stop', lambda m: self.on_estop(m.data), 10)
        self.cmd_pub = self.create_publisher(Twist, 'dock/cmd_vel', 10)
        self.marker_pub = self.create_publisher(PoseStamped, 'dock/marker', 10)
        self.image_pub = self.create_publisher(Image, 'dock/image', 1)
        self.status_pub = self.create_publisher(String, 'dock/status',
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Dienste in eigener Gruppe: Sie warten (kurz) auf eine Erkennung, während Bild und Regelung weiterlaufen
        svc = ReentrantCallbackGroup()
        self.create_service(Trigger, 'dock/save', self.srv_save, callback_group=svc)
        self.create_service(Trigger, 'dock/start', self.srv_start, callback_group=svc)
        self.create_service(Trigger, 'dock/cancel', self.srv_cancel, callback_group=svc)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.2, self.manage)
        self.create_timer(10.0, self.report)
        self.set_state('bereit', 'Dock-Marker %d (%s, %.0f cm), Dock-Datei %s (Kamera aus, bis sie gebraucht wird)' % (
            self._p('marker_id'), self._p('dictionary'), 100 * self._p('marker_size'), self._p('dock_file') or '(keine)'))

    def _p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ Zustand / Ausgabe
    def set_state(self, state, message=''):
        self.state, self.state_since, self.message = state, time.monotonic(), message
        text = f'{state}: {message}' if message else state
        self.status_pub.publish(String(data=text))
        if state == 'fehler':                       # getrennte Aufrufe: rclpy erlaubt an einer Stelle nicht abwechselnd info und warn
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def busy(self):
        return self.state in ('anfahren', 'suche', 'feinfahren', 'pruefen')

    def send(self, v, w):
        self.cmd_pub.publish(Twist(linear=dc_vec(v), angular=dc_vec(0.0, 0.0, w)))

    # ------------------------------------------------------------------ Kamera
    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
        if self.info_sub is not None:                            # ändert sich nicht: nicht dauernd weiter empfangen
            self.destroy_subscription(self.info_sub)
            self.info_sub = None

    def to_gray(self, msg):
        import cv2
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ('rgb8', 'bgr8'):
            img = data.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)
        if msg.encoding == 'mono8':
            return data.reshape(msg.height, msg.step)[:, :msg.width]
        raise ValueError(f'Bildformat {msg.encoding} nicht unterstützt (rgb8, bgr8, mono8)')

    def watching(self):
        return self.image_pub.get_subscription_count() > 0

    def manage(self):
        """Bildabo ein-/ausschalten und wartende Dienste bedienen (5 Hz)."""
        now = time.monotonic()
        want = (self.state in ('suche', 'feinfahren', 'pruefen') or self.watching() or bool(self.waiters)
                or self._p('idle_detection'))      # beim Nav2-Anfahren (anfahren) braucht es keine Bilder
        if want:
            self.image_last_wanted = now
            if self.image_sub is None:
                self.image_sub = self.create_subscription(Image, self._p('image_topic'), self.on_image,
                                                          qos_profile_sensor_data)
                self.get_logger().info('Farbbild an (%s)' % ('Debug-Ansicht' if self.watching() else 'wird gebraucht'))
        elif self.image_sub is not None and now - self.image_last_wanted > 2.0:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.get_logger().info('Farbbild aus (wird nicht gebraucht)')

    def look(self, seconds):
        """Blockiert (im Dienst-Thread) bis zu `seconds` auf die NÄCHSTE Marker-Erkennung → True/False. Schaltet dafür das
        Farbbild ein (manage, 5 Hz)."""
        w = [threading.Event()]
        self.waiters.append(w)
        try:
            return w[0].wait(seconds)
        finally:
            if w in self.waiters:
                self.waiters.remove(w)

    def on_image(self, msg):
        now = time.monotonic()
        if self.K is None or now - self.last_det < 1.0 / self._p('max_rate'):
            return
        self.last_det = now
        self.stats['frames'] += 1
        try:
            gray = self.to_gray(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        found = self.detector.find(gray)
        self.rejected_n = len(found.rejected)
        cam = None
        try:
            t = self.tf_buffer.lookup_transform(self._p('base_frame'), msg.header.frame_id, Time())
            q, p = t.transform.rotation, t.transform.translation
            cam = dc.quat_to_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])
        except Exception:
            pass                                                 # Kamera-TF noch nicht da (Debug-Bild geht trotzdem)
        hit = self.detector.pose(found, self.K, self.D, cam[0] if cam else None)
        note = None
        if hit is not None and cam is not None:
            x, y, n, tilt = dc.marker_in_base(hit[0], hit[1], cam[0], cam[1])
            if tilt > self._p('max_tilt'):
                note = 'verworfen: hängt nicht senkrecht (Neigung %.2f > %.2f)' % (tilt, self._p('max_tilt'))
            else:
                self.stats['seen'] += 1
                if self.marker is not None and now - self.marker[3] < 0.5:     # glätten (Normale als Vektor)
                    a = 0.5
                    px, py, pn = self.marker[:3]
                    x, y = a * x + (1 - a) * px, a * y + (1 - a) * py
                    n = math.atan2(a * math.sin(n) + (1 - a) * math.sin(pn), a * math.cos(n) + (1 - a) * math.cos(pn))
                self.marker = (x, y, n, now, math.hypot(x, y))
                if self.state == 'pruefen':
                    self.samples.append((x, y, n))
                ps = PoseStamped()
                ps.header.stamp, ps.header.frame_id = msg.header.stamp, self._p('base_frame')
                ps.pose.position.x, ps.pose.position.y = x, y
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(n / 2), math.cos(n / 2)
                self.marker_pub.publish(ps)
                for w in list(self.waiters):
                    w[0].set()
        elif hit is not None:
            note = 'Pose nicht berechenbar: kein TF %s → %s' % (msg.header.frame_id, self._p('base_frame'))
        if self.watching():
            self.publish_debug(msg, gray, found, hit, cam, note)

    def publish_debug(self, msg, gray, found, hit, cam, note):
        """Overlay: was sieht der Knoten? Hilft bei „erkennt keinen Marker“."""
        now = time.monotonic()
        target = self._p('marker_id')
        seen = sorted(set(found.ids))
        lines = ['Ziel: ID %d, Wörterbuch %s, %.0f cm   |   Bild %dx%d, %s' % (
            target, self._p('dictionary'), 100 * self._p('marker_size'), msg.width, msg.height, msg.encoding),
            'Marker im Bild: %s   (verworfene Kandidaten: %d)' % (
                (', '.join('ID %d' % i for i in seen) if seen else 'keiner'), len(found.rejected))]
        hints = []
        if hit is not None and cam is not None:
            x, y, n, tilt = dc.marker_in_base(hit[0], hit[1], cam[0], cam[1])
            xm, ym, th = dc.robot_in_marker(x, y, n)
            lines.append('ZIEL ERKANNT: %.2f m, quer %+.1f cm, Winkel %+.1f°, Neigung %.2f' % (
                math.hypot(x - 0.08, y), 100 * ym, math.degrees(dc.wrap(th - math.pi)), tilt))
        elif seen and target not in seen:
            hints.append('Marker gefunden, aber nicht ID %d (andere ID an der Wand?)' % target)
        elif not seen:
            if now - self.probe_t > self._p('probe_period'):
                self.probe_t = now
                self.probe_hint = self.detector.probe(gray)
            if self.probe_hint:
                hints.append('Kein Treffer im Wörterbuch %s, aber in: %s' % (
                    self._p('dictionary'), '; '.join('%s ID %s' % (nme, ','.join(map(str, ids)))
                                                     for nme, ids in self.probe_hint[:4])))
                hints.append('→ dictionary in config/dock.yaml anpassen')
            else:
                hints.append('Kein Muster gefunden: zu klein/unscharf/blendend, nicht im Bild, falscher Typ?')
        if note:
            hints.append(note)
        if self.busy():
            lines.append('Dock: ' + self.state)
        vis = dc.draw_debug(gray, found, target, lines, hints)
        out = Image(header=msg.header, height=vis.shape[0], width=vis.shape[1], encoding='bgr8',
                    step=vis.shape[1] * 3, data=vis.tobytes())
        self.image_pub.publish(out)

    def visible(self, max_age=0.5):
        return self.marker is not None and time.monotonic() - self.marker[3] <= max_age

    # ------------------------------------------------------------------ Lidar-Wächter
    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float64)
        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        self.scan = (time.monotonic(), r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]))

    def front_blocked(self):
        """Text, wenn der Wächter anschlägt, sonst None."""
        if not self._p('guard_scan'):
            return None
        if self.scan is None or time.monotonic() - self.scan[0] > 1.0:
            return 'kein /scan (Lidar-Wächter)'
        x, y = self.scan[1], self.scan[2]
        hit = (x >= self._p('guard_x_min')) & (x <= self._p('guard_x_max')) & (np.abs(y) <= self._p('guard_half_width'))
        return 'Hindernis direkt vor dem Roboter (Lidar)' if int(hit.sum()) >= 3 else None

    # ------------------------------------------------------------------ Dienste
    def srv_save(self, req, res):
        if not self.look(3.0):                             # schaltet das Farbbild kurz ein und wartet auf den Marker
            res.success = False
            res.message = 'Marker ID %d in 3 s nicht erkannt (Debug-Bild ansehen: dock/image)' % self._p('marker_id')
            return res
        if not self._p('dock_file'):
            res.success, res.message = False, 'keine Dock-Datei (Parameter dock_file)'
            return res
        try:
            t = self.tf_buffer.lookup_transform(self._p('map_frame'), self._p('base_frame'), Time(),
                                                timeout=Duration(seconds=0.5))
        except Exception as e:
            res.success, res.message = False, f'kein TF {self._p("map_frame")} → {self._p("base_frame")} (Lokalisierung läuft?): {e}'
            return res
        bx, by, bn, _, dist = self.marker
        r = t.transform
        x, y, n = dc.marker_to_map(bx, by, bn, r.translation.x, r.translation.y, yaw_of(r.rotation))
        dc.save_dock(self._p('dock_file'), self._p('marker_id'), self._p('marker_size'), x, y, n)
        res.success = True
        res.message = 'Dock gespeichert: Marker bei x %.2f, y %.2f, Normale %.0f° (%.2f m entfernt) → %s' % (
            x, y, math.degrees(n), dist, self._p('dock_file'))
        self.get_logger().info(res.message)
        return res

    def srv_cancel(self, req, res):
        if not self.busy():
            res.success, res.message = True, 'Es läuft kein Andocken'
            return res
        self.abort('abgebrochen')
        res.success, res.message = True, 'abgebrochen'
        return res

    def srv_start(self, req, res):
        self.get_logger().info('Dienst /dock/start aufgerufen (Zustand %s)' % self.state)
        if self.busy():
            res.success, res.message = False, 'Andocken läuft schon'
            return res
        if self.estop:
            res.success, res.message = False, 'Notaus aktiv'
            return res
        self.look(1.5)                                           # steht der Roboter schon vor dem Marker? (Farbbild kurz an)
        if self.busy():                                          # während des Wartens schon gestartet
            res.success, res.message = False, 'Andocken läuft schon'
            return res
        if self.visible() and self.marker[0] > 0.2 and self.marker[0] <= self._p('approach_distance') + 0.5 \
                and abs(self.marker[1]) < 0.6:
            self.start_servo('Marker schon in Sicht (%.2f m), ohne Nav2' % self.marker[4])
            res.success, res.message = True, self.message
            return res
        if not self._p('use_nav2'):
            res.success, res.message = False, 'Marker nicht in Sicht (use_nav2 aus)'
            return res
        try:
            d = dc.load_dock(self._p('dock_file'))
        except (OSError, ValueError) as e:
            res.success, res.message = False, f'kein gespeicherter Dock ({self._p("dock_file") or "keine Datei"}): {e}'
            return res
        if not self.nav.wait_for_server(timeout_sec=1.0):
            res.success, res.message = False, 'Nav2 läuft nicht (navigate_to_pose)'
            return res
        x, y, yaw = dc.approach_pose((d['x'], d['y'], d['normal']), self._p('approach_distance'))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._p('map_frame')
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.nav_goal = (x, y, yaw)
        self.nav.send_goal_async(goal).add_done_callback(self.on_goal_response)
        self.set_state('anfahren', 'Nav2 fährt zum Anfahrpunkt x %.2f, y %.2f, %.0f°' % (x, y, math.degrees(yaw)))
        res.success, res.message = True, self.message
        return res

    # ------------------------------------------------------------------ Nav2-Ergebnis
    def on_goal_response(self, fut):
        h = fut.result()
        if self.state != 'anfahren':
            if h is not None and h.accepted:
                h.cancel_goal_async()
            return
        if h is None or not h.accepted:
            return self.fail('Nav2 hat das Ziel abgelehnt')
        self.goal_handle = h
        h.get_result_async().add_done_callback(self.on_nav_result)

    def on_nav_result(self, fut):
        if self.state != 'anfahren':
            return
        status = fut.result().status
        self.goal_handle = None
        if status == 4:                                         # SUCCEEDED
            self.set_state('suche', 'am Anfahrpunkt, suche den Marker')
        else:
            self.fail('Nav2 hat den Anfahrpunkt nicht erreicht (Status %d)' % status)

    # ------------------------------------------------------------------ Ablauf
    def start_servo(self, why):
        self.servo_t0 = time.monotonic()
        self.set_state('feinfahren', why)

    def fail(self, text):
        self.stop_motion()
        self.set_state('fehler', text)

    def abort(self, text):
        self.stop_motion()
        self.set_state('abgebrochen', text)

    def stop_motion(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.zero_ticks = 6
        self.send(0.0, 0.0)

    def on_estop(self, value):
        self.estop = bool(value)
        if value and self.busy():
            self.abort('Notaus')

    def tick(self):
        now = time.monotonic()
        if self.zero_ticks > 0:                                 # nach dem Ende ein paar Nullbefehle, dann ist Ruhe
            self.zero_ticks -= 1
            self.send(0.0, 0.0)
        if self.state == 'anfahren':
            if now - self.state_since > self._p('nav2_timeout'):
                self.fail('Nav2 braucht zu lange (%.0f s)' % self._p('nav2_timeout'))
        elif self.state == 'suche':
            if self.visible():
                self.start_servo('Marker in Sicht (%.2f m)' % self.marker[4])
            elif now - self.state_since > self._p('wait_marker'):
                self.fail('Marker ID %d vom Anfahrpunkt aus nicht zu sehen (Dock neu speichern? Beleuchtung?)'
                          % self._p('marker_id'))
        elif self.state == 'feinfahren':
            self.servo_tick(now)
        elif self.state == 'pruefen':
            self.settle_tick(now)

    def servo_tick(self, now):
        if now - self.servo_t0 > self._p('servo_timeout'):
            return self.fail('Feinfahren dauert zu lange (%.0f s)' % self._p('servo_timeout'))
        if not self.visible(self._p('marker_timeout')):
            return self.fail('Marker verloren (%.1f s nicht gesehen)' % self._p('marker_timeout'))
        blocked = self.front_blocked()
        if blocked:
            return self.fail(blocked)
        if not self.visible(0.35):                              # kurz ohne neue Messung: stehen, nicht blind weiterfahren
            self.send(0.0, 0.0)
            return
        mx, my, n = self.marker[:3]
        v, w, state = self.servo.command(mx, my, n)
        if state == 'fahren':
            self.send(v, w)
            return
        # am Haltepunkt: stehen bleiben und die Messung mitteln, bevor geurteilt wird
        self.stop_motion()
        self.samples = []
        self.set_state('pruefen', 'am Haltepunkt, prüfe Lage')

    def settle_tick(self, now):
        if now - self.state_since < self._p('settle_time'):
            return
        if len(self.samples) < 3:
            return self.fail('Marker am Ziel nicht sicher erkannt (%d Messungen)' % len(self.samples))
        a = np.array(self.samples)
        mx, my = float(np.median(a[:, 0])), float(np.median(a[:, 1]))
        n = math.atan2(float(np.mean(np.sin(a[:, 2]))), float(np.mean(np.cos(a[:, 2]))))
        _, _, state = self.servo.command(mx, my, n)
        xm, ym, th = dc.robot_in_marker(mx, my, n)
        detail = 'Abstand %.0f cm (Kamera), quer %+.1f cm, Winkel %+.1f° (Mittel aus %d Messungen)' % (
            100 * (xm - 0.08), 100 * ym, math.degrees(dc.wrap(th - math.pi)), len(self.samples))
        if state == 'fertig':
            self.set_state('fertig', detail)
        elif state == 'ungenau':
            self.set_state('fehler', 'angekommen, aber zu ungenau: ' + detail)
        else:                                                    # Haltepunkt noch nicht erreicht: weiterfahren
            self.servo_t0 = time.monotonic()
            self.set_state('feinfahren', 'noch ' + detail)

    def report(self):
        s = self.stats
        if s['frames']:
            self.get_logger().info('letzte 10 s: %d Bilder ausgewertet, Marker in %d gesehen' % (s['frames'], s['seen']))
        self.stats = dict.fromkeys(s, 0)


def dc_vec(x=0.0, y=0.0, z=0.0):
    from geometry_msgs.msg import Vector3
    return Vector3(x=float(x), y=float(y), z=float(z))


def main(args=None):
    rclpy.init(args=args)
    node = DockNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop_motion()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
