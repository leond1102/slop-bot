"""ROS-Seite der Weboberfläche: Fahren (Totmann), Notaus, Status. Läuft in einem eigenen Thread neben dem Webserver.

  teleop/cmd_vel          geometry_msgs/Twist   Pub  wie keyboard_teleop: Eingang „teleop“ von twist_mux (Priorität 50)
  /safety/stop            std_msgs/Bool         Pub/Sub  Notaus = Hand-Stopp-Sperre von twist_mux (Priorität 100)
  /safety/remote_lock     std_msgs/Bool         Sub  Fahrsperre ohne RustDesk (nur Anzeige, wird NICHT umgangen)
  /safety/remote_active   std_msgs/Bool         Sub  RustDesk-Sitzung erkannt (Anzeige)
  /relocalize/score       std_msgs/Float32      Sub  Lokalisierung (Anzeige)
  /wheel/odom             nav_msgs/Odometry     Sub  Geschwindigkeit, „Antrieb läuft“
  /scan, /camera/obstacles, /map                Sub  nur „kommen Daten?“ (roh, ohne Auspacken)
  dock/image              sensor_msgs/Image     Sub  Debug-Bild der Dockingstation (Overlay) – NUR solange eine Seite es anschaut
  /safety/sensor_status   std_msgs/String       Sub  Sensor-Wächter: „frei“ oder „gesperrt: …“ (Anzeige, alle 1 s)
  /dock/status            std_msgs/String       Sub  Dockingstation: „zustand: text“ (Anzeige, nur bei Änderung gesendet)

Notaus: setzt die Hand-Stopp-Sperre (/safety/stop true) und bricht laufende Nav2-Ziele ab, damit der Roboter nach dem
Aufheben nicht einfach weiterfährt. Solange er aktiv ist, wird true alle 0,5 s wiederholt (überlebt einen Neustart von
twist_mux). Aufheben (false) – auch aus dem Terminal – beendet das Wiederholen. Achtung: Das ist ein Software-Stopp über
Netzwerk, kein Not-Halt nach Norm.
"""
import threading
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, String

from slop_web.drive import Drive


class Bridge(Node):
    def __init__(self, limits):
        super().__init__('slop_web')
        self.lock = threading.Lock()
        self.drive = Drive(limits)
        self.estop = None               # letzter Wert auf /safety/stop (None = noch nichts gesehen)
        self.estop_hold = False         # wir wiederholen true
        self.seen = {}                  # Topic → (Wert, monotone Zeit)
        self.cmd_pub = self.create_publisher(Twist, 'teleop/cmd_vel', 10)
        self.stop_pub = self.create_publisher(Bool, '/safety/stop', 10)
        self.create_subscription(Bool, '/safety/stop', self.on_stop, 10)
        for topic in ('/safety/remote_lock', '/safety/remote_active'):
            self.create_subscription(Bool, topic, lambda m, t=topic: self._seen(t, m.data), 10)
        self.create_subscription(Float32, '/relocalize/score', lambda m: self._seen('/relocalize/score', m.data), 10)
        self.create_subscription(Odometry, '/wheel/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', lambda m: self._seen('/scan', None), qos_profile_sensor_data,
                                 raw=True)
        self.create_subscription(PointCloud2, '/camera/obstacles', lambda m: self._seen('/camera/obstacles', None),
                                 qos_profile_sensor_data, raw=True)
        # slam_toolbox sendet /map alle 2 s: zeigt, dass Kartieren/Erweitern wirklich läuft (nicht nur der Lidar)
        self.create_subscription(OccupancyGrid, '/map', lambda m: self._seen('/map', None), 1, raw=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/safety/sensor_status', lambda m: self._seen('/safety/sensor_status', m.data), 10)
        self.create_subscription(String, '/dock/status', lambda m: self._seen('/dock/status', m.data), latched)
        self.viewers = 0                # Seiten, die das Dock-Debug-Bild gerade ansehen
        self.dock_sub = None
        self.dock_frame_data = None     # (bgr8-Bytes, Breite, Höhe, Zähler, monotone Zeit)
        self.dock_count = 0
        self.create_timer(0.5, self.manage_dock_view)
        self.cancel_clients = [self.create_client(CancelGoal, f'/{a}/_action/cancel_goal')
                               for a in ('navigate_to_pose', 'navigate_through_poses')]
        self.create_timer(0.05, self.send_drive)        # 20 Hz wie keyboard_teleop
        self.create_timer(0.5, self.repeat_estop)

    # ---------------------------------------------------------------- Eingänge
    def _seen(self, topic, value):
        with self.lock:
            self.seen[topic] = (value, time.monotonic())

    def on_odom(self, msg):
        t = msg.twist.twist
        self._seen('/wheel/odom', (t.linear.x, t.angular.z))

    def on_stop(self, msg):
        with self.lock:
            self.estop = msg.data
            if not msg.data:
                self.estop_hold = False         # aufgehoben (auch aus dem Terminal)

    # ---------------------------------------------------------------- Dock-Debug-Bild
    def viewer(self, delta):
        with self.lock:
            self.viewers = max(0, self.viewers + delta)

    def manage_dock_view(self):
        """Bild nur abonnieren, wenn jemand zuschaut: Der Dock-Knoten empfängt das Farbbild ebenfalls nur dann."""
        with self.lock:
            want = self.viewers > 0
        if want and self.dock_sub is None:
            self.dock_sub = self.create_subscription(Image, 'dock/image', self.on_dock_image, 1)
        elif not want and self.dock_sub is not None:
            self.destroy_subscription(self.dock_sub)
            self.dock_sub = None
            with self.lock:
                self.dock_frame_data = None

    def on_dock_image(self, m):
        if m.encoding != 'bgr8':
            return
        with self.lock:
            self.dock_count += 1
            self.dock_frame_data = (bytes(m.data), m.width, m.height, self.dock_count, time.monotonic())

    def dock_frame(self):
        with self.lock:
            return self.dock_frame_data

    # ---------------------------------------------------------------- Fahren
    def drive_command(self, v, w, scale):
        with self.lock:
            if self.estop_hold or self.estop:
                return False                    # bei Notaus nicht einmal Befehle senden
            self.drive.command(v, w, scale, time.monotonic())
            return True

    def drive_halt(self):
        with self.lock:
            self.drive.halt(time.monotonic())

    def send_drive(self):
        with self.lock:
            out = self.drive.output(time.monotonic())
        if out is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = out
            self.cmd_pub.publish(msg)

    # ---------------------------------------------------------------- Notaus
    def set_estop(self, on):
        with self.lock:
            self.estop_hold = on
            self.estop = on
            self.drive.halt(time.monotonic())
        self.stop_pub.publish(Bool(data=on))
        if on:
            for c in self.cancel_clients:
                if c.service_is_ready():
                    c.call_async(CancelGoal.Request())   # leere Anfrage = alle Ziele

    def repeat_estop(self):
        with self.lock:
            hold = self.estop_hold
        if hold:
            self.stop_pub.publish(Bool(data=True))

    # ---------------------------------------------------------------- Status
    def fresh(self, topic, max_age=1.0):
        with self.lock:
            v = self.seen.get(topic)
        return v is not None and time.monotonic() - v[1] <= max_age

    def status(self):
        now = time.monotonic()
        with self.lock:
            seen = dict(self.seen)
            estop, driving = self.estop, self.drive.active

        def val(topic, max_age):
            v = seen.get(topic)
            return v[0] if v is not None and now - v[1] <= max_age else None

        odom = val('/wheel/odom', 1.0)
        lock = val('/safety/remote_lock', 1.5)
        return {
            'estop': estop,
            'driving': driving,
            'remote_guard': lock is not None,               # remote_guard läuft (gehört zum Antrieb)
            'remote_lock': lock,                            # True = gesperrt (keine RustDesk-Sitzung)
            'remote_active': val('/safety/remote_active', 1.5),
            'drive_running': odom is not None,
            'speed': odom[0] if odom else None,
            'turn': odom[1] if odom else None,
            'score': val('/relocalize/score', 3.0),
            'scan': self.fresh('/scan', 1.0),
            'camera': self.fresh('/camera/obstacles', 1.5),
            'sensor': val('/safety/sensor_status', 3.0),    # Wächter sendet jede Sekunde; verstummt er, ist es None
            'dock_image': self.dock_frame_data is not None and now - self.dock_frame_data[4] < 2.0,
            'dock': (seen.get('/dock/status') or (None,))[0],   # nur bei Änderung gesendet → bleibt stehen
        }
