#!/usr/bin/env python3
"""Sensor-Wächter: sperrt Nav2, wenn eine Quelle der Collision Monitore ausfällt oder veraltet ist.

Topics:
  (sources)              je Quelle ein Abo (Typ aus der Config)   Sub  nur der Header-Zeitstempel zählt
  safety/sensor_lock     std_msgs/Bool                            Pub  true = gesperrt. Lock für twist_mux (Priorität 30)
  safety/sensor_status   std_msgs/String                          Pub  „frei“ oder „gesperrt: …“ (bei Änderung + 1 Hz)

Der Collision Monitor von Nav2 (Humble) ignoriert Quellen ohne frische Daten oder ohne TF still (kein Stopp). Der
Wächter sperrt vorher. Die Sperre hat Priorität 30: Nav2 (20) steht, Tastatur und Web (50) fahren weiter – die gehen
ohnehin nicht durch die Collision Monitore.

Fail-Safe: Startet gesperrt. Stürzt die Node ab oder läuft nicht, kommt nichts auf safety/sensor_lock und twist_mux
behandelt die Sperre nach 0,5 s als aktiv. Nav2 fährt also nie ohne Wächter.
Prüfregeln siehe slop_safety/sensor_guard.py, Quellen in config/sensor_guard.yaml.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, String

from slop_safety.sensor_guard import Guard, Source

# Standard, falls die Config fehlt (gleich wie config/sensor_guard.yaml). camera ist bekannt, aber vorerst nicht in der Liste.
DEFAULT_SOURCES = {
    'scan': ('/scan', 'sensor_msgs/msg/LaserScan', 0.4),
    'unknown': ('/safety/unknown_points', 'sensor_msgs/msg/PointCloud2', 0.4),
    'camera': ('/camera/obstacles', 'sensor_msgs/msg/PointCloud2', 0.4),
    'odom': ('/odom', 'nav_msgs/msg/Odometry', 0.3),
}
DEFAULT_LIST = ['scan', 'unknown', 'odom']


class SensorGuardNode(Node):

    def __init__(self):
        super().__init__('sensor_guard')
        for name, default in [
            ('rate', 10.0),              # Hz Prüf- und Sendetakt; twist_mux-Timeout der Sperre muss deutlich länger sein
            ('release_after', 1.0),      # s alle Quellen in Ordnung → Freigabe
            ('max_future', 0.2),         # s, so weit darf ein Zeitstempel in der Zukunft liegen
            ('report_period', 60.0),     # s; Log mit dem höchsten Alter je Quelle, 0 = aus
            ('sources', DEFAULT_LIST),
        ]:
            self.declare_parameter(name, default)

        self.sources = []
        for name in self._p('sources'):
            topic, type_name, max_age = DEFAULT_SOURCES.get(name, ('', '', 0.4))
            self.declare_parameter(f'{name}.topic', topic)
            self.declare_parameter(f'{name}.type', type_name)
            self.declare_parameter(f'{name}.max_age', max_age)
            topic, type_name = self._p(f'{name}.topic'), self._p(f'{name}.type')
            if not topic or not type_name:
                raise ValueError(f'Quelle {name}: topic und type angeben')
            src = Source(topic, float(self._p(f'{name}.max_age')), float(self._p('max_future')))
            # Best Effort passt zu jedem Sender (zuverlässig oder nicht)
            self.create_subscription(get_message(type_name), topic, lambda msg, s=src: self.on_msg(s, msg),
                                     qos_profile_sensor_data)
            self.sources.append(src)
        if not self.sources:
            raise ValueError('keine Quellen (Parameter sources)')
        self.guard = Guard(self.sources, self._p('release_after'))

        self.lock_pub = self.create_publisher(Bool, 'safety/sensor_lock', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'safety/sensor_status', latched)
        self.last_status = None
        self.last_status_pub = 0.0
        self.problem_set = None          # Quellen mit Problem, für Logs nur bei Änderung
        self.was_locked = True
        self.locks = 0                   # Sperren seit dem letzten Bericht (nach einer Freigabe)
        self.create_timer(1.0 / self._p('rate'), self.check)
        if self._p('report_period') > 0:
            self.create_timer(self._p('report_period'), self.report)
        limits = ', '.join(f'{s.label} ≤ {s.max_age:g} s' for s in self.sources)
        self.get_logger().info(f'Sensor-Wächter: {limits}. Nav2 gesperrt, bis alle Quellen frisch sind')

    def _p(self, name):
        return self.get_parameter(name).value

    def on_msg(self, src, msg):
        st = msg.header.stamp
        src.update(st.sec + st.nanosec * 1e-9, time.monotonic())

    def check(self):
        now_mono = time.monotonic()
        locked = self.guard.update(self.get_clock().now().nanoseconds * 1e-9, now_mono)
        self.lock_pub.publish(Bool(data=locked))

        status = self.guard.status(now_mono)
        problems = frozenset(self.guard.problems)
        if locked and not self.was_locked:
            self.locks += 1
            self.get_logger().warn(f'Nav2 GESPERRT: {status[len("gesperrt: "):]}')
        elif self.was_locked and not locked:
            self.get_logger().info('Alle Quellen frisch → Nav2 frei')
        elif locked and problems and problems != self.problem_set:
            self.get_logger().info(f'Nav2 gesperrt: {status[len("gesperrt: "):]}')
        self.was_locked = locked
        self.problem_set = problems

        # Status bei Änderung des Zustands sofort, sonst 1 Hz (Zahlen im Text ändern sich laufend)
        state = (locked, problems)
        if state != self.last_status or now_mono - self.last_status_pub >= 1.0:
            self.status_pub.publish(String(data=status))
            self.last_status, self.last_status_pub = state, now_mono

    def report(self):
        ages = ', '.join(f'{s.label} {s.worst_age:.2f} s' for s in self.sources)
        self.get_logger().info(f'letzte {self._p("report_period"):g} s: höchstes Alter {ages}; '
                               f'{self.locks} Sperren')
        for s in self.sources:
            s.reset_stats()
        self.locks = 0

    def publish_lock(self):
        """Beim Beenden sofort sperren statt auf den twist_mux-Timeout zu warten."""
        self.lock_pub.publish(Bool(data=True))
        self.status_pub.publish(String(data='gesperrt: Sensor-Wächter beendet'))


def main(args=None):
    # Ohne rclpy-Signalhandler: Strg+C kommt als KeyboardInterrupt, der Kontext lebt noch, die letzte Sperre geht raus
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = SensorGuardNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.publish_lock()
            except Exception:
                pass                     # Kontext schon beendet: twist_mux sperrt nach seinem Timeout
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
