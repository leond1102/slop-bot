#!/usr/bin/env python3
"""Fahrsperre ohne RustDesk-Sitzung: Der Roboter darf nur fahren, wenn jemand per RustDesk zuschaut.

Topics:
  safety/remote_lock    std_msgs/Bool  Pub  true = gesperrt. Lock für twist_mux und diff_drive_node.
  safety/remote_active  std_msgs/Bool  Pub  true = RustDesk-Sitzung erkannt (nur zur Anzeige)

Sperrverzögerung: Gesperrt wird erst, wenn die Sitzung lock_delay s lang ununterbrochen fehlt. Freigabe
kommt sofort. Achtung: Bei Netzausfall kommt max_silence noch dazu (Erkennung), Sperre also nach bis zu
max_silence + lock_delay s.

Fail-Safe: Startet gesperrt. Jeder Fehler bei der Prüfung zählt als "keine Sitzung". Stürzt die Node ab,
kommen keine Nachrichten mehr und twist_mux / diff_drive_node behandeln die Sperre nach ihrem Timeout als aktiv.
Erkennung siehe slop_safety/remote_guard.py.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from slop_safety.remote_guard import LockDelay, session_state


class RemoteGuardNode(Node):

    def __init__(self):
        super().__init__('remote_guard')
        for name, default in [
            ('rate', 5.0),               # Hz; Sperr-Timeout bei den Empfängern muss deutlich länger sein
            ('max_silence', 1.5),        # s ohne Daten vom Client → Sitzung gilt als weg
            ('lock_delay', 0.1),         # s, so lange muss die Sitzung ununterbrochen fehlen → Sperre
            ('require_cm', True),        # zusätzlich "rustdesk --cm" verlangen
            ('direct_port', 21118),
            ('relay_port', 21117),
        ]:
            self.declare_parameter(name, default)
        self.lock_pub = self.create_publisher(Bool, 'safety/remote_lock', 10)
        self.active_pub = self.create_publisher(Bool, 'safety/remote_active', 10)
        self.delay = LockDelay(self._p('lock_delay'))
        self.waiting = False             # Sitzung weg, Sperre läuft noch nicht
        self.last_why = None
        self.create_timer(1.0 / self._p('rate'), self.check)
        self.get_logger().info('RustDesk-Fahrsperre aktiv: ohne Sitzung kein Fahren')

    def _p(self, name):
        return self.get_parameter(name).value

    def check(self):
        try:
            active, why = session_state(self._p('direct_port'), self._p('relay_port'),
                                        self._p('max_silence'), self._p('require_cm'))
        except Exception as e:
            active, why = False, f'Prüfung fehlgeschlagen ({e})'
        was_allowed = self.delay.allowed
        now = time.monotonic()
        allowed = self.delay.update(active, now)
        self.lock_pub.publish(Bool(data=not allowed))
        self.active_pub.publish(Bool(data=active))

        # Nur Zustandswechsel loggen (why enthält wechselnde Zahlen)
        waiting = self.delay.remaining(now) is not None
        if allowed and not was_allowed:
            self.get_logger().info(f'{why} → Fahren erlaubt')
        elif was_allowed and not allowed:
            self.get_logger().warn(f'{why} → Fahren GESPERRT (seit {self._p("lock_delay"):g} s keine Sitzung)')
        elif waiting and not self.waiting:
            self.get_logger().warn(f'{why} → Sperre in {self._p("lock_delay"):g} s, falls sie nicht zurückkommt')
        elif self.waiting and not waiting and allowed:
            self.get_logger().info('Sitzung wieder da, keine Sperre')
        elif self.last_why is None and not allowed:
            self.get_logger().warn(f'{why} → Fahren GESPERRT')
        self.waiting = waiting
        self.last_why = why


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RemoteGuardNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
