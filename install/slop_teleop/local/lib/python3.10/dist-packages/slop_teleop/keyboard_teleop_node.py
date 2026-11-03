#!/usr/bin/env python3
"""Tastatursteuerung für den Slop-Bot. Läuft im Terminal, braucht eine echte Konsole (nicht ros2 launch):

    ros2 run slop_teleop keyboard_teleop

Parameter: config/keyboard_teleop.yaml wird automatisch geladen. Eigene Datei mit
--ros-args --params-file datei.yaml, Einzelwerte mit --ros-args -p name:=wert (beides überschreibt).

Topics:
  teleop/cmd_vel        geometry_msgs/Twist  Pub  Eingang "teleop" von twist_mux (Priorität 50), cmd_vel_topic
  /safety/remote_lock   std_msgs/Bool        Sub  nur zur Anzeige, ob gerade gesperrt ist

Totmann: Gefahren wird nur, solange eine Taste gehalten wird (Erkennung siehe slop_teleop/keys.py).
Nach dem Loslassen kommen stop_hold s lang Nullbefehle, danach sendet die Node nichts mehr. Dann läuft
der teleop-Eingang in twist_mux ab und Navigation kann wieder fahren.
Stürzt die Node ab oder wird das Terminal geschlossen, kommen keine Befehle mehr: twist_mux und
diff_drive_node stoppen nach ihrem Timeout (0,5 s).
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

from slop_teleop.keys import HELP, KeyTeleop, parse_keys


class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        # Standardwerte, falls die Config fehlt. Erklärung der Werte in config/keyboard_teleop.yaml
        for name, default in [
            ('cmd_vel_topic', 'teleop/cmd_vel'),
            ('max_linear', 0.037),
            ('max_angular', 0.31),
            ('scale', 1.0),
            ('scale_step', 0.1),
            ('first_timeout', 0.7),
            ('repeat_timeout', 0.25),
            ('stop_hold', 0.3),
            ('rate', 20.0),
            ('lock_topic', '/safety/remote_lock'),
            ('lock_timeout', 1.0),
        ]:
            self.declare_parameter(name, default)
        step = min(1.0, max(0.01, self._p('scale_step')))
        self.keys = KeyTeleop(self._p('first_timeout'), self._p('repeat_timeout'),
                              min(1.0, max(step, self._p('scale'))), step)
        self.pub = self.create_publisher(Twist, self._p('cmd_vel_topic'), 10)
        self.get_logger().info(f'Sende auf {self.pub.topic_name}, 100 % = {self._p("max_linear"):g} m/s, '
                               f'{self._p("max_angular"):g} rad/s')
        self.stop_until = 0.0
        self.lock_data, self.lock_time = None, None
        if self._p('lock_topic'):
            self.create_subscription(Bool, self._p('lock_topic'), self._lock_cb, 10)
        self.status = None

    def _p(self, name):
        return self.get_parameter(name).value

    def _lock_cb(self, msg):
        self.lock_data, self.lock_time = msg.data, time.monotonic()

    def on_keys(self, data, now):
        for key in parse_keys(data):
            self.keys.press(key, now)

    def tick(self, now):
        """Befehl berechnen und senden. Aufruf mit rate Hz und direkt nach jedem Tastendruck."""
        lin, ang = self.keys.command(now)
        moving = lin != 0.0 or ang != 0.0
        if moving:
            self.stop_until = now + self._p('stop_hold')
        if moving or now < self.stop_until:
            msg = Twist()
            msg.linear.x = lin * self._p('max_linear')
            msg.angular.z = ang * self._p('max_angular')
            self.pub.publish(msg)
        self._show(lin, ang, now)

    def stop(self):
        self.keys.release()
        self.pub.publish(Twist())

    def _lock_text(self, now):
        if not self._p('lock_topic'):
            return ''
        if self.lock_time is None or now - self.lock_time > self._p('lock_timeout'):
            return '  | GESPERRT (remote_guard läuft nicht?)'
        if self.lock_data:
            return '  | GESPERRT (keine RustDesk-Sitzung)'
        return '  | frei'

    def _show(self, lin, ang, now):
        status = (f'Tempo {self.keys.scale * 100:3.0f} %  |  '
                  f'v {lin * self._p("max_linear"):+.3f} m/s  w {ang * self._p("max_angular"):+.2f} rad/s'
                  + self._lock_text(now))
        if status != self.status:
            sys.stdout.write('\r' + status + '\x1b[K')
            sys.stdout.flush()
            self.status = status


def run(node, fd):
    period = 1.0 / node._p('rate')
    next_tick = time.monotonic()
    while rclpy.ok():
        ready, _, _ = select.select([fd], [], [], max(0.0, min(next_tick - time.monotonic(), 0.05)))
        now = time.monotonic()
        if ready:
            data = os.read(fd, 64)
            if not data:                 # Terminal geschlossen
                return
            node.on_keys(data, now)
            next_tick = now              # sofort reagieren, v. a. auf Stopp
        if now >= next_tick:
            node.tick(now)
            next_tick = max(next_tick + period, now)
        rclpy.spin_once(node, timeout_sec=0.0)


def with_default_config(args):
    """config/keyboard_teleop.yaml vor die übrigen Argumente setzen.

    Steht vorn, damit eigene --params-file und -p vom Aufruf sie überschreiben.
    """
    args = list(sys.argv if args is None else args)
    try:
        cfg = os.path.join(get_package_share_directory('slop_teleop'), 'config', 'keyboard_teleop.yaml')
    except PackageNotFoundError:
        return args
    if not os.path.isfile(cfg):
        return args
    return args[:1] + ['--ros-args', '--params-file', cfg, '--'] + args[1:]


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_teleop braucht ein Terminal: ros2 run slop_teleop keyboard_teleop', file=sys.stderr)
        return 1
    # Kein rclpy-Signalhandler: Strg+C soll den Kontext nicht sofort beenden, damit noch ein Stopp rausgeht
    rclpy.init(args=with_default_config(args), signal_handler_options=SignalHandlerOptions.NO)
    node = None
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        node = KeyboardTeleopNode()
        print(HELP)
        tty.setcbreak(fd)                # Tasten einzeln, ohne Echo; Strg+C bleibt aktiv
        run(node, fd)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()
        if node is not None:
            try:
                node.stop()
            except Exception:
                pass
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
