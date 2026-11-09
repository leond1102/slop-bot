#!/usr/bin/env python3
"""Greifer des Slop-Bots: zwei Spindel-Stepper (Open Loop) an Achse 1 und 2 des CL57E-4A.

Aufbau: zwei gleiche Spindeln übereinander, ihre Mitte liegt auf der X-Achse des Roboters, die Backen fahren in ±Y.
Die Achsen bedient der Busprozess ethercat_bus (gleicher EtherCAT-Master wie die Räder, eigene Freigabe); diese Node
rechnet Y-Koordinaten in Backenpositionen um (slop_base/gripper_geometry.py) und schickt Zielzähler über bus_link.

Services (alle blockieren, bis die Fahrt fertig ist):
  gripper/home    std_srvs/Trigger         beide Backen gegen den äußeren Anschlag, home_backoff zurück = Null
  gripper/grip    slop_interfaces/Grip     Objekt mit Mitte y [m] und Breite width [m] klemmen (+squeeze)
  gripper/move    slop_interfaces/Move     gegriffenes Objekt auf neue Mitte y [m] schieben (Breite bleibt)
  gripper/open    std_srvs/Trigger         beide Backen ganz auf (Null)
  gripper/stop    std_srvs/Trigger         laufende Fahrt mit Rampe anhalten
  gripper/enable  std_srvs/SetBool         false = Endstufen aus (Spindeln selbsthemmend), true = Freigabe
Topics:
  gripper/status        std_msgs/String         Klartext: nicht gehomt / offen / greift … / Fehler …
  gripper/joint_states  sensor_msgs/JointState  Backenwege [m] von der Null nach innen (gripper_left/right)
  /safety/stop          std_msgs/Bool (Sub)     true = anhalten, keine neuen Fahrten

Open Loop: Die Null merkt sich die Node in state_file (Zähler des Treibers). Nach Aus-/Einschalten des Treibers oder
Verschieben der Backen von Hand passt der Zähler nicht mehr → einmal gripper/home. Schritte, die an einem Anschlag
oder am Objekt verloren gehen, merkt niemand; darum greift die Node auf Maß (Breite − squeeze) statt bis zum Widerstand.

Start (mit Antrieb, Busprozess mit Greifer-Config):
  ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py      (gripper:=true ist Standard)
"""
import json
import os
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from slop_base.bus_link import DEFAULT_NAME, GripClient
from slop_base.cl57e_bus import Cl57eError
from slop_base.gripper_geometry import Geometry, GripperError
from slop_interfaces.srv import Grip, Move

STATE_TOL = 100     # Einheiten: so weit darf der Zähler seit dem letzten Lauf abweichen (0,04 mm)


class Abort(Exception):
    pass


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        for name, default in [
            ('axis_left', 1),          # linke Backe (von vorne auf den Roboter gesehen), obere Spindel
            ('axis_right', 2),         # rechte Backe, untere Spindel
            ('left_close_dir', -1),    # Zählrichtung beim Schließen (gemessen)
            ('right_close_dir', 1),
            ('left_is_plus_y', False),  # liegt die linke Backe auf der +Y-Seite des Roboters?
            ('lead', 0.004),           # m Spindelsteigung pro Umdrehung
            ('stroke', 0.182),         # m Schlittenweg Anschlag–Anschlag
            ('inner_margin', 0.002),   # m Abstand zum inneren Anschlag, den keine Fahrt unterschreitet
            ('open_width', 0.0),       # m Abstand der Backen-Innenflächen in der Null (nach home messen!), 0 = Greifen gesperrt
            ('squeeze', 0.0005),       # m Gesamtquetschung beim Greifen
            ('speed', 10000.0),        # Einheiten/s greifen, verschieben, öffnen
            ('acc', 20000.0),          # Einheiten/s²
            ('home_speed', 50000.0),
            ('home_acc', 100000.0),
            ('home_extra', 0.003),     # m über den Anschlag hinaus (rutscht durch)
            ('home_backoff', 0.002),   # m zurück vom Anschlag = Null
            ('state_file', '~/.ros/slop_gripper.json'),
            ('rate', 20.0),            # Hz Status, Keepalive
            ('bus_socket', DEFAULT_NAME),
            ('connect_timeout', 20.0),
            ('bus_timeout', 1.0),      # s ohne Status → Fahrt abbrechen
            # nur im Busprozess genutzt (liest dieselbe Datei), hier deklariert, damit ros2 param sie zeigt
            ('units_per_rev', 0), ('loop', 'open'), ('motor_current_ma', 0), ('max_ferr_rev', 1.0),
            ('grip_timeout', 1.0),
        ]:
            self.declare_parameter(name, default)
        p = self._p
        self.axes = {'links': p('axis_left'), 'rechts': p('axis_right')}
        self.close_dir = {p('axis_left'): p('left_close_dir'), p('axis_right'): p('right_close_dir')}
        if any(d not in (1, -1) for d in self.close_dir.values()):
            raise Cl57eError('left_close_dir/right_close_dir müssen ±1 sein')
        for name in ('lead', 'stroke', 'inner_margin', 'open_width', 'squeeze', 'home_extra', 'home_backoff'):
            if not 0 <= p(name) < 1.0:     # alles in Metern; 112 statt 0.112 ergäbe sonst Backenwege von 56 m
                raise Cl57eError(f'{name} = {p(name)}: in Metern angeben (z. B. 0.112 statt 112)')
        self.travel = p('stroke') - p('home_backoff') - p('inner_margin')
        self.geom = None
        if p('open_width') > 0:
            self.geom = Geometry(p('open_width'), self.travel, p('squeeze'), p('left_is_plus_y'))
        self.state_file = os.path.expanduser(p('state_file'))

        self.bus = GripClient(p('bus_socket'))
        self.get_logger().info('Verbinde mit dem Busprozess ethercat_bus …')
        info = self.bus.connect(p('connect_timeout'))
        self.upm = {}
        for name, axis in self.axes.items():
            upr = info.get('grip_upr', {}).get(str(axis))
            if not upr:
                raise Cl57eError(f'Busprozess bedient Greifer-Achse {axis} ({name}) nicht – gripper.yaml prüfen')
            self.upm[axis] = int(upr) / p('lead')        # Einheiten pro Meter Backenweg
        self._wait_status()

        self.zero = None           # {Achse: Zähler in der Null}
        self.gripped = None        # (y, width) solange ein Objekt gehalten wird
        self.estop = False
        self.abort = threading.Event()
        self.last_status = ''
        self._load_state()

        self.status_pub = self.create_publisher(String, 'gripper/status', 10)
        self.joint_pub = self.create_publisher(JointState, 'gripper/joint_states', 10)
        cmd = MutuallyExclusiveCallbackGroup()       # immer nur eine Fahrt
        fast = MutuallyExclusiveCallbackGroup()      # stop/Notaus laufen neben einer Fahrt
        self.create_service(Trigger, 'gripper/home', self.srv_home, callback_group=cmd)
        self.create_service(Grip, 'gripper/grip', self.srv_grip, callback_group=cmd)
        self.create_service(Move, 'gripper/move', self.srv_move, callback_group=cmd)
        self.create_service(Trigger, 'gripper/open', self.srv_open, callback_group=cmd)
        self.create_service(SetBool, 'gripper/enable', self.srv_enable, callback_group=cmd)
        self.create_service(Trigger, 'gripper/stop', self.srv_stop, callback_group=fast)
        self.create_subscription(Bool, '/safety/stop', self.on_estop, 10, callback_group=fast)
        self.create_timer(1.0 / p('rate'), self.tick, callback_group=MutuallyExclusiveCallbackGroup())

        ow = p('open_width')
        self.get_logger().info(
            f'Greifer: links Achse {self.axes["links"]}, rechts Achse {self.axes["rechts"]}, '
            f'{self.upm[self.axes["links"]] / 1000:g} Einh./mm, Weg {self.travel * 1000:.0f} mm, '
            + (f'offen {ow * 1000:.1f} mm breit. ' if ow > 0 else 'open_width fehlt (nach home messen) → nur home/open. ')
            + ('Null aus dem letzten Lauf übernommen.' if self.zero else 'Noch nicht gehomt → gripper/home.'))

    def _p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ Bus
    def _wait_status(self, timeout=3.0):
        t_end = time.monotonic() + timeout
        while self.bus.status() is None:
            if time.monotonic() > t_end:
                raise Cl57eError('Kein Status vom Busprozess')
            self.bus.poll(0.05)

    def _status(self):
        st = self.bus.status()
        if st is None or self.bus.status_age() > self._p('bus_timeout'):
            raise Abort('kein Status vom Busprozess')
        if self.bus.exit_reason:
            raise Abort(f'Busprozess beendet: {self.bus.exit_reason}')
        return st

    def tick(self):
        self.bus.poll(0.0)
        self.bus.keepalive()
        st = self.bus.status()
        if st is None:
            return
        text = self._status_text(st)
        if text != self.last_status:
            self.last_status = text
            (self.get_logger().warn if text.startswith('Fehler') else self.get_logger().info)(f'Greifer: {text}')
        self.status_pub.publish(String(data=text))
        if self.zero:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['gripper_left', 'gripper_right']
            js.position = [self._pos_m(self.axes[n], st['actual'][self.axes[n]]) for n in ('links', 'rechts')]
            self.joint_pub.publish(js)

    def _status_text(self, st):
        if self.bus.exit_reason:
            return f'Fehler: Busprozess beendet ({self.bus.exit_reason})'
        if st.get('error'):
            return f'Fehler: {st["error"]}'
        if self.estop:
            return 'Notaus aktiv'
        if st.get('moving'):
            return 'fährt'
        if not self.zero:
            return 'nicht gehomt'
        if self.gripped:
            return f'greift y={self.gripped[0]:+.3f} m, Breite {self.gripped[1]:.3f} m'
        pl, pr = (self._pos_m(self.axes[n], st['actual'][self.axes[n]]) for n in ('links', 'rechts'))
        return 'offen' if max(pl, pr) < 0.0005 else f'steht (links {pl * 1000:.1f} mm, rechts {pr * 1000:.1f} mm)'

    def _pos_m(self, axis, counter):
        return self.close_dir[axis] * (counter - self.zero[axis]) / self.upm[axis]

    def _counter(self, axis, pos_m):
        return self.zero[axis] + self.close_dir[axis] * pos_m * self.upm[axis]

    def _ensure_enabled(self):
        st = self._status()
        if st.get('enabled') and not st.get('error'):
            return
        self.get_logger().info('Greifer freigeben …')
        self.bus.enable()

    def _drive(self, goals, speed, acc, sync=True):
        """Achsen auf Zielzähler fahren und warten. sync: beide kommen gleichzeitig an (Geschwindigkeit nach Weg)."""
        if self.estop:
            raise Abort('Notaus aktiv')
        self._ensure_enabled()
        self.abort.clear()
        st = self._status()
        start = {a: st['target'][a] for a in goals}
        dist = {a: abs(goals[a] - start[a]) for a in goals}
        d_max = max(dist.values())
        if d_max < 1:
            return
        for a, g in goals.items():
            k = dist[a] / d_max if sync else 1.0
            if k > 0:
                self.bus.move({a: g}, speed * k, acc * k)
        goal_int = {a: int(round(g)) for a, g in goals.items()}
        t_end = time.monotonic() + d_max / speed + 2.0 * speed / acc + 3.0
        while True:
            self.bus.poll(0.02)
            if self.bus.rejected:
                raise Abort(f'Busprozess lehnt die Fahrt ab: {self.bus.rejected}')
            if self.abort.is_set():
                self.bus.stop()
                raise Abort('angehalten')
            st = self._status()
            if st.get('error'):
                raise Abort(st['error'])
            if all(st['target'][a] == goal_int[a] for a in goals) and not st.get('moving'):
                break
            if time.monotonic() > t_end:
                self.bus.stop()
                raise Abort('Zeitüberschreitung bei der Fahrt')
        time.sleep(0.1)
        self.bus.poll(0.02)
        self._save_state()

    def _move_jaws(self, p_left, p_right, sync=True):
        goals = {self.axes['links']: self._counter(self.axes['links'], p_left),
                 self.axes['rechts']: self._counter(self.axes['rechts'], p_right)}
        self._drive(goals, self._p('speed'), self._p('acc'), sync)

    def _jaw_positions(self):
        st = self._status()
        return tuple(self._pos_m(self.axes[n], st['target'][self.axes[n]]) for n in ('links', 'rechts'))

    # ------------------------------------------------------------------ Null merken
    def _key(self):
        return [self.axes['links'], self.axes['rechts'], self.close_dir[self.axes['links']],
                self.close_dir[self.axes['rechts']], {str(a): u for a, u in self.upm.items()}]

    def _load_state(self):
        try:
            with open(self.state_file, encoding='utf-8') as f:
                data = json.load(f)
        except (OSError, ValueError):
            return
        st = self.bus.status()
        if data.get('key') != self._key() or st is None:
            self.get_logger().warn('Gespeicherte Greifer-Null passt nicht zur Konfiguration → gripper/home nötig')
            return
        for a in self.axes.values():
            last = data.get('last', {}).get(str(a))
            if last is None or abs(st['actual'][a] - last) > STATE_TOL:
                self.get_logger().warn(f'Zähler Achse {a} passt nicht zum letzten Lauf (jetzt {st["actual"][a]}, '
                                       f'damals {last}) – Treiber neu gestartet? → gripper/home nötig')
                return
        self.zero = {a: float(data['zero'][str(a)]) for a in self.axes.values()}
        g = data.get('gripped')
        self.gripped = tuple(g) if g else None

    def _save_state(self):
        if not self.zero:
            return
        st = self.bus.status()
        if st is None:
            return
        data = {'key': self._key(), 'zero': {str(a): z for a, z in self.zero.items()},
                'last': {str(a): st['actual'][a] for a in self.axes.values()},
                'gripped': list(self.gripped) if self.gripped else None}
        try:
            os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
            tmp = self.state_file + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(data, f)
            os.replace(tmp, self.state_file)
        except OSError as e:
            self.get_logger().warn(f'Greifer-Null nicht gespeichert ({e})')

    # ------------------------------------------------------------------ Services
    def _run(self, response, what, fn):
        try:
            response.message = fn() or 'ok'
            response.success = True
        except (Abort, GripperError, Cl57eError) as e:
            response.success = False
            response.message = f'{what}: {e}'
            self.get_logger().warn(response.message)
        return response

    def srv_home(self, _req, res):
        def run():
            p = self._p
            st = self._status()
            out = (p('stroke') + p('home_extra'))
            goals = {a: st['target'][a] - self.close_dir[a] * out * self.upm[a] for a in self.axes.values()}
            self.zero = None
            self.gripped = None
            self._drive(goals, p('home_speed'), p('home_acc'), sync=False)
            st = self._status()
            goals = {a: st['target'][a] + self.close_dir[a] * p('home_backoff') * self.upm[a] for a in self.axes.values()}
            self._drive(goals, p('speed'), p('acc'), sync=False)
            st = self._status()
            self.zero = {a: float(st['target'][a]) for a in self.axes.values()}
            self._save_state()
            return 'gehomt (Null = außen, vom Anschlag gelöst)'
        return self._run(res, 'home', run)

    def _need_geometry(self):
        if not self.zero:
            raise Abort('nicht gehomt → erst gripper/home')
        if self.geom is None:
            raise Abort('open_width in gripper.yaml fehlt (Abstand der Backen-Innenflächen nach home messen)')

    def srv_grip(self, req, res):
        def run():
            self._need_geometry()
            pl, pr = self.geom.jaws_from_object(req.y, req.width)
            self.gripped = None
            self._move_jaws(pl, pr)
            self.gripped = (req.y, req.width)
            self._save_state()
            return f'gegriffen: y={req.y:+.3f} m, Breite {req.width:.3f} m (links {pl * 1000:.1f}, rechts {pr * 1000:.1f} mm)'
        return self._run(res, 'grip', run)

    def srv_move(self, req, res):
        def run():
            self._need_geometry()
            if not self.gripped:
                raise Abort('es wird nichts gehalten → erst gripper/grip')
            pl, pr = self._jaw_positions()
            nl, nr = self.geom.shift(pl, pr, req.y - self.gripped[0])
            self._move_jaws(nl, nr)
            self.gripped = (req.y, self.gripped[1])
            self._save_state()
            return f'verschoben auf y={req.y:+.3f} m'
        return self._run(res, 'move', run)

    def srv_open(self, _req, res):
        def run():
            if not self.zero:
                raise Abort('nicht gehomt → erst gripper/home')
            self.gripped = None
            self._move_jaws(0.0, 0.0)
            return 'offen'
        return self._run(res, 'open', run)

    def srv_enable(self, req, res):
        def run():
            if req.data:
                self.bus.enable()
                return 'freigegeben'
            self.bus.stop()
            self.bus.disable()
            return 'Endstufen aus'
        return self._run(res, 'enable', run)

    def srv_stop(self, _req, res):
        self.abort.set()
        self.bus.stop()
        res.success, res.message = True, 'angehalten'
        return res

    def on_estop(self, msg):
        if msg.data and not self.estop:
            self.abort.set()
            self.bus.stop()
            self.get_logger().warn('Notaus → Greifer hält an')
        self.estop = bool(msg.data)

    def shutdown(self):
        self.abort.set()
        try:
            self.bus.stop()
            self.bus.poll(0.1)
            self._save_state()
        finally:
            self.bus.close()


def main():
    rclpy.init()
    node = None
    try:
        node = GripperNode()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Cl57eError as e:
        print(f'[gripper_node] FEHLER: {e}', flush=True)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
