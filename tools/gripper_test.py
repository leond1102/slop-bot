#!/usr/bin/env python3
"""
@file gripper_test.py
@brief Testskript für den Greifer: zwei Spindel-Stepper (Open Loop) am CL57E-4A über EtherCAT

Aufbau (von vorne gesehen): zwei gleiche Spindeln waagerecht übereinander.
  Achse 1 (oben)  = linke Backe,  Achse 2 (unten) = rechte Backe.
Zählrichtung (gemessen): Stepper 2 (rechts) schließt in positive Richtung, Stepper 1 (links) in negative.
Position je Backe in mm von ihrem AUSSEREN Anschlag nach INNEN gezählt (0 = ganz offen, 182 = ganz innen).
Greifen = beide Backen nach innen, Verschieben = beide Backen gleich weit in dieselbe Richtung.

Modi:
  jog    Backen relativ bewegen (--left / --right in mm, + = nach innen). Zum Prüfen der Richtungen. Kein Homen nötig.
  home   Beide Backen gegen den äußeren Anschlag fahren, 2 mm zurück = Null. Nur EINMAL nach dem Einschalten des Treibers.
  close  Backen auf die Greifpositionen fahren (--grip-left / --grip-right, werden gemerkt), mit Greifgeschwindigkeit.
  open   Backen ganz auf (0 mm), mit Greifgeschwindigkeit.
  demo   Greifen, Verschieben nach rechts und zurück, Loslassen (homet nur, wenn noch kein Null bekannt ist).

Die Null merkt sich das Skript in gripper_state.json neben dem Skript. Beim nächsten Start prüft es, ob der Zähler des
Treibers noch zum letzten Lauf passt. Nach Aus-/Einschalten des Treibers, oder wenn die Backen von Hand bewegt wurden,
stimmt das nicht mehr → dann einmal home.

Open Loop: der Zähler ist nur der Schrittzähler des Treibers. Schritte, die am Anschlag oder am Bauteil verloren gehen,
merkt das Skript nicht. Darum die Greifpositionen als Maß vorgeben (Bauteil ohne Spiel gerade geklemmt, ggf. 0,5 mm
Quetschung), nicht gegen das Bauteil "durchdrehen" lassen.

Ablauf beim ersten Test:
  1. jog --left 5   → Backe links muss nach INNEN (nach rechts) fahren, sonst --dir-left -1
  2. jog --right 5  → Backe rechts muss nach INNEN (nach links) fahren, sonst --dir-right -1
  3. home, danach close / open / demo

Beispiele (Raw-Socket braucht root, ROS-Node ethercat_bus muss beendet sein):
  sudo ~/ethercat_venv/bin/python3 gripper_test.py jog --left 5 --right 5
  sudo ~/ethercat_venv/bin/python3 gripper_test.py home
  sudo ~/ethercat_venv/bin/python3 gripper_test.py close --grip-left 62 --grip-right 61
  sudo ~/ethercat_venv/bin/python3 gripper_test.py open
  sudo ~/ethercat_venv/bin/python3 gripper_test.py close
  sudo ~/ethercat_venv/bin/python3 gripper_test.py demo --push 30
"""
import argparse
import json
import os
import sys
import time
from argparse import Namespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import diff_motor as dm   # noqa: E402

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gripper_state.json')
STATE_TOL = 100   # Einheiten (0,04 mm): so viel darf der Zähler seit dem letzten Lauf abweichen


class Gripper:
    def __init__(self, bus, left, right, a):
        self.bus = bus
        self.jaws = {'left': left, 'right': right}
        self.a = a
        self.upmm = left.upr / a.lead           # Einheiten pro mm
        self.sign = {left: a.dir_left, right: a.dir_right}   # Zählrichtung nach innen
        self.zero = {}                          # Achse → Position (Einheiten) bei 0 mm
        self.homed = False

    def goto(self, targets_units, speed, acc):
        """Alle Achsen gleichzeitig auf absolute Positionen (Einheiten) fahren."""
        t_max = 0.0
        with self.bus.lock:
            for ax, goal in targets_units.items():
                dist = abs(goal - ax.gen.pos)
                t_max = max(t_max, dist / speed + 2 * speed / acc)
                ax.gen = dm.PosRamp(ax.gen.pos, goal, speed, acc)
        t0 = time.monotonic()
        while not self.bus.all_done():
            self.bus.check()
            if time.monotonic() - t0 > t_max + 3.0:
                raise dm.AbortTest('Timeout bei der Fahrt')
            time.sleep(0.005)
        time.sleep(0.2)
        self.bus.check()

    def counters(self):
        return {ax: ax.inp.get(0x6064) for ax in self.sign}

    def load_state(self):
        """Gemerkte Null laden, wenn sie zum Zähler des Treibers passt. Gibt (ok, Grund, Zustand) zurück."""
        try:
            with open(STATE_FILE, encoding='utf-8') as f:
                st = json.load(f)
        except (OSError, ValueError):
            return False, 'noch keine Null gespeichert', {}
        a = self.a
        key = [a.axis_left, a.axis_right, a.dir_left, a.dir_right, self.upmm]
        if st.get('key') != key:
            return False, 'Achsen/Richtung/Steigung haben sich geändert', st
        cur = self.counters()
        for ax in self.sign:
            last = st.get('last', {}).get(str(ax.num))
            if last is None or abs(cur[ax] - last) > STATE_TOL:
                return False, (f'Zähler {ax.label} passt nicht zum letzten Lauf (jetzt {cur[ax]}, damals {last}) '
                               '– Treiber neu gestartet oder Backe von Hand bewegt?'), st
        self.zero = {ax: float(st['zero'][str(ax.num)]) for ax in self.sign}
        self.homed = True
        return True, '', st

    def save_state(self, grip=None, old=None):
        """Null, letzten Zähler und Greifpositionen merken (nach dem Lauf, wenn die Backen stehen)."""
        if not self.homed:
            return
        a = self.a
        cur = self.counters()
        if any(v is None for v in cur.values()):
            return
        st = {'key': [a.axis_left, a.axis_right, a.dir_left, a.dir_right, self.upmm],
              'zero': {str(ax.num): self.zero[ax] for ax in self.sign},
              'last': {str(ax.num): cur[ax] for ax in self.sign},
              'grip': grip or (old or {}).get('grip')}
        with open(STATE_FILE, 'w', encoding='utf-8') as f:
            json.dump(st, f)
        if 'SUDO_UID' in os.environ:
            os.chown(STATE_FILE, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))

    def pos_units(self, ax, mm):
        return self.zero[ax] + self.sign[ax] * mm * self.upmm

    def move_mm(self, left_mm, right_mm, speed, acc):
        """Backen auf absolute Positionen [mm von außen] fahren (nur nach dem Homen)."""
        t = {}
        for name, mm in (('left', left_mm), ('right', right_mm)):
            if mm is None:
                continue
            if not 0 <= mm <= self.a.travel:
                raise dm.AbortTest(f'Backe {name}: {mm:g} mm außerhalb 0..{self.a.travel:g} mm')
            ax = self.jaws[name]
            t[ax] = self.pos_units(ax, mm)
        self.goto(t, speed, acc)

    def home(self):
        """Beide Backen schnell nach außen, am Anschlag rutschen sie kurz durch, dann etwas zurück. Dort ist 0."""
        a = self.a
        dist = (a.travel + a.home_extra) * self.upmm
        print(f'Homen: beide Backen {a.travel + a.home_extra:g} mm nach außen mit {a.home_speed:g} Schritte/s ...')
        with self.bus.lock:
            base = {ax: ax.gen.pos for ax in self.sign}
        self.goto({ax: base[ax] - self.sign[ax] * dist for ax in self.sign}, a.home_speed, a.home_acc)
        if a.home_backoff > 0:   # vom Anschlag lösen, damit nichts klemmt; dort ist dann Null
            print(f'  Anschlag erreicht, {a.home_backoff:g} mm zurück ...')
            with self.bus.lock:
                stop = {ax: ax.gen.pos for ax in self.sign}
            self.goto({ax: stop[ax] + self.sign[ax] * a.home_backoff * self.upmm for ax in self.sign},
                      a.speed, a.acc)
        with self.bus.lock:
            self.zero = {ax: ax.gen.pos for ax in self.sign}
        self.homed = True
        print('  Null gesetzt (beide Backen außen, vom Anschlag gelöst).')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=['jog', 'home', 'close', 'open', 'demo'])
    p.add_argument('--left', type=float, default=0.0, help='jog: Backe links [mm], + = nach innen')
    p.add_argument('--right', type=float, default=0.0, help='jog: Backe rechts [mm], + = nach innen')
    p.add_argument('--grip-left', type=float, help='close/demo: Greifposition Backe links [mm von außen] (default: zuletzt benutzte)')
    p.add_argument('--grip-right', type=float, help='close/demo: Greifposition Backe rechts [mm von außen] (default: zuletzt benutzte)')
    p.add_argument('--push', type=float, default=30.0, help='demo: Verschiebeweg nach rechts in mm (default 30)')
    p.add_argument('--axis-left', type=int, default=1, choices=[1, 2, 3, 4], help='Achse linke Backe (oben, default 1)')
    p.add_argument('--axis-right', type=int, default=2, choices=[1, 2, 3, 4], help='Achse rechte Backe (unten, default 2)')
    p.add_argument('--dir-left', type=int, default=-1, choices=[1, -1], help='Zählrichtung links: -1 = Schließen zählt negativ (gemessen)')
    p.add_argument('--dir-right', type=int, default=1, choices=[1, -1], help='Zählrichtung rechts: +1 = Schließen zählt positiv (gemessen)')
    p.add_argument('--lead', type=float, default=4.0, help='Spindelsteigung in mm pro Umdrehung (default 4)')
    p.add_argument('--travel', type=float, default=182.0, help='Gesamtweg je Backe in mm (default 182)')
    p.add_argument('--home-extra', type=float, default=3.0, help='Mehrweg über den Anschlag beim Homen in mm (default 3)')
    p.add_argument('--home-backoff', type=float, default=2.0, help='Abstand vom Anschlag nach dem Homen in mm = Null (default 2)')
    p.add_argument('--speed', type=float, default=10000.0, help='Greifen/Verschieben in Schritten/s (default 10000)')
    p.add_argument('--acc', type=float, default=20000.0, help='Beschleunigung dazu in Schritten/s² (default 20000)')
    p.add_argument('--home-speed', type=float, default=50000.0, help='Homen in Schritten/s (default 50000)')
    p.add_argument('--home-acc', type=float, default=100000.0, help='Beschleunigung beim Homen (default 100000)')
    p.add_argument('--pause', type=float, default=1.0, help='demo: Pause zwischen den Schritten in s')
    p.add_argument('--ifname', default='enP8p1s0', help='Ethernet-Interface (default enP8p1s0)')
    p.add_argument('--units-per-rev', type=int, default=0, help='Einheiten pro Motorumdrehung (default: DivSet)')
    p.add_argument('--settle', type=float, default=1.0, help='Wartezeit nach Freigabe in s')
    p.add_argument('--ignore-params', action='store_true', help='Parameterprüfung (Open Loop) überspringen')
    p.add_argument('--rehome', action='store_true', help='demo: erst neu homen, auch wenn eine Null bekannt ist')
    p.add_argument('--yes', action='store_true', help='Ohne Rückfrage starten')
    a = p.parse_args()

    if a.axis_left == a.axis_right:
        p.error('--axis-left und --axis-right müssen verschieden sein')
    if min(a.speed, a.acc, a.home_speed, a.home_acc, a.lead, a.travel) <= 0:
        p.error('Geschwindigkeiten, Beschleunigungen, --lead und --travel müssen > 0 sein')
    if os.geteuid() != 0:
        print('Hinweis: Raw-Socket braucht root → sudo ~/ethercat_venv/bin/python3 ...')

    left = dm.Axis('links', a.axis_left, a.dir_left, 1.0)
    right = dm.Axis('rechts', a.axis_right, a.dir_right, 1.0)
    args = Namespace(ifname=a.ifname, cycle_ms=2.0, dc=False, settle=a.settle, mode='gripper', csv=False,
                     csv_dir='', loop='open', motor_ma=0, acc_units={left: a.acc, right: a.acc})
    bus = dm.Bus(args, [left, right])
    g = None
    grip = None
    old = {}
    try:
        bus.open()
        bus.read_info()
        for ax in (left, right):
            upr = a.units_per_rev or ax.params.get('DivSet', 0)
            ax.upr = upr if 200 <= upr <= 1_000_000 else 10000
            bus.ferr_limit[ax] = ax.upr
        problems = [prob for ax in (left, right) for prob in dm.check_params(ax, args)]
        for prob in problems:
            print(f'WARNUNG: {prob}')
        if problems and not a.ignore_params:
            raise dm.AbortTest('Parameter erst korrigieren (oder --ignore-params).')

        print('\nPDO-Mapping:')
        bus.configure()
        bus.start()
        g = Gripper(bus, left, right, a)
        print(f'\n{left.upr} Einheiten/Umdrehung, Steigung {a.lead:g} mm → {g.upmm:g} Schritte/mm')

        texts = {'jog': f'jog: Backe links {a.left:+g} mm, Backe rechts {a.right:+g} mm (+ = nach innen). '
                        'Kein Endanschlagsschutz!',
                 'home': 'home: beide Backen fahren nach außen gegen den Anschlag.',
                 'close': 'close: Backen auf die Greifpositionen schließen.',
                 'open': 'open: Backen ganz öffnen.',
                 'demo': f'demo: greifen, {a.push:g} mm nach rechts und zurück, loslassen.'}
        print(texts[a.mode])
        print('Open Loop: Verfahrweg frei? Bauteil bereit?')
        if not a.yes:
            input('Enter = Start, Ctrl+C = Abbruch ... ')
        print('Freigabe ...')
        bus.enable_all()

        ok, why, old = g.load_state()
        need_home = a.mode == 'home' or (a.mode == 'demo' and (a.rehome or not ok))
        if ok:
            print('Null aus dem letzten Lauf übernommen.')
        elif a.mode in ('close', 'open') or (a.mode == 'demo' and not need_home):
            raise dm.AbortTest(f'Keine gültige Null ({why}). Erst: gripper_test.py home')
        elif a.mode in ('demo',):
            print(f'Keine gültige Null ({why}) → homen.')

        def grip_pos():
            gl = a.grip_left if a.grip_left is not None else (old.get('grip') or [None, None])[0]
            gr = a.grip_right if a.grip_right is not None else (old.get('grip') or [None, None])[1]
            if gl is None or gr is None:
                raise dm.AbortTest('Greifposition unbekannt: --grip-left und --grip-right angeben')
            return float(gl), float(gr)

        if a.mode == 'jog':
            with bus.lock:
                tgt = {ax: ax.gen.pos + g.sign[ax] * mm * g.upmm for ax, mm in ((left, a.left), (right, a.right))}
            g.goto(tgt, a.speed, a.acc)
            snap = bus.snapshot()
            print(f'Zähler: links {snap[left][0x6064]:+d}, rechts {snap[right][0x6064]:+d}')
        elif a.mode == 'home':
            g.home()
        elif a.mode == 'open':
            g.move_mm(0, 0, a.speed, a.acc)
            print('Offen.')
        elif a.mode == 'close':
            gl, gr = grip_pos()
            g.move_mm(gl, gr, a.speed, a.acc)
            grip = [gl, gr]
            print(f'Geschlossen (links {gl:g} mm, rechts {gr:g} mm).')
        else:
            gl, gr = grip_pos()
            grip = [gl, gr]
            for name, pos, sign in (('links', gl, 1), ('rechts', gr, -1)):
                end = pos + sign * a.push
                if not 0 <= end <= a.travel:
                    raise dm.AbortTest(f'Verschieben um {a.push:g} mm läuft für Backe {name} auf {end:g} mm, '
                                       f'außerhalb 0..{a.travel:g}')
            if need_home:
                g.home()
            print('Greifen ...')
            g.move_mm(gl, gr, a.speed, a.acc)
            time.sleep(a.pause)
            print(f'Verschieben {a.push:g} mm nach rechts ...')
            g.move_mm(gl + a.push, gr - a.push, a.speed, a.acc)
            time.sleep(a.pause)
            print('Verschieben zurück ...')
            g.move_mm(gl, gr, a.speed, a.acc)
            time.sleep(a.pause)
            print('Loslassen (beide Backen ganz auf) ...')
            g.move_mm(0, 0, a.speed, a.acc)
            print('Fertig.')
        return 0
    except dm.AbortTest as e:
        print(f'\nABBRUCH: {e}')
        return 1
    except KeyboardInterrupt:
        print('\nAbbruch durch Nutzer → abbremsen')
        return 1
    finally:
        bus.shutdown()
        if g is not None:
            try:
                g.save_state(grip, old)
            except OSError as e:
                print(f'Hinweis: Zustand nicht gespeichert ({e})')


if __name__ == '__main__':
    sys.exit(main())
