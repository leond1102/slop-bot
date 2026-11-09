#!/usr/bin/env python3
"""
@file stepper_test.py
@brief Testskript für einen einzelnen Schrittmotor (z.B. Schieber an Gewindestange) am CL57E-4A über EtherCAT

Fährt N Schritte vor und wieder zurück, Open Loop. Kein Getriebe, keine Drehzahlbegrenzung.
Nutzt Bus/Rampen aus diff_motor.py (CSP-Modus, ein EtherCAT-Master). Kein ROS.

"Schritt" = eine Treiber-Einheit. Der Treiber hat DivSet Einheiten pro Motorumdrehung (aktuell meist 10000,
wird beim Start angezeigt). Bei einem 200-Schritt-Motor sind das 50 Einheiten pro Vollschritt.

Beispiele (Raw-Socket braucht root):
  sudo ~/ethercat_venv/bin/python3 stepper_test.py --steps 100
  sudo ~/ethercat_venv/bin/python3 stepper_test.py --steps 100 --speed 2000 --repeat 5
  sudo ~/ethercat_venv/bin/python3 stepper_test.py --steps 500 --no-back       # nur vor
  sudo ~/ethercat_venv/bin/python3 stepper_test.py --steps -100 --no-back      # nur zurück
"""
import argparse
import os
import sys
import time
from argparse import Namespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import diff_motor as dm   # noqa: E402


def move(bus, axis, steps, speed, acc):
    """Relativ um steps Einheiten fahren (Trapezrampe), warten bis Stillstand."""
    with bus.lock:
        axis.gen = dm.PosRamp(axis.gen.pos, axis.gen.pos + steps, speed, acc)
    t0 = time.monotonic()
    timeout = abs(steps) / speed + 2 * speed / acc + 3.0
    while not bus.all_done():
        bus.check()
        if time.monotonic() - t0 > timeout:
            raise dm.AbortTest('Timeout bei der Fahrt')
        time.sleep(0.005)
    time.sleep(0.2)
    bus.check()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--steps', type=int, default=100, help='Schritte (Treiber-Einheiten), negativ = andere Richtung (default 100)')
    p.add_argument('--axis', type=int, default=2, choices=[1, 2, 3, 4], help='Achse am Treiber (default 2)')
    p.add_argument('--dir', type=int, default=1, choices=[1, -1], help='Zählrichtung, -1 dreht alles um')
    p.add_argument('--speed', type=float, default=1000.0, help='Schritte pro Sekunde (default 1000, keine Obergrenze)')
    p.add_argument('--acc', type=float, default=20000.0, help='Beschleunigung in Schritten/s² (default 20000)')
    p.add_argument('--no-back', action='store_true', help='Nur in eine Richtung fahren, nicht zurück')
    p.add_argument('--repeat', type=int, default=1, help='Anzahl Hin-und-zurück-Fahrten (default 1)')
    p.add_argument('--pause', type=float, default=0.5, help='Pause zwischen den Fahrten in s (default 0.5)')
    p.add_argument('--ifname', default='enP8p1s0', help='Ethernet-Interface (default enP8p1s0)')
    p.add_argument('--units-per-rev', type=int, default=0, help='Einheiten pro Motorumdrehung (default: DivSet)')
    p.add_argument('--settle', type=float, default=1.0, help='Wartezeit nach Freigabe in s')
    p.add_argument('--ignore-params', action='store_true', help='Parameterprüfung (Open Loop) überspringen')
    p.add_argument('--yes', action='store_true', help='Ohne Rückfrage starten')
    a = p.parse_args()
    if a.steps == 0 or a.speed <= 0 or a.acc <= 0 or a.repeat < 1:
        p.error('--steps != 0, --speed > 0, --acc > 0, --repeat >= 1')

    if os.geteuid() != 0:
        print('Hinweis: Raw-Socket braucht root → sudo ~/ethercat_venv/bin/python3 ...')

    axis = dm.Axis('stepper', a.axis, a.dir, 1.0)
    args = Namespace(ifname=a.ifname, cycle_ms=2.0, dc=False, settle=a.settle, mode='stepper', csv=False,
                     csv_dir='', loop='open', motor_ma=0, acc_units={axis: a.acc})
    bus = dm.Bus(args, [axis])
    try:
        bus.open()
        bus.read_info()
        upr = a.units_per_rev or axis.params.get('DivSet', 0)
        axis.upr = upr if 200 <= upr <= 1_000_000 else 10000
        bus.ferr_limit[axis] = axis.upr    # Open Loop: Istwert folgt dem Soll, 1 Umdrehung reicht als Grenze
        problems = dm.check_params(axis, args)
        for prob in problems:
            print(f'WARNUNG: {prob}')
        if problems and not a.ignore_params:
            raise dm.AbortTest('Parameter erst korrigieren (oder --ignore-params).')

        print('\nPDO-Mapping:')
        bus.configure()
        bus.start()

        sign = 1 if a.steps > 0 else -1
        print(f'\n{abs(a.steps)} Schritte {"vor" if sign > 0 else "zurück"}'
              f'{"" if a.no_back else " und wieder zurück"}, {a.repeat}x, {a.speed:g} Schritte/s, '
              f'{a.acc:g} Schritte/s². {axis.upr} Einheiten = 1 Motorumdrehung '
              f'→ {a.steps / axis.upr:+.4f} Umdrehungen.')
        print('Open Loop: Verfahrweg des Schiebers frei? Endanschläge beachten.')
        if not a.yes:
            input('Enter = Start, Ctrl+C = Abbruch ... ')
        print('Freigabe ...')
        bus.enable_all()
        start = bus.snapshot()[axis][0x6064]

        for i in range(a.repeat):
            move(bus, axis, a.steps, a.speed, a.acc)
            pos = bus.snapshot()[axis][0x6064] - start
            print(f'[{i + 1}/{a.repeat}] vor:    Zähler {pos:+d}')
            if not a.no_back:
                time.sleep(a.pause)
                move(bus, axis, -a.steps, a.speed, a.acc)
                pos = bus.snapshot()[axis][0x6064] - start
                print(f'[{i + 1}/{a.repeat}] zurück: Zähler {pos:+d} (Soll 0)')
            if i + 1 < a.repeat:
                time.sleep(a.pause)
        print('Fertig. Zähler ist nur der Schrittzähler des Treibers, kein Messwert.')
        return 0
    except dm.AbortTest as e:
        print(f'\nABBRUCH: {e}')
        return 1
    except KeyboardInterrupt:
        print('\nAbbruch durch Nutzer → abbremsen')
        return 1
    finally:
        bus.shutdown()


if __name__ == '__main__':
    sys.exit(main())
