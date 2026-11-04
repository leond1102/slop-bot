#!/usr/bin/env python3
"""
@file diff_motor.py
@brief Testskript für die zwei Antriebsmotoren (Differentialantrieb) am CL57E-4A über EtherCAT

Beide Achsen laufen über EINEN EtherCAT-Master im CSP-Modus (wie b_chain_drive). Das Skript
erzeugt die Sollposition selbst: eine Drehzahlvorgabe wird mit begrenzter Beschleunigung
zur Position aufintegriert. Kein ROS.

Modi:
  info     Parameter beider Achsen per SDO lesen (keine Bewegung)
  compare  ALLE Herstellerparameter der Achsen 1..4 nebeneinander, Unterschiede markiert (nur lesen)
  monitor  In OP gehen, Achsen NICHT freigeben, Positionen live anzeigen.
           Räder von Hand drehen → zeigt, welche Achse links/rechts ist und ob "vorwärts" positiv zählt.
  jog      Räder mit fester Raddrehzahl für --time Sekunden drehen
           (oder mit --revs eine feste Anzahl Radumdrehungen fahren → Getriebe prüfen)
  drive    v [m/s] und w [rad/s] vorgeben → Raddrehzahlen (braucht --wheel-diameter und --track)

Alle Drehzahlen gelten am RAD (hinter dem Getriebe), die Motorwerte werden mit ausgegeben.
Vorzeichen: positiv = Rad dreht vorwärts. Die Zählrichtung je Seite legen --left-dir/--right-dir fest.

Grenzen (max_wheel_rpm, wheel_acc, max_ferr_rev), Getriebe, Achsen und Zählrichtungen liest das Skript aus
src/slop_base/config/diff_drive.yaml; nur dort ändern. Optionen auf der Kommandozeile gelten für den einen Lauf.

Die Radmotoren sind Schrittmotoren OHNE Encoder (Open Loop, --loop open ist Standard).
Die Istposition vom Treiber ist dann nur der Schrittzähler, kein Messwert.

ERSTER TEST: Roboter aufbocken, Räder frei. Achse 1 (Kettenantrieb) darf nicht gleichzeitig
von einer anderen Node angesteuert werden (nur ein EtherCAT-Master pro Interface).

Beispiele (Interface enP8p1s0, Raw-Socket braucht root):
  sudo ~/ethercat_venv/bin/python3 diff_motor.py info
  sudo ~/ethercat_venv/bin/python3 diff_motor.py monitor
  sudo ~/ethercat_venv/bin/python3 diff_motor.py jog --only left --rpm 10 --time 3
  sudo ~/ethercat_venv/bin/python3 diff_motor.py jog --rpm 20 --time 5
  sudo ~/ethercat_venv/bin/python3 diff_motor.py jog --revs 1 --rpm 10
  sudo ~/ethercat_venv/bin/python3 diff_motor.py jog --rpm 10 --time 10 --crash-after 3   # Absturztest
  sudo ~/ethercat_venv/bin/python3 diff_motor.py drive --v 0.05 --w 0 --wheel-diameter 0.07 --track 0.24 --time 3
"""
import argparse
import csv
import math
import os
import re
import struct
import sys
import threading
import time
from datetime import datetime

try:
    import pysoem
except ImportError:
    print('pysoem fehlt. Start mit der venv: sudo ~/ethercat_venv/bin/python3 ' + os.path.basename(__file__))
    sys.exit(1)

VENDOR_ID = 0x00000A79
PRODUCT_CODE = 0x6100          # CL57E-4A
AXIS_OFFSET = 0x800            # Objektindex-Abstand zwischen den Achsen
PDO_OFFSET = 0x10              # 0x1600, 0x1610, ... / 0x1A00, 0x1A10, ...
MODE_CSP = 8

# struct-Format der PDO-Objekte (Index von Achse 1)
PDO_TYPES = {
    0x6040: '<H', 0x607A: '<i', 0x60FF: '<i', 0x60B8: '<H',
    0x603F: '<H', 0x6041: '<H', 0x6064: '<i', 0x60FD: '<I', 0x606C: '<i',
    0x60F4: '<i', 0x60B9: '<H', 0x60BA: '<i', 0x60BC: '<i', 0x6061: '<b',
}

# Herstellerparameter (Index Achse 1), wie in a_system_test/cl57e_servo_test.py
VENDOR_PARAMS = [
    (0x2301, 'RunModeSet', '1=Open Loop, 2=Closed Loop'),
    (0x2302, 'DivSet',     'Einheiten pro Motorumdrehung'),
    (0x2303, 'MaxCurrent', 'max. Strom [mA]'),
    (0x230C, 'Encoder',    'Encoder Counts/U'),
    (0x230D, 'EnAlmPos',   'Positionsfehler-Alarm'),
    (0x2400, 'SensorErr',  'Sensorfehler melden 0/1'),
    (0x2300, 'DefaultDir', 'Drehrichtung 0/1'),
]

# Alle Herstellerparameter laut ESI (Index Achse 1, je UINT16), für den Modus compare
ALL_VENDOR_PARAMS = [
    (0x2100, 'InState'), (0x2101, 'OuState'), (0x2200, 'ParaCmd'),
    (0x2300, 'DefaultDir'), (0x2301, 'RunModeSet'), (0x2302, 'DivSet'), (0x2303, 'MaxCurrent'),
    (0x2304, 'IBasePer'), (0x2305, 'CloseIMaxPer'), (0x2306, 'OpenIMaxPer'), (0x2307, 'IdleIPer'),
    (0x2308, 'IdleTime'), (0x2309, 'InPosErr'), (0x230A, 'InPosTime'), (0x230B, 'FilterTime'),
    (0x230C, 'Encoder'), (0x230D, 'EnAlmPos'),
    (0x2400, 'SensorErr'), (0x2401, 'NodeRegSet'), (0x2402, 'LimitStop'), (0x2404, 'BkSetDelay'),
    (0x2405, 'BkOffDelay'),
    (0x2500, 'InNoNc'), (0x2501, 'OuNoNc'), (0x2510, 'I1Set'), (0x2511, 'I2Set'), (0x2512, 'I3Set'),
    (0x2520, 'O1Set'), (0x2521, 'O2Set'), (0x2530, 'DIFltTimes'), (0x2600, 'SyncCnt'),
    (0x2700, 'I_KpScale'), (0x2701, 'I_KpGain'), (0x2702, 'I_KiGain'), (0x2703, 'I_KcGain'),
    (0x2704, 'Spd_Kp1'), (0x2705, 'Spd_Kv1'), (0x2706, 'Spd_Kp2'), (0x2707, 'Spd_Kv2'),
    (0x2708, 'Spd_Ki'), (0x2709, 'Spd_Kvff'),
]

# Grenzen, Getriebe, Achsen und Zählrichtungen stehen nur in der Config der Node. Das Skript liest sie von dort,
# damit es keine zweite Kopie gibt; Kommandozeilenoptionen überschreiben sie nur für den einzelnen Lauf.
DRIVE_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'slop_base', 'config',
                          'diff_drive.yaml')


def load_drive_yaml(path):
    """Einfache Werte (Schlüssel: Zahl/Text) aus diff_drive.yaml. Reicht für die flache Parameterdatei,
    pyyaml gibt es in der EtherCAT-venv nicht. Fehlt die Datei, ist das Ergebnis leer."""
    cfg = {}
    try:
        with open(path, encoding='utf-8') as f:
            for line in f:
                m = re.match(r'^\s+([A-Za-z_]\w*):\s*([^\s#][^#]*?)\s*(#.*)?$', line)
                if not m:
                    continue
                raw = m.group(2)
                try:
                    cfg[m.group(1)] = int(raw)
                except ValueError:
                    try:
                        cfg[m.group(1)] = float(raw)
                    except ValueError:
                        cfg[m.group(1)] = raw
    except OSError:
        pass
    return cfg

CW_DISABLE_VOLTAGE = 0x0000
CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE = 0x000F
CW_FAULT_RESET = 0x0080


class AbortTest(Exception):
    pass


def drive_state(sw):
    """CiA-402-Zustand aus dem Statusword."""
    if sw & 0x4F == 0x00:
        return 'NOT_READY'
    if sw & 0x4F == 0x40:
        return 'SWITCH_ON_DISABLED'
    if sw & 0x6F == 0x21:
        return 'READY_TO_SWITCH_ON'
    if sw & 0x6F == 0x23:
        return 'SWITCHED_ON'
    if sw & 0x6F == 0x27:
        return 'OPERATION_ENABLED'
    if sw & 0x6F == 0x07:
        return 'QUICK_STOP_ACTIVE'
    if sw & 0x4F == 0x0F:
        return 'FAULT_REACTION_ACTIVE'
    if sw & 0x4F == 0x08:
        return 'FAULT'
    return 'UNKNOWN'


# ==================================================================== Sollwertgeneratoren
class VelRamp:
    """Drehzahlvorgabe: Geschwindigkeit folgt v_cmd mit begrenzter Beschleunigung, Position wird integriert."""

    def __init__(self, pos, amax):
        self.pos = float(pos)
        self.vel = 0.0
        self.v_cmd = 0.0
        self.amax = float(amax)

    def step(self, dt):
        dv = max(-self.amax * dt, min(self.amax * dt, self.v_cmd - self.vel))
        self.vel += dv
        self.pos += self.vel * dt
        return self.pos

    def done(self):
        return self.v_cmd == 0.0 and self.vel == 0.0


class PosRamp:
    """Trapezrampe auf eine Zielposition (wie Ramp in b_chain_drive)."""

    def __init__(self, pos, goal, vmax, amax):
        self.pos = float(pos)
        self.vel = 0.0
        self.goal = float(goal)
        self.vmax = float(vmax)
        self.amax = float(amax)

    def step(self, dt):
        d = self.goal - self.pos
        if d == 0 and self.vel == 0.0:
            return self.pos
        v_des = math.copysign(min(self.vmax, math.sqrt(2.0 * self.amax * abs(d))), d)
        self.vel += max(-self.amax * dt, min(self.amax * dt, v_des - self.vel))
        step = self.vel * dt
        if abs(d) <= abs(step):
            self.pos, self.vel = self.goal, 0.0
        else:
            self.pos += step
        return self.pos

    def done(self):
        return self.pos == self.goal and self.vel == 0.0


# ==================================================================== Achse / Bus
class Axis:
    """Zustand einer Achse am CL57E-4A. Felder ab cw werden unter Bus.lock geändert."""

    def __init__(self, side, num, direction, gear):
        self.side = side              # 'left' / 'right'
        self.num = num                # Achse 1..4 am Treiber
        self.dir = direction          # +1 / -1: Motor-Zählrichtung → Rad vorwärts
        self.gear = gear              # Übersetzung Motor : Rad
        self.ax = (num - 1) * AXIS_OFFSET
        self.rx = {}                  # Index (Achse 1) → (Byte-Offset, Format) im Ausgangsabbild
        self.tx = {}
        self.upr = 10000              # Einheiten pro Motorumdrehung (DivSet)
        self.params = {}

        self.cw = 0
        self.target = 0
        self.follow = True            # Soll = Ist, solange nicht freigegeben
        self.gen = None               # VelRamp / PosRamp
        self.inp = {}
        self.max_ferr = 0             # größter gemessener Schleppfehler seit Reset

    @property
    def label(self):
        return f'{self.side} (Achse {self.num})'


class Bus:
    """Ein EtherCAT-Master, ein CL57E-4A, mehrere Achsen im CSP-Modus."""

    def __init__(self, args, axes):
        self.args = args
        self.axes = axes
        self.cycle = args.cycle_ms / 1000.0
        self.master = pysoem.Master()
        self.slave = None
        self.rx_size = 0
        self.tx_size = 0

        self.lock = threading.Lock()
        self.ferr_limit = {}          # Achse → Einheiten
        self.error = ''
        self.inp_valid = False
        self.comm_lost = False
        self.thread_error = None
        self.stop_evt = threading.Event()
        self.thread = None
        self.op_reached = False
        self.log = None               # Liste, wenn CSV gewünscht (nur bei Fahrten)
        self.t0 = time.perf_counter()
        self.n_cycles = self.n_overruns = self.n_wkc_err = 0
        self.max_period = 0.0

    # ---------------------------------------------------------------- Verbindung / SDO
    def open(self):
        try:
            self.master.open(self.args.ifname)
        except Exception as e:
            raise AbortTest(f'Interface {self.args.ifname} lässt sich nicht öffnen ({e}).\n'
                            '  → Mit sudo starten und Interface-Namen prüfen (ip -br link).')
        n = self.master.config_init()
        if n <= 0:
            raise AbortTest('Kein EtherCAT-Slave gefunden. Kabel am IN-Port? Treiber versorgt?')
        print(f'{n} Slave(s) gefunden:')
        for i, s in enumerate(self.master.slaves):
            print(f'  [{i}] {s.name:16s} Vendor=0x{s.man:08X} Product=0x{s.id:08X} Rev=0x{s.rev:08X}')
        for s in self.master.slaves:
            if s.man == VENDOR_ID and s.id == PRODUCT_CODE:
                self.slave = s
                return
        raise AbortTest('Kein CL57E-4A (Vendor 0x0A79 / Product 0x6100) am Bus.')

    def sdo_r(self, index, sub, fmt):
        data = bytes(self.slave.sdo_read(index, sub))
        size = struct.calcsize(fmt)
        return struct.unpack(fmt, (data + b'\0' * size)[:size])[0]

    def sdo_w(self, index, sub, value, fmt):
        self.slave.sdo_write(index, sub, struct.pack(fmt, value))

    def read_info(self):
        """Herstellerparameter und CiA-402-Objekte aller Achsen lesen und ausgeben."""
        for a in self.axes:
            print(f'\n--- {a.label} ---')
            for idx, name, desc in VENDOR_PARAMS:
                try:
                    val = self.sdo_r(idx + a.ax, 0, '<H')
                    a.params[name] = val
                    print(f'  0x{idx + a.ax:04X} {name:11s} = {val:6d}   {desc}')
                except Exception as e:
                    print(f'  0x{idx + a.ax:04X} {name:11s} = Lesefehler ({e})')
            for idx, name, fmt in [(0x6041, 'Statusword', '<H'), (0x603F, 'Error Code', '<H'),
                                   (0x6061, 'Mode display', '<b'), (0x6064, 'Position', '<i'),
                                   (0x6502, 'Supp. modes', '<I')]:
                try:
                    val = self.sdo_r(idx + a.ax, 0, fmt)
                    extra = f'  → {drive_state(val)}' if idx == 0x6041 else ''
                    shown = f'0x{val:X}' if fmt in ('<H', '<I') else str(val)
                    print(f'  0x{idx + a.ax:04X} {name:11s} = {shown}{extra}')
                except Exception as e:
                    print(f'  0x{idx + a.ax:04X} {name:11s} = Lesefehler ({e})')

    def error_code(self, a):
        try:
            return f'0x{self.sdo_r(0x603F + a.ax, 0, "<H"):04X}'
        except Exception:
            return '?'

    # ---------------------------------------------------------------- PDO / Zustände
    def configure(self):
        """PDOs der gewählten Achsen zuordnen, Mapping lesen, CSP setzen (PRE-OP)."""
        rx_pdos = [0x1600 + (a.num - 1) * PDO_OFFSET for a in self.axes]
        tx_pdos = [0x1A00 + (a.num - 1) * PDO_OFFSET for a in self.axes]
        for assign, pdos in ((0x1C12, rx_pdos), (0x1C13, tx_pdos)):
            self.sdo_w(assign, 0, 0, '<B')
            for i, pdo in enumerate(pdos, 1):
                self.sdo_w(assign, i, pdo, '<H')
            self.sdo_w(assign, 0, len(pdos), '<B')
        self.rx_size = self._build_map(self._read_assign(0x1C12), 'rx')
        self.tx_size = self._build_map(self._read_assign(0x1C13), 'tx')
        for a in self.axes:
            print(f'  {a.label}: Rx {[hex(k + a.ax) for k in a.rx]}  Tx {[hex(k + a.ax) for k in a.tx]}')
            for need, where in ((0x6040, a.rx), (0x607A, a.rx), (0x6041, a.tx), (0x6064, a.tx)):
                if need not in where:
                    raise AbortTest(f'Objekt 0x{need + a.ax:04X} fehlt im PDO-Mapping von {a.label}')
            self.sdo_w(0x6060 + a.ax, 0, MODE_CSP, '<b')

    def _read_assign(self, assign):
        n = self.sdo_r(assign, 0, '<B')
        return [self.sdo_r(assign, i, '<H') for i in range(1, n + 1)]

    def _build_map(self, pdos, which):
        """Byte-Offsets aller Objekte im Prozessabbild, jeweils der passenden Achse zugeordnet."""
        bit = 0
        for pdo in pdos:
            for i in range(1, self.sdo_r(pdo, 0, '<B') + 1):
                entry = self.sdo_r(pdo, i, '<I')
                index = entry >> 16
                for a in self.axes:
                    key = index - a.ax
                    if key in PDO_TYPES and bit % 8 == 0:
                        getattr(a, which)[key] = (bit // 8, PDO_TYPES[key])
                bit += entry & 0xFF
        return (bit + 7) // 8

    def start(self):
        """SAFE-OP → Zyklus-Thread → OP."""
        if self.args.dc:
            self.master.config_dc()
            self.slave.dc_sync(True, int(self.cycle * 1e9))
        self.master.config_map()
        if len(self.slave.output) != self.rx_size or len(self.slave.input) != self.tx_size:
            raise AbortTest(f'Prozessabbild passt nicht: out {len(self.slave.output)}/{self.rx_size}, '
                            f'in {len(self.slave.input)}/{self.tx_size} Byte')
        if self.master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
            self._raise_al('SAFE-OP nicht erreicht')

        self.thread = threading.Thread(target=self._cyclic, daemon=True)
        self.thread.start()
        t_end = time.monotonic() + 1.0
        while not self.inp_valid and time.monotonic() < t_end:
            time.sleep(0.01)

        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        for _ in range(40):
            if self.master.state_check(pysoem.OP_STATE, 50000) == pysoem.OP_STATE:
                self.op_reached = True
                break
        if not self.op_reached:
            self._raise_al('OP nicht erreicht' + ('' if self.args.dc else ' (evtl. --dc nötig)'))
        for a in self.axes:
            mode = self.sdo_r(0x6061 + a.ax, 0, '<b')
            if mode != MODE_CSP:
                raise AbortTest(f'{a.label}: CSP nicht aktiv (0x6061 = {mode})')
        print('EtherCAT-Zustand: OP, CSP aktiv')

    def _raise_al(self, msg):
        self.master.read_state()
        code = self.slave.al_status
        raise AbortTest(f'{msg}: AL-Status 0x{code:04X} ({pysoem.al_status_code_to_string(code)})')

    # ---------------------------------------------------------------- Zyklus
    def _cyclic(self):
        out = bytearray(self.rx_size)
        loss_limit = max(10, int(0.1 / self.cycle))   # ~100 ms ohne gültige Antwort
        wkc_bad = 0
        nxt = last = time.perf_counter()
        try:
            while not self.stop_evt.is_set():
                with self.lock:
                    for a in self.axes:
                        if a.gen is not None:
                            a.target = int(round(a.gen.step(self.cycle)))
                        struct.pack_into('<H', out, a.rx[0x6040][0], a.cw)
                        struct.pack_into('<i', out, a.rx[0x607A][0], a.target)
                        if 0x60FF in a.rx:
                            struct.pack_into('<i', out, a.rx[0x60FF][0], 0)
                self.slave.output = bytes(out)

                self.master.send_processdata()
                wkc = self.master.receive_processdata(2000)
                now = time.perf_counter()

                if wkc >= self.master.expected_wkc:
                    wkc_bad = 0
                    data = self.slave.input
                    with self.lock:
                        for a in self.axes:
                            a.inp = {k: struct.unpack_from(f, data, o)[0] for k, (o, f) in a.tx.items()}
                        self.inp_valid = True
                        self._supervise()
                        if self.log is not None:
                            row = [round(now - self.t0, 4)]
                            for a in self.axes:
                                row += [a.cw, a.inp.get(0x6041), a.target, a.inp.get(0x6064), a.inp.get(0x606C)]
                            self.log.append(row)
                else:
                    wkc_bad += 1
                    self.n_wkc_err += 1
                    if wkc_bad >= loss_limit and self.op_reached:
                        self.comm_lost = True

                self.n_cycles += 1
                if self.n_cycles > 5:
                    self.max_period = max(self.max_period, now - last)
                last = now
                nxt += self.cycle
                sleep = nxt - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    self.n_overruns += 1
                    if sleep < -10 * self.cycle:
                        nxt = time.perf_counter()
        except Exception as e:
            self.thread_error = e
            self.comm_lost = True

    def _supervise(self):
        """Läuft unter self.lock. Fehler an EINER Achse schaltet BEIDE ab (sonst dreht der Roboter)."""
        for a in self.axes:
            actual = a.inp[0x6064]
            if a.follow:
                a.target = actual
                continue
            sw = a.inp[0x6041]
            st = drive_state(sw)
            ferr = a.target - actual
            a.max_ferr = max(a.max_ferr, abs(ferr))
            if st != 'OPERATION_ENABLED':
                self._drop_all(f'{a.label}: Treiber nicht mehr freigegeben ({st}, SW 0x{sw:04X})')
                return
            if abs(ferr) > self.ferr_limit[a]:
                self._drop_all(f'{a.label}: Schleppfehler {ferr} > {self.ferr_limit[a]} Einheiten')
                return

    def _drop_all(self, msg):
        for a in self.axes:
            a.follow = True
            a.gen = None
            a.target = a.inp.get(0x6064, a.target)
            a.cw = CW_DISABLE_VOLTAGE if drive_state(a.inp.get(0x6041, 0)) == 'FAULT' else CW_SHUTDOWN
        if not self.error:
            self.error = msg

    # ---------------------------------------------------------------- Bedienung
    def check(self):
        if self.comm_lost:
            raise AbortTest(f'EtherCAT-Kommunikation verloren ({self.thread_error or "keine Antwort"})')
        if self.error:
            codes = ', '.join(f'{a.side} Error {self.error_code(a)}' for a in self.axes)
            raise AbortTest(f'{self.error} ({codes}) → Endstufen aus')

    def snapshot(self):
        with self.lock:
            return {a: dict(a.inp) for a in self.axes}

    def enable_all(self, timeout=5.0):
        """Fault-Reset und Zustandsmaschine bis OPERATION_ENABLED für alle Achsen."""
        t_end = time.monotonic() + timeout
        last_toggle = 0.0
        while True:
            self.check()
            states = {}
            toggle = time.monotonic() - last_toggle > 0.1
            with self.lock:
                for a in self.axes:
                    sw = a.inp.get(0x6041, 0)
                    st = states[a] = drive_state(sw)
                    if st == 'FAULT':
                        if toggle:   # steigende Flanke an Bit 7
                            a.cw = CW_DISABLE_VOLTAGE if a.cw == CW_FAULT_RESET else CW_FAULT_RESET
                    elif st in ('SWITCH_ON_DISABLED', 'NOT_READY'):
                        a.cw = CW_SHUTDOWN
                    elif st == 'READY_TO_SWITCH_ON':
                        a.cw = CW_SWITCH_ON
                    elif st == 'SWITCHED_ON':
                        a.cw = CW_ENABLE
                    elif st == 'QUICK_STOP_ACTIVE':
                        a.cw = CW_DISABLE_VOLTAGE
            if toggle:
                last_toggle = time.monotonic()
            if all(st == 'OPERATION_ENABLED' for st in states.values()):
                break
            if time.monotonic() > t_end:
                bad = ', '.join(f'{a.label}: {st}' for a, st in states.items() if st != 'OPERATION_ENABLED')
                raise AbortTest(f'Freigabe fehlgeschlagen ({bad})')
            time.sleep(0.02)

        print(f'  OPERATION_ENABLED, warte {self.args.settle} s (Lock-Phase des Treibers)')
        time.sleep(self.args.settle)
        with self.lock:
            for a in self.axes:
                sw = a.inp.get(0x6041, 0)
                if drive_state(sw) != 'OPERATION_ENABLED':
                    raise AbortTest(f'{a.label} nach der Lock-Phase nicht mehr freigegeben (SW 0x{sw:04X})')
            for a in self.axes:   # ab hier hält jede Achse ihre Position
                a.target = a.inp[0x6064]
                a.gen = VelRamp(a.target, self.args.acc_units[a])
                a.follow = False
                a.max_ferr = 0

    def set_wheel_rpm(self, rpm_by_axis):
        """Raddrehzahl [U/min, + = vorwärts] je Achse vorgeben."""
        with self.lock:
            for a, rpm in rpm_by_axis.items():
                if isinstance(a.gen, VelRamp):
                    a.gen.v_cmd = a.dir * rpm / 60.0 * a.gear * a.upr

    def move_wheel_revs(self, revs, rpm):
        """Alle Räder um revs Radumdrehungen (+ = vorwärts) mit |rpm| fahren. Nur aus dem Stillstand."""
        with self.lock:
            for a in self.axes:
                dist = a.dir * revs * a.gear * a.upr
                vmax = abs(rpm) / 60.0 * a.gear * a.upr
                a.gen = PosRamp(a.gen.pos, a.gen.pos + dist, vmax, self.args.acc_units[a])

    def all_done(self):
        with self.lock:
            return all(a.gen is None or a.gen.done() for a in self.axes)

    def stop_all(self, timeout=5.0):
        """Mit Rampe auf 0 bremsen und warten. Gibt False zurück, wenn es zu lange dauert."""
        with self.lock:
            for a in self.axes:
                if a.gen is None or a.follow:
                    continue
                if isinstance(a.gen, PosRamp):   # Ziel auf das Ende des Bremswegs legen
                    brake = a.gen.vel * abs(a.gen.vel) / (2 * a.gen.amax)
                    a.gen.goal = float(round(a.gen.pos + brake))
                else:
                    a.gen.v_cmd = 0.0
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if self.comm_lost or self.error or self.all_done():
                return self.all_done()
            time.sleep(0.01)
        return False

    def shutdown(self):
        """Abbremsen, Endstufen aus, Thread stoppen, Bus schließen. Immer aufrufen."""
        if self.thread is not None and self.thread.is_alive():
            if not self.comm_lost and not self.error:
                self.stop_all(timeout=3.0)
            with self.lock:
                for a in self.axes:
                    a.gen = None
                    a.follow = True
                    a.cw = CW_SHUTDOWN
            time.sleep(0.1)
            with self.lock:
                for a in self.axes:
                    a.cw = CW_DISABLE_VOLTAGE
            time.sleep(0.05)
            self.stop_evt.set()
            self.thread.join(timeout=1.0)
        try:
            self.master.state = pysoem.INIT_STATE
            self.master.write_state()
            self.master.close()
        except Exception:
            pass

    def write_csv(self):
        if not self.log:
            return
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(self.args.csv_dir, exist_ok=True)
        path = os.path.join(self.args.csv_dir, f'diff_motor_{self.args.mode}_{ts}.csv')
        header = ['t_s']
        for a in self.axes:
            header += [f'{a.side}_{n}' for n in ('cw', 'sw', 'target', 'actual', 'velocity')]
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.log)
        if 'SUDO_UID' in os.environ:   # Datei nicht root gehören lassen
            os.chown(path, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))
        print(f'CSV: {path} ({len(self.log)} Zeilen)')


# ==================================================================== Testabläufe
def wheel_deg(a, units, home):
    """Motoreinheiten → Radwinkel [Grad], + = vorwärts."""
    return a.dir * (units - home) / a.upr / a.gear * 360.0


def run_monitor(bus, args):
    print('\nMonitor (Achsen NICHT freigegeben). Räder von Hand drehen. Ctrl+C beendet.')
    print('Vorwärts drehen muss einen POSITIVEN Radwinkel ergeben, sonst --left-dir/--right-dir umdrehen.\n')
    home = {a: inp.get(0x6064, 0) for a, inp in bus.snapshot().items()}
    while True:
        bus.check()
        parts = []
        for a, inp in bus.snapshot().items():
            sw = inp.get(0x6041, 0)
            parts.append(f'{a.side}(A{a.num}) {drive_state(sw)[:14]:14s} SW=0x{sw:04X} '
                         f'{wheel_deg(a, inp.get(0x6064, 0), home[a]):+9.2f}° Rad')
        print('\r' + ' | '.join(parts) + '  ', end='', flush=True)
        time.sleep(0.1)


def confirm(args, text):
    print(text)
    if not args.yes:
        input('Räder frei / Roboter aufgebockt? Enter = Start, Ctrl+C = Abbruch ... ')


def run_jog_time(bus, args, rpm_by_axis):
    """Drehzahl vorgeben, --time Sekunden halten, abbremsen, auswerten."""
    for a, rpm in rpm_by_axis.items():
        print(f'  {a.label:18s} Rad {rpm:+7.2f} U/min  = Motor {rpm * a.gear:+8.1f} U/min '
              f'({rpm * 6:+.1f} °/s am Rad)')
    t_acc = max(abs(rpm) / 60.0 / args.acc for rpm in rpm_by_axis.values())
    if args.time < t_acc + 0.5:
        print(f'WARNUNG: --time {args.time} s ist kurz, Beschleunigen dauert {t_acc:.2f} s')
    confirm(args, f'Fahrt: {args.time} s, Beschleunigung {args.acc} U/s² am Rad')

    print('Freigabe ...')
    bus.enable_all()
    start = {a: inp[0x6064] for a, inp in bus.snapshot().items()}

    bus.set_wheel_rpm(rpm_by_axis)
    t0 = time.monotonic()
    meas_start = None
    while time.monotonic() - t0 < args.time:
        bus.check()
        el = time.monotonic() - t0
        if args.crash_after is not None and el >= args.crash_after:
            # Absturz simulieren: Prozess sofort weg, kein Abbremsen, kein Abschalten, keine Frames mehr
            print(f'\n\nABSTURZ-SIMULATION nach {el:.1f} s: Prozess endet hart. Die Räder müssen sofort stehen.\n'
                  'Danach Treiber-LEDs ansehen und "info" laufen lassen (Statusword/Error Code).', flush=True)
            os._exit(3)
        if meas_start is None and el >= t_acc + 0.2:
            meas_start = (time.monotonic(), {a: inp[0x6064] for a, inp in bus.snapshot().items()})
        snap = bus.snapshot()
        print('\r  t={:5.2f}s  '.format(el) + '  '.join(
            f'{a.side} {wheel_deg(a, snap[a][0x6064], start[a]):+9.1f}° Schleppf. {a.max_ferr:5d}'
            for a in bus.axes) + '  ', end='', flush=True)
        time.sleep(0.05)
    meas_end = (time.monotonic(), {a: inp[0x6064] for a, inp in bus.snapshot().items()})
    print()
    print('Abbremsen ...')
    if not bus.stop_all():
        bus.check()
        raise AbortTest('Achsen kommen nicht zum Stillstand')
    time.sleep(0.3)
    bus.check()
    end = {a: inp[0x6064] for a, inp in bus.snapshot().items()}

    print('\nErgebnis:')
    for a, rpm in rpm_by_axis.items():
        line = f'  {a.label:18s} Soll {rpm:+7.2f} U/min Rad'
        if meas_start is not None:
            dt = meas_end[0] - meas_start[0]
            ist = a.dir * (meas_end[1][a] - meas_start[1][a]) / a.upr / a.gear / dt * 60.0
            line += f'  Ist {ist:+7.2f} U/min (konstante Phase {dt:.1f} s)'
        revs = wheel_deg(a, end[a], start[a]) / 360.0
        line += f'  gesamt {revs:+.3f} Radumdr.'
        if args.wheel_diameter:
            line += f' = {revs * math.pi * args.wheel_diameter:+.3f} m'
        line += f'  max|Schleppfehler| {a.max_ferr} Einh. ({a.max_ferr / a.upr * 360:.1f}° Motor)'
        print(line)
    print_open_loop_note(args)


def print_open_loop_note(args):
    if args.loop == 'open':
        print('Hinweis Open Loop: "Ist" zählt nur die ausgegebenen Schritte, Schrittverluste sieht man hier nicht.\n'
              'Prüfen durch Hinsehen: Läuft der Motor ruhig, ohne Rattern/Pfeifen und ohne stehen zu bleiben?')


def run_jog_revs(bus, args):
    """Feste Anzahl Radumdrehungen fahren → Rad markieren und Getriebe/Richtung prüfen."""
    rpm = abs(args.rpm)
    confirm(args, f'Fahrt: {args.revs:+g} Radumdrehungen mit {rpm:g} U/min Rad '
                  f'({", ".join(f"{a.side} {args.revs * a.gear:+g}" for a in bus.axes)} Motorumdrehungen), '
                  'danach Stillstand.\n'
                  'Tipp: Rad vorher mit Klebeband markieren.')
    print('Freigabe ...')
    bus.enable_all()
    start = {a: inp[0x6064] for a, inp in bus.snapshot().items()}
    bus.move_wheel_revs(args.revs, rpm)
    t0 = time.monotonic()
    timeout = abs(args.revs) / (rpm / 60.0) + rpm / 60.0 / args.acc * 2 + 3.0
    while not bus.all_done():
        bus.check()
        if time.monotonic() - t0 > timeout:
            raise AbortTest('Timeout bei der Fahrt')
        time.sleep(0.02)
    time.sleep(0.3)
    bus.check()
    snap = bus.snapshot()
    print('\nErgebnis:')
    for a in bus.axes:
        cmd = round(a.dir * args.revs * a.gear * a.upr)
        real = snap[a][0x6064] - start[a]
        ratio = real / cmd if cmd else float('nan')
        print(f'  {a.label:18s} Soll {cmd:+9d}  Ist {real:+9d} Einheiten  Verhältnis {ratio:.4f}  '
              f'= {wheel_deg(a, snap[a][0x6064], start[a]) / 360:+.3f} Radumdr.  '
              f'max|Schleppfehler| {a.max_ferr}')
    print(f'Stimmt die Markierung nach {args.revs:g} Umdrehung(en) wieder? Sonst --gear prüfen '
          f'(aktuell {", ".join(f"Achse {a.num} 1:{a.gear:g}" for a in bus.axes)}).')
    print_open_loop_note(args)


def run_compare(bus, args):
    """Alle Herstellerparameter der Achsen 1..4 lesen und nebeneinander zeigen (nur lesen)."""
    nums = (1, 2, 3, 4)
    rows = []
    for idx, name in ALL_VENDOR_PARAMS:
        vals = []
        for n in nums:
            try:
                vals.append(bus.sdo_r(idx + (n - 1) * AXIS_OFFSET, 0, '<H'))
            except Exception:
                vals.append(None)
        rows.append((idx, name, vals))

    pair = (args.left_axis, args.right_axis)
    print(f'\nHerstellerparameter aller Achsen (* = Achse {pair[0]} und {pair[1]} verschieden)')
    print(f'  {"Index":7s}{"Name":13s}' + ''.join(f'{"Achse " + str(n):>10s}' for n in nums))
    n_diff = 0
    for idx, name, vals in rows:
        a, b = vals[pair[0] - 1], vals[pair[1] - 1]
        mark = '*' if a != b else ' '
        n_diff += a != b
        cells = ''.join(f'{"-" if v is None else v:>10}' for v in vals)
        print(f'{mark} 0x{idx:04X} {name:13s}{cells}')
    print(f'\n{n_diff} Unterschied(e) zwischen Achse {pair[0]} und {pair[1]}. '
          '(InState/OuState sind Live-Zustände, Unterschiede dort sind normal.)')

    if args.csv:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(args.csv_dir, exist_ok=True)
        path = os.path.join(args.csv_dir, f'diff_motor_compare_{ts}.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['index_achse1', 'name'] + [f'achse{n}' for n in nums])
            for idx, name, vals in rows:
                w.writerow([f'0x{idx:04X}', name] + ['' if v is None else v for v in vals])
        if 'SUDO_UID' in os.environ:
            os.chown(path, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))
        print(f'CSV: {path}')


# ==================================================================== main
def check_params(a, args):
    problems = []
    p = a.params
    if args.loop == 'open':
        # Closed Loop ohne Encoder würde sofort einen Encoder-/Positionsalarm auslösen
        if p.get('RunModeSet') not in (None, 1):
            problems.append(f'{a.label}: RunModeSet = {p["RunModeSet"]} (erwartet 1 = Open Loop, kein Encoder)')
    else:
        if p.get('RunModeSet') not in (None, 2):
            problems.append(f'{a.label}: RunModeSet = {p["RunModeSet"]} (erwartet 2 = Closed Loop)')
        if p.get('SensorErr') == 0:
            problems.append(f'{a.label}: SensorErr = 0 (Encoderfehler werden ignoriert)')
    cur = p.get('MaxCurrent')
    if args.motor_ma and cur is not None and cur > args.motor_ma:
        problems.append(f'{a.label}: MaxCurrent = {cur} mA > Motornennstrom {args.motor_ma} mA (Überhitzung!)')
    return problems


def main():
    cfg = load_drive_yaml(DRIVE_YAML)
    cfg_acc = cfg['wheel_acc'] / (2 * math.pi) if 'wheel_acc' in cfg else None   # rad/s² → U/s²
    p = argparse.ArgumentParser(description='Test der Antriebsmotoren (Differentialantrieb) am CL57E-4A')
    p.add_argument('mode', choices=['info', 'compare', 'monitor', 'jog', 'drive'])
    p.add_argument('--ifname', default='enP8p1s0', help='Ethernet-Interface (default enP8p1s0)')
    p.add_argument('--left-axis', type=int, default=int(cfg.get('left_axis', 3)), choices=[1, 2, 3, 4], help='Achse linkes Rad (default 3)')
    p.add_argument('--right-axis', type=int, default=int(cfg.get('right_axis', 4)), choices=[1, 2, 3, 4], help='Achse rechtes Rad (default 4)')
    p.add_argument('--left-dir', type=int, default=int(cfg.get('left_dir', 1)), choices=[1, -1], help='Zählrichtung links (default +1)')
    p.add_argument('--right-dir', type=int, default=int(cfg.get('right_dir', -1)), choices=[1, -1],
                   help='Zählrichtung rechts (default -1, Motor gespiegelt eingebaut)')
    p.add_argument('--only', choices=['both', 'left', 'right'], default='both', help='Nur ein Rad testen')
    p.add_argument('--gear', action='append', default=[], metavar='ACHSE=I',
                   help='Getriebe Motor:Rad je Achse, z.B. --gear 3=5.18 --gear 4=13.73 (default: aus diff_drive.yaml)')
    # jog
    p.add_argument('--rpm', type=float, default=10.0, help='Raddrehzahl [U/min] für jog (default 10)')
    p.add_argument('--left-rpm', type=float, help='Raddrehzahl links, überschreibt --rpm')
    p.add_argument('--right-rpm', type=float, help='Raddrehzahl rechts, überschreibt --rpm')
    p.add_argument('--time', type=float, default=3.0, help='Fahrzeit in s (jog/drive, default 3)')
    p.add_argument('--revs', type=float, help='jog: statt Zeit diese Anzahl Radumdrehungen fahren')
    p.add_argument('--crash-after', type=float, metavar='S',
                   help='jog/drive: nach S Sekunden Absturz simulieren (Prozess endet hart, ohne Abschalten)')
    # drive
    p.add_argument('--v', type=float, default=0.0, help='drive: Geschwindigkeit [m/s]')
    p.add_argument('--w', type=float, default=0.0, help='drive: Drehrate [rad/s], + = links herum')
    p.add_argument('--wheel-diameter', type=float, help='Raddurchmesser [m]')
    p.add_argument('--track', type=float, help='Spurweite (Abstand Radmitten) [m]')
    # Grenzen / Bus
    p.add_argument('--acc', type=float, default=cfg_acc, help='Beschleunigung am Rad [U/s²] (default: wheel_acc '
                   'aus diff_drive.yaml)')
    p.add_argument('--max-wheel-rpm', type=float, default=cfg.get('max_wheel_rpm'),
                   help='Obergrenze Raddrehzahl [U/min] (default: max_wheel_rpm aus diff_drive.yaml)')
    p.add_argument('--loop', choices=['open', 'closed'], default='open',
                   help='open = Schrittmotor ohne Encoder (default), closed = mit Encoder')
    p.add_argument('--max-ferr-rev', type=float, default=cfg.get('max_ferr_rev'),
                   help='Abbruch ab diesem Schleppfehler in Motorumdrehungen (default: max_ferr_rev aus '
                        'diff_drive.yaml)')
    p.add_argument('--motor-ma', type=int, help='Nennstrom der Radmotoren [mA] (Prüfung gegen MaxCurrent)')
    p.add_argument('--units-per-rev', type=int, default=0, help='Einheiten pro Motorumdrehung (default: DivSet)')
    p.add_argument('--settle', type=float, default=1.0, help='Wartezeit nach Freigabe in s')
    p.add_argument('--cycle-ms', type=float, default=2.0, help='EtherCAT-Zyklus in ms (default 2)')
    p.add_argument('--dc', action='store_true', help='Distributed Clocks (Sync0) aktivieren')
    p.add_argument('--ignore-params', action='store_true', help='Parameterprüfung überspringen')
    p.add_argument('--yes', action='store_true', help='Ohne Rückfrage starten')
    p.add_argument('--no-csv', dest='csv', action='store_false', help='Kein CSV-Log schreiben')
    p.add_argument('--csv-dir', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs'),
                   help='Ordner für das CSV-Log')
    args = p.parse_args()

    if args.left_axis == args.right_axis:
        p.error('--left-axis und --right-axis müssen verschieden sein')
    for opt, val in (('--acc', args.acc), ('--max-wheel-rpm', args.max_wheel_rpm),
                     ('--max-ferr-rev', args.max_ferr_rev)):
        if val is None:
            p.error(f'{opt} fehlt und {os.path.normpath(DRIVE_YAML)} liefert keinen Wert')
    if args.acc <= 0:
        p.error('--acc muss > 0 sein')
    gears = {int(cfg[f'{side}_axis']): float(cfg[f'{side}_gear'])
             for side in ('left', 'right') if f'{side}_axis' in cfg and f'{side}_gear' in cfg}
    for g in args.gear:
        try:
            num, ratio = g.split('=')
            gears[int(num)] = float(ratio)
        except ValueError:
            p.error(f'--gear {g}: Format ACHSE=ÜBERSETZUNG, z.B. 4=14')
    for side, num in (('left', args.left_axis), ('right', args.right_axis)):
        if gears.get(num, 0) <= 0:
            p.error(f'Kein gültiges Getriebe für Achse {num} ({side}), z.B. --gear {num}=14')

    # Raddrehzahlen bestimmen (vor dem Bus, damit Fehler früh auffallen)
    rpm_side = None
    if args.mode == 'jog' and args.revs is None:
        rpm_side = {'left': args.rpm if args.left_rpm is None else args.left_rpm,
                    'right': args.rpm if args.right_rpm is None else args.right_rpm}
    elif args.mode == 'drive':
        if not args.wheel_diameter or not args.track:
            p.error('drive braucht --wheel-diameter und --track')
        u = math.pi * args.wheel_diameter    # Radumfang
        rpm_side = {'left': (args.v - args.w * args.track / 2) / u * 60.0,
                    'right': (args.v + args.w * args.track / 2) / u * 60.0}
        print(f'v = {args.v} m/s, w = {args.w} rad/s → Rad links {rpm_side["left"]:+.2f}, '
              f'rechts {rpm_side["right"]:+.2f} U/min')
    if rpm_side:
        worst = max(abs(r) for r in rpm_side.values())
        if worst > args.max_wheel_rpm:
            p.error(f'Raddrehzahl {worst:.1f} U/min > --max-wheel-rpm {args.max_wheel_rpm:g}')
    if args.mode == 'jog' and args.revs is not None and not 0 < abs(args.rpm) <= args.max_wheel_rpm:
        p.error('--rpm muss für --revs zwischen 0 und --max-wheel-rpm liegen')

    if os.geteuid() != 0:
        print('Hinweis: Raw-Socket braucht root → sudo ~/ethercat_venv/bin/python3 ...')

    axes = []
    if args.only in ('both', 'left'):
        axes.append(Axis('left', args.left_axis, args.left_dir, gears[args.left_axis]))
    if args.only in ('both', 'right'):
        axes.append(Axis('right', args.right_axis, args.right_dir, gears[args.right_axis]))
    bus = Bus(args, axes)

    try:
        bus.open()
        if args.mode == 'compare':
            run_compare(bus, args)
            return 0
        bus.read_info()
        if args.mode == 'info':
            return 0

        for a in axes:
            upr = args.units_per_rev or a.params.get('DivSet', 0)
            a.upr = upr if 200 <= upr <= 1_000_000 else 10000
            bus.ferr_limit[a] = int(args.max_ferr_rev * a.upr)
        args.acc_units = {a: args.acc * a.gear * a.upr for a in axes}   # Einheiten/s² am Motor
        print('\n' + ', '.join(f'{a.label}: Getriebe 1:{a.gear:g}, {a.upr} Einh./Motorumdr.' for a in axes))

        if args.mode in ('jog', 'drive'):
            problems = [prob for a in axes for prob in check_params(a, args)]
            for prob in problems:
                print(f'WARNUNG: {prob}')
            if problems and not args.ignore_params:
                raise AbortTest('Parameter erst korrigieren (oder --ignore-params).')
            if not args.motor_ma:
                print('Hinweis: --motor-ma nicht angegeben, MaxCurrent wird nicht gegen den Motornennstrom geprüft.')

        print('\nPDO-Mapping:')
        bus.configure()
        bus.start()

        if args.mode == 'monitor':
            run_monitor(bus, args)
            return 0

        if args.csv:
            bus.log = []
        print()
        if args.revs is not None:
            run_jog_revs(bus, args)
        else:
            run_jog_time(bus, args, {a: rpm_side[a.side] for a in axes})
        return 0
    except AbortTest as e:
        print(f'\nABBRUCH: {e}')
        return 1
    except KeyboardInterrupt:
        print('\nAbbruch durch Nutzer → abbremsen')
        return 1
    finally:
        bus.shutdown()
        if bus.n_cycles:
            print(f'Zyklen {bus.n_cycles}, Überläufe {bus.n_overruns}, WKC-Fehler {bus.n_wkc_err}, '
                  f'max. Periode {bus.max_period * 1000:.2f} ms (Soll {args.cycle_ms} ms)')
        bus.write_csv()


if __name__ == '__main__':
    sys.exit(main())
