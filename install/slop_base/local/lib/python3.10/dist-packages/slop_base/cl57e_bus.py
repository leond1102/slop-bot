"""CL57E-4A über EtherCAT (pysoem): mehrere Achsen im CSP-Modus mit Drehzahlvorgabe, ohne ROS.

Vorlage: b_chain_drive/cl57e.py (eine Achse, Positionsziel) und tests/diff_motor_test.py (erprobt).
Objektindizes aus der ESI LC-EH_4A_V1.0.xml (Vendor 0x0A79, Product 0x6100):
Achse n = Index + (n-1)*0x800, PDOs 0x16n0 / 0x1An0, Herstellerparameter ab 0x2300.

Ablauf: open() → configure() → start() → enable() → set_velocity() ... → close().
Ein Thread schickt jeden Zyklus Controlword und Sollposition aller Achsen. Die Sollposition
entsteht aus einer Drehzahlvorgabe mit begrenzter Beschleunigung (VelRamp).

Sicherheit:
- Fehler oder Schleppfehler an EINER Achse schaltet ALLE ab (sonst dreht der Roboter).
- Watchdog: kommt länger als cmd_timeout_s kein set_velocity(), bremsen alle Achsen auf 0.
  Das schützt, wenn die ROS-Node hängt, der Zyklus-Thread aber weiterläuft.
- Stirbt der ganze Prozess, kommen keine Frames mehr und der Treiber meldet nach SyncCnt
  verlorenen Frames einen Alarm.
"""
import os
import struct
import threading
import time

VENDOR_ID = 0x00000A79
PRODUCT_CODE = 0x6100          # CL57E-4A
AXIS_OFFSET = 0x800
PDO_OFFSET = 0x10
MODE_CSP = 8

# Herstellerparameter (Index Achse 1)
PARAM_RUN_MODE = 0x2301        # 1 = Open Loop, 2 = Closed Loop
PARAM_DIV = 0x2302             # Einheiten pro Motorumdrehung
PARAM_MAX_CURRENT = 0x2303     # mA
PARAM_SENSOR_ERR = 0x2400      # 1 = Encoderfehler melden

# struct-Format der PDO-Objekte (Index von Achse 1)
PDO_TYPES = {
    0x6040: '<H', 0x607A: '<i', 0x60FF: '<i', 0x60B8: '<H',
    0x603F: '<H', 0x6041: '<H', 0x6064: '<i', 0x60FD: '<I', 0x606C: '<i',
    0x60F4: '<i', 0x60B9: '<H', 0x60BA: '<i', 0x60BC: '<i', 0x6061: '<b',
}

CW_DISABLE_VOLTAGE = 0x0000
CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE = 0x000F
CW_FAULT_RESET = 0x0080


class Cl57eError(Exception):
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


class VelRamp:
    """Geschwindigkeit folgt v_cmd mit begrenzter Beschleunigung, Position wird integriert."""

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

    def stopped(self):
        return self.v_cmd == 0.0 and self.vel == 0.0


class _Axis:
    """Zustand einer Achse. Felder ab cw werden unter Cl57eBus._lock geändert."""

    def __init__(self, num):
        self.num = num
        self.ax = (num - 1) * AXIS_OFFSET
        self.rx = {}                  # Index (Achse 1) → (Byte-Offset, Format)
        self.tx = {}
        self.amax = 10000.0           # Einheiten/s²
        self.ferr_limit = 10000       # Einheiten

        self.cw = 0
        self.target = 0
        self.ramp = None              # VelRamp, solange freigegeben
        self.inp = {}


class Cl57eBus:
    """Ein EtherCAT-Master, ein CL57E-4A, mehrere Achsen im CSP-Modus mit Drehzahlvorgabe."""

    def __init__(self, ifname, axis_nums, cycle_s=0.002, dc=False, cmd_timeout_s=0.3, rt_priority=0, rt_cpu=-1):
        """rt_priority > 0: Zyklus-Thread mit SCHED_FIFO dieser Priorität (braucht CAP_SYS_NICE).
        rt_cpu >= 0: Zyklus-Thread auf diesen Kern festlegen."""
        if not axis_nums or len(set(axis_nums)) != len(axis_nums) or any(n not in (1, 2, 3, 4) for n in axis_nums):
            raise ValueError('axis_nums: verschiedene Achsen aus 1..4')
        self.ifname = ifname
        self.axes = {n: _Axis(n) for n in axis_nums}
        self.cycle = cycle_s
        self.dc = dc
        self.cmd_timeout = cmd_timeout_s
        self.rt_priority = int(rt_priority)
        self.rt_cpu = int(rt_cpu)
        self.rt_status = ''           # Ergebnis der Echtzeit-Einstellung (für das Log der Node)
        self.settle_s = 1.0           # Lock-Phase des Treibers nach dem Freigeben

        self._pysoem = None
        self._master = None
        self._slave = None
        self.rx_size = 0
        self.tx_size = 0

        self._lock = threading.Lock()
        self._enabled = False
        self._error = ''
        self._last_cmd = 0.0
        self._watchdog = False
        self._inp_valid = False
        self._t_inp = 0.0             # perf_counter der letzten gültigen Antwort

        self.comm_lost = False
        self._thread_error = None
        self._stop = threading.Event()
        self._thread = None
        self._op = False
        self.stats = {'cycles': 0, 'overruns': 0, 'wkc_errors': 0, 'max_period': 0.0}
        # dasselbe seit dem letzten window_stats()-Aufruf, plus Zyklen > 1,5 × Sollzeit
        self._win = {'cycles': 0, 'overruns': 0, 'wkc_errors': 0, 'max_period': 0.0, 'late': 0}

    # ---------------------------------------------------------------- Verbindung / SDO
    def open(self):
        """Interface öffnen und den CL57E-4A suchen. Gibt den Slave-Namen zurück."""
        import pysoem   # erst hier, damit die Tests ohne pysoem laufen
        self._pysoem = pysoem
        master = pysoem.Master()
        try:
            master.open(self.ifname)
        except Exception as e:
            raise Cl57eError(f'Interface {self.ifname} lässt sich nicht öffnen ({e}). '
                             'Fehlen die Rechte? Start über "ros2 run slop_base with_netcaps ..."')
        self._master = master
        if master.config_init() <= 0:
            raise Cl57eError(f'Kein EtherCAT-Slave an {self.ifname} gefunden')
        for s in master.slaves:
            if s.man == VENDOR_ID and s.id == PRODUCT_CODE:
                self._slave = s
                return s.name
        raise Cl57eError('Kein CL57E-4A (Vendor 0x0A79 / Product 0x6100) am Bus')

    def sdo_r(self, index, sub, fmt):
        data = bytes(self._slave.sdo_read(index, sub))
        size = struct.calcsize(fmt)
        return struct.unpack(fmt, (data + b'\0' * size)[:size])[0]

    def sdo_w(self, index, sub, value, fmt):
        self._slave.sdo_write(index, sub, struct.pack(fmt, value))

    def read_param(self, num, index_axis1, fmt='<H'):
        """Objekt der Achse num lesen (Index von Achse 1 angeben)."""
        return self.sdo_r(index_axis1 + self.axes[num].ax, 0, fmt)

    # ---------------------------------------------------------------- Konfiguration
    def configure(self):
        """PDOs der Achsen zuordnen, Mapping lesen, Betriebsart CSP setzen (PRE-OP)."""
        nums = list(self.axes)
        for assign, base in ((0x1C12, 0x1600), (0x1C13, 0x1A00)):
            self.sdo_w(assign, 0, 0, '<B')
            for i, n in enumerate(nums, 1):
                self.sdo_w(assign, i, base + (n - 1) * PDO_OFFSET, '<H')
            self.sdo_w(assign, 0, len(nums), '<B')
        self.rx_size = self._build_map(self._read_assign(0x1C12), 'rx')
        self.tx_size = self._build_map(self._read_assign(0x1C13), 'tx')
        for a in self.axes.values():
            for need, where in ((0x6040, a.rx), (0x607A, a.rx), (0x6041, a.tx), (0x6064, a.tx)):
                if need not in where:
                    raise Cl57eError(f'Objekt 0x{need + a.ax:04X} fehlt im PDO-Mapping von Achse {a.num}')
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
                for a in self.axes.values():
                    key = (entry >> 16) - a.ax
                    if key in PDO_TYPES and bit % 8 == 0:
                        getattr(a, which)[key] = (bit // 8, PDO_TYPES[key])
                bit += entry & 0xFF
        return (bit + 7) // 8

    def set_limits(self, num, amax, ferr_limit):
        """Beschleunigung [Einheiten/s²] und Schleppfehlergrenze [Einheiten] einer Achse."""
        with self._lock:
            a = self.axes[num]
            a.amax, a.ferr_limit = float(amax), int(ferr_limit)
            if a.ramp is not None:
                a.ramp.amax = a.amax

    def start(self):
        """SAFE-OP → Zyklus-Thread starten → OP."""
        pysoem = self._pysoem
        if self.dc:
            self._master.config_dc()
            self._slave.dc_sync(True, int(self.cycle * 1e9))
        self._master.config_map()
        if len(self._slave.output) != self.rx_size or len(self._slave.input) != self.tx_size:
            raise Cl57eError('Prozessabbild passt nicht zum gelesenen PDO-Mapping')
        if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
            self._raise_al('SAFE-OP nicht erreicht')

        self._thread = threading.Thread(target=self._cyclic, name='cl57e_bus', daemon=True)
        self._thread.start()
        t_end = time.monotonic() + 1.0
        while not self._inp_valid and time.monotonic() < t_end:
            time.sleep(0.01)

        self._master.state = pysoem.OP_STATE
        self._master.write_state()
        for _ in range(40):
            if self._master.state_check(pysoem.OP_STATE, 50000) == pysoem.OP_STATE:
                self._op = True
                break
        if not self._op:
            self._raise_al('OP nicht erreicht' + ('' if self.dc else ' (evtl. dc: true nötig)'))
        for a in self.axes.values():
            mode = self.sdo_r(0x6061 + a.ax, 0, '<b')
            if mode != MODE_CSP:
                raise Cl57eError(f'Achse {a.num}: CSP nicht aktiv (0x6061 = {mode})')

    def _raise_al(self, msg):
        self._master.read_state()
        code = self._slave.al_status
        raise Cl57eError(f'{msg}: AL-Status 0x{code:04X} ({self._pysoem.al_status_code_to_string(code)})')

    # ---------------------------------------------------------------- Zyklus
    def _setup_realtime(self):
        """Im Zyklus-Thread aufrufen: Kern festlegen und SCHED_FIFO setzen (beides gilt nur für diesen Thread).
        Fehlt CAP_SYS_NICE, läuft der Zyklus normal weiter, rt_status sagt warum."""
        msgs = []
        if self.rt_cpu >= 0:
            try:
                os.sched_setaffinity(0, {self.rt_cpu})
                msgs.append(f'Kern {self.rt_cpu}')
            except OSError as e:
                msgs.append(f'Kern {self.rt_cpu} NICHT gesetzt ({e.strerror})')
        if self.rt_priority > 0:
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(self.rt_priority))
                msgs.append(f'SCHED_FIFO {self.rt_priority}')
            except OSError as e:
                msgs.append(f'SCHED_FIFO NICHT gesetzt ({e.strerror}; über with_netcaps starten)')
        self.rt_status = ', '.join(msgs) if msgs else 'ohne Echtzeit-Einstellungen'

    def window_stats(self):
        """Zyklusstatistik seit dem letzten Aufruf (und zurücksetzen)."""
        with self._lock:
            w = dict(self._win)
            self._win = {k: (0.0 if k == 'max_period' else 0) for k in self._win}
        return w

    def _cyclic(self):
        self._setup_realtime()
        out = bytearray(self.rx_size)
        loss_limit = max(10, int(0.1 / self.cycle))   # ~100 ms ohne gültige Antwort
        wkc_bad = 0
        nxt = last = time.perf_counter()
        try:
            while not self._stop.is_set():
                with self._lock:
                    self._step_setpoints(time.monotonic())
                    for a in self.axes.values():
                        struct.pack_into('<H', out, a.rx[0x6040][0], a.cw)
                        struct.pack_into('<i', out, a.rx[0x607A][0], a.target)
                        if 0x60FF in a.rx:
                            struct.pack_into('<i', out, a.rx[0x60FF][0], 0)
                self._slave.output = bytes(out)

                self._master.send_processdata()
                wkc = self._master.receive_processdata(2000)
                now = time.perf_counter()

                if wkc >= self._master.expected_wkc:
                    wkc_bad = 0
                    data = self._slave.input
                    with self._lock:
                        for a in self.axes.values():
                            a.inp = {k: struct.unpack_from(f, data, o)[0] for k, (o, f) in a.tx.items()}
                        self._inp_valid = True
                        self._t_inp = now
                        self._supervise()
                else:
                    wkc_bad += 1
                    self.stats['wkc_errors'] += 1
                    self._win['wkc_errors'] += 1
                    if wkc_bad >= loss_limit and self._op:
                        self.comm_lost = True

                self.stats['cycles'] += 1
                self._win['cycles'] += 1
                if self.stats['cycles'] > 5:
                    period = now - last
                    self.stats['max_period'] = max(self.stats['max_period'], period)
                    self._win['max_period'] = max(self._win['max_period'], period)
                    if period > 1.5 * self.cycle:
                        self._win['late'] += 1
                last = now
                nxt += self.cycle
                sleep = nxt - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    self.stats['overruns'] += 1
                    self._win['overruns'] += 1
                    if sleep < -10 * self.cycle:
                        nxt = time.perf_counter()
        except Exception as e:
            self._thread_error = e
            self.comm_lost = True

    def _step_setpoints(self, now):
        """Läuft unter self._lock: Watchdog, dann Sollpositionen einen Zyklus weiterrechnen."""
        if self._enabled and now - self._last_cmd > self.cmd_timeout:
            if not self._watchdog:
                self._watchdog = True
                for a in self.axes.values():
                    a.ramp.v_cmd = 0.0
        for a in self.axes.values():
            if a.ramp is not None:
                a.target = int(round(a.ramp.step(self.cycle)))

    def _supervise(self):
        """Läuft unter self._lock. Nicht freigegeben: Soll folgt Ist. Freigegeben: überwachen."""
        if not self._enabled:
            for a in self.axes.values():
                a.target = a.inp[0x6064]
            return
        for a in self.axes.values():
            sw = a.inp[0x6041]
            st = drive_state(sw)
            if st != 'OPERATION_ENABLED':
                self._drop_all(f'Achse {a.num}: Treiber nicht mehr freigegeben ({st}, SW 0x{sw:04X})')
                return
            ferr = a.target - a.inp[0x6064]
            if abs(ferr) > a.ferr_limit:
                self._drop_all(f'Achse {a.num}: Schleppfehler {ferr} > {a.ferr_limit} Einheiten')
                return

    def _drop_all(self, msg):
        self._enabled = False
        for a in self.axes.values():
            a.ramp = None
            a.target = a.inp.get(0x6064, a.target)
            a.cw = CW_DISABLE_VOLTAGE if drive_state(a.inp.get(0x6041, 0)) == 'FAULT' else CW_SHUTDOWN
        self._error = msg

    # ---------------------------------------------------------------- Bedienung
    def enable(self, timeout=5.0):
        """Fault-Reset und Zustandsmaschine bis OPERATION_ENABLED für alle Achsen. Blockiert."""
        with self._lock:
            self._error = ''
            self._enabled = False
            for a in self.axes.values():
                a.ramp = None
        t_end = time.monotonic() + timeout
        last_toggle = 0.0
        while True:
            if self.comm_lost:
                raise Cl57eError('EtherCAT-Kommunikation verloren')
            toggle = time.monotonic() - last_toggle > 0.1
            with self._lock:
                states = {a.num: drive_state(a.inp.get(0x6041, 0)) for a in self.axes.values()}
                for a in self.axes.values():
                    st = states[a.num]
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
                bad = ', '.join(f'Achse {n}: {st}' for n, st in states.items() if st != 'OPERATION_ENABLED')
                raise Cl57eError(f'Freigabe fehlgeschlagen ({bad})')
            time.sleep(0.02)

        time.sleep(self.settle_s)   # Lock-Phase: Soll folgt noch dem Ist
        with self._lock:
            for a in self.axes.values():
                sw = a.inp.get(0x6041, 0)
                if drive_state(sw) != 'OPERATION_ENABLED':
                    for b in self.axes.values():
                        b.cw = CW_SHUTDOWN
                    raise Cl57eError(f'Achse {a.num} nach der Lock-Phase nicht mehr freigegeben (SW 0x{sw:04X})')
            for a in self.axes.values():   # ab hier hält jede Achse ihre Position
                a.target = a.inp[0x6064]
                a.ramp = VelRamp(a.target, a.amax)
            self._last_cmd = time.monotonic()
            self._watchdog = False
            self._enabled = True

    def disable(self):
        """Endstufen aus (Shutdown). Läuft ein Motor noch, bleibt er ohne Rampe stehen."""
        with self._lock:
            self._enabled = False
            for a in self.axes.values():
                a.ramp = None
                a.cw = CW_SHUTDOWN

    def set_velocity(self, vel_by_axis):
        """Drehzahl je Achse [Einheiten/s] vorgeben. Muss regelmäßig kommen (Watchdog)."""
        with self._lock:
            self._last_cmd = time.monotonic()
            self._watchdog = False
            if not self._enabled:
                return
            for num, v in vel_by_axis.items():
                self.axes[num].ramp.v_cmd = float(v)

    def stop(self, timeout=3.0):
        """Mit Rampe auf 0 bremsen und warten. True, wenn alle Achsen stehen."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.set_velocity({n: 0.0 for n in self.axes})
            with self._lock:
                if not self._enabled or all(a.ramp.stopped() for a in self.axes.values()):
                    return True
            if self.comm_lost:
                return False
            time.sleep(0.01)
        return False

    def status(self):
        """Momentaufnahme: Istpositionen, Zustände, Fehler. 't' = Empfangszeit (perf_counter)."""
        with self._lock:
            return {
                'valid': self._inp_valid,
                't': self._t_inp,
                'actual': {n: a.inp.get(0x6064, 0) for n, a in self.axes.items()},
                'target': {n: a.target for n, a in self.axes.items()},
                'state': {n: drive_state(a.inp.get(0x6041, 0)) for n, a in self.axes.items()},
                'enabled': self._enabled,
                'moving': self._enabled and not all(a.ramp.stopped() for a in self.axes.values()),
                'watchdog': self._watchdog,
                'error': self._error,
                'comm_lost': self.comm_lost,
                'thread_error': self._thread_error,
            }

    def close(self):
        """Abbremsen, Endstufen aus, Thread stoppen, Bus auf INIT, Interface schließen. Immer aufrufen."""
        if self._thread is not None and self._thread.is_alive():
            if not self.comm_lost:
                self.stop()
            self.disable()
            time.sleep(0.1)
            with self._lock:
                for a in self.axes.values():
                    a.cw = CW_DISABLE_VOLTAGE
            time.sleep(0.05)
            self._stop.set()
            self._thread.join(timeout=1.0)
        if self._master is not None:
            try:
                self._master.state = self._pysoem.INIT_STATE
                self._master.write_state()
                self._master.close()
            except Exception:
                pass
            self._master = None
