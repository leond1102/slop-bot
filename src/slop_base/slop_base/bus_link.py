"""Verbindung zwischen EtherCAT-Busprozess (ethercat_bus) und diff_drive_node, ohne ROS.

Warum zwei Prozesse: Der 2-ms-Zyklus lief früher als Thread in der ROS-Node und teilte sich mit rclpy das Python-GIL.
Hing ein publish (z. B. 326 ms, während DDS neue Nodes verbindet), blieben auch die EtherCAT-Frames aus und der Treiber
verlor die Verbindung. Jetzt hat der Busprozess kein rclpy; hängt die Node, greift nur noch der Watchdog (bremsen).

Transport: Unix-Datagramm-Sockets im abstrakten Namensraum (keine Dateien, verschwinden mit dem Prozess), JSON-Nachrichten,
beide Seiten nicht blockierend.

  Node → Bus   {"op": "hello"}                       Bus antwortet {"op": "info", ...} (Treibername, Einheiten/U, Echtzeit)
               {"op": "vel", "v": {"3": u/s, ...}}   Drehzahl je Achse [Einheiten/s], 50 Hz (hält den Watchdog wach)
               {"op": "enable", "id": n}             Bus antwortet {"op": "enable_result", "id": n, "ok": .., "msg": ..}
               {"op": "disable"}                     Endstufen aus
               {"op": "bye"}                         bremsen, Endstufen aus (Node beendet sich)
  Bus → Node   {"op": "status", ...}                 alle status_period s (100 Hz): Istpositionen, Zustände, Fehler
               {"op": "busy"}                        eine andere Node ist schon verbunden
               {"op": "exit", "reason": ..}          Busprozess beendet sich

Sicherheit im Busprozess (BusServer): Kommen keine "vel" mehr, bremst der Watchdog von Cl57eBus (cmd_timeout). Ist die
Node node_timeout s ganz still, schaltet der Server die Endstufen aus. Es nimmt nur eine Node Befehle an.
"""
import itertools
import json
import os
import select
import socket
import time

from slop_base.cl57e_bus import Cl57eError

DEFAULT_NAME = 'slop_base/ethercat'
_client_ids = itertools.count()


def bus_address(name):
    return '\0' + name + '.bus'


def node_address(name):
    return f'\0{name}.node.{os.getpid()}.{next(_client_ids)}'


def _encode(obj):
    return json.dumps(obj, separators=(',', ':')).encode()


def _open_socket(address):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sock.bind(address)
    except OSError:
        sock.close()
        raise
    sock.setblocking(False)
    return sock


def _send(sock, address, obj):
    """Nicht blockierend senden. False, wenn der Empfänger fehlt oder sein Puffer voll ist."""
    try:
        sock.sendto(_encode(obj), address)
        return True
    except OSError:          # ConnectionRefused (kein Empfänger), BlockingIOError (Puffer voll), FileNotFound
        return False


def _receive_all(sock, timeout):
    """Alle wartenden Nachrichten [(dict, Absender)]; wartet höchstens timeout s auf die erste."""
    out = []
    if timeout > 0 and not select.select([sock], [], [], timeout)[0]:
        return out
    while True:
        try:
            data, addr = sock.recvfrom(65536)
        except (BlockingIOError, InterruptedError):
            return out
        try:
            msg = json.loads(data)
        except ValueError:
            continue
        if isinstance(msg, dict):
            out.append((msg, addr))


class BusServer:
    """Busseite: nimmt Befehle einer Node an und gibt sie an `bus` weiter (Cl57eBus oder Nachbau für Tests).

    `bus` braucht: status(), set_velocity(dict), enable(), disable(), stop()."""

    def __init__(self, bus, name=DEFAULT_NAME, info=None, node_timeout=2.0, status_period=0.01, log=print):
        self.bus = bus
        self.name = name
        self.info = dict(info or {})
        self.node_timeout = node_timeout
        self.status_period = status_period
        self.log = log
        self.sock = _open_socket(bus_address(name))
        self.owner = None              # Adresse der verbundenen Node
        self.last_rx = 0.0             # monotonic der letzten Nachricht der Node
        self.node_lost = False
        self.note = ''                 # eigener Fehlertext des Servers (z. B. Node still), bis zur nächsten Freigabe
        self._next_status = 0.0

    def _owner_silent(self, now):
        return self.owner is None or now - self.last_rx > self.node_timeout

    def step(self, timeout=0.01):
        """Nachrichten bearbeiten, Node überwachen, Status senden. Einmal je Schleifendurchlauf aufrufen."""
        for msg, addr in _receive_all(self.sock, timeout):
            self._handle(msg, addr)
        now = time.monotonic()
        if self.owner is not None and not self.node_lost and now - self.last_rx > self.node_timeout:
            self.node_lost = True
            if self.bus.status().get('enabled'):
                self.bus.disable()
                self.note = f'diff_drive_node seit {self.node_timeout:g} s still → Endstufen aus'
                self.log(f'{self.note}. Neu freigeben über diff_drive/enable')
            else:
                self.log(f'diff_drive_node seit {self.node_timeout:g} s still')
        if self.owner is not None and now >= self._next_status:
            self._next_status = now + self.status_period
            _send(self.sock, self.owner, self.status_message())

    def status_message(self):
        st = dict(self.bus.status())
        st['thread_error'] = str(st['thread_error']) if st.get('thread_error') else ''
        if self.note and not st.get('error'):
            st['error'] = self.note
        st['op'] = 'status'
        return st

    def _handle(self, msg, addr):
        op = msg.get('op')
        now = time.monotonic()
        if op == 'hello':
            if addr != self.owner and not self._owner_silent(now):
                _send(self.sock, addr, {'op': 'busy'})
                return
            if addr != self.owner:
                self.log('diff_drive_node verbunden')
            self.owner = addr
        elif addr != self.owner:
            return                     # Befehle nur von der verbundenen Node
        self.last_rx = now
        if self.node_lost:
            self.node_lost = False
            self.log('diff_drive_node meldet sich wieder')
        if op == 'hello':
            _send(self.sock, addr, dict(self.info, op='info'))
        elif op == 'vel':
            self.bus.set_velocity({int(k): float(v) for k, v in msg.get('v', {}).items()})
        elif op == 'enable':
            self.note = ''
            try:
                self.bus.enable()
                ok, text = True, 'freigegeben'
            except Cl57eError as e:
                ok, text = False, str(e)
            self.last_rx = time.monotonic()      # enable() blockiert einige Sekunden
            _send(self.sock, addr, {'op': 'enable_result', 'id': msg.get('id'), 'ok': ok, 'msg': text})
        elif op == 'disable':
            self.bus.disable()
        elif op == 'bye':
            self.bus.stop()
            self.bus.disable()
            self.log('diff_drive_node beendet sich → gebremst, Endstufen aus')
            self.owner = None

    def notify_exit(self, reason):
        if self.owner is not None:
            _send(self.sock, self.owner, {'op': 'exit', 'reason': reason})

    def close(self):
        self.sock.close()


class BusClient:
    """Nodeseite: spricht mit dem Busprozess. Ersetzt in diff_drive_node den direkten Cl57eBus."""

    def __init__(self, name=DEFAULT_NAME):
        self.dest = bus_address(name)
        self.sock = _open_socket(node_address(name))
        self.info = None
        self._status = None
        self._status_time = None       # monotonic des letzten Status
        self.exit_reason = ''
        self.busy = False
        self._results = {}
        self._next_id = 0

    def poll(self, timeout=0.0):
        for msg, _ in _receive_all(self.sock, timeout):
            op = msg.get('op')
            if op == 'status':
                self._status, self._status_time = msg, time.monotonic()
            elif op == 'info':
                self.info = msg
            elif op == 'enable_result':
                self._results[msg.get('id')] = (bool(msg.get('ok')), str(msg.get('msg', '')))
            elif op == 'exit':
                self.exit_reason = str(msg.get('reason', '')) or 'ohne Angabe'
            elif op == 'busy':
                self.busy = True

    def connect(self, timeout=20.0):
        """Mit dem Busprozess verbinden; gibt dessen info zurück (Treibername, upr je Achse, Echtzeit)."""
        t_end = time.monotonic() + timeout
        while self.info is None:
            if time.monotonic() > t_end:
                raise Cl57eError(f'Busprozess ethercat_bus antwortet nicht ({timeout:g} s). Läuft er? '
                                 '(diff_drive.launch.py startet ihn mit)')
            _send(self.sock, self.dest, {'op': 'hello'})
            self.poll(0.2)
            if self.busy:
                raise Cl57eError('Busprozess ist schon mit einer anderen diff_drive_node verbunden')
        if self._status_time is None:
            self._status_time = time.monotonic()     # Frist für den ersten Status beginnt jetzt
        return self.info

    def status_age(self):
        return float('inf') if self._status_time is None else time.monotonic() - self._status_time

    def status(self):
        """Letzter Status des Busprozesses wie Cl57eBus.status() (Achsen als int), None vor dem ersten."""
        if self._status is None:
            return None
        st = dict(self._status)
        for key in ('actual', 'target', 'state'):
            st[key] = {int(k): v for k, v in st.get(key, {}).items()}
        return st

    def set_velocity(self, vel_by_axis):
        _send(self.sock, self.dest, {'op': 'vel', 'v': {str(k): float(v) for k, v in vel_by_axis.items()}})

    def enable(self, timeout=15.0):
        """Freigabe anfordern und auf das Ergebnis warten (Cl57eBus.enable dauert einige Sekunden)."""
        self._next_id += 1
        rid = self._next_id
        if not _send(self.sock, self.dest, {'op': 'enable', 'id': rid}):
            raise Cl57eError('Busprozess nicht erreichbar')
        t_end = time.monotonic() + timeout
        while rid not in self._results:
            if self.exit_reason:
                raise Cl57eError(f'Busprozess beendet: {self.exit_reason}')
            if time.monotonic() > t_end:
                raise Cl57eError(f'Keine Antwort auf die Freigabe ({timeout:g} s)')
            self.poll(0.05)
        ok, text = self._results.pop(rid)
        if not ok:
            raise Cl57eError(text)

    def disable(self):
        _send(self.sock, self.dest, {'op': 'disable'})

    def bye(self):
        _send(self.sock, self.dest, {'op': 'bye'})

    def close(self):
        self.sock.close()
