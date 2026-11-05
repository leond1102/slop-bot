"""Busprozess ↔ diff_drive_node über bus_link: echte Unix-Sockets, nachgebauter Bus, ohne Hardware und ohne ROS."""
import itertools
import os
import threading
import time

import pytest

from slop_base.bus_link import BusClient, BusServer
from slop_base.bus_process import DEFAULTS, load_config
from slop_base.cl57e_bus import Cl57eError

_names = itertools.count()


class FakeBus:
    """Verhält sich nach außen wie Cl57eBus (status, set_velocity, enable, disable, stop)."""

    def __init__(self, enable_error=None):
        self.enabled = False
        self.vel = {}
        self.calls = []
        self.enable_error = enable_error
        self.comm_lost = False

    def status(self):
        return {'valid': True, 't': time.perf_counter(), 'actual': {3: 100, 4: -200}, 'target': {3: 100, 4: -200},
                'state': {3: 'OPERATION_ENABLED', 4: 'OPERATION_ENABLED'}, 'enabled': self.enabled,
                'moving': False, 'watchdog': False, 'error': '', 'comm_lost': self.comm_lost,
                'thread_error': None}

    def set_velocity(self, v):
        self.vel = dict(v)

    def enable(self):
        self.calls.append('enable')
        if self.enable_error:
            raise Cl57eError(self.enable_error)
        self.enabled = True

    def disable(self):
        self.calls.append('disable')
        self.enabled = False

    def stop(self):
        self.calls.append('stop')


@pytest.fixture
def link():
    """Server läuft in einem Thread wie im Busprozess; liefert (bus, server, client, name)."""
    name = f'slop_test/{os.getpid()}/{next(_names)}'
    bus = FakeBus()
    logs = []
    server = BusServer(bus, name, {'name': 'Fake', 'upr': {'3': 10000, '4': 10000}, 'cycle_ms': 2.0},
                       node_timeout=0.3, status_period=0.01, log=logs.append)
    stop = threading.Event()
    thread = threading.Thread(target=lambda: [server.step(0.005) for _ in iter(stop.is_set, True)], daemon=True)
    thread.start()
    client = BusClient(name)
    yield bus, server, client, logs
    stop.set()
    thread.join(1.0)
    client.close()
    server.close()


def _wait(cond, timeout=1.0):
    t_end = time.monotonic() + timeout
    while not cond() and time.monotonic() < t_end:
        time.sleep(0.005)
    return cond()


def test_connect_returns_info_and_status_arrives(link):
    bus, server, client, _ = link
    info = client.connect(2.0)
    assert info['name'] == 'Fake' and info['upr'] == {'3': 10000, '4': 10000}
    assert _wait(lambda: (client.poll(), client.status())[1] is not None)
    st = client.status()
    assert st['actual'] == {3: 100, 4: -200} and st['state'][4] == 'OPERATION_ENABLED'
    assert client.status_age() < 0.5


def test_velocity_enable_disable_are_forwarded(link):
    bus, server, client, _ = link
    client.connect(2.0)
    client.enable(2.0)
    assert bus.enabled and bus.calls == ['enable']
    client.set_velocity({3: 1234.5, 4: -99.0})
    assert _wait(lambda: bus.vel == {3: 1234.5, 4: -99.0})
    client.disable()
    assert _wait(lambda: not bus.enabled)


def test_enable_error_reaches_node(link):
    bus, server, client, _ = link
    bus.enable_error = 'Achse 4: FAULT'
    client.connect(2.0)
    with pytest.raises(Cl57eError, match='Achse 4: FAULT'):
        client.enable(2.0)


def test_silent_node_disables_drives(link):
    bus, server, client, logs = link
    client.connect(2.0)
    client.enable(2.0)
    assert _wait(lambda: not bus.enabled, timeout=2.0)          # node_timeout 0,3 s ohne Nachricht
    assert any('still' in text for text in logs)
    assert _wait(lambda: (client.poll(), client.status() or {})[1].get('error', '').startswith('diff_drive_node'))


def test_bye_brakes_and_disables(link):
    bus, server, client, _ = link
    client.connect(2.0)
    client.enable(2.0)
    client.bye()
    assert _wait(lambda: bus.calls[-2:] == ['stop', 'disable'])
    assert _wait(lambda: server.owner is None)


def test_second_node_is_rejected_while_first_is_alive(link):
    bus, server, client, _ = link
    client.connect(2.0)
    other = BusClient(server.name)
    try:
        # die erste Node hält die Verbindung mit Sollwerten wach
        keep = threading.Thread(target=lambda: [client.set_velocity({3: 0.0}) or time.sleep(0.02) for _ in range(30)])
        keep.start()
        with pytest.raises(Cl57eError, match='anderen'):
            other.connect(0.5)
        other.set_velocity({3: 777.0})                          # Befehle der zweiten Node zählen nicht
        time.sleep(0.05)
        assert bus.vel.get(3) != 777.0
        keep.join()
    finally:
        other.close()


def test_exit_message_and_missing_bus():
    name = f'slop_test/{os.getpid()}/{next(_names)}'
    client = BusClient(name)
    try:
        with pytest.raises(Cl57eError, match='antwortet nicht'):
            client.connect(0.3)
        server = BusServer(FakeBus(), name, {'upr': {}}, log=lambda _: None)
        stop = threading.Event()
        thread = threading.Thread(target=lambda: [server.step(0.005) for _ in iter(stop.is_set, True)], daemon=True)
        thread.start()
        client.connect(1.0)
        server.notify_exit('EtherCAT-Kommunikation verloren')
        assert _wait(lambda: (client.poll(), client.exit_reason)[1] == 'EtherCAT-Kommunikation verloren')
        stop.set()
        thread.join(1.0)
        server.close()
    finally:
        client.close()


def test_load_config_reads_node_block(tmp_path):
    p = tmp_path / 'd.yaml'
    p.write_text('diff_drive_node:\n  ros__parameters:\n    ifname: eth9\n    right_gear: 14.0\n    rate: 50.0\n')
    cfg = load_config(str(p))
    assert cfg['ifname'] == 'eth9' and cfg['right_gear'] == 14.0
    assert 'rate' not in cfg and cfg['cycle_ms'] == DEFAULTS['cycle_ms']
