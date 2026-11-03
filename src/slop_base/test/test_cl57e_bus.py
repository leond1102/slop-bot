"""PDO-Mapping, Rampe und Watchdog von Cl57eBus, ohne Hardware, ohne pysoem und ohne ROS."""
import struct

import pytest

from slop_base.cl57e_bus import Cl57eBus, Cl57eError, VelRamp


class FakeSlave:
    """SDO-Speicher mit dem Mapping, das der echte Treiber pro Achse meldet (0x6040/607A/60FF, 0x6041/6064/606C)."""

    def __init__(self):
        self.od = {}
        for n in (1, 2, 3, 4):
            ax = (n - 1) * 0x800
            for base, entries in ((0x1600, [0x6040, 0x607A, 0x60FF]), (0x1A00, [0x6041, 0x6064, 0x606C])):
                pdo = base + (n - 1) * 0x10
                self.od[(pdo, 0)] = len(entries)
                for i, idx in enumerate(entries, 1):
                    self.od[(pdo, i)] = ((idx + ax) << 16) | (16 if idx & 0xFF in (0x40, 0x41) else 32)

    def sdo_read(self, index, sub):
        return struct.pack('<I', self.od.get((index, sub), 0))

    def sdo_write(self, index, sub, data):
        self.od[(index, sub)] = int.from_bytes(data, 'little')


def test_configure_axes_3_and_4():
    bus = Cl57eBus('dummy', [3, 4])
    bus._slave = FakeSlave()
    bus.configure()
    od = bus._slave.od
    assert [od[(0x1C12, i)] for i in range(od[(0x1C12, 0)] + 1)][1:] == [0x1620, 0x1630]
    assert [od[(0x1C13, i)] for i in range(od[(0x1C13, 0)] + 1)][1:] == [0x1A20, 0x1A30]
    assert bus.axes[3].rx == {0x6040: (0, '<H'), 0x607A: (2, '<i'), 0x60FF: (6, '<i')}
    assert bus.axes[4].rx == {0x6040: (10, '<H'), 0x607A: (12, '<i'), 0x60FF: (16, '<i')}
    assert bus.axes[4].tx == {0x6041: (10, '<H'), 0x6064: (12, '<i'), 0x606C: (16, '<i')}
    assert bus.rx_size == 20 and bus.tx_size == 20
    assert od[(0x6060 + 0x1000, 0)] == 8 and od[(0x6060 + 0x1800, 0)] == 8


def test_invalid_axes():
    with pytest.raises(ValueError):
        Cl57eBus('dummy', [3, 3])
    with pytest.raises(ValueError):
        Cl57eBus('dummy', [5])


def test_vel_ramp_accel_limited_and_stops():
    r = VelRamp(0, amax=1000.0)
    r.v_cmd = 500.0
    r.step(0.002)
    assert r.vel == pytest.approx(2.0)
    for _ in range(1000):
        r.step(0.002)
    assert r.vel == 500.0
    r.v_cmd = 0.0
    for _ in range(1000):
        r.step(0.002)
    assert r.stopped()


def _enabled_bus():
    """Bus im Zustand 'freigegeben', ohne Thread und ohne Hardware."""
    bus = Cl57eBus('dummy', [3, 4], cmd_timeout_s=0.3)
    for a in bus.axes.values():
        a.ramp = VelRamp(0, 1e6)
    bus._enabled = True
    bus._last_cmd = 100.0
    return bus


def test_watchdog_brakes_without_commands():
    bus = _enabled_bus()
    bus.set_velocity({3: 1000.0, 4: -1000.0})
    t = bus._last_cmd
    bus._step_setpoints(t + 0.1)
    assert bus.axes[3].ramp.v_cmd == 1000.0 and not bus._watchdog
    bus._step_setpoints(t + 0.5)
    assert bus._watchdog
    assert bus.axes[3].ramp.v_cmd == 0.0 and bus.axes[4].ramp.v_cmd == 0.0


def test_fault_on_one_axis_drops_both():
    bus = _enabled_bus()
    bus.axes[3].inp = {0x6041: 0x0237, 0x6064: 0}     # OPERATION_ENABLED
    bus.axes[4].inp = {0x6041: 0x0218, 0x6064: 0}     # FAULT
    bus._supervise()
    assert not bus._enabled and 'Achse 4' in bus._error
    assert all(a.ramp is None for a in bus.axes.values())


def test_following_error_drops_both():
    bus = _enabled_bus()
    for a in bus.axes.values():
        a.inp = {0x6041: 0x0237, 0x6064: 0}
        a.ferr_limit = 100
    bus.axes[3].target = 500
    bus._supervise()
    assert not bus._enabled and 'Schleppfehler' in bus._error


def test_set_velocity_ignored_when_disabled():
    bus = Cl57eBus('dummy', [3, 4])
    bus.set_velocity({3: 1000.0, 4: 1000.0})      # darf nicht werfen, nichts passiert
    assert not bus._enabled


def test_error_type():
    assert issubclass(Cl57eError, Exception)
