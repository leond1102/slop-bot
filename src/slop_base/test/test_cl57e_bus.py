"""PDO-Mapping, Rampe und Watchdog von Cl57eBus, ohne Hardware, ohne pysoem und ohne ROS."""
import struct

import pytest

from slop_base.cl57e_bus import Cl57eBus, Cl57eError, PosRamp, VelRamp


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


def _assigned(od, index):
    return [od[(index, i)] for i in range(1, od[(index, 0)] + 1)]


def test_configure_assigns_all_four_pdos():
    """Immer alle vier PDOs ab Achse 1, sonst versteckt der Treiber Achse 1/2 (siehe cl57e_bus-Docstring)."""
    bus = Cl57eBus('dummy', [3, 4])
    bus._slave = FakeSlave()
    bus.configure()
    od = bus._slave.od
    assert _assigned(od, 0x1C12) == [0x1600, 0x1610, 0x1620, 0x1630]
    assert _assigned(od, 0x1C13) == [0x1A00, 0x1A10, 0x1A20, 0x1A30]
    assert bus.axes[3].rx == {0x6040: (20, '<H'), 0x607A: (22, '<i'), 0x60FF: (26, '<i')}
    assert bus.axes[4].rx == {0x6040: (30, '<H'), 0x607A: (32, '<i'), 0x60FF: (36, '<i')}
    assert bus.axes[4].tx == {0x6041: (30, '<H'), 0x6064: (32, '<i'), 0x606C: (36, '<i')}
    assert bus.rx_size == 40 and bus.tx_size == 40
    assert od[(0x6060 + 0x1000, 0)] == 8 and od[(0x6060 + 0x1800, 0)] == 8
    assert (0x6060, 0) not in od           # unbenutzte Achsen bleiben, wie sie sind


def test_configure_with_gripper_axes():
    bus = Cl57eBus('dummy', [3, 4], grip_nums=[1, 2])
    bus._slave = FakeSlave()
    bus.configure()
    assert bus.gaxes[1].rx[0x6040] == (0, '<H') and bus.gaxes[2].tx[0x6064] == (12, '<i')
    assert all(bus._slave.od[(0x6060 + (n - 1) * 0x800, 0)] == 8 for n in (1, 2, 3, 4))


def test_invalid_axes():
    with pytest.raises(ValueError):
        Cl57eBus('dummy', [3, 3])
    with pytest.raises(ValueError):
        Cl57eBus('dummy', [5])
    with pytest.raises(ValueError):
        Cl57eBus('dummy', [3, 4], grip_nums=[3])


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


# ---------------------------------------------------------------- Greifer
def _grip_bus():
    """Räder und Greifer, Greifer freigegeben, Räder nicht. Ohne Thread und ohne Hardware."""
    bus = Cl57eBus('dummy', [3, 4], grip_nums=[1, 2])
    for a in bus.axes.values():
        a.inp = {0x6041: 0x0231, 0x6064: 0}
    for a in bus.gaxes.values():
        a.inp = {0x6041: 0x0237, 0x6064: 0}
        a.ramp = PosRamp(0, 0, 1.0, 1e5)
        a.ferr_limit = 10000
    bus._genabled = True
    return bus


def test_pos_ramp_reaches_goal_and_stops_early():
    r = PosRamp(0, 1000, vmax=10000.0, amax=50000.0)
    for _ in range(2000):
        r.step(0.002)
    assert r.done() and r.pos == 1000
    r = PosRamp(0, 100000, vmax=10000.0, amax=50000.0)
    for _ in range(100):
        r.step(0.002)
    r.stop_here()
    for _ in range(2000):
        r.step(0.002)
    assert r.done() and 0 < r.pos < 100000


def test_grip_move_steps_only_gripper():
    bus = _grip_bus()
    assert bus.grip_move({1: 500.0, 2: -500.0}, 10000.0, 1e6)
    for _ in range(200):
        bus._step_setpoints(0.0)
    assert bus.gaxes[1].target == 500 and bus.gaxes[2].target == -500
    assert bus.axes[3].ramp is None and bus.axes[3].target == 0
    assert bus.grip_status()['moving'] is False
    with pytest.raises(ValueError):
        bus.grip_move({1: 0.0}, 0.0, 1.0)


def test_grip_move_rejected_when_disabled():
    bus = _grip_bus()
    bus.grip_disable()
    assert not bus.grip_move({1: 500.0}, 1000.0, 1000.0)


def test_grip_fault_drops_only_gripper():
    bus = _grip_bus()
    for a in bus.axes.values():
        a.ramp = VelRamp(0, 1e6)
        a.inp = {0x6041: 0x0237, 0x6064: 0}
    bus._enabled = True
    bus.gaxes[2].inp = {0x6041: 0x0218, 0x6064: 0}     # FAULT am Greifer
    bus._supervise()
    assert not bus._genabled and 'Greifer-Achse 2' in bus._gerror
    assert bus._enabled and bus._error == ''           # Räder laufen weiter


def test_wheel_fault_leaves_gripper_enabled():
    bus = _grip_bus()
    for a in bus.axes.values():
        a.ramp = VelRamp(0, 1e6)
    bus._enabled = True
    bus.axes[3].inp = {0x6041: 0x0218, 0x6064: 0}
    bus.axes[4].inp = {0x6041: 0x0237, 0x6064: 0}
    bus._supervise()
    assert not bus._enabled and bus._genabled


def test_status_contains_grip_section():
    bus = _grip_bus()
    st = bus.status()
    assert st['grip']['enabled'] and set(st['grip']['actual']) == {1, 2}
    assert 'grip' not in Cl57eBus('dummy', [3, 4]).status()
