"""Messfenster, Neigungsrechnung und Quaternion-Drehung, ohne ROS."""
import math

import pytest

from slop_calibration.still import G, StillStats, rotate, tilt_from_gravity


def test_stats_mean_and_std():
    s = StillStats()
    for i in range(1000):
        noise = 0.001 if i % 2 else -0.001
        s.add(i * 0.005, (0.01 + noise, 0.0, -0.02), (0.0, 0.0, G))
    gm, gs, am, asd = s.result()
    assert gm == pytest.approx((0.01, 0.0, -0.02), abs=1e-9)
    assert gs[0] == pytest.approx(0.001, rel=1e-3) and gs[1] == pytest.approx(0.0, abs=1e-9)
    assert am[2] == pytest.approx(G) and asd[2] == pytest.approx(0.0, abs=1e-6)
    assert s.duration == pytest.approx(4.995)


def test_level_sensor_has_zero_tilt():
    assert tilt_from_gravity((0.0, 0.0, G)) == (0.0, 0.0)


@pytest.mark.parametrize('roll,pitch', [(0.05, 0.0), (0.0, -0.08), (0.1, 0.07), (-0.2, 0.15)])
def test_tilt_roundtrip(roll, pitch):
    # Schwerkraft (0, 0, g) im Sensorframe, der um rpy (fest x, y, z) gedreht ist: a = R^T g
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    a = (-sp * G, sr * cp * G, cr * cp * G)
    r, p = tilt_from_gravity(a)
    assert r == pytest.approx(roll) and p == pytest.approx(pitch)


def test_rotate_quaternion():
    s = math.sqrt(0.5)
    # 90° um z: x → y
    assert rotate((0.0, 0.0, s, s), (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
    # Identität
    assert rotate((0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0)) == pytest.approx((1.0, 2.0, 3.0))
    # 90° um x: z → -y
    assert rotate((s, 0.0, 0.0, s), (0.0, 0.0, 1.0)) == pytest.approx((0.0, -1.0, 0.0))
