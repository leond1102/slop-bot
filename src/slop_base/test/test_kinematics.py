"""Kinematik und Odometrie des Differentialantriebs, ohne Hardware und ohne ROS."""
import math

import pytest

from slop_base.kinematics import Odometry2D, limit_wheel_speeds, wheel_speeds

TRACK = 0.24
R = 0.035


def test_straight():
    wl, wr = wheel_speeds(0.035, 0.0, TRACK, R)
    assert wl == pytest.approx(1.0) and wr == pytest.approx(1.0)


def test_turn_in_place_left():
    wl, wr = wheel_speeds(0.0, 1.0, TRACK, R)
    assert wl == pytest.approx(-TRACK / 2 / R) and wr == pytest.approx(TRACK / 2 / R)


def test_limit_keeps_curve():
    wl, wr = wheel_speeds(0.5, 0.5, TRACK, R)
    wl2, wr2, k = limit_wheel_speeds(wl, wr, 1.0)
    assert max(abs(wl2), abs(wr2)) == pytest.approx(1.0)
    assert wl2 / wr2 == pytest.approx(wl / wr)
    assert 0 < k < 1


def test_limit_untouched_below_max():
    assert limit_wheel_speeds(0.5, -0.3, 1.0) == (0.5, -0.3, 1.0)


def test_odometry_straight_then_turn():
    odom = Odometry2D(TRACK)
    for _ in range(100):
        odom.update(0.01, 0.01)                 # 1 m geradeaus
    assert odom.x == pytest.approx(1.0) and odom.y == pytest.approx(0.0)
    quarter = math.pi / 2 * TRACK / 2           # 90° links auf der Stelle
    for _ in range(100):
        odom.update(-quarter / 100, quarter / 100)
    assert odom.yaw == pytest.approx(math.pi / 2)
    assert odom.x == pytest.approx(1.0) and odom.y == pytest.approx(0.0, abs=1e-12)


def test_odometry_full_circle_returns_home():
    odom = Odometry2D(TRACK)
    radius = 0.5                                 # Kreis um einen Punkt links vom Roboter
    n = 1000
    arc = 2 * math.pi / n
    for _ in range(n):
        odom.update(arc * (radius - TRACK / 2), arc * (radius + TRACK / 2))
    assert odom.x == pytest.approx(0.0, abs=1e-6) and odom.y == pytest.approx(0.0, abs=1e-6)
    assert abs(odom.yaw) < 1e-9 or abs(abs(odom.yaw) - 2 * math.pi) < 1e-9
