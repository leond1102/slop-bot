"""Kinematik des Differentialantriebs, ohne Hardware und ohne ROS."""
import pytest

from slop_base.kinematics import limit_wheel_speeds, wheel_speeds

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
