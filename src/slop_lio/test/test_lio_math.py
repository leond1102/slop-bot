"""Posenrechnung der LIO-Brücke, ohne Hardware und ohne ROS."""
import math

import numpy as np
import pytest

from slop_lio.lio_math import (body_twist, inverse, planar_inverse, planar_map_to_odom, to_matrix, to_pose,
                               with_translation, yaw_of)


def _rot(yaw=0.0, pitch=0.0, roll=0.0, xyz=(0, 0, 0)):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return to_matrix(*xyz, qx, qy, qz, qw)


@pytest.mark.parametrize('yaw,pitch,roll', [(0.3, 0.0, 0.0), (-2.5, 0.1, -0.05), (3.0, -0.2, 0.2), (0.0, 0.0, 1.0)])
def test_matrix_pose_roundtrip(yaw, pitch, roll):
    t = _rot(yaw, pitch, roll, (1.0, -2.0, 0.5))
    assert np.allclose(to_matrix(*to_pose(t)), t, atol=1e-9)


def test_inverse():
    t = _rot(0.7, 0.1, -0.2, (1.0, 2.0, 3.0))
    assert np.allclose(t @ inverse(t), np.eye(4), atol=1e-12)


def test_yaw_of():
    assert yaw_of(_rot(0.4, 0.05, 0.02)) == pytest.approx(0.4, abs=1e-2)
    assert yaw_of(_rot(-3.0)) == pytest.approx(-3.0)


def test_planar_inverse_makes_start_the_origin():
    start = _rot(1.2, 0.02, -0.03, (4.0, -1.0, 0.3))        # Roboter startet schief in der LIO-Welt
    align = planar_inverse(start)
    at_start = align @ start
    assert at_start[:3, 3] == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert yaw_of(at_start) == pytest.approx(0.0, abs=1e-9)
    # Fährt der Roboter danach 1 m geradeaus, ist er in map bei x = 1
    moved = start @ with_translation((1.0, 0.0, 0.0))
    assert (align @ moved)[:2, 3] == pytest.approx((1.0, 0.0), abs=2e-2)


def test_twist_straight_and_turn():
    t0 = _rot(0.5, xyz=(1.0, 1.0, 0.0))
    t1 = t0 @ with_translation((0.02, 0.0, 0.0))               # 2 cm vor in 0,1 s → 0,2 m/s
    vx, vy, wz = body_twist(t0, t1, 0.1)
    assert (vx, vy, wz) == pytest.approx((0.2, 0.0, 0.0), abs=1e-9)
    t2 = t0 @ _rot(0.05)                                         # 0,05 rad in 0,1 s → 0,5 rad/s auf der Stelle
    vx, vy, wz = body_twist(t0, t2, 0.1)
    assert (vx, vy) == pytest.approx((0.0, 0.0), abs=1e-9) and wz == pytest.approx(0.5)


def test_twist_wraps_yaw():
    t0 = _rot(math.pi - 0.01)
    t1 = _rot(-math.pi + 0.01)                                   # über ±π hinweg: +0,02 rad
    assert body_twist(t0, t1, 0.1)[2] == pytest.approx(0.2, abs=1e-6)


def test_map_to_odom_consistent():
    """map_T_odom · odom_T_base muss (in der Ebene) wieder map_T_base ergeben."""
    map_to_base = _rot(1.0, xyz=(3.0, 2.0, 0.0))
    odom_to_base = _rot(0.4, xyz=(1.0, -0.5, 0.0))
    back = planar_map_to_odom(map_to_base, odom_to_base) @ odom_to_base
    assert back[:2, 3] == pytest.approx(map_to_base[:2, 3], abs=1e-9)
    assert yaw_of(back) == pytest.approx(yaw_of(map_to_base), abs=1e-9)


def test_map_to_odom_ignores_tilt_and_height():
    map_to_base = _rot(0.5, 0.1, -0.05, (1.0, 1.0, 0.3))      # LIO: etwas schief und höher
    odom_to_base = _rot(0.5, xyz=(1.0, 1.0, 0.0))             # EKF: eben
    m = planar_map_to_odom(map_to_base, odom_to_base)
    assert m[2, 3] == 0.0 and m[2, 2] == 1.0
    assert m[:2, 3] == pytest.approx((0.0, 0.0), abs=1e-9) and yaw_of(m) == pytest.approx(0.0, abs=1e-3)
