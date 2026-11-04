"""Transformationen für lio_tf, ohne ROS."""
import math

import pytest

from slop_slam.transforms import Transform, body_twist, lio_to_map, predict, q_scale, yaw_of


def rpy_q(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def test_inverse():
    T = Transform(rpy_q(0.3, -0.2, 1.1), (1.0, -2.0, 0.5))
    assert (T * T.inverse()).close_to(Transform())
    assert (T.inverse() * T).close_to(Transform())


def test_compose_matches_point_mapping():
    A = Transform(rpy_q(0.1, 0.2, 0.3), (1, 2, 3))
    B = Transform(rpy_q(-0.4, 0.5, -0.6), (-1, 0.5, 2))
    p = (0.3, -0.7, 1.2)
    assert (A * B).apply(p) == pytest.approx(A.apply(B.apply(p)))


def test_lio_start_is_origin():
    T_base_imu = Transform(rpy_q(-0.012, 0.022, 0.0), (0.011, 0.023, 0.256))
    _, T_map_base = lio_to_map(T_base_imu, Transform())
    assert T_map_base.close_to(Transform())


def test_lio_rotation_on_the_spot():
    # Roboter dreht sich 90° um seine Mitte; Lidar sitzt 0,2 m vor der Mitte, aufrecht
    T_base_imu = Transform(t=(0.2, 0.0, 0.3))
    yaw = math.pi / 2
    # IMU bewegt sich im camera_init-Frame auf einem Kreis um die Robotermitte
    T_ci_body = Transform(rpy_q(0, 0, yaw), (0.2 * math.cos(yaw) - 0.2, 0.2 * math.sin(yaw), 0.0))
    _, T_map_base = lio_to_map(T_base_imu, T_ci_body)
    assert T_map_base.t == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert yaw_of(T_map_base.q) == pytest.approx(yaw)


def test_lio_upside_down_lidar_keeps_robot_yaw_sign():
    # Lidar kopfüber (roll π): Dreht der Roboter links, dreht die IMU um ihre z-Achse rechts
    T_base_imu = Transform(rpy_q(math.pi, 0, 0), (0, 0, 0.3))
    T_ci_body = Transform(rpy_q(0, 0, -0.5))
    _, T_map_base = lio_to_map(T_base_imu, T_ci_body)
    assert yaw_of(T_map_base.q) == pytest.approx(0.5)
    assert T_map_base.t == pytest.approx((0, 0, 0), abs=1e-9)


def arc_pose(t, v=0.03, w=0.3):
    """Gleichmäßige Kreisfahrt ab dem Ursprung: v [m/s] vorwärts, w [rad/s] links."""
    yaw = w * t
    return Transform(rpy_q(0, 0, yaw), (v / w * math.sin(yaw), v / w * (1 - math.cos(yaw)), 0.0))


def test_q_scale():
    q = rpy_q(0.0, 0.0, 0.8)
    assert yaw_of(q_scale(q, 0.25)) == pytest.approx(0.2)
    assert yaw_of(q_scale(q, -1.0)) == pytest.approx(-0.8)
    assert Transform(q_scale(q, 0.0)).close_to(Transform())


def test_predict_arc_exact_for_equal_step():
    T = predict(arc_pose(0.5), 0.5, arc_pose(1.0), 1.0, 1.5)
    assert T.close_to(arc_pose(1.5), 1e-9)


def test_predict_arc_short_horizon():
    # 0,15 s vorausgerechnet aus einem 0,5-s-Fenster: auf < 0,1 mm und < 0,01° genau
    T = predict(arc_pose(0.5), 0.5, arc_pose(1.0), 1.0, 1.15)
    ref = arc_pose(1.15)
    assert math.dist(T.t, ref.t) < 1e-4
    assert abs(yaw_of(T.q) - yaw_of(ref.q)) < math.radians(0.01)


def test_predict_standing_still():
    T0 = Transform(rpy_q(0.01, -0.02, 1.0), (1.0, 2.0, 0.1))
    assert predict(T0, 0.0, T0, 0.5, 0.7).close_to(T0)


def test_body_twist_arc():
    v, w = body_twist(arc_pose(1.0), 1.0, arc_pose(1.1), 1.1)
    assert v[0] == pytest.approx(0.03, rel=1e-2) and abs(v[2]) < 1e-12
    assert w == pytest.approx((0.0, 0.0, 0.3))
