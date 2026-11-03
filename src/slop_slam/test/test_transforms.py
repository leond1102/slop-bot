"""Transformationen für lio_tf, ohne ROS."""
import math

import pytest

from slop_slam.transforms import Transform, lio_to_map, yaw_of


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


def test_map_to_odom_with_wheel_odometry():
    T_map_base = Transform(rpy_q(0, 0, 0.8), (1.0, 2.0, 0.0))
    T_odom_base = Transform(rpy_q(0, 0, 0.6), (0.9, 1.5, 0.0))      # Radodometrie driftet
    T_map_odom = T_map_base * T_odom_base.inverse()
    assert (T_map_odom * T_odom_base).close_to(T_map_base)
