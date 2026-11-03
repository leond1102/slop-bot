"""Mathematik für mount_calib, ohne ROS."""
import math

import numpy as np
import pytest

from slop_slam.calib_math import (blend, camera_mount_from_icp, circular_mean, inv, matrix_to_quat,
                                  matrix_to_rpy, pose, quat_to_matrix, rotation_angle, rpy_to_matrix,
                                  yaw_from_motion)


@pytest.mark.parametrize('rpy', [(0.1, -0.2, 0.3), (-0.5, 0.4, -2.5), (3.0, 0.1, 1.0)])
def test_rpy_round_trip(rpy):
    assert matrix_to_rpy(rpy_to_matrix(*rpy)) == pytest.approx(rpy)


def test_quat_round_trip():
    R = rpy_to_matrix(0.3, -0.7, 2.9)
    assert quat_to_matrix(matrix_to_quat(R)) == pytest.approx(R)


def test_blend():
    A = pose(rpy_to_matrix(0, 0, 0), [0, 0, 0])
    B = pose(rpy_to_matrix(0, 0, 0.4), [1, 0, 0])
    H = blend(A, B, 0.25)
    assert matrix_to_rpy(H[:3, :3])[2] == pytest.approx(0.1)
    assert H[:3, 3] == pytest.approx([0.25, 0, 0])
    assert blend(A, B, 1.0) == pytest.approx(B)
    assert blend(A, A, 0.5) == pytest.approx(A)


def test_camera_mount_from_icp_recovers_truth():
    T_map_base = pose(rpy_to_matrix(0.01, -0.02, 1.2), [2.0, -1.0, 0.0])
    T_base_cam_true = pose(rpy_to_matrix(0.0, -0.15, 0.05), [0.12, 0.03, 0.25])
    T_base_cam_est = pose(rpy_to_matrix(0.0, 0.0, 0.0), [0.15, 0.0, 0.20])
    # ICP findet die Korrektur, die die geschätzte Kamerapose in map auf die wahre schiebt
    dT = (T_map_base @ T_base_cam_true) @ inv(T_map_base @ T_base_cam_est)
    assert camera_mount_from_icp(T_map_base, T_base_cam_est, dT) == pytest.approx(T_base_cam_true)


@pytest.mark.parametrize('yaw', [0.0, 0.3, -1.2, math.pi, 2.5])
@pytest.mark.parametrize('roll,pitch', [(0.0, 0.0), (-0.012, 0.023), (0.2, -0.15), (math.pi, 0.0)])
def test_yaw_from_motion(yaw, roll, pitch):
    R_base_sensor = rpy_to_matrix(roll, pitch, yaw)
    d_base = np.array([0.08, 0.0, 0.0])                  # 8 cm geradeaus
    d_sensor = R_base_sensor.T @ d_base
    est = yaw_from_motion(d_sensor, roll, pitch)
    assert abs(math.atan2(math.sin(est - yaw), math.cos(est - yaw))) < 1e-9


def test_circular_mean_wraps():
    assert circular_mean([math.pi - 0.1, -math.pi + 0.1], [1, 1]) == pytest.approx(math.pi, abs=1e-9) or \
        circular_mean([math.pi - 0.1, -math.pi + 0.1], [1, 1]) == pytest.approx(-math.pi, abs=1e-9)
    assert circular_mean([0.1, 0.3], [1, 3]) == pytest.approx(0.25, abs=0.01)


def test_rotation_angle():
    assert rotation_angle(rpy_to_matrix(0, 0, 0.5)) == pytest.approx(0.5)
