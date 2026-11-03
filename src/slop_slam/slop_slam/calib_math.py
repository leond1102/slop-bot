"""Mathematik für mount_calib: Montage der Sensoren aus LIO nachführen, ohne ROS (damit testbar).

Rotationen als 3×3-Matrizen (numpy), Posen als 4×4. Winkel wie TF: R = Rz(yaw)·Ry(pitch)·Rx(roll).
"""
import math

import numpy as np


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_matrix(roll, pitch, yaw):
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 0.999999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                   # Gimbal Lock: yaw beliebig wählen
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def pose(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    return pose(R.T, -R.T @ t)


def rotation_angle(R):
    return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))


def blend(T_old, T_new, alpha):
    """Pose um den Anteil alpha (0..1) von T_old Richtung T_new schieben (Translation linear, Drehung entlang
    der kürzesten Achse)."""
    dR = T_old[:3, :3].T @ T_new[:3, :3]
    ang = rotation_angle(dR)
    if ang < 1e-9:
        R = T_old[:3, :3]
    else:
        axis = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * math.sin(ang))
        a = alpha * ang
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        R = T_old[:3, :3] @ (np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K)
    t = (1 - alpha) * T_old[:3, 3] + alpha * T_new[:3, 3]
    return pose(R, t)


def camera_mount_from_icp(T_map_base, T_base_cam_old, dT_map):
    """ICP-Korrektur in map (dT_map · T_map_cam_geschätzt = T_map_cam_wahr) → neue Montage base → Kamera.

    T_map_cam = T_map_base · T_base_cam  ⇒  T_base_cam_neu = T_map_base⁻¹ · dT · T_map_base · T_base_cam_alt
    """
    return inv(T_map_base) @ dT_map @ T_map_base @ T_base_cam_old


def yaw_from_motion(d_sensor, roll, pitch):
    """Geradeausfahrt (vorwärts) als Verschiebung im Sensorframe → yaw der Sensormontage in der Basis.

    Ein Differentialantrieb fährt nur entlang seiner x-Achse. Mit der Montage R = Rz(yaw)·Ry(pitch)·Rx(roll)
    ist die Fahrtrichtung im Sensor R⁻¹·(1,0,0). Roll/pitch herausrechnen: Ry(pitch)·Rx(roll)·d = (cos yaw,
    -sin yaw, 0)·s  ⇒  yaw = atan2(-y, x).
    """
    v = rot_y(pitch) @ rot_x(roll) @ np.asarray(d_sensor, dtype=float)
    return math.atan2(-v[1], v[0])


def circular_mean(angles, weights):
    s = sum(w * math.sin(a) for a, w in zip(angles, weights))
    c = sum(w * math.cos(a) for a, w in zip(angles, weights))
    return math.atan2(s, c)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))
