"""Posenrechnung der LIO-Brücke, ohne ROS (damit testbar). Posen sind 4×4-Matrizen (numpy).

Koordinaten (ROS): x vorne, y links, z oben; Drehung positiv gegen den Uhrzeigersinn.
"""
import math

import numpy as np


def to_matrix(x, y, z, qx, qy, qz, qw):
    """Pose aus Position und Quaternion (x, y, z, w) als 4×4-Matrix."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    t = np.eye(4)
    t[:3, :3] = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                 [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                 [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]
    t[:3, 3] = (x, y, z)
    return t


def to_pose(t):
    """4×4-Matrix → (x, y, z, qx, qy, qz, qw)."""
    r = t[:3, :3]
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw, qx, qy, qz = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        qw, qx, qy, qz = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        qw, qx, qy, qz = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        qw, qx, qy, qz = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    return (*t[:3, 3], qx, qy, qz, qw)


def inverse(t):
    inv = np.eye(4)
    inv[:3, :3] = t[:3, :3].T
    inv[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return inv


def yaw_of(t):
    """Gierwinkel [rad] (Drehung um z) einer Pose."""
    return math.atan2(t[1, 0], t[0, 0])


def planar_inverse(t):
    """Inverse nur von Position (x, y, z) und Gier der Pose; Roll/Pitch bleiben unberücksichtigt.

    Damit wird die Anfangslage des Roboters zum Ursprung der Karte (map), ohne die Schwerkraftausrichtung der
    LIO-Welt zu verdrehen."""
    c, s = math.cos(yaw_of(t)), math.sin(yaw_of(t))
    p = np.eye(4)
    p[:2, :2] = [[c, -s], [s, c]]
    p[:3, 3] = t[:3, 3]
    return inverse(p)


def body_twist(t0, t1, dt):
    """Geschwindigkeit im Roboterframe (vx, vy, wz) aus zwei Posen im Abstand dt [s]."""
    d = inverse(t0) @ t1
    return d[0, 3] / dt, d[1, 3] / dt, math.atan2(d[1, 0], d[0, 0]) / dt


def with_translation(xyz):
    """Reine Verschiebung als 4×4-Matrix."""
    t = np.eye(4)
    t[:3, 3] = xyz
    return t


def planar_map_to_odom(map_to_base, odom_to_base):
    """Transformation map → odom nur in der Ebene (x, y, Gier), z = 0, Roll/Pitch 0.

    Aus der Pose von base_footprint in map (LIO) und in odom (EKF): Die EKF-Odometrie ist eben, die LIO-Pose nicht;
    Roll/Pitch/z des LIO sollen den odom-Frame nicht kippen."""
    dyaw = yaw_of(map_to_base) - yaw_of(odom_to_base)
    c, s = math.cos(dyaw), math.sin(dyaw)
    bx, by = map_to_base[0, 3], map_to_base[1, 3]
    ox, oy = odom_to_base[0, 3], odom_to_base[1, 3]
    t = np.eye(4)
    t[:2, :2] = [[c, -s], [s, c]]
    t[0, 3] = bx - (c * ox - s * oy)
    t[1, 3] = by - (s * ox + c * oy)
    return t
