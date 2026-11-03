"""Starre Transformationen (Drehung als Quaternion + Verschiebung), ohne ROS (damit testbar).

T_a_b = Pose von Frame b in Frame a:  p_a = T_a_b * p_b.  Quaternion (x, y, z, w).
"""
import math


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def q_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Transform:
    __slots__ = ('q', 't')

    def __init__(self, q=(0.0, 0.0, 0.0, 1.0), t=(0.0, 0.0, 0.0)):
        self.q = q_normalize(tuple(float(c) for c in q))
        self.t = tuple(float(c) for c in t)

    def __mul__(self, other):
        """self ∘ other: erst other, dann self (T_a_b * T_b_c = T_a_c)."""
        rt = q_rotate(self.q, other.t)
        return Transform(q_mul(self.q, other.q), (rt[0] + self.t[0], rt[1] + self.t[1], rt[2] + self.t[2]))

    def inverse(self):
        qi = q_conj(self.q)
        ti = q_rotate(qi, self.t)
        return Transform(qi, (-ti[0], -ti[1], -ti[2]))

    def apply(self, p):
        r = q_rotate(self.q, p)
        return (r[0] + self.t[0], r[1] + self.t[1], r[2] + self.t[2])

    def close_to(self, other, tol=1e-6):
        dq = min(sum(abs(a - b) for a, b in zip(self.q, other.q)),
                 sum(abs(a + b) for a, b in zip(self.q, other.q)))   # q und -q sind dieselbe Drehung
        return dq < tol and sum(abs(a - b) for a, b in zip(self.t, other.t)) < tol

    def __repr__(self):
        return f'Transform(q={self.q}, t={self.t})'


def lio_to_map(T_base_imu, T_ci_body):
    """FAST-LIO-Pose (IMU-Frame `body` im Startframe `camera_init`) → Roboterpose im map-Frame.

    map ist so festgelegt, dass base_footprint beim Start im Ursprung liegt:
        T_map_ci = T_base_imu        (camera_init = IMU-Lage beim Start)
        T_map_base = T_map_ci * T_ci_body * T_body_base,   T_body_base = T_base_imu⁻¹
    Rückgabe: (T_map_ci, T_map_base)
    """
    return T_base_imu, T_base_imu * T_ci_body * T_base_imu.inverse()
