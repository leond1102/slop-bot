"""Messfenster für ruhende IMU-Kalibrierungen und die zugehörige Rechnung, ohne ROS.

Roboter steht still und waagrecht: Das Gyro misst dann nur seinen Bias, der Beschleunigungsmesser nur die Schwerkraft
(daraus Roll und Pitch des Sensors).
"""
import math

G = 9.80665          # m/s²


class StillStats:
    """Mittelwert und Streuung von Gyro (rad/s) und Beschleunigung (m/s²) über ein Zeitfenster."""

    def __init__(self):
        self.n = 0
        self.t0 = None
        self.t = None
        self._s = [[0.0] * 3, [0.0] * 3]       # Summen: gyro, accel
        self._q = [[0.0] * 3, [0.0] * 3]       # Quadratsummen

    def add(self, t, gyro, accel):
        if self.t0 is None:
            self.t0 = t
        self.t = t
        self.n += 1
        for k, vec in enumerate((gyro, accel)):
            for i, v in enumerate(vec):
                self._s[k][i] += v
                self._q[k][i] += v * v

    @property
    def duration(self):
        return 0.0 if self.t0 is None else self.t - self.t0

    def _mean_std(self, k):
        mean = [s / self.n for s in self._s[k]]
        std = [math.sqrt(max(0.0, q / self.n - m * m)) for q, m in zip(self._q[k], mean)]
        return mean, std

    def result(self):
        """(gyro_mean, gyro_std, accel_mean, accel_std); braucht mindestens einen Wert."""
        gm, gs = self._mean_std(0)
        am, asd = self._mean_std(1)
        return gm, gs, am, asd


def tilt_from_gravity(accel):
    """Roll und Pitch [rad] eines Sensors aus der gemessenen Schwerkraft (x, y, z) im Sensorframe.

    Entspricht den Winkeln rpy im URDF (Drehung fest um x, y, z): Sensor waagrecht → 0. Gier ist so nicht messbar."""
    ax, ay, az = accel
    return math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))


def rotate(q, v):
    """Vektor v mit dem Quaternion q = (x, y, z, w) drehen."""
    x, y, z, w = q
    vx, vy, vz = v
    # v' = v + 2w (u × v) + 2 u × (u × v), u = (x, y, z)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    dx, dy, dz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + dx), vy + 2 * (w * cy + dy), vz + 2 * (w * cz + dz))
