"""IMU-Aufbereitung für den EKF, ohne ROS (damit testbar).

Livox Mid-360s: Winkelgeschwindigkeit in rad/s, Beschleunigung in g, keine Kovarianzen, Orientierung 0 (ungültig).
robot_localization erwartet m/s² und eine Kovarianz je Messgröße; ohne sie vertraut der Filter der IMU blind.
"""
import math

G = 9.80665          # m/s² je g


def accel_g_to_ms2(a_g, g=G):
    """Beschleunigung (x, y, z) von g nach m/s²."""
    return tuple(a * g for a in a_g)


def diag_covariance(stddev):
    """3×3-Kovarianz (zeilenweise, 9 Werte) mit gleicher Standardabweichung je Achse."""
    var = stddev * stddev
    return [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]


class GyroBias:
    """Mittelwert des Gyros über ein Zeitfenster, solange der Roboter still steht.

    add() sammelt Messungen. Nach `duration` Sekunden ist das Fenster voll: Ist die Streuung klein genug, gilt der
    Mittelwert als Bias (ready), sonst verworfen und neu begonnen (der Roboter hat sich bewegt). Ist ein Bias da, wird
    nichts mehr gesammelt, bis restart() ihn neu anfordert (sonst würde eine gleichmäßige Drehung als Bias gelten)."""

    def __init__(self, duration, max_std):
        self.duration = duration
        self.max_std = max_std
        self.bias = None            # (x, y, z) oder None, solange nicht kalibriert
        self.rejected = 0
        self._active = True         # Messfenster läuft
        self._reset()

    def _reset(self):
        self._t0 = None
        self._n = 0
        self._sum = [0.0, 0.0, 0.0]
        self._sq = [0.0, 0.0, 0.0]

    def restart(self):
        """Neu kalibrieren (z. B. per Service); bis zum Ende des Fensters gilt der alte Bias weiter."""
        self._active = True
        self._reset()

    def set(self, bias):
        """Bias von außen setzen (z. B. aus calib/gyro.yaml): sofort bereit, ohne Messfenster."""
        self.bias = tuple(float(v) for v in bias)
        self._active = False
        self._reset()

    @property
    def ready(self):
        return self.bias is not None

    def add(self, t, gyro):
        """Messung (t in s, gyro = (x, y, z) in rad/s). Gibt True zurück, wenn dabei ein neuer Bias entstand."""
        if not self._active:
            return False
        if self._t0 is None:
            self._t0 = t
        self._n += 1
        for i, v in enumerate(gyro):
            self._sum[i] += v
            self._sq[i] += v * v
        if t - self._t0 < self.duration:
            return False
        mean = [s / self._n for s in self._sum]
        std = [math.sqrt(max(0.0, q / self._n - m * m)) for q, m in zip(self._sq, mean)]
        self._reset()
        if max(std) > self.max_std:
            self.rejected += 1
            return False
        self.bias = tuple(mean)
        self._active = False
        return True

    def correct(self, gyro):
        if self.bias is None:
            return tuple(gyro)
        return tuple(v - b for v, b in zip(gyro, self.bias))
