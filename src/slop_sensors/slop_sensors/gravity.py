"""Einbaulage eines Sensors aus der Schwerkraft, ohne ROS (damit testbar).

Ein ruhender Beschleunigungssensor misst die Gegenkraft zur Schwerkraft, also einen Vektor nach OBEN.
Steht die Basis (base_footprint) waagrecht, folgen daraus roll und pitch des Sensors. Die Drehung um die
Senkrechte (yaw) ändert diesen Vektor nicht und ist so nicht bestimmbar.

Winkel wie bei TF / static_transform_publisher: R = Rz(yaw) · Ry(pitch) · Rx(roll), Sensor in der Basis.
"""
import math


def roll_pitch_from_up(ax, ay, az):
    """Vektor nach oben im Sensorframe (Einheit egal) → (roll, pitch) [rad] des Sensors in der Basis."""
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-9:
        raise ValueError('Beschleunigung ist 0')
    ax, ay, az = ax / n, ay / n, az / n
    return math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))


def quat_from_rpy(roll, pitch, yaw):
    """(roll, pitch, yaw) → Quaternion (x, y, z, w), Reihenfolge wie tf2 setRPY."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def rotate(q, v):
    """Vektor v mit Quaternion q = (x, y, z, w) drehen."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 · (q_vec × v);  v' = v + w·t + q_vec × t
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class StillWindow:
    """Sammelt Beschleunigungen über `duration` s und liefert den Mittelwert, wenn der Sensor dabei stillstand.

    Still heißt: Die Einzelmessungen weichen im Mittel (RMS) höchstens `max_rms_deg` Grad von der mittleren
    Richtung ab und der Betrag schwankt um höchstens `max_norm_rel` (relativ). Bewegt sich der Sensor, wird
    das Fenster verworfen und neu begonnen.
    """

    def __init__(self, duration=2.0, max_rms_deg=2.0, max_norm_rel=0.03, min_samples=50):
        self.duration = duration
        self.max_rms_deg = max_rms_deg
        self.max_norm_rel = max_norm_rel
        self.min_samples = min_samples
        self.reset()

    def reset(self):
        self.t0 = None
        self.samples = []

    def add(self, t, a):
        """Messung (Zeit [s], (ax, ay, az)) hinzufügen.

        Rückgabe: None solange das Fenster nicht voll ist, sonst (still, mittelwert, rms_grad).
        Nach einer Rückgabe beginnt ein neues Fenster.
        """
        if self.t0 is None:
            self.t0 = t
        self.samples.append(a)
        if t - self.t0 < self.duration or len(self.samples) < self.min_samples:
            return None
        result = self.evaluate(self.samples)
        self.reset()
        return result

    def evaluate(self, samples):
        k = len(samples)
        mean = tuple(sum(s[i] for s in samples) / k for i in range(3))
        mn = math.sqrt(sum(c * c for c in mean))
        if mn < 1e-9:
            return False, mean, 180.0
        sq_angle = 0.0
        norms = []
        for s in samples:
            sn = math.sqrt(sum(c * c for c in s))
            norms.append(sn)
            if sn < 1e-9:
                sq_angle += math.pi ** 2
                continue
            cos = sum(s[i] * mean[i] for i in range(3)) / (sn * mn)
            sq_angle += math.acos(max(-1.0, min(1.0, cos))) ** 2
        rms_deg = math.degrees(math.sqrt(sq_angle / k))
        avg_norm = sum(norms) / k
        norm_rel = math.sqrt(sum((n - avg_norm) ** 2 for n in norms) / k) / avg_norm
        still = rms_deg <= self.max_rms_deg and norm_rel <= self.max_norm_rel
        return still, mean, rms_deg
