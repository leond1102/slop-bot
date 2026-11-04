"""Differentialantrieb: Kinematik und Radodometrie, ohne ROS.

Koordinaten (ROS): x vorne, y links, Drehung positiv gegen den Uhrzeigersinn.
Ursprung (base_footprint) = Mitte zwischen den angetriebenen Rädern am Boden.
Die frei schwenkenden Schleppräder spielen für die Kinematik keine Rolle.
"""
import math


def wheel_speeds(v, w, track, radius):
    """Twist (v [m/s], w [rad/s]) → Radwinkelgeschwindigkeit links, rechts [rad/s]."""
    return (v - w * track / 2.0) / radius, (v + w * track / 2.0) / radius


def limit_wheel_speeds(w_left, w_right, w_max):
    """Beide Räder gleich stark herunterskalieren, falls eins über w_max liegt (Kurvenradius bleibt)."""
    worst = max(abs(w_left), abs(w_right))
    if worst <= w_max or worst == 0.0:
        return w_left, w_right, 1.0
    k = w_max / worst
    return w_left * k, w_right * k, k


class Odometry2D:
    """Integriert Radwege zu Pose (x, y, yaw) im odom-Frame."""

    def __init__(self, track):
        self.track = track
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def update(self, d_left, d_right):
        """Radwege seit dem letzten Aufruf [m]. Gibt (Weg, Drehwinkel) des Roboters zurück."""
        ds = (d_left + d_right) / 2.0
        dyaw = (d_right - d_left) / self.track
        mid = self.yaw + dyaw / 2.0     # Mittelpunktregel: genauer bei Kurven
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        return ds, dyaw
