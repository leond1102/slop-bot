"""Differentialantrieb: Kinematik, ohne ROS. (Keine Radodometrie mehr, die Pose kommt aus LIO.)

Koordinaten (ROS): x vorne, y links, Drehung positiv gegen den Uhrzeigersinn.
Ursprung (base_footprint) = Mitte zwischen den angetriebenen Rädern am Boden.
Die frei schwenkenden Schleppräder spielen für die Kinematik keine Rolle.
"""


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
