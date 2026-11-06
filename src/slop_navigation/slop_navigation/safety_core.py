"""Dynamisches Sicherheitsfeld ohne ROS: Scanpunkte in „bekannte Struktur“ und „unbekannt“ trennen.

Bekannte Struktur (Wand, Schrank, Tisch) darf der Roboter eng passieren; alles andere (Mensch, Stuhl, Kiste) bekommt
die größeren Zonen des zweiten Collision Monitors. Grundsatz: im Zweifel unbekannt. Ohne Karte, ohne sichere Lokalisierung,
außerhalb der Karte oder ohne Meinung gilt ein Punkt als unbekannt – das macht den Roboter vorsichtiger, nie mutiger.

Bekannt ist feste Struktur nach experience_core.structure (dieselbe Regel wie Relokalisierung und Export): Erfahrung
„fest“, oder Kartenwand, die weder beweglich (vielleicht, z. B. ein Stuhl, der beim Kartieren da stand) noch verschwunden
ist (frei und im Umkreis von 20 cm nichts). Danach um known_dilate (Lokalisierungsfehler) erweitert.
"""
import numpy as np

from slop_navigation.experience_core import near, structure

OCCUPIED_MIN = 65          # OccupancyGrid-Wert ab dem eine Kartenzelle belegt ist (map_server: 100)


def known_mask(width, height, map_data, exp_data=None, dilate_cells=2):
    """Bool-Raster (Zeile 0 = kleinstes y) der bekannten Struktur."""
    m = np.asarray(map_data, dtype=np.int16).reshape(height, width)
    e = None if exp_data is None else np.asarray(exp_data, dtype=np.int16).reshape(height, width)
    known, _, _ = structure(m >= OCCUPIED_MIN, e)
    return near(known, dilate_cells) if dilate_cells > 0 else known


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, max_range):
    """Endliche Scanpunkte bis max_range als (x, y) im Scan-Frame."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + np.arange(r.size) * angle_increment
    ok = np.isfinite(r) & (r >= range_min) & (r <= min(range_max, max_range))
    return r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])


def outside_body(x, y, x_min, x_max, half_width, margin):
    """Punkte außerhalb des Footprint-Rechtecks (+ margin): Reste des eigenen Körpers zählen nicht als Hindernis."""
    return ~((x >= x_min - margin) & (x <= x_max + margin) & (np.abs(y) <= half_width + margin))


def is_known(known, origin_x, origin_y, resolution, xs, ys):
    """Bool je Punkt (Weltkoordinaten in map): liegt auf bekannter Struktur. Außerhalb der Karte: unbekannt."""
    h, w = known.shape
    cx = np.floor((np.asarray(xs) - origin_x) / resolution).astype(np.int64)
    cy = np.floor((np.asarray(ys) - origin_y) / resolution).astype(np.int64)
    inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    out = np.zeros(cx.shape, dtype=bool)
    out[inside] = known[cy[inside], cx[inside]]
    return out


def transform(x, y, tx, ty, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y
