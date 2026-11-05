"""Globale Relokalisierung in einer 2D-Karte, ohne ROS (testbar).

Maß für "Pose passt": Anteil der Scanpunkte, die in der Karte auf einer Wand liegen (belegte Zelle, mit `tolerance`
aufgeweitet). Am Slop-Bot gemessen: richtige Pose 90–97 %, nach dem Versetzen 48 %.

Suche über die ganze Karte in zwei Stufen:
  1. grob: jede `coarse_step`-te freie Zelle, Gier in `coarse_yaw_deg`-Schritten, mit großzügiger Toleranz
  2. fein: um die besten Kandidaten ±`refine_xy` m und ±`refine_yaw_deg`° in kleinen Schritten, mit der normalen Toleranz
Eindeutig ist ein Ergebnis nur, wenn der beste Kandidat mindestens `min_score` erreicht und um `min_margin` vor dem
besten Kandidaten an einem *anderen* Ort liegt (weiter als `distinct_xy` m oder `distinct_yaw_deg`° entfernt).
"""
import math
from dataclasses import dataclass

import numpy as np


def dilate(mask, radius):
    """Belegte Zellen um `radius` Zellen (Kreis) aufweiten, ohne Umlauf an den Rändern."""
    if radius <= 0:
        return mask.copy()
    h, w = mask.shape
    pad = np.pad(mask, radius)
    out = np.zeros_like(mask)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                out |= pad[radius + dy:radius + dy + h, radius + dx:radius + dx + w]
    return out


@dataclass
class Candidate:
    score: float
    x: float
    y: float
    yaw: float           # rad

    def distance_to(self, other):
        dyaw = abs((self.yaw - other.yaw + math.pi) % (2 * math.pi) - math.pi)
        return math.hypot(self.x - other.x, self.y - other.y), math.degrees(dyaw)


class MapModel:
    """Belegungskarte (Werte wie nav_msgs/OccupancyGrid: -1 unbekannt, 0..100) mit vorberechneten Wandmasken."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y, tolerance=0.10, coarse_tolerance=0.20):
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.res, self.ox, self.oy = float(resolution), float(origin_x), float(origin_y)
        self.width, self.height = width, height
        occupied = grid > 50
        self.free = (grid >= 0) & (grid <= 50)
        self.wall = dilate(occupied, int(round(tolerance / self.res)))
        self.wall_coarse = dilate(occupied, int(round(coarse_tolerance / self.res)))

    def _hits(self, mask, px, py):
        ix = np.floor((px - self.ox) / self.res).astype(np.int64)
        iy = np.floor((py - self.oy) / self.res).astype(np.int64)
        inside = (ix >= 0) & (iy >= 0) & (ix < self.width) & (iy < self.height)
        hit = np.zeros(px.shape, dtype=bool)
        hit[inside] = mask[iy[inside], ix[inside]]
        return hit

    def score(self, r, a, x, y, yaw, coarse=False):
        """Anteil der Scanpunkte (Abstand r, Winkel a im Roboterframe) auf einer Wand, wenn der Roboter bei x, y, yaw steht."""
        if len(r) == 0:
            return 0.0
        px = x + np.cos(yaw + a) * r
        py = y + np.sin(yaw + a) * r
        return float(self._hits(self.wall_coarse if coarse else self.wall, px, py).mean())

    def free_cells(self, step):
        iy, ix = np.where(self.free)
        sel = (iy % step == 0) & (ix % step == 0)
        return self.ox + (ix[sel] + 0.5) * self.res, self.oy + (iy[sel] + 0.5) * self.res


def scan_points(ranges, angle_min, angle_increment, min_range=0.2, max_range=15.0, max_points=120):
    """Gültige Scanpunkte (r, a), gleichmäßig auf höchstens max_points ausgedünnt."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_increment
    ok = np.isfinite(r) & (r >= min_range) & (r <= max_range)
    r, a = r[ok], a[ok]
    if len(r) > max_points:
        idx = np.linspace(0, len(r) - 1, max_points).astype(int)
        r, a = r[idx], a[idx]
    return r, a


def refine(model, r, a, cand, xy=0.25, xy_step=0.05, yaw_deg=8.0, yaw_step_deg=1.0):
    """Feinsuche um einen Kandidaten (normale Toleranz)."""
    best = Candidate(model.score(r, a, cand.x, cand.y, cand.yaw), cand.x, cand.y, cand.yaw)
    offsets = np.arange(-xy, xy + 1e-9, xy_step)
    for dyaw in np.radians(np.arange(-yaw_deg, yaw_deg + 1e-9, yaw_step_deg)):
        yaw = cand.yaw + dyaw
        c, s = np.cos(yaw + a), np.sin(yaw + a)
        for dx in offsets:
            for dy in offsets:
                x, y = cand.x + dx, cand.y + dy
                sc = float(model._hits(model.wall, x + c * r, y + s * r).mean())
                if sc > best.score:
                    best = Candidate(sc, x, y, yaw)
    best.yaw = (best.yaw + math.pi) % (2 * math.pi) - math.pi
    return best


def global_search(model, r, a, coarse_step=4, coarse_yaw_deg=5.0, keep=12, distinct_xy=0.5, distinct_yaw_deg=20.0):
    """Ganze Karte durchsuchen. Liefert nach Feinsuche sortierte, voneinander verschiedene Kandidaten (bester zuerst)."""
    cx, cy = model.free_cells(coarse_step)
    if len(cx) == 0 or len(r) == 0:
        return []
    found = []
    for yaw in np.radians(np.arange(0.0, 360.0, coarse_yaw_deg)):
        px = cx[:, None] + np.cos(yaw + a)[None, :] * r[None, :]
        py = cy[:, None] + np.sin(yaw + a)[None, :] * r[None, :]
        sc = model._hits(model.wall_coarse, px, py).mean(axis=1)
        for i in np.argsort(sc)[-keep:]:
            found.append(Candidate(float(sc[i]), float(cx[i]), float(cy[i]), float(yaw)))
    found.sort(key=lambda c: -c.score)
    distinct = []
    for c in found:                      # grobe Kandidaten an verschiedenen Orten
        if all(_different(c, d, distinct_xy, distinct_yaw_deg) for d in distinct):
            distinct.append(c)
        if len(distinct) >= keep:
            break
    refined = sorted((refine(model, r, a, c) for c in distinct), key=lambda c: -c.score)
    out = []                             # nach der Feinsuche können zwei auf demselben Ort gelandet sein
    for c in refined:
        if all(_different(c, d, distinct_xy, distinct_yaw_deg) for d in out):
            out.append(c)
    return out


def _different(c, d, distinct_xy, distinct_yaw_deg):
    dist, dyaw = c.distance_to(d)
    return dist > distinct_xy or dyaw > distinct_yaw_deg


def decide(candidates, min_score=0.80, min_margin=0.08):
    """(bester Kandidat oder None, Begründung)."""
    if not candidates:
        return None, 'keine Kandidaten (Karte oder Scan leer)'
    best = candidates[0]
    second = candidates[1].score if len(candidates) > 1 else 0.0
    if best.score < min_score:
        return None, f'bester Treffer nur {best.score:.0%} (< {min_score:.0%})'
    if best.score - second < min_margin:
        c = candidates[1]
        return None, (f'nicht eindeutig: {best.score:.0%} bei ({best.x:.2f}, {best.y:.2f}, {math.degrees(best.yaw):.0f}°) '
                      f'gegen {c.score:.0%} bei ({c.x:.2f}, {c.y:.2f}, {math.degrees(c.yaw):.0f}°)')
    return best, f'{best.score:.0%}, nächstbester Ort {second:.0%}'
