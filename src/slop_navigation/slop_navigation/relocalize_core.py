"""Globale Relokalisierung in einer 2D-Karte, ohne ROS (testbar).

Maß für "Pose passt": Anteil der Scanpunkte, die auf FESTER Struktur liegen (mit `tolerance` aufgeweitet). Am Slop-Bot
gemessen (nur Karte, ohne Erfahrung): richtige Pose 90–97 %, nach dem Versetzen 48 %.

Referenz ist nur, was sich nicht bewegt. Mit der Erfahrungskarte (/experience/known: 100 fest, 50 vielleicht, 0 frei,
-1 keine Meinung) teilt experience_core.structure die Zellen ein (dieselbe Regel wie Sicherheitsfeld und Export):
  fest        Erfahrung „fest“, oder Kartenwand, die weder beweglich noch verschwunden ist → Treffer
  beweglich   Erfahrung „vielleicht“ (Stuhl, Mülleimer – auch wenn er beim Kartieren dort stand) → neutral: ein
              Scanpunkt dort zählt weder als Treffer noch als Fehler (steht der Stuhl gerade da oder nicht – beides ok)
  sonst       Fehler (Scanpunkt mitten im freien Raum: falsche Pose oder etwas Unbekanntes, z. B. ein Mensch)
Eine Kartenwand gilt erst als verschwunden, wenn im Umkreis von 20 cm nichts belegt war – Karte und Scans liegen oft
5–20 cm gegeneinander versetzt.
Daraus zwei Werte je Pose:
  rank   Treffer / alle Punkte. Entscheidet, WELCHE Pose gewinnt (Suche, Feinsuche, Eindeutigkeit). Bewegliches hilft hier
         keiner Pose: Ein Stuhl ist wie jedes andere Hindernis ein Ausreißer, er kann die Suche weder anziehen noch einen
         symmetrischen Raum eindeutig machen.
  score  Treffer / (Punkte − neutrale), höchstens die Hälfte der Punkte darf neutral herausfallen. Entscheidet, OB die
         Pose gut genug ist (min_score, bad_score in relocalize): Ein Stuhl an seinem üblichen Platz macht die Pose nicht
         schlechter, ein voller Raum löst keine falsche Suche aus. Die Grenze verhindert, dass eine Pose, deren Scan fast
         nur auf Stuhl-Stellen fällt, mit wenigen Treffern perfekt aussieht.
Ohne Erfahrung (oder ohne Meinung) sind beide gleich der alten Bewertung „Anteil der Punkte auf belegten Kartenzellen“.

Suche über die ganze Karte in zwei Stufen:
  1. grob: jede `coarse_step`-te freie Zelle, Gier in `coarse_yaw_deg`-Schritten, mit großzügiger Toleranz
  2. fein: um die besten Kandidaten ±`refine_xy` m und ±`refine_yaw_deg`° in kleinen Schritten, mit der normalen Toleranz
Eindeutig ist ein Ergebnis nur, wenn der beste Kandidat mindestens `min_score` erreicht und um `min_margin` vor dem
besten Kandidaten an einem *anderen* Ort liegt (weiter als `distinct_xy` m oder `distinct_yaw_deg`° entfernt).
"""
import math
from dataclasses import dataclass

import numpy as np

from slop_navigation.experience_core import structure


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
    score: float         # Güte (bewegliche Stellen neutral) – reicht die Pose?
    x: float
    y: float
    yaw: float           # rad
    rank: float = None   # nur feste Struktur – welche Pose gewinnt? (None = wie score)

    @property
    def key(self):
        return self.score if self.rank is None else self.rank

    def distance_to(self, other):
        dyaw = abs((self.yaw - other.yaw + math.pi) % (2 * math.pi) - math.pi)
        return math.hypot(self.x - other.x, self.y - other.y), math.degrees(dyaw)


class MapModel:
    """Belegungskarte (Werte wie nav_msgs/OccupancyGrid: -1 unbekannt, 0..100) mit vorberechneten Masken für feste
    Struktur (Treffer) und bewegliche Stellen (neutral). experience: Raster wie /experience/known, gleiche Größe."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y, tolerance=0.10, coarse_tolerance=0.20,
                 experience=None, min_counted=0.5):
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.res, self.ox, self.oy = float(resolution), float(origin_x), float(origin_y)
        self.width, self.height = width, height
        self.min_counted = float(min_counted)
        occupied = grid > 50
        self.free = (grid >= 0) & (grid <= 50)
        exp = None if experience is None else np.asarray(experience, dtype=np.int16).reshape(height, width)
        fixed, maybe, self.stats = structure(occupied, exp)      # stats: Zahlen fürs Log
        if exp is not None:
            self.free |= exp == 0                        # inzwischen erkundete freie Fläche: mögliche Standorte
        t, ct = int(round(tolerance / self.res)), int(round(coarse_tolerance / self.res))
        self.wall, self.wall_coarse = dilate(fixed, t), dilate(fixed, ct)
        self.has_neutral = bool(maybe.any())
        self.neutral = dilate(maybe, t) & ~self.wall
        self.neutral_coarse = dilate(maybe, ct) & ~self.wall_coarse

    def _hits(self, mask, px, py):
        ix = np.floor((px - self.ox) / self.res).astype(np.int64)
        iy = np.floor((py - self.oy) / self.res).astype(np.int64)
        inside = (ix >= 0) & (iy >= 0) & (ix < self.width) & (iy < self.height)
        hit = np.zeros(px.shape, dtype=bool)
        hit[inside] = mask[iy[inside], ix[inside]]
        return hit

    def score_points(self, px, py, coarse=False, excuse=True):
        """Bewertung von Scanpunkten in Weltkoordinaten; die letzte Achse sind die Punkte eines Scans (für die
        Suche auch mehrere Kandidaten auf einmal: Form (Kandidaten, Punkte) → je Kandidat ein Wert).
        excuse=True: score (bewegliche Stellen neutral), False: rank (nur feste Struktur)."""
        n = px.shape[-1]
        if n == 0:
            return np.zeros(px.shape[:-1]) if px.ndim > 1 else 0.0
        hits = self._hits(self.wall_coarse if coarse else self.wall, px, py).sum(axis=-1)
        if excuse and self.has_neutral:
            neutral = self._hits(self.neutral_coarse if coarse else self.neutral, px, py).sum(axis=-1)
            counted = np.maximum(n - neutral, self.min_counted * n)
        else:
            counted = n
        sc = np.minimum(hits / counted, 1.0)
        return sc if px.ndim > 1 else float(sc)

    def score(self, r, a, x, y, yaw, coarse=False, excuse=True):
        """Güte (0..1) der Scanpunkte (Abstand r, Winkel a im Roboterframe), wenn der Roboter bei x, y, yaw steht.
        excuse=False: rank (nur feste Struktur)."""
        if len(r) == 0:
            return 0.0
        return self.score_points(x + np.cos(yaw + a) * r, y + np.sin(yaw + a) * r, coarse, excuse)

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
    """Feinsuche um einen Kandidaten (normale Toleranz): beste Pose nach rank (nur feste Struktur), dazu ihre Güte."""
    bx, by, byaw = cand.x, cand.y, cand.yaw
    best_rank = model.score(r, a, bx, by, byaw, excuse=False)
    offsets = np.arange(-xy, xy + 1e-9, xy_step)
    for dyaw in np.radians(np.arange(-yaw_deg, yaw_deg + 1e-9, yaw_step_deg)):
        yaw = cand.yaw + dyaw
        c, s = np.cos(yaw + a), np.sin(yaw + a)
        for dx in offsets:
            for dy in offsets:
                x, y = cand.x + dx, cand.y + dy
                rk = model.score_points(x + c * r, y + s * r, excuse=False)
                if rk > best_rank:
                    best_rank, bx, by, byaw = rk, x, y, yaw
    byaw = (byaw + math.pi) % (2 * math.pi) - math.pi
    return Candidate(model.score(r, a, bx, by, byaw), bx, by, byaw, rank=best_rank)


def global_search(model, r, a, coarse_step=4, coarse_yaw_deg=5.0, keep=12, distinct_xy=0.5, distinct_yaw_deg=20.0):
    """Ganze Karte durchsuchen. Liefert nach Feinsuche sortierte, voneinander verschiedene Kandidaten (bester zuerst)."""
    cx, cy = model.free_cells(coarse_step)
    if len(cx) == 0 or len(r) == 0:
        return []
    found = []
    for yaw in np.radians(np.arange(0.0, 360.0, coarse_yaw_deg)):
        px = cx[:, None] + np.cos(yaw + a)[None, :] * r[None, :]
        py = cy[:, None] + np.sin(yaw + a)[None, :] * r[None, :]
        sc = model.score_points(px, py, coarse=True, excuse=False)     # Suche nur nach fester Struktur
        for i in np.argsort(sc)[-keep:]:
            found.append(Candidate(float(sc[i]), float(cx[i]), float(cy[i]), float(yaw), rank=float(sc[i])))
    found.sort(key=lambda c: -c.key)
    distinct = []
    for c in found:                      # grobe Kandidaten an verschiedenen Orten
        if all(_different(c, d, distinct_xy, distinct_yaw_deg) for d in distinct):
            distinct.append(c)
        if len(distinct) >= keep:
            break
    refined = sorted((refine(model, r, a, c) for c in distinct), key=lambda c: -c.key)
    out = []                             # nach der Feinsuche können zwei auf demselben Ort gelandet sein
    for c in refined:
        if all(_different(c, d, distinct_xy, distinct_yaw_deg) for d in out):
            out.append(c)
    return out


def _different(c, d, distinct_xy, distinct_yaw_deg):
    dist, dyaw = c.distance_to(d)
    return dist > distinct_xy or dyaw > distinct_yaw_deg


def decide(candidates, min_score=0.80, min_margin=0.08):
    """(bester Kandidat oder None, Begründung). Güte (score) muss min_score erreichen, der Vorsprung nach rank (nur feste
    Struktur) vor dem besten anderen Ort min_margin – Bewegliches kann eine Pose also nicht eindeutig machen."""
    if not candidates:
        return None, 'keine Kandidaten (Karte oder Scan leer)'
    candidates = sorted(candidates, key=lambda c: -c.key)
    best = candidates[0]
    second = candidates[1].key if len(candidates) > 1 else 0.0
    if best.score < min_score:
        return None, f'bester Treffer nur {best.score:.0%} (< {min_score:.0%})'
    if best.key - second < min_margin:
        c = candidates[1]
        return None, (f'nicht eindeutig: {best.key:.0%} bei ({best.x:.2f}, {best.y:.2f}, {math.degrees(best.yaw):.0f}°) '
                      f'gegen {c.key:.0%} bei ({c.x:.2f}, {c.y:.2f}, {math.degrees(c.yaw):.0f}°) (feste Struktur)')
    return best, f'{best.score:.0%}, nächstbester Ort {second:.0%}'
