"""Erfahrungskarte ohne ROS: zählt je Kartenzelle, wie oft sie gesehen und wie oft dort etwas war.

Beständigkeit p = belegt / gesehen (0..1). Daraus drei Klassen:
  fest        p ≥ p_fixed       Wand, Schrank, Tisch – bei fast jedem Vorbeifahren da
  vielleicht  p_free ≤ p < p_fixed   Stuhl, Mülleimer, Rollcontainer – mal da, mal nicht
  frei        p < p_free        Gang
Zellen mit weniger als min_seen Beobachtungen haben keine Meinung.

Beide Zähler vergessen mit einer Halbwertszeit: Das Verhältnis p bleibt dabei gleich, aber neue Beobachtungen wiegen
schwerer, je länger die alten zurückliegen. So wandert ein dauerhaft weggeräumter Schrank nach einigen Tagen von „fest“
über „vielleicht“ nach „frei“.

Zusätzlich gilt für die laufende Sitzung: Eine Zelle, die zuletzt frei gesehen wurde, hat in der Maske keine Kosten mehr
(„erst mal schauen“). Steht dort wirklich etwas, trägt es die Sensorschicht der Costmap als Hindernis ein.

Raster wie nav_msgs/OccupancyGrid: Zeile 0 = kleinstes y, Index = Zeile * Breite + Spalte.
"""
import math
import os
import time

import numpy as np

UNKNOWN = -1                      # OccupancyGrid: keine Information
# 2 = Zellen nahe an Treffern werden nicht freigeräumt (ray_cells hit_protect). Mit Version 1 gelernte Zähler hielten
# die meisten Wände für „vielleicht“ und werden deshalb nicht weiterverwendet (experience legt sie als Sicherung ab).
FORMAT_VERSION = 2


class OldFormat(ValueError):
    """Zähler stammen von einem älteren Lernverfahren."""
PGM_OCCUPIED, PGM_FREE, PGM_UNKNOWN = 0, 254, 205   # Grauwerte von map_saver (trinary)


class ExperienceGrid:
    def __init__(self, width, height, resolution, origin_x, origin_y):
        self.width, self.height = int(width), int(height)
        self.resolution = float(resolution)
        self.origin_x, self.origin_y = float(origin_x), float(origin_y)
        n = self.width * self.height
        self.seen = np.zeros(n, dtype=np.float32)       # gewichtete Anzahl Beobachtungen
        self.hit = np.zeros(n, dtype=np.float32)        # davon belegt
        self.stamp = time.time()                        # Stand der Zähler (für das Vergessen)
        # laufende Sitzung: wann zuletzt frei / belegt gesehen (s seit Sitzungsbeginn, -1 = nie)
        self.session_free = np.full(n, -1.0, dtype=np.float64)
        self.session_hit = np.full(n, -1.0, dtype=np.float64)

    # ------------------------------------------------------------------ Geometrie
    def same_grid(self, width, height, resolution, origin_x, origin_y):
        return (self.width == int(width) and self.height == int(height)
                and abs(self.resolution - resolution) < 1e-6
                and abs(self.origin_x - origin_x) < 1e-3 and abs(self.origin_y - origin_y) < 1e-3)

    def cell_index(self, xs, ys):
        """Weltkoordinaten → Zellindex; -1 außerhalb der Karte."""
        cx = np.floor((np.asarray(xs) - self.origin_x) / self.resolution).astype(np.int64)
        cy = np.floor((np.asarray(ys) - self.origin_y) / self.resolution).astype(np.int64)
        inside = (cx >= 0) & (cx < self.width) & (cy >= 0) & (cy < self.height)
        return np.where(inside, cy * self.width + cx, -1)

    def ray_cells(self, x, y, yaw, ranges, angle_min, angle_increment, range_min, range_max,
                  max_range=6.0, inf_free_range=4.0, beam_step=1, hit_protect=2):
        """Strahlverfolgung eines 2D-Scans von der Pose (x, y, yaw) aus.

        Gibt (frei, belegt) als eindeutige Zellindizes zurück. Ein Treffer bis max_range markiert seine Zelle als belegt,
        die Zellen davor als frei. Treffer weiter weg räumen nur bis max_range frei (Winkelfehler wachsen mit dem
        Abstand). inf (kein Echo im Höhenband) räumt bis inf_free_range frei.
        Zellen bis hit_protect Zellen um einen Treffer DIESES Scans zählen nicht als frei: Sonst räumen Strahlen, die
        knapp neben einer Wand vorbeistreifen, und kleine Posenfehler die Wandzellen frei – eine Wand sähe dann aus
        wie ein Stuhl, der mal da ist und mal nicht (am Slop-Bot gemessen: so nur 123 statt 305 Wandzellen „fest“).
        """
        r = np.asarray(ranges, dtype=np.float64)[::beam_step]
        ang = yaw + angle_min + np.arange(0, len(ranges), beam_step)[:len(r)] * angle_increment
        finite = np.isfinite(r)
        valid = (finite & (r >= range_min) & (r <= range_max)) | (np.isposinf(r) & (inf_free_range > 0))
        r, ang, finite = r[valid], ang[valid], finite[valid]
        if r.size == 0:
            return np.empty(0, np.int64), np.empty(0, np.int64)

        is_hit = finite & (r <= max_range)
        free_len = np.where(finite, np.minimum(r, max_range), inf_free_range)
        free_len = np.where(is_hit, free_len - 0.5 * self.resolution, free_len)   # Trefferzelle nicht frei räumen

        cos_a, sin_a = np.cos(ang), np.sin(ang)
        hit_idx = self.cell_index(x + r[is_hit] * cos_a[is_hit], y + r[is_hit] * sin_a[is_hit])
        hit_idx = np.unique(hit_idx[hit_idx >= 0])

        step = 0.5 * self.resolution
        n_steps = int(math.ceil(max(free_len.max(), 0.0) / step))
        if n_steps == 0:
            return np.empty(0, np.int64), hit_idx
        d = np.arange(n_steps) * step                                    # Stützstellen entlang jedes Strahls
        inside = d[None, :] < free_len[:, None]
        px = x + d[None, :] * cos_a[:, None]
        py = y + d[None, :] * sin_a[:, None]
        free_idx = self.cell_index(px[inside], py[inside])
        free_idx = np.unique(free_idx[free_idx >= 0])
        free_idx = np.setdiff1d(free_idx, self.around(hit_idx, hit_protect), assume_unique=True)
        return free_idx, hit_idx

    def around(self, idx, radius):
        """Zellindizes idx samt Nachbarn bis radius Zellen (Quadrat), eindeutig, ohne Umlauf an den Rändern."""
        if radius <= 0 or idx.size == 0:
            return idx
        row, col = np.divmod(idx, self.width)
        d = np.arange(-radius, radius + 1)
        rr = (row[:, None, None] + d[None, :, None]).repeat(d.size, axis=2)
        cc = (col[:, None, None] + d[None, None, :]).repeat(d.size, axis=1)
        ok = (rr >= 0) & (rr < self.height) & (cc >= 0) & (cc < self.width)
        return np.unique(rr[ok] * self.width + cc[ok])

    # ------------------------------------------------------------------ Lernen
    def decay_to(self, now, half_life_s):
        """Zähler auf den Zeitpunkt now vergessen lassen (Halbwertszeit half_life_s; ≤ 0 = nie)."""
        dt = now - self.stamp
        if half_life_s > 0 and dt > 0:
            f = np.float32(0.5 ** (dt / half_life_s))
            self.seen *= f
            self.hit *= f
        self.stamp = max(self.stamp, now)

    def integrate(self, free_idx, hit_idx):
        """Eine Beobachtung in die Langzeit-Zähler übernehmen."""
        self.seen[free_idx] += 1.0
        self.seen[hit_idx] += 1.0
        self.hit[hit_idx] += 1.0

    def observe_session(self, free_idx, hit_idx, t):
        """Für die laufende Sitzung merken, was gerade frei bzw. belegt war (t in s, monoton)."""
        self.session_free[free_idx] = t
        self.session_hit[hit_idx] = t

    def reset_session(self):
        self.session_free.fill(-1.0)
        self.session_hit.fill(-1.0)

    # ------------------------------------------------------------------ Auswerten
    def persistence(self):
        """Beständigkeit p je Zelle (NaN ohne Beobachtung)."""
        with np.errstate(invalid='ignore', divide='ignore'):
            return np.where(self.seen > 0, self.hit / self.seen, np.nan)

    def classes(self, min_seen, p_free, p_fixed):
        """0 = keine Meinung, 1 = frei, 2 = vielleicht, 3 = fest."""
        p = self.persistence()
        known = self.seen >= min_seen
        c = np.zeros(self.seen.shape, dtype=np.int8)
        c[known & (p < p_free)] = 1
        c[known & (p >= p_free) & (p < p_fixed)] = 2
        c[known & (p >= p_fixed)] = 3
        return c

    def mask(self, min_seen, p_free, p_fixed, max_cost, live=True):
        """Maske für den Keepout-Filter (OccupancyGrid-Werte): -1 = keine Meinung, sonst 1..max_cost.

        Nav2 rechnet Wert v in Kosten v·2,54 um und übernimmt nur, was höher ist als die Costmap schon hat. Werte unter
        100 sind nie „verboten“, nur teurer. 0 wird bewusst nie gesendet: Der Filter würde damit unbekannte Kartenzellen
        auf frei setzen. live: Zellen, die in dieser Sitzung zuletzt frei gesehen wurden, bekommen keine Kosten.
        """
        p = self.persistence()
        m = np.full(self.seen.shape, UNKNOWN, dtype=np.int8)
        sel = (self.seen >= min_seen) & (p >= p_free)
        scale = np.clip((p[sel] - p_free) / max(p_fixed - p_free, 1e-6), 0.0, 1.0)
        m[sel] = np.clip(np.round(1 + scale * (max_cost - 1)), 1, 99).astype(np.int8)
        if live:
            m[self.session_free > self.session_hit] = UNKNOWN
        return m

    def known_grid(self, min_seen, p_free, p_fixed):
        """Für Sicherheitsfeld und Relokalisierung (/experience/known):
        100 = fest, 50 = vielleicht (mal belegt, z. B. Stuhl), 0 = frei, -1 = keine Meinung (zu selten gesehen)."""
        c = self.classes(min_seen, p_free, p_fixed)
        g = np.full(c.shape, UNKNOWN, dtype=np.int8)
        g[c == 3] = 100
        g[c == 2] = 50
        g[c == 1] = 0
        return g

    def persistence_grid(self, min_seen):
        """Anzeige: p in Prozent (0 weiß … 100 schwarz), -1 ohne Meinung."""
        p = self.persistence()
        g = np.full(self.seen.shape, UNKNOWN, dtype=np.int8)
        known = self.seen >= min_seen
        g[known] = np.round(np.clip(p[known], 0.0, 1.0) * 100).astype(np.int8)
        return g

    def summary(self, min_seen, p_free, p_fixed):
        c = self.classes(min_seen, p_free, p_fixed)
        return {'frei': int((c == 1).sum()), 'vielleicht': int((c == 2).sum()), 'fest': int((c == 3).sum())}

    # ------------------------------------------------------------------ Speichern
    def save(self, path):
        tmp = path + '.tmp.npz'
        np.savez_compressed(tmp, seen=self.seen, hit=self.hit, stamp=self.stamp, version=FORMAT_VERSION,
                            grid=np.array([self.width, self.height, self.resolution, self.origin_x, self.origin_y]))
        os.replace(tmp, path)

    @classmethod
    def load(cls, path):
        with np.load(path) as d:
            version = int(d['version']) if 'version' in d else 1
            if version != FORMAT_VERSION:
                raise OldFormat(f'Lernverfahren Version {version}, aktuell {FORMAT_VERSION}')
            w, h, res, ox, oy = d['grid']
            g = cls(int(w), int(h), res, ox, oy)
            g.seen = d['seen'].astype(np.float32)
            g.hit = d['hit'].astype(np.float32)
            g.stamp = float(d['stamp'])
        if g.seen.size != g.width * g.height or g.hit.size != g.seen.size:
            raise ValueError('Zählerlänge passt nicht zur Rastergröße')
        return g


# ---------------------------------------------------------------------- Feste Struktur (Relokalisierung, Sicherheitsfeld, Export)
KNOWN_FIXED, KNOWN_MAYBE, KNOWN_FREE = 100, 50, 0      # Werte von known_grid / /experience/known (-1 = keine Meinung)


def structure(occupied, known=None, gone_radius=4):
    """Was ist feste Struktur, was beweglich? Gemeinsame Regel für Relokalisierung, Sicherheitsfeld und Export.

    occupied: Bool-Raster der Kartenwände, known: Raster wie /experience/known (gleiche Form) oder None.
      fest       Erfahrung „fest“, oder Kartenwand, die weder beweglich noch verschwunden ist
      beweglich  Erfahrung „vielleicht“ (mal da, mal nicht – auch ein Sofa, das beim Kartieren dastand)
    Verschwunden ist eine Kartenwand erst, wenn die Erfahrung dort „frei“ sagt UND im Umkreis von gone_radius Zellen
    nichts belegt war (fest oder vielleicht). Grund: Karte und aktuelle Scans liegen oft 5–20 cm gegeneinander versetzt
    (am Slop-Bot gemessen); dann sieht die Erfahrung die Wand eine Zelle daneben, und die Kartenzelle selbst wirkt frei.
    Rückgabe (fest, beweglich, Zahlen).
    """
    if known is None:
        return occupied.copy(), np.zeros_like(occupied), {'fest': int(occupied.sum()), 'fest_aus_erfahrung': 0,
                                                          'karte_beweglich': 0, 'karte_verschwunden': 0, 'vielleicht': 0}
    known = np.asarray(known).reshape(occupied.shape)
    exp_fixed = known == KNOWN_FIXED
    maybe = known == KNOWN_MAYBE
    something_near = near(exp_fixed | maybe, gone_radius)
    gone = occupied & (known == KNOWN_FREE) & ~something_near
    fixed = exp_fixed | (occupied & ~maybe & ~gone)
    stats = {'fest': int(fixed.sum()), 'fest_aus_erfahrung': int((exp_fixed & ~occupied).sum()),
             'karte_beweglich': int((occupied & maybe).sum()), 'karte_verschwunden': int(gone.sum()),
             'vielleicht': int(maybe.sum())}
    return fixed, maybe & ~fixed, stats


# ---------------------------------------------------------------------- Karte aktualisieren (Export)
def read_pgm(path):
    """Binäres PGM (P5, 8 bit) → Bild als uint8-Array (Zeile 0 = oben)."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b'P5' or int(tokens[3]) > 255:
        raise ValueError(f'{path}: nur binäres 8-bit-PGM (P5) wird unterstützt')
    w, h = int(tokens[1]), int(tokens[2])
    pos += 1                                           # genau ein Leerzeichen nach maxval
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos).reshape(h, w).copy()


def write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())


def near(mask, radius):
    """Zellen mit einer True-Zelle im Umkreis von radius Zellen (Quadrat), ohne scipy."""
    h, w = mask.shape
    padded = np.pad(mask, radius)
    out = np.zeros_like(mask)
    for dy in range(2 * radius + 1):
        for dx in range(2 * radius + 1):
            out |= padded[dy:dy + h, dx:dx + w]
    return out


def updated_map_image(img, grid, min_seen, p_free, p_fixed, gone_radius=4):
    """Kartenbild mit der Erfahrung abgleichen (Regel wie structure; nur oft genug gesehene Zellen, sonst bleibt alles):
      Kartenwand verschwunden oder beweglich → frei, außer sie liegt direkt an fester Struktur (Wandkante: Posenfehler
                                               machen Randzellen schnell zu „vielleicht“)
      neue feste Struktur (Erfahrung „fest“, nicht in der Karte) → Wand
      unbekannt in der Karte, Erfahrung frei/vielleicht → frei (inzwischen erkundet; Stühle gehören nicht in die Karte)
    Gibt (neues Bild, Anzahl neu belegt, Anzahl neu frei, Anzahl entfernter Kartenwände) zurück."""
    if img.shape != (grid.height, grid.width):
        raise ValueError(f'Bild {img.shape[1]}×{img.shape[0]} passt nicht zur Erfahrung {grid.width}×{grid.height}')
    known = grid.known_grid(min_seen, p_free, p_fixed).reshape(grid.height, grid.width)[::-1]   # Bildzeile 0 = oben
    occ = img == PGM_OCCUPIED
    fixed, _, _ = structure(occ, known, gone_radius)
    removed = occ & ~fixed & ~near(fixed, 1)
    added = fixed & ~occ
    explored = (img == PGM_UNKNOWN) & ((known == KNOWN_FREE) | (known == KNOWN_MAYBE)) & ~fixed
    out = img.copy()
    out[removed | explored] = PGM_FREE
    out[added] = PGM_OCCUPIED
    return out, int(added.sum()), int((removed | explored).sum()), int(removed.sum())
