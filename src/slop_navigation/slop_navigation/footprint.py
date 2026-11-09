"""Footprint und Sicherheitszonen aus dem URDF, ohne ROS (testbar).

Footprint = Umriss aller sichtbaren Teile des Roboters (box, cylinder, sphere) von oben gesehen, in base_footprint,
Gelenke in Nullstellung (Greifer offen = am breitesten), quer aufgefüllt (der Schlitz zwischen den Backen zählt mit). Ergebnis ist ein achsparalleles Polygon, z. B. die T-Form aus
Körper und Greifer. Die Zonen der Collision Monitore entstehen daraus mit Zuschlägen (config/zones.yaml):

  grow: [vorn, hinten, seitlich]   Footprint in jede Richtung um so viel vergrößert (Minkowski-Summe mit einem Rechteck)
  band: [tiefe, seitlich]          Streifen VOR der Front (je y die vorderste Kante des Footprints), seitlich verbreitert,
                                   samt Footprint (ein zusammenhängendes Polygon)

Gerechnet wird auf einem Raster (RES, Kanten nach außen gerundet), daraus wird der Umriss als Polygon gezogen.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

RES = 0.005     # m Raster; alle Maße des Slop-Bots sind Vielfache davon, Kleinteile werden nach außen gerundet


# ------------------------------------------------------------------ URDF → Rechtecke
def _floats(text, n=3):
    return [float(v) for v in (text or ' '.join(['0'] * n)).split()]


def _rot(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _tf(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, :3] = _rot(_floats(origin.get('rpy')))
        t[:3, 3] = _floats(origin.get('xyz'))
    return t


def rects_from_urdf(urdf_xml, root='base_footprint'):
    """Achsparallele Rechtecke (x0, x1, y0, y1) aller Visuals in root, Gelenke in Nullstellung. Meshes zählen nicht."""
    robot = ET.fromstring(urdf_xml)
    links = {l.get('name'): l for l in robot.findall('link')}
    children = {}
    for j in robot.findall('joint'):
        children.setdefault(j.find('parent').get('link'), []).append(j)
    rects, todo = [], [(root, np.eye(4))]
    while todo:
        name, t_link = todo.pop()
        for vis in links[name].findall('visual'):
            t = t_link @ _tf(vis.find('origin'))
            geo = vis.find('geometry')
            box, cyl, sph = geo.find('box'), geo.find('cylinder'), geo.find('sphere')
            if box is not None:
                half = np.array(_floats(box.get('size'))) / 2.0
            elif cyl is not None:
                r = float(cyl.get('radius'))
                half = np.array([r, r, float(cyl.get('length')) / 2.0])
            elif sph is not None:
                r = float(sph.get('radius'))
                half = np.array([r, r, r])
            else:
                continue
            corners = np.array([[sx, sy, sz, 1.0] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            corners[:, :3] *= half
            pts = (t @ corners.T).T
            rects.append((pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max()))
        for j in children.get(name, []):
            todo.append((j.find('child').get('link'), t_link @ _tf(j.find('origin'))))
    return rects


# ------------------------------------------------------------------ Raster
class Grid:
    """Belegtes Raster um den Ursprung; Zelle (i, j) = x in [x0 + i*RES, …), y in [y0 + j*RES, …)."""

    def __init__(self, cells, x0, y0):
        self.cells, self.x0, self.y0 = cells, x0, y0

    @classmethod
    def from_rects(cls, rects, pad=1.0):
        x0 = math.floor((min(r[0] for r in rects) - pad) / RES) * RES
        y0 = math.floor((min(r[2] for r in rects) - pad) / RES) * RES
        nx = int(round((max(r[1] for r in rects) + pad - x0) / RES)) + 1
        ny = int(round((max(r[3] for r in rects) + pad - y0) / RES)) + 1
        cells = np.zeros((nx, ny), dtype=bool)
        for xa, xb, ya, yb in rects:
            i0, i1 = math.floor((xa - x0) / RES + 1e-6), math.ceil((xb - x0) / RES - 1e-6)
            j0, j1 = math.floor((ya - y0) / RES + 1e-6), math.ceil((yb - y0) / RES - 1e-6)
            cells[i0:i1, j0:j1] = True
        return cls(cells, x0, y0)

    def fill_across(self):
        """Je x alles zwischen der äußersten linken und rechten Zelle füllen: Schlitze (z. B. zwischen den offenen
        Greiferbacken, dort sitzt das Objekt) zählen zum Roboter. Die T-Form (schmaler Greifer vor breitem Körper) bleibt."""
        out = self.cells.copy()
        for i in range(out.shape[0]):
            filled = np.nonzero(out[i])[0]
            if filled.size:
                out[i, filled.min():filled.max() + 1] = True
        return Grid(out, self.x0, self.y0)

    def grow(self, front, rear, side):
        """Minkowski-Summe mit dem Rechteck [-rear, front] × [-side, side]."""
        f, b, s = (int(round(v / RES)) for v in (front, rear, side))
        rows = np.zeros_like(self.cells)
        for di in range(-b, f + 1):
            rows |= np.roll(self.cells, di, axis=0)
        out = np.zeros_like(self.cells)
        for dj in range(-s, s + 1):
            out |= np.roll(rows, dj, axis=1)
        return Grid(out, self.x0, self.y0)

    def band(self, depth, side):
        """Streifen der Tiefe depth vor der vordersten Kante (je y), seitlich um side verbreitert, zusammen mit dem Footprint
        selbst (sonst zerfiele er bei der T-Form in zwei Teile: vor dem Greifer und vor den Schultern des Körpers)."""
        d, s = int(round(depth / RES)), int(round(side / RES))
        front = np.zeros_like(self.cells)
        for j in range(self.cells.shape[1]):
            filled = np.nonzero(self.cells[:, j])[0]
            if filled.size:
                i = filled.max() + 1          # erste freie Zelle vor der Front
                front[i:i + d, j] = True
        out = self.cells.copy()
        for dj in range(-s, s + 1):
            out |= np.roll(front, dj, axis=1)
        return Grid(out, self.x0, self.y0)

    def polygon(self):
        """Umriss als Punktliste [[x, y], …] gegen den Uhrzeigersinn, ohne Punkte auf geraden Kanten.
        Nur ein zusammenhängendes Gebiet ohne Löcher, sonst ValueError."""
        c = np.pad(self.cells, 1)
        edges = {}                             # Startecke → Endecke, belegte Zelle links der Kante
        nx, ny = c.shape
        for i in range(nx):
            for j in range(ny):
                if not c[i, j]:
                    continue
                if not c[i, j - 1]:
                    edges[(i, j)] = (i + 1, j)          # unten, nach +x
                if not c[i + 1, j]:
                    edges[(i + 1, j)] = (i + 1, j + 1)  # rechts, nach +y
                if not c[i, j + 1]:
                    edges[(i + 1, j + 1)] = (i, j + 1)  # oben, nach -x
                if not c[i - 1, j]:
                    edges[(i, j + 1)] = (i, j)          # links, nach -y
        if not edges:
            raise ValueError('leerer Footprint')
        start = min(edges)
        loop, p = [start], edges[start]
        while p != start:
            loop.append(p)
            p = edges[p]
        if len(loop) != len(edges):
            raise ValueError('Footprint besteht aus mehreren Teilen oder hat Löcher')
        corners = [q for k, q in enumerate(loop)
                   if (q[0] - loop[k - 1][0], q[1] - loop[k - 1][1]) !=
                   (loop[(k + 1) % len(loop)][0] - q[0], loop[(k + 1) % len(loop)][1] - q[1])]
        return [[round(self.x0 + (i - 1) * RES, 4), round(self.y0 + (j - 1) * RES, 4)] for i, j in corners]


# ------------------------------------------------------------------ Ausgabe
def footprint_grid(urdf_xml):
    """Raster des Footprints: alle Teile, quer aufgefüllt (siehe Grid.fill_across)."""
    return Grid.from_rects(rects_from_urdf(urdf_xml)).fill_across()


def zone(grid, spec):
    """spec {'grow': [vorn, hinten, seitlich]} oder {'band': [tiefe, seitlich]} → Polygon."""
    if 'grow' in spec:
        return grid.grow(*spec['grow']).polygon()
    if 'band' in spec:
        return grid.band(*spec['band']).polygon()
    raise ValueError(f'Zone braucht grow oder band: {spec}')


def nav2_string(poly):
    """Footprint-Parameter für die Costmaps: "[[x, y], …]"."""
    return '[' + ', '.join(f'[{x:g}, {y:g}]' for x, y in poly) + ']'


def flat(poly):
    """points-Parameter der Collision Monitore: [x1, y1, x2, y2, …]."""
    return [float(v) for p in poly for v in p]


def radii(poly):
    """(Innenkreis, Außenkreis) wie Nav2: kleinster Abstand des Ursprungs zu einer Kante, größter zu einer Ecke."""
    def seg(a, b):
        ax, ay, bx, by = *a, *b
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, -(ax * dx + ay * dy) / (dx * dx + dy * dy)))
        return math.hypot(ax + t * dx, ay + t * dy)
    inner = min(seg(poly[k - 1], poly[k]) for k in range(len(poly)))
    return inner, max(math.hypot(x, y) for x, y in poly)


def point_in_polygon(xs, ys, poly):
    """Bool je Punkt: liegt im Polygon (Strahlverfahren, auch für nicht konvexe Polygone)."""
    xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
    inside = np.zeros(xs.shape, dtype=bool)
    n = len(poly)
    for k in range(n):
        (x1, y1), (x2, y2) = poly[k - 1], poly[k]
        cross = (y1 > ys) != (y2 > ys)
        with np.errstate(divide='ignore', invalid='ignore'):
            xi = x1 + (ys - y1) * (x2 - x1) / (y2 - y1)
        inside ^= cross & (xs < xi)
    return inside
