#!/usr/bin/env python3
"""Karte erweitern: gespeicherte Karte mit slam_toolbox weiterkartieren (mapping.launch.py extend:=<karte>) und danach
Erfahrung und letzte Pose auf die größere Karte übertragen.

    ros2 launch slop_navigation mapping.launch.py extend:=office-3                        # ab der letzten guten Pose
    ros2 launch slop_navigation mapping.launch.py extend:=office-3 x:=1.0 y:=-2.0 yaw:=1.57
    ... in den neuen Bereich fahren, dann unter NEUEM Namen speichern (STARTBEFEHLE.md, „Karte speichern“) und:
    ros2 run slop_navigation experience_transfer ~/slop-bot_ws/maps/office-3.yaml ~/slop-bot_ws/maps/office-4.yaml

Startpose: slam_toolbox gleicht den ersten Scan nur mit Knoten in der Nähe der Startpose ab – sie muss also stimmen
(± einige 10 cm). Deshalb bestimmt extend_start (vor slam_toolbox) die Pose am AKTUELLEN Scan, wie relocalize:
  1. Feinsuche um x/y/yaw (falls angegeben) bzw. die letzte gespeicherte Pose (<karte>.last_pose.yaml, egal wie alt)
  2. passt das nicht (< min_score), die ganze Karte durchsuchen; nur ein eindeutiges Ergebnis wird genommen
Findet sich keine eindeutige Pose, startet slam_toolbox nicht (Meldung im Log). Der Roboter muss dabei stillstehen.

Koordinaten bleiben beim Weiterkartieren gleich (derselbe Posegraph), nur die Karte wächst (Größe, Ursprung). Darum
lassen sich Erfahrung und letzte Pose über Weltkoordinaten auf die neue Karte umrechnen.
"""
import argparse
import os
import shutil
import sys
import time

import numpy as np
import yaml

from slop_navigation.experience_core import ExperienceGrid, OldFormat, read_pgm
from slop_navigation.relocalize_core import Candidate, MapModel, decide, global_search, refine

MAPS_DIR = os.path.join(os.path.expanduser('~'), 'slop-bot_ws', 'maps')


class ExtendError(RuntimeError):
    pass


def map_base(name, maps_dir=MAPS_DIR):
    """'office-3', 'office-3.yaml' oder Pfad → Pfad ohne Endung (wie slam_toolbox map_file_name)."""
    name = os.path.expanduser(name)
    if not os.path.isabs(name):
        name = os.path.join(maps_dir, name)
    return name[:-5] if name.endswith('.yaml') else name


def check_pose_graph(base):
    missing = [base + ext for ext in ('.posegraph', '.data') if not os.path.exists(base + ext)]
    if missing:
        raise ExtendError(f'Zum Weiterkartieren fehlt der slam_toolbox-Stand: {", ".join(missing)} '
                          '(beim Speichern mit /slam_toolbox/serialize_map entstanden)')


def initial_guess(base, x='', y='', yaw=''):
    """Vermutete Startpose (Candidate, Herkunft) oder (None, Grund): x/y/yaw von Hand (alle drei) haben Vorrang, sonst
    <base>.last_pose.yaml – egal wie alt, sie wird ohnehin am Scan geprüft."""
    given = [v for v in (x, y, yaw) if str(v).strip() != '']
    if len(given) == 3:
        return Candidate(0.0, float(x), float(y), float(yaw)), 'von Hand angegeben'
    if given:
        raise ExtendError('Startpose von Hand: x, y und yaw alle drei angeben')
    path = base + '.last_pose.yaml'
    try:
        with open(path, encoding='utf-8') as f:
            d = yaml.safe_load(f)
        return Candidate(0.0, float(d['x']), float(d['y']), float(d['yaw'])), f'letzte Pose von {d.get("stamp", "?")}'
    except (OSError, KeyError, TypeError, ValueError):
        return None, 'keine gespeicherte Pose'


def load_map_model(base, tolerance=0.10):
    """Gespeicherte Karte (<base>.yaml/.pgm) als MapModel für die Startsuche, mit Erfahrung (feste Struktur als Referenz),
    wenn eine passende im aktuellen Format daneben liegt."""
    with open(base + '.yaml', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    folder = os.path.dirname(base)
    img = read_pgm(meta['image'] if os.path.isabs(meta['image']) else os.path.join(folder, meta['image']))
    h, w = img.shape
    occ = (255 - img.astype(np.float64)) / 255.0
    if meta.get('negate', 0):
        occ = 1.0 - occ
    data = np.full((h, w), -1, dtype=np.int8)                       # wie map_server (trinary): -1 / 0 / 100
    data[occ > float(meta.get('occupied_thresh', 0.65))] = 100
    data[occ < float(meta.get('free_thresh', 0.196))] = 0
    data = data[::-1].ravel()                                        # OccupancyGrid: Zeile 0 = unten
    res, ox, oy = float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])
    experience = None
    try:
        g = ExperienceGrid.load(base + '.experience.npz')
        if g.same_grid(w, h, res, ox, oy):
            experience = g.known_grid(5.0, 0.15, 0.80)
    except (OSError, ValueError, KeyError):
        pass                                                         # keine oder alte Erfahrung: nur Karte
    return MapModel(data, w, h, res, ox, oy, tolerance=tolerance, experience=experience)


def find_start_pose(model, r, a, guess=None, min_score=0.80, min_margin=0.08):
    """Startpose am aktuellen Scan (r, a): erst um die Vermutung, sonst ganze Karte. (Candidate, Text) oder
    ExtendError, wenn nichts eindeutig passt."""
    notes = []
    if guess is not None:
        cand = refine(model, r, a, guess, xy=0.3, yaw_deg=10.0)
        if cand.score >= min_score:
            return cand, f'Vermutung bestätigt ({cand.score:.0%} auf fester Struktur)'
        notes.append(f'Vermutung passt nur zu {cand.score:.0%}')
    best, why = decide(global_search(model, r, a), min_score, min_margin)
    if best is None:
        raise ExtendError(f'{"; ".join(notes + [why])} – Startpose nicht eindeutig. Roboter an eine Stelle mit '
                          'markanten Wänden stellen oder x, y, yaw angeben')
    return best, '; '.join(notes + [f'in der ganzen Karte gefunden ({why})'])


def map_grid(yaml_path):
    """(Breite, Höhe, Auflösung, Ursprung x, Ursprung y) einer gespeicherten Karte."""
    with open(yaml_path, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    folder = os.path.dirname(os.path.abspath(yaml_path))
    img = read_pgm(meta['image'] if os.path.isabs(meta['image']) else os.path.join(folder, meta['image']))
    return img.shape[1], img.shape[0], float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


def regrid(grid, width, height, resolution, origin_x, origin_y):
    """Erfahrung auf ein anderes Raster derselben Auflösung umrechnen (über Weltkoordinaten, auf ganze Zellen
    gerundet – slam_toolbox legt den Ursprung nicht auf das alte Raster, Fehler höchstens eine halbe Zelle).
    Zellen außerhalb der neuen Karte fallen weg, neue Zellen beginnen ohne Beobachtung."""
    if abs(grid.resolution - resolution) > 1e-6:
        raise ExtendError(f'Auflösung verschieden ({grid.resolution} / {resolution} m) – nicht übertragbar')
    out = ExperienceGrid(width, height, resolution, origin_x, origin_y)
    out.stamp = grid.stamp
    dx = int(round((grid.origin_x - origin_x) / resolution))       # alte Spalte 0 liegt in der neuen Spalte dx
    dy = int(round((grid.origin_y - origin_y) / resolution))
    src_seen = grid.seen.reshape(grid.height, grid.width)
    src_hit = grid.hit.reshape(grid.height, grid.width)
    dst_seen = out.seen.reshape(height, width)
    dst_hit = out.hit.reshape(height, width)
    x0, x1 = max(0, dx), min(width, dx + grid.width)
    y0, y1 = max(0, dy), min(height, dy + grid.height)
    if x0 < x1 and y0 < y1:
        dst_seen[y0:y1, x0:x1] = src_seen[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
        dst_hit[y0:y1, x0:x1] = src_hit[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
    return out, (dx, dy)


def transfer(old_yaml, new_yaml, overwrite=False):
    """Erfahrung und letzte Pose von old_yaml auf new_yaml (erweiterte Karte) übertragen. Gibt Textzeilen zurück."""
    old_base, new_base = map_base(old_yaml), map_base(new_yaml)
    lines = []
    exp_old, exp_new = old_base + '.experience.npz', new_base + '.experience.npz'
    if os.path.exists(exp_new) and not overwrite:
        raise ExtendError(f'{exp_new} gibt es schon (--ueberschreiben)')
    if os.path.exists(exp_old):
        try:
            g = ExperienceGrid.load(exp_old)
        except OldFormat as e:
            lines.append(f'Erfahrung nicht übertragen: {e} (lernt auf der neuen Karte neu)')
        else:
            new, (dx, dy) = regrid(g, *map_grid(new_base + '.yaml'))
            new.save(exp_new)
            kept = float(new.seen.sum()) / max(float(g.seen.sum()), 1e-9)
            lines.append(f'Erfahrung übertragen: {g.width}×{g.height} → {new.width}×{new.height} Zellen, '
                         f'versetzt um ({dx}, {dy}) Zellen, {kept:.0%} der Beobachtungen liegen in der neuen Karte')
    else:
        lines.append('Keine Erfahrung zum Übertragen')
    pose_old, pose_new = old_base + '.last_pose.yaml', new_base + '.last_pose.yaml'
    if os.path.exists(pose_old) and (overwrite or not os.path.exists(pose_new)):
        shutil.copy2(pose_old, pose_new)
        lines.append('Letzte Pose übernommen (gleiche Koordinaten)')
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description='Erfahrung und letzte Pose auf eine erweiterte Karte übertragen')
    ap.add_argument('alt', help='bisherige Karte (.yaml)')
    ap.add_argument('neu', help='erweiterte Karte (.yaml, mit slam_toolbox aus der bisherigen weiterkartiert)')
    ap.add_argument('--ueberschreiben', action='store_true', help='vorhandene Erfahrung/Pose der neuen Karte ersetzen')
    a = ap.parse_args(argv)
    if not os.path.exists(map_base(a.neu) + '.yaml'):
        sys.exit(f'{map_base(a.neu)}.yaml gibt es nicht – erst die erweiterte Karte speichern')
    try:
        for line in transfer(a.alt, a.neu, a.ueberschreiben):
            print(line)
    except ExtendError as e:
        sys.exit(str(e))


if __name__ == '__main__':
    main()
