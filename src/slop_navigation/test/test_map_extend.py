"""Karte erweitern ohne ROS: Startpose, Erfahrung auf die größere Karte umrechnen, Übertragen."""
import os
import time

import numpy as np
import pytest
import yaml

from slop_navigation.experience_core import ExperienceGrid, write_pgm
import math

from slop_navigation.map_extend import (ExtendError, check_pose_graph, find_start_pose, initial_guess, load_map_model,
                                        map_base, regrid, transfer)
from slop_navigation.relocalize_core import Candidate, scan_points
from test_relocalize import raycast, room

RES = 0.05


def write_pose(base, age_s, score=0.9):
    stamp = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(time.time() - age_s))
    with open(base + '.last_pose.yaml', 'w') as f:
        yaml.safe_dump({'x': 1.5, 'y': -2.0, 'yaw': 0.7, 'score': score, 'stamp': stamp}, f)


def write_map(base, w, h, ox, oy):
    write_pgm(base + '.pgm', np.full((h, w), 254, dtype=np.uint8))
    with open(base + '.yaml', 'w') as f:
        yaml.safe_dump({'image': os.path.basename(base) + '.pgm', 'mode': 'trinary', 'resolution': RES,
                        'origin': [ox, oy, 0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}, f)


def test_map_base_variants(tmp_path):
    assert map_base('office-3', str(tmp_path)) == str(tmp_path / 'office-3')
    assert map_base(str(tmp_path / 'office-3.yaml')) == str(tmp_path / 'office-3')


def test_initial_guess_sources(tmp_path):
    base = str(tmp_path / 'k')
    g, why = initial_guess(base, '1', '2', '0.5')
    assert (g.x, g.y, g.yaw) == (1.0, 2.0, 0.5) and 'Hand' in why            # von Hand hat Vorrang
    with pytest.raises(ExtendError, match='alle drei'):
        initial_guess(base, '1', '', '')
    assert initial_guess(base)[0] is None                                      # nichts gespeichert: ganze Karte
    write_pose(base, age_s=3 * 3600)                                           # alt ist egal – wird am Scan geprüft
    g, why = initial_guess(base)
    assert (g.x, g.y, g.yaw) == (1.5, -2.0, 0.7) and 'letzte Pose' in why


def save_room_map(base, grid):
    """Raster (Zeile 0 = unten, 100/0/-1) als map_saver-Karte speichern."""
    img = np.where(grid > 50, 0, np.where(grid == 0, 254, 205)).astype(np.uint8)[::-1]
    write_pgm(base + '.pgm', img)
    with open(base + '.yaml', 'w') as f:
        yaml.safe_dump({'image': os.path.basename(base) + '.pgm', 'mode': 'trinary', 'resolution': RES,
                        'origin': [0.0, 0.0, 0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}, f)


def test_find_start_pose_from_old_or_missing_guess(tmp_path):
    base = str(tmp_path / 'raum')
    g = room()
    save_room_map(base, g)
    model = load_map_model(base)
    assert model.stats['fest'] == int((g > 50).sum())                          # Karte wie map_server gelesen
    x, y, yaw = 2.0, 2.0, 0.3
    r, a = scan_points(raycast(g, x, y, yaw), -math.pi, 2 * math.pi / 360)
    near_guess = Candidate(0, x + 0.15, y - 0.1, yaw + 0.1)                   # 18 cm / 6° daneben
    pose, why = find_start_pose(model, r, a, near_guess)
    assert pose.distance_to(Candidate(0, x, y, yaw))[0] < 0.08 and 'bestätigt' in why
    far_guess = Candidate(0, 6.0, 4.5, -2.0)                                   # Roboter wurde seither weggefahren
    pose, why = find_start_pose(model, r, a, far_guess)
    assert pose.distance_to(Candidate(0, x, y, yaw))[0] < 0.12 and 'ganzen Karte' in why
    pose, _ = find_start_pose(model, r, a, None)                               # keine Vermutung
    assert pose.distance_to(Candidate(0, x, y, yaw))[0] < 0.12


def test_find_start_pose_refuses_ambiguous(tmp_path):
    base = str(tmp_path / 'sym')
    g = room(asymmetric=False)
    save_room_map(base, g)
    r, a = scan_points(raycast(g, 2.5, 2.0, 0.0), -math.pi, 2 * math.pi / 360)
    with pytest.raises(ExtendError, match='nicht eindeutig'):
        find_start_pose(load_map_model(base), r, a, None)


def test_check_pose_graph(tmp_path):
    base = str(tmp_path / 'k')
    with pytest.raises(ExtendError, match='posegraph'):
        check_pose_graph(base)
    for ext in ('.posegraph', '.data'):
        open(base + ext, 'w').close()
    check_pose_graph(base)


def test_regrid_keeps_world_positions():
    """Karte wächst nach links unten (Ursprung kleiner, nicht auf dem alten Raster): Zellen behalten ihre Weltlage."""
    old = ExperienceGrid(40, 30, RES, -1.0, -1.0)
    i = old.cell_index([0.52], [0.27])[0]
    old.seen[i], old.hit[i] = 7.0, 6.0
    new, (dx, dy) = regrid(old, 80, 60, RES, -2.012, -1.49)                     # Ursprung 1,2 cm neben dem alten Raster
    assert (dx, dy) == (20, 10)
    j = new.cell_index([0.52], [0.27])[0]
    assert new.seen[j] == 7.0 and new.hit[j] == 6.0
    assert new.seen.sum() == 7.0                                                # nichts verdoppelt, Rest leer
    with pytest.raises(ExtendError, match='Auflösung'):
        regrid(old, 40, 30, 0.1, -1.0, -1.0)
    shrunk, _ = regrid(old, 20, 20, RES, 0.0, 0.0)                              # nur 0…1 m überlappt: kein Fehler
    assert shrunk.seen.sum() == 7.0


def test_transfer_files(tmp_path):
    old_base, new_base = str(tmp_path / 'office-3'), str(tmp_path / 'office-4')
    write_map(old_base, 40, 30, -1.0, -1.0)
    write_map(new_base, 80, 60, -2.0, -1.5)
    g = ExperienceGrid(40, 30, RES, -1.0, -1.0)
    g.seen[:] = 2.0
    g.save(old_base + '.experience.npz')
    write_pose(old_base, age_s=10)
    lines = transfer(old_base + '.yaml', new_base + '.yaml')
    assert any('Erfahrung übertragen' in l and '100%' in l for l in lines)
    assert ExperienceGrid.load(new_base + '.experience.npz').same_grid(80, 60, RES, -2.0, -1.5)
    assert os.path.exists(new_base + '.last_pose.yaml')
    with pytest.raises(ExtendError, match='gibt es schon'):
        transfer(old_base + '.yaml', new_base + '.yaml')                       # nichts still überschreiben
