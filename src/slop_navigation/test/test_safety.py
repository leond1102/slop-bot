"""Dynamisches Sicherheitsfeld ohne ROS: bekannte Struktur gegen unbekannte Hindernisse."""
import math

import numpy as np

from slop_navigation.experience_core import ExperienceGrid
from slop_navigation.safety_core import is_known, known_mask, outside_body, scan_points, transform

W, H, RES, OX, OY = 60, 40, 0.05, -1.0, -1.0          # 3 × 2 m, x -1..2, y -1..1


def world_map():
    m = np.zeros((H, W), dtype=np.int8)
    m[:, 50] = 100                                     # Wand bei x = 1.525
    m[10, 20] = m[10, 21] = 100                        # zwei Stühle, die beim Kartieren da standen: (0.025 | 0.075, -0.475)
    return m


def known(x, y, mask):
    return bool(is_known(mask, OX, OY, RES, np.array([x]), np.array([y]))[0])


def test_wall_known_with_tolerance_free_space_unknown():
    k = known_mask(W, H, world_map().ravel(), None, dilate_cells=2)
    assert known(1.53, 0.0, k)                        # auf der Wand
    assert known(1.45, 0.0, k)                        # 8 cm davor (Lokalisierungsfehler)
    assert not known(1.30, 0.0, k)                    # 23 cm davor: Mensch vor der Wand
    assert not known(0.5, 0.5, k)                     # freier Raum
    assert not known(5.0, 0.0, k)                     # außerhalb der Karte: unbekannt


def test_experience_overrides_map_both_ways():
    m = world_map()
    m[30, 30] = 100                                    # Kiste beim Kartieren, längst weg: (0.525, 0.525)
    exp = np.full((H, W), -1, dtype=np.int8)
    exp[10, 20:22] = 50                                # Stühle aus der Karte: oft gesehen, mal da, mal nicht
    exp[30, 30] = 0                                    # Kiste: immer frei gesehen, nichts in der Nähe → verschwunden
    exp[30, 10] = 100                                  # neuer Schrank, nicht in der Karte: fest
    exp[:, 50] = 0                                     # Wand selbst frei gesehen (Karte versetzt) …
    exp[:, 53] = 100                                   # … die Treffer liegen 15 cm daneben → Wand bleibt
    k = known_mask(W, H, m.ravel(), exp.ravel(), dilate_cells=0)
    assert not known(0.025, -0.475, k)                 # Karten-Stuhl gilt als unbekannt (vorsichtig)
    assert not known(0.075, -0.475, k)
    assert not known(0.525, 0.525, k)                  # verschwundene Kiste
    assert known(-0.475, 0.525, k)                     # neuer Schrank gilt als bekannt
    assert known(1.53, 0.0, k)                         # Kartenwand trotz „frei“: Treffer 15 cm daneben


def test_experience_known_grid_values():
    g = ExperienceGrid(W, H, RES, OX, OY)
    i_fixed, i_maybe, i_free = 5, 6, 7
    g.seen[[i_fixed, i_maybe, i_free]] = 10.0
    g.hit[i_fixed], g.hit[i_maybe] = 10.0, 5.0
    kg = g.known_grid(min_seen=5, p_free=0.15, p_fixed=0.8)
    assert kg[i_fixed] == 100 and kg[i_maybe] == 50 and kg[i_free] == 0 and kg[8] == -1


def test_scan_points_body_filter_and_transform():
    ranges = [0.12, 0.30, math.inf, 2.0, 5.0]
    x, y = scan_points(ranges, 0.0, math.pi / 2, 0.1, 30.0, max_range=3.0)
    assert len(x) == 3                                 # inf und 5 m (> max_range) fallen weg
    keep = outside_body(x, y, -0.40, 0.10, 0.16, 0.03)
    assert list(keep) == [False, True, True]           # 0.12 m vorn liegt im Körper (Front 0.10 + 3 cm)
    mx, my = transform(np.array([1.0]), np.array([0.0]), 2.0, 1.0, math.pi / 2)
    assert np.allclose([mx[0], my[0]], [2.0, 2.0])
