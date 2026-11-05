"""Relokalisierung ohne ROS: künstliche Karte, Scan per Strahlverfolgung, Suche über die ganze Karte."""
import math

import numpy as np
import pytest

from slop_navigation.relocalize_core import Candidate, MapModel, decide, dilate, global_search, scan_points

RES = 0.05


def room(asymmetric=True):
    """8 × 6 m Raum mit Wänden; asymmetric: dazu ein Schrank und eine Nische, damit jede Stelle eindeutig ist."""
    w, h = 160, 120
    g = np.zeros((h, w), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    if asymmetric:
        g[20:40, 30:34] = 100            # Schrank links unten
        g[80:84, 100:140] = 100          # Wandstück rechts oben
        g[50:70, 150:159] = -1           # Nische (unbekannt)
        g[50, 150:159] = g[70, 150:159] = 100
    return g


def raycast(grid, x, y, yaw, n=360, max_range=15.0):
    """Abstände eines 360°-Scans (n Strahlen ab -π) bis zur ersten belegten Zelle."""
    ranges = []
    for i in range(n):
        ang = yaw - math.pi + i * 2 * math.pi / n
        r = 0.0
        while r < max_range:
            r += RES / 2
            ix, iy = int((x + r * math.cos(ang)) / RES), int((y + r * math.sin(ang)) / RES)
            if not (0 <= ix < grid.shape[1] and 0 <= iy < grid.shape[0]) or grid[iy, ix] > 50:
                break
        ranges.append(r if r < max_range else float('inf'))
    return ranges


def model_of(grid):
    return MapModel(grid.ravel(), grid.shape[1], grid.shape[0], RES, 0.0, 0.0)


def test_dilate_has_no_wraparound():
    m = np.zeros((5, 5), dtype=bool)
    m[0, 0] = True
    d = dilate(m, 1)                     # Kreis mit Radius 1: Nachbarn, aber keine Diagonale
    assert d[0, 1] and d[1, 0] and not d[1, 1] and not d[4, 4] and not d[0, 4] and not d[4, 0]
    assert dilate(m, 2)[1, 1]


def test_score_high_at_true_pose_low_elsewhere():
    g = room()
    model = model_of(g)
    r, a = scan_points(raycast(g, 2.0, 2.0, 0.3), -math.pi, 2 * math.pi / 360)
    assert model.score(r, a, 2.0, 2.0, 0.3) > 0.95
    assert model.score(r, a, 5.0, 4.0, 2.0) < 0.6


@pytest.mark.parametrize('x,y,yaw', [(2.0, 2.0, 0.3), (6.0, 4.5, -2.5), (4.0, 1.0, 1.6)])
def test_global_search_finds_pose(x, y, yaw):
    g = room()
    r, a = scan_points(raycast(g, x, y, yaw), -math.pi, 2 * math.pi / 360)
    cands = global_search(model_of(g), r, a)
    best, why = decide(cands)
    assert best is not None, why
    dist, dyaw = best.distance_to(Candidate(0, x, y, yaw))
    assert dist < 0.12 and dyaw < 4.0


def test_symmetric_room_is_refused():
    """Leerer rechteckiger Raum: Die Pose um 180° gedreht und gespiegelt passt genauso gut → nicht eindeutig."""
    g = room(asymmetric=False)
    r, a = scan_points(raycast(g, 2.5, 2.0, 0.0), -math.pi, 2 * math.pi / 360)
    best, why = decide(global_search(model_of(g), r, a))
    assert best is None and 'nicht eindeutig' in why


def test_decide_rules():
    good = Candidate(0.95, 0, 0, 0)
    assert decide([good, Candidate(0.80, 3, 0, 0)])[0] is good
    assert decide([Candidate(0.70, 0, 0, 0)])[0] is None
    assert decide([good, Candidate(0.92, 3, 0, 0)])[0] is None
    assert decide([])[0] is None


def test_scan_points_filters_and_thins():
    r, a = scan_points([0.1, 1.0, float('inf'), float('nan'), 2.0, 20.0], 0.0, 0.1)
    assert list(r) == [1.0, 2.0] and a[0] == pytest.approx(0.1)
    r, _ = scan_points([1.0] * 721, -math.pi, 0.0087266, max_points=120)
    assert len(r) == 120
