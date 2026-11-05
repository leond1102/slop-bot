"""Erfahrungskarte ohne ROS: Strahlverfolgung, Klassen über mehrere Fahrten, Vergessen, Maske, Speichern, Export."""
import math

import numpy as np
import pytest

from slop_navigation.experience_core import (PGM_FREE, PGM_OCCUPIED, PGM_UNKNOWN, UNKNOWN, ExperienceGrid, read_pgm,
                                             updated_map_image, write_pgm)

RES = 0.05
P = dict(min_seen=3, p_free=0.15, p_fixed=0.8)


def grid():
    return ExperienceGrid(100, 80, RES, -2.0, -1.0)      # 5 × 4 m, x -2..3, y -1..3


def scan_to(objects, x=0.0, y=0.0, n=720, max_r=30.0):
    """360°-Scan von (x, y): Abstand zum nächsten Kreis (cx, cy, radius) je Strahl, sonst inf."""
    ranges = []
    for i in range(n):
        a = -math.pi + i * 2 * math.pi / n
        best = math.inf
        for cx, cy, rad in objects:
            dx, dy = cx - x, cy - y
            t = dx * math.cos(a) + dy * math.sin(a)
            d2 = dx * dx + dy * dy - t * t
            if t > 0 and d2 <= rad * rad:
                best = min(best, t - math.sqrt(rad * rad - d2))
        ranges.append(best if best <= max_r else math.inf)
    return ranges, -math.pi, 2 * math.pi / n


def observe(g, objects, x=0.0, y=0.0, **kw):
    r, amin, inc = scan_to(objects, x, y)
    free, hit = g.ray_cells(x, y, 0.0, r, amin, inc, 0.2, 30.0, **kw)
    g.integrate(free, hit)
    return free, hit


def cell(g, x, y):
    return int(g.cell_index([x], [y])[0])


WALL = (2.5, 1.0, 0.3)       # „Säule“, immer da
CHAIR = (1.0, 0.0, 0.15)     # Stuhl, nur manchmal da


def test_ray_cells_free_before_hit_and_hit_cell():
    g = grid()
    free, hit = observe(g, [WALL])
    front = cell(g, 2.5 - 0.3 + 0.01, 1.0)                  # Vorderkante der Säule
    assert front in hit or cell(g, 2.5 - 0.3 - 0.01, 1.0) in hit
    assert cell(g, 1.0, 0.5) in free                         # zwischen Roboter und Säule
    assert not set(free) & set(hit)
    assert cell(g, 2.5, 1.0) not in free                     # hinter der Vorderkante: verdeckt, keine Aussage


def test_ranges_beyond_max_range_and_inf():
    g = grid()
    r, amin, inc = scan_to([(2.8, 0.0, 0.1)])                # Hindernis 2,7 m entfernt
    free, hit = g.ray_cells(0.0, 0.0, 0.0, r, amin, inc, 0.2, 30.0, max_range=1.0, inf_free_range=0.5)
    assert hit.size == 0                                     # zu weit weg: kein Treffer
    assert cell(g, 0.9, 0.0) in free and cell(g, 1.2, 0.0) not in free
    assert cell(g, 0.0, 0.4) in free and cell(g, 0.0, 0.7) not in free   # inf: nur bis 0,5 m


def test_wall_fixed_chair_maybe_aisle_free():
    g = grid()
    for k in range(10):
        observe(g, [WALL, CHAIR] if k % 2 == 0 else [WALL])  # Stuhl jedes zweite Mal da
    c = g.classes(**P)
    assert c[cell(g, 2.5 - 0.3 + 0.01, 1.0)] == 3 or c[cell(g, 2.5 - 0.3 - 0.01, 1.0)] == 3     # Säule fest
    assert c[cell(g, 1.0 - 0.15 + 0.01, 0.0)] == 2 or c[cell(g, 1.0 - 0.15 - 0.01, 0.0)] == 2   # Stuhl vielleicht
    assert c[cell(g, 0.5, 0.0)] == 1                                                             # Gang frei
    assert c[cell(g, 2.95, 2.95)] == 0                                                           # > 4 m, nie gesehen


def test_mask_soft_costs_and_live_clear():
    g = grid()
    for k in range(10):
        observe(g, [WALL, CHAIR] if k % 2 == 0 else [WALL])
    chair = max((cell(g, 1.0 - 0.15 + d, 0.0) for d in (0.01, -0.01)), key=lambda i: g.persistence()[i])
    m = g.mask(**P, max_cost=60)
    assert 1 <= m[chair] < 60                                 # weiche Kosten, nie verboten
    assert m[cell(g, 0.5, 0.0)] == UNKNOWN                   # frei: keine Meinung senden (nie 0)
    assert m[cell(g, -1.9, 2.9)] == UNKNOWN
    assert (m != 0).all() and m.max() <= 60

    # Sitzung: Stuhl gerade nicht da → Zelle frei gesehen → keine Kosten mehr
    r, amin, inc = scan_to([WALL])
    free, hit = g.ray_cells(0.0, 0.0, 0.0, r, amin, inc, 0.2, 30.0)
    g.observe_session(free, hit, 1.0)
    assert g.mask(**P, max_cost=60)[chair] == UNKNOWN
    assert g.mask(**P, max_cost=60, live=False)[chair] == m[chair]
    # später doch wieder belegt gesehen → Kosten zurück
    r, amin, inc = scan_to([WALL, CHAIR])
    free, hit = g.ray_cells(0.0, 0.0, 0.0, r, amin, inc, 0.2, 30.0)
    g.observe_session(free, hit, 2.0)
    if chair in hit:
        assert g.mask(**P, max_cost=60)[chair] == m[chair]


def test_decay_keeps_ratio_but_new_observations_win():
    g = grid()
    for _ in range(10):
        observe(g, [WALL, CHAIR])                             # Stuhl war 10× da → fest
    chair = cell(g, 1.0 - 0.15 - 0.01, 0.0)
    if g.hit[chair] == 0:
        chair = cell(g, 1.0 - 0.15 + 0.01, 0.0)
    p0 = g.persistence()[chair]
    g.stamp = 0.0
    g.decay_to(4 * 3600.0, half_life_s=3600.0)                # 4 Halbwertszeiten
    assert g.persistence()[chair] == pytest.approx(p0)        # Verhältnis bleibt
    assert g.seen[chair] == pytest.approx(10 / 16, rel=1e-4)
    for _ in range(4):
        observe(g, [WALL])                                     # Stuhl weg
    assert g.classes(**P)[chair] == 1                          # nach 4 Fahrten frei
    # zum Vergleich ohne Vergessen: 10 alte Treffer überwiegen, der Stuhl bliebe „vielleicht“
    h = grid()
    for _ in range(10):
        observe(h, [WALL, CHAIR])
    for _ in range(4):
        observe(h, [WALL])
    assert h.classes(**P)[chair] == 2


def test_save_load_roundtrip(tmp_path):
    g = grid()
    observe(g, [WALL, CHAIR])
    path = str(tmp_path / 'karte.experience.npz')
    g.save(path)
    h = ExperienceGrid.load(path)
    assert h.same_grid(100, 80, RES, -2.0, -1.0)
    assert not h.same_grid(100, 80, RES, -2.5, -1.0)
    np.testing.assert_array_equal(g.seen, h.seen)
    np.testing.assert_array_equal(g.hit, h.hit)
    assert h.stamp == pytest.approx(g.stamp)


def test_export_updates_only_well_observed_cells(tmp_path):
    g = grid()
    for _ in range(12):
        observe(g, [WALL])                                     # Säule fest, Gang frei
    img = np.full((80, 100), PGM_UNKNOWN, dtype=np.uint8)
    img[:, :] = PGM_FREE
    ix, iy = 99, 0                                             # Spalte 99 / Bildzeile 0 = (2.98, 2.98): > 4 m, nie gesehen
    img[80 - 1 - 20, 60] = PGM_OCCUPIED                        # alte Wand im Gang bei (1.0, 0.0): Bildzeile 59
    img[iy, ix] = PGM_OCCUPIED                                 # Wand in einem nie gesehenen Bereich
    path = str(tmp_path / 'k.pgm')
    write_pgm(path, img)
    assert (read_pgm(path) == img).all()

    new, n_occ, n_free = updated_map_image(img, g, min_seen=10, p_free=0.15, p_fixed=0.8)
    assert new[80 - 1 - 20, 60] == PGM_FREE                    # verschwundene Wand im Gang → frei
    assert new[iy, ix] == PGM_OCCUPIED                         # nie gesehen → bleibt
    pillar = g.classes(10, 0.15, 0.8).reshape(80, 100)[::-1]
    assert (new[pillar == 3] == PGM_OCCUPIED).all() and n_occ > 0 and n_free > 0


def test_export_keeps_wall_edges_but_removes_lone_chair():
    """„vielleicht“ + in der Karte belegt: neben einer festen Zelle (Wandkante) bleibt es Wand, allein (Stuhl) wird frei."""
    g = grid()
    wall_edge, wall, chair = cell(g, 2.025, 2.025), cell(g, 2.075, 2.025), cell(g, 0.025, 2.025)
    g.seen[[wall_edge, wall, chair]] = 20.0
    g.hit[wall], g.hit[wall_edge], g.hit[chair] = 20.0, 10.0, 10.0          # fest, vielleicht, vielleicht
    img = np.full((80, 100), PGM_FREE, dtype=np.uint8)
    rows = {i: 80 - 1 - i // 100 for i in (wall_edge, wall, chair)}
    cols = {i: i % 100 for i in (wall_edge, wall, chair)}
    for i in (wall_edge, wall, chair):
        img[rows[i], cols[i]] = PGM_OCCUPIED
    new, _, n_free = updated_map_image(img, g, min_seen=10, p_free=0.15, p_fixed=0.8)
    assert new[rows[wall], cols[wall]] == PGM_OCCUPIED
    assert new[rows[wall_edge], cols[wall_edge]] == PGM_OCCUPIED            # Wandkante bleibt
    assert new[rows[chair], cols[chair]] == PGM_FREE and n_free == 1        # einzelner Stuhl verschwindet
