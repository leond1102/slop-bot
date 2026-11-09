"""Footprint und Zonen aus dem URDF (slop_navigation/footprint.py), ohne ROS-Laufzeit."""
import os

import numpy as np
import pytest
import yaml

from slop_navigation import footprint as fp

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')

# Körper 0,5 × 0,32 ab 0,10 vor dem Ursprung, Greifer: Träger 3 cm tief, zwei Backen mit Schlitz dazwischen, Rad innen
URDF = '''<robot name="t">
  <link name="base_footprint"/>
  <link name="body"><visual><origin xyz="-0.15 0 0.1"/><geometry><box size="0.5 0.32 0.2"/></geometry></visual></link>
  <joint name="b" type="fixed"><parent link="base_footprint"/><child link="body"/></joint>
  <link name="wheel"><visual><origin rpy="1.5708 0 0"/><geometry><cylinder radius="0.035" length="0.03"/></geometry></visual></link>
  <joint name="w" type="continuous"><parent link="base_footprint"/><child link="wheel"/><origin xyz="0 0.12 0.035"/></joint>
  <link name="grip"><visual><origin xyz="0.015 0 0"/><geometry><box size="0.03 0.2 0.06"/></geometry></visual></link>
  <joint name="g" type="fixed"><parent link="base_footprint"/><child link="grip"/><origin xyz="0.10 0 0.1"/></joint>
  <link name="jl"><visual><origin xyz="0.105 0 0"/><geometry><box size="0.21 0.032 0.03"/></geometry></visual></link>
  <joint name="jl" type="prismatic"><parent link="grip"/><child link="jl"/><origin xyz="0 -0.072 0.015"/></joint>
  <link name="jr"><visual><origin xyz="0.105 0 0"/><geometry><box size="0.21 0.032 0.03"/></geometry></visual></link>
  <joint name="jr" type="prismatic"><parent link="grip"/><child link="jr"/><origin xyz="0 0.072 -0.015"/></joint>
  <link name="flag"><visual><geometry><mesh filename="package://x/flag.stl"/></geometry></visual></link>
  <joint name="f" type="fixed"><parent link="body"/><child link="flag"/></joint>
</robot>'''

T_SHAPE = [[-0.4, -0.16], [0.1, -0.16], [0.1, -0.1], [0.13, -0.1], [0.13, -0.09], [0.31, -0.09], [0.31, 0.09],
           [0.13, 0.09], [0.13, 0.1], [0.1, 0.1], [0.1, 0.16], [-0.4, 0.16]]


def test_t_shape_with_filled_slot():
    poly = fp.footprint_grid(URDF).polygon()
    assert poly == T_SHAPE                             # Schlitz zwischen den Backen gefüllt, Rad und Mesh ohne Einfluss
    inner, outer = fp.radii(poly)
    assert inner == pytest.approx(np.hypot(0.10, 0.10)) and outer == pytest.approx(np.hypot(0.40, 0.16))


def test_grow_is_minkowski_sum():
    g = fp.footprint_grid(URDF)
    poly = fp.zone(g, {'grow': [0.10, 0.05, 0.02]})
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    assert max(xs) == pytest.approx(0.41) and min(xs) == pytest.approx(-0.45)
    assert max(ys) == pytest.approx(0.18) and min(ys) == pytest.approx(-0.18)
    assert [0.2, 0.18] in poly                         # Schulter des Körpers (0,10 | 0,16) um vorn/seitlich verschoben


def test_band_in_front_of_gripper_and_shoulders():
    g = fp.footprint_grid(URDF)
    poly = fp.zone(g, {'band': [0.10, 0.06]})
    inside = lambda x, y: bool(fp.point_in_polygon([x], [y], poly)[0])
    assert inside(0.35, 0.0) and inside(0.35, 0.14) and not inside(0.43, 0.0)    # vor dem Greifer
    assert inside(0.15, 0.20) and not inside(0.25, 0.20)                       # vor den Schultern
    assert not inside(0.0, 0.20)                                               # nicht neben dem Körper


def test_point_in_non_convex_polygon():
    inside = fp.point_in_polygon([0.2, 0.2, 0.0, 0.5, -0.3], [0.0, 0.13, 0.0, 0.0, 0.15], T_SHAPE)
    assert list(inside) == [True, False, True, False, True]


def test_strings():
    assert fp.nav2_string([[0.1, -0.2], [0.3, 0.4]]) == '[[0.1, -0.2], [0.3, 0.4]]'
    assert fp.flat([[0.1, -0.2], [0.3, 0.4]]) == [0.1, -0.2, 0.3, 0.4]


def test_shipped_values_match_urdf():
    """nav2.yaml/nav2_locker.yaml tragen den Stand, den navigation.launch.py aus dem URDF rechnet (sonst veralten sie)."""
    xacro = pytest.importorskip('xacro')
    try:
        from ament_index_python.packages import get_package_share_directory
        urdf = os.path.join(get_package_share_directory('slop_description'), 'urdf', 'slop_bot.urdf.xacro')
        xml = xacro.process_file(urdf).toxml()
    except Exception as e:                             # ohne gebauten Workspace
        pytest.skip(f'URDF nicht verfügbar: {e}')
    g = fp.footprint_grid(xml)
    load = lambda n: yaml.safe_load(open(os.path.join(CONFIG, n), encoding='utf-8'))
    nav2, locker, zones = load('nav2.yaml'), load('nav2_locker.yaml'), load('zones.yaml')
    for name in ('local_costmap', 'global_costmap'):
        assert yaml.safe_load(nav2[name][name]['ros__parameters']['footprint']) == g.polygon()
    for mode, cfg in (('streng', nav2), ('locker', locker)):
        for node, polys in zones['zones'][mode].items():
            for name, spec in polys.items():
                params = (cfg if name in cfg.get(node, {}).get('ros__parameters', {}) else nav2)[node]['ros__parameters']
                assert params[name]['points'] == pytest.approx(fp.flat(fp.zone(g, spec))), f'{mode}/{node}/{name}'
