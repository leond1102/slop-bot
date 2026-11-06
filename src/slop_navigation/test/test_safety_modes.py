"""Prüft die beiden Sicherheitsstufen (nav2.yaml, nav2.yaml + nav2_locker.yaml) ohne ROS: Befehlskette, Zonen, Quellen."""
import copy
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')
TREES = os.path.join(os.path.dirname(__file__), '..', 'behavior_trees')
SUFFIX = {'streng': '', 'locker': '_locker'}            # wie navigation.launch.py


def load(name):
    with open(os.path.join(CONFIG, name), encoding='utf-8') as f:
        return yaml.safe_load(f)


def merge(base, over):
    """Wie mehrere --params-file: spätere Werte gewinnen, Unterbäume werden zusammengeführt."""
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def modes():
    base = load('nav2.yaml')
    return {'streng': base, 'locker': merge(base, load('nav2_locker.yaml'))}


def p(cfg, node):
    return cfg[node]['ros__parameters']


def costmap(cfg, name):
    return cfg[name][name]['ros__parameters']


MONITORS = {'streng': ['collision_monitor_dynamic', 'collision_monitor'], 'locker': ['collision_monitor']}


@pytest.mark.parametrize('mode', ['streng', 'locker'])
def test_only_lidar_sources(mode):
    cfg = modes()[mode]
    for name in ('local_costmap', 'global_costmap'):
        assert costmap(cfg, name)['obstacle_layer']['observation_sources'].split() == ['scan']
    for node in MONITORS[mode]:
        assert 'camera' not in p(cfg, node)['observation_sources']


@pytest.mark.parametrize('mode', ['streng', 'locker'])
def test_polygons_and_sources_defined(mode):
    cfg = modes()[mode]
    for node in MONITORS[mode]:
        params = p(cfg, node)
        for name in params['polygons'] + params['observation_sources']:
            assert name in params, f'{node}: {name} fehlt'
        for name in params['polygons']:
            assert params[name]['action_type'] in ('stop', 'slowdown', 'approach')


@pytest.mark.parametrize('mode', ['streng', 'locker'])
def test_command_chain(mode):
    """Vom Glätter bis twist_mux ohne Lücke: Ausgang jedes Gliedes = Eingang des nächsten."""
    cfg = modes()[mode]
    topic = 'cmd_vel_smoothed'                       # Ausgang velocity_smoother (navigation.launch.py)
    for node in MONITORS[mode]:
        assert p(cfg, node)['cmd_vel_in_topic'] == topic, node
        topic = p(cfg, node)['cmd_vel_out_topic']
    assert topic == '/navigation/cmd_vel'


def test_streng_never_reverses():
    cfg = modes()['streng']
    assert p(cfg, 'controller_server')['FollowPath']['allow_reversing'] is False
    assert p(cfg, 'velocity_smoother')['min_velocity'][0] == 0.0
    assert 'backup' not in p(cfg, 'behavior_server')['behavior_plugins']
    actions = [p(cfg, 'collision_monitor')[n]['action_type'] for n in p(cfg, 'collision_monitor')['polygons']]
    assert 'slowdown' in actions


def test_locker_reverses_close_and_fast():
    cfg = modes()['locker']
    rpp = p(cfg, 'controller_server')['FollowPath']
    assert rpp['allow_reversing'] is True
    # Humble-RPP schaltet allow_reversing still ab, wenn use_rotate_to_heading an ist
    assert rpp['use_rotate_to_heading'] is False
    assert rpp['use_cost_regulated_linear_velocity_scaling'] is False
    # Schritt 1 (FollowPath) prüft nur die Position, Schritt 2 (RotateToGoal) dreht in die Zielrichtung
    ctrl = p(cfg, 'controller_server')
    assert ctrl['position_goal_checker']['yaw_goal_tolerance'] >= 3.14
    assert ctrl['general_goal_checker']['yaw_goal_tolerance'] <= 0.3
    # RPP dreht erst, wenn es näher als die Toleranz des Checkers am Ziel ist: Schritt 2 muss Schritt 1 einschließen
    assert ctrl['general_goal_checker']['xy_goal_tolerance'] > ctrl['position_goal_checker']['xy_goal_tolerance']
    turn = ctrl['RotateToGoal']
    assert turn['use_rotate_to_heading'] is True and turn['allow_reversing'] is False
    smoother = p(cfg, 'velocity_smoother')
    assert smoother['min_velocity'][0] == -smoother['max_velocity'][0]
    cm = p(cfg, 'collision_monitor')
    actions = {cm[n]['action_type'] for n in cm['polygons']}
    assert actions == {'stop', 'approach'}           # keine Langsam-Zone
    for name in ('local_costmap', 'global_costmap'):
        assert costmap(cfg, name)['footprint_padding'] < costmap(modes()['streng'], name)['footprint_padding']


def test_locker_stop_zone_hugs_footprint():
    """1 cm um den Footprint, nicht mehr."""
    cfg = modes()['locker']
    pts = p(cfg, 'collision_monitor')['LooseStop']['points']
    xs, ys = pts[0::2], pts[1::2]
    fp = [[0.10, 0.16], [0.10, -0.16], [-0.40, -0.16], [-0.40, 0.16]]
    assert max(xs) == pytest.approx(max(x for x, _ in fp) + 0.01)
    assert min(xs) == pytest.approx(min(x for x, _ in fp) - 0.01)
    assert max(ys) == pytest.approx(0.17) and min(ys) == pytest.approx(-0.17)


def follow_paths(tree):
    return [e for e in tree.iter('FollowPath')]


@pytest.mark.parametrize('mode', ['streng', 'locker'])
def test_trees_use_existing_controllers_and_checkers(mode):
    """Jeder FollowPath im Baum nennt einen Controller/Goal-Checker, den es gibt; bei mehreren muss er ihn nennen
    (sonst lehnt controller_server unter Humble ab)."""
    ctrl = p(modes()[mode], 'controller_server')
    for name in ('navigate_to_pose', 'navigate_through_poses'):
        tree = ET.parse(os.path.join(TREES, f'{name}{SUFFIX[mode]}.xml')).getroot()
        for fp in follow_paths(tree):
            assert fp.get('controller_id') in ctrl['controller_plugins'], (name, fp.attrib)
            gc = fp.get('goal_checker_id')
            if len(ctrl['goal_checker_plugins']) > 1:
                assert gc in ctrl['goal_checker_plugins'], (name, fp.attrib)
            else:
                assert gc in (None, '', ctrl['goal_checker_plugins'][0])
        for tag in ('BackUp', 'Spin'):
            assert tree.find(f'.//{tag}') is None, f'{name}: {tag} in der Recovery'


@pytest.mark.parametrize('name', ['navigate_to_pose', 'navigate_through_poses'])
def test_locker_tree_drives_then_turns(name):
    tree = ET.parse(os.path.join(TREES, f'{name}_locker.xml')).getroot()
    pipeline = tree.find('.//PipelineSequence')
    inside = {id(e) for e in pipeline.iter('FollowPath')}
    drive = [e for e in follow_paths(tree) if id(e) in inside]
    turn = [e for e in follow_paths(tree) if id(e) not in inside]
    assert [e.get('controller_id') for e in drive] == ['FollowPath']
    assert drive[0].get('goal_checker_id') == 'position_goal_checker'
    # Drehen nach der Pipeline (in ihr würde sie das Hinfahren neu anstoßen) und mit Richtungsprüfung
    assert [(e.get('controller_id'), e.get('goal_checker_id')) for e in turn] == [('RotateToGoal', 'general_goal_checker')]
    seq = tree.find('.//Sequence[@name="DriveThenTurn"]')
    assert list(seq)[0] is pipeline
