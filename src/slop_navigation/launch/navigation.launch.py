"""Nav2 für den Slop-Bot: Lokalisierung (map_server + AMCL + relocalize, LIO) + Planer, Controller, Costmaps,
Verhaltensbaum, Geschwindigkeitsglättung, Collision Monitore + Sensor-Wächter + RViz. Vorerst nur mit dem Lidar
(die Kamera ist aus Costmaps und Collision Monitoren herausgenommen, siehe config/nav2.yaml).

    ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py    # Terminal 1: Antrieb, Modell, EKF (ZUERST)
    ros2 launch slop_navigation navigation.launch.py                              # Terminal 2: dieses
    (Ziel in RViz mit "Nav2 Goal"; Tastatur in einem 3. Terminal hat immer Vorrang)

    ros2 launch slop_navigation navigation.launch.py safety:=locker         # extrem nah, volles Tempo, rückwärts erlaubt
    ros2 launch slop_navigation navigation.launch.py map:=$HOME/slop-bot_ws/maps/office-3.yaml
    ros2 launch slop_navigation navigation.launch.py localization:=false    # localization.launch.py läuft schon
    ros2 launch slop_navigation navigation.launch.py experience_costs:=true # Erfahrungskarte beeinflusst den Planer

Sicherheitsstufen (safety:=):
  streng (Standard): nie rückwärts, Abstand zu Unbekanntem, langsamer nahe an Hindernissen. Befehlskette:
      controller_server/behavior_server → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
      → collision_monitor_dynamic (B: unbekannte Hindernisse aus unknown_obstacles, größere Zonen) → cmd_vel_dynamic
      → collision_monitor (A: alles, sehr kleine Zonen) → /navigation/cmd_vel → twist_mux (Priorität 20)
  locker: extrem nah an alles (bekannt oder nicht), volles Tempo, rückwärts erlaubt. Zwei Schritte (RPP kann unter Humble
      nicht zugleich rückwärts und auf der Stelle drehen): hinfahren mit FollowPath, dann mit RotateToGoal in die
      Zielrichtung drehen (behavior_trees/*_locker.xml). B und unknown_obstacles laufen nicht: velocity_smoother →
      cmd_vel_smoothed → collision_monitor (A, nur 1-cm-Stopp und Vorausschau) → /navigation/cmd_vel.
      Werte: config/nav2_locker.yaml (überschreibt nav2.yaml).
twist_mux: Tastatur 50, Sperren 100, sensor_guard 30. sensor_guard (slop_safety) sperrt Nav2, sobald /scan, /odom oder
(nur streng) /safety/unknown_points ausfallen oder veraltet sind – die Collision Monitore ignorierten sie sonst still.
Einstellungen: config/nav2.yaml (+ nav2_locker.yaml), Verhaltensbäume: behavior_trees/ (Recovery ohne Rückwärtsfahren).
Footprint und Zonen der Collision Monitore kommen aus dem URDF (slop_description) mit den Zuschlägen aus
config/zones.yaml: Sie werden beim Start berechnet und überschreiben footprint/points aus nav2.yaml.
"""
import os

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SAFETY_MODES = {
    'streng': 'nie rückwärts, Abstand zu Unbekanntem, langsamer nahe an Hindernissen',
    'locker': 'extrem nah an alles, volles Tempo, rückwärts erlaubt; Zielrichtung erst am Ziel (auf der Stelle drehen)',
}


def urdf_params(share, mode):
    """Footprint (Costmaps), Zonen (Collision Monitore) und Körperpolygon (unknown_obstacles) aus dem URDF.
    Gibt (Pfad einer Parameterdatei, Footprint-Text fürs Log) zurück."""
    from slop_navigation import footprint as fp
    urdf = os.path.join(get_package_share_directory('slop_description'), 'urdf', 'slop_bot.urdf.xacro')
    with open(os.path.join(share, 'config', 'zones.yaml'), encoding='utf-8') as f:
        zones = yaml.safe_load(f)
    grid = fp.footprint_grid(xacro.process_file(urdf).toxml())
    poly = grid.polygon()
    out = {name: {name: {'ros__parameters': {'footprint': fp.nav2_string(poly)}}}
           for name in ('local_costmap', 'global_costmap')}
    for node, polygons in zones['zones'][mode].items():
        out[node] = {'ros__parameters': {name: {'points': fp.flat(fp.zone(grid, spec))}
                                         for name, spec in polygons.items()}}
    out['unknown_obstacles'] = {'ros__parameters': {
        'body_polygon': fp.flat(fp.zone(grid, zones['unknown_obstacles']))}}
    folder = os.path.join(os.path.expanduser('~'), '.ros', 'slop_navigation')
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f'footprint_{mode}.yaml')        # wird bei jedem Start neu geschrieben
    with open(path, 'w', encoding='utf-8') as f:
        f.write('# Von navigation.launch.py aus dem URDF und config/zones.yaml erzeugt – nicht von Hand ändern.\n')
        yaml.safe_dump(out, f)
    inner, outer = fp.radii(poly)
    return path, f'{fp.nav2_string(poly)} (Innenkreis {inner:.2f} m, Außenkreis {outer:.2f} m)'


def nav_nodes(context):
    """Nav2-Knoten je nach Sicherheitsstufe."""
    mode = context.launch_configurations['safety'].strip().lower()
    if mode not in SAFETY_MODES:
        raise RuntimeError(f'safety:={mode} unbekannt, erlaubt: {", ".join(SAFETY_MODES)}')
    locker = mode == 'locker'

    share = get_package_share_directory('slop_navigation')
    params = [os.path.join(share, 'config', 'nav2.yaml')]
    if locker:
        params.append(os.path.join(share, 'config', 'nav2_locker.yaml'))     # später geladen = gewinnt
    footprint_file, footprint_text = urdf_params(share, mode)
    params.append(footprint_file)                                            # Footprint und Zonen aus dem URDF
    trees = os.path.join(share, 'behavior_trees')
    suffix = '_locker' if locker else ''              # locker: erst hinfahren (auch rückwärts), dann auf der Stelle drehen
    guard_cfg = os.path.join(get_package_share_directory('slop_safety'), 'config', 'sensor_guard.yaml')
    nav_cmd = [('cmd_vel', 'cmd_vel_nav')]
    lifecycle = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother']

    nodes = [
        LogInfo(msg=f'Sicherheitsstufe {mode}: {SAFETY_MODES[mode]}'),
        LogInfo(msg=f'Footprint aus dem URDF: {footprint_text}'),
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=params, remappings=nav_cmd),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=params),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=params, remappings=nav_cmd),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=params + [{
                 'default_nav_to_pose_bt_xml': os.path.join(trees, f'navigate_to_pose{suffix}.xml'),
                 'default_nav_through_poses_bt_xml': os.path.join(trees, f'navigate_through_poses{suffix}.xml')}]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=params,
             remappings=nav_cmd + [('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
    ]
    if not locker:
        # Dynamisches Sicherheitsfeld: Scanpunkte ohne bekannte Struktur → collision_monitor_dynamic (größere Zonen)
        nodes += [
            Node(package='slop_navigation', executable='unknown_obstacles', name='unknown_obstacles', output='screen',
                 parameters=[footprint_file]),
            Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor_dynamic',
                 output='screen', parameters=params),
        ]
        lifecycle.append('collision_monitor_dynamic')
    lifecycle.append('collision_monitor')
    # Wächter: nur die Quellen, die in dieser Stufe ein Collision Monitor nutzt
    sources = ['scan', 'odom'] if locker else ['scan', 'unknown', 'odom']
    nodes += [
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             output='screen', parameters=params),
        # Quelle der Collision Monitore fehlt oder ist veraltet → safety/sensor_lock (twist_mux sperrt Nav2)
        Node(package='slop_safety', executable='sensor_guard', name='sensor_guard', output='screen',
             parameters=[guard_cfg, {'sources': sources}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{'autostart': True, 'node_names': lifecycle, 'bond_timeout': 4.0}]),
    ]
    return nodes


def generate_launch_description():
    share = get_package_share_directory('slop_navigation')
    rviz_cfg = os.path.join(get_package_share_directory('slop_description'), 'rviz', 'navigation.rviz')
    default_map = os.path.join(os.path.expanduser('~'), 'slop-bot_ws', 'maps', 'office-3.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Karte (.yaml), voller Pfad'),
        DeclareLaunchArgument('safety', default_value='streng',
                              description='Sicherheitsstufe: streng (Abstand, nie rückwärts) oder locker (extrem nah, '
                                          'volles Tempo, rückwärts erlaubt)'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='localization.launch.py mitstarten (LIO, map_server, AMCL, relocalize)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz mit Navigationsansicht'),
        DeclareLaunchArgument('experience_costs', default_value='false',
                              description='Erfahrungskarte: „vielleicht belegt“ als weiche Kosten an den Planer'),
        # In einer Gruppe: Sonst gälte das rviz:=false der Lokalisierung auch hier, und RViz für Nav2 startete nicht
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'localization.launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'), 'rviz': 'false',
                              'experience_costs': LaunchConfiguration('experience_costs')}.items())],
            scoped=True, condition=IfCondition(LaunchConfiguration('localization'))),
        OpaqueFunction(function=nav_nodes),
        Node(package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
