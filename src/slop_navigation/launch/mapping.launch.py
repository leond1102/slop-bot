"""Karte aufnehmen mit slam_toolbox: LIO (Lidar, /scan, EKF-Geschwindigkeit) + slam_toolbox + RViz.

    ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py    # Terminal 1: Antrieb, Modell, EKF
    ros2 launch slop_navigation mapping.launch.py                                 # Terminal 2: dieses
    ros2 run slop_teleop keyboard_teleop                                          # Terminal 3: fahren

    ros2 launch slop_navigation mapping.launch.py rviz:=false    # ohne RViz
    ros2 launch slop_navigation mapping.launch.py lio:=false     # lio.launch.py läuft schon (dann mit map_tf:=false!)

Bestehende Karte ERWEITERN (weiterkartieren statt neu beginnen, Details: slop_navigation/map_extend.py):

    ros2 launch slop_navigation mapping.launch.py extend:=office-3                   # Startpose wird gesucht
    ros2 launch slop_navigation mapping.launch.py extend:=office-3 x:=1.0 y:=-2.0 yaw:=1.57   # Vermutung vorgeben

    Lokalisierung/Navigation vorher beenden, Roboter beim Start stillstehen lassen. extend_start bestimmt die Startpose
    am aktuellen Scan (um x/y/yaw bzw. die letzte gespeicherte Pose, sonst in der ganzen Karte); erst danach startet
    slam_toolbox mit <karte>.posegraph/.data genau dort. Ist die Pose nicht eindeutig, beendet sich alles mit Meldung.
    Danach unter NEUEM Namen speichern (Original bleibt) und Erfahrung übertragen (STARTBEFEHLE.md).

slam_toolbox besitzt dabei map → odom; lio.launch.py läuft deshalb mit map_tf:=false (die LIO-Brücke liefert nur noch die
Geschwindigkeit an den EKF). Speichern: siehe STARTBEFEHLE.md (map_saver_cli für Nav2, serialize_map für slam_toolbox).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import tempfile

import yaml
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from slop_navigation.map_extend import ExtendError, check_pose_graph, map_base


def slam(params):
    return Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
                parameters=params)


def slam_node(context):
    """Neue Karte: slam_toolbox sofort. Erweitern: erst extend_start (Startpose am Scan), dann slam_toolbox mit dem
    gespeicherten Posegraph genau dort – oder Abbruch der ganzen Launch, wenn keine eindeutige Pose gefunden wurde."""
    params = [os.path.join(get_package_share_directory('slop_navigation'), 'config', 'slam_mapping.yaml')]
    extend = context.launch_configurations['extend'].strip()
    if not extend:
        return [slam(params)]
    base = map_base(extend)
    try:
        check_pose_graph(base)
    except ExtendError as e:
        raise RuntimeError(f'Karte erweitern geht nicht: {e}') from None
    out_file = os.path.join(tempfile.gettempdir(), f'slop_extend_start_{os.getpid()}.yaml')
    finder = Node(package='slop_navigation', executable='extend_start', name='extend_start', output='screen',
                  parameters=[{'map_base': base, 'out_file': out_file, 'x': context.launch_configurations['x'],
                               'y': context.launch_configurations['y'],
                               'yaw': context.launch_configurations['yaw']}])

    def after_finder(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='Karte erweitern: keine sichere Startpose – slam_toolbox startet nicht, alles wird beendet.'),
                    EmitEvent(event=Shutdown(reason='keine Startpose zum Erweitern'))]
        with open(out_file, encoding='utf-8') as f:
            p = yaml.safe_load(f)
        os.remove(out_file)
        return [LogInfo(msg=f'Karte erweitern: {base} ab x {p["x"]:.2f} m, y {p["y"]:.2f} m, Gier {p["yaw"]:.2f} rad. '
                            'Später unter NEUEM Namen speichern.'),
                slam(params + [{'map_file_name': base, 'map_start_pose': [p['x'], p['y'], p['yaw']]}])]

    return [finder, RegisterEventHandler(OnProcessExit(target_action=finder, on_exit=after_finder))]


def generate_launch_description():
    lio = os.path.join(get_package_share_directory('slop_lio'), 'launch', 'lio.launch.py')
    rviz_cfg = os.path.join(get_package_share_directory('slop_description'), 'rviz', 'mapping.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('lio', default_value='true', description='lio.launch.py mitstarten (Lidar, /scan, LIO)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz mit Kartenansicht'),
        DeclareLaunchArgument('extend', default_value='',
                              description='bestehende Karte erweitern: Name in maps/ (z. B. office-3) oder Pfad; '
                                          'leer = neue Karte'),
        DeclareLaunchArgument('x', default_value='',
                              description='Erweitern: vermutete Startpose [m] (sonst letzte gespeicherte; wird am Scan geprüft)'),
        DeclareLaunchArgument('y', default_value='', description='Erweitern: vermutete Startpose [m]'),
        DeclareLaunchArgument('yaw', default_value='', description='Erweitern: vermutete Startpose [rad]'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lio),
                                 launch_arguments={'map_tf': 'false', 'scan': 'true'}.items(),
                                 condition=IfCondition(LaunchConfiguration('lio'))),
        OpaqueFunction(function=slam_node),
        Node(package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
