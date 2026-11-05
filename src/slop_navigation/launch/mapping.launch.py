"""Karte aufnehmen mit slam_toolbox: LIO (Lidar, /scan, EKF-Geschwindigkeit) + slam_toolbox + RViz.

    ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py    # Terminal 1: Antrieb, Modell, EKF
    ros2 launch slop_navigation mapping.launch.py                                 # Terminal 2: dieses
    ros2 run slop_teleop keyboard_teleop                                          # Terminal 3: fahren

    ros2 launch slop_navigation mapping.launch.py rviz:=false    # ohne RViz
    ros2 launch slop_navigation mapping.launch.py lio:=false     # lio.launch.py läuft schon (dann mit map_tf:=false!)

slam_toolbox besitzt dabei map → odom; lio.launch.py läuft deshalb mit map_tf:=false (die LIO-Brücke liefert nur noch die
Geschwindigkeit an den EKF). Speichern: siehe STARTBEFEHLE.md (map_saver_cli für Nav2, serialize_map für slam_toolbox).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('slop_navigation')
    lio = os.path.join(get_package_share_directory('slop_lio'), 'launch', 'lio.launch.py')
    rviz_cfg = os.path.join(get_package_share_directory('slop_description'), 'rviz', 'mapping.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('lio', default_value='true', description='lio.launch.py mitstarten (Lidar, /scan, LIO)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz mit Kartenansicht'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lio),
                                 launch_arguments={'map_tf': 'false', 'scan': 'true'}.items(),
                                 condition=IfCondition(LaunchConfiguration('lio'))),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             parameters=[os.path.join(share, 'config', 'slam_mapping.yaml')]),
        Node(package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
