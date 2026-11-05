"""Lokalisieren in einer gespeicherten Karte: LIO (Lidar, /scan, EKF-Geschwindigkeit) + map_server + AMCL + RViz.

    ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py    # Terminal 1: Antrieb, Modell, EKF
    ros2 launch slop_navigation localization.launch.py                            # Terminal 2: dieses
    ros2 run slop_teleop keyboard_teleop                                          # Terminal 3: fahren

    ros2 launch slop_navigation localization.launch.py map:=$HOME/slop-bot_ws/maps/office.yaml   # voller Pfad, kein ~
    ros2 launch slop_navigation localization.launch.py x:=2.0 y:=-1.0 yaw:=1.57   # Startpose in der Karte
    ros2 launch slop_navigation localization.launch.py rviz:=false lio:=false

Startpose: Standard (0, 0, 0) = die Stelle und Richtung, an der das Kartieren begann. Steht der Roboter woanders, die
Pose mitgeben oder in RViz mit "2D Pose Estimate" setzen. AMCL sendet map → odom; lio.launch.py läuft deshalb mit
map_tf:=false.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('slop_navigation')
    params = os.path.join(share, 'config', 'localization.yaml')
    lio = os.path.join(get_package_share_directory('slop_lio'), 'launch', 'lio.launch.py')
    rviz_cfg = os.path.join(get_package_share_directory('slop_description'), 'rviz', 'localization.rviz')
    default_map = os.path.join(os.path.expanduser('~'), 'slop-bot_ws', 'maps', 'office.yaml')

    def num(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Karte (.yaml von map_saver_cli)'),
        DeclareLaunchArgument('x', default_value='0.0', description='Startpose x [m] in der Karte'),
        DeclareLaunchArgument('y', default_value='0.0', description='Startpose y [m] in der Karte'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Startpose Gier [rad] in der Karte'),
        DeclareLaunchArgument('lio', default_value='true', description='lio.launch.py mitstarten (Lidar, /scan, LIO)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz mit Lokalisierungsansicht'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lio),
                                 launch_arguments={'map_tf': 'false', 'scan': 'true'}.items(),
                                 condition=IfCondition(LaunchConfiguration('lio'))),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, {'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'initial_pose.x': num('x'), 'initial_pose.y': num('y'),
                                  'initial_pose.yaw': num('yaw')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen', parameters=[params]),
        Node(package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
