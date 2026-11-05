"""Nav2 für den Slop-Bot: Lokalisierung (map_server + AMCL + relocalize, LIO) + Planer, Controller, Costmaps,
Verhaltensbaum, Geschwindigkeitsglättung und Collision Monitor + RViz.

    ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py    # Terminal 1: Antrieb, Modell, EKF (ZUERST)
    ros2 launch slop_sensors camera.launch.py                                     # Terminal 2: Kamera + Hinderniserkennung
    ros2 launch slop_navigation navigation.launch.py                              # Terminal 3: dieses
    (Ziel in RViz mit "Nav2 Goal"; Tastatur in einem 4. Terminal hat immer Vorrang)

    ros2 launch slop_navigation navigation.launch.py map:=$HOME/slop-bot_ws/maps/office.yaml
    ros2 launch slop_navigation navigation.launch.py localization:=false    # localization.launch.py läuft schon
    ros2 launch slop_navigation navigation.launch.py experience_costs:=true # Erfahrungskarte beeinflusst den Planer

Befehlskette: controller_server/behavior_server → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor
→ /navigation/cmd_vel → twist_mux (Priorität 20, Tastatur 50, Sperren 100) → /cmd_vel → diff_drive_node.
Einstellungen: config/nav2.yaml, Verhaltensbäume: behavior_trees/ (ohne Rückwärtsfahren).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother',
             'collision_monitor']


def generate_launch_description():
    share = get_package_share_directory('slop_navigation')
    params = os.path.join(share, 'config', 'nav2.yaml')
    trees = os.path.join(share, 'behavior_trees')
    rviz_cfg = os.path.join(get_package_share_directory('slop_description'), 'rviz', 'navigation.rviz')
    default_map = os.path.join(os.path.expanduser('~'), 'slop-bot_ws', 'maps', 'office-3.yaml')
    nav_cmd = [('cmd_vel', 'cmd_vel_nav')]
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Karte (.yaml), voller Pfad'),
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
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params], remappings=nav_cmd),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[params], remappings=nav_cmd),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params, {
                 'default_nav_to_pose_bt_xml': os.path.join(trees, 'navigate_to_pose.xml'),
                 'default_nav_through_poses_bt_xml': os.path.join(trees, 'navigate_through_poses.xml')}]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[params],
             remappings=nav_cmd + [('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{'autostart': True, 'node_names': NAV_NODES, 'bond_timeout': 4.0}]),
        Node(package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
