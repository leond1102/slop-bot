"""Nur RViz mit einer Konfiguration aus rviz/ öffnen, ohne robot_state_publisher.

    ros2 launch slop_description rviz.launch.py                      # all: Modell, Odometrie, Lidar, Kamera (Fixed Frame odom)
    ros2 launch slop_description rviz.launch.py rviz_config:=sensors

Für den Betrieb neben bringup.launch.py, das das Modell schon veröffentlicht (sonst gäbe es zwei robot_state_publisher).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('slop_description')
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value='all',
                              description='Name der Konfiguration in rviz/ ohne .rviz (description, all, sensors, lio)'),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', [LaunchConfiguration('rviz_config'), '.rviz']])]),
    ])
