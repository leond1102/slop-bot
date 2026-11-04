"""Differentialantrieb starten (bestromt die Motoren!).

Braucht Netzwerk-Capabilities für EtherCAT, deshalb über den Wrapper starten:
    ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py
    ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py config:=/pfad/zu.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('slop_base'), 'config', 'diff_drive.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Parameterdatei des Differentialantriebs'),
        # /cmd_vel → Räder, /odom + TF odom→base_footprint + /joint_states zurück
        Node(package='slop_base', executable='diff_drive', name='diff_drive_node', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
