"""twist_mux und RustDesk-Fahrsperre starten.

    ros2 launch slop_safety safety.launch.py
Teil von slop_base/launch/bringup.launch.py, einzeln nur zum Testen nötig.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('slop_safety')
    return LaunchDescription([
        DeclareLaunchArgument('twist_mux_config', default_value=os.path.join(share, 'config', 'twist_mux.yaml'),
                              description='Eingänge und Sperren von twist_mux'),
        # teleop/cmd_vel, navigation/cmd_vel + Sperren safety/* → /cmd_vel
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[LaunchConfiguration('twist_mux_config')],
             remappings=[('cmd_vel_out', '/cmd_vel')]),
        # RustDesk-Sitzung → safety/remote_lock
        Node(package='slop_safety', executable='remote_guard', name='remote_guard', output='screen'),
    ])
