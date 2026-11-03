"""Plattform komplett starten: Antrieb + twist_mux + RustDesk-Fahrsperre (bestromt die Motoren!).

    ros2 run slop_base with_netcaps ros2 launch slop_base bringup.launch.py

Fahren dann nur über twist_mux, z. B.:
    ros2 run slop_teleop keyboard_teleop
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base = get_package_share_directory('slop_base')
    safety = get_package_share_directory('slop_safety')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(safety, 'launch', 'safety.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(base, 'launch', 'diff_drive.launch.py'))),
    ])
