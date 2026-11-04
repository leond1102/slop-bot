"""Plattform komplett starten: Antrieb + twist_mux + RustDesk-Fahrsperre + Robotermodell (TF) + EKF
(bestromt die Motoren!).

    ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py
    ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py ekf:=false   # ohne EKF, dann kein /odom

Fahren dann nur über twist_mux, z. B.:
    ros2 run slop_teleop keyboard_teleop

Das Modell (robot_state_publisher) liefert das TF base_footprint → Sensoren, das der EKF für die IMU braucht. RViz dazu:
    ros2 launch slop_description rviz.launch.py
Fehlt robot_localization (sudo apt install ros-humble-robot-localization), startet alles andere trotzdem, nur ohne EKF
und damit ohne /odom.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = get_package_share_directory('slop_base')
    safety = get_package_share_directory('slop_safety')
    description = get_package_share_directory('slop_description')
    localization = get_package_share_directory('slop_localization')
    try:
        get_package_share_directory('robot_localization')
        ekf_missing = False
    except PackageNotFoundError:
        ekf_missing = True

    actions = [
        DeclareLaunchArgument('ekf', default_value='true',
                              description='EKF starten (/odom und TF odom → base_footprint aus allen Quellen)'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(safety, 'launch', 'safety.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(base, 'launch', 'diff_drive.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(description, 'launch', 'description.launch.py'))),
    ]
    if ekf_missing:
        actions.append(LogInfo(msg='WARNUNG: robot_localization fehlt (sudo apt install ros-humble-robot-localization) '
                                   '– EKF nicht gestartet, es gibt kein /odom und kein TF odom → base_footprint'))
    else:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'localization.launch.py')),
            condition=IfCondition(LaunchConfiguration('ekf'))))
    return LaunchDescription(actions)
