"""EKF starten (robot_localization): bildet /odom und das TF odom → base_footprint aus allen Quellen.

    ros2 launch slop_localization localization.launch.py
    ros2 launch slop_localization localization.launch.py ekf_config:=/pfad/zu.yaml

Startet auch imu_adapter (Livox-IMU → /imu/livox, Gyro-Bias beim Start: Roboter stillhalten). Läuft der Lidar nicht,
bleibt der Adapter still und der EKF fährt auf den Rädern allein.
Eingänge und ihre Gewichtung stehen nur in config/ekf.yaml. Quellen einbauen: dort einen Block odomN / imuN
ergänzen (Vorlagen stehen in der Datei), sonst nichts ändern.
Braucht: sudo apt install ros-humble-robot-localization
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('slop_localization'), 'config', 'ekf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('ekf_config', default_value=default_config, description='Parameterdatei des EKF'),
        Node(package='slop_localization', executable='imu_adapter', name='imu_adapter', output='screen'),
        # Name muss zum Schlüssel in ekf.yaml passen. Ausgang odometry/filtered → /odom
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[LaunchConfiguration('ekf_config')],
             remappings=[('odometry/filtered', '/odom')]),
    ])
