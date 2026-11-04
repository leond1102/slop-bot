"""Modell + Lidar-Punktwolke + Kamerabild und -Punktwolke in RViz, ohne Antrieb (Roboter steht, Fixed Frame base_footprint).

    ros2 launch slop_description view.launch.py
    ros2 launch slop_description view.launch.py camera_pointcloud:=false   # Kamera ohne Punktwolke (weniger CPU)

Startet die Sensortreiber (Lidar als PointCloud2), robot_state_publisher, RViz (rviz/sensors.rviz) und einen
joint_state_publisher, der die Räder auf 0 setzt. Nicht zusammen mit dem Antrieb benutzen (zwei Quellen für
/joint_states): Für Fahrt mit Sensoren description.launch.py mit rviz_config:=all nehmen.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = get_package_share_directory('slop_description')
    sensors = get_package_share_directory('slop_sensors')
    return LaunchDescription([
        DeclareLaunchArgument('camera_pointcloud', default_value='true',
                              description='Kamera-Punktwolke /camera/depth/color/points (kostet CPU)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'description.launch.py')),
            launch_arguments={'rviz': 'true', 'rviz_config': 'sensors'}.items()),
        Node(package='joint_state_publisher', executable='joint_state_publisher', output='log'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sensors, 'launch', 'sensors.launch.py')),
            launch_arguments={'xfer_format': '0',
                              'camera_pointcloud': LaunchConfiguration('camera_pointcloud')}.items()),
    ])
