"""Sensoren starten und in RViz anschauen.

    ros2 launch slop_sensors view.launch.py
    ros2 launch slop_sensors view.launch.py camera:=false          # nur Lidar
    ros2 launch slop_sensors view.launch.py camera_pointcloud:=false

Fixed Frame ist base_footprint: Ansicht fest am Roboter, die Welt dreht sich beim Fahren. Ohne LIO gibt es keine
Roboterpose (keine Radodometrie) und damit kein odom/map. Welt fest: ros2 launch slop_slam lio.launch.py

Startet sensors.launch.py mit dem Lidar als PointCloud2 (xfer_format 0), weil RViz das
Livox-CustomMsg-Format nicht anzeigen kann. Für FAST-LIO später wieder sensors.launch.py (CustomMsg).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('slop_sensors')
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('mount_gravity_on_start', default_value='false'),
        DeclareLaunchArgument('camera_pointcloud', default_value='true',
                              description='Farbige Kamera-Punktwolke in RViz'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'sensors.rviz')),
        DeclareLaunchArgument('fixed_frame', default_value='base_footprint',
                              description='base_footprint = Roboter fest; odom/map gibt es nur mit LIO (lio.launch.py)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sensors.launch.py')),
            launch_arguments={
                'lidar': LaunchConfiguration('lidar'),
                'camera': LaunchConfiguration('camera'),
                'camera_pointcloud': LaunchConfiguration('camera_pointcloud'),
                'mount_gravity_on_start': LaunchConfiguration('mount_gravity_on_start'),
                'xfer_format': '0',
            }.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', LaunchConfiguration('rviz_config'), '-f', LaunchConfiguration('fixed_frame')]),
    ])
