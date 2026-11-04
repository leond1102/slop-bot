"""Alle Sensoren starten: Livox Mid-360s + RealSense D435i (nur die Treiber, keine TFs zwischen den Sensoren).

    ros2 launch slop_sensors sensors.launch.py
    ros2 launch slop_sensors sensors.launch.py camera:=false          # nur Lidar
    ros2 launch slop_sensors sensors.launch.py lidar:=false           # nur Kamera
    ros2 launch slop_sensors sensors.launch.py xfer_format:=0         # Lidar als PointCloud2
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('slop_sensors'), 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='Livox Mid-360s starten'),
        DeclareLaunchArgument('camera', default_value='true', description='RealSense D435i starten'),
        DeclareLaunchArgument('xfer_format', default_value='1',
                              description='Lidar: 0 = PointCloud2, 1 = Livox CustomMsg'),
        DeclareLaunchArgument('camera_pointcloud', default_value='false',
                              description='Kamera: farbige Punktwolke /camera/depth/color/points'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'lidar.launch.py')),
                                 launch_arguments={'xfer_format': LaunchConfiguration('xfer_format')}.items(),
                                 condition=IfCondition(LaunchConfiguration('lidar'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'camera.launch.py')),
                                 launch_arguments={'camera_pointcloud': LaunchConfiguration('camera_pointcloud')}.items(),
                                 condition=IfCondition(LaunchConfiguration('camera'))),
    ])
