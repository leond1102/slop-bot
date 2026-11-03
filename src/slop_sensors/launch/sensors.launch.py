"""Alle Sensoren starten: Livox Mid-360s + RealSense D435i + statische TFs base_footprint→Sensor.

    ros2 launch slop_sensors sensors.launch.py
    ros2 launch slop_sensors sensors.launch.py camera:=false          # nur Lidar
    ros2 launch slop_sensors sensors.launch.py lidar:=false           # nur Kamera
    ros2 launch slop_sensors sensors.launch.py xfer_format:=0         # Lidar als PointCloud2
    ros2 launch slop_sensors sensors.launch.py mount_from_gravity:=false   # Neigung nur aus mounts.yaml

Montagepositionen stehen in config/mounts.yaml. roll/pitch misst mount_tf beim Start aus der Schwerkraft
(Roboter still und waagrecht halten), x/y/z/yaw kommen aus mounts.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _mount_tf(context):
    # Nur die Sensoren, die auch gestartet werden
    sensors = [name for name, arg in (('livox', 'lidar'), ('camera', 'camera'))
               if LaunchConfiguration(arg).perform(context).lower() == 'true']
    if not sensors:
        return []
    return [Node(package='slop_sensors', executable='mount_tf', name='mount_tf', output='screen',
                 parameters=[{
                     'mounts_file': LaunchConfiguration('mounts').perform(context),
                     'sensors': sensors,
                     'gravity': LaunchConfiguration('mount_from_gravity').perform(context).lower() == 'true',
                 }])]


def generate_launch_description():
    share = get_package_share_directory('slop_sensors')
    launch_dir = os.path.join(share, 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='Livox Mid-360s starten'),
        DeclareLaunchArgument('camera', default_value='true', description='RealSense D435i starten'),
        DeclareLaunchArgument('xfer_format', default_value='1',
                              description='Lidar: 0 = PointCloud2, 1 = livox CustomMsg'),
        DeclareLaunchArgument('camera_pointcloud', default_value='false',
                              description='Kamera: farbige Punktwolke /camera/depth/color/points'),
        DeclareLaunchArgument('mount_from_gravity', default_value='true',
                              description='roll/pitch der Sensoren beim Start aus der IMU-Schwerkraft messen'),
        DeclareLaunchArgument('mounts', default_value=os.path.join(share, 'config', 'mounts.yaml'),
                              description='Montagepositionen der Sensoren'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'lidar.launch.py')),
                                 launch_arguments={'xfer_format': LaunchConfiguration('xfer_format')}.items(),
                                 condition=IfCondition(LaunchConfiguration('lidar'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'camera.launch.py')),
                                 launch_arguments={'camera_pointcloud': LaunchConfiguration('camera_pointcloud')}.items(),
                                 condition=IfCondition(LaunchConfiguration('camera'))),
        OpaqueFunction(function=_mount_tf),
    ])
