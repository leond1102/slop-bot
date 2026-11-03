"""Alle Sensoren starten: Livox Mid-360s + RealSense D435i + statische TFs base_footprint→Sensor.

    ros2 launch slop_sensors sensors.launch.py
    ros2 launch slop_sensors sensors.launch.py camera:=false          # nur Lidar
    ros2 launch slop_sensors sensors.launch.py lidar:=false           # nur Kamera
    ros2 launch slop_sensors sensors.launch.py xfer_format:=0         # Lidar als PointCloud2

Montagepositionen stehen in config/mounts.yaml.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BASE_FRAME = 'base_footprint'


def _mount_tfs(context):
    with open(LaunchConfiguration('mounts').perform(context)) as f:
        mounts = yaml.safe_load(f)
    nodes = []
    for name, m in mounts.items():
        if LaunchConfiguration('lidar' if name == 'livox' else name).perform(context).lower() != 'true':
            continue
        args = []
        for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            args += [f'--{key}', str(float(m.get(key, 0.0)))]
        args += ['--frame-id', BASE_FRAME, '--child-frame-id', m['frame']]
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name=f'{name}_mount_tf', arguments=args, output='log'))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('slop_sensors')
    launch_dir = os.path.join(share, 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='Livox Mid-360s starten'),
        DeclareLaunchArgument('camera', default_value='true', description='RealSense D435i starten'),
        DeclareLaunchArgument('xfer_format', default_value='1',
                              description='Lidar: 0 = PointCloud2, 1 = livox CustomMsg'),
        DeclareLaunchArgument('mounts', default_value=os.path.join(share, 'config', 'mounts.yaml'),
                              description='Montagepositionen der Sensoren'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'lidar.launch.py')),
                                 launch_arguments={'xfer_format': LaunchConfiguration('xfer_format')}.items(),
                                 condition=IfCondition(LaunchConfiguration('lidar'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'camera.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('camera'))),
        OpaqueFunction(function=_mount_tfs),
    ])
