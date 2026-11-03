"""Livox Mid-360s starten (livox_ros_driver2).

    ros2 launch slop_sensors lidar.launch.py
    ros2 launch slop_sensors lidar.launch.py xfer_format:=0   # PointCloud2, z. B. zum Anschauen in RViz

Gerät ist ein Mid-360s (SDK-Typ 35) – daher MID360s_config.json, nicht MID360_config.json!
Topics: /livox/lidar (Punkte), /livox/imu (eingebaute IMU, Beschleunigung in g!)
Netz: Lidar 192.168.1.162, Jetson 192.168.1.50 am USB-LAN-Adapter (NetworkManager-Verbindung "livox-mid360").
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('slop_sensors')
    default_config = os.path.join(share, 'config', 'MID360s_config.json')
    return LaunchDescription([
        DeclareLaunchArgument('data_config', default_value=os.path.join(share, 'config', 'data_reduction.yaml'),
                              description='zentrale Kürzungen der Sensordaten (publish_freq, ...)'),
        DeclareLaunchArgument('lidar_config', default_value=default_config,
                              description='Livox-Netzwerkkonfiguration (JSON)'),
        DeclareLaunchArgument('xfer_format', default_value='1',
                              description='0 = PointCloud2 (RViz), 1 = livox CustomMsg (FAST-LIO)'),
        DeclareLaunchArgument('frame_id', default_value='livox_frame'),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node', name='livox_lidar_publisher',
             output='screen',
             parameters=[{
                 'xfer_format': ParameterValue(LaunchConfiguration('xfer_format'), value_type=int),
                 'multi_topic': 0,
                 'data_src': 0,
                 'output_data_type': 0,
                 'frame_id': LaunchConfiguration('frame_id'),
                 'user_config_path': LaunchConfiguration('lidar_config'),
                 'cmdline_input_bd_code': 'livox0000000001',
             }, LaunchConfiguration('data_config')]),   # publish_freq: zentral in data_reduction.yaml
    ])
