"""RealSense D435i starten (realsense2_camera).

    ros2 launch slop_sensors camera.launch.py
    ros2 launch slop_sensors camera.launch.py camera_config:=/pfad/zu.yaml

Topics u. a.: /camera/color/image_raw, /camera/depth/image_rect_raw,
/camera/aligned_depth_to_color/image_raw
(IMU der Kamera ist aus, siehe config/realsense.yaml)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('slop_sensors'), 'config', 'realsense.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_config', default_value=default_config,
                              description='Parameterdatei der RealSense'),
        Node(package='realsense2_camera', executable='realsense2_camera_node', name='camera', namespace='',
             output='screen', parameters=[LaunchConfiguration('camera_config')]),
    ])
