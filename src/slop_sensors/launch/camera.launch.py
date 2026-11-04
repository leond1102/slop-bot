"""RealSense D435i starten (realsense2_camera).

    ros2 launch slop_sensors camera.launch.py
    ros2 launch slop_sensors camera.launch.py camera_config:=/pfad/zu.yaml
    ros2 launch slop_sensors camera.launch.py camera_pointcloud:=true   # mit Punktwolke, Tiefe auf 1/3 verkleinert

Topics u. a.: /camera/color/image_raw, /camera/depth/image_rect_raw,
/camera/imu (Gyro + Accel, braucht den RSUSB-Build, siehe config/realsense.yaml)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def ld_path_without_mvs():
    """LD_LIBRARY_PATH ohne /opt/MVS: ~/.bashrc stellt das Hikrobot-MVS-SDK voran, dessen alte libusb
    (ohne libusb_set_option) sonst statt der System-libusb geladen wird (symbol lookup error)."""
    return ':'.join(p for p in os.environ.get('LD_LIBRARY_PATH', '').split(':') if p and not p.startswith('/opt/MVS'))


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('slop_sensors'), 'config', 'realsense.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_config', default_value=default_config,
                              description='Parameterdatei der RealSense'),
        DeclareLaunchArgument('camera_pointcloud', default_value='false',
                              description='Farbige Punktwolke /camera/depth/color/points (kostet CPU)'),
        Node(package='realsense2_camera', executable='realsense2_camera_node', name='camera', namespace='',
             output='screen', additional_env={'LD_LIBRARY_PATH': ld_path_without_mvs()},
             parameters=[LaunchConfiguration('camera_config'),
                         # mit Punktwolke auch die Tiefe verkleinern (decimation_filter.filter_magnitude in der Config)
                         {'pointcloud__neon_.enable': ParameterValue(LaunchConfiguration('camera_pointcloud'), value_type=bool),
                          'decimation_filter.enable': ParameterValue(LaunchConfiguration('camera_pointcloud'), value_type=bool)}]),
    ])
