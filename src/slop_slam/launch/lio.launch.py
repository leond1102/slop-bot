"""LiDAR-Inertial-Odometrie (FAST-LIO2) mit dem Livox Mid-360s, optional mit RViz.

    ros2 launch slop_slam lio.launch.py                  # Sensoren + FAST-LIO + lio_tf + RViz
    ros2 launch slop_slam lio.launch.py rviz:=false
    ros2 launch slop_slam lio.launch.py camera:=false    # ohne RealSense (spart CPU/USB)
    ros2 launch slop_slam lio.launch.py sensors:=false   # Sensoren laufen schon (mit xfer_format 1!)

Beim Start Roboter 2–3 s STILL und waagrecht stehen lassen (IMU-Initialisierung, Montage aus der Schwerkraft).
map = Roboterlage beim Start. Läuft slop_base (Radodometrie), korrigiert LIO sie über map → odom,
sonst liefert LIO odom → base_footprint selbst (siehe slop_slam/lio_tf_node.py).

Karte speichern (PCD):   ros2 service call /map_save std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def ld_path_without_mvs():
    """LD_LIBRARY_PATH ohne /opt/MVS: ~/.bashrc stellt das Hikrobot-MVS-SDK voran, dessen alte libusb
    (ohne libusb_set_option) sonst statt der System-libusb geladen wird (PCL/FAST-LIO stürzt ab)."""
    return ':'.join(p for p in os.environ.get('LD_LIBRARY_PATH', '').split(':') if p and not p.startswith('/opt/MVS'))


def generate_launch_description():
    share = get_package_share_directory('slop_slam')
    sensors_share = get_package_share_directory('slop_sensors')
    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true', description='slop_sensors mitstarten'),
        DeclareLaunchArgument('camera', default_value='true', description='RealSense mitstarten'),
        DeclareLaunchArgument('camera_pointcloud', default_value='false',
                              description='farbige Kamera-Punktwolke (kostet CPU)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('lio_config', default_value=os.path.join(share, 'config', 'fast_lio_mid360s.yaml')),
        DeclareLaunchArgument('map_file', default_value=os.path.expanduser('~/slop-bot_ws/maps/lio_map.pcd'),
                              description='Ziel für /map_save'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'lio.rviz')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sensors_share, 'launch', 'sensors.launch.py')),
            launch_arguments={'lidar': 'true', 'camera': LaunchConfiguration('camera'),
                              'camera_pointcloud': LaunchConfiguration('camera_pointcloud'),
                              'xfer_format': '1'}.items(),     # FAST-LIO braucht das Livox-CustomMsg
            condition=IfCondition(LaunchConfiguration('sensors'))),

        Node(package='fast_lio', executable='fastlio_mapping', name='fast_lio', output='screen',
             additional_env={'LD_LIBRARY_PATH': ld_path_without_mvs()},
             parameters=[LaunchConfiguration('lio_config'), {'map_file_path': LaunchConfiguration('map_file')}]),
        Node(package='slop_slam', executable='lio_tf', name='lio_tf', output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
