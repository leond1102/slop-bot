"""LiDAR-Inertial-Odometrie: Livox-Treiber + FAST-LIO2 + Brücke zu TF und EKF.

    ros2 launch slop_lio lio.launch.py                  # Lidar + FAST-LIO + Brücke (zusätzlich bringup.launch.py starten)
    ros2 launch slop_lio lio.launch.py lidar:=false     # Lidar läuft schon (sensors.launch.py)
    ros2 launch slop_lio lio.launch.py clouds:=true     # /cloud_registered für RViz
    ros2 launch slop_lio lio.launch.py crop:=false      # ohne Ausschneiden von Körper und Mast

Beim Start den Roboter 2–3 s stillstehen lassen (IMU-Initialisierung von FAST-LIO). Läuft schon ein Livox-Treiber
(sensors.launch.py), lidar:=false setzen, sonst stören sich zwei Treiber. Die Brücke braucht das Modell und den EKF
(bringup.launch.py): TF base_footprint → livox_frame und odom → base_footprint.

Körper und Mast werden vor FAST-LIO aus den Scans geschnitten (Quader aus dem URDF und config/crop.yaml, im Patch von FAST-LIO).

FAST-LIO läuft auf den Kernen cpus (Standard 2–6): Kern 0 hat den EtherCAT-Interrupt, 1 die USB-Interrupts, 7 den
EtherCAT-Zyklus. Alle Parameter von FAST-LIO stehen in config/fast_lio_mid360s.yaml.
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from slop_lio.crop import boxes_from_urdf


def ld_path_without_mvs():
    """LD_LIBRARY_PATH ohne /opt/MVS: ~/.bashrc stellt das Hikrobot-MVS-SDK voran, dessen alte libusb (ohne
    libusb_set_option) sonst statt der System-libusb geladen wird; FAST-LIO (PCL) stirbt dann sofort mit
    "symbol lookup error" (Exit 127)."""
    return ':'.join(p for p in os.environ.get('LD_LIBRARY_PATH', '').split(':') if p and not p.startswith('/opt/MVS'))


def exclude_boxes(share):
    """Quader, die aus den Scans geschnitten werden: Körpermaße aus dem URDF, Zuschläge aus config/crop.yaml."""
    with open(os.path.join(share, 'config', 'crop.yaml'), encoding='utf-8') as f:
        crop = yaml.safe_load(f)['crop']
    urdf = os.path.join(get_package_share_directory('slop_description'), 'urdf', 'slop_bot.urdf.xacro')
    return boxes_from_urdf(xacro.process_file(urdf).toxml(), crop['body_margin_side'], crop['body_margin_up'],
                           tuple(crop['rear_box']))


def lio_nodes(context, config, extrinsic_t):
    """FAST-LIO und Brücke; crop:=true gibt die Quader zum Ausschneiden an FAST-LIO (Parameter) und zur Anzeige an die
    Brücke (leere Listen wären nicht typisierbar, deshalb dann ganz weglassen)."""
    share = get_package_share_directory('slop_lio')
    params = {'publish.scan_publish_en': ParameterValue(LaunchConfiguration('clouds'), value_type=bool)}
    bridge = {'imu_to_lidar_xyz': extrinsic_t}
    if context.launch_configurations['crop'].lower() == 'true':
        boxes = [float(v) for v in exclude_boxes(share)]
        params['preprocess.exclude_boxes'] = boxes
        bridge['crop_boxes'] = boxes
    return [Node(package='fast_lio', executable='fastlio_mapping', name='fast_lio', output='screen',
                 additional_env={'LD_LIBRARY_PATH': ld_path_without_mvs()},
                 prefix=['taskset -c ', LaunchConfiguration('cpus'), ' '],
                 parameters=[config, params]),
            Node(package='slop_lio', executable='lio_bridge', name='lio_bridge', output='screen', parameters=[bridge])]


def generate_launch_description():
    share = get_package_share_directory('slop_lio')
    config = os.path.join(share, 'config', 'fast_lio_mid360s.yaml')
    with open(config, encoding='utf-8') as f:
        extrinsic_t = yaml.safe_load(f)['/**']['ros__parameters']['mapping']['extrinsic_T']
    sensors = os.path.join(get_package_share_directory('slop_sensors'), 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='Livox-Treiber mitstarten'),
        DeclareLaunchArgument('clouds', default_value='false',
                              description='FAST-LIO veröffentlicht /cloud_registered (kostet CPU)'),
        DeclareLaunchArgument('crop', default_value='true',
                              description='Körper und Mast aus den Scans schneiden (config/crop.yaml)'),
        DeclareLaunchArgument('cpus', default_value='2-6', description='Kerne für FAST-LIO (taskset)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sensors, 'lidar.launch.py')),
            launch_arguments={'xfer_format': '1'}.items(),        # FAST-LIO braucht das Livox-CustomMsg
            condition=IfCondition(LaunchConfiguration('lidar'))),
        OpaqueFunction(function=lio_nodes, args=[config, extrinsic_t]),
    ])
