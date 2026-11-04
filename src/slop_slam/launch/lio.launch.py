"""LiDAR-Inertial-Odometrie (FAST-LIO2) mit dem Livox Mid-360s, optional mit RViz.

    ros2 launch slop_slam lio.launch.py                  # ALLES: Sensoren + FAST-LIO + Montage + RViz
                                                         # (nicht zusätzlich view.launch.py starten!)
    ros2 launch slop_slam lio.launch.py rviz:=false
    ros2 launch slop_slam lio.launch.py camera:=false    # ohne RealSense (spart CPU/USB)
    ros2 launch slop_slam lio.launch.py sensors:=false   # Sensoren laufen schon (mit xfer_format 1!)

Beim Start Roboter 2–3 s STILL stehen lassen (IMU-Initialisierung von FAST-LIO). Die Sensor-Neigung wird beim
Start nicht gemessen (gespeichert in ~/slop-bot_ws/calib/mounts_lio.yaml), neu messen auf Anforderung:
    ros2 service call /mount_tf/recalibrate std_srvs/srv/Trigger      (Roboter still und waagrecht)
map = Roboterlage beim Start. Die Roboterpose kommt allein aus LIO (keine Radodometrie): lio_tf sendet
map → odom (Identität) und odom → base_footprint, bis jetzt vorausgerechnet (siehe slop_slam/lio_tf_node.py).

Karte speichern (PCD):   ros2 service call /lio_map/save std_srvs/srv/Trigger

Montage kalibrieren (nur bei Bedarf, kostet CPU):   ros2 launch slop_slam lio.launch.py calibrate:=true
mount_calib führt dann die Montage nach (RealSense per ICP an die Lidar-Karte, Lidar-yaw aus Geradeausfahrt
mit Fahrbefehlen auf /cmd_vel) und speichert sie in ~/slop-bot_ws/calib/mounts_lio.yaml. Ohne calibrate lädt
mount_tf diese Datei.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def ld_path_without_mvs():
    """LD_LIBRARY_PATH ohne /opt/MVS: ~/.bashrc stellt das Hikrobot-MVS-SDK voran, dessen alte libusb
    (ohne libusb_set_option) sonst statt der System-libusb geladen wird (PCL/FAST-LIO stürzt ab)."""
    return ':'.join(p for p in os.environ.get('LD_LIBRARY_PATH', '').split(':') if p and not p.startswith('/opt/MVS'))


def generate_launch_description():
    share = get_package_share_directory('slop_slam')
    sensors_share = get_package_share_directory('slop_sensors')
    # zweite volle Wolke (/cloud_registered_body) nur senden, wenn mount_calib sie braucht
    lio_overrides = {'map_file_path': LaunchConfiguration('map_file'),
                     'publish.scan_bodyframe_pub_en': ParameterValue(LaunchConfiguration('calibrate'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('sensors', default_value='true', description='slop_sensors mitstarten'),
        DeclareLaunchArgument('camera', default_value='true', description='RealSense mitstarten'),
        DeclareLaunchArgument('camera_pointcloud', default_value=LaunchConfiguration('rviz'),
                              description='farbige Kamera-Punktwolke (kostet CPU), Standard: an, wenn RViz läuft'),
        DeclareLaunchArgument('composable', default_value='true',
                              description='Livox-Treiber und FAST-LIO in einem Container (Punktwolken ohne Serialisierung); '
                                          'false = getrennte Prozesse wie früher'),
        DeclareLaunchArgument('calibrate', default_value='false',
                              description='Montage von RealSense (ICP) und Lidar-yaw (Geradeausfahrt) nachführen. '
                                          'Braucht /cloud_registered_body (FAST-LIO sendet es nur dann) und CPU'),
        DeclareLaunchArgument('lio_config', default_value=os.path.join(share, 'config', 'fast_lio_mid360s.yaml')),
        DeclareLaunchArgument('data_config', default_value=os.path.join(sensors_share, 'config', 'data_reduction.yaml'),
                              description='zentrale Kürzungen der Sensordaten (Auflösung, Raten, Ausdünnung)'),
        DeclareLaunchArgument('map_file', default_value=os.path.expanduser('~/slop-bot_ws/maps/lio_map.pcd'),
                              description='Ziel für /lio_map/save'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'lio.rviz')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sensors_share, 'launch', 'sensors.launch.py')),
            launch_arguments={'lidar': 'true', 'camera': LaunchConfiguration('camera'),
                              # im Container-Modus startet lio.launch.py den Livox-Treiber selbst
                              'lidar_driver': PythonExpression(["'false' if '", LaunchConfiguration('composable'),
                                                                "' == 'true' else 'true'"]),
                              'camera_pointcloud': LaunchConfiguration('camera_pointcloud'),
                              'data_config': LaunchConfiguration('data_config'),
                              'xfer_format': '1'}.items(),     # FAST-LIO braucht das Livox-CustomMsg
            condition=IfCondition(LaunchConfiguration('sensors'))),

        # getrennte Prozesse (composable:=false)
        Node(package='fast_lio', executable='fastlio_mapping', name='fast_lio', output='screen',
             additional_env={'LD_LIBRARY_PATH': ld_path_without_mvs()},
             parameters=[LaunchConfiguration('lio_config'), LaunchConfiguration('data_config'),
                         lio_overrides],
             condition=UnlessCondition(LaunchConfiguration('composable'))),
        # Container (composable:=true): Livox-Treiber + FAST-LIO, /livox/lidar und /livox/imu intra-process.
        # Multithreaded (2 Threads), damit FAST-LIOs Rechenschleife den Treiber nicht aufhält. Nachteil: Stürzt einer ab
        # (der Livox-Treiber beim Beenden), endet der ganze Container – im Betrieb unkritisch.
        ComposableNodeContainer(
            name='lio_container', namespace='', package='rclcpp_components', executable='component_container_mt',
            output='screen', additional_env={'LD_LIBRARY_PATH': ld_path_without_mvs()},
            # 2 Threads: FAST-LIO (eine Callback-Gruppe) + Livox-Treiber. Standard wäre einer je Kern (8):
            # kostete ~20 % CPU nur fürs gegenseitige Aufwecken
            parameters=[{'thread_num': 2}],
            composable_node_descriptions=[
                ComposableNode(package='livox_ros_driver2', plugin='livox_ros::DriverNode',
                               name='livox_lidar_publisher',
                               parameters=[{
                                   'xfer_format': 1,          # CustomMsg für FAST-LIO
                                   'multi_topic': 0,
                                   'data_src': 0,
                                   'output_data_type': 0,
                                   'frame_id': 'livox_frame',
                                   'user_config_path': os.path.join(sensors_share, 'config', 'MID360s_config.json'),
                                   'cmdline_input_bd_code': 'livox0000000001',
                               }, LaunchConfiguration('data_config')],   # publish_freq
                               extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(package='fast_lio', plugin='LaserMappingNode', name='fast_lio',
                               parameters=[LaunchConfiguration('lio_config'), LaunchConfiguration('data_config'),
                                           lio_overrides],
                               extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            condition=IfCondition(LaunchConfiguration('composable'))),
        Node(package='slop_slam', executable='lio_tf', name='lio_tf', output='screen'),
        Node(package='slop_slam', executable='lio_map', name='lio_map', output='screen',
             parameters=[LaunchConfiguration('data_config'), {'map_file': LaunchConfiguration('map_file')}]),
        Node(package='slop_slam', executable='mount_calib', name='mount_calib', output='screen',
             additional_env={'OMP_NUM_THREADS': '2'},   # open3d-ICP höchstens 2 Kerne (sonst kurz alle 8, stört EtherCAT)
             parameters=[LaunchConfiguration('data_config'),
                         {'camera': ParameterValue(LaunchConfiguration('camera'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('calibrate'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
