"""Alle Sensoren starten: Livox Mid-360s + RealSense D435i + statische TFs base_footprint→Sensor.

    ros2 launch slop_sensors sensors.launch.py
    ros2 launch slop_sensors sensors.launch.py camera:=false          # nur Lidar
    ros2 launch slop_sensors sensors.launch.py lidar:=false           # nur Kamera
    ros2 launch slop_sensors sensors.launch.py xfer_format:=0         # Lidar als PointCloud2
    ros2 launch slop_sensors sensors.launch.py mount_gravity_on_start:=true   # Neigung beim Start messen

Montagepositionen stehen in config/mounts.yaml (x/y/z). Die Drehung kommt aus der gespeicherten Kalibrierung
~/slop-bot_ws/calib/mounts_lio.yaml, falls vorhanden, sonst aus mounts.yaml. Beim Start wird nichts gemessen.
Neigung neu messen (Roboter still und waagrecht): ros2 service call /mount_tf/recalibrate std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def _mount_tf(context):
    # Nur die Sensoren, die auch gestartet werden
    sensors = [name for name, arg in (('livox', 'lidar'), ('camera', 'camera'))
               if LaunchConfiguration(arg).perform(context).lower() == 'true']
    if not sensors:
        return []
    data_config = LaunchConfiguration('data_config').perform(context)
    # IMUs für mount_tf drosseln (200 Hz → ~25 Hz). C++-Komponenten in einem Container: 200 Hz in Python zu
    # empfangen kostete mount_tf ~45 % eines Kerns. Topics: /livox/imu/slow, /camera/imu/slow
    imu_topics = {'livox': '/livox/imu', 'camera': '/camera/imu'}
    throttles = [ComposableNode(package='slop_sensors', plugin='slop_sensors::ImuThrottle',
                                name=f'{name}_imu_throttle', parameters=[data_config],
                                remappings=[('imu_in', imu_topics[name]), ('imu_out', imu_topics[name] + '/slow')],
                                extra_arguments=[{'use_intra_process_comms': True}])
                 for name in sensors]
    helpers = ComposableNodeContainer(name='sensor_helpers', namespace='', package='rclcpp_components',
                                      executable='component_container', composable_node_descriptions=throttles,
                                      output='screen')
    return [helpers, Node(package='slop_sensors', executable='mount_tf', name='mount_tf', output='screen',
                 parameters=[LaunchConfiguration('data_config').perform(context), {
                     'mounts_file': LaunchConfiguration('mounts').perform(context),
                     'sensors': sensors,
                     'gravity_on_start': LaunchConfiguration('mount_gravity_on_start').perform(context).lower() == 'true',
                     'calib_file': LaunchConfiguration('mount_calib_file').perform(context),
                 }])]


def _check_not_running(context):
    """Abbrechen, wenn schon ein Sensor-Stack läuft (z. B. view.launch.py neben lio.launch.py).
    Zwei Livox-Treiber am selben Lidar nehmen sich die Daten weg, zwei mount_tf senden konkurrierende TFs."""
    if LaunchConfiguration('check_running').perform(context).lower() != 'true':
        return []
    import time
    import rclpy
    from rclpy.context import Context
    ctx = Context()
    rclpy.init(context=ctx)
    try:
        node = rclpy.create_node('sensors_launch_check', context=ctx)
        time.sleep(2.5)   # Discovery abwarten
        running = {f'{ns.rstrip("/")}/{n}' for n, ns in node.get_node_names_and_namespaces()}
        node.destroy_node()
    finally:
        rclpy.shutdown(context=ctx)
    wanted = {'/mount_tf': 'mount_tf'}
    if LaunchConfiguration('lidar').perform(context).lower() == 'true' and \
            LaunchConfiguration('lidar_driver').perform(context).lower() == 'true':
        wanted['/livox_lidar_publisher'] = 'Livox-Treiber'
    if LaunchConfiguration('camera').perform(context).lower() == 'true':
        wanted['/camera'] = 'RealSense'
    found = [label for name, label in wanted.items() if name in running]
    if found:
        raise RuntimeError(
            f'Sensoren laufen schon ({", ".join(found)}) – vermutlich ein zweiter Launch (sensors/view/lio). '
            'Zwei Treiber am selben Gerät stören sich. Den anderen erst beenden. '
            'Alles zusammen ansehen: ros2 launch slop_slam lio.launch.py '
            '(oder mit laufenden Sensoren: lio.launch.py sensors:=false). Prüfung aus: check_running:=false')
    return []


def generate_launch_description():
    share = get_package_share_directory('slop_sensors')
    launch_dir = os.path.join(share, 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='Livox Mid-360s starten'),
        DeclareLaunchArgument('camera', default_value='true', description='RealSense D435i starten'),
        DeclareLaunchArgument('lidar_driver', default_value='true',
                              description='Livox-Treiber hier starten (false: läuft woanders, z. B. im LIO-Container; '
                                          'Montage-TF und Schwerkraft des Lidars laufen trotzdem)'),
        DeclareLaunchArgument('xfer_format', default_value='1',
                              description='Lidar: 0 = PointCloud2, 1 = livox CustomMsg'),
        DeclareLaunchArgument('camera_pointcloud', default_value='false',
                              description='Kamera: farbige Punktwolke /camera/depth/color/points'),
        DeclareLaunchArgument('mount_gravity_on_start', default_value='false',
                              description='roll/pitch bei jedem Start aus der IMU-Schwerkraft messen (sonst nur über '
                                          '/mount_tf/recalibrate)'),
        DeclareLaunchArgument('mount_calib_file', default_value=os.path.expanduser('~/slop-bot_ws/calib/mounts_lio.yaml'),
                              description='gespeicherte Montage (mount_tf/recalibrate, slop_slam/mount_calib), '
                                          'wird geladen falls vorhanden ("" = nicht laden und nicht speichern)'),
        DeclareLaunchArgument('data_config', default_value=os.path.join(share, 'config', 'data_reduction.yaml'),
                              description='zentrale Kürzungen der Sensordaten (Auflösung, Raten, Ausdünnung)'),
        DeclareLaunchArgument('mounts', default_value=os.path.join(share, 'config', 'mounts.yaml'),
                              description='Montagepositionen der Sensoren'),
        DeclareLaunchArgument('check_running', default_value='true',
                              description='abbrechen, wenn schon Sensoren laufen (zweiter Launch)'),
        OpaqueFunction(function=_check_not_running),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'lidar.launch.py')),
                                 launch_arguments={'xfer_format': LaunchConfiguration('xfer_format'),
                                                   'data_config': LaunchConfiguration('data_config')}.items(),
                                 condition=IfCondition(PythonExpression([
                                     "'", LaunchConfiguration('lidar'), "' == 'true' and '",
                                     LaunchConfiguration('lidar_driver'), "' == 'true'"]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'camera.launch.py')),
                                 launch_arguments={'camera_pointcloud': LaunchConfiguration('camera_pointcloud'),
                                                   'data_config': LaunchConfiguration('data_config')}.items(),
                                 condition=IfCondition(LaunchConfiguration('camera'))),
        OpaqueFunction(function=_mount_tf),
    ])
