"""Differentialantrieb starten (bestromt die Motoren!): EtherCAT-Busprozess + ROS-Node.

Der Busprozess braucht Netzwerk- und Echtzeit-Rechte, deshalb über den Wrapper starten:
    ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py
    ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py config:=/pfad/zu.yaml

  ethercat_bus     2-ms-EtherCAT-Zyklus (CL57E-4A), ohne ROS; liest die Parameterdatei selbst
  diff_drive_node  /cmd_vel → Drehzahlen an den Busprozess; /wheel/odom, /joint_states zurück
Beide reden über einen lokalen Unix-Socket (slop_base/bus_link.py). Endet der Busprozess (z. B. EtherCAT verloren),
beendet sich auch die Node.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('slop_base'), 'config', 'diff_drive.yaml')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Parameterdatei des Differentialantriebs (lesen beide Prozesse)'),
        # output both: Terminal und ~/.ros/log/<Startzeit>/launch.log (der Busprozess hat kein ROS-Logging)
        Node(package='slop_base', executable='ethercat_bus', name='ethercat_bus', output='both',
             arguments=['--config', config]),
        Node(package='slop_base', executable='diff_drive', name='diff_drive_node', output='screen',
             parameters=[config]),
    ])
