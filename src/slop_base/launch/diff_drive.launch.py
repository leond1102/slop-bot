"""Differentialantrieb und Greifer starten (bestromt die Motoren!): EtherCAT-Busprozess + ROS-Nodes.

Der Busprozess braucht Netzwerk- und Echtzeit-Rechte, deshalb über den Wrapper starten:
    ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py
    ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py config:=/pfad/zu.yaml
    ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py gripper:=false   # ohne Greifer

  ethercat_bus     2-ms-EtherCAT-Zyklus (CL57E-4A), ohne ROS; liest die Parameterdateien selbst
  diff_drive_node  /cmd_vel → Drehzahlen an den Busprozess; /wheel/odom, /joint_states zurück
  gripper_node     gripper/home|grip|move|open → Positionsfahrten der Greifer-Achsen 1 und 2
Alle reden über einen lokalen Unix-Socket (slop_base/bus_link.py). Endet der Busprozess (z. B. EtherCAT verloren),
beenden sich auch die Nodes. Ein Fehler am Greifer schaltet die Räder nicht ab.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    config = LaunchConfiguration('config').perform(context)
    gripper = LaunchConfiguration('gripper').perform(context).lower() in ('true', '1', 'yes')
    gripper_config = LaunchConfiguration('gripper_config').perform(context)
    bus_args = ['--config', config] + (['--gripper-config', gripper_config] if gripper else [])
    nodes = [
        # output both: Terminal und ~/.ros/log/<Startzeit>/launch.log (der Busprozess hat kein ROS-Logging)
        Node(package='slop_base', executable='ethercat_bus', name='ethercat_bus', output='both', arguments=bus_args),
        Node(package='slop_base', executable='diff_drive', name='diff_drive_node', output='screen',
             parameters=[config]),
    ]
    if gripper:
        nodes.append(Node(package='slop_base', executable='gripper', name='gripper_node', output='screen',
                          parameters=[gripper_config]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('slop_base')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'diff_drive.yaml'),
                              description='Parameterdatei des Differentialantriebs (lesen Busprozess und Node)'),
        DeclareLaunchArgument('gripper', default_value='true', description='Greifer (Achse 1 und 2) mit starten'),
        DeclareLaunchArgument('gripper_config', default_value=os.path.join(share, 'config', 'gripper.yaml'),
                              description='Parameterdatei des Greifers'),
        OpaqueFunction(function=_nodes),
    ])
