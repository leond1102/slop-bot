"""Dock-Knoten starten (neben der Kamera und, fürs Anfahren, der Navigation).

    ros2 launch slop_sensors camera.launch.py                      # Farbbild der RealSense
    ros2 launch slop_navigation navigation.launch.py               # Nav2 + Lokalisierung (für „Dock speichern“ und Anfahren)
    ros2 launch slop_dock dock.launch.py                           # dieses (Standardkarte)
    ros2 launch slop_dock dock.launch.py map:=$HOME/slop-bot_ws/maps/office-4.yaml

    ros2 service call /dock/save std_srvs/srv/Trigger              # Roboter steht vor dem Marker: Dock in der Karte vermerken
    ros2 service call /dock/start std_srvs/srv/Trigger             # andocken
    ros2 service call /dock/cancel std_srvs/srv/Trigger
    ros2 topic echo /dock/status
Der Dock liegt als maps/<karte>.dock.yaml neben der Karte. Einstellungen: config/dock.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def dock_node(context):
    map_yaml = context.launch_configurations['map']
    cfg = os.path.join(get_package_share_directory('slop_dock'), 'config', 'dock.yaml')
    return [Node(package='slop_dock', executable='dock', name='dock', output='screen',
                 parameters=[cfg, {'dock_file': os.path.splitext(map_yaml)[0] + '.dock.yaml'}])]


def generate_launch_description():
    default_map = os.path.join(os.path.expanduser('~'), 'slop-bot_ws', 'maps', 'office-3.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Karte (.yaml), der Dock liegt daneben'),
        OpaqueFunction(function=dock_node),
    ])
