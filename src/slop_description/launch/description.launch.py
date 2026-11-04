"""Robotermodell veröffentlichen: robot_state_publisher mit dem URDF (/robot_description, TF base_footprint → Sensoren).

    ros2 launch slop_description description.launch.py
    ros2 launch slop_description description.launch.py rviz:=true             # mit RViz
    ros2 launch slop_description description.launch.py rviz:=true gui:=true   # Räder per Schieberegler (ohne Antrieb)

Die Radwinkel kommen von diff_drive_node (/joint_states). gui:=true nur ohne laufenden Antrieb benutzen,
sonst gibt es zwei Quellen für /joint_states.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('slop_description')
    urdf = os.path.join(share, 'urdf', 'slop_bot.urdf.xacro')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='RViz mit dem Modell öffnen'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='joint_state_publisher_gui (nur ohne Antrieb)'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type=str)}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'description.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
