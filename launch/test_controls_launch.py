from ast import arguments
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = "ezrassor_autonomous_control"

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='test_control',
            arguments=[
                LaunchConfiguration('spawn_x'),
                LaunchConfiguration('spawn_y'),
                LaunchConfiguration('target_x'),
                LaunchConfiguration('target_y'),
                LaunchConfiguration('model')
            ],
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='auto_controller',
            output='screen',
            arguments = [LaunchConfiguration('model')]),
            
    ])
