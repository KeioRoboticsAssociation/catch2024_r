import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir=get_package_share_directory('catch2024_teamr')
    return LaunchDescription([
        Node(
            package='catch2024_teamr',
            executable='mainarm_lowlayer',
            name='mainarm_lowlayer',
            output='screen'
        )
    ])