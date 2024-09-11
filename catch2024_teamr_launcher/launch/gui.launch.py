from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # get package source directory
    package_dir = '/home/oyaki/ros2_ws/src/catch2024_r/catch2024_teamr_gui'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server',
                 'rosbridge_websocket_launch.xml'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['npm', 'run', 'dev'],
            output='screen',
            env={'PATH': os.environ['PATH']},
            cwd=os.path.join(package_dir, 'catch2024_teamr_gui_app')
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/index'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/emergency'],
            output='screen'
        ),
    ])
