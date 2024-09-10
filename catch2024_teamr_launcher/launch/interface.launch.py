import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    rogidrive_node = Node(
        package="rogidrive",
        executable="rogidrive",
        name="rogidrive",
        output="screen",
        # parameters=[{
        #     'config_path':
        #     "/home/rogi/ros2_ws/src/catch2024_r/rogidrive/rogidrive/config/catch_r.jsonc"

        # }],
        parameters=[{
            'config_path': os.path.join(
                get_package_share_directory(
                    "catch2024_teamr_launcher"),
                "config", "catch_r.jsonc")
        }],
        # ros_arguments=["--log-level", "warn"]
    )

    rogilink3_node = Node(package="rogilink3",
                          executable="rogilink3",
                          name="rogilink3",
                          output="screen",
                          parameters=[
                              os.path.join(
                                  get_package_share_directory(
                                      "catch2024_teamr_launcher"),
                                  "config", "rogilink.yaml")
                          ])

    ld.add_action(rogidrive_node)
    ld.add_action(rogilink3_node)
    return ld
