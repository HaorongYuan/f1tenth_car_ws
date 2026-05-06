import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("reactive_racing"),
        "config",
        "reactive_racing.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("drive_topic", default_value="/drive"),
            DeclareLaunchArgument(
                "marker_topic", default_value="/reactive_racing/arrow_marker"
            ),
            DeclareLaunchArgument(
                "debug_scan_topic", default_value="/reactive_racing/front_scan"
            ),
            Node(
                package="reactive_racing",
                executable="reactive_racing_node",
                name="reactive_racing_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "drive_topic": LaunchConfiguration("drive_topic"),
                        "marker_topic": LaunchConfiguration("marker_topic"),
                        "debug_scan_topic": LaunchConfiguration("debug_scan_topic"),
                    },
                ],
            ),
        ]
    )
