"""Launch the depth self-filter node.

Remap `image` / `camera_info` to your depth camera, and make sure a
robot_state_publisher is running so TF and /robot_description are available.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("depth_self_filter")
    default_params = os.path.join(pkg_share, "config", "params.yaml")

    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    image_arg = DeclareLaunchArgument("image", default_value="/camera/depth/image_rect_raw")
    info_arg = DeclareLaunchArgument("camera_info", default_value="/camera/depth/camera_info")
    out_arg = DeclareLaunchArgument("output", default_value="/camera/depth/image_self_filtered")

    node = Node(
        package="depth_self_filter",
        executable="depth_self_filter_node",
        name="depth_self_filter",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[
            ("image", LaunchConfiguration("image")),
            ("camera_info", LaunchConfiguration("camera_info")),
            ("image_filtered", LaunchConfiguration("output")),
        ],
    )

    return LaunchDescription([params_arg, image_arg, info_arg, out_arg, node])
