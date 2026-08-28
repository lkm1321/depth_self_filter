"""Zero-copy composable launch: run the self-filter in the SAME process as the
depth camera driver, with intra-process comms so depth frames are passed by
pointer instead of serialized/copied between processes.

Both the camera driver and this filter are rclcpp components. Loading them into
one `component_container_mt` with `use_intra_process_comms=True` lets the raw
`sensor_msgs/Image` flow driver -> filter without an inter-process copy. This
works because the filter subscribes to the RAW image topic (not a compressed
image_transport plugin) with volatile best-effort `SensorDataQoS`, and publishes
the result as a `unique_ptr` (move-published) — the conditions rclcpp requires
for intra-process zero-copy.

Usage:
  # filter only (attach to an already-running camera container, or a bag):
  ros2 launch depth_self_filter depth_self_filter_composable.launch.py

  # filter + Luxonis driver in one process (true driver->filter zero-copy):
  ros2 launch depth_self_filter depth_self_filter_composable.launch.py \
       load_camera:=true

Note: intra-process comms eliminates the transport copy between the two
components. The one unavoidable per-frame transform remains the 16UC1 mm ->
32FC1 m conversion inside the filter (different encoding); the output message is
allocated once and move-published.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("depth_self_filter")
    default_params = os.path.join(pkg_share, "config", "params.yaml")

    params_file = LaunchConfiguration("params_file").perform(context)
    image = LaunchConfiguration("image").perform(context)
    camera_info = LaunchConfiguration("camera_info").perform(context)
    output = LaunchConfiguration("output").perform(context)
    load_camera = LaunchConfiguration("load_camera").perform(context).lower() in ("1", "true", "yes")

    ipc = {"use_intra_process_comms": True}

    composable = [
        ComposableNode(
            package="depth_self_filter",
            plugin="depth_self_filter::DepthSelfFilterNode",
            name="depth_self_filter",
            parameters=[params_file or default_params],
            remappings=[
                ("image", image),
                ("camera_info", camera_info),
                ("image_filtered", output),
            ],
            extra_arguments=[ipc],
        ),
    ]

    # Optionally bring up the Luxonis DepthAI driver in the SAME container so the
    # depth frames never leave the process. Requires the `depthai_ros_driver`
    # package to be installed; only added when load_camera:=true.
    if load_camera:
        composable.append(ComposableNode(
            package="depthai_ros_driver",
            plugin="depthai_ros_driver::Camera",
            name="oak",
            extra_arguments=[ipc],
        ))

    container = ComposableNodeContainer(
        name="depth_self_filter_container",
        namespace="",
        package="rclcpp_components",
        # _mt = multithreaded executor; the driver + filter callbacks run
        # concurrently while still sharing the process (and thus IPC).
        executable="component_container_mt",
        composable_node_descriptions=composable,
        output="screen",
    )
    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("image", default_value="/oak/stereo/image_raw"),
        DeclareLaunchArgument("camera_info", default_value="/oak/stereo/camera_info"),
        DeclareLaunchArgument("output", default_value="/oak/stereo/image_self_filtered"),
        DeclareLaunchArgument(
            "load_camera", default_value="false",
            description="Also load the Luxonis depthai_ros_driver in this container"),
        OpaqueFunction(function=_setup),
    ])
