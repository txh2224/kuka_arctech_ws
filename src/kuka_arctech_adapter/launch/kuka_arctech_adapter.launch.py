from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="kuka_arctech_adapter",
        description="Name of the KUKA ArcTech adapter node.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 logging level (debug, info, warn, error, fatal).",
    )
    transport_mode_arg = DeclareLaunchArgument(
        "transport_mode",
        default_value="mock",
        description="KRC transport mode ('mock' or 'manual'; 'verified' planned).",
    )
    manual_export_dir_arg = DeclareLaunchArgument(
        "manual_export_dir",
        default_value="output",
        description="Output directory for generated .src/.dat/manifest when transport_mode:=manual.",
    )

    adapter_node = Node(
        package="kuka_arctech_adapter",
        executable="adapter_node",
        name=LaunchConfiguration("node_name"),
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "transport_mode": LaunchConfiguration("transport_mode"),
                "manual_export_dir": LaunchConfiguration("manual_export_dir"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            node_name_arg,
            log_level_arg,
            transport_mode_arg,
            manual_export_dir_arg,
            adapter_node,
        ]
    )
