from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    templates_dir_arg = DeclareLaunchArgument(
        "templates_dir",
        default_value="",
        description="Optional override directory for KRL golden templates (empty uses installed share/templates).",
    )
    debug_dir_arg = DeclareLaunchArgument(
        "debug_dir",
        default_value="debug",
        description="Directory for Action, Stop service and status debug records. Empty disables recording.",
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
                "templates_dir": LaunchConfiguration("templates_dir"),
                "debug_dir": ParameterValue(LaunchConfiguration("debug_dir"), value_type=str),
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
            templates_dir_arg,
            debug_dir_arg,
            adapter_node,
        ]
    )
