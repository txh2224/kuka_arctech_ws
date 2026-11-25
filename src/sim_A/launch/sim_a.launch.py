from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    job_file_arg = DeclareLaunchArgument(
        "job_file",
        default_value="job_8steps_demo.json",
        description="Job JSON filename under sim_A/config/ or a file path.",
    )
    action_name_arg = DeclareLaunchArgument(
        "action_name",
        default_value="/adapter/kuka_arctech/execute_job",
        description="Target ExecuteBackendJob action name.",
    )
    wait_server_sec_arg = DeclareLaunchArgument(
        "wait_server_sec",
        default_value="5.0",
        description="Seconds to wait for the action server.",
    )
    cancel_after_sec_arg = DeclareLaunchArgument(
        "cancel_after_sec",
        default_value="0.0",
        description="If > 0, send cancel request after this many seconds.",
    )
    auto_shutdown_arg = DeclareLaunchArgument(
        "auto_shutdown",
        default_value="true",
        description="Shutdown node automatically after receiving action result.",
    )

    sim_a_node = Node(
        package="sim_A",
        executable="sim_a_node",
        name="sim_a_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "job_file": LaunchConfiguration("job_file"),
                "action_name": LaunchConfiguration("action_name"),
                "wait_server_sec": LaunchConfiguration("wait_server_sec"),
                "cancel_after_sec": LaunchConfiguration("cancel_after_sec"),
                "auto_shutdown": LaunchConfiguration("auto_shutdown"),
            }
        ],
    )

    return LaunchDescription(
        [
            job_file_arg,
            action_name_arg,
            wait_server_sec_arg,
            cancel_after_sec_arg,
            auto_shutdown_arg,
            sim_a_node,
        ]
    )
