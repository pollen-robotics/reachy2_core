from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_utils.launch_utils import use_sim_time_arg, generate_moveit_config

from launch_ros.actions import Node


def opaque_launch_description(context, *args, **kwargs):
    moveit_config = generate_moveit_config(context)

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=str(moveit_config.package_path / "config/moveit.rviz"),
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        {'use_sim_time': LaunchConfiguration("use_sim_time")},

    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return [
        use_sim_time_arg,
        rviz_config_arg,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=opaque_launch_description)
    ])
