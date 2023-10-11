from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch
from launch_ros.actions import Node
from moveit_utils.launch_utils import use_sim_time_arg, generate_moveit_config


def opaque_launch_description(context, *args, **kwargs):

    moveit_config = generate_moveit_config(context)

    publish_frequency_arg = DeclareLaunchArgument("publish_frequency", default_value="15.0")

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
            {'use_sim_time': LaunchConfiguration("use_sim_time")},
        ],
    )
    print("\n\n")
    print(LaunchConfiguration("use_sim_time").perform(context))
    print("\n\n")

    return [
        use_sim_time_arg,
        publish_frequency_arg,
        rsp_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=opaque_launch_description)
    ])