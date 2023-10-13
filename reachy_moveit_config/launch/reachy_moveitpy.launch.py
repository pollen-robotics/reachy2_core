from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction, OpaqueFunction
)
from launch_ros.actions import Node, SetUseSimTime
from moveit_utils.launch_utils import generate_moveit_config_args, generate_moveit_config




def opaque_launch_description(context, *args, **kwargs):
    """
    Launches a self contained demo

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    SetUseSimTime(True)

    moveit_config = generate_moveit_config(context)
    parameters_dict = moveit_config.to_dict()
    parameters_dict['use_sim_time'] = True
    moveit_py_node = Node(
        name="moveit_py",
        package="reachy_moveitpy_test",
        executable="moveit_test",
        output="both",
        parameters=[
                    parameters_dict,
                    ],
    )

    delayed_moveit_py_node = TimerAction(
        period=0.1,
        actions=[
            moveit_py_node,
        ],
    )

    return [
        delayed_moveit_py_node,

    ]


def generate_launch_description():
    return LaunchDescription([
        *generate_moveit_config_args(),
        OpaqueFunction(function=opaque_launch_description)
    ])
