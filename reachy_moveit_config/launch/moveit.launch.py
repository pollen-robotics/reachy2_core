from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command, FindExecutable

from launch_ros.actions import Node, SetUseSimTime
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from srdfdom.srdf import SRDF

from moveit_utils.launch_utils import generate_moveit_config_args, generate_moveit_config

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from moveit_configs_utils.launches import generate_demo_launch, generate_rsp_launch
import os

# FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT = 'full_kit', 'starter_kit_right', 'starter_kit_left'
#
#
# def get_reachy_config():
#     import yaml
#     import os
#     config_file = os.path.expanduser('~/.reachy.yaml')
#     try:
#         with open(config_file) as f:
#             config = yaml.load(f, Loader=yaml.FullLoader)
#             return config["model"] if config["model"] in [FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT] else False
#     except (FileNotFoundError, TypeError):
#         return False
#
#
# robot_model_file = get_reachy_config()


def generate_demo_launch(context, *args, **kwargs):
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

    # os.system(
    #     "topic echo --once -f /robot_description | grep 'data: ' | awk -F'data: ' '{print $2}' > ~/robot_description.urdf"
    # )

    # import subprocess
    # output = subprocess.check_output("ros2 topic echo --once -f /robot_description | grep 'data: ' | awk -F'data: ' '{print $2}'", shell=True)
    # # wrtie raw output to file
    # with open(os.path.expanduser('~/dev/.reachy.raw.urdf'), 'w') as f:
    #     f.write(output.decode('utf-8'))
    # # convert output to string
    # output = output.decode('utf-8')
    # output = output.replace('\"', '"')
    # output = output.replace('\\"', '"')
    # output = output.replace('\\n', '\n')
    # # remove last and forst char
    # output = output[1:-2]
    #
    # print("program output:", output)
    # # wrtie output to file
    # with open(os.path.expanduser('~/dev/.reachy.urdf'), 'w') as f:
    #     f.write(output)

    # get the result of my system command

    robot_model = "full_kit"
    # if robot_model_file:
    #     # TODO find a ROS way to log (without rebuilding a whole node ?
    #     print("Using robot_model described in ~/.reachy.yaml ...")
    #     robot_model = robot_model_file
    # print("Robot Model :: {}".format(robot_model))
    moveit_config = generate_moveit_config(context)

    ld_array = []

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)


    ld_array.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_config.package_path / "launch/move_group.launch.py")),
            launch_arguments={
                "use_sim_time": f"{use_sim_time}",
            }.items(),
        )
    )

    # Given the published joint states, publish tf for the robot links
    # ld_array.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(str(moveit_config.package_path / "launch/rsp.launch.py")),
    #         launch_arguments={"use_sim_time": f"{use_sim_time}"}.items(),
    #     )
    # )

    # Run Rviz and load the default config to see the state of the move_group node
    ld_array.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_config.package_path / "launch/moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            launch_arguments={"use_sim_time": f"{use_sim_time}"}.items(),
        )
    )

    # ld_array.append(
    #     TimerAction(
    #         period=5.0,
    #         actions=[
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(str(moveit_config.package_path / "launch/spawn_controllers.launch.py")),
    #                 launch_arguments={"use_sim_time": f"{use_sim_time}"}.items(),
    #             )
    #         ],
    #     )
    # )

    return ld_array


def generate_launch_description():
    return LaunchDescription([*generate_moveit_config_args(), OpaqueFunction(function=generate_demo_launch)])
