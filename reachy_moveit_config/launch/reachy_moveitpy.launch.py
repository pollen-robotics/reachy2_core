from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command, FindExecutable
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

from launch.actions import LogInfo
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from srdfdom.srdf import SRDF


from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT = 'full_kit', 'starter_kit_right', 'starter_kit_left'


def get_reachy_config():
    import yaml
    import os
    config_file = os.path.expanduser('~/.reachy.yaml')
    try:
        with open(config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            return config["model"] if config["model"] in [FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT] else False
    except (FileNotFoundError, TypeError):
        return False


robot_model_file = get_reachy_config()


def generate_demo_launch(moveit_config):
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
    robot_model = "full_kit"
    if robot_model_file:
        # TODO find a ROS way to log (without rebuilding a whole node ?
        print("Using robot_model described in ~/.reachy.yaml ...")
        robot_model = robot_model_file
    print("Robot Model :: {}".format(robot_model))



    # ld = LaunchDescription([moveit_py_node])
    ld = LaunchDescription()

    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict['use_sim_time'] = True

    moveit_py_node = Node(
        name="moveit_py",
        package="reachy_moveitpy_test",
        executable="moveit_test",
        output="both",
        parameters=[moveit_config_dict
                    'use_sim_time': get_parama(usetime),'],
    )

    ld.add_action(

        TimerAction(
            period=10.0,
            actions=[
                moveit_py_node,
            ],
        )
    )

    return ld


def generate_launch_description():
    # TODO transform this in opaque, example reachy.launch.py
    moveit_config = MoveItConfigsBuilder("reachy_v2", package_name="reachy_moveit_config")
    # moveit_config = moveit_config.sensors_3d(None)  # be sure to disable the 3D sensor
    moveit_config = moveit_config.robot_description(mappings={'use_fake_hardware': 'true', 'use_gazebo': 'true', 'use_moveit_gazebo': 'true'})  # pass parameters to xacro (this should work be it does not...)
    moveit_config = moveit_config.moveit_cpp(file_path=get_package_share_directory("reachy_moveit_config")+ "/config/moveit_cpp.yaml")


    return generate_demo_launch(moveit_config.to_moveit_configs())
