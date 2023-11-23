import os
from pathlib import Path

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import LifecycleNode, Node, SetUseSimTime
from launch_ros.descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare

FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT, HEADLESS, MINI = (
    "full_kit",
    "starter_kit_right",
    "starter_kit_left",
    "headless",
    "mini",
)
STARTER_KIT_RIGHT_NO_HEAD = "starter_kit_right_no_head"

REACHY_CONFIG_MODEL = "model"
REACHY_CONFIG_NECK = "neck_config"
REACHY_CONFIG_RIGHT_SHOULDER = "right_shoulder_config"
REACHY_CONFIG_RIGHT_ELBOW = "right_elbow_config"
REACHY_CONFIG_RIGHT_WRIST = "right_wrist_config"

REACHY_CONFIG_LEFT_SHOULDER = "left_shoulder_config"
REACHY_CONFIG_LEFT_ELBOW = "left_elbow_config"
REACHY_CONFIG_LEFT_WRIST = "left_wrist_config"


class ReachyConfig:
    def __init__(self, config_file_path="~/.reachy.yaml"):
        config_file = os.path.expanduser(config_file_path)
        with open(config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

            # Robot model (Only full kit for now, TODO)
            if config[REACHY_CONFIG_MODEL] in [
                FULL_KIT
            ]:  # , STARTER_KIT_RIGHT, STARTER_KIT_LEFT, HEADLESS, MINI, STARTER_KIT_RIGHT_NO_HEAD]:
                self.model = config[REACHY_CONFIG_MODEL]
            else:
                raise ValueError(
                    'Bad robot model "{}". Expected values are {}'.format(
                        config[REACHY_CONFIG_MODEL],
                        [
                            FULL_KIT,
                            STARTER_KIT_RIGHT,
                            STARTER_KIT_LEFT,
                            HEADLESS,
                            MINI,
                            STARTER_KIT_RIGHT_NO_HEAD,
                        ],
                    )
                )

            # TODO Multiple config

            # orbita zero
            try:
                self.neck_config = config[REACHY_CONFIG_NECK]

            except KeyError as e:
                raise KeyError("orbita3d neck config key not found :: {}".format(e))

            # right shoulder
            try:
                self.right_shoulder_config = config[REACHY_CONFIG_RIGHT_SHOULDER]

            except KeyError as e:
                raise KeyError("orbita2d right shoulder key not found :: {}".format(e))

            # right elbow
            try:
                self.right_elbow_config = config[REACHY_CONFIG_RIGHT_ELBOW]
            except KeyError as e:
                raise KeyError("orbita2d right elbow key not found :: {}".format(e))

            # right wrist
            try:
                self.right_wrist_config = config[REACHY_CONFIG_RIGHT_WRIST]
            except KeyError as e:
                raise KeyError("orbita3d right wrist key not found :: {}".format(e))

            # left shoulder
            try:
                self.left_shoulder_config = config[REACHY_CONFIG_LEFT_SHOULDER]

            except KeyError as e:
                raise KeyError("orbita2d left shoulder key not found :: {}".format(e))

            # left elbow
            try:
                self.left_elbow_config = config[REACHY_CONFIG_LEFT_ELBOW]
            except KeyError as e:
                raise KeyError("orbita2d left elbow key not found :: {}".format(e))

            # left wrist
            try:
                self.left_wrist_config = config[REACHY_CONFIG_LEFT_WRIST]
            except KeyError as e:
                raise KeyError("orbita3d left wrist key not found :: {}".format(e))

    def __str__(self):
        return (
            "robot_model".ljust(25, " ")
            + "{}\n".format(self.model)
            + "neck_config".ljust(25, " ")
            + "{}\n".format(self.neck_config)
            + "right_shoulder_config".ljust(25, " ")
            + "{}\n".format(self.right_shoulder_config)
            + "right_elbow_config".ljust(25, " ")
            + "{}\n".format(self.right_elbow_config)
            + "right_wrist_config".ljust(25, " ")
            + "{}\n".format(self.right_wrist_config)
            + "left_shoulder_config".ljust(25, " ")
            + "{}\n".format(self.left_shoulder_config)
            + "left_elbow_config".ljust(25, " ")
            + "{}\n".format(self.left_elbow_config)
            + "left_wrist_config".ljust(25, " ")
            + "{}\n".format(self.left_wrist_config)
        )


def launch_setup(context, *args, **kwargs):
    # perform(context) returns arg as a string, hence the conversion
    # var_rl is a ROS launch type object
    # var_py is a converted version, python friendly
    start_rviz_rl = LaunchConfiguration("start_rviz")
    start_rviz_py = start_rviz_rl.perform(context)
    fake_rl = LaunchConfiguration("fake")
    fake_py = fake_rl.perform(context) == "true"
    gazebo_rl = LaunchConfiguration("gazebo")
    gazebo_py = gazebo_rl.perform(context) == "true"
    start_sdk_server_rl = LaunchConfiguration("start_sdk_server")
    start_sdk_server_py = start_sdk_server_rl.perform(context) == "true"
    controllers_rl = LaunchConfiguration("controllers")
    controllers_py = controllers_rl.perform(context)

    # Robot config
    reachy_config = ReachyConfig()
    LogInfo(msg="Reachy config : \n{}".format(reachy_config)).execute(context=context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("reachy_description"), "urdf", "reachy.urdf.xacro"]
            ),
            *(
                (" ", "use_fake_hardware:=true", " ")
                if fake_py
                else (
                    " ",
                    "use_fake_hardware:=true use_gazebo:=true depth_camera:=true ",
                    " ",
                )
                if gazebo_py
                else (" ",)
            ),
            f"robot_config:={reachy_config.model}",
            " ",
            'neck_config:="{}"'.format(reachy_config.neck_config),
            " ",
            'r_shoulder_config:="{}"'.format(reachy_config.right_shoulder_config),
            " ",
            'r_elbow_config:="{}"'.format(reachy_config.right_elbow_config),
            " ",
            'r_wrist_config:="{}"'.format(reachy_config.right_wrist_config),
            " ",
            'l_shoulder_config:="{}"'.format(reachy_config.left_shoulder_config),
            " ",
            'l_elbow_config:="{}"'.format(reachy_config.left_elbow_config),
            " ",
            'l_wrist_config:="{}"'.format(reachy_config.left_wrist_config),
            " ",
        ]
    )

    # print(robot_description_content.perform(context=context))

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("reachy_bringup"),
            "config",
            # f"ros2_controllers_ultimate_combo_top_moumoute.yaml",
            # f"reachy_{reachy_config.model}_controllers.yaml",
            f"reachy_{reachy_config.model}_controllers.yaml"
            if controllers_py == "default"
            else f"ros2_controllers_ultimate_combo_top_moumoute.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("reachy_description"),
            "config",
            f"{start_rviz_py}.rviz" if start_rviz_py != "true" else "reachy.rviz",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    sdk_server_node = Node(
        package="reachy_sdk_server",
        executable="reachy_sdk_server",
        output="both",
        arguments=[reachy_config.model],
        condition=IfCondition(start_sdk_server_rl),
    )

    # camera_publisher_node = Node(
    #     package='camera_controllers',
    #     executable='camera_publisher',
    #     output='both',
    #     condition=IfCondition(
    #         PythonExpression(
    #             f"not {fake_py} and not {gazebo_py} and '{reachy_config.model}' not in ['{HEADLESS}', '{STARTER_KIT_RIGHT_NO_HEAD}']"
    #         )),
    # )

    # camera_focus_node = Node(
    #     package='camera_controllers',
    #     executable='camera_focus',
    #     output='both',
    #     condition=IfCondition(
    #         PythonExpression(
    #             f"not {fake_py} and not {gazebo_py} and '{reachy_config.model}' not in ['{HEADLESS}', '{STARTER_KIT_RIGHT_NO_HEAD}']"
    #         )),
    # )

    # camera_zoom_node = Node(
    #     package='camera_controllers',
    #     executable='camera_zoom_service',
    #     output='both',
    #     condition=IfCondition(
    #         PythonExpression(
    #             f"not {fake_py} and not {gazebo_py} and '{reachy_config.model}' not in ['{HEADLESS}', '{STARTER_KIT_RIGHT_NO_HEAD}']"
    #         )),
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(PythonExpression(f"'{start_rviz_py}' != 'false'")),
    )

    gazebo_state_broadcaster_params = PathJoinSubstitution(
        [
            FindPackageShare("reachy_gazebo"),
            "config",
            "gz_state_broadcaster_params.yaml",
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *(
                ("joint_state_broadcaster", "-p", gazebo_state_broadcaster_params)
                if gazebo_py
                else ("joint_state_broadcaster",)
            ),
            "--controller-manager",
            "/controller_manager",
        ],
    )

    neck_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["neck_forward_position_controller", "-c", "/controller_manager"],
        condition=IfCondition(
            PythonExpression(f"'{reachy_config.model}' != '{HEADLESS}'")
        ),
    )

    r_arm_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r_arm_forward_position_controller", "-c", "/controller_manager"],
        condition=IfCondition(
            PythonExpression(
                f"'{reachy_config.model}' in ['{STARTER_KIT_RIGHT}', '{FULL_KIT}', '{HEADLESS}']"
            )
        ),
    )

    l_arm_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["l_arm_forward_position_controller", "-c", "/controller_manager"],
        condition=IfCondition(
            PythonExpression(
                f"'{reachy_config.model}' in ['{STARTER_KIT_LEFT}', '{FULL_KIT}', '{HEADLESS}']"
            )
        ),
    )

    # antenna_forward_position_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['antenna_forward_position_controller', '-c', '/controller_manager'],
    #     condition=IfCondition(
    #         PythonExpression(
    #             f"'{reachy_config.model}' not in ['{HEADLESS}', '{STARTER_KIT_RIGHT_NO_HEAD}']")
    #     )
    # )

    gripper_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_forward_position_controller", "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(f"'{reachy_config.model}' != '{MINI}'")),
    )

    forward_torque_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_torque_controller", "-c", "/controller_manager"],
    )

    forward_torque_limit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_torque_limit_controller", "-c", "/controller_manager"],
    )

    forward_speed_limit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_speed_limit_controller", "-c", "/controller_manager"],
    )

    forward_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_pid_controller", "-c", "/controller_manager"],
    )

    # forward_fan_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['forward_fan_controller', '-c', '/controller_manager'],
    # )

    # fan_controller_spawner = Node(
    #     package='fans_controller',
    #     executable='fans_controller',
    # )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )

    kinematics_node = LifecycleNode(
        name="kinematics",
        namespace="",
        package="pollen_kdl_kinematics",
        executable="pollen_kdl_kinematics",
    )

    dynamic_state_router_node = Node(
        package="dynamic_state_router",
        executable="dynamic_state_router",
        arguments=[robot_controllers],
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("reachy_gazebo"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={"robot_config": f"{reachy_config.model}"}.items(),
    )
    # For Gazebo simulation, we should not launch the controller manager (Gazebo does its own stuff)

    # TODO propper refacto of this https://github.com/pollen-robotics/reachy_v2_wip/issues/20
    trajectory_controllers = []
    for traj_controller in [
        "left_arm_controller",
        "right_arm_controller",
        "head_controller",
        "left_gripper_controller",
        "right_gripper_controller",
    ]:
        trajectory_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[traj_controller, "-c", "/controller_manager"],
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                neck_forward_position_controller_spawner,
                r_arm_forward_position_controller_spawner,
                l_arm_forward_position_controller_spawner,
                # antenna_forward_position_controller_spawner,
                gripper_forward_position_controller_spawner,
                *(forward_torque_controller_spawner if not fake_py else []),
                *(forward_torque_limit_controller_spawner if not fake_py else []),
                *(forward_speed_limit_controller_spawner if not fake_py else []),
                *(forward_pid_controller_spawner if not fake_py else []),
                # *(forward_fan_controller_spawner if not fake_py else []),
                # *(fan_controller_spawner if not fake_py else []),
                *(trajectory_controllers if controllers_py == "trajectory" else []),
                kinematics_node,
            ],
        ),
    )

    delay_sdk_server_after_kinematics = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=kinematics_node,
            goal_state="inactive",
            entities=[sdk_server_node],
        )
    )

    # gripper_safe_controller_node = Node(
    #     package='gripper_safe_controller',
    #     executable='gripper_safe_controller',
    #     arguments=['--controllers-file', robot_controllers],
    #     condition=IfCondition(
    #                 PythonExpression(
    #                     f"'{reachy_config.model}' != '{MINI}'")
    #     ),
    # )

    # fake_camera_node = Node(
    #     package='reachy_fake',
    #     executable='fake_camera',
    #     condition=IfCondition(fake_rl),
    # )

    return [
        *(
            (control_node,) if not gazebo_py else (SetUseSimTime(True), gazebo_node)
        ),  # does not seem to work...
        # fake_camera_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        # gripper_safe_controller_node,
        delay_sdk_server_after_kinematics,
        # camera_publisher_node,
        # camera_focus_node,
        # camera_zoom_node,
        # sdk_camera_server_node,
        dynamic_state_router_node,
    ]


def generate_launch_description():
    # for each file, if it is a .rviz file, add it to the list of choices without the .rviz extension
    rviz_config_choices = []
    for file in os.listdir(
        os.path.dirname(os.path.realpath(__file__)) + "/../../reachy_description/config"
    ):
        if file.endswith(".rviz"):
            rviz_config_choices.append(file[:-5])

    return LaunchDescription(
        [
            # Needed by camera publisher - See: https://github.com/ros2/rosidl_python/issues/79
            SetEnvironmentVariable("PYTHONOPTIMIZE", "1"),
            DeclareLaunchArgument(
                "fake",
                default_value="false",
                description="Start on fake_reachy mode with this launch file.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "gazebo",
                default_value="false",
                description="Start a fake_hardware with gazebo as simulation tool.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "start_sdk_server",
                default_value="false",
                description="Start sdk_server along with reachy nodes with this launch file.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
                description="Start RViz2 automatically with this launch file.",
                choices=["true", "false", *rviz_config_choices],
            ),
            DeclareLaunchArgument(
                "controllers",
                default_value="default",
                description="Controller Mode",
                choices=["default", "trajectory"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )


# TODO use a OnProcessIO to check whether every node has sent its 'OK' message and log accordingly ?
