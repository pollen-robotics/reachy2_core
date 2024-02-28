import os

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
from reachy_utils.config import (
    FULL_KIT,
    HEADLESS,
    MINI,
    STARTER_KIT_LEFT,
    STARTER_KIT_RIGHT,
    STARTER_KIT_RIGHT_NO_HEAD,
    ReachyConfig,
    log_config,
)
from reachy_utils.launch import get_fake


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

    ### Robot config
    reachy_config = ReachyConfig()
    LogInfo(msg="Reachy config : \n{}".format(reachy_config)).execute(context=context)

    xacro_config = (
        f" use_fake_hardware:=true" if fake_py or gazebo_py else " ",
        f" use_gazebo:=true" if gazebo_py else " ",
        f" depth_camera:=true" if gazebo_py else " ",
        f" robot_config:={reachy_config.model}",
        f' neck_config:="{reachy_config.neck_config if not fake_py else get_fake("orbita3d_description", "fake.yaml", context)}"',
        f' right_arm_config:="{reachy_config.right_arm_config if not fake_py else get_fake("arm_description", "fake_arm.yaml", context)}"',
        f' left_arm_config:="{reachy_config.left_arm_config if not fake_py else get_fake("arm_description", "fake_arm.yaml", context)}"',
    )
    # LogInfo(msg=f"Xacro config : \n{log_config(xacro_config)}").execute(context=context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("reachy_description"), "urdf", "reachy.urdf.xacro"]
            ),
            *xacro_config,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("reachy_bringup"),
            "config",
            (
                f"reachy_{reachy_config.model}_controllers.yaml"
                if controllers_py == "default"
                else f"ros2_controllers_ultimate_combo_top_moumoute.yaml"
            ),
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("reachy_description"),
            "config",
            f"{start_rviz_py}.rviz" if start_rviz_py != "true" else "reachy.rviz",
        ]
    )

    ### Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    sdk_server_node = Node(
        package="reachy_sdk_server",
        executable="reachy_grpc_joint_sdk_server",
        output="both",
        arguments=[reachy_config.config_file],
        condition=IfCondition(start_sdk_server_rl),
    )

    video_sdk_server_node = Node(
        package="reachy_sdk_server",
        executable="reachy_grpc_video_sdk_server",
        output="both",
        condition=IfCondition(start_sdk_server_rl),
    )

    # sdk_server_audio_node = Node(
    #     package="reachy_sdk_server",
    #     executable="reachy_grpc_audio_sdk_server",
    #     output="both",
    #     condition=IfCondition(start_sdk_server_rl),
    # )

    # audio_node = Node(
    #     package="sound_play",
    #     executable="soundplay_node.py",
    #     output="both",
    #     condition=IfCondition(start_sdk_server_rl),
    # )

    goto_server_node = Node(
        package="pollen_goto",
        executable="goto_server",
        output="both",
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

    position_controllers = []
    for controller, condition in [
        [
            "neck_forward_position_controller",
            f"'{reachy_config.model}' != '{HEADLESS}'",
        ],
        [
            "r_arm_forward_position_controller",
            f"'{reachy_config.model}' in ['{STARTER_KIT_RIGHT}', '{FULL_KIT}', '{HEADLESS}']",
        ],
        [
            "l_arm_forward_position_controller",
            f"'{reachy_config.model}' in ['{STARTER_KIT_LEFT}', '{FULL_KIT}', '{HEADLESS}']",
        ],
        ["gripper_forward_position_controller", f"'{reachy_config.model}' != '{MINI}'"],
        ["forward_torque_controller", f"not {fake_py} and not {gazebo_py}"],
        ["forward_torque_limit_controller", f"not {fake_py} and not {gazebo_py}"],
        ["forward_speed_limit_controller", f"not {fake_py} and not {gazebo_py}"],
        ["forward_pid_controller", f"not {fake_py} and not {gazebo_py}"],
    ]:
        position_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                condition=IfCondition(PythonExpression(condition)),
            )
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

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    *position_controllers,
                    *(trajectory_controllers if controllers_py == "trajectory" else []),
                    kinematics_node,
                ],
            ),
        )
    )

    print(
        "Launching Mobile Base: {}".format(
            "true" if None not in reachy_config.mobile_base_config.values() else "false"
        )
    )
    mobile_base_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("zuuu_hal"), "/hal.launch.py"]),
        condition=IfCondition(
            "true" if None not in reachy_config.mobile_base_config.values() else "false"
        ),
    )

    gripper_safe_controller_node = Node(
        package="gripper_safe_controller",
        executable="gripper_safe_controller",
        arguments=["--controllers-file", robot_controllers],
    )

    return [
        *(
            (control_node,) if not gazebo_py else (SetUseSimTime(True), gazebo_node)
        ),  # does not seem to work...
        # fake_camera_node,
        mobile_base_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        gripper_safe_controller_node,
        sdk_server_node,
        video_sdk_server_node,
        # sdk_server_audio_node,
        # audio_node,
        goto_server_node,
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
