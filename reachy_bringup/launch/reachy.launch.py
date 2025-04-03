import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    LocalSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import LifecycleNode, Node, SetUseSimTime
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from reachy2_sdk_api.reachy_pb2 import ReachyCoreMode

from reachy_config import (
    BETA,
    DVT,
    FULL_KIT,
    HEADLESS,
    MINI,
    STARTER_KIT_LEFT,
    STARTER_KIT_RIGHT,
    STARTER_KIT_RIGHT_NO_HEAD,
    ReachyConfig,
    log_config,
)
from reachy_utils.launch import (
    ROSBAG_TOPICS,
    build_watchers_from_node_list,
    clear_bags_and_logs,
    get_current_run_log_dir,
    get_fake,
    get_node_list,
    get_rviz_conf_choices,
    title_print,
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
    foxglove_rl = LaunchConfiguration("foxglove")
    foxglove_py = foxglove_rl.perform(context) == "true"
    orbbec_rl = LaunchConfiguration("orbbec")
    orbbec_py = orbbec_rl.perform(context) == "true"
    verbose_logger_log_level_rl = LaunchConfiguration("log")
    nodes = []

    clear_bags_and_logs(nb_runs_to_keep=25)

    ####################
    ### Robot config ###
    ####################

    title_print("Configuration").execute(context=context)
    reachy_config = ReachyConfig()
    LogInfo(msg="Reachy config : \n{}".format(reachy_config)).execute(context=context)

    # reachy_urdf_config = (
    #     f" use_fake_hardware:=true" if fake_py or gazebo_py else " ",
    #     f" use_gazebo:=true" if gazebo_py else " ",
    #     f" depth_camera:=true" if gazebo_py or orbbec_py else " ",
    #     f" robot_config:={reachy_config.model}",
    #     f' neck_config:="{reachy_config.neck_config if not fake_py and not gazebo_py else get_fake("orbita3d_description", "fake_neck.yaml", context)}"',
    #     f' right_shoulder_config:="{reachy_config.right_shoulder_config if not fake_py and not gazebo_py else get_fake("orbita2d_description", "fake_r_shoulder.yaml", context)}"',
    #     f' right_elbow_config:="{reachy_config.right_elbow_config if not fake_py and not gazebo_py else get_fake("orbita2d_description", "fake_r_elbow.yaml", context)}"',
    #     f' right_wrist_config:="{reachy_config.right_wrist_config if not fake_py and not gazebo_py else get_fake("orbita3d_description", "fake.yaml", context)}"',
    #     f' left_shoulder_config:="{reachy_config.left_shoulder_config if not fake_py and not gazebo_py else get_fake("orbita2d_description", "fake_l_shoulder.yaml", context)}"',
    #     f' left_elbow_config:="{reachy_config.left_elbow_config if not fake_py and not gazebo_py else get_fake("orbita2d_description", "fake_l_elbow.yaml", context)}"',
    #     f' left_wrist_config:="{reachy_config.left_wrist_config if not fake_py and not gazebo_py else get_fake("orbita3d_description", "fake.yaml", context)}"',
    #     f' antenna_config:="{reachy_config.antenna_config if not fake_py and not gazebo_py else get_fake("dynamixel_description", "two_fake.yaml", context)}"',
    #     f' grippers_config:="{reachy_config.grippers_config if not fake_py and not gazebo_py else get_fake("dynamixel_description", "two_fake.yaml", context)}"',
    #     f' robot_model:="{BETA if reachy_config.beta else DVT }"',  # for now PVT urdf is assumed to be the same as dvt
    # )

    reachy_urdf_config = (
        f" use_fake_hardware:=true" if fake_py or gazebo_py else " ",
        f" use_gazebo:=true" if gazebo_py else " ",
        f" depth_camera:=true" if gazebo_py or orbbec_py else " ",
        f" robot_config:={reachy_config.model}",
        f' neck_config:="{reachy_config.part_conf("neck_config", fake= fake_py or gazebo_py)}"',
        f' right_shoulder_config:="{reachy_config.part_conf("right_shoulder_config", fake= fake_py or gazebo_py)}"',
        f' right_elbow_config:="{reachy_config.part_conf("right_elbow_config", fake= fake_py or gazebo_py)}"',
        f' right_wrist_config:="{reachy_config.part_conf("right_wrist_config", fake= fake_py or gazebo_py)}"',
        f' left_shoulder_config:="{reachy_config.part_conf("left_shoulder_config", fake= fake_py or gazebo_py)}"',
        f' left_elbow_config:="{reachy_config.part_conf("left_elbow_config", fake= fake_py or gazebo_py)}"',
        f' left_wrist_config:="{reachy_config.part_conf("left_wrist_config", fake= fake_py or gazebo_py)}"',
        f' antenna_config:="{reachy_config.part_conf("antenna_config", fake= fake_py or gazebo_py)}"',
        f' grippers_config:="{reachy_config.part_conf("grippers_config", fake= fake_py or gazebo_py)}"',
        # f' right_shoulder_config:="{reachy_config.right_shoulder_config if not fake_py and not gazebo_py else get_fake("orbita2d_description", "fake_r_shoulder.yaml", context)}"',
        # f' right_elbow_config:="{reachy_config.right_elbow_config if not fake_py and not gazebo_py else get_fake("orbita2d_description", "fake_r_elbow.yaml", context)}"',
        # f' right_wrist_config:="{reachy_config.right_wrist_config if not fake_py and not gazebo_py else get_fake("orbita3d_description", "fake.yaml", context)}"',
        # f' left_shoulder_config:="{reachy_config.left_shoulder_config if not fake_py and not gazebo_py else get_fake("orbita2d_description", "fake_l_shoulder.yaml", context)}"',
        # f' left_elbow_config:="{reachy_config.left_elbow_config if not fake_py and not gazebo_py else get_fake("orbita2d_description", "fake_l_elbow.yaml", context)}"',
        # f' left_wrist_config:="{reachy_config.left_wrist_config if not fake_py and not gazebo_py else get_fake("orbita3d_description", "fake.yaml", context)}"',
        # f' antenna_config:="{reachy_config.antenna_config if not fake_py and not gazebo_py else get_fake("dynamixel_description", "two_fake.yaml", context)}"',
        # f' grippers_config:="{reachy_config.grippers_config if not fake_py and not gazebo_py else get_fake("dynamixel_description", "two_fake.yaml", context)}"',
        f' robot_model:="{BETA if reachy_config.beta else DVT }"',  # for now PVT urdf is assumed to be the same as dvt
    )
    LogInfo(msg=f"Reachy URDF config : \n{log_config(reachy_urdf_config)}").execute(context=context)

    if gazebo_py:
        LogInfo(msg="Starting Gazebo simulation, setting UseSimTime").execute(context=context)
        SetUseSimTime(True)

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution([FindPackageShare("reachy_description"), "urdf", "reachy.urdf.xacro"]),
                    *reachy_urdf_config,
                ]
            ),
            value_type=str,
        ),
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

    gazebo_state_broadcaster_params = PathJoinSubstitution(
        [
            FindPackageShare("reachy_gazebo"),
            "config",
            "gz_state_broadcaster_params.yaml",
        ]
    )

    # start_mobile_base = "true" if None not in reachy_config.mobile_base_config.values() else "false"
    # start_mobile_base_py = start_mobile_base == "true"

    # LogInfo(msg=f"Launching Mobile Base: {start_mobile_base_py}").execute(context=context)

    #############
    ### Nodes ###
    #############
    title_print("Launching nodes...").execute(context=context)

    # start ethercat server
    ethercat_master_server = ExecuteProcess(
        name="ethercat_master_server",
        cmd=[
            "/bin/bash",
            "-c",
            f'$HOME/dev/poulpe_ethercat_controller/start_ethercat_server.sh {reachy_config.config["robot_ethercat_config"]["path"]}',
        ],
        output="both",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(f"not {fake_py}")),
        # Ensure the process is killed when the launch file is stopped
        sigterm_timeout="2",  # Grace period before sending SIGKILL (optional)
        sigkill_timeout="2",  # Time to wait after SIGTERM before sending SIGKILL (optional)
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", verbose_logger_log_level_rl],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=["--ros-args", "--log-level", verbose_logger_log_level_rl],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        exec_name="joint_state_broadcaster",
        name="joint_state_broadcaster",
        # namespace="/",
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

    ###################
    ### Controllers ###
    ###################

    position_controllers = []
    for controller, condition in [
        [
            "neck_forward_position_controller",
            f"'{reachy_config.model}' != '{HEADLESS}'",
        ],
        [
            "antenna_forward_position_controller",
            f"'{reachy_config.model}' != '{HEADLESS}'",
        ],
        [
            "tripod_forward_position_controller",
            f"'{reachy_config.model}' != '{MINI}'",
        ],
        # [
        #     "r_shoulder_forward_position_controller",
        #     f"'{reachy_config.model}' in ['{STARTER_KIT_RIGHT}', '{FULL_KIT}', '{HEADLESS}']",
        # ],
        # [
        #     "r_elbow_forward_position_controller",
        #     f"'{reachy_config.model}' in ['{STARTER_KIT_RIGHT}', '{FULL_KIT}', '{HEADLESS}']",
        # ],
        # [
        #     "r_wrist_forward_position_controller",
        #     f"'{reachy_config.model}' in ['{STARTER_KIT_RIGHT}', '{FULL_KIT}', '{HEADLESS}']",
        # ],
        # [
        #     "l_shoulder_forward_position_controller",
        #     f"'{reachy_config.model}' in ['{STARTER_KIT_LEFT}', '{FULL_KIT}', '{HEADLESS}']",
        # ],
        # [
        #     "l_elbow_forward_position_controller",
        #     f"'{reachy_config.model}' in ['{STARTER_KIT_LEFT}', '{FULL_KIT}', '{HEADLESS}']",
        # ],
        # [
        #     "l_wrist_forward_position_controller",
        #     f"'{reachy_config.model}' in ['{STARTER_KIT_LEFT}', '{FULL_KIT}', '{HEADLESS}']",
        # ],
        [
            "r_arm_forward_position_controller",
            f"'{reachy_config.model}' in ['{STARTER_KIT_RIGHT}', '{FULL_KIT}', '{HEADLESS}']",
        ],
        [
            "l_arm_forward_position_controller",
            f"'{reachy_config.model}' in ['{STARTER_KIT_LEFT}', '{FULL_KIT}', '{HEADLESS}']",
        ],
        ["gripper_forward_position_controller", f"'{reachy_config.model}' != '{MINI}'"],
    ]:
        position_controllers.append(
            Node(
                package="controller_manager",
                exec_name=controller,
                name=controller,
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                condition=IfCondition(PythonExpression(condition)),
            )
        )

    generic_controllers = []
    for controller, condition in [
        ["forward_torque_controller", f"not {gazebo_py}"],
        ["forward_torque_limit_controller", f"not {gazebo_py}"],
        ["forward_speed_limit_controller", f"not {gazebo_py}"],
        ["forward_pid_controller", f"not {fake_py} and not {gazebo_py}"],
        ["gripper_current_controller", f"not {fake_py} and not {gazebo_py}"],
        ["gripper_mode_controller", f"not {fake_py} and not {gazebo_py}"],
        ["antenna_current_controller", f"not {fake_py} and not {gazebo_py}"],
        ["antenna_mode_controller", f"not {gazebo_py}"],
        # ["tripod_forward_position_controller", f"not {gazebo_py}"],
    ]:
        generic_controllers.append(
            Node(
                package="controller_manager",
                exec_name=controller,
                name=controller,
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                condition=IfCondition(PythonExpression(condition)),
            )
        )

    kinematics_node = LifecycleNode(
        name="kinematics",
        namespace="",
        package="pollen_kdl_kinematics",
        executable="pollen_kdl_kinematics",
        output="both",
        emulate_tty=True,
        additional_env={"RCUTILS_CONSOLE_OUTPUT_FILE": "/home/reachy/.ros/log/kinematics.log"},
    )

    dynamic_state_router_node = Node(
        package="dynamic_state_router",
        executable="dynamic_state_router",
        arguments=[robot_controllers],
    )

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
                exec_name=traj_controller,
                arguments=[traj_controller, "-c", "/controller_manager"],
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                *generic_controllers,
                *(position_controllers if controllers_py != "trajectory" else []),
                *(trajectory_controllers if controllers_py == "trajectory" else []),
                kinematics_node,
            ],
        ),
    )

    ###########
    ### SDK ###
    ###########
    sdk_server_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="reachy_sdk_server",
                executable="reachy_grpc_joint_sdk_server",
                output="both",
                emulate_tty=True,
                arguments=[
                    reachy_config.config["reachy"]["path"],
                    str(ReachyCoreMode.GAZEBO if gazebo_py else ReachyCoreMode.FAKE if fake_py else ReachyCoreMode.REAL),
                ],
                condition=IfCondition(start_sdk_server_rl),
            )
        ],
    )

    sdk_server_video_node = Node(
        package="reachy_sdk_server",
        executable="reachy_grpc_video_sdk_server",
        output="both",
        condition=IfCondition(start_sdk_server_rl),
        arguments=["--gazebo"] if gazebo_py else [],
    )

    orbbec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("orbbec_camera"), "/launch", "/gemini_330_series.launch.py"]),
        launch_arguments={"depth_width": "1280", "enable_colored_point_cloud": "true"}.items(),
        condition=IfCondition(PythonExpression(f"{orbbec_py} and not {gazebo_py}")),
    )

    goto_server_node = Node(
        package="pollen_goto",
        executable="goto_server",
        output="both",
        condition=IfCondition(start_sdk_server_rl),
    )

    sdk_server_audio_server = ExecuteProcess(
        name="reachy_sdk_audio_server",
        cmd=["/bin/bash", "-c", "reachy2_sdk_audio_server_rs"],
        output="both",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(f"{start_sdk_server_py}")),
        # Ensure the process is killed when the launch file is stopped
        sigterm_timeout="2",  # Grace period before sending SIGKILL (optional)
        sigkill_timeout="2",  # Time to wait after SIGTERM before sending SIGKILL (optional)
    )

    ####################
    ### Tools et al. ###
    ####################
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="log",
                    arguments=["-d", rviz_config_file],
                    condition=IfCondition(PythonExpression(f"'{start_rviz_py}' != 'false'")),
                )
            ],
        ),
    )

    # start foxglove bridge like this ros2 launch foxglove_bridge foxglove_bridge_launch.xml
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="both",
        arguments=["foxglove_bridge_launch.xml"],
        condition=IfCondition(foxglove_rl),
    )

    if reachy_config.mobile_base["enable"]:
        mobile_base_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("zuuu_hal"), "/hal.launch.py"]),
            launch_arguments={"use_sim_time": f"{gazebo_py}", "fake": f"{fake_py}", "gazebo": f"{gazebo_py}"}.items(),
        )
        nodes.append(mobile_base_node)

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("reachy_gazebo"), "/launch", "/gazebo.launch.py"]),
        launch_arguments={
            "robot_config": f"{reachy_config.model}",
            # "robot_model": {BETA if reachy_config.beta else DVT},
        }.items(),
    )
    # For Gazebo simulation, we should not launch the controller manager (Gazebo does its own stuff)

    rosbag = ExecuteProcess(
        name="rosbag",
        cmd=[
            "ros2",
            "bag",
            "record",
            "--snapshot-mode",
            "--max-cache-size",
            "400000000",
            "-o",
            f"{get_current_run_log_dir()}/reachy.bag",
            *ROSBAG_TOPICS,
        ],
        output="screen",
    )

    nodes.extend(
        [
            # *((control_node,) if not gazebo_py else (gazebo_node,)),  # SetUseSimTime does not seem to work...
            # fake_camera_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            sdk_server_node,
            sdk_server_audio_server,
            sdk_server_video_node,
            orbbec_node,
            goto_server_node,
            dynamic_state_router_node,
            foxglove_bridge_node,
            rosbag,
        ]
    )

    # Mujoco stuff
    # Replace with your robot's URDF file

    # Replace with your controller configuration file

    # Define the MuJoCo model path
    reachy_mujoco_model_path = "/home/reachy/dev/reachy2_mujoco/reachy2_mujoco/description/mjcf/reachy2.xml"

    # Define MuJoCo node
    # node_mujoco_ros2_control = Node(
    #     package="mujoco_ros2_control",
    #     executable="mujoco_ros2_control",
    #     output="both",
    #     parameters=[robot_description, robot_controllers, {"mujoco_model_path": mujoco_model_path}],
    # )

    start_control_after_ehtercat = TimerAction(
        period=3.0 if not gazebo_py else 0.5,
        actions=[
            control_node if not gazebo_py else gazebo_node,
        ],
        cancel_on_shutdown=True,
    )

    start_everything_after_control = TimerAction(
        period=4.0 if not gazebo_py else 1.5,
        actions=[
            *nodes,
        ],
        cancel_on_shutdown=True,
    )

    # MUJOCO DEMO STUFF
    import xacro
    from ament_index_python.packages import get_package_share_directory

    # cart conf
    mujoco_ros2_control_demos_path = os.path.join(get_package_share_directory("mujoco_ros2_control_demos"))
    xacro_file = os.path.join(mujoco_ros2_control_demos_path, "urdf", "test_cart_position.xacro.urdf")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml()}
    print(robot_description)
    controller_config_file = os.path.join(mujoco_ros2_control_demos_path, "config", "cartpole_controller_position.yaml")
    mujoco_model_path = os.path.join(mujoco_ros2_control_demos_path, "mujoco_models", "test_cart.xml")

    # reachy conf
    reachy2_urdl_path = "/home/reachy/dev/reachy2_mujoco/reachy2_mujoco/description/modified_urdf/reachy2.urdf"
    with open(reachy2_urdl_path, "r") as f:
        mujoco_urdf_model = f.read()
    robot_description = {"robot_description": mujoco_urdf_model}
    # print(robot_description)
    controller_config_file = robot_controllers
    mujoco_model_path = reachy_mujoco_model_path

    node_mujoco_ros2_control = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            controller_config_file,
            # {"mujoco_model_path": os.path.join(mujoco_ros2_control_demos_path, "mujoco_models", "test_cart.xml")},
            {"mujoco_model_path": mujoco_model_path},
        ],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen", parameters=[robot_description]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"], output="screen"
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_trajectory_controller"], output="screen"
    )

    return [
        # RegisterEventHandler(
        #     event_handler=OnProcessStart(
        #         target_action=node_mujoco_ros2_control,
        #         on_start=[load_joint_state_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        node_mujoco_ros2_control,
        node_robot_state_publisher,
    ]

    return [
        *build_watchers_from_node_list(get_node_list(nodes, context) + [ethercat_master_server] + [control_node]),
        ethercat_master_server,
        start_control_after_ethercat,
        start_everything_after_control,
        # SetEnvironmentVariable(
        #     name="PYTHONPATH",
        #     value=f"/home/reachy/.local/lib/python3.10/site-packages/:{os.environ['PYTHONPATH']}",
        # ),
        # cf notion page about python site packages
    ]


def generate_launch_description():
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
                choices=["true", "false", *get_rviz_conf_choices()],
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value="false",
                description="Start FoxGlove bridge with this launch file.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "orbbec",
                default_value="true",
                description="Start Orbbec depth camera with this launch file.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "controllers",
                default_value="default",
                description="Controller Mode",
                choices=["default", "trajectory"],
            ),
            DeclareLaunchArgument(
                "log",
                default_value="WARN",
                description="Log level for needlessly verbose nodes",
                choices=["DEBUG", "INFO", "WARN", "ERROR"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )


# TODO use a OnProcessIO to check whether every node has sent its 'OK' message and log accordingly ?
# TODO NodeLifeCycle and proper node state management ?
