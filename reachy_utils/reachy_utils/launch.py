import os
import shutil
import signal
import threading
from datetime import datetime, timedelta

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LocalSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rclpy.logging import get_logging_directory

report_lock = threading.Lock()
failed_nodes = []
success_nodes = []
CORE_UP_SOUND = "MA_BANT_GAME_PACK_2.wav"
CORE_DOWN_SOUND = "MA_BANT_Bubbles_Pops_2.wav"
SHUTDOWN_GRACE_PERIOD = 2
ROSBAG_TOPICS = [
    "/clicked_point",
    "/dynamic_joint_commands",
    "/dynamic_joint_states",
    "/events/write_split",
    "/forward_pid_controller/commands",
    "/forward_speed_limit_controller/commands",
    "/forward_speed_limit_controller/transition_event",
    "/forward_torque_controller/commands",
    "/forward_torque_controller/transition_event",
    "/forward_torque_limit_controller/commands",
    "/forward_torque_limit_controller/transition_event",
    "/goal_pose",
    "/gripper_current_controller/commands",
    "/gripper_forward_position_controller/commands",
    "/gripper_forward_position_controller/transition_event",
    "/grippers/commands",
    "/head/averaged_target_pose",
    "/head/cart_target_pose",
    "/head/ik_target_pose",
    "/head/target_pose",
    "/initialpose",
    "/joint_commands",
    "/joint_state_broadcaster/transition_event",
    "/joint_states",
    "/kinematics/transition_event",
    "/l_arm/averaged_target_pose",
    "/l_arm/cart_target_pose",
    "/l_arm/ik_target_pose",
    "/l_arm/target_pose",
    "/l_arm_forward_position_controller/commands",
    "/l_arm_forward_position_controller/transition_event",
    "/l_arm_reachability_states",
    "/markers_grasp_triplet",
    "/mobile_base_state",
    "/neck_forward_position_controller/commands",
    "/neck_forward_position_controller/transition_event",
    "/parameter_events",
    "/r_arm/averaged_target_pose",
    "/r_arm/cart_target_pose",
    "/r_arm/ik_target_pose",
    "/r_arm/target_pose",
    "/r_arm_forward_position_controller/commands",
    "/r_arm_forward_position_controller/transition_event",
    "/r_arm_reachability_states",
    "/robot_description",
    "/rosout",
    "/tf",
    "/tf_static",
]


# Helper function to get configuration file path
def get_fake(package: str, filename: str, context: LaunchContext) -> str:
    return PathJoinSubstitution([FindPackageShare(package), "config", filename]).perform(context)


def make_blup(sound: str):
    return ExecuteProcess(
        cmd=["aplay", f"/home/reachy/dev/reachy2_sounds/{sound}"],
        output="both",
        log_cmd=True,
        shell=True,
    )


def get_rviz_conf_choices() -> list[str]:
    """Lists all .rviz files in reachy_description/config, stripping the .rviz extension."""

    rviz_config_choices = []
    for file in os.listdir(os.path.dirname(os.path.realpath(__file__)) + "/../../reachy_description/config"):
        if file.endswith(".rviz"):
            rviz_config_choices.append(file[:-5])
    return rviz_config_choices


# To avoid any confusion, duty commands to explains tacus as a ROS launch entity
def parseTacus(tacus, context):
    """Recursively parses a list of various launch object and returns a list of nodes."""

    def browse_sub_tacus(sub_tacus_list):
        for sub_tacus in sub_tacus_list:
            extract_tacus = parseTacus(sub_tacus, context)
            # if tacus is not empty
            if extract_tacus:
                node_list.append(*extract_tacus)

    node_list = []
    if isinstance(tacus, Node):
        node_list.append(tacus)

    elif isinstance(tacus, (IncludeLaunchDescription, LaunchDescription)):
        # launchDescription does not have a condition attribute
        if isinstance(tacus, LaunchDescription) or (tacus.condition is not None and tacus.condition.evaluate(context=context)):
            browse_sub_tacus(tacus.visit(context=context))

    elif isinstance(tacus, RegisterEventHandler):
        browse_sub_tacus(tacus.event_handler.describe()[1])

    elif isinstance(tacus, TimerAction):
        browse_sub_tacus(tacus.actions)

    elif isinstance(
        tacus, (DeclareLaunchArgument, LogInfo, ExecuteProcess, GroupAction, OpaqueFunction, SetLaunchConfiguration)
    ):
        pass
    else:
        print("Unhandled type of tacus. Exiting.")
        print(tacus)
        exit(1)

    return node_list


def title_print(title: str) -> LogInfo:
    return LogInfo(msg=f"\n{'-'*50}\n{' '*20}{title}\n{'-'*50}\n")


def get_node_list(nodes, context: LaunchContext):
    # "Unpack not allowed in comprehension" :facepalm:
    packed_node_list = [parseTacus(node, context) for node in nodes]
    # print(packed_node_list)
    flattened_list = [node for sublist in packed_node_list for node in sublist]
    return flattened_list


non_critical_nodes = ["foxglove_bridge", "rviz2"]


def check_node_status(context):
    name = context.get_locals_as_dict()["event"].action.name
    # some clean up
    name = "-".join(name.rsplit("-", 1)[:-1])

    with report_lock:
        if context.get_locals_as_dict()["event"].returncode == 0:
            success_nodes.append(name)
        else:
            failed_nodes.append(name)
            if name not in non_critical_nodes:
                LogInfo(msg=f"Critical node failed : [{name}]").execute(context)
                # LogInfo(msg=f"Grace period of {SHUTDOWN_GRACE_PERIOD} seconds before shutdown, to write rosbag").execute(
                #     context
                # )
                # instead of exit , just send a ctrl+c signal to the launch file, exit left zombies
                ExecuteProcess(
                    name="rosbag_snapshot_dump",
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/rosbag2_recorder/snapshot",
                        "rosbag2_interfaces/Snapshot",
                    ],
                    output="screen",
                ).execute(context)
                EmitEvent(event=Shutdown(reason=f"Node failed : [{name}]")).execute(context)
                # send the emit event after a timer
                # TimerAction(
                #     period=float(SHUTDOWN_GRACE_PERIOD),  # sadly TimerAction does not accept int
                #     actions=[
                #         EmitEvent(event=Shutdown(reason=f"Node failed : [{name}]")),
                #     ],
                # ).execute(context)
                # os.kill(os.getpid(), signal.SIGINT)


def watcher_report(nb_node: int, delay: float = 10.0) -> TimerAction:
    """Sets up a timer to log a report on node statuses after a delay."""

    def print_report(context):
        reporting_nodes = len(failed_nodes) + len(success_nodes)
        unmonitored_nodes = nb_node - reporting_nodes
        report_message = f"Node Watcher Report\n Logging to {get_current_run_log_dir()}\n Un-monitored nodes[{unmonitored_nodes}/{nb_node}]\n Failed[{len(failed_nodes)}/{reporting_nodes}]: {failed_nodes}\n Success[{len(success_nodes)}/{reporting_nodes}]: {success_nodes}"
        return [title_print(report_message)]

    return TimerAction(
        period=delay,
        actions=[
            OpaqueFunction(function=lambda context: print_report(context)),
            make_blup(CORE_UP_SOUND),
        ],
    )


def build_watchers_from_node_list(node_list: list[Node]) -> list[RegisterEventHandler]:
    """Creates event handlers to monitor node exits."""
    watchmen = []
    for node in node_list:
        watchmen.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=node,
                    on_exit=[
                        OpaqueFunction(function=lambda context: check_node_status(context)),
                    ],
                )
            )
        )

    watchmen.append(watcher_report(len(node_list)))
    watchmen.append(
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    LogInfo(
                        msg=[
                            "Launch was asked to shutdown: ",
                            LocalSubstitution("event.reason"),
                        ]
                    )
                ],
            )
        ),
    )
    return watchmen


def get_current_run_log_dir() -> str:
    log_dir = get_logging_directory()

    # List all items in the log directory
    paths = [os.path.join(log_dir, name) for name in os.listdir(log_dir)]
    # Filter out files, keep only directories
    dirs = [path for path in paths if os.path.isdir(path)]
    # Sort directories by modification time, newest first
    return max(dirs, key=os.path.getmtime)


def clear_bags_and_logs(nb_runs_to_keep: int = 10):
    # list dir inside the log directory
    log_dir = get_logging_directory()

    # List all items in the log directory
    paths = [os.path.join(log_dir, name) for name in os.listdir(log_dir)]

    # Filter out files, keep only directories
    dirs = [path for path in paths if os.path.isdir(path)]
    # filter dir list, keep only dir that are not nameds Log, Orbbec log dir irrelvant to consider
    dirs = [dir for dir in dirs if os.path.basename(dir) != "Log"]

    current_log_dir = get_current_run_log_dir()
    # Pre-filter, keeping only dir with non-empty rosbag
    for dir in dirs:

        # dont remove current log dir, ofc its quite empty
        if dir == current_log_dir:
            continue

        if os.path.exists(os.path.join(dir, "reachy.bag", "metadata.yaml")):
            try:
                with open(os.path.join(dir, "reachy.bag", "metadata.yaml"), "r") as file:
                    metadata = yaml.load(file, Loader=yaml.FullLoader)
                    if metadata["rosbag2_bagfile_information"]["message_count"] > 0:
                        # keeping dir
                        continue

            except Exception as e:
                shutil.rmtree(dir)
                dirs.remove(dir)

        shutil.rmtree(dir)
        dirs.remove(dir)

    for file in os.listdir(log_dir):
        if file.endswith(".log"):
            os.remove(os.path.join(log_dir, file))

    if len(dirs) <= nb_runs_to_keep:
        print("No logs&bags to clear")
        return

    # Sort directories in name descending order
    dirs.sort(reverse=True)

    for dir in dirs[nb_runs_to_keep:]:
        shutil.rmtree(dir)
