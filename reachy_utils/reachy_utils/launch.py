import os
import signal
import threading

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

    elif isinstance(tacus, (DeclareLaunchArgument, LogInfo, ExecuteProcess, GroupAction)):
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
                # instead of exit , just send a ctrl+c signal to the launch file, exit left zombies
                EmitEvent(event=Shutdown(reason=f"Node failed : [{name}]")).execute(context)
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
