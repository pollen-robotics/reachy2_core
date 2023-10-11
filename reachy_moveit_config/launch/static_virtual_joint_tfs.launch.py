from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from moveit_utils.launch_utils import generate_moveit_config_args, generate_moveit_config
from launch_ros.actions import Node
from srdfdom.srdf import SRDF


def opaque_launch_description(context, *args, **kwargs):
    moveit_config = generate_moveit_config(context)
    # moveit_config = MoveItConfigsBuilder("reachy_v2", package_name="reachy_moveit_config").to_moveit_configs()
    # return generate_static_virtual_joint_tfs_launch(moveit_config)

    name_counter = 0
    virtual_joints_nodes = []
    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            virtual_joints_nodes.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                    parameters=[
                        {'use_sim_time': LaunchConfiguration("use_sim_time")},
                    ]
                )
            )
            name_counter += 1
    return [
        *virtual_joints_nodes,
    ]


def generate_launch_description():
    return LaunchDescription([
        *generate_moveit_config_args(),
        OpaqueFunction(function=opaque_launch_description)
    ])
