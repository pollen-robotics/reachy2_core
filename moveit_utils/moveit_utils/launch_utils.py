from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Start RViz2 automatically with this launch file.',
    choices=['true', 'false']
)


def generate_moveit_config_args():
    return (DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Start RViz2 automatically with this launch file. toto',
        choices=['true', 'false']
    ),
    DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Sgz.',
        choices=['true', 'false']
    ),
    use_sim_time_arg,
    DeclareLaunchArgument(
        'use_moveit_gazebo',
        default_value='false',
        description='Start RViz2 automatically with this launch file.',
        choices=['true', 'false']
    ),
    DeclareBooleanLaunchArg(
        "use_rviz",
        default_value=True
    ))


def generate_moveit_config(context):
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    use_gazebo = LaunchConfiguration('use_gazebo').perform(context)
    use_moveit_gazebo = LaunchConfiguration('use_moveit_gazebo').perform(context)

    moveit_config = MoveItConfigsBuilder("reachy_v2", package_name="reachy_moveit_config")
    # moveit_config = moveit_config.sensors_3d(None)  # be sure to disable the 3D sensor
    moveit_config = moveit_config.robot_description(mappings={'use_fake_hardware': use_fake_hardware,
                                                              'use_gazebo': use_gazebo,
                                                              'use_moveit_gazebo': use_moveit_gazebo})
    moveit_config = moveit_config.moveit_cpp(
        file_path=get_package_share_directory("reachy_moveit_config") + "/config/moveit_cpp.yaml")
    return moveit_config.to_moveit_configs()
