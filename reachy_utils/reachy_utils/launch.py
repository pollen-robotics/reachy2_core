from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def get_fake(package, filename: str, context) -> str:
    return PathJoinSubstitution(
        [FindPackageShare(package), "config", filename]
    ).perform(context)
