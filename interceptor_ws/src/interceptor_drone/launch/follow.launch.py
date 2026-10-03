import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Full system in FOLLOW mode; other interceptor_full arguments pass through."""
    pkg_interceptor = get_package_share_directory('interceptor_drone')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_interceptor, 'launch', 'interceptor_full.launch.py')),
            launch_arguments={'mission_mode': 'follow'}.items()
        ),
    ])
