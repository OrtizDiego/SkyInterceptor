import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """The INTERCEPT demo scenario, ready to run.

    A Crazyflie intruder accelerates from a standstill and curves while the
    interceptor climbs and engages it with proportional navigation. The interceptor
    starts off to one side, so it has to lead the target rather than chase it.

    Switch the guidance law with the launch argument:

        ros2 launch interceptor_drone intercept_demo.launch.py guidance_law:=pn
    """
    pkg_interceptor = get_package_share_directory('interceptor_drone')

    return LaunchDescription([
        DeclareLaunchArgument(
            'guidance_law',
            default_value='apn',
            choices=['pn', 'apn'],
            description='Guidance law: pure PN or augmented PN',
        ),
        DeclareLaunchArgument(
            'auto_arm',
            default_value='true',
            description='Arm the INTERCEPT mission automatically after a delay',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_interceptor, 'launch', 'interceptor_full.launch.py')),
            launch_arguments={
                'mission_mode': 'intercept',
                'perception_source': 'groundtruth',
                # 'apn' (augmented PN) reaches the intruder; 'pn' (pure PN)
                # misses it, which is the point of the comparison.
                'guidance_law': LaunchConfiguration('guidance_law'),
                # The demo runs unattended: the mission is armed shortly after start.
                'auto_arm': LaunchConfiguration('auto_arm'),
            }.items()
        ),
    ])
