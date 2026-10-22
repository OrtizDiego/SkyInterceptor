import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch the INTERCEPT demo scenario, ready to run.

    A Crazyflie intruder accelerates from a standstill and curves while the
    interceptor climbs and engages it with proportional navigation. The
    interceptor starts behind the intruder, so the stereo cameras keep it in
    frame and the guidance still has to lead the target's turn.

    Switch the guidance law with the launch argument:

        ros2 launch interceptor_drone intercept_demo.launch.py guidance_law:=pn

    and the behaviour of the intruder with:

        ros2 launch interceptor_drone intercept_demo.launch.py target_behaviour:=weave

    The default perception source is the ground truth: the vision pipeline needs
    a YOLO model with a 'uav' class, which does not exist yet (plan 3.2). With
    'vision' the cameras, the depth map and the detection overlay come alive in
    RViz, but the intruder is not detected: COCO has no drone class.
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
            'target_behaviour',
            default_value='curve',
            choices=['hover', 'waypoints', 'straight', 'weave', 'curve'],
            description='The behaviour of the intruder: hover, waypoints, straight, '
                        'weave or curve. The default is the accelerating and curving '
                        'profile the demo was built on.',
        ),
        DeclareLaunchArgument(
            'auto_arm',
            default_value='true',
            description='Arm the INTERCEPT mission automatically after a delay',
        ),
        DeclareLaunchArgument(
            'perception_source',
            default_value='groundtruth',
            choices=['vision', 'groundtruth'],
            description='Where /target/detection_3d comes from. Use "vision" to '
                        'see the cameras, the depth map and the detection overlay '
                        'in RViz, but the intruder is not detected: COCO has no '
                        'drone class (plan 3.2).',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_interceptor, 'launch', 'interceptor_full.launch.py')),
            launch_arguments={
                'mission_mode': 'intercept',
                'perception_source': LaunchConfiguration('perception_source'),
                # 'apn' (augmented PN) reaches the intruder; 'pn' (pure PN)
                # misses it, which is the point of the comparison.
                'guidance_law': LaunchConfiguration('guidance_law'),
                # The behaviour of the intruder: hover, waypoints, straight, weave
                # or the accelerating and curving profile of the demo.
                'target_behaviour': LaunchConfiguration('target_behaviour'),
                # The demo runs unattended: the mission is armed shortly after start.
                'auto_arm': LaunchConfiguration('auto_arm'),
            }.items()
        ),
    ])
