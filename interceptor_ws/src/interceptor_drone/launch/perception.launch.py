import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    perception_params = os.path.join(pkg_interceptor, 'config', 'perception_params.yaml')
    stereo_sync_params = os.path.join(pkg_interceptor, 'config', 'stereo_sync_params.yaml')
    # The detector of the INTERCEPT mode is a model with a uav class, the one
    # of the FOLLOW mode is the COCO model. The file with the parameters of
    # the detector is loaded on top of the perception parameters, so it
    # replaces the block of the detector.
    uav_detector_params = os.path.join(
        pkg_interceptor, 'config', 'uav_detector_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    detector = LaunchConfiguration('detector')

    stereo_sync_node = Node(
        package='interceptor_drone',
        executable='stereo_sync_node',
        name='stereo_sync_node',
        output='screen',
        parameters=[stereo_sync_params, {'use_sim_time': use_sim_time}],
    )

    stereo_depth_processor = Node(
        package='interceptor_drone',
        executable='stereo_depth_processor',
        name='stereo_depth_processor',
        output='screen',
        parameters=[perception_params, {'use_sim_time': use_sim_time}],
    )

    target_detector = Node(
        package='interceptor_drone',
        executable='target_detector.py',
        name='target_detector',
        output='screen',
        parameters=[
            perception_params,
            uav_detector_params,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(PythonExpression(["'", detector, "' == 'uav'"])),
    )

    coco_detector = Node(
        package='interceptor_drone',
        executable='target_detector.py',
        name='target_detector',
        output='screen',
        parameters=[perception_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", detector, "' == 'coco'"])),
    )

    target_3d_localizer = Node(
        package='interceptor_drone',
        executable='target_3d_localizer',
        name='target_3d_localizer',
        output='screen',
        parameters=[perception_params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'detector',
            default_value='coco',
            choices=['coco', 'uav'],
            description='The detector of the target: the COCO model (FOLLOW) '
                        'or the model with the uav class (INTERCEPT, plan 3.2)'
        ),
        stereo_sync_node,
        stereo_depth_processor,
        target_detector,
        coco_detector,
        target_3d_localizer,
    ])
