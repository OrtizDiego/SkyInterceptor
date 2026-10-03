import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    config_dir = os.path.join(pkg_interceptor, 'config')
    ekf_params = os.path.join(config_dir, 'ekf_params.yaml')
    follow_params = os.path.join(config_dir, 'follow_params.yaml')
    intercept_params = os.path.join(config_dir, 'intercept_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_mode = LaunchConfiguration('mission_mode')

    # Tracks every target in both modes; the mode files carry the class whitelists
    target_tracker = Node(
        package='interceptor_drone',
        executable='target_tracker_node',
        name='target_tracker_node',
        output='screen',
        parameters=[ekf_params, follow_params, intercept_params, {'use_sim_time': use_sim_time}],
    )

    # INTERCEPT guidance (stub until P3.3 replaces it with intercept_guidance_node)
    guidance_controller = Node(
        package='interceptor_drone',
        executable='guidance_controller_node',
        name='guidance_controller_node',
        output='screen',
        parameters=[intercept_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", mission_mode, "' == 'intercept'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'mission_mode',
            default_value='follow',
            choices=['follow', 'intercept'],
            description='Mission mode'
        ),
        target_tracker,
        guidance_controller,
    ])
