import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Target tracker
    target_tracker = Node(
        package='interceptor_drone',
        executable='target_tracker_node',
        name='target_tracker_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ekf.process_noise_pos': 0.1,
            'ekf.process_noise_vel': 0.5,
            'ekf.process_noise_acc': 1.0,
            'ekf.measurement_noise_pos': 0.1,
        }]
    )
    
    # Guidance controller
    guidance_controller = Node(
        package='interceptor_drone',
        executable='guidance_controller_node',
        name='guidance_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'guidance.nav_constant_far': 4.0,
            'guidance.nav_constant_mid': 5.0,
            'guidance.max_acceleration': 20.0,
            'guidance.terminal_range': 10.0,
            'guidance.mid_range': 50.0,
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        target_tracker,
        guidance_controller,
    ])
