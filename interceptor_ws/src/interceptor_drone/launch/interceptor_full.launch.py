import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Include simulation launch
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_interceptor, 'launch', 'simulation.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Include perception launch
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_interceptor, 'launch', 'perception.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Include guidance launch
    guidance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_interceptor, 'launch', 'guidance.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Trajectory controller
    trajectory_controller = Node(
        package='interceptor_drone',
        executable='trajectory_controller_node',
        name='trajectory_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'controller.kp_pos': 1.0,
            'controller.ki_pos': 0.0,
            'controller.kd_pos': 0.5,
            'controller.kp_vel': 2.0,
            'controller.ki_vel': 0.1,
            'controller.kd_vel': 0.5,
            'controller.max_velocity': 41.7,
            'controller.max_altitude': 200.0,
            'controller.min_altitude': 2.0,
        }]
    )
    
    # Hector interface
    hector_interface = Node(
        package='interceptor_drone',
        executable='hector_interface_node',
        name='hector_interface_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_interceptor, 'rviz', 'interceptor_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        simulation,
        perception,
        guidance,
        trajectory_controller,
        hector_interface,
        rviz,
    ])
