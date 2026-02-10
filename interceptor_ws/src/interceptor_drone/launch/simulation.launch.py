import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default=os.path.join(
        pkg_interceptor, 'worlds', 'intercept_scenario.world'))
    
    # Process xacro file
    xacro_file = os.path.join(pkg_interceptor, 'urdf', 'interceptor_quadrotor.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Spawn interceptor drone
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'interceptor',
            '-x', '0',
            '-y', '0',
            '-z', '1.0',
            '-Y', '0'
        ],
        output='screen'
    )
    
    # Spawn target (person)
    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-database', 'person_standing',
            '-entity', 'target_person',
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '3.14'
        ],
        output='screen'
    )
    
    # Static TF: map to odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value=os.path.join(pkg_interceptor, 'worlds', 'intercept_scenario.world'),
            description='Full path to world file'
        ),
        gazebo,
        spawn_drone,
        spawn_target,
        static_tf,
        robot_state_publisher,
    ])
