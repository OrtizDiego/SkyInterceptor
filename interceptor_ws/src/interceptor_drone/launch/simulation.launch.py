import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package share directory
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default=os.path.join(
        pkg_interceptor, 'worlds', 'intercept_scenario.world'))

    # Process the xacro file at launch time so the wind can be set from the command line
    xacro_file = os.path.join(pkg_interceptor, 'urdf', 'interceptor_quadrotor.urdf.xacro')
    robot_desc = ParameterValue(Command([
        'xacro ', xacro_file,
        ' wind_x:=', LaunchConfiguration('wind_x'),
        ' wind_y:=', LaunchConfiguration('wind_y'),
        ' wind_z:=', LaunchConfiguration('wind_z'),
        ' wind_gust_stddev:=', LaunchConfiguration('wind_gust_stddev'),
        ' command_frame:=', LaunchConfiguration('command_frame'),
    ]), value_type=str)

    # Target drone (the intruder): a Crazyflie 2.1 spawned next to the interceptor.
    # robot_state_publisher always publishes on /robot_description, so the target
    # description is published by a plain node on its own topic.
    target_desc = ParameterValue(Command([
        'xacro ', os.path.join(pkg_interceptor, 'urdf', 'target_drone.urdf.xacro'),
    ]), value_type=str)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'gui': LaunchConfiguration('gui'),
            'params_file': os.path.join(pkg_interceptor, 'config', 'gazebo_params.yaml'),
        }.items()
    )

    # Spawn interceptor drone on its skids, next to the person's patrol square and
    # facing it (the person walks the 15 x 15 m square with corners (0, 0) and (15, 15))
    #
    # The timeout is generous on purpose: gzserver first fetches the online Gazebo
    # model database, which on a cold or slow network takes well over a minute, and
    # /spawn_entity is not available until the world has finished loading. The
    # default 30 s timeout makes the spawn fail on a slow start.
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'interceptor',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.3',
            '-Y', LaunchConfiguration('yaw'),
            '-timeout', LaunchConfiguration('spawn_timeout'),
        ],
        output='screen'
    )

    # Spawn the target drone. It is kinematic: target_drone_behavior_node moves it
    # with /set_entity_state, so it needs no flight dynamics of its own.
    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'target_description',
            '-entity', 'target_drone',
            '-x', LaunchConfiguration('target_x'),
            '-y', LaunchConfiguration('target_y'),
            '-z', LaunchConfiguration('target_z'),
            '-Y', LaunchConfiguration('target_yaw'),
            '-timeout', LaunchConfiguration('spawn_timeout'),
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

    # Robot state publisher (propeller joint angles come from the dynamics plugin)
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

    # The target drone needs its own state publisher for the spinning propellers.
    # robot_state_publisher only ever publishes on /robot_description, so it cannot
    # be used twice: the target description goes on its own topic instead.
    target_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='target_robot_state_publisher',
        output='screen',
        remappings=[('robot_description', 'target_description')],
        parameters=[{
            'robot_description': target_desc,
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
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('x', default_value='-3.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='-3.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.785', description='Spawn yaw [rad]'),
        DeclareLaunchArgument('target_x', default_value='30.0',
                              description='Target drone spawn x [m]'),
        DeclareLaunchArgument('target_y', default_value='0.0',
                              description='Target drone spawn y [m]'),
        DeclareLaunchArgument('target_z', default_value='12.0',
                              description='Target drone spawn z [m]'),
        DeclareLaunchArgument('target_yaw', default_value='0.0',
                              description='Target drone spawn yaw [rad]'),
        DeclareLaunchArgument(
            'wind_x', default_value='0.0', description='Mean wind towards +x (east) [m/s]'),
        DeclareLaunchArgument(
            'wind_y', default_value='0.0', description='Mean wind towards +y (north) [m/s]'),
        DeclareLaunchArgument(
            'wind_z', default_value='0.0', description='Mean vertical wind [m/s]'),
        DeclareLaunchArgument(
            'wind_gust_stddev', default_value='0.0',
            description='Turbulence intensity (std. dev. of the gusts) [m/s]'),
        DeclareLaunchArgument(
            'command_frame', default_value='heading', choices=['heading', 'world'],
            description='Frame of /cmd_vel: heading (keyboard teleop) or world '
                        '(trajectory_controller_node)'),
        DeclareLaunchArgument(
            'spawn_timeout', default_value='180.0',
            description='Seconds to wait for /spawn_entity. gzserver fetches the '
                        'online model database on start, which can take a minute '
                        'before the world is loaded and the service exists.'),
        gazebo,
        spawn_drone,
        spawn_target,
        static_tf,
        robot_state_publisher,
        target_state_publisher,
    ])
