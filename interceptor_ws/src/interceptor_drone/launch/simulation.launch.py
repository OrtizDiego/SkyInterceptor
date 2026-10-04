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
        DeclareLaunchArgument(
            'wind_x', default_value='0.0', description='Mean wind towards +x (east) [m/s]'),
        DeclareLaunchArgument(
            'wind_y', default_value='0.0', description='Mean wind towards +y (north) [m/s]'),
        DeclareLaunchArgument(
            'wind_z', default_value='0.0', description='Mean vertical wind [m/s]'),
        DeclareLaunchArgument(
            'wind_gust_stddev', default_value='0.0',
            description='Turbulence intensity (std. dev. of the gusts) [m/s]'),
        gazebo,
        spawn_drone,
        static_tf,
        robot_state_publisher,
    ])
