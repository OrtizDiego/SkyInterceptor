import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    launch_dir = os.path.join(pkg_interceptor, 'launch')
    controller_params = os.path.join(pkg_interceptor, 'config', 'controller_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_mode = LaunchConfiguration('mission_mode')
    perception_source = LaunchConfiguration('perception_source')
    use_vision = PythonExpression(["'", perception_source, "' == 'vision'"])
    use_groundtruth = PythonExpression(["'", perception_source, "' == 'groundtruth'"])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Vision pipeline: stereo + YOLO + 3D localizer -> /target/detection_3d
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'perception.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_vision),
    )

    # Ground-truth target source: Gazebo entity poses -> /target/detection_3d
    groundtruth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'groundtruth.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_groundtruth),
    )

    # Owns the mission mode: /mission/set_mode service and latched /mission/mode topic
    mission_manager = Node(
        package='interceptor_drone',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'initial_mode': mission_mode}],
    )

    # Tracker and mode-specific planners
    guidance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'guidance.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'mission_mode': mission_mode,
        }.items()
    )

    trajectory_controller = Node(
        package='interceptor_drone',
        executable='trajectory_controller_node',
        name='trajectory_controller_node',
        output='screen',
        parameters=[controller_params, {'use_sim_time': use_sim_time}],
    )

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
        DeclareLaunchArgument(
            'mission_mode',
            default_value='follow',
            choices=['follow', 'intercept'],
            description='Mission mode: follow (aerial filming) or intercept (counter-UAS)'
        ),
        DeclareLaunchArgument(
            'perception_source',
            default_value='groundtruth',
            choices=['vision', 'groundtruth'],
            description='Where /target/detection_3d comes from'
        ),
        simulation,
        perception,
        groundtruth,
        mission_manager,
        guidance,
        trajectory_controller,
        rviz,
    ])
