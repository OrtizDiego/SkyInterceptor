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
    safety_params = os.path.join(pkg_interceptor, 'config', 'safety_params.yaml')
    intercept_params = os.path.join(pkg_interceptor, 'config', 'intercept_params.yaml')
    # RViz is off for the scripted runs: there is no display for it.
    rviz_enabled = LaunchConfiguration('rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_mode = LaunchConfiguration('mission_mode')
    perception_source = LaunchConfiguration('perception_source')
    guidance_law = LaunchConfiguration('guidance_law', default='apn')
    # The behaviour of the target drone: hover, waypoints, straight, weave or curve.
    target_behaviour = LaunchConfiguration('target_behaviour', default='curve')
    # The mission manager always starts disarmed. The demo arms it after a delay so
    # the engagement starts on its own.
    auto_arm = LaunchConfiguration('auto_arm', default='false')
    # Where the interceptor and the target drone start. The interceptor starts
    # behind the intruder on the same line, so the line of sight runs along its
    # nose: the stereo cameras then keep the intruder in frame for the whole
    # approach, and the guidance still has to lead the target's turn.
    interceptor_x = LaunchConfiguration('interceptor_x', default='2.0')
    interceptor_y = LaunchConfiguration('interceptor_y', default='5.0')
    target_x = LaunchConfiguration('target_x', default='10.0')
    target_y = LaunchConfiguration('target_y', default='5.0')
    # The seed of the ground-truth measurement noise (0 = random). Fixing it makes
    # a run reproducible.
    groundtruth_seed = LaunchConfiguration('groundtruth_seed', default='0')
    use_vision = PythonExpression(["'", perception_source, "' == 'vision'"])
    use_groundtruth = PythonExpression(["'", perception_source, "' == 'groundtruth'"])
    # Gazebo GUI: headless runs (CI, scripted perception checks) need it off
    gui = LaunchConfiguration('gui')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulation.launch.py')),
        # trajectory_controller_node commands world-frame velocities
        launch_arguments={
            'use_sim_time': use_sim_time,
            'command_frame': 'world',
            'gui': gui,
            'x': interceptor_x,
            'y': interceptor_y,
            'target_x': target_x,
            'target_y': target_y,
        }.items()
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
        launch_arguments={
            'use_sim_time': use_sim_time,
            'seed': groundtruth_seed,
        }.items(),
        condition=IfCondition(use_groundtruth),
    )

    # Owns the mission mode: /mission/set_mode service and latched /mission/mode topic
    mission_manager = Node(
        package='interceptor_drone',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'initial_mode': mission_mode,
            'auto_arm': auto_arm,
        }],
    )

    # Tracker and mode-specific planners
    guidance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'guidance.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'mission_mode': mission_mode,
            'guidance_law': guidance_law,
            'target_behaviour': target_behaviour,
            'target_drone_params_file': LaunchConfiguration('target_drone_params_file'),
        }.items()
    )

    # The safety filter: /setpoint/raw -> /setpoint/safe. It sits between the
    # planner and the controller and has the final say on every setpoint. It
    # also owns the INTERCEPT engagement gate and the abort, so it reads the
    # intercept parameters as well.
    safety_filter = Node(
        package='interceptor_drone',
        executable='safety_filter_node',
        name='safety_filter_node',
        output='screen',
        parameters=[safety_params, intercept_params, {'use_sim_time': use_sim_time}],
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
        output='screen',
        condition=IfCondition(rviz_enabled),
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
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI (false for headless runs)'
        ),
        DeclareLaunchArgument(
            'guidance_law',
            default_value='apn',
            choices=['pn', 'apn'],
            description='INTERCEPT guidance law: pure PN or augmented PN',
        ),
        DeclareLaunchArgument(
            'groundtruth_seed',
            default_value='0',
            description='Seed of the ground-truth measurement noise (0 = random)',
        ),
        DeclareLaunchArgument(
            'target_behaviour',
            default_value='curve',
            choices=['hover', 'waypoints', 'straight', 'weave', 'curve'],
            description='The behaviour of the target drone: hover, waypoints, '
                        'straight, weave or curve',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz (false for the scripted runs)',
        ),
        DeclareLaunchArgument(
            'target_drone_params_file',
            default_value=os.path.join(pkg_interceptor, 'config', 'target_drone_params.yaml'),
            description='Parameter file of the target drone (the intruder)',
        ),
        DeclareLaunchArgument(
            'auto_arm',
            default_value='false',
            description='Arm the INTERCEPT mission automatically after a delay',
        ),
        DeclareLaunchArgument(
            'interceptor_x',
            default_value='2.0',
            description='Interceptor spawn x [m]',
        ),
        DeclareLaunchArgument(
            'interceptor_y',
            default_value='5.0',
            description='Interceptor spawn y [m]',
        ),
        DeclareLaunchArgument(
            'target_x',
            default_value='10.0',
            description='Target drone spawn x [m]',
        ),
        DeclareLaunchArgument(
            'target_y',
            default_value='5.0',
            description='Target drone spawn y [m]',
        ),
        simulation,
        perception,
        groundtruth,
        mission_manager,
        guidance,
        safety_filter,
        trajectory_controller,
        rviz,
    ])
