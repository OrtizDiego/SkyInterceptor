import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch one scripted INTERCEPT run and record its metrics.

    The full system in INTERCEPT mode, headless, armed automatically, plus
    intercept_metrics_node, which appends one row of CSV to output_path when the
    engagement ends (captured, aborted or timed out) and then exits.

    The intruder's path comes from target_drone_params_file: the batch runner
    (run_intercept_batch.py) writes one per run, with the start pose and the
    behaviour of the scenario, so a run is reproducible from its seed.

        ros2 launch interceptor_drone intercept_scenario.launch.py scenario:=hover seed:=1
    """
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    intercept_params = os.path.join(pkg_interceptor, 'config', 'intercept_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_interceptor, 'launch', 'interceptor_full.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'mission_mode': 'intercept',
            'perception_source': 'groundtruth',
            'guidance_law': LaunchConfiguration('guidance_law'),
            'target_behaviour': LaunchConfiguration('target_behaviour'),
            'target_drone_params_file': LaunchConfiguration('target_drone_params_file'),
            'auto_arm': 'true',
            'groundtruth_seed': LaunchConfiguration('seed'),
            'interceptor_x': LaunchConfiguration('interceptor_x'),
            'interceptor_y': LaunchConfiguration('interceptor_y'),
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )

    # Reads the capture thresholds from the guidance's own parameter file, so the
    # scorer and the guidance agree on what a capture is.
    metrics = Node(
        package='interceptor_drone',
        executable='intercept_metrics_node',
        name='intercept_metrics_node',
        output='screen',
        parameters=[
            intercept_params,
            {
                'use_sim_time': use_sim_time,
                'scenario': LaunchConfiguration('scenario'),
                'seed': LaunchConfiguration('seed'),
                'output_path': LaunchConfiguration('output_path'),
                'timeout': LaunchConfiguration('timeout'),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        DeclareLaunchArgument('scenario', default_value='hover',
                              description='The name the run is recorded under'),
        DeclareLaunchArgument('seed', default_value='1',
                              description='The seed of the run (ground-truth noise)'),
        DeclareLaunchArgument('output_path', default_value='/tmp/intercept_runs.csv',
                              description='The CSV the row of the run is appended to'),
        DeclareLaunchArgument('timeout', default_value='75.0',
                              description='The simulation time a run may take [s], '
                                          'counted from the first odometry'),
        DeclareLaunchArgument('guidance_law', default_value='apn', choices=['pn', 'apn'],
                              description='Guidance law: pure PN or augmented PN'),
        DeclareLaunchArgument(
            'target_behaviour', default_value='hover',
            choices=['hover', 'waypoints', 'straight', 'weave', 'curve'],
            description='The behaviour the intruder starts with'),
        DeclareLaunchArgument(
            'target_drone_params_file',
            default_value=os.path.join(pkg_interceptor, 'config', 'target_drone_params.yaml'),
            description='Parameter file of the intruder (start pose and path)'),
        DeclareLaunchArgument('interceptor_x', default_value='40.0',
                              description='Interceptor spawn x [m]'),
        DeclareLaunchArgument('interceptor_y', default_value='50.0',
                              description='Interceptor spawn y [m]'),
        DeclareLaunchArgument('target_x', default_value='60.0',
                              description='Intruder spawn x [m]'),
        DeclareLaunchArgument('target_y', default_value='50.0',
                              description='Intruder spawn y [m]'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Start the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start RViz'),
        system,
        metrics,
    ])
