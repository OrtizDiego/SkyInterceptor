import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    """
    Launch one scenario of the FOLLOW mode, with the metrics of the exit criteria.

    The scenarios are the four of plan 5.2.4: walker, jogger, occlusion and
    bystander. The launch argument follow_scenario picks one of them.

    The run is reproducible: the seed of the ground-truth source and the seed of
    the scenario (which shifts the start of the target along its path) fix the
    whole run.
    """
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    config_dir = os.path.join(pkg_interceptor, 'config')
    scenario_params = os.path.join(config_dir, 'follow_scenario_params.yaml')
    safety_params = os.path.join(config_dir, 'safety_params.yaml')
    follow_world = os.path.join(pkg_interceptor, 'worlds', 'follow_scenario.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scenario = LaunchConfiguration('follow_scenario')
    seed = LaunchConfiguration('groundtruth_seed')
    result_dir = LaunchConfiguration('result_dir')
    run_duration_s = LaunchConfiguration('run_duration_s')

    # The whole system in FOLLOW mode, on the world of the scenarios, with the
    # ground-truth source that sees the people of the scenarios.
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_interceptor, 'launch', 'interceptor_full.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'mission_mode': 'follow',
            'perception_source': 'groundtruth',
            'groundtruth_params_file': scenario_params,
            'world_file': follow_world,
            'groundtruth_seed': seed,
            'spawn_target_drone': 'false',
            'rviz': LaunchConfiguration('rviz'),
            # The drone spawns 8 m behind the start of the target and climbs: that
            # is the framing point of the first frame of the run.
            'interceptor_x': '-2.0',
            'interceptor_y': '0.0',
            # The world of the scenarios is loaded from the model database on a
            # cold start, which can take minutes: the spawn waits for it.
            'spawn_timeout': LaunchConfiguration('spawn_timeout'),
        }.items()
    )

    # The scenario: it moves the target and the second person of the scenario.
    scenario_node = Node(
        package='interceptor_drone',
        executable='follow_scenario_node',
        name='follow_scenario_node',
        output='screen',
        parameters=[
            scenario_params,
            {
                'use_sim_time': use_sim_time,
                'scenario': scenario,
                'seed': seed,
            },
        ],
    )

    # The metrics: the minimum distances, the framing error and the retention of
    # the target track, with the verdict against the exit criteria of the phase.
    metrics = Node(
        package='interceptor_drone',
        executable='metrics_recorder_node',
        name='metrics_recorder_node',
        output='screen',
        parameters=[
            scenario_params,
            safety_params,
            {
                'use_sim_time': use_sim_time,
                'metrics.scenario': scenario,
                'metrics.seed': seed,
                'metrics.run_duration_s': run_duration_s,
                'metrics.output_path': PathJoinSubstitution([result_dir, 'summary.json']),
                'metrics.csv_path': PathJoinSubstitution([result_dir, 'timeseries.csv']),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'follow_scenario',
            default_value='walker',
            choices=['walker', 'jogger', 'occlusion', 'bystander'],
            description='The scenario of the run'
        ),
        DeclareLaunchArgument(
            'groundtruth_seed',
            default_value='0',
            description='Seed of the run: the measurement noise and the start of the '
                        'target on its path (0 = random)'
        ),
        DeclareLaunchArgument(
            'run_duration_s',
            default_value='60.0',
            description='The length of the run [s]'
        ),
        DeclareLaunchArgument(
            'result_dir',
            default_value='/tmp/follow_result',
            description='Where the summary and the time series are written'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz (false for the scripted runs)'
        ),
        DeclareLaunchArgument(
            'spawn_timeout',
            default_value='300.0',
            description='Seconds to wait for the drone to appear in the world'
        ),
        system,
        scenario_node,
        metrics,
    ])
