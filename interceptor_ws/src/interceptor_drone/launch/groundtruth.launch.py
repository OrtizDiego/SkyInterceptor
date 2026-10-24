import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    # The parameter file of the ground-truth source. The follow scenarios see the
    # people of their own world, so they pass their own file: it is loaded on top
    # of the default one, so the settings it does not mention (the noise, the
    # occlusion cylinders) are the ones of the default file.
    default_params = os.path.join(pkg_interceptor, 'config', 'groundtruth_params.yaml')
    groundtruth_params = LaunchConfiguration('params_file', default=default_params)

    use_sim_time = LaunchConfiguration('use_sim_time')
    # The seed of the measurement noise and the dropout. 0 draws a fresh one on
    # every run; a fixed value makes the run reproducible.
    seed = LaunchConfiguration('seed', default='0')

    # Gazebo ground truth -> /target/detection_3d, in place of the vision pipeline
    groundtruth_target_node = Node(
        package='interceptor_drone',
        executable='groundtruth_target_node',
        name='groundtruth_target_node',
        output='screen',
        parameters=[
            default_params,
            groundtruth_params,
            {
                'use_sim_time': use_sim_time,
                'seed': ParameterValue(seed, value_type=int),
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
            'seed',
            default_value='0',
            description='Seed of the measurement noise and the dropout (0 = random)'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_interceptor, 'config', 'groundtruth_params.yaml'),
            description='Parameter file of the ground-truth source',
        ),
        groundtruth_target_node,
    ])
