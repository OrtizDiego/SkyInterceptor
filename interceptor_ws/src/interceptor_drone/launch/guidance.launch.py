import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    config_dir = os.path.join(pkg_interceptor, 'config')
    ekf_params = os.path.join(config_dir, 'ekf_params.yaml')
    follow_params = os.path.join(config_dir, 'follow_params.yaml')
    intercept_params = os.path.join(config_dir, 'intercept_params.yaml')
    # The intruder's parameter file. The intercept scenarios pass a file of their
    # own, with the start pose and the path of the run.
    target_drone_params = LaunchConfiguration('target_drone_params_file')

    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_mode = LaunchConfiguration('mission_mode')
    # "pn" (pure proportional navigation) or "apn" (augmented PN). The launch
    # argument overrides the value in intercept_params.yaml.
    guidance_law = LaunchConfiguration('guidance_law', default='apn')
    # The behaviour of the target drone: hover, waypoints, straight, weave or curve.
    # The launch argument overrides the value in target_drone_params.yaml.
    target_behaviour = LaunchConfiguration('target_behaviour', default='curve')

    # Tracks every target in both modes; the mode files carry the class whitelists
    target_tracker = Node(
        package='interceptor_drone',
        executable='target_tracker_node',
        name='target_tracker_node',
        output='screen',
        parameters=[ekf_params, follow_params, intercept_params, {'use_sim_time': use_sim_time}],
    )

    # INTERCEPT guidance: PN / augmented PN with the SEARCH -> PURSUE -> CAPTURE
    # phase machine (plan 3.3).
    intercept_guidance = Node(
        package='interceptor_drone',
        executable='intercept_guidance_node',
        name='intercept_guidance_node',
        output='screen',
        parameters=[
            intercept_params,
            {
                'use_sim_time': use_sim_time,
                'guidance.law': guidance_law,
            },
        ],
        condition=IfCondition(PythonExpression(["'", mission_mode, "' == 'intercept'"])),
    )

    # FOLLOW planner: the framing presets (BEHIND, SIDE, LEAD, ORBIT) of plan 4.2. It
    # publishes on /setpoint/raw, like the intercept guidance, and the safety filter
    # is between the planner and the controller. Only one of the two planners runs in
    # a launch, so they never fight over the topic.
    follow_planner = Node(
        package='interceptor_drone',
        executable='follow_planner_node',
        name='follow_planner_node',
        output='screen',
        parameters=[
            follow_params,
            {
                'use_sim_time': use_sim_time,
            },
        ],
        condition=IfCondition(PythonExpression(["'", mission_mode, "' == 'follow'"])),
    )

    # Target drone behaviour: the intruder flies the scenario of the INTERCEPT demo.
    # Simulation only, and it has its own config file.
    target_behavior = Node(
        package='interceptor_drone',
        executable='target_drone_behavior_node',
        name='target_drone_behavior_node',
        output='screen',
        parameters=[
            target_drone_params,
            {
                'use_sim_time': use_sim_time,
                'behaviour': target_behaviour,
            },
        ],
        condition=IfCondition(PythonExpression(["'", mission_mode, "' == 'intercept'"])),
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
            description='Mission mode'
        ),
        DeclareLaunchArgument(
            'guidance_law',
            default_value='apn',
            choices=['pn', 'apn'],
            description='Guidance law: pure PN or augmented PN',
        ),
        DeclareLaunchArgument(
            'target_behaviour',
            default_value='curve',
            choices=['hover', 'waypoints', 'straight', 'weave', 'curve'],
            description='The behaviour of the target drone: hover, waypoints, '
                        'straight, weave or curve',
        ),
        DeclareLaunchArgument(
            'target_drone_params_file',
            default_value=os.path.join(config_dir, 'target_drone_params.yaml'),
            description='Parameter file of the target drone (the intruder)',
        ),
        target_tracker,
        intercept_guidance,
        follow_planner,
        target_behavior,
    ])
