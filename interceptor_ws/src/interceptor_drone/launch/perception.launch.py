import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_interceptor = get_package_share_directory('interceptor_drone')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Stereo sync node
    stereo_sync_node = Node(
        package='interceptor_drone',
        executable='stereo_sync_node',
        name='stereo_sync_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'sync_slop': 0.005,  # 5ms tolerance
        }],
        remappings=[
            ('/stereo/left/image_raw', '/stereo/left/image_raw'),
            ('/stereo/right/image_raw', '/stereo/right/image_raw'),
            ('/stereo/left/camera_info', '/stereo/left/camera_info'),
            ('/stereo/right/camera_info', '/stereo/right/camera_info'),
        ]
    )
    
    # Stereo depth processor
    stereo_depth_processor = Node(
        package='interceptor_drone',
        executable='stereo_depth_processor',
        name='stereo_depth_processor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'stereo.baseline': 0.12,
            'stereo.fx': 535.4,
            'stereo.fy': 535.4,
            'stereo.cx': 320.5,
            'stereo.cy': 240.5,
            'num_disparities': 128,
            'block_size': 11,
        }]
    )
    
    # YOLO Target Detector
    target_detector = Node(
        package='interceptor_drone',
        executable='target_detector.py',
        name='target_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': 'yolov8n.pt',
            'confidence_threshold': 0.3,
            'device': 'cpu',
        }]
    )
    
    # Target 3D Localizer
    target_3d_localizer = Node(
        package='interceptor_drone',
        executable='target_3d_localizer',
        name='target_3d_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'map',
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        stereo_sync_node,
        stereo_depth_processor,
        target_detector,
        target_3d_localizer,
    ])
