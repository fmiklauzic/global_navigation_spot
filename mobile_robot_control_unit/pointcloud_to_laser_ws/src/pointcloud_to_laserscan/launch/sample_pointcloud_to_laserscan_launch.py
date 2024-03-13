from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in','/camera/camera/depth/color/points'),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
            parameters=[{
                'target_frame': 'camera_link',
                'transform_tolerance': 1.0,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -0.7592,  
                'angle_max': 0.7592,  
                'angle_increment': 0.01,  # M_PI/360.0
                'scan_time': 0.0112, 
                'range_min': 0.2,
                'range_max': 5.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 10
            }],
            name='pointcloud_to_laserscan'
        )
    ])
