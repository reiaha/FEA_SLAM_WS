#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_share = os.path.expanduser('~/FEA_SLAM_WS/src/localization')

    return LaunchDescription([

        # Include your existing sensor fusion (EKF) launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensor_fusion_launch.py')
            )
        ),

        # Publish a static transform from base_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # SLAM Toolbox node (mapping)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'slam_params_file': os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
            }],
            remappings=[
                ('/scan', '/scan'),
                ('/odom', '/odom'), 
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),

        # RViz visualization for mapping
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam_toolbox.rviz')]
        )
    ])
