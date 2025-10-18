#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localization',
            executable='sensor_fusion',
            name='imu_lidar_ekf',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[
                ('scan', '/scan'),
                ('imu/data_raw', '/imu/data_raw'),
                ('odom', '/odom')
            ]
        )
    ])
