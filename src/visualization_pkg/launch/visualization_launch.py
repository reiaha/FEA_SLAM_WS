import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/pi/FEA_SLAM_WS/src/fea_slam/rviz/robot_autonomous.rviz'],
            output='screen',
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}  # force Mesa software rendering — avoids VideoCore GLSL sampler conflict
        ),
    ])