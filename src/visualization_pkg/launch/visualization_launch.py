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
            additional_env={
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'QT_OPENGL': 'software',
                'QT_XCB_FORCE_SOFTWARE_OPENGL': '1',
                'MESA_GL_VERSION_OVERRIDE': '3.3',
                'MESA_GLSL_VERSION_OVERRIDE': '330',
                'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
                'GALLIUM_DRIVER': 'llvmpipe',
                'OGRE_RTT_MODE': 'Copy',
            }  # Robust software GL path for Pi/Mesa to avoid GLSL sampler conflicts in RViz map shaders
        ),
    ])