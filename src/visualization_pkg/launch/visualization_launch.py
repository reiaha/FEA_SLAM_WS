from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    default_rviz = '/home/pi/FEA_SLAM_WS/src/fea_slam/rviz/robot_autonomous.rviz'
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config file path'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
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
            }                                                                                           
        ),
    ])