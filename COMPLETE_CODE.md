# FEA-SLAM Complete Code Repository

**Hardware Operation Only - No Gazebo Simulation**

This document contains the complete code for all key files in the FEA-SLAM project. This is a hardware-focused implementation with no simulation support. All code is designed to run on real robot hardware.

---

## 1. robot_core.xacro
**Location**: `src/fea_slam/description/robot_core.xacro`

```xml
<?xml version="1.0"?>
<robot name="stack_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials (use local include so xacro can resolve when launched via ament/launch) -->
  <xacro:include filename="materials.xacro"/>

  <!-- Parameters -->
  <xacro:property name="wheel_radius" value="0.065"/>
  <xacro:property name="wheel_width" value="0.04"/>
  <!-- Lateral offset of each drive wheel from robot centerline (increase to push wheels outward) -->
  <xacro:property name="wheel_y_offset" value="0.09"/>
  <xacro:property name="caster_radius" value="0.03"/>
  <xacro:property name="shelf_distance" value="0.08"/>
  <!-- Plate heights (z positions). Use a single spacing so distances can be equal. -->
  <xacro:property name="plate_spacing" value="0.09"/>
  <xacro:property name="lower_z" value="0"/>
  <xacro:property name="middle_z" value="${plate_spacing}"/>
  <xacro:property name="upper_z" value="${2*plate_spacing}"/>
  <xacro:property name="plate_radius" value="0.105"/>
  <xacro:property name="plate_thickness" value="0.01"/>
  <xacro:property name="lidar_radius" value="0.07"/>
  <xacro:property name="lidar_length" value="0.03"/>
  <xacro:property name="lidar_height" value="0.05"/>
  <xacro:property name="wheel_z_offset" value="-0.03"/>
  <xacro:property name="caster_support_height" value="${wheel_radius}"/>
  <xacro:property name="ultrasonic_z_offset" value="0.03"/>

  <!-- Support stick macro -->
  <xacro:macro name="support_stick" params="name x y height parent_link z_offset">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 ${height/2}" rpy="0 0 0"/>
        <geometry><cylinder radius="0.005" length="${height}"/></geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${height/2}" rpy="0 0 0"/>
        <geometry><cylinder radius="0.005" length="${height}"/></geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 ${height/2}" rpy="0 0 0"/>
        <mass value="0.05"/>
        <inertia ixx="0.00005" iyy="0.00005" izz="0.00005" ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>
    <joint name="${name}_joint" type="fixed">
      <parent link="${parent_link}"/>
      <child link="${name}"/>
      <origin xyz="${x} ${y} ${z_offset}" rpy="0 0 0"/>
    </joint>
  </xacro:macro>

  <!-- Base footprint (ground projection of robot center) -->
  <link name="base_footprint"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="${plate_radius}" length="0.002"/></geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="${plate_radius}" length="0.002"/></geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-05" iyy="1e-05" izz="1e-05" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- Middle plate -->
  <link name="middle_plate">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="${plate_radius}" length="${plate_thickness}"/></geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="${plate_radius}" length="${plate_thickness}"/></geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="middle_plate_joint" type="fixed">
    <parent link="base_link"/>
    <child link="middle_plate"/>
    <origin xyz="0 0 ${middle_z}" rpy="0 0 0"/>
  </joint>

  <!-- Lower plate -->
  <link name="lower_plate">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="${plate_radius}" length="${plate_thickness}"/></geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="${plate_radius}" length="${plate_thickness}"/></geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="lower_plate_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lower_plate"/>
    <origin xyz="0 0 ${lower_z}" rpy="0 0 0"/>
  </joint>

  <!-- Upper plate -->
  <link name="upper_plate">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="${plate_radius}" length="${plate_thickness}"/></geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="${plate_radius}" length="${plate_thickness}"/></geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="upper_plate_joint" type="fixed">
    <parent link="base_link"/>
    <child link="upper_plate"/>
    <origin xyz="0 0 ${upper_z}" rpy="0 0 0"/>
  </joint>

  <!-- Lidar -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 ${lidar_height}" rpy="0 0 0"/>
      <geometry><cylinder radius="${lidar_radius}" length="${lidar_length}"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${lidar_height}" rpy="0 0 0"/>
      <geometry><cylinder radius="${lidar_radius}" length="${lidar_length}"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${lidar_height}" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.0005" iyy="0.0005" izz="0.0005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="upper_plate"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Ultrasonic sensor (single) -->
  <link name="ultrasonic_link">
   <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://fea_slam/description/meshes/hcsr04_high.stl" scale="1 1 1"/>
      </geometry>
      <material name="grey"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.04 0.02 0.02"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.02"/>
      <inertia ixx="1e-05" iyy="1e-05" izz="1e-05" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="ultrasonic_joint" type="fixed">
    <parent link="middle_plate"/>
    <child link="ultrasonic_link"/>
    <origin xyz="0.10 0 ${ultrasonic_z_offset}" rpy="0 0 0"/>
  </joint>

  <!-- Wheels (spaced outward) -->
  <link name="wheel_left">
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0003" iyy="0.0003" izz="0.0003" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 ${wheel_y_offset} ${wheel_z_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_right">
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0003" iyy="0.0003" izz="0.0003" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -${wheel_y_offset} ${wheel_z_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Casters -->
  <link name="caster_front">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="${caster_radius}"/></geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="${caster_radius}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00005" iyy="0.00005" izz="0.00005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="caster_front_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_front"/>
    <origin xyz="0.07 0 -${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <link name="caster_rear">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="${caster_radius}"/></geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="${caster_radius}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00005" iyy="0.00005" izz="0.00005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="caster_rear_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_rear"/>
    <origin xyz="-0.07 0 -${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- Support sticks between plates -->
  <xacro:support_stick name="support_fl" x="0.07" y="0.07" height="${upper_z - lower_z}" parent_link="base_link" z_offset="${lower_z}"/>
  <xacro:support_stick name="support_fr" x="0.07" y="-0.07" height="${upper_z - lower_z}" parent_link="base_link" z_offset="${lower_z}"/>
  <xacro:support_stick name="support_rl" x="-0.07" y="0.07" height="${upper_z - lower_z}" parent_link="base_link" z_offset="${lower_z}"/>
  <xacro:support_stick name="support_rr" x="-0.07" y="-0.07" height="${upper_z - lower_z}" parent_link="base_link" z_offset="${lower_z}"/>

  <!-- Support sticks for lidar -->
  <xacro:support_stick name="lidar_stick_fl" x="0.05" y="0.05" height="${lidar_height}" parent_link="upper_plate" z_offset="0"/>
  <xacro:support_stick name="lidar_stick_fr" x="0.05" y="-0.05" height="${lidar_height}" parent_link="upper_plate" z_offset="0"/>
  <xacro:support_stick name="lidar_stick_rl" x="-0.05" y="0.05" height="${lidar_height}" parent_link="upper_plate" z_offset="0"/>
  <xacro:support_stick name="lidar_stick_rr" x="-0.05" y="-0.05" height="${lidar_height}" parent_link="upper_plate" z_offset="0"/>

  <!-- Support sticks for casters -->
  <xacro:support_stick name="support_caster_fl_1" x="0.065" y="0.02" height="${caster_support_height}" parent_link="base_link" z_offset="-${wheel_radius}"/>
  <xacro:support_stick name="support_caster_fl_2" x="0.065" y="-0.02" height="${caster_support_height}" parent_link="base_link" z_offset="-${wheel_radius}"/>
  <xacro:support_stick name="support_caster_fl_3" x="0.085" y="0.02" height="${caster_support_height}" parent_link="base_link" z_offset="-${wheel_radius}"/>
  <xacro:support_stick name="support_caster_fl_4" x="0.085" y="-0.02" height="${caster_support_height}" parent_link="base_link" z_offset="-${wheel_radius}"/>

  <xacro:support_stick name="support_caster_rr_1" x="-0.065" y="0.02" height="${caster_support_height}" parent_link="base_link" z_offset="-${wheel_radius}"/>
  <xacro:support_stick name="support_caster_rr_2" x="-0.065" y="-0.02" height="${caster_support_height}" parent_link="base_link" z_offset="-${wheel_radius}"/>
  <xacro:support_stick name="support_caster_rr_3" x="-0.085" y="0.02" height="${caster_support_height}" parent_link="base_link" z_offset="-${wheel_radius}"/>
  <xacro:support_stick name="support_caster_rr_4" x="-0.085" y="-0.02" height="${caster_support_height}" parent_link="base_link" z_offset="-${wheel_radius}"/>

</robot>
```

---

## 2. display.launch.py
**Location**: `src/fea_slam/launch/display.launch.py`

```python
#!/usr/bin/env python3

"""
Complete launch file for FEA SLAM robot visualization and operation.
Launches robot state publisher, joint state publisher (with GUI), and RViz.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    
    # Get package directories
    pkg_fea_slam = get_package_share_directory('fea_slam')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('gui')
    rviz_config = LaunchConfiguration('rviz_config')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true (for bag playback or external simulators)'
    )
    
    declare_use_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch joint state publisher GUI for manual joint control'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_fea_slam, 'config', 'view_robot.rviz'),
        description='Full path to RViz config file'
    )
    
    # Process robot description from XACRO
    xacro_file = os.path.join(pkg_fea_slam, 'description', 'robot.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint State Publisher Node (without GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_gui)
    )
    
    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_gui)
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_gui,
        declare_rviz_config,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
```

---

## 3. robot_full.launch.py
**Location**: `src/fea_slam/launch/robot_full.launch.py`

```python
#!/usr/bin/env python3

"""
Full robot launch file for FEA SLAM robot (Hardware Operation).
Includes robot state publisher, sensor drivers (LiDAR, ultrasonic), 
localization (SLAM Toolbox), and visualization.
Designed for real hardware, not Gazebo simulation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Get package directories
    pkg_fea_slam = get_package_share_directory('fea_slam')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_rviz = LaunchConfiguration('rviz')
    use_slam = LaunchConfiguration('slam')
    use_lidar = LaunchConfiguration('lidar')
    
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true (for bag playback or external simulators)'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    declare_use_slam = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Launch SLAM Toolbox for mapping and localization'
    )
    
    declare_use_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='true',
        description='Launch YDLiDAR driver'
    )
    
    # Process robot description from XACRO
    xacro_file = os.path.join(pkg_fea_slam, 'description', 'robot.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time,
             'frame_prefix': namespace}
        ]
    )
    
    # Joint State Publisher Node (for non-simulated joints)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # YDLiDAR Launch (conditional)
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
        ]),
        condition=IfCondition(use_lidar)
    )
    
    # SLAM Toolbox Launch (conditional)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('localization'), 'launch', 'slam_toolbox_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_slam)
    )
    
    # Sensor Fusion Launch (if available)
    try:
        sensor_fusion_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('localization'), 'launch', 'sensor_fusion_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        )
        has_sensor_fusion = True
    except:
        has_sensor_fusion = False
    
    # RViz Node (conditional)
    rviz_config_file = os.path.join(pkg_fea_slam, 'config', 'view_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    # Visualization Package Launch (if available)
    try:
        visualization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('visualization_pkg'), 'launch', 'visualization_launch.py')
            ])
        )
        has_visualization = True
    except:
        has_visualization = False
    
    # Build launch description
    ld = LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_use_rviz,
        declare_use_slam,
        declare_use_lidar,
        robot_state_publisher_node,
        joint_state_publisher_node,
        ydlidar_launch,
        slam_toolbox_launch,
        rviz_node
    ])
    
    # Add optional launches if available
    if has_sensor_fusion:
        ld.add_action(sensor_fusion_launch)
    if has_visualization:
        ld.add_action(visualization_launch)
    
    return ld
```

---

## 4. package.xml
**Location**: `src/fea_slam/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>fea_slam</name>
  <version>0.1.0</version>
  <description>FEA-SLAM: Frontier-based Exploration and SLAM for Autonomous Robot Navigation in GPS-Denied Environments. Integrates frontier-based exploration with SLAM for autonomous localization, mapping, and area coverage.</description>
  <maintainer email="rheakhimgaudiano00@gmail.com">Rhea Khim Gaudiano</maintainer>
  <license>Apache-2.0</license>
  
  <!-- Authors and Contributors -->
  <author email="rheakhimgaudiano00@gmail.com">Rhea Khim Gaudiano</author>
  
  <!-- Keywords and project tags -->
  <url type="repository">https://github.com/rhea-khim/FEA_SLAM_WS</url>
  <url type="bugtracker">https://github.com/rhea-khim/FEA_SLAM_WS/issues</url>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Runtime Dependencies -->
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>joint_state_publisher_gui</depend>
  <depend>rviz2</depend>
  <depend>xacro</depend>
  <depend>slam_toolbox</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  
  <!-- Optional Dependencies (for full system operation) -->
  <exec_depend>ydlidar_ros2_driver</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_core</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 5. CMakeLists.txt
**Location**: `src/fea_slam/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.5)
project(fea_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

## Build and Installation Instructions

### Build the Package
```bash
cd ~/FEA_SLAM_WS
colcon build --packages-select fea_slam
source install/setup.bash
```

### Launch Commands

**Display Robot Model Only:**
```bash
ros2 launch fea_slam display.launch.py
```

**Full SLAM System:**
```bash
ros2 launch fea_slam robot_full.launch.py
```

**Without RViz:**
```bash
ros2 launch fea_slam robot_full.launch.py rviz:=false
```

**With Bag File Playback:**
```bash
ros2 launch fea_slam robot_full.launch.py use_sim_time:=true lidar:=false
# In another terminal:
ros2 bag play <bag_file> --clock
```

---

## Project Structure

```
FEA_SLAM_WS/
├── src/
│   └── fea_slam/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── README.md
│       ├── config/
│       │   ├── view_robot.rviz
│       │   └── empty.yaml
│       ├── description/
│       │   ├── robot.urdf.xacro
│       │   ├── robot_core.xacro (with base_footprint)
│       │   ├── materials.xacro
│       │   └── meshes/
│       ├── launch/
│       │   ├── rsp.launch.py
│       │   ├── display.launch.py
│       │   ├── robot_full.launch.py
│       │   └── README.md
│       └── worlds/
├── build/
├── install/
├── log/
└── PROJECT_OBJECTIVES.md
```

---

## Key Features

✅ **Base Footprint**: Added as ground reference frame (root link)
✅ **SLAM Toolbox Integration**: Full support for graph-based SLAM with loop closure
✅ **Frontier-Based Exploration**: Framework for autonomous boundary discovery
✅ **Hardware-Ready**: YDLiDAR and ultrasonic sensor integration
✅ **Visualization**: Pre-configured RViz with LaserScan, Map, and TF displays
✅ **Parameterized Design**: All values computed from base properties for easy modification
✅ **Multi-Robot Support**: Namespace support for multi-robot systems
✅ **Research-Focused**: Complete documentation of FEA-SLAM objectives

