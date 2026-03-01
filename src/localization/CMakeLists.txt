cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0009 NEW)
project(localization)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_module(
  MODULE localization
)

ament_package()
