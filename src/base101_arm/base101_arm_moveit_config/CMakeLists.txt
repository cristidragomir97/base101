cmake_minimum_required(VERSION 3.8)
project(base101_arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY srdf config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
