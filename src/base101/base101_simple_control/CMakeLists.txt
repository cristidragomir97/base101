cmake_minimum_required(VERSION 3.8)
project(base101_simple_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
