cmake_minimum_required(VERSION 3.8)
project(base101_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
