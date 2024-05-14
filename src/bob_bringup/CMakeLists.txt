cmake_minimum_required(VERSION 3.8)
project(bob_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()