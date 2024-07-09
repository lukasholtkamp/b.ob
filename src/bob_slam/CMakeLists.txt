cmake_minimum_required(VERSION 3.8)
project(bob_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()