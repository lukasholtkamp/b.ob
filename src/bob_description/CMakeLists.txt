cmake_minimum_required(VERSION 3.8)
project(bob_description)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# install
install(
  DIRECTORY urdf ros2_control launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()