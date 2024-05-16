cmake_minimum_required(VERSION 3.8)
project(bob_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY config launch src
  DESTINATION share/${PROJECT_NAME}
)

add_executable(drive_selection_node src/drive_selection_node.cpp)
ament_target_dependencies(drive_selection_node
"rclcpp"
"sensor_msgs")

install(TARGETS drive_selection_node DESTINATION lib/${PROJECT_NAME})

ament_package()