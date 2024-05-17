cmake_minimum_required(VERSION 3.8)
project(bob_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

#install
install(
  DIRECTORY config launch src
  DESTINATION share/${PROJECT_NAME}
)

## COMPILE
add_executable(drive_selection_node src/drive_selection_node.cpp)
ament_target_dependencies(drive_selection_node
"rclcpp"
"sensor_msgs")

#install
install(TARGETS drive_selection_node DESTINATION lib/${PROJECT_NAME})

ament_package()