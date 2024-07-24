cmake_minimum_required(VERSION 3.8)
project(bob_scan)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Install
install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

## Compile
add_executable(scan_node src/scan_node.cpp)
ament_target_dependencies(scan_node
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "control_msgs"
)

install(TARGETS scan_node DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/keyboard_input_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
