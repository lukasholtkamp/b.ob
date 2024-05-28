cmake_minimum_required(VERSION 3.8)
project(bob_test)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

#install
install(
  DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}
)

## COMPILE
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node
"rclcpp"
"sensor_msgs")

#install
install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})

ament_package()