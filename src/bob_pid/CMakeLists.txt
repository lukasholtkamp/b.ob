cmake_minimum_required(VERSION 3.8)
project(bob_pid)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include)

#install
install(
  DIRECTORY src config include
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

## COMPILE
add_executable(pid_node src/pid_node.cpp)
ament_target_dependencies(pid_node
"rclcpp"
"sensor_msgs"
"geometry_msgs"
"nav_msgs"
"tf2"
"control_msgs"
"std_msgs"
)

#install
install(TARGETS pid_node DESTINATION lib/${PROJECT_NAME})

ament_package()
