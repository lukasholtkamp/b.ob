cmake_minimum_required(VERSION 3.8)
project(gazebo_radar_plugin)

# Set compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(TBB REQUIRED)  # Include TBB

# Include directories
include_directories(
  include
  ${CMAKE_INCLUDE_CURRENT_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}  # Include TBB directories
)

# Generate ROS interfaces
set(msg_files
  "msg/ModeCRadar.msg"
  "msg/ModeCRadarSummary.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# Executable target
add_library(ModeCRadarPlugin SHARED
  src/ModeCRadarPlugin.cc
)

# Dependencies for the executable
ament_target_dependencies(ModeCRadarPlugin
  rclcpp
)

# Link against ROS typesupport
rosidl_get_typesupport_target(typesupport_cpp "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(ModeCRadarPlugin
  ${typesupport_cpp}
)

# Link against TBB
target_link_libraries(ModeCRadarPlugin
  ${TBB_IMPORTED_TARGETS}
)

# Install targets
install(TARGETS ModeCRadarPlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Ament package export
ament_export_dependencies(rosidl_default_runtime
  DEPENDENCIES std_msgs
)

ament_package()

# Testing (if needed)
if(BUILD_TESTING)
  ament_lint_auto_find_test_dependencies()
endif()
