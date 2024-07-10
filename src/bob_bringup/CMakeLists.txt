cmake_minimum_required(VERSION 3.8)
project(bob_bringup)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
add_library(mpu6050_hardware_interface SHARED
  src/mpu6050_hardware_interface.cpp
  src/mpu6050_lib.cpp
)

target_include_directories(mpu6050_hardware_interface PRIVATE	include)

ament_target_dependencies(mpu6050_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})


# Link i2c library to mpu6050_hardware_interface target
target_link_libraries(mpu6050_hardware_interface i2c)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface mpu6050_hardware.xml)

#install
install(
  DIRECTORY include/
  DESTINATION include/mpu6050_hardware_interface
)

install(
  TARGETS mpu6050_hardware_interface
  DESTINATION share/${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch src
  DESTINATION share/${PROJECT_NAME}
)

# EXPORTS
ament_export_libraries(mpu6050_hardware_interface)

include_directories(include)

## COMPILE
add_executable(drive_selection_node src/drive_selection_node.cpp)
add_executable(mpu6050_offsets src/mpu6050_lib.cpp src/mpu6050_offsets.cpp)
add_executable(mpu6050_covariances src/mpu6050_lib.cpp src/mpu6050_covariances.cpp)



# Install Cpp executables
install(TARGETS
  mpu6050_offsets
  mpu6050_covariances
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(drive_selection_node
"rclcpp"
"sensor_msgs"
"geometry_msgs"
"nav_msgs"
"tf2"
"control_msgs")


#install
install(TARGETS drive_selection_node DESTINATION lib/${PROJECT_NAME})

# Link i2c to mpu6050_offsets and mpu6050_covariances targets
target_link_libraries(mpu6050_offsets i2c)
target_link_libraries(mpu6050_covariances i2c)

ament_package()