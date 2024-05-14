cmake_minimum_required(VERSION 3.8)
project(bob_base)

list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pigpio REQUIRED)

# COMPILE
add_library(bob_hardware SHARED 
          src/bob_hardware.cpp
          src/motor_encoder.cpp
          src/wheel.cpp
)

include_directories(
  include/bob_base
  pigpio_INCLUDES
)

target_include_directories(bob_hardware PRIVATE include)

target_compile_features(bob_hardware PUBLIC cxx_std_17)
target_include_directories(bob_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/bob_base>
)
ament_target_dependencies(
  bob_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link pigpio library to bob_hardware target
target_link_libraries(bob_hardware pigpiod_if2)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface bob_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/bob_hardware
)

install(
  TARGETS bob_hardware
  DESTINATION share/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# EXPORTS 
ament_export_libraries(bob_hardware)
ament_package()