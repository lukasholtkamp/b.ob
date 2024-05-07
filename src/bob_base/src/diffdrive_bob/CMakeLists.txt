cmake_minimum_required(VERSION 3.16)
project(diffdrive_bob LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")
find_package(pigpio)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  diffdrive_bob
  SHARED
  hardware/diffbot_system.cpp
  hardware/Wheel/MotorAlarm.cpp
  hardware/Wheel/MotorDriver.cpp
  hardware/Wheel/MotorEncoder.cpp
  hardware/Wheel/Wheel.cpp
  hardware/Wheel/wheel.cpp
)

include_directories(hardware/include/wheel Wheel)

include_directories(
  include
  pigpio_INCLUDES
)

target_compile_features(diffdrive_bob PUBLIC cxx_std_17)
target_include_directories(diffdrive_bob PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/diffdrive_bob>
)
ament_target_dependencies(
  diffdrive_bob PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(diffdrive_bob PUBLIC pigpiod_if2)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_BOB_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive_bob.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffdrive_bob
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/diffdrive_bob
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diffdrive_bob
)
install(TARGETS diffdrive_bob
  EXPORT export_diffdrive_bob
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_diffdrive_bob HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
