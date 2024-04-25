cmake_minimum_required(VERSION 3.16)
project(Wheel_test LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(fmt)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")
find_package(pigpio)

include_directories(include/motor_alarm include/motor_encoder include/motor_driver include/wheel)
include_directories(src)

add_library(${PROJECT_NAME}
  src/MotorAlarm.cpp
  src/MotorDriver.cpp
  src/MotorEncoder.cpp
  src/Wheel.cpp
)

add_executable(Wheel_test test/Wheel_test.cpp)

target_include_directories(Wheel_test PRIVATE incude)


target_link_libraries(motor_encoder_test fmt::fmt)
target_link_libraries(motor_encoder_test pigpio)

ament_target_dependencies(Wheel_test PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_Wheel_test
    test/test_Wheel_test.cpp)
  target_link_libraries(test_Wheel_test
    Wheel_test
  )
  ament_target_dependencies(test_Wheel_test
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

endif()

install(
  DIRECTORY include/
  DESTINATION include/Wheel_test
)
install(TARGETS Wheel_test Wheel_test_parameters
  EXPORT export_Wheel_test
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_Wheel_test HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()