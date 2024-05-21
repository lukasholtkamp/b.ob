# Based on: https://github.com/TheNoobInventor/lidarbot/blob/main/lidarbot_base/CMakeLists.txt
# Date of Retrieval: 17.05.2024

cmake_minimum_required(VERSION 3.16)
project(bob_base LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  pigpio
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library( bob_base SHARED
  src/diffbot_system.cpp
  src/motor.cpp
  src/wheel.cpp
)

# include folder for hpp files
include_directories(
  include
  pigpio_INCLUDES
  include/bob_base
)

target_compile_features(bob_base PUBLIC cxx_std_17)
target_include_directories(bob_base PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/bob_base>
)
ament_target_dependencies(
  bob_base PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# link library to executable 
target_link_libraries(bob_base PUBLIC pigpiod_if2)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_BOB_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface bob_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/bob_base
)
install(TARGETS bob_base
  EXPORT export_bob_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_bob_base HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

