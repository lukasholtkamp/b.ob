cmake_minimum_required(VERSION 3.5)

project(Motor_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fmt)
find_package(rclcpp REQUIRED)
list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")
find_package(pigpio)

include(CheckIncludeFiles)

include_directories(include/motor_alarm include/motor_encoder include/motor_driver include/wheel src)
add_library(${PROJECT_NAME} src/MotorDriver.cpp src/MotorEncoder.cpp src/MotorAlarm.cpp src/Wheel.cpp)

add_executable(Wheel_test test/Wheel_test.cpp)

ament_target_dependencies(Wheel_test
"fmt"
"pigpio"
"rclcpp")

install(TARGETS Wheel_test DESTINATION lib/${PROJECT_NAME})

ament_package()