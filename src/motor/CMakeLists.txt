cmake_minimum_required(VERSION 3.5)

project(Motor_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(fmt)
list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")
find_package(pigpio)

include_directories(include/wheel src/Wheel src/Odometry ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/Wheel/MotorDriver.cpp src/Wheel/MotorEncoder.cpp src/Wheel/MotorAlarm.cpp src/Wheel/Wheel.cpp)

add_executable(Wheel_test test/Wheel_test.cpp)

target_link_libraries(Wheel_test fmt::fmt)
target_link_libraries(Wheel_test pigpio)
