cmake_minimum_required(VERSION 3.0.2)
project(motor_driver_library)

find_package(fmt)

list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")
find_package(pigpio)

include_directories(include/motor_alarm include/motor_encoder include/motor_driver include/wheel)

include_directories(src)

add_library(${PROJECT_NAME} src/MotorDriver.cpp src/MotorEncoder.cpp src/MotorAlarm.cpp src/Wheel.cpp)

add_executable(Wheel_test test/Wheel_test.cpp)

target_include_directories(Wheel_test PRIVATE include)

target_link_libraries(Wheel_test fmt::fmt)
target_link_libraries(Wheel_test pigpio)