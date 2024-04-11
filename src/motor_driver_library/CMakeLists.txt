cmake_minimum_required(VERSION 3.0.2)
project(motor_driver_library)

find_package(fmt)

list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")
find_package(pigpio)

include_directories(include/motor_driver)
include_directories(src)

add_library(${PROJECT_NAME} src/MotorDriver.cpp)

add_executable(motor_controller_test test/motor_controller_test.cpp)

target_include_directories(motor_controller_test PRIVATE include)

target_link_libraries(motor_controller_test fmt::fmt)
target_link_libraries(motor_controller_test pigpio)