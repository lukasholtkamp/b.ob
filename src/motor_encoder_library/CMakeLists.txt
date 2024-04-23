cmake_minimum_required(VERSION 3.0.2)
project(motor_encoder_library)

find_package(fmt)

list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")
find_package(pigpio)

include_directories(include/motor_encoder)
include_directories(include/matplotlib)

include_directories(src)

add_library(${PROJECT_NAME} src/MotorEncoder.cpp)

add_executable(motor_encoder_test test/motor_encoder_test.cpp)

target_include_directories(motor_encoder_test PRIVATE include)

target_link_libraries(motor_encoder_test fmt::fmt)
target_link_libraries(motor_encoder_test pigpio)