cmake_minimum_required(VERSION 3.5)
project(wheel)

find_package(fmt)

list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")
find_package(pigpio)

include_directories(
    include/wheel
    src/Wheel
)

add_library(
    ${PROJECT_NAME}
    src/Wheel/MotorDriver.cpp 
    src/Wheel/MotorEncoder.cpp 
    src/Wheel/MotorAlarm.cpp 
    src/Wheel/Wheel.cpp
)

add_executable(Wheel_test test/Wheel_test.cpp)

target_include_directories(Wheel_test PRIVATE include)

target_link_libraries(Wheel_test fmt::fmt)
target_link_libraries(Wheel_test pigpio)
