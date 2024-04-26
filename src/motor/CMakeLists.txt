cmake_minimum_required(VERSION 3.5)

project(Wheel_test)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Werror=format)
endif()

list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/pigpio-master/util")

find_package(fmt)
find_package(pigpio)

include_directories(
    include/wheel
    src/Wheel
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED 
    src/Wheel/MotorDriver.cpp 
    src/Wheel/MotorEncoder.cpp 
    src/Wheel/MotorAlarm.cpp 
    src/Wheel/Wheel.cpp
)

add_executable(Wheel_test test/Wheel_test.cpp)

target_link_libraries(Wheel_test fmt::fmt)
target_link_libraries(Wheel_test pigpio)
