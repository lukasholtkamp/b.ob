cmake_minimum_required(VERSION 3.10)

project(MotorDriver
		LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(JetsonGPIO)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${CMAKE_BINARY_DIR}/lib)

# Sources
set(MotorDriver_LIB_SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/src/MotorDriver.cpp
	#${CMAKE_CURRENT_SOURCE_DIR}/src/runMotor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/MotorDriver.h
	)

add_library(MotorDriver SHARED ${MotorDriver_LIB_SRCS})

target_include_directories(MotorDriver PUBLIC
							$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
							$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
							$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
		
#add_executable(RunMotor ${CMAKE_CURRENT_SOURCE_DIR}/src/runMotor.cpp)

#target_link_libraries(RunMotor MotorDriver JetsonGPIO::JetsonGPIO)
        

