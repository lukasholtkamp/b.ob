cmake_minimum_required(VERSION 3.5)
project(bob_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(tf2 REQUIRED)

include_directories(
  include
)

set(dependencies 
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  Ceres
  tf2
)

# add_executable(mpc_controller src/mpc_controller.cpp)
# target_include_directories(mpc_controller PUBLIC ${Eigen3_INCLUDE_DIRS})
# ament_target_dependencies(mpc_controller ${dependencies})
# target_link_libraries(mpc_controller ${CERES_LIBRARIES})

add_executable(mpc_obstacle_avoid src/mpc_obstacle_avoid.cpp)
target_include_directories(mpc_obstacle_avoid PUBLIC ${Eigen3_INCLUDE_DIRS})
ament_target_dependencies(mpc_obstacle_avoid ${dependencies})
target_link_libraries(mpc_obstacle_avoid ${CERES_LIBRARIES})

install(TARGETS mpc_obstacle_avoid
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()