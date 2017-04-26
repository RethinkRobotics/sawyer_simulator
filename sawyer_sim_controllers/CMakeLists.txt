cmake_minimum_required(VERSION 2.8.3)
project(sawyer_sim_controllers)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  effort_controllers
  roscpp
  intera_core_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sawyer_sim_controllers
  CATKIN_DEPENDS effort_controllers roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/sawyer_position_controller.cpp
   src/joint_group_position_controller.cpp
 )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

