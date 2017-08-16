cmake_minimum_required(VERSION 2.8.3)
project(sawyer_sim_controllers)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  effort_controllers
  roscpp
  roslint
  intera_core_msgs
  realtime_tools
  sawyer_hardware_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sawyer_sim_controllers
  CATKIN_DEPENDS sawyer_hardware_interface roscpp
)
roslint_cpp()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/sawyer_position_controller.cpp
  src/sawyer_velocity_controller.cpp
  src/sawyer_effort_controller.cpp
  src/sawyer_gravity_controller.cpp
  ## Copies of upstream ros_controllers/effort_controllers:
  #  (remove once inheritable change is upstreamed)
  src/sawyer_joint_position_controller.cpp
  src/sawyer_joint_velocity_controller.cpp
  src/sawyer_joint_effort_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
