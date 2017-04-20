cmake_minimum_required(VERSION 2.8.3)
project(sawyer_gazebo)

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    gazebo_ros_control
    std_msgs
    intera_core_msgs
)

# Depend on system install of Gazebo
find_package(GAZEBO REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


catkin_package(
  CATKIN_DEPENDS
  gazebo_ros_control
  roscpp
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(sawyer_gazebo_ros_control
  src/sawyer_gazebo_ros_control_plugin.cpp
)

target_link_libraries(sawyer_gazebo_ros_control
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  gazebo_ros_control
)

add_dependencies(
  sawyer_gazebo_ros_control
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS sawyer_gazebo_ros_control
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
