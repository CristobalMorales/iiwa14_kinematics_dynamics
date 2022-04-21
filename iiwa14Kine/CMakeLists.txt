cmake_minimum_required(VERSION 2.8.3)
project(iiwa14Kine)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

set(HEADER_FILES include/iiwa14Kine/iiwa14Kine.h)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES iiwa14Kine
        CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2_ros rospy
        DEPENDS EIGEN3
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
        src/iiwa14Kine.cpp 
        ${HEADER_FILES}
)