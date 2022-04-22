cmake_minimum_required(VERSION 2.8.3)
project(iiwa_kdl)

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
  kdl_parser
)

find_package(Eigen3 REQUIRED)

set(HEADER_FILES include/iiwa_kdl/kdl_kine_solver.h)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES iiwa_kdl
        CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2_ros rospy kdl_parser
        DEPENDS EIGEN3
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
        src/kdl_kine_solver.cpp 
        ${HEADER_FILES}
)