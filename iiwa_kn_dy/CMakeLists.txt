cmake_minimum_required(VERSION 2.8.3)
project(iiwa_kn_dy)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  iiwa14Kine
  rosbag
  roscpp
  rospy
  trajectory_msgs
  geometry_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED) 

set(HEADER_FILES include/iiwa_kn_dy/kdl_kine_solver.h)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES iiwa_kn_dy
        CATKIN_DEPENDS iiwa14Kine geometry_msgs roscpp sensor_msgs rospy rosbag trajectory_msgs 
        DEPENDS EIGEN3 orocos_kdl
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OROCOS_KDL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
        src/kdl_kine_solver.cpp 
        ${HEADER_FILES}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_node src/joint_acc.cpp)
add_executable(${PROJECT_NAME}_node src/get_mass.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


