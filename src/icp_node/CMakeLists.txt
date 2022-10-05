cmake_minimum_required(VERSION 3.0.2)
project(icp_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(solver src/solver.cpp)
target_link_libraries(solver ${catkin_LIBRARIES})

add_executable(points src/points_draw.cpp)
target_link_libraries(points ${catkin_LIBRARIES})

