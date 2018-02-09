cmake_minimum_required(VERSION 2.8.3)
project(arena_tf_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  cmake_modules
)

find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_executable(tf_publisher 
               src/tf_publisher.cpp
               src/transformations.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})