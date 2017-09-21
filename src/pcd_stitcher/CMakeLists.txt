cmake_minimum_required(VERSION 2.8.3)
project(pcd_stitcher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(PCL 1.8 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcd_stitcher
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Declare a C++ executable
add_executable(pcd_stitcher_node src/pcd_stitcher_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pcd_stitcher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pcd_stitcher_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})
