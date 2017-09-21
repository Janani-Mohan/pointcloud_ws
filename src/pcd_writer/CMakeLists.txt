cmake_minimum_required(VERSION 2.8.3)
project(pcd_writer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(PCL 1.8 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pcd_writer
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(pcd_writer_node src/pcd_writer_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pcd_writer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pcd_writer_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)
