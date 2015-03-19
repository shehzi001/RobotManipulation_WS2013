cmake_minimum_required(VERSION 2.8.3)
project(kdl_parser_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kdl_parser_test
#  CATKIN_DEPENDS kdl_parser roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(kdl_parser_test
#   src/${PROJECT_NAME}/kdl_parser_test.cpp
# )

## Declare a cpp executable
add_executable(kdl_parser_test src/kdl_parser_test.cpp)

target_link_libraries(kdl_parser_test 
   /opt/ros/hydro/lib/liborocos-kdl.so.1.2
   ${catkin_LIBRARIES}
   
)


add_executable(cube_stacking_test src/cube_stacking_test.cpp)

target_link_libraries(cube_stacking_test 
   /opt/ros/hydro/lib/liborocos-kdl.so.1.2
   ${catkin_LIBRARIES}
   
)





