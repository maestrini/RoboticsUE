cmake_minimum_required(VERSION 2.8.3)
project(tug_tools)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

####################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(teleop_node src/keyboard.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(teleop_node
  ${catkin_LIBRARIES}
)

