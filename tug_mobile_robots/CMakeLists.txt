cmake_minimum_required(VERSION 2.8.3)
project(tug_mobile_robots)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CallMovement.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(mobile_robots_node
  include/${PROJECT_NAME}/mobile_robots_node.hpp
  src/mobile_robots_node.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(mobile_robots_node
  tug_stage_ros_generate_messages_cpp
  tug_mobile_robots_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(mobile_robots_node
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(mobile_robots_test
  src/mobile_robots_test.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(mobile_robots_test
  tug_mobile_robots_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(mobile_robots_test
  ${catkin_LIBRARIES}
)
