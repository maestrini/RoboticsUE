cmake_minimum_required(VERSION 2.8.3)
project(tug_ros_examples)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  actionlib
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoNumbers.srv
)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   Factorial.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp actionlib_msgs tf
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
)

## Declare a cpp executable
add_executable(talker_node
  src/talker.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(talker_node
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(listener_node
  src/listener.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(listener_node
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(converter_node
  src/converter.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(converter_node
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(add_two_numbers_client
  src/AddTwoNumbersClient.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(add_two_numbers_client
  tug_ros_examples_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(add_two_numbers_client
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(add_two_numbers_server
  src/AddTwoNumbersServer.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(add_two_numbers_server
  tug_ros_examples_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(add_two_numbers_server
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(factorial_action_server
  src/FactorialActionServer.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(factorial_action_server
  tug_ros_examples_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(factorial_action_server
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(factorial_action_client
  src/FactorialActionClient.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(factorial_action_client
  tug_ros_examples_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(factorial_action_client
  ${catkin_LIBRARIES}
)
