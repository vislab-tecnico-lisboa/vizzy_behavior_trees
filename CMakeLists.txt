cmake_minimum_required(VERSION 2.8.3)
project(vizzy_behavior_trees)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  behaviortree_cpp_v3
  geometry_msgs
  roscpp
  rospy
  std_msgs
  vizzy_msgs
  woz_dialog_msgs
  actionlib
  move_base_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vizzy_actions
  CATKIN_DEPENDS actionlib_msgs behaviortree_cpp_v3 geometry_msgs roscpp rospy std_msgs vizzy_msgs woz_dialog_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(vizzy_actions STATIC
  src/actions/speech/speech_actions.cpp
  src/actions/navigation/move_base_actions.cpp
  src/actions/navigation/charging_actions.cpp
  src/actions/general/general.cpp
  src/actions/gaze/gaze_actions.cpp
  src/actions/arms/arms_cartesian_actions.cpp
  src/conditions/general/general.cpp
)
add_dependencies(vizzy_actions vizzy_msgs_generate_messages_cpp woz_dialog_msgs_generate_messages_cpp)
target_link_libraries(vizzy_actions PRIVATE ${BEHAVIOR_TREE_LIBRARY} ${catkin_LIBRARIES})
set_target_properties(vizzy_actions PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(main_tree_node src/main_tree_node.cpp)
target_link_libraries(main_tree_node
  ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY} vizzy_actions
)

