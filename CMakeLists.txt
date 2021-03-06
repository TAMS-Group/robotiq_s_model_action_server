cmake_minimum_required(VERSION 3.0.2)
project(robotiq_s_model_action_server)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  robotiq_3f_gripper_control
  roscpp
  control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib actionlib_msgs robotiq_3f_gripper_control roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${robotiq_s_model_action_server_INCLUDE_DIRS}
)

# The action server
add_executable(robotiq_s_model_action_server_node
  src/robotiq_s_model_action_server_node.cpp
  src/robotiq_s_model_action_server.cpp
  include/robotiq_action_server/robotiq_s_model_action_server.h
)
add_dependencies(robotiq_s_model_action_server_node ${catkin_EXPORTED_TARGETS}
  ${robotiq_s_model_action_server_EXPORTED_TARGETS}
  ${robotiq_s_model_control_EXPORTED_TARGETS})

 target_link_libraries(robotiq_s_model_action_server_node
   ${catkin_LIBRARIES}
 )

install(TARGETS robotiq_s_model_action_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
