cmake_minimum_required(VERSION 2.8.3)
project(pc_attitude_estimation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  cmake_modules
  geometric_shapes
  kinova_driver
  message_generation
  message_runtime
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  roslib
  rospy
  std_msgs
  tf2
  manipulation_control
  parameter_utils
)

find_package(Boost REQUIRED system filesystem date_time thread serialization regex timer)
find_package(Eigen3 REQUIRED)
find_package(kinova_driver REQUIRED)

add_message_files(
	FILES
	AttitudeEst.msg
	AttitudeEstVec.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetAttitudeEst.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES GetAttitude.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
	CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs actionlib_msgs moveit_core moveit_ros_planning_interface
  parameter_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${kinova_driver_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

link_directories(
  ${catkin_LIBRARY_DIRS}
)
## Declare a C++ library
add_library(attitude_estimation src/attitude_estimation.cpp)
target_link_libraries(attitude_estimation
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${kinova_driver_LIBRARIES}
)

## Declare a C++ executable
add_executable(attitude_estimation src/attitude_estimation_node.cpp)
target_link_libraries(attitude_estimation_node
	${catkin_LIBRARIES}
	attitude_estimation
)

#############
## Install ##
#############
install(TARGETS attitude_estimation DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
