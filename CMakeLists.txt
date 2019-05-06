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
	attitude_est.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetAttitudeEst.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES BuildMap.action
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
add_library(build_map_action src/build_map_action.cpp)
target_link_libraries(build_map_action
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${kinova_driver_LIBRARIES}
)

## Declare a C++ executable
add_executable(build_map_action_node src/build_map_action_node.cpp)
target_link_libraries(build_map_action_node
	${catkin_LIBRARIES}
	build_map_action
)

## Declare a C++ library
add_library(flower_mapper src/flower_mapper.cpp)
target_link_libraries(flower_mapper
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${kinova_driver_LIBRARIES}
)

## Declare a C++ executable
add_executable(flower_mapper_node src/flower_mapper_node.cpp)
target_link_libraries(flower_mapper_node
	${catkin_LIBRARIES}
	flower_mapper
)

## Declare a C++ executable
add_executable(test_build_map src/test_build_map.cpp)
target_link_libraries(test_build_map
	${catkin_LIBRARIES}
	build_map_action
)

#############
## Install ##
#############
install(TARGETS build_map_action DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS flower_mapper DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
