cmake_minimum_required(VERSION 2.8.3)
project(laser_clustering)



set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(catkin REQUIRED COMPONENTS
  roslib
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  ClustersMsg.msg
  ClusterLabelsMsg.msg
)

generate_messages(
	DEPENDENCIES
  sensor_msgs
	std_msgs
)

catkin_package(
   	LIBRARIES ${PROJECT_NAME}
  	CATKIN_DEPENDS roslib std_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${custom_python_tools_INCLUDE_DIRS} src)

aux_source_directory(src SRC_LIST)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall" )
