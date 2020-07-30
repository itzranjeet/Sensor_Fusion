#------------------------------------------------------------------------------
# Copyright (C) 2019, KPIT
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8.3) # KMS: 2.x is legacy. Recommend 3.4+
project(sensor_fusion)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  geometry_msgs
  tf2_ros
  rosbag

  cv_bridge
  image_transport
  OpenCV
)

add_message_files(
  DIRECTORY msg
  FILES
  SensorMessage.msg
  SensorMessageArray.msg
  RadarObjects.msg
  RadarObject.msg
)
#generate_messages(DEPENDENCIES)
generate_messages(
  DEPENDENCIES
  geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_library(csv_reader_lib
	src/message_reader_factory.cpp
	src/sensor_message_from_image_rosbag.cpp
        src/sensor_message_from_lidar_rosbag.cpp
	src/sensor_message_from_radar_rosbag.cpp
)
add_dependencies(csv_reader_lib sensor_fusion_generate_messages_cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  ${PROJECT_SOURCE_DIR}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/platform_communicator
)


add_executable(
    Node1
    src/node_base.cpp
    src/Node1.cpp
    src/node_base.cpp # KMS: Why node_base & converter not a shared library ?
)
target_link_libraries(Node1 csv_reader_lib ${catkin_LIBRARIES})
add_dependencies(Node1 sensor_fusion_generate_messages_cpp)




