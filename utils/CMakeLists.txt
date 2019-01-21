cmake_minimum_required(VERSION 2.8.12)
project(self_driving_car_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  Point.msg
  PointArray.msg
  CamInfo.msg
  BoundingBox.msg
  BoundingBoxArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs 
)
