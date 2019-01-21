cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  self_driving_car_msgs
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  tf
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(ignition-msgs1 REQUIRED)
find_package(ignition-transport4 REQUIRED)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   self_driving_car_msgs
 )



catkin_package(
   INCLUDE_DIRS include
   LIBRARIES perception #  layers
   CATKIN_DEPENDS self_driving_car_msgs roscpp rospy laser_geometry
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)


add_executable(perception_node src/perception_node.cpp
src/sensor_perception.cpp
src/grid_cell_costmap.cpp
src/cloud_pixel_fusion.cpp
)

add_dependencies(perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries( perception_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   ${IGNITION-TRANSPORT_LIBRARIES}
 )

 





