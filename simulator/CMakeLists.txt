cmake_minimum_required(VERSION 2.8.3)
project(simulator)

set (PRIUSCUP_MAJOR_VERSION 0)
set (PRIUSCUP_MINOR_VERSION 1)
set (PRIUSCUP_PATCH_VERSION 0)
set (PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_plugins
  geometry_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
find_package(ignition-msgs0 REQUIRED)

catkin_package(
 LIBRARIES gazebo_ros_velodyne_laser 
 CATKIN_DEPENDS roscpp gazebo_ros 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} )
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(BloomVisualPlugin SHARED plugins/BloomVisualPlugin.cc)
target_link_libraries(BloomVisualPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})


 
add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
add_dependencies(PriusHybridPlugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

 
add_library(gazebo_ros_velodyne_laser plugins/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin)



install(TARGETS PriusHybridPlugin gazebo_ros_velodyne_laser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
