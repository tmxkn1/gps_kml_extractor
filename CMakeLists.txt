cmake_minimum_required(VERSION 2.8.3)
project(gps_kml_extractor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kml_extractor_node src/kml_extractor_node.cpp)
target_link_libraries(kml_extractor_node ${catkin_LIBRARIES})

add_executable(odom_extractor_node src/odom_extractor_node.cpp)
target_link_libraries(odom_extractor_node ${catkin_LIBRARIES})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY src/include
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/include
)
