cmake_minimum_required(VERSION 2.8.3)
project(waypoint_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_publisher
  src/waypoint_pub.cpp
)



target_link_libraries(waypoint_publisher
  ${catkin_LIBRARIES}
)



install(TARGETS waypoint_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

