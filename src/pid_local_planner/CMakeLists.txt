cmake_minimum_required(VERSION 3.0.2)
project(pid_local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  costmap_2d
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp nav_core costmap_2d geometry_msgs nav_msgs
    tf2 tf2_ros tf2_geometry_msgs pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pid_local_planner.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
