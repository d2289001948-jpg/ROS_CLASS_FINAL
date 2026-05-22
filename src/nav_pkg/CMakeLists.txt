cmake_minimum_required(VERSION 3.0.2)
project(nav_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  move_base_msgs
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/navigate_to_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
