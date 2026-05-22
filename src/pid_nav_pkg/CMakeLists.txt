cmake_minimum_required(VERSION 3.0.2)
project(pid_nav_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pid_navigator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
