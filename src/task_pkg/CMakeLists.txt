cmake_minimum_required(VERSION 3.0.2)
project(task_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  move_base_msgs
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/state_machine.py
  scripts/pid_navigate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
