cmake_minimum_required(VERSION 3.0.2)
project(voice_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/tag_voice_function.py
  scripts/tag_audio_once.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
