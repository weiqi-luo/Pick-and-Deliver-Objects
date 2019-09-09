cmake_minimum_required(VERSION 2.8.3)
project(face_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  std_msgs
  sensor_msgs
  final_msg_srv
)

catkin_package()


install(PROGRAMS
  scripts/face.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
