cmake_minimum_required(VERSION 2.8.3)
project(gesture_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  std_msgs
  std_srvs
  sensor_msgs
)

catkin_package()


install(PROGRAMS
  scripts/gesture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
