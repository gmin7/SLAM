cmake_minimum_required(VERSION 2.8.3)
project(lane_mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()
catkin_install_python(PROGRAMS bin/lane_detection DESTINATION ${PROJECT_NAME}/bin)
