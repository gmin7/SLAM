cmake_minimum_required(VERSION 2.8.3)
project(odom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  robot_localization
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)
