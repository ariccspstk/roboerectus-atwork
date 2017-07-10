cmake_minimum_required(VERSION 2.8.3)
project(aricc_vision_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  std_msgs sensor_msgs geometry_msgs message_generation pcl_msgs)

add_message_files(
  FILES
  RotatedRect.msg
  RotatedRectArray.msg
  Contour.msg
  ContourArray.msg
  Object.msg
  ObjectArray.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs pcl_msgs
)
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs pcl_msgs
)
