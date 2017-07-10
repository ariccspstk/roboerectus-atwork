cmake_minimum_required(VERSION 2.8.3)
project(aricc_std_msgs)

find_package(catkin REQUIRED
  std_msgs message_generation )

add_message_files(
  FILES
  Float64Array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)
catkin_package(
  CATKIN_DEPENDS std_msgs
)
