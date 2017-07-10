cmake_minimum_required(VERSION 2.8.3)
project(aricc_test)
set(ROS_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs atwork_ros_msgs geometry_msgs aricc_rcwork_task aricc_utils actionlib actionlib_msgs)
include_directories( include ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aricc_bnt
  CATKIN_DEPENDS 
    roscpp
    atwork_ros_msgs
#  DEPENDS system_lib
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
