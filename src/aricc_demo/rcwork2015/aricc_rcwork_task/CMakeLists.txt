cmake_minimum_required(VERSION 2.8.3)
project(aricc_rcwork_task)

find_package(catkin REQUIRED roscpp geometry_msgs)
include_directories( include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES aricc_rcwork_task
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
