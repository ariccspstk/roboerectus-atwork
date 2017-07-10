cmake_minimum_required(VERSION 2.8.3)
project(aricc_lowpass_controller)
find_package(catkin REQUIRED roscpp std_msgs geometry_msgs)

catkin_package(
   CATKIN_DEPENDS
#  INCLUDE_DIRS include
#  LIBRARIES aricc_lowpass_filter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(${cakin_INCLUDE_DIRS})
## Declare a cpp executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES})

