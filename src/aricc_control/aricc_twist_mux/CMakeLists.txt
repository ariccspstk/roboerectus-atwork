cmake_minimum_required(VERSION 2.8.3)
project(aricc_twist_mux)
find_package(catkin REQUIRED roscpp geometry_msgs)
set(ROS_BUILD_TYPE Release)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aricc_twist_mux
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
