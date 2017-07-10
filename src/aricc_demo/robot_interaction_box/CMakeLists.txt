cmake_minimum_required(VERSION 2.8.3)
project(robot_interaction_box)

find_package(catkin REQUIRED 
  sensor_msgs geometry_msgs cv_bridge std_msgs roscpp OpenCV )
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system signals)

catkin_package(
  CATKIN_DEPENDS 
  std_msgs sensor_msgs geometry_msgs
  DEPENDS OpenCV
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${cakin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}
        src/main.cpp)
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES})


