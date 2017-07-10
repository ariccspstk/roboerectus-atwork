cmake_minimum_required(VERSION 2.8.3)
project(youbot_teleop)
find_package(catkin
             REQUIRED
             COMPONENTS
             roscpp
             rospy
             std_msgs
             tf
             geometry_msgs
             sensor_msgs
             pr2_msgs
             brics_actuator
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(include ${cakin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}
        src/youbot_teleop_node.cpp)
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES})
