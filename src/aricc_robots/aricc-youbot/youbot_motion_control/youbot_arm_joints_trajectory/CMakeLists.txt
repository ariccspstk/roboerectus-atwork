cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_joints_trajectory)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation dynamic_reconfigure actionlib actionlib_msgs sensor_msgs trajectory_msgs pr2_msgs brics_actuator std_msgs control_msgs)
find_package(Boost REQUIRED COMPONENTS system)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

add_action_files(
  DIRECTORY action
  FILES Trajectory.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#add_executable(${PROJECT_NAME}_server src/main.cpp)
#target_link_libraries( ${PROJECT_NAME}_server ${catkin_LIBRARIES} )

add_executable( ${PROJECT_NAME}_controller src/controller.cpp )
target_link_libraries( ${PROJECT_NAME}_controller ${catkin_LIBRARIES} )

add_executable( ${PROJECT_NAME}_server src/main.cpp )
target_link_libraries( ${PROJECT_NAME}_server ${catkin_LIBRARIES} )
