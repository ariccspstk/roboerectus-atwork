cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_ik)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation geometry_msgs std_msgs cmake_modules actionlib actionlib_msgs dynamic_reconfigure aricc_utils)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)


generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

add_action_files(
  DIRECTORY action
  FILES Ik.action
)
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  std_msgs
  message_runtime
  actionlib_msgs
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)

add_executable(${PROJECT_NAME}_server src/main.cpp src/YoubotGraspIK.cpp)
add_dependencies(${PROJECT_NAME}_server ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_server
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(${PROJECT_NAME}_controller src/controller.cpp)
add_dependencies(${PROJECT_NAME}_controller ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_controller
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)
