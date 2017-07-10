cmake_minimum_required(VERSION 2.8.3)
project(aricc_laser_pipeline)

find_package(catkin REQUIRED COMPONENTS
  mk message_generation std_msgs sensor_msgs geometry_msgs sensor_msgs 
  dynamic_reconfigure cmake_modules roscpp nodelet rostest tf rospack
  aricc_topic_tools pcl_ros aricc_utils aricc_std_msgs)

find_package(Boost REQUIRED COMPONENTS filesystem system signals)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# Dynamic reconfigure support
#generate_dynamic_reconfigure_options(
#  cfg/ProjectRotatedRect.cfg
#  )
add_message_files(
  FILES
  BeamDist.msg
  BeamDistArray.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs pcl_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  std_msgs sensor_msgs geometry_msgs message_runtime
  DEPENDS 
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS})


add_library(${PROJECT_NAME} SHARED 
  src/GetBeamDist/get_beam_dist.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME} 
  ${${PROJECT_NAME}_nodelet_executables}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
