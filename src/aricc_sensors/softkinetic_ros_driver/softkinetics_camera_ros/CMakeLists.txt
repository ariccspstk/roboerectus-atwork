cmake_minimum_required(VERSION 2.8.3)
project(softkinetics_camera_ros)

find_package(catkin REQUIRED COMPONENTS cmake_modules tf pcl_ros cv_bridge roscpp rospy std_msgs nodelet message_generation image_transport )

# find the DepthSense SDK
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(DepthSenseSDK)

if(NOT DepthSenseSDK_FOUND)
  message(FATAL_ERROR "+++++++++ Could not locate the DepthSense SDK +++++++++
Please install the SDK before trying to build the 'softkinetic_camera' package, see README.txt.

Cannot continue, aborting.")
  return()
endif()


#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
  pcl_ros roscpp rospy std_msgs message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DepthSenseSDK_INCLUDE_DIRS}
)

set(SRC
    src/softkinetics_camera_node.cpp
    src/softkinetics_camera_driver.cpp
)
add_executable(
  ${PROJECT_NAME}
  ${SRC}
)

target_link_libraries(${PROJECT_NAME}  ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
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


