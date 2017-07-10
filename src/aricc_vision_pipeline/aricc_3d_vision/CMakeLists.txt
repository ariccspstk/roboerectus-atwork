cmake_minimum_required(VERSION 2.8.3)
project(aricc_3d_vision)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure pr2_msgs pcl_ros cv_bridge roscpp rospy std_msgs image_transport message_generation tf_conversions tf tf2_ros eigen_conversions nodelet cmake_modules aricc_topic_tools)

find_package(OpenCV REQUIRED core imgproc)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/Segmentation.cfg
    cfg/BorderEstimator.cfg
    cfg/PassThroughFilter.cfg
)

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
  DEPENDS pcl
  CATKIN_DEPENDS 
  pcl_ros 
  roscpp 
  rospy 
  std_msgs
  sensor_msgs 
  geometry_msgs 
  dynamic_reconfigure 
  message_runtime
  tf_conversions
  tf
  tf2_ros
  nodelet
  aricc_topic_tools
  INCLUDE_DIRS include
  LIBRARIES aricc_3d_vision
)
###########
## Build ##
###########
add_library(${PROJECT_NAME} SHARED 
  src/PCLConversionUtil/pcl_conversion_util.cpp
  src/BorderEstimator/border_estimator.cpp
  src/PassThroughFilter/pass_through_filter.cpp
  src/ShowPCLIndices/show_pcl_indices.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

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
