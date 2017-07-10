cmake_minimum_required(VERSION 2.8.3)
project(aricc_depth_vision)

find_package(catkin REQUIRED COMPONENTS
  mk message_generation std_msgs sensor_msgs geometry_msgs cv_bridge
  image_geometry image_transport image_view2 driver_base dynamic_reconfigure 
  cmake_modules roscpp nodelet rostest tf rospack
  aricc_topic_tools pcl_ros)

find_package(Boost REQUIRED COMPONENTS filesystem system signals)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()


include_directories(include 
  SYSTEM ${catkin_INCLUDE_DIRS} 
         ${OpenCV_INCLUDE_DIRS} 
         ${Boost_INCLUDE_DIRS}
         ${Eigen_INCLUDE_DIRS})

#generate_dynamic_reconfigure_options()
#add_service_files(FILES EuclideanSegment.srv  SetTemplate.srv  WhiteBalancePoints.srv  WhiteBalance.srv))

#generate_messages(
#  DEPENDENCIES std_msgs sensor_msgs geometry_msgs aricc_recognition_msgs
#)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/DepthToGrayImage.cfg
  )

catkin_package(
  CATKIN_DEPENDS 
  std_msgs sensor_msgs geometry_msgs message_runtime
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


# compiling jsk_perception library for nodelet
add_library(${PROJECT_NAME} SHARED
  src/DepthRegister/depth_register.cpp 
  src/DepthToGrayImage/depth_to_gray_image.cpp 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME} 
  ${${PROJECT_NAME}_nodelet_executables}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
