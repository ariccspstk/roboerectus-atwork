cmake_minimum_required(VERSION 2.8.3)
project(aricc_2d_vision)

find_package(catkin REQUIRED COMPONENTS
  mk message_generation std_msgs sensor_msgs geometry_msgs cv_bridge
  image_geometry image_transport image_view2 driver_base dynamic_reconfigure 
  cmake_modules roscpp nodelet rostest tf rospack
  aricc_topic_tools pcl_ros aricc_utils)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system signals)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#generate_dynamic_reconfigure_options()
#add_service_files(FILES EuclideanSegment.srv  SetTemplate.srv  WhiteBalancePoints.srv  WhiteBalance.srv))

#generate_messages(
#  DEPENDENCIES std_msgs sensor_msgs geometry_msgs aricc_recognition_msgs
#)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/MorphologicalMaskImageOperator.cfg
  cfg/RectToMaskImage.cfg
  cfg/CreateThresholdImage.cfg
  cfg/EdgeFinder.cfg
  cfg/MaskImageOperator.cfg
  cfg/SmoothingImage.cfg
  cfg/ContourFinder.cfg
  cfg/RotatedRectFinder.cfg
  cfg/RectBasedObjectDetection.cfg
  cfg/ProjectRotatedRect.cfg
  cfg/CheckBoardDetector.cfg
  cfg/ColourFinder.cfg
  cfg/ShapeMatch.cfg
  cfg/RTTImage.cfg
  )

catkin_package(
  CATKIN_DEPENDS 
  std_msgs sensor_msgs geometry_msgs message_runtime
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS})

# compiling library for nodelet
add_library(${PROJECT_NAME} SHARED
  src/AsadVision/asad_vision.cpp 
  src/ApplyMaskImage/apply_mask_image.cpp
  src/UnapplyMaskImage/unapply_mask_image.cpp
  src/ImageUtils/image_utils.cpp
  src/ContourFinder/contour_finder.cpp
  src/ColourFinder/colour_finder.cpp
  src/RotatedRectFinder/rotated_rect_finder.cpp
  src/EdgeFinder/edge_finder.cpp
  src/RectToMaskImage/rect_to_mask_image.cpp
  src/RectToPointCloud/rect_to_point_cloud.cpp
  src/CreateThresholdImage/create_threshold_image.cpp
  src/SmoothingImage/smoothing_image.cpp
  src/MaskImageOperator/mask_image_operator.cpp
  src/ProjectRotatedRect/project_rotated_rect.cpp
  src/RectBasedObjectDetection/rect_based_object_detection.cpp
  src/DilateErodeImage/dilate_erode_image.cpp
  src/CheckBoardDetector/check_board_detector.cpp
  src/ShapeMatch/shape_match.cpp
  src/RTTImage/RTTImage.cpp)

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
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
