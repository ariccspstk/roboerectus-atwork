cmake_minimum_required(VERSION 2.8.3)
project(aricc_topic_tools)

find_package(catkin REQUIRED COMPONENTS topic_tools message_generation roscpp rostest std_msgs std_srvs
  nodelet tf
  geometry_msgs
  eigen_conversions
  topic_tools)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS topic_tools message_runtime nodelet std_msgs std_srvs
    INCLUDE_DIRS include
    LIBRARIES aricc_topic_tools
)
include_directories(include ${catkin_INCLUDE_DIRS})
#include(${PROJECT_SOURCE_DIR}/cmake/nodelet.cmake)

add_library(aricc_topic_tools SHARED
  ${aricc_topic_tools_nodelet_sources}
  src/connection_based_nodelet.cpp)
add_dependencies(aricc_topic_tools ${PROJECT_NAME}_gencpp)
target_link_libraries(aricc_topic_tools ${catkin_LIBRARIES})


install(TARGETS aricc_topic_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES aricc_topic_tools_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY scripts launch test cmake DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
  )
