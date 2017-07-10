cmake_minimum_required(VERSION 2.8.3)
project(aricc_utils)

find_package(catkin REQUIRED geometry_msgs roscpp std_msgs cmake_modules)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  std_msgs geometry_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED 
  src/geometry_utils.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

