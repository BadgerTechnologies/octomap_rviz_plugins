cmake_minimum_required(VERSION 2.8)
project(octomap_distance_tracker)

set(CMAKE_CXX_STANDARD 11)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(dynamicEDT3D REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

find_package(OCTOMAP REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${dynamicEDT3D_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP PCL
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/octomap_distance_tracker.cpp
)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(octomap_distance_tracker_node src/octomap_distance_tracker_node.cpp)
target_link_libraries(octomap_distance_tracker_node ${PROJECT_NAME} ${LINK_LIBS})


# install targets:
install(TARGETS ${PROJECT_NAME}
  octomap_distance_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
