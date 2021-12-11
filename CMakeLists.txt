cmake_minimum_required(VERSION 3.13)
project(rs_orientation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  DroneSensors.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
  LIBRARIES rs_orientation
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(pointcloud_access
  src/PointCloud2Access.cpp
)

add_library(navigation src/rsOrientation.cpp)
add_dependencies(navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/rs_orientation_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  pointcloud_access
  ${catkin_LIBRARIES}
)

add_executable(testing src/test.cpp)

target_link_libraries(testing
  ${catkin_LIBRARIES}
  navigation
  yaml-cpp
)

install(TARGETS ${PROJECT_NAME}_node testing
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
