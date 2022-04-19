cmake_minimum_required(VERSION 3.10)
project(plane_navigation)

file(GLOB catkin_LIBRARIES
     "/opt/ros/noetic/lib/*.so"
     "/opt/ros/melodic/lib/*.so"
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  /opt/ros/noetic/include
  ../RangefinderManager/src
)

add_library(navigation
  src/plane_navigation.cpp
  src/Segment.cpp
  src/NavigationStructs.cpp
  src/PlaneNavigationManager.cpp
  src/Filters.cpp
  src/Mapper.cpp
)
target_link_libraries (navigation yaml-cpp)

add_executable(grabber src/small_scans_grabber.cpp)
target_link_libraries( grabber
  ${catkin_LIBRARIES}
  navigation
)

add_executable(yaw src/yaw_by_lasers.cpp)
target_link_libraries(yaw
  ${catkin_LIBRARIES}
)

add_executable(roomMeter src/roomMeter.cpp)
target_link_libraries(roomMeter rangefinder_manager_lib yaml-cpp)

add_executable(nav_test src/nav_test.cpp)
target_link_libraries(nav_test navigation)