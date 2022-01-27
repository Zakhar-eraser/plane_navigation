cmake_minimum_required(VERSION 3.10)
project(plane_navigation)

file(GLOB catkin_LIBRARIES
     "/opt/ros/noetic/lib/*.so"
)

find_package(Threads)

include_directories(
  include/${PROJECT_NAME}
  /opt/ros/noetic/include
)

add_library(navigation src/plane_navigation.cpp src/Segment.cpp src/NavigationStructs.cpp)
target_link_libraries (navigation yaml-cpp Threads::Threads)

add_executable(grabber src/small_scans_grabber.cpp)
target_link_libraries( grabber
  ${catkin_LIBRARIES}
  navigation
)

add_executable(yaw src/test.cpp)
target_link_libraries(yaw
  ${catkin_LIBRARIES}
)