cmake_minimum_required(VERSION 3.10)
project(plane_navigation)

file(GLOB catkin_LIBRARIES
     "/opt/ros/melodic/lib/*.so"
)

find_package(Threads)

include_directories(
  include/${PROJECT_NAME}
  # ${catkin_INCLUDE_DIRS}
  /opt/ros/melodic/include
  ../RangefinderManager/src
)

add_library(navigation src/plane_navigation.cpp src/Segment.cpp src/NavigationStructs.cpp src/PlaneNavigationManager.cpp)
target_link_libraries (navigation yaml-cpp Threads::Threads)

add_executable(grabber src/small_scans_grabber.cpp)
target_link_libraries( grabber
  ${catkin_LIBRARIES}
  rangefinder_manager_lib
  navigation
  ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(yaw src/test.cpp)
target_link_libraries(yaw
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(roomMeter src/roomMeter.cpp)
target_link_libraries(roomMeter rangefinder_manager_lib yaml-cpp ${CMAKE_THREAD_LIBS_INIT})