cmake_minimum_required(VERSION 2.8.3)
project(STUDY1)

set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++11 -DNDEBUG")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  ackermann_msgs
  geometry_msgs
  hellocm_msgs
  autoware_msgs
  tf
  tf2
  eigen_conversions
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs autoware_msgs
)

find_path(
  GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR} )

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)
add_definitions(${GeographicLib_DEFINITIONS})

add_executable(UDP_to_carmaker src/UDP_to_carmaker.cpp)
target_link_libraries(
  UDP_to_carmaker
        ${catkin_LIBRARIES}
  )

add_executable(joy_control src/joy_control.cpp)
target_link_libraries(
  joy_control
        ${catkin_LIBRARIES}
  )

add_executable(UTM_waypoint src/UTM_waypoint.cpp)
target_link_libraries(
  UTM_waypoint
        ${catkin_LIBRARIES}
        ${GeographicLib_LIBRARIES}
  )

add_executable(EKF src/EKF.cpp)
target_link_libraries(
  EKF
        ${catkin_LIBRARIES}
        ${GeographicLib_LIBRARIES}
  )

add_executable(LatLong2UTM src/LatLong2UTM.cpp)
target_link_libraries(
  LatLong2UTM
        ${catkin_LIBRARIES}
        ${GeographicLib_LIBRARIES}
  )

add_executable(waypoint_saver src/waypoint_saver.cpp)
target_link_libraries(
  waypoint_saver
        ${catkin_LIBRARIES}
        ${GeographicLib_LIBRARIES}
  )

add_executable(waypoint_loader src/waypoint_loader.cpp)
target_link_libraries(
  waypoint_loader
        ${catkin_LIBRARIES}
        ${GeographicLib_LIBRARIES}
  )

add_executable(waypoint_tracker src/waypoint_tracker.cpp)
target_link_libraries(
  waypoint_tracker
        ${catkin_LIBRARIES}
        ${GeographicLib_LIBRARIES}
  )
  