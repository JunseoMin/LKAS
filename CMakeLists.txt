cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

project(olny_cv)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  xycar_msgs
  std_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES olny_cv
  CATKIN_DEPENDS roscpp sensor_msgs xycar_msgs std_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(bird_eye_view
  src/bird_eye_view.cc
)
target_include_directories(bird_eye_view PRIVATE include)

add_library(xyc_control
  src/xyc_control.cc
)
target_include_directories(xyc_control PRIVATE include)

add_library(lane_detection
  src/lane_detection.cc
)
target_include_directories(xyc_control PRIVATE include)

## Declare the main executable
add_executable(LKAS_node src/LKAS_node.cc)
target_include_directories(LKAS_node PRIVATE include)

## Specify libraries to link a library or executable target against
target_link_libraries(LKAS_node
  bird_eye_view
  xyc_control
  lane_detection
  ${catkin_LIBRARIES}
)
