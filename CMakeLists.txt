cmake_minimum_required(VERSION 3.1.2)
project(led_strip_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  nodelet
  mrs_msgs
  )


catkin_package(
  INCLUDE_DIRS
  LIBRARIES LedStripDriver
  CATKIN_DEPENDS roscpp mrs_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )


add_library(LedStripDriver
  src/led_strip_driver.cpp
)
