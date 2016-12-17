cmake_minimum_required(VERSION 2.8.3)
project(ur_rangefinder_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  )

catkin_package(
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
  )

add_executable(ur_rangefinder_driver_node
  src/rangefinder_config.cpp
  src/rangefinder_node.cpp
  src/main.cpp
  )

target_link_libraries(ur_rangefinder_driver_node
  ${catkin_LIBRARIES}
  serial
  )
