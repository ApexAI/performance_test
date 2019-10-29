cmake_minimum_required(VERSION 2.8.3)
project(performance_test_ros1_publisher)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp performance_test_ros1_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
