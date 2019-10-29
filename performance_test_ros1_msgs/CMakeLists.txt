cmake_minimum_required(VERSION 2.8.3)
project(performance_test_ros1_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_message_files(
  DIRECTORY msg
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime)
