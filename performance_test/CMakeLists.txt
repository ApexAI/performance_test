# Copyright 2017 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

# Workaround to get FastRTPS and ROS 2 running together.
# This woraround is not necessary anymore for ROS 2 bouncy
# and therefore both are enabled now at the same time.
add_definitions(-DPERFORMANCE_TEST_USE_FASTRTPS)
add_definitions(-DPERFORMANCE_TEST_USE_ROS2)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
include(compile_options)
project(performance_test)

SET(OPTIONAL_AMENT_DEPENDENCES)
SET(OPTIONAL_LIBRARIES)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
find_package(fastrtps_cmake_module REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(FastRTPS REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rcutils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(osrf_testing_tools_cpp)
if(${osrf_testing_tools_cpp_FOUND})
    LIST(APPEND OPTIONAL_AMENT_DEPENDENCES "osrf_testing_tools_cpp")
    LIST(APPEND OPTIONAL_LIBRARIES osrf_testing_tools_cpp::memory_tools)
    add_definitions(-DMEMORY_TOOLS_ENABLED)
endif()

ament_export_include_directories(include)

add_subdirectory(src/idlgen/fast_rtps)
# This is a workaround for broken include paths on some systems.
include_directories(${FastRTPS_INCLUDE_DIR}/fastrtps/include ${fastcdr_INCLUDE_DIR})
include_directories(include ${FAST_RTPS_IDL_INCLUDE_DIR} ${osrf_testing_tools_cpp_INCLUDE_DIR})

#find_package(micro_dds_cmake_module)
#if(${micro_dds_cmake_module_FOUND_AMENT_PACKAGE})
#    find_package(MicroDDS MODULE)
#    if(MicroDDS_FOUND)
#        set(CONNEXT_DDS_MICRO_ENABLED TRUE)
#
#        add_definitions(-DCONNEXT_DDS_MICRO_ENABLED)
#        add_subdirectory(src/idlgen/micro)
#
#        include_directories(${RTI_MICRO_IDL_INCLUDE_DIR})
#        ament_export_definitions(${MicroDDS_DEFINITIONS})
#
#        LIST(APPEND OPTIONAL_AMENT_DEPENDENCES "MicroDDS")
#        LIST(APPEND OPTIONAL_LIBRARIES rti_micro_idl)
#    endif()
#endif()

rosidl_generate_interfaces(${PROJECT_NAME}
    "src/msg/Array1k.msg"
    "src/msg/Array4k.msg"
    "src/msg/Array16k.msg"
    "src/msg/Array32k.msg"
    "src/msg/Array60k.msg"
    "src/msg/Array1m.msg"
    "src/msg/Array2m.msg"
    "src/msg/Array4m.msg"

    "src/msg/Struct16.msg"
    "src/msg/Struct256.msg"
    "src/msg/Struct4k.msg"
    "src/msg/Struct32k.msg"

    "src/msg/PointCloud1m.msg"
    "src/msg/PointCloud2m.msg"
    "src/msg/PointCloud4m.msg"
    "src/msg/PointCloud8m.msg"
    "src/msg/PointCloud512k.msg"

    "src/msg/NavSatStatus.msg"
    "src/msg/NavSatFix.msg"
    "src/msg/RadarDetection.msg"
    "src/msg/RadarTrack.msg"
    "src/msg/Range.msg"

    "src/msg/Point.msg"
    "src/msg/Point32.msg"
    "src/msg/Polygon.msg"
    "src/msg/Vector3.msg"


    DEPENDENCIES builtin_interfaces std_msgs sensor_msgs)

set(EXE_NAME perf_test)
add_executable(${EXE_NAME}
    src/main.cpp
    src/communication_abstractions/communicator.hpp
    src/communication_abstractions/communicator.cpp
    src/communication_abstractions/ros2_callback_communicator.hpp
    src/experiment_configuration/topics.hpp
    src/data_running/data_runner_base.hpp
    src/data_running/data_runner.hpp
    src/communication_abstractions/fast_rtps_communicator.hpp
    src/communication_abstractions/resource_manager.cpp
    src/communication_abstractions/resource_manager.hpp
    src/experiment_execution/analyze_runner.cpp
    src/experiment_execution/analyze_runner.hpp
    src/experiment_execution/analysis_result.cpp
    src/experiment_execution/analysis_result.hpp
    src/utilities/rt_enabler.hpp
    src/utilities/statistics_tracker.hpp
    src/experiment_configuration/communication_mean.hpp
    src/experiment_configuration/qos_abstraction.cpp
    src/experiment_configuration/qos_abstraction.hpp
    src/experiment_configuration/experiment_configuration.cpp
    src/experiment_configuration/experiment_configuration.hpp
    src/data_running/data_runner_factory.cpp
    src/data_running/data_runner_factory.hpp
    src/utilities/spin_lock.hpp)
set_compile_options(${EXE_NAME})

rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")

    ament_target_dependencies(${EXE_NAME}
        "rclcpp" ${OPTIONAL_AMENT_DEPENDENCES})

    target_link_libraries(${EXE_NAME}
        ${Boost_LIBRARIES}
        ${FastRTPS_LIBRARIES}
        fast_rtps_idl
        fastcdr
        ${OPTIONAL_LIBRARIES})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    set(APEX_PERFORMANCE_TEST_GTEST apex_performance_test_gtest)

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${APEX_PERFORMANCE_TEST_GTEST}
        test/src/test_performance_test.cpp
        test/src/test_statistics_tracker.hpp)

    target_include_directories(${APEX_PERFORMANCE_TEST_GTEST} PRIVATE "test/include")
    target_link_libraries(${APEX_PERFORMANCE_TEST_GTEST})

    set_compile_options(${APEX_PERFORMANCE_TEST_GTEST})
endif()



install(TARGETS
    ${EXE_NAME}
    DESTINATION lib/${PROJECT_NAME})

ament_package()
