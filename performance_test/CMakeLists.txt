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

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
include(compile_options)
project(performance_test)

set(OPTIONAL_AMENT_DEPENDENCES)
set(OPTIONAL_LIBRARIES)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(ament_cmake REQUIRED)

# FastRTPS
option(PERFORMANCE_TEST_FASTRTPS_ENABLED "Enable FastRTPS" OFF)
option(PERFORMANCE_TEST_OPENDDS_ENABLED "Enable OpenDDS" ON)

if(PERFORMANCE_TEST_OPENDDS_ENABLED)
    add_definitions(-DPERFORMANCE_TEST_OPENDDS_ENABLED)
endif()

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
    find_package(fastrtps_cmake_module QUIET REQUIRED)
    find_package(FastRTPS QUIET REQUIRED)
    add_definitions(-DPERFORMANCE_TEST_FASTRTPS_ENABLED)
endif()


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
    list(APPEND OPTIONAL_AMENT_DEPENDENCES "osrf_testing_tools_cpp")
    list(APPEND OPTIONAL_LIBRARIES osrf_testing_tools_cpp::memory_tools)
    add_definitions(-DPERFORMANCE_TEST_MEMORYTOOLS_ENABLED)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  add_subdirectory(src/idlgen/fast_rtps)
endif()

if(PERFORMANCE_TEST_OPENDDS_ENABLED)
    add_subdirectory(src/idlgen/opendds)
    message("PARENT ACE_ROOT = " ${PARENT_ACE_ROOT})
    include_directories(${OPENDDS_IDL_INCLUDE_DIR} ${PARENT_DDS_ROOT} ${PARENT_ACE_ROOT} ${PARENT_MPC_ROOT} ${PARENT_TAO_ROOT})
endif()

# This is a workaround for broken include paths on some systems.
include_directories(${FastRTPS_INCLUDE_DIR} ${FastRTPS_INCLUDE_DIR}/fastrtps/include ${fastcdr_INCLUDE_DIR})
include_directories(include ${FAST_RTPS_IDL_INCLUDE_DIR} ${osrf_testing_tools_cpp_INCLUDE_DIR})

find_package(micro_dds_cmake_module QUIET)
if(${micro_dds_cmake_module_FOUND_AMENT_PACKAGE})
    find_package(MicroDDS MODULE)
    if(MicroDDS_FOUND)
        set(PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED TRUE)

        add_definitions(-DPERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED)
        add_subdirectory(src/idlgen/micro)

        include_directories(${RTI_MICRO_IDL_INCLUDE_DIR})
        ament_export_definitions(${MicroDDS_DEFINITIONS})
    endif()
endif()

#find_package(opendds_cmake_module QUIET REQUIRED)
#if(${opendds_cmake_module_FOUND_AMENT_PACKAGE})
#    find_package(OpenDDS MODULE REQUIRED)
#    if(OpenDDS_FOUND)
#        set(PERFORMANCE_TEST_OPENDDS_ENABLED)
#
#        add_definitions(-DPERFORMANCE_TEST_OPENDDS_ENABLED)
#        add_subdirectory(src/idlgen/opendds)
#
#        #include_directories(${RTI_MICRO_IDL_INCLUDE_DIR})
#        #ament_export_definitions(${MicroDDS_DEFINITIONS})
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

set(sources
    src/main.cpp
    src/communication_abstractions/communicator.hpp
    src/communication_abstractions/communicator.cpp
    src/communication_abstractions/resource_manager.cpp
    src/communication_abstractions/resource_manager.hpp
    src/communication_abstractions/ros2_callback_communicator.hpp
    src/experiment_configuration/topics.hpp
    src/data_running/data_runner.hpp
    src/data_running/data_runner_base.hpp
    src/data_running/data_runner_factory.cpp
    src/data_running/data_runner_factory.hpp
    src/experiment_execution/analyze_runner.cpp
    src/experiment_execution/analyze_runner.hpp
    src/experiment_execution/analysis_result.cpp
    src/experiment_execution/analysis_result.hpp
    src/experiment_configuration/communication_mean.hpp
    src/experiment_configuration/qos_abstraction.cpp
    src/experiment_configuration/qos_abstraction.hpp
    src/experiment_configuration/experiment_configuration.cpp
    src/experiment_configuration/experiment_configuration.hpp
    src/utilities/rt_enabler.hpp
    src/utilities/spin_lock.hpp
    src/utilities/statistics_tracker.hpp
)

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
    set(sources ${sources}
        src/communication_abstractions/fast_rtps_communicator.hpp
    )
endif()

if(PERFORMANCE_TEST_OPENDDS_ENABLED)
    set(sources ${sources}
            src/communication_abstractions/opendds_communicator.hpp
            )
endif()

set(EXE_NAME perf_test)
add_executable(${EXE_NAME} ${sources})
set_compile_options(${EXE_NAME})

rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(${EXE_NAME}
    "rclcpp" ${OPTIONAL_AMENT_DEPENDENCES})

target_link_libraries(${EXE_NAME}
    ${Boost_LIBRARIES}
    ${OPTIONAL_LIBRARIES})

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  target_link_libraries(${EXE_NAME}
      ${FastRTPS_LIBRARIES}
      fast_rtps_idl
      fastcdr)
endif()

if (PERFORMANCE_TEST_OPENDDS_ENABLED)
    set(DDS_DYN_LIBS ${PARENT_ACE_ROOT}/lib/libACE.so 
		     ${PARENT_ACE_ROOT}/lib/libTAO.so
                     ${PARENT_DDS_ROOT}/dds/libOpenDDS_Dcps.so
                     ${PARENT_DDS_ROOT}/dds/DCPS/RTPS/libOpenDDS_Rtps.so
                     ${PARENT_DDS_ROOT}/dds/DCPS/transport/rtps_udp/libOpenDDS_Rtps_Udp.so
                     ${PARENT_ACE_ROOT}/lib/libTAO_PortableServer.so
                     ${PARENT_ACE_ROOT}/lib/libTAO_PortableServer.so
                     ${PARENT_ACE_ROOT}/lib/libTAO_PI.so
                     ${PARENT_ACE_ROOT}/lib/libTAO_CodecFactory.so
                     ${PARENT_ACE_ROOT}/lib/libTAO_AnyTypeCode.so
                     ${PARENT_ACE_ROOT}/lib/libTAO_BiDirGIOP.so
                     )

    target_link_libraries(${EXE_NAME} opendds_idl ${DDS_DYN_LIBS})

endif()

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

install(PROGRAMS
    helper_scripts/performance_test_file_reader.py
    DESTINATION lib/${PROJECT_NAME})
ament_package()
