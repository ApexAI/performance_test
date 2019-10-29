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
option(PERFORMANCE_TEST_FASTRTPS_ENABLED "Enable FastRTPS" ON)

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  find_package(fastrtps_cmake_module QUIET REQUIRED)
  find_package(FastRTPS QUIET REQUIRED)
  add_definitions(-DPERFORMANCE_TEST_FASTRTPS_ENABLED)
endif()

# Use ODB support to create SQL database
option(PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED "Enable ODB for SQL" OFF)

if(PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)
  add_definitions(-DPERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)
  option(PERFORMANCE_TEST_ODB_SQLITE "Select DB type" ON)
  option(PERFORMANCE_TEST_ODB_MYSQL "Select DB type" OFF)
  option(PERFORMANCE_TEST_ODB_PGSQL "Select DB type" OFF)
  if(PERFORMANCE_TEST_ODB_SQLITE)
    add_definitions(-DPERFORMANCE_TEST_ODB_SQLITE)
    set(DB_TYPE sqlite)
  endif()
  if(PERFORMANCE_TEST_ODB_MYSQL)
    add_definitions(-DPERFORMANCE_TEST_ODB_MYSQL)
    set(DB_TYPE mysql)
  endif()
  if(PERFORMANCE_TEST_ODB_PGSQL)
    add_definitions(-DPERFORMANCE_TEST_ODB_PGSQL)
    set(DB_TYPE pgsql)
  endif()
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

# Dashing version looks like 0.7.X.
if(${rclcpp_VERSION_MINOR} STRGREATER "7")
  message(STATUS "Ros Eloquent detected")
  add_definitions(-DROS_ELOQUENT)
else()
  message(STATUS "Ros Dashing detected")
  add_definitions(-DUSE_LEGACY_QOS_API)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  add_subdirectory(src/idlgen/fast_rtps)
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

# CycloneDDS
option(PERFORMANCE_TEST_FORCE_CYCLONEDDS "Require Cyclone DDS to be available, fail otherwise" OFF)

if(PERFORMANCE_TEST_FORCE_CYCLONEDDS)
  find_package(CycloneDDS REQUIRED)
else()
  find_package(CycloneDDS QUIET)
endif()
if(CycloneDDS_FOUND)
  set(PERFORMANCE_TEST_CYCLONEDDS_ENABLED TRUE)
  add_definitions(-DPERFORMANCE_TEST_CYCLONEDDS_ENABLED)
  add_subdirectory(src/idlgen/cyclonedds)
  include_directories(${CYCLONEDDS_IDL_INCLUDE_DIR})
endif()

if(PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")
  find_package(ODB REQUIRED COMPONENTS sqlite OPTIONAL_COMPONENTS mysql pgsql)
  include(${ODB_USE_FILE})

  # Get the setup.sh to update the LD_LIBRARY_PATH
  get_filename_component(ODB_LIBRARY_DIRECTORY ${ODB_LIBODB_LIBRARIES} DIRECTORY)
  ament_environment_hooks(env_hook/odb_library_path.sh.in)
endif()

set(${PROJECT_NAME}_SOURCES
    src/experiment_execution/analyze_runner.cpp)

set(${PROJECT_NAME}_HEADERS
    src/experiment_configuration/experiment_configuration.hpp
    src/experiment_execution/analysis_result.hpp)

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

if(PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)
  odb_compile(${PROJECT_NAME}_SOURCES FILES ${${PROJECT_NAME}_HEADERS} DB "${DB_TYPE}"
      GENERATE_QUERY GENERATE_SESSION GENERATE_SCHEMA SCHEMA_FORMAT embedded INCLUDE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/experiment_configuration ${CMAKE_CURRENT_SOURCE_DIR}/src/experiment_execution
      STANDARD c++11 PROFILE boost/uuid X_CPP_OPTION -DPERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)
endif()

set(sources
    src/main.cpp
    src/communication_abstractions/communicator.hpp
    src/communication_abstractions/communicator.cpp
    src/communication_abstractions/resource_manager.cpp
    src/communication_abstractions/resource_manager.hpp
    src/communication_abstractions/ros2_communicator.hpp
    src/communication_abstractions/ros2_callback_communicator.hpp
    src/experiment_configuration/topics.hpp
    src/data_running/data_runner.hpp
    src/data_running/data_runner_base.hpp
    src/data_running/data_runner_factory.cpp
    src/data_running/data_runner_factory.hpp
    src/experiment_execution/analyze_runner.hpp
    src/experiment_execution/analysis_result.cpp
    src/experiment_configuration/communication_mean.hpp
    src/experiment_configuration/qos_abstraction.cpp
    src/experiment_configuration/qos_abstraction.hpp
    src/experiment_configuration/experiment_configuration.cpp
    src/utilities/rt_enabler.hpp
    src/utilities/spin_lock.hpp
    src/utilities/statistics_tracker.hpp
    src/utilities/cpu_usage_tracker.hpp
)

# ROS2 Waitset
option(PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED "Option to enable Polling Subscription Plugin.
  The plugin can only work if ApexOS is present, otherwise it will fail." OFF)

if(PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED)
    add_definitions(-DPERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED)
    set(sources ${sources}
      src/communication_abstractions/ros2_waitset_communicator.hpp
    )
endif()

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
    set(sources ${sources}
        src/communication_abstractions/fast_rtps_communicator.hpp
    )
endif()

if(PERFORMANCE_TEST_CYCLONEDDS_ENABLED)
    set(sources ${sources}
        src/communication_abstractions/cyclonedds_communicator.hpp
    )
endif()

if(PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/experiment_configuration
        ${CMAKE_CURRENT_SOURCE_DIR}/src/experiment_execution
        ${ODB_INCLUDE_DIRS})
endif()

set(EXE_NAME perf_test)

add_executable(${EXE_NAME} ${${PROJECT_NAME}_SOURCES} ${sources} ${${PROJECT_NAME}_HEADERS})

set_compile_options(${EXE_NAME})

# Try to bake the githash into the perf_test EXE:
execute_process(
  COMMAND ./version_check.bash
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE PERF_TEST_VERSION
)
add_definitions(-DPERFORMANCE_TEST_VERSION="${PERF_TEST_VERSION}")

rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(${EXE_NAME}
    "rclcpp" ${OPTIONAL_AMENT_DEPENDENCES})

target_link_libraries(${EXE_NAME}
    ${Boost_LIBRARIES}
    ${OPTIONAL_LIBRARIES})

if(PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)
  target_link_libraries(${EXE_NAME} ${ODB_LIBRARIES})
  target_include_directories(${EXE_NAME}
      PRIVATE
        ${ODB_INCLUDE_DIRS}
        ${ODB_COMPILE_OUTPUT_DIR})
  if(PERFORMANCE_TEST_ODB_SQLITE)
  target_compile_definitions(${EXE_NAME}
      PRIVATE
        DATABASE_SQLITE)
  endif()
  if(PERFORMANCE_TEST_ODB_MYSQL)
  target_compile_definitions(${EXE_NAME}
      PRIVATE
        DATABASE_MYSQL)
  endif()
  if(PERFORMANCE_TEST_ODB_PGSQL)
    target_compile_definitions(${EXE_NAME}
      PRIVATE
        DATABASE_PGSQL)
  endif()
endif()

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  target_link_libraries(${EXE_NAME}
      ${FastRTPS_LIBRARIES}
      fast_rtps_idl
      fastcdr)
endif()

if(PERFORMANCE_TEST_CYCLONEDDS_ENABLED)
  target_link_libraries(${EXE_NAME}
      cyclonedds_idl
      CycloneDDS::ddsc)
endif()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_copyright(${${PROJECT_NAME}_SOURCES} ${sources} ${${PROJECT_NAME}_HEADERS})

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
    DESTINATION lib/${PROJECT_NAME})
ament_package()
