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

set(SOURCE_EXTENSION cxx)

if(NOT DEFINED RTI_SRC_INSTALL_DIR)
    message(ERROR " :Set the path to source dir of installed RTI micro")
    return()
endif()

include_directories(${RTI_SRC_INSTALL_DIR}/source/unix/include)
link_directories(${RTI_SRC_INSTALL_DIR}/source/unix/lib/build)
set(RTIDDSGEN ${RTI_SRC_INSTALL_DIR}/rtiddsgen/scripts/rtiddsgen)

set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

set(IDL_SOURCE_C
    ../Array1k_.idl
    ../Array4k_.idl
    ../Array16k_.idl
    ../Array32k_.idl
    ../Array60k_.idl
    ../Array1m_.idl
    ../Array2m_.idl
    ../Array4m_.idl

    ../Struct16_.idl
    ../Struct256_.idl
    ../Struct4k_.idl
    ../Struct32k_.idl

    ../PointCloud512k_.idl
    ../PointCloud1m_.idl
    ../PointCloud2m_.idl
    ../PointCloud4m_.idl

    ../Range_.idl
    ../NavSatFix_.idl
    ../NavSatStatus_.idl

    ../RadarDetection_.idl
    ../RadarTrack_.idl

    ../Polygon_.idl
    ../Point_.idl
    ../Point32_.idl
    ../PointField_.idl
    ../Vector3_.idl

    ../Header_.idl
    ../Time_.idl)

set(IDL_GEN_ROOT ${CMAKE_CURRENT_BINARY_DIR}/gen/micro)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})

file(MAKE_DIRECTORY ${IDL_GEN_ROOT})

set(RTI_LIB_PREFIX rti_me)
set(RTI_LIB_SUFFIX_STATIC)
set(RTI_LIB_SUFFIX_DYNAMIC)

if(${CMAKE_BUILD_TYPE} MATCHES "[Rr]elease")
  set(RTI_LIB_SUFFIX_STATIC "z")
  set(RTI_LIB_SUFFIX_DYNAMIC "")
  set(RTI_DEBUG_BUILD FALSE)
  add_definitions(-DNDEBUG)
else()
  set(RTI_LIB_SUFFIX_STATIC "zd")
  set(RTI_LIB_SUFFIX_DYNAMIC "d")
  set(RTI_DEBUG_BUILD TRUE)
endif()

set(MICRO_C_LIBS rti_me_cpp${RTI_LIB_SUFFIX_STATIC}
    rti_me_netioshmem${RTI_LIB_SUFFIX_STATIC}
    rti_me_netiosdm${RTI_LIB_SUFFIX_STATIC}
    rti_me_rhsm${RTI_LIB_SUFFIX_STATIC}
    rti_me_whsm${RTI_LIB_SUFFIX_STATIC}
    rti_me_discdpse${RTI_LIB_SUFFIX_STATIC}
    rti_me_discdpde${RTI_LIB_SUFFIX_STATIC}
    rti_me${RTI_LIB_SUFFIX_STATIC})

include_directories(${IDL_GEN_ROOT} $ENV{RTIMEHOME}/include $ENV{RTIMEHOME}/include/rti_me)
link_directories($ENV{RTIMEHOME}/lib/${RTIME_TARGET_NAME})

foreach(idl ${IDL_SOURCE_C})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})
  set(IDL_GEN_C ${IDL_GEN_ROOT}/${basename}.${SOURCE_EXTENSION}
      ${IDL_GEN_ROOT}/${basename}Plugin.${SOURCE_EXTENSION}
      ${IDL_GEN_ROOT}/${basename}Support.${SOURCE_EXTENSION})

  set(IDL_GEN_H ${IDL_GEN_ROOT}/${basename}.h
      ${IDL_GEN_ROOT}/${basename}Plugin.h
      ${IDL_GEN_ROOT}/${basename}Support.h)

  add_custom_command(OUTPUT ${IDL_GEN_C} ${IDL_GEN_H}
      COMMAND ${RTIDDSGEN} -verbosity 1 -language MicroC++ -micro
      -replace
      -d ${IDL_GEN_ROOT}/. ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
      COMMENT "Regeneration type support plugin from ${idl}")
  set_source_files_properties(${IDL_GEN_C} ${IDL_GEN_H} PROPERTIES GENERATED 1)
  list(APPEND IDL_GEN_H_LIST ${IDL_GEN_H})
  list(APPEND IDL_GEN_C_LIST ${IDL_GEN_C})

endforeach()

add_library(rti_micro_idl
    ${IDL_GEN_H_LIST} ${IDL_GEN_C_LIST})
target_link_libraries(rti_micro_idl ${MICRO_C_LIBS})
set_compile_options(rti_micro_idl)
set(RTI_MICRO_IDL_INCLUDE_DIR ${IDL_GEN_ROOT}/.. $ENV{RTIMEHOME}/include $ENV{RTIMEHOME}/include/rti_me PARENT_SCOPE)
