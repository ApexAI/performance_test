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

set(SOURCE_EXTENSION c)

#find_package(CycloneDDS REQUIRED CONFIG)

set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

set(SUPP_IDL_SOURCE_C
  ../Header_.idl
  ../PointField_.idl
  ../Time_.idl
  ../NavSatStatus_.idl
  ../Polygon_.idl
  ../Point_.idl
  ../Point32_.idl
  ../Vector3_.idl)

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

  ../RadarDetection_.idl
  ../RadarTrack_.idl)

set(IDL_GEN_ROOT ${CMAKE_CURRENT_BINARY_DIR}/gen/cyclonedds)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})

file(MAKE_DIRECTORY ${IDL_GEN_ROOT})

foreach(idl ${SUPP_IDL_SOURCE_C} ${IDL_SOURCE_C})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})

  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/${idl}" DESTINATION "${IDL_GEN_ROOT}")
  if(${idl} IN_LIST IDL_SOURCE_C)
    file(APPEND ${IDL_GEN_ROOT}/${basename}.idl
      "#pragma keylist performance_test_msgs::msg::dds_::${basename}\n")
  endif()
  
  set(IDL_GEN_C ${IDL_GEN_ROOT}/${basename}.c)
  set(IDL_GEN_H ${IDL_GEN_ROOT}/${basename}.h)

  add_custom_command(
    OUTPUT   ${IDL_GEN_C} ${IDL_GEN_H}
    COMMAND  "${IDLC_DIR}/${IDLC}"
    ARGS     -d ${IDL_GEN_ROOT}/. ${IDL_GEN_ROOT}/${basename}.idl
    DEPENDS  ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
    COMMENT  "Running idlc on ${FIL}"
    VERBATIM)
  set_source_files_properties(${IDL_GEN_C} ${IDL_GEN_H} PROPERTIES GENERATED 1)
  list(APPEND IDL_GEN_H_LIST ${IDL_GEN_H})
  list(APPEND IDL_GEN_C_LIST ${IDL_GEN_C})
endforeach()

add_library(cyclonedds_idl ${IDL_GEN_H_LIST} ${IDL_GEN_C_LIST})

#target_include_directories(cyclonedds_idl PRIVATE CycloneDDS::ddsc)
get_target_property(cdds_include_dir CycloneDDS::ddsc INTERFACE_INCLUDE_DIRECTORIES)
include_directories("${cdds_include_dir}")

set_compile_options(cyclonedds_idl)
set(CYCLONEDDS_IDL_INCLUDE_DIR ${IDL_GEN_ROOT}/.. PARENT_SCOPE)
