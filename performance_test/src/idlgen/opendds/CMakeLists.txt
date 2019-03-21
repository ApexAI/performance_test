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

set(SOURCE_EXTENSION cpp)

set(ENV{LD_LIBRARY_PATH} $ENV{LD_LIBRARY_PATH}:/home/salus/OpenDDS-3.13.1/ACE_wrappers/lib:/home/salus/OpenDDS-3.13.1/lib)
set(ENV{ACE_ROOT} /home/salus/OpenDDS-3.13.1/ACE_wrappers)
set(ENV{MPC_ROOT} /home/salus/OpenDDS-3.13.1/ACE_wrappers/MPC)
set(ENV{TAO_ROOT} /home/salus/OpenDDS-3.13.1/ACE_wrappers/TAO)

set(TAO_IDL DDS_ROOT=/home/salus/OpenDDS-3.13.1 ACE_ROOT=/home/salus/OpenDDS-3.13.1/ACE_wrappers TAO_ROOT=/home/salus/OpenDDS-3.13.1/ACE_wrappers/TAO MPC_ROOT=/home/salus/OpenDDS-3.13.1/ACE_wrappers/MPC LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):/home/salus/OpenDDS-3.13.1/ACE_wrappers/lib:/home/salus/OpenDDS-3.13.1/lib $ENV{HOME}/OpenDDS-3.13.1/ACE_wrappers/bin/tao_idl)
set(OPENDDS_IDL DDS_ROOT=/home/salus/OpenDDS-3.13.1 ACE_ROOT=/home/salus/OpenDDS-3.13.1/ACE_wrappers TAO_ROOT=/home/salus/OpenDDS-3.13.1/ACE_wrappers/TAO MPC_ROOT=/home/salus/OpenDDS-3.13.1/ACE_wrappers/MPC LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):/home/salus/OpenDDS-3.13.1/ACE_wrappers/lib:/home/salus/OpenDDS-3.13.1/lib $ENV{HOME}/OpenDDS-3.13.1/bin/opendds_idl)
set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

set(IDL_SOURCE_C
    ../Array1k_.idl
)

set(IDL_GEN_ROOT ${CMAKE_CURRENT_BINARY_DIR}/gen/opendds)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})

file(MAKE_DIRECTORY ${IDL_GEN_ROOT})

set(OPENDDS_LIB_PREFIX rti_me)
set(OPENDDS_LIB_SUFFIX_STATIC)
set(OPENDDS_LIB_SUFFIX_DYNAMIC)

if(${CMAKE_BUILD_TYPE} MATCHES "[Rr]elease")
  set(OPENDDS_LIB_SUFFIX_STATIC "z")
  set(OPENDDS_LIB_SUFFIX_DYNAMIC "")
  set(OPENDDS_DEBUG_BUILD FALSE)
  add_definitions(-DNDEBUG)
else()
  set(OPENDDS_LIB_SUFFIX_STATIC "zd")
  set(OPENDDS_LIB_SUFFIX_DYNAMIC "d")
  set(OPENDDS_DEBUG_BUILD TRUE)
endif()

set(MICRO_C_LIBS rti_me_rhsm${RTI_LIB_SUFFIX_STATIC}
        rti_me_whsm${RTI_LIB_SUFFIX_STATIC}
        rti_me_discdpse${RTI_LIB_SUFFIX_STATIC}
        rti_me_discdpde${RTI_LIB_SUFFIX_STATIC}
        rti_me${RTI_LIB_SUFFIX_STATIC})


set(OPENDDS_HOME /home/salus/OpenDDS-3.13.1)
link_directories(${OPENDDS_HOME}/lib ${OPENDDS_HOME}/ACE_wrappers/lib)
include_directories(${IDL_GEN_ROOT} ${OPENDDS_HOME} ${OPENDDS_HOME}/ACE_wrappers/ ${OPENDDS_HOME}/ACE_wrappers/TAO)

foreach(idl ${IDL_SOURCE_C})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})
  set(TAO_IDL_GEN_CPP ${IDL_GEN_ROOT}/${basename}C.${SOURCE_EXTENSION}
          ${IDL_GEN_ROOT}/${basename}S.${SOURCE_EXTENSION}
          ${IDL_GEN_ROOT}/${basename}C.inl)

  set(TAO_IDL_GEN_H ${IDL_GEN_ROOT}/${basename}C.h
          ${IDL_GEN_ROOT}/${basename}S.h)

  set(OPENDDS_IDL_GEN_CPP ${IDL_GEN_ROOT}/${basename}TypeSupportImpl.${SOURCE_EXTENSION}
          ${IDL_GEN_ROOT}/${basename}TypeSupport.idl)

  set(OPENDDS_IDL_GEN_H ${IDL_GEN_ROOT}/${basename}TypeSupportImpl.h)

  message("Running tao_idl Command: Output files " ${TAO_IDL_GEN_CPP})
  message("Location IDL: " ${CMAKE_CURRENT_SOURCE_DIR}/${idl})
  add_custom_command(OUTPUT ${TAO_IDL_GEN_CPP} ${TAO_IDL_GEN_H}
          COMMAND ${TAO_IDL} -o ${IDL_GEN_ROOT}/. ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          COMMENT "Regeneration type support plugin from ${idl}")

  message("Running opendds_idl Command: Output files: " ${OPENDDS_IDL_GEN_CPP})
  add_custom_command(OUTPUT ${OPENDDS_IDL_GEN_CPP} ${OPENDDS_IDL_GEN_H}
          COMMAND ${OPENDDS_IDL} -o ${IDL_GEN_ROOT}/. ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          COMMENT "Regeneration type support plugin from ${idl}")

  set_source_files_properties(${TAO_IDL_GEN_CPP} ${TAO_IDL_GEN_H} PROPERTIES GENERATED 1)
  set_source_files_properties(${OPENDDS_IDL_GEN_CPP} ${OPENDDS_IDL_GEN_H} PROPERTIES GENERATED 1)
  list(APPEND IDL_GEN_H_LIST ${TAO_IDL_GEN_H} ${TAO_IDL_GEN_CPP})
  list(APPEND IDL_GEN_C_LIST ${TAO_IDL_GEN_CPP} ${OPENDDS_IDL_GEN_CPP})

endforeach()

MESSAGE(ERROR  ${IDL_GEN_H_LIST} ${IDL_GEN_C_LIST})
add_library(opendds_idl
        ${IDL_GEN_H_LIST} ${IDL_GEN_C_LIST})
set_compile_options(opendds_idl)
set(OPENDDS_IDL_INCLUDE_DIR ${IDL_GEN_ROOT}/.. PARENT_SCOPE)
