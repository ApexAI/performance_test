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

set(DDS_ROOT /opt/opendds/share/dds)
set(ACE_ROOT ${DDS_ROOT}/../ace)
set(MPC_ROOT ${DDS_ROOT}/MPC)
set(TAO_ROOT ${DDS_ROOT}/../tao)

set(TAO_IDL DDS_ROOT=${DDS_ROOT} ACE_ROOT=${ACE_ROOT} TAO_ROOT=${TAO_ROOT} MPC_ROOT=${MPC_ROOT} LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):${ACE_ROOT}/lib:${DDS_ROOT}/lib ${ACE_ROOT}/bin/tao_idl)
set(OPENDDS_IDL DDS_ROOT=${DDS_ROOT} ACE_ROOT=${ACE_ROOT} TAO_ROOT=${TAO_ROOT} MPC_ROOT=${MPC_ROOT} LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):${ACE_ROOT}/lib:${DDS_ROOT}/lib ${DDS_ROOT}/bin/opendds_idl)
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


link_directories(${DDS_ROOT}/lib ${ACE_ROOT}/lib)
include_directories(${IDL_GEN_ROOT} ${DDS_ROOT}/../../include/)

foreach(idl ${IDL_SOURCE_C})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})
  set(TAO_IDL_GEN_CPP ${IDL_GEN_ROOT}/${basename}C.${SOURCE_EXTENSION}
          ${IDL_GEN_ROOT}/${basename}S.${SOURCE_EXTENSION}
          ${IDL_GEN_ROOT}/${basename}C.inl)

  set(TAO_IDL_GEN_H ${IDL_GEN_ROOT}/${basename}C.h
          ${IDL_GEN_ROOT}/${basename}S.h)

  set (TAO_IDL_GEN_CPP_SECOND_STAGE ${IDL_GEN_ROOT}/${basename}TypeSupportC.${SOURCE_EXTENSION}
                                    ${IDL_GEN_ROOT}/${basename}TypeSupportS.${SOURCE_EXTENSION}
                                    ${IDL_GEN_ROOT}/${basename}TypeSupportC.inl)

  set(TAO_IDL_GEN_H_SECOND_STAGE ${IDL_GEN_ROOT}/${basename}TypeSupportC.h
                                 ${IDL_GEN_ROOT}/${basename}TypeSupportS.h)

  set(OPENDDS_IDL_GEN_CPP ${IDL_GEN_ROOT}/${basename}TypeSupportImpl.${SOURCE_EXTENSION}
				${IDL_GEN_ROOT}/${basename}TypeSupport.idl)

  set(OPENDDS_IDL_GEN_H ${IDL_GEN_ROOT}/${basename}TypeSupportImpl.h)

  add_custom_command(OUTPUT ${TAO_IDL_GEN_CPP} ${TAO_IDL_GEN_H}
          COMMAND ${TAO_IDL} -o ${IDL_GEN_ROOT}/. ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          COMMENT "Regeneration type support plugin from ${idl}")

  add_custom_command(OUTPUT ${OPENDDS_IDL_GEN_CPP} ${OPENDDS_IDL_GEN_H}
          COMMAND ${OPENDDS_IDL} -o ${IDL_GEN_ROOT}/. ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          COMMENT "Regeneration type support plugin from ${idl}")

  message ("${DDS_ROOT}/../../include/dds" = ${DDS_ROOT}/../../include/dds)

  add_custom_command(OUTPUT ${IDL_GEN_ROOT}/${basename}TypeSupport.idl ${TAO_IDL_GEN_CPP_SECOND_STAGE} ${TAO_IDL_GEN_H_SECOND_STAGE}
          COMMAND ${TAO_IDL} -o ${IDL_GEN_ROOT}/. -I${CMAKE_CURRENT_SOURCE_DIR}/.. -I${DDS_ROOT}/../../include/tao -I${DDS_ROOT}/../../include/ ${IDL_GEN_ROOT}/${basename}TypeSupport.idl
          DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          COMMENT "Regeneration type support plugin from ${idl}")

  set_source_files_properties(${TAO_IDL_GEN_CPP} ${TAO_IDL_GEN_H} PROPERTIES GENERATED 1)
  set_source_files_properties(${OPENDDS_IDL_GEN_CPP} ${OPENDDS_IDL_GEN_H} PROPERTIES GENERATED 1)
  set_source_files_properties(${TAO_IDL_GEN_CPP_SECOND_STAGE} ${TAO_IDL_GEN_H_SECOND_STAGE} PROPERTIES GENERATED 1)
  list(APPEND IDL_GEN_H_LIST ${TAO_IDL_GEN_H} ${OPENDDS_IDL_GEN_H} ${TAO_IDL_GEN_H_SECOND_STAGE})
  list(APPEND IDL_GEN_C_LIST ${TAO_IDL_GEN_CPP} ${OPENDDS_IDL_GEN_CPP} ${TAO_IDL_GEN_CPP_SECOND_STAGE})

endforeach()

add_library(opendds_idl
        ${IDL_GEN_H_LIST} ${IDL_GEN_C_LIST})
set_compile_options(opendds_idl)
set(OPENDDS_IDL_INCLUDE_DIR ${IDL_GEN_ROOT}/.. PARENT_SCOPE)
set(PARENT_DDS_ROOT_INCLUDE ${DDS_ROOT}/../../include PARENT_SCOPE)
set(PARENT_DDS_ROOT_LIBS ${DDS_ROOT}/../../lib PARENT_SCOPE)
