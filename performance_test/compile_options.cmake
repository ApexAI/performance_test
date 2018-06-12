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

function(set_compile_options target)
    if(WIN32)
        # Causes the visibility macros to use dllexport rather than dllimport,
        # which is appropriate when building the dll but not consuming it.
        string(TOUPPER ${target} PROJECT_NAME_UPPER)
        target_compile_definitions(${target} PRIVATE ${PROJECT_NAME_UPPER}_BUILDING_DLL)
        add_definitions(-D_CRT_NONSTDC_NO_WARNINGS)
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
        add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
    else()
        target_compile_options(${target} PRIVATE  -Wall
                -Wextra
                #-Wshadow             # causes issues with ROS2 headers
                #-Wnon-virtual-dtor   # causes issues with ROS2 headers
                -pedantic
                -Wcast-align
                -Wunused
                -Woverloaded-virtual
                -Wconversion
                -Wsign-conversion
                -Wlogical-op
                -Wuseless-cast
                -Wdouble-promotion
                -Wold-style-cast
                #-Wnull-dereference    # gcc6
                #-Wduplicated-branches # gcc7
                #-Wduplicated-cond     # gcc6
                #-Wrestrict            # gcc7
                -fvisibility=hidden)
        if(CMAKE_BUILD_TYPE EQUAL "Debug")
            set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-Og")
        else()
            set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-O3 -ftree-vectorize")
        endif()
    endif()
endfunction()
