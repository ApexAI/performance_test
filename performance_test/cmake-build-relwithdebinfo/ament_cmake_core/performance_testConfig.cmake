# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_performance_test_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED performance_test_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(performance_test_FOUND FALSE)
  elseif(NOT performance_test_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(performance_test_FOUND FALSE)
  endif()
  return()
endif()
set(_performance_test_CONFIG_INCLUDED TRUE)

# output package information
if(NOT performance_test_FIND_QUIETLY)
  message(STATUS "Found performance_test: 0.0.1 (${performance_test_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'performance_test' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(performance_test_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_dependencies-extras.cmake;rosidl_cmake-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${performance_test_DIR}/${_extra}")
endforeach()
