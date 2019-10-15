set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

if(DEFINED ENV{SYSROOT})
  set(_SYSROOT $ENV{SYSROOT})
else()
  # Github CI expects this to be set automatically.  Other CI might want to set its own
  # SYSROOT
  set(_SYSROOT /sysroot)
endif()

set(CMAKE_SYSROOT ${_SYSROOT})

set(CMAKE_FIND_ROOT_PATH ${_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
