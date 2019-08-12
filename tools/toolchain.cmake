set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
set(CMAKE_SYSROOT /sysroot/)
set(TINYXML2_LIBRARY /foo)

# colcon build --merge-install --cmake-args -DCMAKE_TOOLCHAIN_FILE=`pwd`/tools/toolchain.cmake -DBUILD_TESTING=OFF
