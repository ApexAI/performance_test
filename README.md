# Build Status
[![Build Status](https://dev.azure.com/ApexAI/performance_test/_apis/build/status/ApexAI.performance_test?branchName=master)](https://dev.azure.com/ApexAI/performance_test/_build/latest?definitionId=2&branchName=master)

# Introduction

**Version Support:** ROS2 Dashing, Fast-RTPS 1.8.0

This test allows you to test performance and latency of various communication means
like ROS 2, FastRTPS, Connext DDS Micro and Eclipse Cyclone DDS.

It can be extended to other communication frameworks easily.

A detailed description can be found here: [Design Article](performance_test/design/performance_test-design.md)

# Requirements

## Building and running performance test

ROS 2: https://github.com/ros2/ros2/wiki/Installation

Additional dependencies are Java and others declared in the `package.xml` file 
```
sudo apt-get install default-jre
rosdep install -y --from performance_test --ignore-src
```

## Generating graphical plots

The script which generates the graphical plots needs matplotlib and pandas:
```
sudo apt-get install python-matplotlib python-pip
pip install pandas
```

# How to build and run

```
source ros2_install_path/setup.bash
mkdir -p perf_test_ws/src
cd perf_test_ws/src
git clone https://github.com/ApexAI/performance_test.git
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run performance_test perf_test --help
```
# Plot results

After building, a simple experiment can be run using the following.

Before you start please create a directory for the output.
```
mkdir experiment
cd experiment
```

then run the test.
```
ros2 run performance_test perf_test -c ROS2 -l log -t Array1k --max_runtime 10
```

The generated log-files can then be plotted into a PDF file using:
```
ros2 run performance_test performance_test_file_reader.py .
```

# Save results to SQL database

## Requirements
***ODB 2.5.0 (this is required for gcc 7.4.0):***
Follow the instructions [here](https://www.codesynthesis.com/products/odb/doc/install-build2.xhtml)
 to install build2 toolchain, odb compiler and runtime libraries.

Remember to install the correct gcc plugin, for example (for gcc 7):
```
sudo apt-get update
sudo apt-get install gcc-7-plugin-dev
```

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

***ODB 2.4.0:***
All the downloads can be found [here](https://www.codesynthesis.com/products/odb/download.xhtml).
* Odb compiler ([Instructions](https://www.codesynthesis.com/products/odb/doc/install-unix.xhtml))
* Common Runtime Library: libodb-2.4.0
* Database Runtime Library: libodb-sqlite-2.4.0
* Profile Libraries: libodb-boost-2.4.0, libodb-qt-2.4.0

## How to build and run
```
source ros2_install_path/setup.bash
mkdir -p perf_test_ws/src
cd perf_test_ws/src
git clone https://github.com/ApexAI/performance_test.git
cd performance_test
git checkout 3749-add-sql-support
cd ../..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DODB_FOR_SQL_ENABLED=ON -DSQLITE=ON
source install/setup.bash
ros2 run performance_test perf_test -c ROS2 -l log -t Array1k --max_runtime 10
```

The default name of resulting database is "test_database", you can change it by using `--db_name`
argument in `ros2 run`. Additionally, if you want to use MySQL or PostgreSQL instead of SQLite, use
-DMYSQL=ON or -DPGSQL and disable the SQLite by adding _DSQLITE=OFF option.

# Batch run experiments (for advanced users)

Multiple experiments can be run using the following command:

```
python src/performance_test/performance_test/helper_scripts/run_experiment.py
```

You need to edit the python script to call the performance test tool with the desired parameters.

# Relay mode

Testing latency between multiple machines is difficult as it is hard precisely synchronize clocks between them.
To overcome this issue performance test supports relay mode which allows for a round-trip style of communication.

On the main machine: `ros2 run performance_test perf_test -c ROS2 -t Array1k --roundtrip_mode Main`
On the relay machine: `ros2 run performance_test perf_test -c ROS2 -t Array1k --roundtrip_mode Relay`

Note that on the main machine the round trip latency is reported and will be roughly double the latency compared to
the latency reported in non-relay mode.

# Memory analysis

You can use OSRF memory tools to find memory allocations in your application. To enable it
you need to do the following steps, assuming you already did compile performance test before:

1. Enter your work space: `cd perf_test_ws/src`
1. Clone OSRF memory memory tools: `git clone https://github.com/osrf/osrf_testing_tools_cpp.git`
1. Build everything `cd .. && ament build --parallel --build-tests --cmake-args -DCMAKE_BUILD_TYPE=Release`
1. You need to preload the memory library to make diagnostics work: `export LD_PRELOAD=$(pwd)/install/lib/libmemory_tools_interpose.so`
1. Run with memory check enabled: `ros2 run performance_test perf_test -c ROS2 -l log -t Array1k --max_runtime 10 --check_memory`

Note that enabling this feature will cause a huge performance impact.

# Custom environment data

You can set the `APEX_PERFORMANCE_TEST` environment variable before running performance test
to add custom data to the output CSV file.
This information will then also be visible in the files outputted by the plotter script.

Example:
```
export APEX_PERFORMANCE_TEST="
My Version: 1.0.4
My Image Version: 5.2
My OS Version: Ubuntu 16.04
"
ros2 run performance_test -c ROS2 -t Array1k
```

# Troubleshooting

1. When running performance test it prints
`ERROR: You must compile with FastRTPS support to enable FastRTPS as communication mean.`

You need to build with `--cmake-args -DPERFORMANCE_TEST_FASTRTPS_ENABLED=ON` to switch from ROS 2 to FastRTPS.

# Known Limitations

* FastRTPS waitset does not support a timeout which can lead to the receiving not aborting. In that case the performance test must be manually killed.

