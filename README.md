# Introduction

**WARNING: This version does not support ROS 2 Ardent anymore! Use the following commit if you are still using ROS 2
Ardent: ee89af590f432c6bfd972358ce3c7f3f4292a4c2**

This test allows you to test performance and latency of various communication means
like ROS 2, FastRTPS and Connext DDS Micro.

It can be extended to other communication frameworks easily.

A detailed description can be found here: [Design Article](performance_test/design/performance_test-design.md)

# Requirements

## Building and running performance test

ROS 2: https://github.com/ros2/ros2/wiki/Installation

## Generating graphical plots

The script which generates the graphical plots needs matplotlib and pandas:
```
sudo apt-get install python-matplotlib python-pip
pip install pandas
```

# How to build and run

## RTI Connext Micro
Obtain the following two files and unzip them in e.g. `$HOME/code/rti`:
1. RTI_Connext_MicroSecure-2.5.0-source.zip
2. RTI_Connext_MicroSecure_Host-2.5.0.zip

```sh
cd $HOME/code/rti/rti_connext_micro_secure.2.5.0/source/unix
mkdir build
cd build
cmake .. -DRTIME_TRUST_INCLUDE_BUILTIN=false #workaround special version of openSSL library that RTI Connext Micro expects
make
```

## Performance Test Tool

```
source ros2_install_path/setup.bash
mkdir -p perf_test_ws/src
cd perf_test_ws/src
git clone https://github.com/ApexAI/performance_test.git
cd performance_test
git checkout cust_z
cd ../..
colcon build --cmake-args -DRTI_SRC_INSTALL_DIR="$HOME/code/rti/rti_connext_micro_secure.2.5.0"
source install/setup.bash
ros2 run performance_test perf_test --help
```

To be able to benchmark FastRPTS directly `--cmake-args -DPERFORMANCE_TEST_USE_FASTRTPS` must be set when building.
This is due to some issue when using ROS 2 with FastRTPS (rmw_fastrtps_cpp) and FastRTPS directly in the same application.

# Plot results

After building, a simple experiment can be run using the following.

Before you start please create a directory for the output.
```
mkdir experiment
cd experiment
```

then run the test.
```
ros2 run performance_test perf_test -r 50 -c ConnextDDSMicro -t PointCloud4m -l log -s 1 --max_runtime 300
```

The generated log-files can then be plotted into a PDF file using:
```
ros2 run performance_test performance_test_file_reader.py .
```

# Batch run experiments (for advanced users)

Multiple experiments can be run using using the following command:

```
python src/performance_test/performance_test/helper_scripts/run_experiment.py
```

You need to edit the python script to call the performance test tool with the desired parameters.

# Memory analysis

You can use OSRF memory tools to find memory allocations in your application. To enable it
you need to do the following steps, assuming you already did compile performance test before:

1. Enter your work space: `cd perf_test_ws/src`
1. Clone OSRF memory memory tools: `git clone https://github.com/osrf/osrf_testing_tools_cpp.git`
1. Build everything `cd .. && ament build --parallel --build-tests --cmake-args -DCMAKE_BUILD_TYPE=Release`
1. You need to preload the memory library to make diagnostics work: `export LD_PRELOAD=$(pwd)/install/lib/libmemory_tools_interpose.so`
1. Run with memory check enabled: `ros2 run performance_test perf_test -c ROS2 -l log -t Array1k --max_runtime 10 --memory_check`

Note that enabling this feature will cause a huge performance impact.

# Troubleshooting

1. When running performance test it prints  
`ERROR: You must compile with FastRTPS support to enable FastRTPS as communication mean.`

You need to build with `--cmake-args -DPERFORMANCE_TEST_USE_FASTRTPS` to switch from ROS 2 to FastRTPS.
