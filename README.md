# Introduction

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
sudo apt-get install python-matplotlib
pip install pandas
```

# How to build and run

```
source ros2_install_path/local_setup.bash
mkdir -p perf_test_ws/src
cd perf_test_ws/src
git clone git@github.com:ApexAI/performance_test
cd ..
ament build --parallel --build-tests --cmake-args -DCMAKE_BUILD_TYPE=Debug --
ros2 run performance_test perf_test --help
```

To be able to benchmark FastRPTS directly `--cmake-args -DPERFORMANCE_TEST_USE_FASTRTPS` must be set when building.
This is due to some issue when using ROS 2 with FastRTPS and FastRTPS directly in the same application.

# Plot results

After building a simple experiment can be run using:
```
ros2 run performance_test perf_test -c ROS2 -l log -t Array1k --max_runtime 10
```

The generated log-files can then plotted into a PDF file using:
```
python src/performance_test/performance_test/helper_scripts/performance_test_file_reader.py .
```

# Batch run experiments (for advanced users)

Multiple experiments can be run using using the the following command:

```
python src/performance_test/performance_test/helper_scripts/run_experiment.py
```

You need to edit the python script to call the performance test tool with the desired parameters.

# Troubleshooting

** When running performance test it prints
`ERROR: You must compile with FastRTPS support to enable FastRTPS as communication mean.`
**

You need to build with `--cmake-args -DPERFORMANCE_TEST_USE_FASTRTPS` to switch from ROS 2 to FastRTPS.
