# Introduction

This test allows you to test performance and latency of various communication means
like ROS 2, FastRTPS, Connext DDS Micro.

It can be extended to other communication framworks easily.

Are detailed description can be found here: [Design Article](performance_test/design/performance_test-design.md)

# How to build and run

1. Clone the repository into your ament workspace.
1. Source your ROS 2 environment: `source /opt/ros/ardent/local_setup.bash`
1. Build: `ament build --parallel --build-tests --cmake-args -DCMAKE_BUILD_TYPE=Release --`
1. Source local installation: `source install/local_setup.bash`
1. Configure experiment configuratin in `performance_test/helper_scripts/run_experiment.py`
1. Run an experiment: `performance_test/helper_scripts/run_experiment.py`
1. Plot results (requires `pandas`):  `performance_test/helper_scripts/performance_test_file_reader.py`
