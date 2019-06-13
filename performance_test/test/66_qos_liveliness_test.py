# Copyright 2019 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest

import ament_index_python

import launch
import launch.actions

import launch_testing

TEST_PROC_PATH = os.path.join(
    ament_index_python.get_package_prefix('performance_test'),
    'lib/performance_test',
    'perf_test'
)

# This is necessary to get unbuffered output from the process under test
proc_env = os.environ.copy()
proc_env['PYTHONUNBUFFERED'] = '1'

perf_test_process_main = launch.actions.ExecuteProcess(
    cmd=[TEST_PROC_PATH, '-c', 'ROS2', '-t', 'Array1k', '--roundtrip_mode', 'Main'],
    env=proc_env,
)

perf_test_process_relay = launch.actions.ExecuteProcess(
    cmd=[TEST_PROC_PATH, '-c', 'ROS2', '-t', 'Array1k', '--roundtrip_mode', 'Relay'],
    env=proc_env,
)


def generate_test_description(ready_fn):

    return launch.LaunchDescription([
        perf_test_process_main,
        perf_test_process_relay,

        # Start tests right away - no need to wait for anything
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
    ])


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_main_relay(self):
        print("Process output")
        complete_output = [pout.from_stdout for pout in self.proc_output]
        self.assertEquals(complete_output, "foo")
        launch_testing.asserts.assertExitCodes(self.proc_info)
