# Copyright 2017 Apex.AI, Inc.
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

"""Script to run a batch of performance experiments."""
from enum import Enum
import itertools
import os
import signal
import subprocess
import sys

experiment_length = 120  # In seconds


class Type(Enum):
    """Define the enumeration for the experiment types."""

    PUBLISHER = 0
    SUBSCRIBER = 1
    BOTH = 2


class Instance:
    """perf_test process encapsulation."""

    def __init__(self, operation_type):
        """
        Constructor.

        :param operation_type: Type of the operation
        """
        topics = [
            'Array1k', 'Array4k', 'Array16k', 'Array32k', 'Array60k',
            'Array1m', 'Array2m', 'Struct16', 'Struct256', 'Struct4k',
            'Struct32k', 'PointCloud512k', 'PointCloud1m', 'PointCloud2m',
            'PointCloud4m', 'Range', 'NavSatFix', 'RadarDetection',
            'RadarTrack'
        ]

        rates = ['20', '50', '1000']

        num_subs = ['1', '3', '10']

        reliability = ['', '--reliable']
        durability = ['', '--transient']

        self.product = list(itertools.product(
            topics, rates, num_subs, reliability, durability)
        )
        self.process = None

        self.type = operation_type

    def run(self, index):
        """
        Run the embedded perf_test process.

        :param index: The test configuration to run.
        """
        print('*******************')
        print(self.cmd(index))
        print('*******************')

        self.process = subprocess.Popen(self.cmd(index), shell=True)

        # Comment out the following lines to run the experiments
        # with soft realtime priority.
        # We sleeping here to make sure the process is started before
        # changing its priority.
        # time.sleep(2)
        # Enabling (pseudo-)realtime
        # subprocess.Popen(
        #     'chrt -p 99 $(ps -o pid -C 'perf_test' --no-headers)', shell=True
        # )

    def cmd(self, index):
        """
        Return the command line necessary to execute the performance test.

        :param index: The test configuration the returned command line should
            contain.
        :return: The command line argument to execute the performance test.
        """
        command = 'ros2 run  performance_test perf_test'

        c = list(self.product[index])

        if self.type == Type.PUBLISHER:
            c[2] = '0'
            pubs_args = ' -p1 '
        elif self.type == Type.SUBSCRIBER:
            pubs_args = ' -p0 '
        elif self.type == Type.BOTH:
            pubs_args = ' -p1 '
        else:
            raise ValueError('Unsupported type')

        dir_name = 'rate_' + c[1] + '/subs_' + c[2]

        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        fixed_args = ' --communication ROS2 '
        dyn_args = "-l '{}/log' --topic {} --rate {} -s {} {} {}".format(
            dir_name, c[0], c[1], c[2], c[3], c[4]
        )

        return command + ' ' + fixed_args + dyn_args + pubs_args

    def kill(self):
        """Kill the associated performance test process."""
        if self.process is not None:
            self.process.kill()

    def num_runs(self):
        """Return the number of experiments runs this instance can execute."""
        return len(self.product)

    def __del__(self):
        """Kill the associated performance test process."""
        self.kill()


current_index = 0

num_pub_processes = 0
num_sub_processes = 0
num_both = 1

pub_list = [Instance(Type.PUBLISHER) for _ in range(0, num_pub_processes)]
sub_list = [Instance(Type.SUBSCRIBER) for _ in range(0, num_sub_processes)]
both_list = [Instance(Type.BOTH) for _ in range(0, num_both)]
full_list = pub_list + sub_list + both_list


def signal_handler(sig, frame):
    """Signal handler to handle Ctrl-C."""
    print('You pressed Ctrl+C! Terminating experiment')
    subprocess.Popen('killall perf_test', shell=True)
    sys.exit(0)


def timer_handler(sig=None, frame=None):
    """Signal handler to handle the timer."""
    global current_index
    global full_list

    [e.kill() for e in full_list]
    subprocess.Popen('killall -9 perf_test', shell=True)
    current_index = current_index + 1
    if current_index >= full_list[0].num_runs():
        print('Done with experiments.')
        exit(0)
    else:
        [e.run(current_index) for e in full_list]


signal.signal(signal.SIGALRM, timer_handler)
signal.signal(signal.SIGINT, signal_handler)
signal.setitimer(signal.ITIMER_REAL, experiment_length, experiment_length)

timer_handler()
print('Press Ctrl+C to abort experiment')
while True:
    signal.pause()
    print('Next experiment.')
