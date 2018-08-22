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

import os
import signal
import subprocess
import sys
import time
import itertools
from enum import Enum
experiment_length = 120  # In seconds

class Type(Enum):
    PUBLISHER = 0
    SUBSCRIBER = 1
    BOTH = 2

class Instance:
    def __init__(self, type):

        topics = ["Array1k", "Array4k", "Array16k", "Array32k", "Array60k", "Array1m", "Array2m",
                  "Struct16", "Struct256", "Struct4k", "Struct32k", "PointCloud512k", "PointCloud1m", "PointCloud2m",
                  "PointCloud4m", "Range", "NavSatFix", "RadarDetection", "RadarTrack"]

        rates = ["50", "1000"]

        num_subs = ["1", "3", "10"]

        reliability = ["", "--reliable"]
        durability = ["", "--transient"]

        self.product = list(itertools.product(topics, rates, num_subs, reliability, durability))
        self.process = None

        self.type = type

    def run(self, index):
        print("*******************")
        print(self.cmd(index))
        print("*******************")

        self.process = subprocess.Popen(self.cmd(index), shell=True)
        # We sleeping here to make sure the process is started before changing its priority.
        # time.sleep(2)
        # Enabling (pseudo-)realtime
        # subprocess.Popen('chrt -p 99 $(ps -o pid -C "perf_test" --no-headers)', shell=True)


    def cmd(self, index):
        command = "ros2 run  performance_test perf_test"

        c = list(self.product[index])

        if self.type == Type.PUBLISHER:
            c[2] = "0"
            pubs_args = " -p1 "
        elif self.type == Type.SUBSCRIBER:
            pubs_args = " -p0 "
        elif self.type == Type.BOTH:
            pubs_args = " -p1 "
        else:
            raise "Unsupported type"

        dir = "rate_"+c[1]+"/subs_"+c[2]

        if not os.path.exists(dir):
            os.makedirs(dir)
        fixed_args = " --communication ROS2 "
        dyn_args = "-l '" + dir + "/log' " + "--topic " + c[0] + " --rate " + c[1] + " -s " + c[2] + " " + c[3] + " " + c[4]

        return command + " " + fixed_args + dyn_args + pubs_args

    def kill(self):
        if self.process is not None:
            self.process.kill()

    def num_runs(self):
        return len(self.product)

    def __del__(self):
        self.kill()

current_index = 0

num_pubs = 1
num_subs = 30
num_both = 0

pub_list = [Instance(Type.PUBLISHER) for _ in range(0, num_pubs)]
sub_list = [Instance(Type.SUBSCRIBER) for _ in range(0, num_subs)]
both_list = [Instance(Type.BOTH) for _ in range(0, num_both)]
full_list = pub_list + sub_list + both_list



def signal_handler(signal, frame):
    global p
    print('You pressed Ctrl+C! Terminating experiment')
    p.kill()
    subprocess.Popen("killall perf_test", shell=True)
    sys.exit(0)


def timer_handler(signal=None, frame=None):
    global current_index
    global full_list

    [e.kill() for e in full_list]
    subprocess.Popen("killall -9 perf_test", shell=True)
    current_index = current_index + 1
    if current_index >= full_list[0].num_runs():
        print("Done with experiments.")
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
    print("Next experiment.")
