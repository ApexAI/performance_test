# Copyright 2017 Apex.AI, Inc.
import os
import signal
import subprocess
import sys
import time
import itertools

# Select DDS implementation for ROS 2. This setting is only used when ROS 2 is used as a
# communication mean.

experiment_length = 120  # In seconds

topics = ["Array1k", "Array4k", "Array16k", "Array32k", "Array60k", "Array1m", "Array2m",
          "Struct16", "Struct256", "Struct4k", "Struct32k", "PointCloud512k", "PointCloud1m", "PointCloud2m",
          "PointCloud4m", "Range", "NavSatFix", "RadarDetection", "RadarTrack"]

rates = ["50", "1000"]

num_subs = ["1", "3", "10"]

reliability = ["", "--reliable"]
durability = ["", "--transient"]

current_index = 0

def cmd(index):
    command = "ros2 run  performance_test perf_test"

    product = list(itertools.product(topics, rates, num_subs, reliability, durability))
    c = product[index]


    dir = "rate_"+c[1]+"/subs_"+c[2]

    if not os.path.exists(dir):
        os.makedirs(dir)
    fixed_args = " --communication ROS2 -p 1 "
    dyn_args = "-l '" + dir + "/log' " + "--topic " + c[0] + " --rate " + c[1] + " -s " + c[2] + " " + c[3] + " " + c[4]

    return command + " " + fixed_args + dyn_args


def exec_cmd(index):
    print(cmd(index))
    p = subprocess.Popen(cmd(index), shell=True)
    # We sleeping here to make sure the process is started before changing its priority.
    time.sleep(2)
    # Enabling (pseudo-)realtime
    # subprocess.Popen('chrt -p 99 $(ps -o pid -C "perf_test" --no-headers)', shell=True)

    return p


p = exec_cmd(current_index)


def signal_handler(signal, frame):
    global p
    print('You pressed Ctrl+C! Terminating experiment')
    p.kill()
    subprocess.Popen("killall perf_test", shell=True)
    sys.exit(0)


def timer_handler(signal, frame):
    global current_index
    global p
    p.kill()
    subprocess.Popen("killall perf_test", shell=True)
    current_index = current_index + 1
    if current_index >= len(topics):
        print("Done with experiments.")
        exit(0)
    p = exec_cmd(current_index)


signal.signal(signal.SIGALRM, timer_handler)
signal.signal(signal.SIGINT, signal_handler)
signal.setitimer(signal.ITIMER_REAL, experiment_length, experiment_length)
print('Press Ctrl+C to abort experiment')
while True:
    signal.pause()
    print("Next experiment.")
