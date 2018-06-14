# Copyright 2017 Apex.AI, Inc.
import os
import signal
import subprocess
import sys
import time

# Select DDS implementation for ROS 2. This setting is only used when ROS 2 is used as a
# communication mean.
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

experiment_length = 1500  # In seconds

topics = ["Array1k", "Array4k", "Array16k", "Array32k", "Array60k", "Array1m", "Array2m",
          "Struct16", "Struct256", "Struct4k", "Struct32k"]
current_index = 0


def cmd(index):
    cmd = "ros2 run  performance_test perf_test"
    args = "-l 'log' --communication FastRTPS --rate 1000 -p 1 -s 3 --topic "
    return cmd + " " + args + topics[index]


def exec_cmd(index):
    p = subprocess.Popen(cmd(current_index), shell=True)
    # We sleeping here to make sure the process is started before changing its priority.
    time.sleep(2)
    # Enabling (pseudo-)realtime
    subprocess.Popen('chrt -p 99 $(ps -o pid -C "perf_test" --no-headers)', shell=True)

    return p


p = exec_cmd(0)


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
