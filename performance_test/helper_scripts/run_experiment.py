# Copyright 2017 Apex.AI, Inc.
import os
import signal
import subprocess
import sys
import time

experiment_length = 300  # In seconds

topics = ["Array1k",  "Array60k",  "Array2m",
           "Struct256","Struct32k"]
current_index = 0


def cmd(index):
    cmd = "ros2 run  performance_test perf_test"
    args = "-l 'log' --communication ROS2 --rate 10000 -p 1 -s 3 --topic "
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
