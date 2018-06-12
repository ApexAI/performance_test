# Copyright 2017 Apex.AI, Inc.

# flake8: noqa This file is for plotting data. Its dependencies are not necessarily on the CI.
import os
import sys
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt # noqa:

if len(sys.argv) == 2:
    directory = sys.argv[1]
else:
    print("Usage: python performance_test_file_reader.py /path_to_log_files")
    sys.exit(0)

logfiles = []
N = 14  # Number of line to skip before CSV data starts.

for file in os.listdir(directory):
    if file.startswith("log_"):
        print(os.path.join(directory, file))
        logfiles.append(os.path.join(directory, file))

for f in logfiles:
    try:
        print("Parsing file:" + str(f))
        dataframe = pd.read_csv(f, skiprows=N + 1, sep="[ \t]*,[ \t]*", engine='python')
        # dataframe = dataframe.drop(columns=['Unnamed: 19'])
        with open(f) as myfile:
            head = [next(myfile) for x in range(0, N)]
            print(''.join(head))

        if not dataframe.empty:
            pd.options.display.float_format = '{:.4f}'.format
            dataframe.drop(list(dataframe.filter(regex='ru_')), axis=1, inplace=True)
            dataframe["latency_variance (ms) * 100"] = 100.0 * dataframe["latency_variance (ms)"]
            dataframe[["T_experiment",
                       "latency_min (ms)",
                       "latency_max (ms)",
                       "latency_mean (ms)",
                       "latency_variance (ms) * 100"]] \
                .plot(x='T_experiment')
            plt.figtext(0.0, 1.0, ''.join(head), fontsize=8, horizontalalignment='left')
            plt.figtext(0.65, 0.9, dataframe.mean().round(4), fontsize=8,
                        horizontalalignment='left')
            plt.savefig(os.path.basename(f) + ".pdf",
                        bbox_inches=matplotlib.transforms.Bbox(np.array(((0, 0), (8, 8)))))
    except:  # noqa: E722 I do rethrow.
        print("Could not parse file: " + str(f) + "\n")
        raise
