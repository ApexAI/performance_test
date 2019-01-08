#!/usr/bin/env python

# Copyright 2017 Apex.AI, Inc.

# flake8: noqa This file is for plotting data about page faults and context switches.
# Its dependencies are not necessarily on the CI.
import os
import sys
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt # noqa:
import matplotlib.backends.backend_pdf


def parse_file(file, single_file=None):
    if not os.path.basename(file).startswith("log_") or file.endswith(".pdf"):
        print("Skipping "+str(file))
        return
    try:
        print("Parsing file:" + str(file))
        dataframe = pd.read_csv(file, skiprows=N + 1, sep="[ \t]*,[ \t]*", engine='python')
        with open(file) as myfile:
            head = [next(myfile) for x in range(0, N)]

        if not dataframe.empty:
            pd.options.display.float_format = '{:.4f}'.format
            dataframe = dataframe.iloc[20:] #dropping the first few rows to not mess up the graphs.
            # ru_maxrss is in KB: http://man7.org/linux/man-pages/man2/getrusage.2.html. Converting to Mb
            dataframe["maxrss (Mb)"] = dataframe["ru_maxrss"] / 1e3

            dataframe[["T_experiment",
                       "ru_minflt",
                       "ru_majflt",
                       "ru_nivcsw",
                        "maxrss (Mb)"]] \
                .plot(x='T_experiment', secondary_y=["maxrss (Mb)"])
            df_last = dataframe.tail(1)
            cpu_per = ((df_last["ru_utime"] + df_last["ru_stime"]) / (NCPUS * df_last["T_experiment"])) * 100.0

            dataframe["CPU util (%)"] = cpu_per
            plt.figtext(0.0, 1.0, ''.join(head), fontsize=8, horizontalalignment='left')

            # Reduced dataframe containing few resource usages
            res_df = pd.DataFrame()
            res_df["CPU util (%)"] = cpu_per
            res_df["CPU user time(ru_utime)"] = df_last["ru_utime"]
            res_df["CPU system time (ru_stime)"] = df_last["ru_stime"]
            res_df["Invol context switch (ru_nivcsw)"] = df_last["ru_nivcsw"]
            res_df["Minor fault (ru_minflt)"] = df_last["ru_minflt"]
            res_df["Major fault (ru_majflt)"] = df_last["ru_majflt"]
            res_df["Max res size (maxrss (Kb))"] = df_last["ru_maxrss"]

            plt.figtext(0.75, 0.9, res_df.mean().round(4), fontsize=8,
                        horizontalalignment='left')

            if single_file is None:
                plt.savefig(file + "_pf.pdf",
                            bbox_inches=matplotlib.transforms.Bbox(np.array(((0, 0), (8, 8)))))
            else:
                single_file.savefig(bbox_inches=matplotlib.transforms.Bbox(np.array(((0, 0), (8, 8)))))
            plt.close('all')
    except:  # noqa: E722 I do rethrow.
        print("Could not parse file: " + str(file) + "\n")


pdf = None
if len(sys.argv) == 2:
    directory = sys.argv[1]

elif len(sys.argv) == 3:
    directory = sys.argv[1]
    pdf = matplotlib.backends.backend_pdf.PdfPages(sys.argv[2])

else:
    print("Usage: python performance_test_file_reader.py path_to_log_files [single_pdf_output_file]")
    sys.exit(0)

logfiles = []
N = 17  # Number of line to skip before CSV data starts.
NCPUS = 6 #Number of cpu's in the system

# If the given directory actually a file.
os.path.isfile(directory)
parse_file(os.path.join(os.path.dirname(directory), os.path.basename(directory)), pdf)

for root, subdirs, files in os.walk(directory):
    for file in files:
        parse_file(os.path.join(root, file), pdf)

if pdf is not None:
    pdf.close()
