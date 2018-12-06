#!/usr/bin/env python

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

# flake8: noqa This file is for plotting data. Its dependencies are not necessarily on the CI.
import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt # noqa:
import matplotlib.backends.backend_pdf

def generate_report(sec_on_file, sec_off_file, single_file=None):
    try:
        print("Parsing security on data file:" + str(sec_on_file))
        print("Parsing security on data file:" + str(sec_off_file))
        on_dataframe = pd.read_csv(sec_on_file, skiprows=N + 1, sep="[ \t]*,[ \t]*", engine='python')
        off_dataframe = pd.read_csv(sec_off_file, skiprows=N + 1, sep="[ \t]*,[ \t]*", engine='python')

        if not on_dataframe.empty:
            pd.options.display.float_format = '{:.4f}'.format

            # ru_maxrss is in KB: http://man7.org/linux/man-pages/man2/getrusage.2.html. Converting to Mb
            on_dataframe["maxrss (Mb)"] = on_dataframe["ru_maxrss"] / 1e3
            on_dataframe.drop(list(on_dataframe.filter(regex='ru_')), axis=1, inplace=True)
            on_dataframe["latency_variance (ms) * 100"] = 100.0 * on_dataframe["latency_variance (ms)"]
            ax = on_dataframe[["T_experiment",
                               "latency_mean (ms)",
                               "latency_variance (ms) * 100","maxrss (Mb)"
                               ]] \
                .plot(x='T_experiment', secondary_y=["maxrss (Mb)"])

            off_dataframe["maxrss (Mb)"] = off_dataframe["ru_maxrss"] / 1e3
            off_dataframe.drop(list(off_dataframe.filter(regex='ru_')), axis=1, inplace=True)
            off_dataframe["latency_variance (ms) * 100"] = 100.0 * off_dataframe["latency_variance (ms)"]
            off_dataframe[["T_experiment",
                           "latency_mean (ms)",
                           "latency_variance (ms) * 100","maxrss (Mb)"
                           ]] \
                .plot(ax=ax, x='T_experiment', secondary_y=["maxrss (Mb)"])

            plt.figtext(0.15, 0.9, off_dataframe.mean().round(4), fontsize=8,
                        horizontalalignment='left')
            plt.figtext(0.50, 0.9, on_dataframe.mean().round(4), fontsize=8,
                        horizontalalignment='left')


            if single_file is None:
                plt.savefig("performance_report.pdf",
                            bbox_inches=matplotlib.transforms.Bbox(np.array(((0, 0), (8, 8)))))
            else:
                single_file.savefig(bbox_inches=matplotlib.transforms.Bbox(np.array(((0, 0), (8, 8)))))
            plt.close('all')
    except:  # noqa: E722 I do rethrow.
        print("Could not parse files: " + str(sec_on_file) + " and " + str(sec_off_file) + ".\n")

pdf = None
if len(sys.argv) < 3 or len(sys.argv) > 4:
    print("Usage: python performance_test_file_reader.py path_to_sec_on_log_file path_to_sec_off_log_file [single_pdf_output_file]")
    sys.exit(0)

directory_1 = sys.argv[1]
directory_2 = sys.argv[2]
if len(sys.argv) == 4:
    pdf = matplotlib.backends.backend_pdf.PdfPages(sys.argv[3])

N = 15  # Number of line to skip before CSV data starts.

# If the given directory actually a file.
if os.path.isfile(directory_1) and os.path.isfile(directory_2):
    generate_report(os.path.join(os.path.dirname(directory_1), os.path.basename(directory_1)), os.path.join(os.path.dirname(directory_2), os.path.basename(directory_2)), pdf)
else:
    print("Usage: Unable to generate a report due to invalid file paths.")
    sys.exit(0)

if pdf is not None:
    pdf.close()
