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

#!/usr/bin/env python

# flake8: noqa This file is for plotting data. Its dependencies are not necessarily on the CI.
import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
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
            # ru_maxrss is in KB: http://man7.org/linux/man-pages/man2/getrusage.2.html. Converting to Mb
            dataframe["maxrss (Mb)"] = dataframe["ru_maxrss"] / 1e3
            dataframe.drop(list(dataframe.filter(regex='ru_')), axis=1, inplace=True)
            dataframe["latency_variance (ms) * 100"] = 100.0 * dataframe["latency_variance (ms)"]
            dataframe[["T_experiment",
                       "latency_min (ms)",
                       "latency_max (ms)",
                       "latency_mean (ms)",
                       "latency_variance (ms) * 100",
                        "maxrss (Mb)"]] \
                .plot(x='T_experiment', secondary_y=["maxrss (Mb)"])

            plt.figtext(0.0, 1.0, ''.join(head), fontsize=8, horizontalalignment='left')
            plt.figtext(0.65, 0.9, dataframe.mean().round(4), fontsize=8,
                        horizontalalignment='left')

            if single_file is None:
                plt.savefig(file + ".pdf",
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
N = 15  # Number of line to skip before CSV data starts.

# If the given directory actually a file.
os.path.isfile(directory)
parse_file(os.path.join(os.path.dirname(directory), os.path.basename(directory)), pdf)

for root, subdirs, files in os.walk(directory):
    for file in files:
        parse_file(os.path.join(root, file), pdf)

if pdf is not None:
    pdf.close()
