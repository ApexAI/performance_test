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

GETRUSAGE = '\\href{http://man7.org/linux/man-pages/man2/getrusage.2.html}{man getrusage}'


def generate_figures(xaxis,
                     min_latency_ydata,
                     max_latency_ydata,
                     mean_latency_ydata,
                     latency_variance_ydata,
                     maxrss_ydata,
                     minflt_ydata,
                     majflt_ydata,
                     nivcsw_ydata,
                     ):
    """
    Generate the figures for a perf_plot PDF

    :param xaxis: The time axis data for all of the plots
    """
    if not xaxis:
        return _generate_empty_figures()

    return [
        # The left figure, containing latency data
        {
            'caption': 'Latencies',
            'xlabel': 'time',
            'ylabel': 'latency (ms)',
            'traces': [
                {'name': 'min', 'x': xaxis, 'y': min_latency_ydata},
                {'name': 'max', 'x': xaxis, 'y': max_latency_ydata},
                {'name': 'mean', 'x': xaxis, 'y': mean_latency_ydata},
                {'name': 'variance * 100',
                 'x': xaxis,
                 'y': [n * 100 for n in latency_variance_ydata]},
            ],
            'xrange': get_range(5, xaxis),
            'yrange': get_range(0.5, min_latency_ydata, max_latency_ydata, mean_latency_ydata),
            'axis2': {
                'traces': [
                ],
            },
        },
        # The right figure, containing resource data, page faults, and context switches
        {
            'caption': 'Resource usage ({})'.format(GETRUSAGE),
            'xlabel': 'time',
            'ylabel': 'usage',
            'traces': [
                {'name': 'ru_minflt', 'x': xaxis, 'y': minflt_ydata},
                {'name': 'ru_majflt', 'x': xaxis, 'y': majflt_ydata},
                {'name': 'ru_nivcsw', 'x': xaxis, 'y': nivcsw_ydata},
            ],
            'xrange': get_range(5, xaxis),
            'yrange': get_range(2500, minflt_ydata, majflt_ydata, nivcsw_ydata),
            'axis2': {
                'ylabel': 'maxrss (MB)',
                'traces': [
                    {'name': 'maxrss (MB)', 'x': xaxis, 'y': maxrss_ydata},
                ],
                'xrange': get_range(5, xaxis),
                'yrange': get_range(0.5, maxrss_ydata)
            },
        },
    ]


def get_range(padding, *args):
    """
    Get a [min, max] for all of the data passed as *args.
    """
    if not isinstance(padding, tuple):
        padding = (padding, padding)

    all_data = [item for lst in args for item in lst]
    return [min(all_data) - padding[0], max(all_data) + padding[1]]


def _generate_empty_figures():

    # This will generate no plots
    return []

    # This will generate empty plots
    # return [
    #     # The left figure, containing latency data
    #     {
    #         'caption': 'Latencies',
    #         'xlabel': 'time',
    #         'ylabel': 'latency (ms)',
    #         'traces': [
    #         ],
    #         'xrange': [0, 1],
    #         'yrange': [0, 1],
    #         'axis2': {
    #             'traces': [
    #             ],
    #         },
    #     },
    #     # The right figure, containing resource data, page faults, and context switches
    #     {
    #         'caption': 'Resource usage ({})'.format(GETRUSAGE),
    #         'xlabel': 'time',
    #         'ylabel': 'usage',
    #         'traces': [
    #         ],
    #         'xrange': [0, 1],
    #         'yrange': [0, 1],
    #         'axis2': {
    #             'ylabel': 'maxrss (MB)',
    #             'traces': [
    #             ],
    #             'xrange': [0, 1],
    #             'yrange': [0, 100],
    #         },
    #     },
    # ]
