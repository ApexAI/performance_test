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

"""Script to compare result trees of apex experiments."""
import argparse
import sys

import ApexComparison


if __name__ == '__main__':
    # Get logger configured logger
    logger = ApexComparison.get_logger()
    # Get arguments
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'reference_directory',
        type=ApexComparison.directory_type,
        help='Reference directory for comparison'
    )
    parser.add_argument(
        'target_directory',
        type=ApexComparison.directory_type,
        help='Target directory for comparison'
    )
    parser.add_argument(
        '-l',
        '--latency_threshold',
        type=ApexComparison.percentage_float,
        help='A tolerance percentage over the reference latency',
        required=False,
        default=5
    )
    parser.add_argument(
        '-r', '--rss_threshold',
        type=ApexComparison.percentage_float,
        help='A tolerance percentage over the reference RSS',
        required=False,
        default=5
    )
    parser.add_argument(
        '-j',
        '--jitter_threshold',
        type=ApexComparison.percentage_float,
        help='A tolerance percentage over the reference jitter',
        required=False,
        default=5
    )
    parser.add_argument(
        '-p',
        '--print_results',
        type=ApexComparison.bool_type,
        help='Enable result printing and logging',
        required=False,
        default=True
    )
    args = parser.parse_args()

    ref_dir = args.reference_directory
    target_dir = args.target_directory
    latency_threshold = args.latency_threshold
    rss_threshold = args.rss_threshold
    jitter_threshold = args.jitter_threshold
    print_results = args.print_results

    # Log files column names
    columns_ids = ApexComparison.get_column_ids()
    # Columns we are interested in
    columns_of_interest = ['latency_min (ms)', 'ru_maxrss', 'latency_max (ms)']
    if not all(elem in columns_ids for elem in columns_of_interest):
        logger.error('The columns of interest do not exist in the log files')
        exit(1)

    rates = ['20', '50', '1000']  # Expected sampling rates
    num_subs = ['1', '3', '10']  # Expected number of subscribers

    # Get subdirectory names
    sub_dirs = ApexComparison.get_subdirectories(
        rates=rates, num_subs=num_subs
    )

    # Get full path file names
    ref_files_list = ApexComparison.get_file_names(ref_dir, sub_dirs)
    target_files_list = ApexComparison.get_file_names(target_dir, sub_dirs)

    # Check whether there are the same number or ref and target files
    if len(ref_files_list) != len(target_files_list):
        msg = ('Error: There are NOT the same number of ' +
               'REFERENCE and TARGET files')
        logger.error(msg)
        logger.error('')
        sys.exit(1)

    # Begin comparisons
    exit_value = 0  # An exit value of 0 means signifies
    for i in range(0, len(ref_files_list)):
        try:
            ref_file = ref_files_list[i]
            target_file = target_files_list[i]

            # Create comparisson instance
            comparision = ApexComparison.ApexComparision(
                logger,
                ref_file,
                target_file,
                columns_ids,
                columns_of_interest,
                latency_threshold=latency_threshold,
                rss_threshold=rss_threshold,
                jitter_threshold=jitter_threshold,
                print_enable=print_results
            )
            if comparision.compare() is False:
                exit_value = 1  # An exit value of 1 signifies failure
        except Exception as e:
            logger.warning('Exception occurred: {}'.format(e))
            exit_value = 1  # An exit value of 1 signifies failure

    logger.info('Script exit value: {}'.format(exit_value))
    sys.exit(exit_value)
