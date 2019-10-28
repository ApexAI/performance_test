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

"""Provives tools to compare 2 apex performance log files."""
import csv
from datetime import datetime
import itertools
import logging
from os import listdir
from os.path import isfile
from os.path import join
import time


class ApexComparision():
    """Class to perform a comparison between 2 test log files."""

    def __init__(self, logger, ref_file, target_file, columns_ids,
                 columns_of_interest, latency_threshold=5, rss_threshold=0,
                 jitter_threshold=5, print_enable=True):
        """
        Initialize class attributes.

        :param logger (logging.logger): The logger to use
        :param ref_file (str): The reference file
        :param target_file (str): The reference file
        :param columns_ids (list): The ids of the columns present in the files
        :param columns_of_interest (list): The ids of the columns to compare
        :param lantency_threshold (float): A tolerance percentage over the
            reference latency
        :param rss_threshold (float): A tolerance percentage over the reference
            rss
        :param jitter_threshold (float): A tolerance percentage over the
            reference jitter
        :param print_enable (bool): Enable result printing and logging
        """
        self.logger = logger
        self.ref_file = ref_file
        self.target_file = target_file
        self.columns_ids = columns_ids
        self.columns_of_interest = columns_of_interest
        self.ref_results = {}
        self.target_results = {}
        self.latency_threshold = latency_threshold
        self.rss_threshold = rss_threshold
        self.jitter_threshold = jitter_threshold
        self.print_enable = print_enable
        self.WORSE = 'WORSE'
        self.SIMILAR = 'SIMILAR'
        self.BETTER = 'BETTER'

    def _read_file_results(self, file):
        """
        Extract the results from a test log file.

        :return: A dictionary containing the results grouped by columns
        """
        results = {}
        for col in self.columns_of_interest:
            results[col] = []

        with open(file, 'r') as f:
            csv_reader = csv.DictReader(f, delimiter=',',
                                        fieldnames=self.columns_ids)
            line_count = 0
            experiment_start = False
            for row in csv_reader:
                # Experiment NOT started
                if not experiment_start:
                    if '--EXPERIMENT-START---' not in row[self.columns_ids[0]]:
                        continue
                    else:
                        experiment_start = True
                # Experiment started
                else:
                    if line_count > 0:
                        for col in self.columns_of_interest:
                            results[col].append(float(row[col]))
                    line_count += 1
        return results

    def _calc_jitter(self, latencies):
        """
        Calculate the jitter of a set of latencies.

        :param latencies (list): The set of latencies

        :return: A list containing the jitters
        """
        jitter = []
        prev_i = 0
        first = True
        for i in latencies:
            if first:
                first = False
            else:
                jitter.append(abs(i - prev_i))
            prev_i = i
        return jitter

    def _get_min_latency(self, results):
        """Get the minimum latency of a set of results."""
        return min(results[self.columns_of_interest[0]])

    def _get_max_rss(self, results):
        """Get the maximum rss of a set of results."""
        return max(results[self.columns_of_interest[1]])

    def _analyse(self):
        """
        Analyse the results.

        :return: The result of the analysis:
            0 if comparison yields better results than the reference
            1 otherwise.
        """
        return_value = 0

        self.lat_analysis = self.WORSE
        self.lat_diff = self.target_latency_min - self.ref_latency_min
        self.lat_perc = self.lat_diff * 100. / self.ref_latency_min
        if self.lat_perc < 0:
            self.lat_analysis = self.BETTER
        elif self.lat_perc <= self.latency_threshold:
            self.lat_analysis = self.SIMILAR
        else:
            return_value = 1

        self.rss_analysis = self.WORSE
        self.rss_diff = self.target_max_rss - self.ref_max_rss
        self.rss_perc = self.rss_diff * 100. / self.ref_max_rss
        if self.rss_perc < 0:
            self.rss_analysis = self.BETTER
        elif self.rss_perc <= self.rss_threshold:
            self.rss_analysis = self.SIMILAR
        else:
            return_value = 1

        self.ref_jitternalysis = self.WORSE
        self.jitter_diff = self.max_target_jitter - self.max_ref_jitter
        self.jitter_perc = self.jitter_diff * 100. / self.max_ref_jitter
        if self.jitter_perc < 0:
            self.ref_jitternalysis = self.BETTER
        elif self.jitter_perc <= self.jitter_threshold:
            self.ref_jitternalysis = self.SIMILAR
        else:
            return_value = 1

        return return_value

    def _print_results(self):
        """Print and logs the results."""
        # Show results
        self.logger.info('RESULTS')
        self.logger.info('---------------------------------------------------')
        self.logger.info('REFERENCE FILE:   {}'.format(self.ref_file))
        self.logger.info(
            'Min Latency (ms): {:.5f}'.format(self.ref_latency_min)
        )
        self.logger.info('Max RSS (KB):     {}'.format(self.ref_max_rss))
        self.logger.info(
            'Max Jitter (ms):  {:.5f}'.format(self.max_ref_jitter)
        )
        self.logger.info('---------------------------------------------------')
        self.logger.info('TARGET FILE:      {}'.format(self.target_file))
        self.logger.info(
            'Min Latency (ms): {:.5f}'.format(self.target_latency_min)
        )
        self.logger.info('Max RSS (KB):     {}'.format(self.target_max_rss))
        self.logger.info(
            'Max Jitter (ms):  {:.5f}'.format(self.max_target_jitter)
        )
        self.logger.info('---------------------------------------------------')
        self.logger.info('COMPARISON:')
        self.logger.info('{} Latency (ms): {:.5f} ({:.5f} %)'.format(
            self.lat_analysis, self.lat_diff, self.lat_perc)
        )
        self.logger.info('{} RSS (KB):     {:.5f} ({:.5f} %)'.format(
            self.rss_analysis, self.rss_diff, self.rss_perc)
        )
        self.logger.info('{} Jitter (ms):  {:.5f} ({:.5f} %)'.format(
            self.ref_jitternalysis, self.jitter_diff, self.jitter_perc)
        )
        self.logger.info('---------------------------------------------------')
        self.logger.info('FINAL RESULT')
        if self.return_value == 0:
            self.logger.info('Comparison passed =)')
        else:
            self.logger.warning('Comparison failed =(')
        self.logger.info('---------------------------------------------------')
        self.logger.debug('')

    def _print_start(self):
        """Print and log the configuration."""
        self.logger.debug('')
        self.logger.info('###################################################')
        self.logger.info('       RUNNING COMPARISON WITH CONFIGURATION       ')
        self.logger.info('###################################################')
        self.logger.info('Reference file: {}'.format(self.ref_file))
        self.logger.info('Target file: {}'.format(self.target_file))
        self.logger.info(
            'Columns of interest: {}'.format(self.columns_of_interest)
        )
        self.logger.info(
            'Latency threshold: {}'.format(self.latency_threshold)
        )
        self.logger.info('RSS threshold: {}'.format(self.rss_threshold))
        self.logger.info('Jitter threshold: {}'.format(self.jitter_threshold))
        self.logger.info('Print results: {}'.format(self.print_enable))
        self.logger.info('###################################################')

    def compare(self):
        """
        Compare the files.

        :return: The result of the analysis.
            0 if comparison yields better results than the reference
            1 otherwise
        """
        self._print_start()

        self.ref_results = self._read_file_results(self.ref_file)
        self.target_results = self._read_file_results(self.target_file)

        # Get min latencies
        self.ref_latency_min = self._get_min_latency(self.ref_results)
        self.target_latency_min = self._get_min_latency(self.target_results)

        # Get max rss'
        self.ref_max_rss = self._get_max_rss(self.ref_results)
        self.target_max_rss = self._get_max_rss(self.target_results)

        # Calculate Jitter
        self.ref_jitter = self._calc_jitter(
            self.ref_results[self.columns_of_interest[2]]
        )
        self.target_jitter = self._calc_jitter(
            self.target_results[self.columns_of_interest[2]]
        )
        self.max_ref_jitter = max(self.ref_jitter)
        self.max_target_jitter = max(self.target_jitter)

        self.return_value = self._analyse()
        if (self.print_enable):
            self._print_results()

        return self.return_value


def bool_cast(string):
    """
    Cast a string to a boolean.

    :param string (str): The string to cast. Supported values are:
            - yes, y, true, y, 1, no, n, false, f, 0. Both lower and upper case
    :return: The boolean representation of the string.
    :raise: ValueError if the argument is not a string or
        if it is not supported.
    """
    # Check whether input is a string
    if not isinstance(string, str):
        raise ValueError
    # Convert to lower case
    string = string.lower()
    true_tuple = ('yes', 'y', 'true', 't', '1')
    false_tuple = ('no', 'n', 'false', 'f', '0')

    if string not in true_tuple and string not in false_tuple:
        raise ValueError
    else:
        return string in true_tuple


def print_help_tree(logger, script_name):
    """
    Print and logs script help.

    :param logger (logging.logger): The logger to use.
    :param script_name (str): The name of the script to show.
    """
    logger.debug(
        'USAGE: python3 {} <ref_dir> <target_dir> [OPTIONS]'.format(
            script_name
        )
    )
    logger.debug('  Allowed options:')
    logger.debug('    --latency_threshold arg (=5)')
    logger.debug('    --rss_threshold arg (=0)')
    logger.debug('    --jitter_threshold arg (=5)')
    logger.debug('    --print_results arg (=True)')
    logger.debug('')


def print_help(logger, script_name):
    """
    Print and logs script help.

    :param logger (logging.logger): The logger to use.
    :param script_name (str): The name of the script to show.
    """
    logger.debug(
        'USAGE: python3 {} <ref_file> <target_file> [OPTIONS]'.format(
            script_name
        )
    )
    logger.debug('  Allowed options:')
    logger.debug('    --latency_threshold arg (=5)')
    logger.debug('    --rss_threshold arg (=0)')
    logger.debug('    --jitter_threshold arg (=5)')
    logger.debug('    --print_results arg (=True)')
    logger.debug('')


def get_logger():
    """Get a logger with handlers for file and terminal output."""
    ts = time.time()
    file_name = datetime.fromtimestamp(ts).strftime('%Y-%m-%dT%H.%M.%S')

    # Create a custom logger
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)

    # Create handlers
    c_handler = logging.StreamHandler()
    f_handler = logging.FileHandler('comparison_{}.log'.format(file_name))
    c_handler.setLevel(logging.DEBUG)
    f_handler.setLevel(logging.INFO)

    # Create formatters and add it to handlers
    c_format = logging.Formatter('[%(asctime)s] %(message)s')
    f_format = logging.Formatter('[%(asctime)s][%(levelname)s] %(message)s')
    c_handler.setFormatter(c_format)
    f_handler.setFormatter(f_format)

    # Add handlers to the logger
    logger.addHandler(c_handler)
    logger.addHandler(f_handler)
    return logger


def parse_arguments(logger, arguments):
    """
    Parse the script arguments for Apex Comparisons.

    :param logger (logging.logger): The logger to use.
    :param arguments (list): The script arguments.

    :return:
        A dictionary containing the script arguments.
        None if there are wrong arguments.

    """
    # Supported ptional arguments
    valid_args = [
        '--latency_threshold',
        '--rss_threshold',
        '--jitter_threshold',
        '--print_results'
    ]

    # Initialize optional arguments to default values
    latency_threshold = 5
    rss_threshold = 0
    jitter_threshold = 5
    print_results = True

    # Get arguments
    script_name = arguments[0]

    number_of_arguments = len(arguments)
    # Wrong number of arguments
    if number_of_arguments < 3 or number_of_arguments > 11:
        logger.error('Wrong number of arguments')
        return None
    # Parse arguments
    else:
        # Get directories
        ref_dir = arguments[1]
        if ref_dir.endswith('/'):
            ref_dir = ref_dir[:-1]  # Remove last frontslash

        target_dir = arguments[2]
        if target_dir.endswith('/'):
            target_dir = target_dir[:-1]  # Remove last frontslash

        # Loop optional arguments
        i = 3
        while i < number_of_arguments:
            argument = arguments[i]
            # Check if argument is valid
            if argument in valid_args:
                # Check that there is a next argument
                if number_of_arguments <= i + 1:
                    logger.error('Cannot find value for "{}"'.format(argument))
                    return None
                # Check if value is latency_threshold
                elif argument == valid_args[0]:
                    # Cast to int
                    try:
                        latency_threshold = int(arguments[i + 1])
                    except ValueError:
                        logger.error('Incorrect argument in {}'.format(
                            valid_args[0])
                        )
                        return None
                    # Check if value is a percentage
                    if latency_threshold >= 0 and latency_threshold <= 100:
                        i += 1
                    else:
                        logger.error('Incorrect argument in {}'.format(
                            valid_args[0])
                        )
                        return None
                # Check if value is rss_threshold
                elif argument == valid_args[1]:
                    # Cast to int
                    try:
                        rss_threshold = int(arguments[i + 1])
                    except ValueError:
                        logger.inerrorfo('Incorrect argument in {}'.format(
                            valid_args[1])
                        )
                        return None
                    # Check if value is a percentage
                    if rss_threshold >= 0 and rss_threshold <= 100:
                        i += 1
                    else:
                        logger.error('Incorrect argument in {}'.format(
                            valid_args[1])
                        )
                        return None
                # Check if value is jitter_threshold
                elif argument == valid_args[2]:
                    try:
                        jitter_threshold = int(arguments[i + 1])
                    except ValueError:
                        logger.error('Incorrect argument in {}'.format(
                            valid_args[2])
                        )
                        return None
                    # Check if value is a percentage
                    if jitter_threshold >= 0 and jitter_threshold <= 100:
                        i += 1
                    else:
                        logger.error('Incorrect argument in {}'.format(
                            valid_args[2])
                        )
                        return None
                # Check if value is print_results
                elif argument == valid_args[3]:
                    # Check if value is boolean
                    try:
                        print_results = bool_cast(arguments[i + 1])
                        i += 1
                    except ValueError:
                        logger.error('Incorrect argument in {}'.format(
                            valid_args[3])
                        )
                        return None
            else:
                return None

            # Increase loop counter
            i += 1

    ret = {
        'script_name': script_name,
        'ref_dir': ref_dir,
        'target_dir': target_dir,
        'latency_threshold': latency_threshold,
        'rss_threshold': rss_threshold,
        'jitter_threshold': jitter_threshold,
        'print_results': print_results,
    }

    return ret


def get_column_ids():
    """Get a list of log file column ids."""
    columns_ids = [
        'T_experiment', 'T_loop', 'received', 'sent', 'lost', 'relative_loss',
        'data_received', 'latency_min (ms)', 'latency_max (ms)',
        'latency_mean (ms)', 'latency_variance (ms)', 'pub_loop_res_min (ms)',
        'pub_loop_res_max (ms)', 'pub_loop_res_mean (ms)',
        'pub_loop_res_variance (ms)', 'sub_loop_res_min (ms)',
        'sub_loop_res_max (ms)', 'sub_loop_res_mean (ms)',
        'sub_loop_res_variance (ms)', 'ru_utime', 'ru_stime', 'ru_maxrss',
        'ru_ixrss', 'ru_idrss', 'ru_isrss', 'ru_minflt', 'ru_majflt',
        'ru_nswap', 'ru_inblock', 'ru_oublock', 'ru_msgsnd', 'ru_msgrcv',
        'ru_nsignals', 'ru_nvcsw', 'ru_nivcsw'
    ]
    return columns_ids


def get_subdirectories(rates, num_subs):
    """
    Get the list of expected subdirectories in every directory.

    The expected subdirectories are obtaines from the Cartesian product
    of the elements of rates and num_subs.

    :param rates (list): The sending rates
    :param num_subs (list): All the different number of subscribers.

    :return:
        A list containing the path of all the subdirectories.

    """
    # Get subdirectory names
    combinations = list(itertools.product(rates, num_subs))
    sub_dirs = []
    for combination in combinations:
        sub_dirs.append('rate_{}/subs_{}'.format(
            combination[0], combination[1])
        )
    return sub_dirs


def get_file_names(directory, sub_dirs):
    """
    Get the file name of all the files in every subdirectory of directory.

    :param directory (str): The main directory.
    :param sub_dirs (list): A list of subdirectories to look for files.

    :return: A list of file names in the form:
        'directory/subdirectory/filename'
    """
    files = {}
    for sub_dir in sub_dirs:
        dir_name = '{}/{}'.format(directory, sub_dir)

        files_l = [f for f in listdir(dir_name) if isfile(join(dir_name, f))]
        files_l.sort()
        files[sub_dir] = files_l

    files_list = []
    for sub_dir in sub_dirs:
        for f in files[sub_dir]:
            files_list.append('{}/{}/{}'.format(directory, sub_dir, f))

    return files_list
