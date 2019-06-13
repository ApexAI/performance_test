"""Script to compare results of 2 files of apex experiments."""
import ApexComparison
import itertools
import os
import sys

from os.path import isfile


if __name__ == "__main__":
    # Get logger configured logger
    logger = ApexComparison.get_logger()
    # Get arguments 
    arguments = ApexComparison.parse_arguments(logger=logger, arguments=sys.argv)
    if not arguments:
        ApexComparison.print_help(logger, sys.argv[0])
        exit(1)

    script_name = arguments['script_name']
    ref_file = arguments['ref_dir']
    target_file = arguments['target_dir']
    latency_threshold = arguments['latency_threshold']
    rss_threshold = arguments['rss_threshold']
    jitter_threshold = arguments['jitter_threshold']
    print_results = arguments['print_results']

    # Check if the files exist
    if not isfile(ref_file):
        logger.error("The reference file does not exists")
        exit(1)
    if not isfile(target_file):
        logger.error("The reference file does not exists")
        exit(1)

    # Log files column names
    columns_ids = ApexComparison.get_column_ids()
    # Columns we are interested in
    columns_of_interest = ["latency_min (ms)", "ru_maxrss", "latency_max (ms)"]
    if not all(elem in columns_ids for elem in columns_of_interest):
        logger.error("The columns of interest do not exist in the log files")
        exit(1)

    rates = ['20', '50', '1000']  # Expected sampling rates
    num_subs = ['1', '3', '10']  # Expected number of subscribers

    # Begin comparisons
    exit_value = 0
    try:
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
        ret = comparision.compare()
        if ret > 0:
            exit_value = ret
    except Exception as e:
        logger.warning("Exception occurred: {}".format(e))
        exit_value = 1

    logger.info("Script exit value: {}".format(exit_value))
    sys.exit(exit_value)
