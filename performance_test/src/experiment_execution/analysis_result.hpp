// Copyright 2017 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EXPERIMENT_EXECUTION__ANALYSIS_RESULT_HPP_
#define EXPERIMENT_EXECUTION__ANALYSIS_RESULT_HPP_

#include <sys/time.h>
#include <sys/resource.h>
#include <cstddef>
#include <chrono>
#include <sstream>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "../utilities/statistics_tracker.hpp"

#ifdef ODB_FOR_SQL_ENABLED
  #include <odb/boost/date-time/exceptions.hxx>
  #include <odb/core.hxx>
  //#include "../experiment_configuration/experiment_configuration.hpp"
#endif

namespace performance_test
{

/// Outstream operator for timeval to seconds (double).
std::ostream & operator<<(std::ostream & stream, const timeval & e);
#ifdef ODB_FOR_SQL_ENABLED
#pragma db value(StatisticsTracker) definition
#pragma db value(rusage) definition
#pragma db value(timeval) definition

/// Represents the results of an experiment iteration.
#pragma db object no_id
class AnalysisResult
#else
class AnalysisResult
#endif
{
public:
  /**
   * \brief Constructs an result with the specified parameters.
   * \param experiment_start Time the experiment started.
   * \param loop_start  Time the loop iteration started.
   * \param num_samples_received Number of samples received during the experiment iteration.
   * \param num_samples_sent Number of samples sent during the experiment iteration.
   * \param num_samples_lost Number of samples lost during the experiment iteration.
   * \param total_data_received Total data received during the experiment iteration in bytes.
   * \param latency Latency statistics of samples received.
   * \param pub_loop_time_reserve Loop time statistics of the publisher threads.
   * \param sub_loop_time_reserve Loop time statistics of the subscriber threads.
   */
  AnalysisResult(
    const boost::posix_time::time_duration experiment_start,
    const boost::posix_time::time_duration loop_start,
    const uint64_t num_samples_received,
    const uint64_t num_samples_sent,
    const uint64_t num_samples_lost,
    const std::size_t total_data_received,
    const StatisticsTracker latency,
    const StatisticsTracker pub_loop_time_reserve,
    const StatisticsTracker sub_loop_time_reserve
  );

  /**
   * \brief Returns a header for a CVS file containing the analysis result data as a string.
   * \param pretty_print If set, inserts additional tabs to format the output nicer.
   * \param st The data seperator.
   * \return A string containing the CVS header.
   */
  static std::string csv_header(const bool pretty_print = false, std::string st = ",");

  /**
   * \brief Returns the data contained the analysis result as a string.
   * \param pretty_print If set, inserts additional tabs to format the output nicer.
   * \param st The data seperator.
   *
   * \return A string with the contained data as CSV row.
   */
  std::string to_csv_string(const bool pretty_print = false, std::string st = ",") const;

private:
#ifdef ODB_FOR_SQL_ENABLED
  friend class odb::access;
#endif



  const boost::posix_time::time_duration m_experiment_start;
  const boost::posix_time::time_duration m_loop_start;
  const uint64_t m_num_samples_received;
  const uint64_t m_num_samples_sent;
  const uint64_t m_num_samples_lost;
  const std::size_t m_total_data_received;

  const StatisticsTracker m_latency;
  const StatisticsTracker m_pub_loop_time_reserve;
  const StatisticsTracker m_sub_loop_time_reserve;

  rusage m_sys_usage;
/*
#ifdef ODB_FOR_SQL_ENABLED
#pragma db id
  unsigned long id_;
#endif

#ifdef ODB_FOR_SQL_ENABLED
#pragma db not_null
  std::shared_ptr<ExperimentConfiguration> configuration;
#endif
*/
};

}  // namespace performance_test

#endif  // EXPERIMENT_EXECUTION__ANALYSIS_RESULT_HPP_
