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
#include <boost/uuid/uuid_io.hpp>
#include <cstddef>
#include <chrono>
#include <sstream>
#include <string>

#include "../utilities/statistics_tracker.hpp"
#ifdef ODB_FOR_SQL_ENABLED
  #include <odb/core.hxx>
  #include "experiment_configuration.hpp"
#endif

namespace performance_test
{

/// Outstream operator for timeval to seconds (double).
std::ostream & operator<<(std::ostream & stream, const timeval & e);
#ifdef ODB_FOR_SQL_ENABLED
class RusageTracker
{
public:
  void setValues(rusage sys_usage) const
  {
    m_ru_utime = sys_usage.ru_utime;
    m_ru_stime = sys_usage.ru_stime;
    m_ru_maxrss = sys_usage.ru_maxrss;
    m_ru_ixrss = sys_usage.ru_ixrss;
    m_ru_idrss = sys_usage.ru_idrss;
    m_ru_isrss = sys_usage.ru_isrss;
    m_ru_minflt = sys_usage.ru_minflt;
    m_ru_majflt = sys_usage.ru_majflt;
    m_ru_nswap = sys_usage.ru_nswap;
    m_ru_inblock = sys_usage.ru_inblock;
    m_ru_oublock = sys_usage.ru_oublock;
    m_ru_msgsnd = sys_usage.ru_msgsnd;
    m_ru_msgrcv = sys_usage.ru_msgrcv;
    m_ru_nsignals = sys_usage.ru_nsignals;
    m_ru_nvcsw = sys_usage.ru_nvcsw;
    m_ru_nivcsw = sys_usage.ru_nivcsw;
  }

private:
  friend class odb::access;
  mutable struct timeval m_ru_utime = {};
  mutable struct timeval m_ru_stime = {};
  mutable uint64_t m_ru_maxrss = {};
  mutable uint64_t m_ru_ixrss = {};
  mutable uint64_t m_ru_idrss = {};
  mutable uint64_t m_ru_isrss = {};
  mutable uint64_t m_ru_minflt = {};
  mutable uint64_t m_ru_majflt = {};
  mutable uint64_t m_ru_nswap = {};
  mutable uint64_t m_ru_inblock = {};
  mutable uint64_t m_ru_oublock = {};
  mutable uint64_t m_ru_msgsnd = {};
  mutable uint64_t m_ru_msgrcv = {};
  mutable uint64_t m_ru_nsignals = {};
  mutable uint64_t m_ru_nvcsw = {};
  mutable uint64_t m_ru_nivcsw = {};
};
#pragma \
  db map type(std::chrono::nanoseconds) as(std::chrono::nanoseconds::rep) to((?).count ()) \
  from(std::chrono::nanoseconds (?))
#pragma db value(StatisticsTracker) definition
#pragma db value(RusageTracker) definition
#pragma db value(rusage) definition
#pragma db value(timeval) definition

/// Represents the results of an experiment iteration.
#pragma db object pointer(std::shared_ptr)
#endif
class AnalysisResult
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
    const std::chrono::nanoseconds experiment_start,
    const std::chrono::nanoseconds loop_start,
    const uint64_t num_samples_received,
    const uint64_t num_samples_sent,
    const uint64_t num_samples_lost,
    const std::size_t total_data_received,
    const StatisticsTracker latency,
    const StatisticsTracker pub_loop_time_reserve,
    const StatisticsTracker sub_loop_time_reserve
  );

  AnalysisResult() {}
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
   * \return A string with the contained data as CSV row.
   */
  std::string to_csv_string(const bool pretty_print = false, std::string st = ",") const;

#ifdef ODB_FOR_SQL_ENABLED
  void set_configuration_ptr(const ExperimentConfiguration * ec)
  {
    m_configuration_ptr = ec;
  }
#endif

private:
#ifdef ODB_FOR_SQL_ENABLED
  friend class odb::access;
#pragma db not_null
  const ExperimentConfiguration * m_configuration_ptr;
#pragma db id auto
  uint64_t m_id;
#endif
  const std::chrono::nanoseconds m_experiment_start = {};
  const std::chrono::nanoseconds m_loop_start = {};
  const uint64_t m_num_samples_received = {};
  const uint64_t m_num_samples_sent = {};
  const uint64_t m_num_samples_lost = {};
  const std::size_t m_total_data_received = {};

  const StatisticsTracker m_latency;
  const StatisticsTracker m_pub_loop_time_reserve;
  const StatisticsTracker m_sub_loop_time_reserve;
#ifdef ODB_FOR_SQL_ENABLED
  RusageTracker m_sys_tracker;
#pragma db transient
#endif
  rusage m_sys_usage;
};

}  // namespace performance_test

#endif  // EXPERIMENT_EXECUTION__ANALYSIS_RESULT_HPP_
