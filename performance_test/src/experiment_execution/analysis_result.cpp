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

#include "analysis_result.hpp"

#include <iomanip>
#include <string>
#include <iostream>

namespace performance_test
{

std::ostream & operator<<(std::ostream & stream, const timeval & e)
{
  return stream << double(e.tv_sec) + double(e.tv_usec) / 1000000.0;
}

AnalysisResult::AnalysisResult(
  const std::chrono::nanoseconds experiment_start,
  const std::chrono::nanoseconds loop_start,
  const uint64_t num_samples_received,
  const uint64_t num_samples_sent,
  const uint64_t num_samples_lost,
  const std::size_t total_data_received,
  const StatisticsTracker latency,
  const StatisticsTracker pub_loop_time_reserve,
  const StatisticsTracker sub_loop_time_reserve
)
: m_experiment_start(experiment_start),
  m_loop_start(loop_start),
  m_num_samples_received(num_samples_received),
  m_num_samples_sent(num_samples_sent),
  m_num_samples_lost(num_samples_lost),
  m_total_data_received(total_data_received),
  m_latency(latency),
  m_pub_loop_time_reserve(pub_loop_time_reserve),
  m_sub_loop_time_reserve(sub_loop_time_reserve)
{
  const auto ret = getrusage(RUSAGE_SELF, &m_sys_usage);
  m_sys_tracker = RusageTracker(m_sys_usage);
  if (ret != 0) {
    throw std::runtime_error("Could not get system resource usage.");
  }
  if (m_num_samples_received != m_latency.n()) {
    // TODO(andreas.pasternak): Commented out flaky assertion. Need to check if it actually a bug.
    /*throw std::runtime_error("Statistics result sample size does not match: "
                             + std::to_string(m_num_samples_received) + " / "
                             + std::to_string(m_latency.n()));*/
  }
}

std::string AnalysisResult::csv_header(const bool pretty_print, std::string st)
{
  if (pretty_print) {
    st += "\t";
  }

  std::stringstream ss;
  ss << "T_experiment" << st;
  ss << "T_loop" << st;
  ss << "received" << st;
  ss << "sent" << st;
  ss << "lost" << st;
  ss << "relative_loss" << st;

  ss << "data_received" << st;

  ss << "latency_min (ms)" << st;
  ss << "latency_max (ms)" << st;
  ss << "latency_mean (ms)" << st;
  ss << "latency_variance (ms)" << st;

  ss << "pub_loop_res_min (ms)" << st;
  ss << "pub_loop_res_max (ms)" << st;
  ss << "pub_loop_res_mean (ms)" << st;
  ss << "pub_loop_res_variance (ms)" << st;

  ss << "sub_loop_res_min (ms)" << st;
  ss << "sub_loop_res_max (ms)" << st;
  ss << "sub_loop_res_mean (ms)" << st;
  ss << "sub_loop_res_variance (ms)" << st;

  ss << "ru_utime" << st;
  ss << "ru_stime" << st;
  ss << "ru_maxrss" << st;
  ss << "ru_ixrss" << st;
  ss << "ru_idrss" << st;
  ss << "ru_isrss" << st;
  ss << "ru_minflt" << st;
  ss << "ru_majflt" << st;
  ss << "ru_nswap" << st;
  ss << "ru_inblock" << st;
  ss << "ru_oublock" << st;
  ss << "ru_msgsnd" << st;
  ss << "ru_msgrcv" << st;
  ss << "ru_nsignals" << st;
  ss << "ru_nvcsw" << st;
  ss << "ru_nivcsw" << st;

  return ss.str();
}

std::string AnalysisResult::to_csv_string(const bool pretty_print, std::string st) const
{
  if (pretty_print) {
    st += "\t\t";
  }

  std::stringstream ss;

  ss << std::fixed;
  ss << std::chrono::duration_cast<std::chrono::duration<float>>(m_experiment_start).count() << st;
  ss << std::chrono::duration_cast<std::chrono::duration<float>>(m_loop_start).count() << st;
  ss << std::setprecision(0);
  ss << m_num_samples_received << st;
  ss << m_num_samples_sent << st;
  ss << m_num_samples_lost << st;
  ss << std::setprecision(2);
  ss << static_cast<double>(m_num_samples_lost) / static_cast<double>(m_num_samples_sent) << st;

  ss << m_total_data_received << st;

  ss << std::setprecision(4);
  ss << std::defaultfloat;

  ss << m_latency.min() * 1000.0 << st;
  ss << m_latency.max() * 1000.0 << st;
  ss << m_latency.mean() * 1000.0 << st;
  ss << m_latency.variance() * 1000.0 << st;

  ss << m_pub_loop_time_reserve.min() * 1000.0 << st;
  ss << m_pub_loop_time_reserve.max() * 1000.0 << st;
  ss << m_pub_loop_time_reserve.mean() * 1000.0 << st;
  ss << m_pub_loop_time_reserve.variance() * 1000.0 << st;

  ss << m_sub_loop_time_reserve.min() * 1000.0 << st;
  ss << m_sub_loop_time_reserve.max() * 1000.0 << st;
  ss << m_sub_loop_time_reserve.mean() * 1000.0 << st;
  ss << m_sub_loop_time_reserve.variance() * 1000.0 << st;

  /* See http://www.gnu.org/software/libc/manual/html_node/Resource-Usage.html for a detailed explanation of the
   * output below
   */

  ss << m_sys_usage.ru_utime << st;
  ss << m_sys_usage.ru_stime << st;
  ss << m_sys_usage.ru_maxrss << st;
  ss << m_sys_usage.ru_ixrss << st;
  ss << m_sys_usage.ru_idrss << st;
  ss << m_sys_usage.ru_isrss << st;
  ss << m_sys_usage.ru_minflt << st;
  ss << m_sys_usage.ru_majflt << st;
  ss << m_sys_usage.ru_nswap << st;
  ss << m_sys_usage.ru_inblock << st;
  ss << m_sys_usage.ru_oublock << st;
  ss << m_sys_usage.ru_msgsnd << st;
  ss << m_sys_usage.ru_msgrcv << st;
  ss << m_sys_usage.ru_nsignals << st;
  ss << m_sys_usage.ru_nvcsw << st;
  ss << m_sys_usage.ru_nivcsw << st;

  return ss.str();
}

}  // namespace performance_test
