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

#ifndef UTILITIES__CPU_USAGE_TRACKER_HPP_
#define UTILITIES__CPU_USAGE_TRACKER_HPP_


#include <sys/times.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <string>
#include <vector>

namespace performance_test
{
///  Calculate the CPU load for the running experiment in the performance test
class CPUsageTracker
{
private:
  uint32_t m_cpu_cores;
  inline static int64_t m_prev_active_time = 0;
  inline static int64_t m_prev_total_time = 0;

public:
  CPUsageTracker()
  : m_cpu_cores(),
    m_cpu_total_time(),
    m_cpu_load() {}

  enum class CpuTimeState : uint8_t
  {
    ///< Field for time spent in user mode
    CS_USER = 0U,
    ///< Field for time spent in user mode with low priority ("nice")
    CS_NICE = 1U,
    ///< Field for time spent in system mode
    CS_SYSTEM = 2U,
    ///< Field for time spent in idle task
    CS_IDLE = 3U,
    ///< Field for time waiting for I/O to complete
    CS_IOWAIT = 4U,
    ///< Field for time spent in servicing interrupts
    CS_IRQ = 5U,
    ///< Field for time spent in servicing soft irq's
    CS_SOFTIRQ = 6U,
    ///< Field for time spent in other OS (stolen time)
    CS_STEAL = 7U,
    ///< Field for time spent for running virtual CPU for guest OS
    CS_GUEST = 8U,
    ///< Field for time spent for running virtual CPU with nice prio for guest OS
    CS_GUEST_NICE = 9U,
    ///< number of states
    CPU_TIME_STATES_NUM = 10U
  };  // enum CpuTimeState

  typedef struct cpu_info
  {
    std::string cpu_label;
    size_t cpu_time_array[static_cast<uint8_t>(CpuTimeState::CPU_TIME_STATES_NUM)];
  } cpu_info_obj;

  int64_t m_cpu_total_time;
  std::vector<CPUsageTracker::cpu_info> entries;
  tms m_process_times;
  float_t m_cpu_load;

  void read_cpu_times(std::vector<cpu_info> & entries)
  {
    std::ifstream proc_stat_file("/proc/stat");
    std::string line;
    const std::string cpu_string("cpu");
    const std::size_t cpu_string_len = cpu_string.size();

    while (std::getline(proc_stat_file, line)) {
      // cpu stats line found
      if (!line.compare(0, cpu_string_len, cpu_string)) {
        std::istringstream ss(line);

        // store entry
        entries.emplace_back(cpu_info_obj());
        cpu_info_obj & entry = entries.back();

        // read cpu label
        ss >> entry.cpu_label;

        // count the number of cpu cores
        if (entry.cpu_label.size() > cpu_string_len) {
          ++m_cpu_cores;
        }

        // read times
        for (uint8_t i = 0U; i < static_cast<uint8_t>(CpuTimeState::CPU_TIME_STATES_NUM); ++i) {
          ss >> entry.cpu_time_array[i];
        }
      }
    }

    // compute cpu total time
    // Guest and Guest_nice are not included in the total time calculation since, they are
    // already accounted in user and nice.

    m_cpu_total_time = (entries[0].cpu_time_array[static_cast<uint8_t>(CpuTimeState::CS_USER)] +
      entries[0].cpu_time_array[static_cast<uint8_t>(CpuTimeState::CS_NICE)] +
      entries[0].cpu_time_array[static_cast<uint8_t>(CpuTimeState::CS_SYSTEM)] +
      entries[0].cpu_time_array[static_cast<uint8_t>(CpuTimeState::CS_IDLE)] +
      entries[0].cpu_time_array[static_cast<uint8_t>(CpuTimeState::CS_IOWAIT)] +
      entries[0].cpu_time_array[static_cast<uint8_t>(CpuTimeState::CS_IRQ)] +
      entries[0].cpu_time_array[static_cast<uint8_t>(CpuTimeState::CS_SOFTIRQ)] +
      entries[0].cpu_time_array[static_cast<uint8_t>(CpuTimeState::CS_STEAL)]);
  }

  void get_load(
    const int64_t active_time,
    const int64_t total_time)
  {
    // active time and total time are valid
    if ((active_time > 0) &&
      (total_time > 0) &&
      (total_time >= active_time))
    {
      // The CPU times should always be incrementing
      if ((active_time >= CPUsageTracker::m_prev_active_time) &&
        (total_time >= CPUsageTracker::m_prev_total_time))
      {
        // The diff should never be negative
        const int64_t active_time_diff = active_time - CPUsageTracker::m_prev_active_time;
        const int64_t total_time_diff = total_time - CPUsageTracker::m_prev_total_time;
        // compute CPU load
        if (active_time_diff != 0) {  // if the active time diff is non zero
          if (total_time_diff != 0) {  // if the total time diff is non zero
            m_cpu_load =
              (static_cast<float_t>(active_time_diff) /
              static_cast<float_t>(total_time_diff)) * 100.0F;
          } else {  // when the total time diff is 0
            // this should never happen
            throw std::runtime_error("get_load: CPU times are not updated!");
          }
        } else {
          if (total_time_diff != 0) {
            // when the active time diff is 0 and total time diff is non 0
            // CPU if completely idle (ideal case)
            m_cpu_load = 0.0F;
          } else {  // when the total time diff is 0
            // this should never happen
            throw std::runtime_error("get_load: CPU times are not updated!");
          }
        }

        // update previous times
        CPUsageTracker::m_prev_active_time = active_time;
        CPUsageTracker::m_prev_total_time = total_time;

      } else {
        throw std::invalid_argument("get_load: time travelled backwards.");
      }
    } else {
      throw std::invalid_argument("get_load: active time > total time");
    }
  }
};
}  // namespace performance_test

#endif  // UTILITIES__CPU_USAGE_TRACKER_HPP_
