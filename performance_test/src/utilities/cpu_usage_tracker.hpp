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
#include <utility>
#include <string>
#include <vector>

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
#include <odb/core.hxx>
#endif

namespace performance_test
{

struct CpuInfo
{
  CpuInfo(uint32_t cpu_cores, float_t cpu_usage)
  : m_cpu_cores(cpu_cores), m_cpu_usage(cpu_usage) {}

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  CpuInfo() {}
#endif

  uint32_t cpu_cores() const
  {
    return m_cpu_cores;
  }

  float_t cpu_usage() const
  {
    return m_cpu_usage;
  }

private:
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  friend class odb::access;
#endif
  uint32_t m_cpu_cores;
  float_t m_cpu_usage;
};

///  Calculate the CPU usage for the running experiment in the performance test
class CPUsageTracker
{
public:
  CPUsageTracker()
  : proc_stat_file("/proc/stat", std::ifstream::in) {}

  /**
* \brief Computes the CPU usage % as (process_active_time/total_cpu_time)*100
* This throws an exception if the total time is not updated
*
*/
  CpuInfo get_cpu_usage()
  {
    // get process cpu times from http://man7.org/linux/man-pages/man2/times.2.html
    const auto retval = times(&m_process_cpu_times);
    if (retval == -1) {
      throw std::runtime_error("Could not get process CPU times.");
    }

    // compute total process time
    const int64_t process_active_time = m_process_cpu_times.tms_cstime + m_process_cpu_times
      .tms_cutime +
      m_process_cpu_times.tms_stime +
      m_process_cpu_times.tms_utime;

    // get total CPU times from http://man7.org/linux/man-pages/man5/proc.5.html
    const auto bundle = read_total_cpu_times();
    const int64_t cpu_total_time = bundle.first;
    float_t cpu_usage_local{};

    // active time and total time are valid
    if ((process_active_time > 0) &&
      (cpu_total_time > 0) &&
      (cpu_total_time >= process_active_time))
    {
      // The CPU times should always be incrementing
      if ((process_active_time >= m_prev_active_time) &&
        (cpu_total_time >= m_prev_total_time))
      {
        // The diff should never be negative
        const int64_t active_time_diff = process_active_time - m_prev_active_time;
        const int64_t total_time_diff = cpu_total_time - m_prev_total_time;
        // compute CPU usage
        if (active_time_diff != 0) {  // if the active time diff is non zero
          if (total_time_diff != 0) {  // if the total time diff is non zero
            cpu_usage_local =
              (static_cast<float_t>(active_time_diff) /
              static_cast<float_t>(total_time_diff)) * 100.0F;
          } else {  // when the total time diff is 0
            // this should never happen
            throw std::runtime_error("get_cpu_usage: CPU times are not updated!");
          }
        } else {
          if (total_time_diff != 0) {
            // when the active time diff is 0 and total time diff is non 0
            // CPU if completely idle (ideal case)
            cpu_usage_local = 0.0F;
          } else {  // when the total time diff is 0
            // this should never happen
            throw std::runtime_error("get_cpu_usage: CPU times are not updated!");
          }
        }

        // update previous times
        m_prev_active_time = process_active_time;
        m_prev_total_time = cpu_total_time;

        CpuInfo cpu_info_local{bundle.second, cpu_usage_local};
        return cpu_info_local;

      } else {
        throw std::invalid_argument("get_cpu_usage: time travelled backwards.");
      }
    } else {
      throw std::invalid_argument("get_cpu_usage: process_active_time > cpu_total_time");
    }
  }

private:
  /**
* \brief Class for specifying CPU Time states.
*
*  The class CpuTimeState is used for specifying the index from which to parse
*  the specific CPU time from `/proc/stat` output.
*
*  e.g. user nice system idle iowait  irq  softirq steal guest guest_nice
*  cpu  4705 356  584    3699   23    23     0       0     0          0
*/
  class CpuTimeState
  {
public:
    /**
   * \brief Overloaded istream operator to parse contents of `/proc/stat` file
   */
    explicit CpuTimeState(std::istream & stream)
    {
      stream >> m_cpu_label >>
      m_cs_user >>
      m_cs_nice >>
      m_cs_system >>
      m_cs_idle >>
      m_cs_iowait >>
      m_cs_irq >>
      m_cs_softirq >>
      m_cs_steal;
    }

    /**
    * \brief Calculates the total cpu time since boot
    */
    int64_t compute_total_cpu_time() const
    {
      return m_cs_user + m_cs_nice + m_cs_system + m_cs_idle + m_cs_iowait + m_cs_irq +
             m_cs_softirq + m_cs_steal;
    }


    int64_t cs_user() const
    {
      return m_cs_user;
    }

    int64_t cs_nice() const
    {
      return m_cs_nice;
    }

    int64_t cs_system() const
    {
      return m_cs_system;
    }

    int64_t cs_idle() const
    {
      return m_cs_idle;
    }

    int64_t cs_iowait() const
    {
      return m_cs_iowait;
    }

    int64_t cs_irq() const
    {
      return m_cs_irq;
    }

    int64_t cs_softirq() const
    {
      return m_cs_softirq;
    }

    int64_t cs_steal() const
    {
      return m_cs_steal;
    }

    int64_t cs_guest() const
    {
      return m_cs_guest;
    }

    int64_t cs_guest_nice() const
    {
      return m_cs_guest_nice;
    }

    std::string cpu_label() const
    {
      return m_cpu_label;
    }

private:
    /// Time spent in user mode
    int64_t m_cs_user{};
    /// Time spent in user mode with low priority ("nice")
    int64_t m_cs_nice{};
    /// Time spent in system mode
    int64_t m_cs_system{};
    /// Time spent in idle task
    int64_t m_cs_idle{};
    /// Time waiting for I/O to complete
    int64_t m_cs_iowait{};
    /// Time spent in servicing interrupts
    int64_t m_cs_irq{};
    /// Time spent in servicing soft irq's
    int64_t m_cs_softirq{};
    /// Time spent in other OS (stolen time)
    int64_t m_cs_steal{};
    /// Time spent for running virtual CPU for guest OS
    int64_t m_cs_guest{};
    /// Time spent for running virtual CPU with nice prio for guest OS
    int64_t m_cs_guest_nice{};
    /// Cpu label (cpu 0 ,cpu 1 ..)
    std::string m_cpu_label{};
  };

  int64_t m_prev_active_time{};
  int64_t m_prev_total_time{};
  tms m_process_cpu_times;
  std::ifstream proc_stat_file;

  /**
  * \brief Vector for storing cpu information
  *
  * This vector is used to store the cpu label (cpu 0, cpu1 ..etc) along with the cpu ticks in
  * cpu_time_array for different CPU usages as defined by CpuTimeState struct above.
  *
  */
  std::vector<CpuTimeState> m_entries{};

  /**
 * \brief Reads the total cpu time.
 *
 *  This function parses the output of /proc/stat which contains information on cpu time spent
 *  since boot. It computes the total CPU time since boot as well as counts the number of CPU
 *  cores and returns a bundle of both.
 *
 */
  std::pair<int64_t, uint32_t> read_total_cpu_times()
  {
    std::string line;
    const std::string cpu_string("cpu");
    const std::size_t cpu_string_len = cpu_string.size();
    int64_t cpu_total_time {};
    uint32_t num_cpu_cores {};
    while (std::getline(proc_stat_file, line)) {
      // cpu stats line found
      if (!line.compare(0, cpu_string_len, cpu_string)) {
        std::istringstream ss(line);
        // store entry
        CpuTimeState entry(ss);
        // count the number of cpu cores
        if (entry.cpu_label().size() > cpu_string_len) {
          ++num_cpu_cores;
        }
        // read times
        m_entries.push_back(entry);
      }
    }
    // compute cpu total time
    // Guest and Guest_nice are not included in the total time calculation since they are
    // already accounted in user and nice.

    cpu_total_time = m_entries[0].compute_total_cpu_time();

    // Clear entries vector for next iteration
    m_entries.clear();
    // Reset the eof file flag and move file pointer to beginning for next read
    proc_stat_file.clear();
    proc_stat_file.seekg(0, std::ifstream::beg);
    std::pair<int64_t, uint32_t> bundle {cpu_total_time, num_cpu_cores};
    return bundle;
  }
};
}  // namespace performance_test

#endif  // UTILITIES__CPU_USAGE_TRACKER_HPP_
