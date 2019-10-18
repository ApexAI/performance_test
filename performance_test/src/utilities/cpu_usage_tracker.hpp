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

struct CpuInfo
{
  uint32_t m_cpu_cores = 0;
  float_t m_cpu_usage = 0.0;
};

///  Calculate the CPU usage for the running experiment in the performance test
class CPUsageTracker
{
public:
  CPUsageTracker()
  : m_cpu_total_time(0),
    m_prev_active_time(0),
    m_prev_total_time(0),
    proc_stat_file("/proc/stat", std::ifstream::in) {}


  /**
* \brief Computes the CPU usage % as (process_active_time/total_cpu_time)*100
* This throws an exception if the total time is not updated
*
*/
  CpuInfo get_cpu_usage()
  {
    // get process cpu times from http://man7.org/linux/man-pages/man2/times.2.html
    auto retval = times(&m_process_cpu_times);
    if (retval == -1) {
      throw std::runtime_error("Could not get process CPU times.");
    }

    // compute total process time
    const int64_t process_active_time = m_process_cpu_times.tms_cstime + m_process_cpu_times
      .tms_cutime +
      m_process_cpu_times.tms_stime + m_process_cpu_times.tms_utime;

    // get total CPU times from http://man7.org/linux/man-pages/man5/proc.5.html
    m_cpu_total_time = read_total_cpu_times();
    const int64_t cpu_total_time = m_cpu_total_time;

    // Create local cpu info object to return
    CpuInfo cpu_info_local;
    cpu_info_local.m_cpu_cores = m_cpu_info.m_cpu_cores;

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
            cpu_info_local.m_cpu_usage =
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
            cpu_info_local.m_cpu_usage = 0.0F;
          } else {  // when the total time diff is 0
            // this should never happen
            throw std::runtime_error("get_cpu_usage: CPU times are not updated!");
          }
        }

        // update previous times
        m_prev_active_time = process_active_time;
        m_prev_total_time = cpu_total_time;

        return cpu_info_local;

      } else {
        throw std::invalid_argument("get_cpu_usage: time travelled backwards.");
      }
    } else {
      throw std::invalid_argument("get_cpu_usage: process_active_time > cpu_total_time");
    }
  }

  /// Returns the CPU usage of the current process
  CpuInfo cpu_info() const
  {
    return m_cpu_info;
  }

private:
  CpuInfo m_cpu_info;
  int64_t m_cpu_total_time;
  int64_t m_prev_active_time;
  int64_t m_prev_total_time;
  tms m_process_cpu_times;
  std::ifstream proc_stat_file;

  /**
 * \brief Struct for specifying CPU Time states.
 *
 *  The struct CpuTimeState is used for specifying the index from which to parse
 *  the specific CPU time from `/proc/stat` output.
 *
 *  e.g. user nice system idle iowait  irq  softirq steal guest guest_nice
 *  cpu  4705 356  584    3699   23    23     0       0     0          0
 */
  struct CpuTimeState
  {
    ///< Field for time spent in user mode
    int64_t CS_USER = {};
    ///< Field for time spent in user mode with low priority ("nice")
    int64_t CS_NICE = {};
    ///< Field for time spent in system mode
    int64_t CS_SYSTEM = {};
    ///< Field for time spent in idle task
    int64_t CS_IDLE = {};
    ///< Field for time waiting for I/O to complete
    int64_t CS_IOWAIT = {};
    ///< Field for time spent in servicing interrupts
    int64_t CS_IRQ = {};
    ///< Field for time spent in servicing soft irq's
    int64_t CS_SOFTIRQ = {};
    ///< Field for time spent in other OS (stolen time)
    int64_t CS_STEAL = {};
    ///< Field for time spent for running virtual CPU for guest OS
    int64_t CS_GUEST = {};
    ///< Field for time spent for running virtual CPU with nice prio for guest OS
    int64_t CS_GUEST_NICE = {};

    std::string cpu_label;

    /**
  * \brief Calculates the total cpu time since boot
  */
    int64_t compute_total_cpu_time() const
    {
      return CS_USER + CS_NICE + CS_SYSTEM + CS_IDLE + CS_IOWAIT + CS_IRQ + CS_SOFTIRQ + CS_STEAL;
    }
    /**
   * \brief Overloaded istream operator to parse contents of `/proc/stat` file
   */
    friend std::istream & operator>>(std::istream & stream, CpuTimeState & cpuTimeState)
    {
      stream >> cpuTimeState.cpu_label >> cpuTimeState.CS_USER >> cpuTimeState.CS_NICE >>
      cpuTimeState.CS_SYSTEM >> cpuTimeState.CS_IDLE >> cpuTimeState.CS_IOWAIT >> cpuTimeState
      .CS_IRQ >> cpuTimeState.CS_SOFTIRQ >> cpuTimeState.CS_STEAL;
      return stream;
    }
  };

  /**
 * \brief Vector for storing cpu information
 *
 * This vector is used to store the cpu label (cpu 0, cpu1 ..etc) along with the cpu ticks in
 * cpu_time_array for different CPU usages as defined by CpuTimeState struct above.
 *
 */
  std::vector<CpuTimeState> m_entries;

  /**
 * \brief Reads the total cpu time.
 *
 *  This function parses the output of /proc/stat which contains information on cpu time spent
 *  since boot. It computes the total CPU time since boot and updates the m_cpu_total_time.
 *
 */
  int64_t read_total_cpu_times()
  {
    std::string line;
    const std::string cpu_string("cpu");
    const std::size_t cpu_string_len = cpu_string.size();
    int64_t cpu_total_time = {};
    while (std::getline(proc_stat_file, line)) {
      // cpu stats line found
      if (!line.compare(0, cpu_string_len, cpu_string)) {
        std::istringstream ss(line);
        // store entry
        CpuTimeState entry;
        ss >> entry;
        // count the number of cpu cores
        if (entry.cpu_label.size() > cpu_string_len) {
          ++m_cpu_info.m_cpu_cores;
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

    return cpu_total_time;
  }
};
}  // namespace performance_test

#endif  // UTILITIES__CPU_USAGE_TRACKER_HPP_
