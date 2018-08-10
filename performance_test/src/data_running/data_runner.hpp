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

#ifndef DATA_RUNNING__DATA_RUNNER_HPP_
#define DATA_RUNNING__DATA_RUNNER_HPP_

#include "data_runner_base.hpp"
#ifdef MEMORY_TOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#endif
#include <atomic>
#include <thread>

#include "../utilities/spin_lock.hpp"

namespace performance_test
{

/// Generic class which effectively executes the experiment and collects the experiment results.
template<class TCommunicator>
class DataRunner : public DataRunnerBase
{
public:
  /**
   * \brief Constructs an object and starts the internal worker thread.
   * \param run_type Specifies which type of operation to execute.
   */
  explicit DataRunner(const RunType run_type)
  : m_com(m_lock),
    m_run(true),
    m_sum_received_samples(0),
    m_sum_lost_samples(0),
    m_sum_received_data(0),
    m_sum_sent_samples(0),
    m_last_sync(std::chrono::steady_clock::now()),
    m_run_type(run_type),
    m_thread(std::bind(&DataRunner::thread_function, this))
  {
  }

  ~DataRunner() noexcept override {
    m_run = false;
    m_thread.join();
  }

  uint64_t sum_received_samples() const override
  {
    if (m_run_type == RunType::PUBLISHER) {
      throw std::logic_error("Not available on a publisher.");
    }
    return m_sum_received_samples;
  }
  uint64_t sum_lost_samples() const override
  {
    if (m_run_type == RunType::PUBLISHER) {
      throw std::logic_error("Not available on a publisher.");
    }
    return m_sum_lost_samples;
  }
  uint64_t sum_sent_samples() const override
  {
    if (m_run_type == RunType::SUBSCRIBER) {
      throw std::logic_error("Not available on a subscriber.");
    }
    return m_sum_sent_samples;
  }
  std::size_t sum_data_received() const override
  {
    if (m_run_type == RunType::PUBLISHER) {
      throw std::logic_error("Not available on a publisher.");
    }
    return m_sum_received_data;
  }
  StatisticsTracker latency_statistics() const override
  {
    if (m_run_type == RunType::PUBLISHER) {
      throw std::logic_error("Not available on a publisher.");
    }
    return m_latency_statistics;
  }
  StatisticsTracker loop_time_reserve_statistics() const override
  {
    return m_time_reserve_statistics_store;
  }
  void sync_reset() override
  {
    namespace sc = std::chrono;
    m_lock.lock();
    sc::duration<double> iteration_duration = sc::steady_clock::now() - m_last_sync;

    if (m_run_type == RunType::PUBLISHER) {
      m_sum_sent_samples = static_cast<decltype(m_sum_sent_samples)>(
        static_cast<double>(m_com.num_sent_samples()) / iteration_duration.count());
      m_sum_lost_samples = static_cast<decltype(m_sum_lost_samples)>(
        static_cast<double>(m_com.num_lost_samples()) / iteration_duration.count());
    }
    if (m_run_type == RunType::SUBSCRIBER) {
      m_sum_received_samples = static_cast<decltype(m_sum_received_samples)>(
        static_cast<double>(m_com.num_received_samples()) / iteration_duration.count());
      m_sum_received_data = static_cast<decltype(m_sum_received_data)>(
        static_cast<double>(m_com.data_received()) / iteration_duration.count());
      m_sum_lost_samples = static_cast<decltype(m_sum_lost_samples)>(
        static_cast<double>(m_com.num_lost_samples()) / iteration_duration.count());
      m_latency_statistics = m_com.latency_statistics();
    }
    m_time_reserve_statistics_store = m_time_reserve_statistics;
    m_time_reserve_statistics = StatisticsTracker();
    m_com.reset();
    m_last_sync = sc::steady_clock::now();

    m_lock.unlock();
  }

private:
  /// The function running inside the thread doing all the work.
  void thread_function()
  {
    typename TCommunicator::DataType data;

    while (m_run) {
      const auto next_run = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(1.0 / m_ec.rate());
      if (m_run_type == RunType::PUBLISHER) {
        std::chrono::nanoseconds epoc_time =
          std::chrono::steady_clock::now().time_since_epoch();
        m_com.publish(data, epoc_time);
      }
      if (m_run_type == RunType::SUBSCRIBER) {
        m_com.update_subscription();
      }
      {
        // We track here how much time (can also be negative) was left for the loop iteration given
        // the desired loop rate.
        m_lock.lock();
        const std::chrono::duration<double> reserve = next_run - std::chrono::steady_clock::now();
        m_time_reserve_statistics.add_sample(reserve.count());
        m_lock.unlock();
      }
      if (m_ec.rate() > 0 && m_run_type == RunType::PUBLISHER) {
        std::this_thread::sleep_until(next_run);
      }

      // Enabling memory checker after the first run:
      enable_memory_tools();
    }
  }

  /// Enables the memory tool checker.
  void enable_memory_tools() {
    #ifdef MEMORY_TOOLS_ENABLED
    // Do not turn the memory tools on several times.
    if(m_memory_tools_on) {
      return;
    }

    osrf_testing_tools_cpp::memory_tools::expect_no_calloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_free_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_realloc_begin();

    m_memory_tools_on = true;
    #endif
  }
  TCommunicator m_com;
  std::atomic<bool> m_run;
  SpinLock m_lock;

  uint64_t m_sum_received_samples;
  uint64_t m_sum_lost_samples;
  std::size_t m_sum_received_data;

  std::uint64_t m_sum_sent_samples;

  StatisticsTracker m_latency_statistics;
  StatisticsTracker m_time_reserve_statistics, m_time_reserve_statistics_store;

  std::chrono::steady_clock::time_point m_last_sync;
  const RunType m_run_type;

  std::thread m_thread;

  bool m_memory_tools_on = false;
};

}  // namespace performance_test

#endif  // DATA_RUNNING__DATA_RUNNER_HPP_
