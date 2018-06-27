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

#ifndef DATA_RUNNING__DATA_RUNNER_BASE_HPP_
#define DATA_RUNNING__DATA_RUNNER_BASE_HPP_

#include <boost/core/noncopyable.hpp>

#ifdef MEMORY_TOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <osrf_testing_tools_cpp/scope_exit.hpp>
#endif

#include "../utilities/statistics_tracker.hpp"
#include "../experiment_configuration/experiment_configuration.hpp"

namespace performance_test
{

/// Indicates what functionality an object should exhibit.
enum class RunType
{
  /// Indicates to sending/publish data.
  PUBLISHER,
  /// Indicates to receive/subscribe to data.
  SUBSCRIBER
};

/// Interface for generic object which effectively executes the experiment and collects the
/// experiment results.
class DataRunnerBase : boost::noncopyable
{
public:
  DataRunnerBase()
  : m_ec(ExperimentConfiguration::get())
  {
    if (m_ec.check_memory()) {
#ifdef MEMORY_TOOLS_ENABLED
      osrf_testing_tools_cpp::memory_tools::initialize();

      auto on_unexpected_memory =
              [](osrf_testing_tools_cpp::memory_tools::MemoryToolsService &service) {
                std::cerr << "unexpected malloc";
                // this will cause a backtrace to be printed for each unexpected memory operations
                service.print_backtrace();
              };
      osrf_testing_tools_cpp::memory_tools::on_unexpected_calloc(on_unexpected_memory);
      osrf_testing_tools_cpp::memory_tools::on_unexpected_free(on_unexpected_memory);
      osrf_testing_tools_cpp::memory_tools::on_unexpected_malloc(on_unexpected_memory);
      osrf_testing_tools_cpp::memory_tools::on_unexpected_realloc(on_unexpected_memory);

      osrf_testing_tools_cpp::memory_tools::enable_monitoring_in_all_threads();
#else
      throw std::runtime_error("OSRF memory tools is not installed. Memory check must be disabled.");
#endif
    }
  }
  virtual ~DataRunnerBase() = default;
  /// Sum of the received samples per second.
  virtual uint64_t sum_received_samples() const = 0;
  /// Sum of the lost samples per second.
  virtual uint64_t sum_lost_samples() const = 0;
  /// Sum of the data received in bytes per second.
  virtual std::size_t sum_data_received() const = 0;
  /// Sum of the samples sent per second.
  virtual uint64_t sum_sent_samples() const = 0;

  /// Statistics about the latency of received samples.
  virtual StatisticsTracker latency_statistics() const = 0;
  /// Statistics about how much time every loop iteration had left over.
  virtual StatisticsTracker loop_time_reserve_statistics() const = 0;

  /// Resets all the stored metrics and replaces them with current ones from the running threads.
  virtual void sync_reset() = 0;

protected:
  /// A reference to the experiment configuration.
  const ExperimentConfiguration & m_ec;
};

}  // namespace performance_test
#endif  // DATA_RUNNING__DATA_RUNNER_BASE_HPP_
