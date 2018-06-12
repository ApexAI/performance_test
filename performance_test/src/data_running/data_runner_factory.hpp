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

#ifndef DATA_RUNNING__DATA_RUNNER_FACTORY_HPP_
#define DATA_RUNNING__DATA_RUNNER_FACTORY_HPP_

#include <memory>
#include <string>

#include "data_runner_base.hpp"
#include "../experiment_configuration/communication_mean.hpp"

namespace performance_test
{

/// Factory to create a data runner.
class DataRunnerFactory
{
public:
  /**
   * \brief The effective factory getter creating the desired object.
   * \param requested_topic_name The topic name the created object should use.
   * \param com_mean The communication mean the created object should use.
   * \param run_type The run type the created object should user.
   * \return The created object subject to the provided parameters.
   */
  static std::shared_ptr<DataRunnerBase> get(
    const std::string & requested_topic_name,
    CommunicationMean com_mean,
    const RunType run_type);
};

}  // namespace performance_test

#endif  // DATA_RUNNING__DATA_RUNNER_FACTORY_HPP_
