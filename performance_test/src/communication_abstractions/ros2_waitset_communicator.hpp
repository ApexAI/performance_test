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

#ifndef COMMUNICATION_ABSTRACTIONS__ROS2_WAITSET_COMMUNICATOR_HPP
#define COMMUNICATION_ABSTRACTIONS__ROS2_WAITSET_COMMUNICATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <atomic>

#include "../experiment_configuration/topics.hpp"
#include "../experiment_configuration/qos_abstraction.hpp"

#include "ros2_communicator.hpp"
#include "resource_manager.hpp"

namespace performance_test
{

/// Communication plugin for ROS 2 using waitsets for the subscription side.
template<class Topic>
class ROS2WaitsetCommunicator : public ROS2Communicator<Topic>
{
public:
  /// The data type to publish and subscribe to.
  using DataType = typename ROS2Communicator<Topic>::DataType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit ROS2WaitsetCommunicator(SpinLock & lock)
  : ROS2Communicator<Topic>(lock),
    m_polling_subscription(nullptr) {}

  /// Reads received data from ROS 2 using waitsets
  void update_subscription() override
  {
    auto ros2QOSAdapter = this->m_ROS2QOSAdapter;
    if (!m_polling_subscription) {
      m_polling_subscription = this->m_node->template create_polling_subscription<DataType>(
        Topic::topic_name() + this->m_ec.sub_topic_postfix(), *ros2QOSAdapter);

    }
    this->lock();
    try {
      rclcpp::Waitset<> m_waitset {m_polling_subscription};
      auto wait_ret = m_waitset.wait(std::chrono::milliseconds(100));
      if (wait_ret.all()) {
        auto received_sample = m_polling_subscription->take();
        DataType tmp = received_sample.data();
        if (received_sample) {
          this->template callback(tmp);
        }
      }
    } catch (const rclcpp::TimeoutError &) {

      std::cerr << "Waitset timed out without receiving a sample." << std::endl;
    }
    this->unlock();
  }

private:
  std::shared_ptr<::rclcpp::PollingSubscription<DataType, std::allocator<void>>>
  m_polling_subscription;
};

}  // namespace performance_test

#endif //COMMUNICATION_ABSTRACTIONS__ROS2_WAITSET_COMMUNICATOR_HPP
