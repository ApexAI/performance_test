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

#ifndef COMMUNICATION_ABSTRACTIONS__ROS2_CALLBACK_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__ROS2_CALLBACK_COMMUNICATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <atomic>

#include "../experiment_configuration/topics.hpp"

#include "ros2_communicator.hpp"
#include "resource_manager.hpp"

namespace performance_test
{
/// Communication plugin for ROS 2 using ROS 2 callbacks for the subscription side.
template<class Topic>
class ROS2CallbackCommunicator : public ROS2Communicator<Topic>
{
public:
  /// The data type to publish and subscribe to.
  using DataType = typename ROS2Communicator<Topic>::DataType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit ROS2CallbackCommunicator(SpinLock & lock)
  : ROS2Communicator<Topic>(lock),
    m_subscription(nullptr)
  {
    m_executor.add_node(this->m_node);
  }

  /// Reads received data from ROS 2 using callbacks
  void update_subscription() override
  {
    if (!m_subscription) {
      m_subscription = this->m_node->template create_subscription<DataType>(
        Topic::topic_name() + this->m_ec.sub_topic_postfix(), this->m_ROS2QOSAdapter,
        [this](const typename DataType::SharedPtr data) {this->callback(data);});
    }
    this->lock();
    m_executor.spin_once(std::chrono::milliseconds(100));
    this->unlock();
  }

private:
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::shared_ptr<::rclcpp::Subscription<DataType, std::allocator<void>>> m_subscription;
};


}  // namespace performance_test
#endif  // COMMUNICATION_ABSTRACTIONS__ROS2_CALLBACK_COMMUNICATOR_HPP_
