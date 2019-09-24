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

#ifndef COMMUNICATION_ABSTRACTIONS__RESOURCE_MANAGER_HPP_
#define COMMUNICATION_ABSTRACTIONS__RESOURCE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
  #include <fastrtps/participant/Participant.h>
  #include <fastrtps/attributes/ParticipantAttributes.h>
  #include <fastrtps/xmlparser/XMLProfileManager.h>
  #include <fastrtps/Domain.h>
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  #include <rti_me_cpp.hxx>
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  #include <dds/dds.h>
#endif

#include <cstdlib>
#include <memory>
#include <mutex>

#include "../experiment_configuration/experiment_configuration.hpp"

namespace performance_test
{

/// Stores and manages global resources for the communication plugins.
class ResourceManager
{
public:
  // Standard C++11 singleton pattern.
  /// Singleton instance getter.
  static ResourceManager & get()
  {
    static ResourceManager instance;

    return instance;
  }

  ResourceManager(ResourceManager const &) = delete;
  ResourceManager(ResourceManager &&) = delete;
  ResourceManager & operator=(ResourceManager const &) = delete;
  ResourceManager & operator=(ResourceManager &&) = delete;

  /// Returns the ROS 2 node.
  std::shared_ptr<rclcpp::Node> ros2_node() const;

  /// Returns true if a single participant is used.
  bool is_using_single_participant() const;

#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
  /// Returns FastRTPS participant.
  eprosima::fastrtps::Participant * fastrtps_participant() const;
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  /// Returns Connext DDS Micro participant.
  DDSDomainParticipant * connext_DDS_micro_participant() const;

  /**
   * \brief Creates a new Connext DDS Micro publisher.
   * \param publisher Will be overwritten with the created publisher.
   * \param dw_qos Will be overwritten with the default QOS from the created publisher.
   */
  void connext_dds_micro_publisher(DDSPublisher * & publisher, DDS_DataWriterQos & dw_qos) const;

  /**
   * \brief Creates a new Connext DDS Micro subscriber.
   * \param subscriber Will be overwritten with the created subscriber.
   * \param dr_qos Will be overwritten with the default QOS from the created subscriber.
   */
  void connext_dds_micro_subscriber(DDSSubscriber * & subscriber, DDS_DataReaderQos & dr_qos) const;
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  /// Returns Cyclone DDS participant.
  dds_entity_t cyclonedds_participant() const;
#endif

private:
  ResourceManager()
  : m_ec(ExperimentConfiguration::get()),
    m_node(nullptr)
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
    , m_fastrtps_participant(nullptr)
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
    , m_connext_dds_micro_participant(nullptr)
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
    , m_cyclonedds_participant(0)
#endif
  {}

  const ExperimentConfiguration & m_ec;

  mutable std::shared_ptr<rclcpp::Node> m_node;

#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
  mutable eprosima::fastrtps::Participant * m_fastrtps_participant;
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  mutable DDSDomainParticipant * m_connext_dds_micro_participant;
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  mutable dds_entity_t m_cyclonedds_participant;
#endif

  mutable std::mutex m_global_mutex;
};

}  // namespace performance_test
#endif  // COMMUNICATION_ABSTRACTIONS__RESOURCE_MANAGER_HPP_
