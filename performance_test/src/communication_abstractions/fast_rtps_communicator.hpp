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

#ifndef COMMUNICATION_ABSTRACTIONS__FAST_RTPS_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__FAST_RTPS_COMMUNICATOR_HPP_

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/Domain.h>
#include <fastrtps/utils/eClock.h>

#include <atomic>

#include "communicator.hpp"
#include "resource_manager.hpp"
#include "../experiment_configuration/topics.hpp"
#include "../experiment_configuration/qos_abstraction.hpp"

namespace performance_test
{

/// Translates abstract QOS settings to specific QOS settings for FastRTPS.
class FastRTPSQOSAdapter
{
public:
  /**
   * \brief Constructs the QOS adapter.
   * \param qos The abstract QOS settings the adapter should use to derive the implementation specific QOS settings.
   */
  explicit FastRTPSQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}

  /// Returns derived FastRTPS reliability setting from the stored abstract QOS setting.
  inline eprosima::fastrtps::ReliabilityQosPolicyKind reliability() const
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      return eprosima::fastrtps::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      return eprosima::fastrtps::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }
  /// Returns derived FastRTPS durability setting from the stored abstract QOS setting.
  inline eprosima::fastrtps::DurabilityQosPolicyKind durability() const
  {
    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      return eprosima::fastrtps::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      return eprosima::fastrtps::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }
  /// Returns derived FastRTPS history policy setting from the stored abstract QOS setting.
  inline eprosima::fastrtps::HistoryQosPolicyKind history_kind() const
  {
    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      return eprosima::fastrtps::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      return eprosima::fastrtps::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }
  /// Returns derived FastRTPS history depth setting from the stored abstract QOS setting.
  int32_t history_depth() const
  {
    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      return static_cast<int32_t>(m_qos.history_depth);
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      // Keep all, keeps all. No depth required, but setting to dummy value.
      return 1;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }
  /// Returns the publish mode policy from the stored abstract QOS setting.
  inline eprosima::fastrtps::PublishModeQosPolicyKind publish_mode() const
  {
    if (m_qos.sync_pubsub) {
      return eprosima::fastrtps::PublishModeQosPolicyKind::SYNCHRONOUS_PUBLISH_MODE;
    } else {
      return eprosima::fastrtps::PublishModeQosPolicyKind::ASYNCHRONOUS_PUBLISH_MODE;
    }
  }

private:
  const QOSAbstraction m_qos;
};

/**
 * \brief Communication plugin for FastRTPS.
 * \tparam Topic The topic type to use.
 *
 * The code in there is derived from
 * https://github.com/eProsima/Fast-RTPS/tree/master/examples/C%2B%2B/HelloWorldExample.
 */
template<class Topic>
class FastRTPSCommunicator : public Communicator
{
public:
  /// The topic type to use.
  using TopicType = typename Topic::EprosimaTopicType;
  /// The data type to publish and subscribe to.
  using DataType = typename Topic::EprosimaType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit FastRTPSCommunicator(SpinLock & lock)
  : Communicator(lock),
    m_publisher(nullptr),
    m_subscriber(nullptr),
    m_topic_type(new TopicType())
  {
    m_participant = ResourceManager::get().fastrtps_participant();
    if (m_ec.use_single_participant()) {
      if (!s_type_registered) {
        eprosima::fastrtps::Domain::registerType(m_participant, m_topic_type);
        s_type_registered = true;
      }
    } else {
      eprosima::fastrtps::Domain::registerType(m_participant, m_topic_type);
    }
  }
  /**
   * \brief Publishes the provided data.
   *
   *  The first time this function is called it also creates the data writer.
   *  Further it updates all internal counters while running.
   * \param data The data to publish.
   * \param time The time to fill into the data field.
   */
  void publish(DataType & data, const std::chrono::duration<double> time)
  {
    if (!m_publisher) {
      const FastRTPSQOSAdapter qos(m_ec.qos());

      eprosima::fastrtps::PublisherAttributes wparam;
      wparam.topic.topicKind = NO_KEY;
      wparam.topic.topicDataType = m_topic_type->getName();
      wparam.topic.topicName = Topic::topic_name();
      wparam.topic.historyQos.kind = qos.history_kind();
      wparam.topic.historyQos.depth = qos.history_depth();
      wparam.topic.resourceLimitsQos.max_samples = 100;
      wparam.topic.resourceLimitsQos.allocated_samples = 100;
      wparam.times.heartbeatPeriod.seconds = 2;
      wparam.times.heartbeatPeriod.fraction = 200 * 1000 * 1000;
      wparam.qos.m_reliability.kind = qos.reliability();
      wparam.qos.m_durability.kind = qos.durability();
      wparam.qos.m_publishMode.kind = qos.publish_mode();
      m_publisher = Domain::createPublisher(m_participant, wparam);
    }
    lock();
    data.time_(time.count());
    data.id_(next_sample_id());
    increment_sent();  // We increment before publishing so we don't have to lock twice.
    unlock();
    m_publisher->write(static_cast<void *>(&data));
  }
  /**
   * \brief Reads received data from DDS.
   *
   * In detail this function:
   * * Reads samples from DDS.
   * * Verifies that the data arrived in the right order, chronologically and also consistent with the publishing order.
   * * Counts received and lost samples.
   * * Calculates the latency of the samples received and updates the statistics accordingly.
   */
  void update_subscription()
  {
    if (!m_subscriber) {
      const FastRTPSQOSAdapter qos(m_ec.qos());

      eprosima::fastrtps::SubscriberAttributes rparam;
      rparam.topic.topicKind = NO_KEY;
      rparam.topic.topicDataType = m_topic_type->getName();
      rparam.topic.topicName = Topic::topic_name();
      rparam.topic.historyQos.kind = qos.history_kind();
      rparam.topic.historyQos.depth = qos.history_depth();
      rparam.topic.resourceLimitsQos.max_samples = 100;
      rparam.topic.resourceLimitsQos.allocated_samples = 100;
      rparam.qos.m_reliability.kind = qos.reliability();
      rparam.qos.m_durability.kind = qos.durability();
      m_subscriber = Domain::createSubscriber(m_participant, rparam);
    }

    if (!m_ec.no_waitset()) {
      m_subscriber->waitForUnreadMessage();
    }
    lock();
    while (m_subscriber->takeNextData(static_cast<void *>(&m_data), &m_info)) {
      if (m_info.sampleKind == ALIVE) {
        if (m_prev_timestamp >= m_data.time_()) {
          throw std::runtime_error("Data consistency violated. Received sample with not strictly "
                  "older timestamp. Time diff: " + std::to_string(
                    m_data.time_() - m_prev_timestamp) + " Data Time: " +
                  std::to_string(m_data.time_())
          );
        }

        m_prev_timestamp = m_data.time_();
        update_lost_samples_counter(m_data.id_());
        add_latency_to_statistics(m_data.time_());
        increment_received();
      }
    }
    unlock();
  }

  /// Returns the data received in bytes.
  std::size_t data_received()
  {
    return num_received_samples() * sizeof(DataType);
  }

private:
  eprosima::fastrtps::Participant * m_participant;
  eprosima::fastrtps::Publisher * m_publisher;
  eprosima::fastrtps::Subscriber * m_subscriber;

  static bool s_type_registered;
  eprosima::fastrtps::SampleInfo_t m_info;

  TopicType * m_topic_type;
  DataType m_data;
};

template<class Topic>
bool FastRTPSCommunicator<Topic>::s_type_registered = false;

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__FAST_RTPS_COMMUNICATOR_HPP_
