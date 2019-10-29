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

#ifndef COMMUNICATION_ABSTRACTIONS__OPENDDS_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__OPENDDS_COMMUNICATOR_HPP_

#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/WaitSet.h>

#include "communicator.hpp"
#include "resource_manager.hpp"

#define BLOCK_SEC 10
#define BLOCK_NANO_SEC 0

namespace performance_test
{

/**
 * \brief Translates abstract QOS settings to specific QOS settings for OpenDDS data writers and readers.
 *
 * The reason that this class is constructed like this is that one usually gets a partially specified QOS from the topic
 * or similar higher level entity and just changes some settings from these.
 */

class OpenDdsQOSAdapter
{
public:
  /**
   * \brief Constructs the QOS adapter.
   * \param qos The abstract QOS settings the adapter should use to derive the implementation specific QOS settings.
   */
  explicit OpenDdsQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}
  /**
   * \brief  Applies the abstract QOS to an existing QOS leaving unsupported values as they were.
   * \tparam ConnextDDSMicroQos The type of the QOS setting, for example data reader or data writer QOS.
   * \param qos The QOS settings to fill supported values in.
   */

  void apply_dr(DDS::DataReaderQos& qos)
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
      qos.reliability.max_blocking_time.sec = BLOCK_SEC;
      qos.reliability.max_blocking_time.nanosec = BLOCK_NANO_SEC; 
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      qos.durability.kind = DDS::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      qos.durability.kind = DDS::TRANSIENT_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      qos.history.kind = DDS::KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.kind = DDS::KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.depth = static_cast<decltype(qos.history.depth)>(m_qos.history_depth);
    } else {
      // Keep all, keeps all. No depth required.
    }
  }

  void apply_dw(DDS::DataWriterQos& qos)
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
      qos.reliability.max_blocking_time.sec = BLOCK_SEC;
      qos.reliability.max_blocking_time.nanosec = BLOCK_NANO_SEC; 
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      qos.durability.kind = DDS::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      qos.durability.kind = DDS::TRANSIENT_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      qos.history.kind = DDS::KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.kind = DDS::KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.depth = static_cast<decltype(qos.history.depth)>(m_qos.history_depth);
    } else {
      // Keep all, keeps all. No depth required.
    }
  }

private:
  const QOSAbstraction m_qos;
};

/**
 * \brief The plugin for Connext DDS Micro.
 * \tparam Topic The topic type to use.
 *
 * The code in here is derived from the C++ example in the Connext DDS Micro installation folder.
 */
template<class Topic>
class OpenDDSCommunicator : public Communicator
{
public:
  /// The data type to use.
  using DataType = typename Topic::OpenDDSTopicType;
  /// The type of the data writer.
  using DataWriterType = typename Topic::OpenDDSDataWriterType;
  /// The type of the data reader.
  using DataReaderType = typename Topic::OpenDDSDataReaderType;
  /// The type of a sequence of data.
  using DataTypeSeq = typename Topic::OpenDDSDataTypeSeq;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit OpenDDSCommunicator(SpinLock & lock)
  : Communicator(lock),
    m_datawriter(nullptr),
    m_datareader(nullptr),
    m_typed_datareader(nullptr)
  {

    m_participant = ResourceManager::get().opendds_participant();
    register_topic();
  }

  /**
   * \brief Publishes the provided data.
   *
   *  The first time this function is called it also creates the data writer.
   *  Further it updates all internal counters while running.
   * \param data The data to publish.
   * \param time The time to fill into the data field.
   */
  void publish(DataType & data, const std::chrono::nanoseconds time)
  {
    if (m_datawriter == nullptr) {
      DDS::Publisher_ptr publisher;
      DDS::DataWriterQos dw_qos;
      ResourceManager::get().opendds_publisher(publisher, dw_qos);

      dw_qos.resource_limits.max_samples = 32;
      dw_qos.resource_limits.max_samples_per_instance = 32;
      dw_qos.resource_limits.max_instances = 1;

      OpenDdsQOSAdapter qos_adapter(m_ec.qos());
      qos_adapter.apply_dw(dw_qos);

      m_datawriter = publisher->create_datawriter(m_topic,
          dw_qos, nullptr, OpenDDS::DCPS::DEFAULT_STATUS_MASK);
      if (CORBA::is_nil(m_datawriter)) {
        throw std::runtime_error("Could not create datawriter");
      }

      m_typed_datawriter = DataWriterType::_narrow(m_datawriter);
      if (CORBA::is_nil(m_typed_datawriter)) {
        throw std::runtime_error("failed datawriter narrow");
      }
    }
    lock();
    data.time_ = time.count();
    data.id_ = next_sample_id();
    increment_sent();  // We increment before publishing so we don't have to lock twice.
    unlock();
    auto retcode = m_typed_datawriter->write(data, DDS::HANDLE_NIL);
    if (retcode != DDS::RETCODE_OK) {
      throw std::runtime_error("Failed to write to sample");
    }
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
    if (CORBA::is_nil(m_datareader)) {
      DDS::Subscriber_ptr subscriber = nullptr;
      DDS::DataReaderQos dr_qos;
      ResourceManager::get().opendds_subscriber(subscriber, dr_qos);

      dr_qos.resource_limits.max_samples = 32;
      dr_qos.resource_limits.max_instances = 10;
      dr_qos.resource_limits.max_samples_per_instance = 32;
      /* if there are more remote writers, you need to increase these limits */
      
      OpenDdsQOSAdapter qos_adapter(m_ec.qos());
      qos_adapter.apply_dr(dr_qos);

      /* Only DDS_DATA_AVAILABLE_STATUS supported currently */
      m_datareader = subscriber->create_datareader(
        m_topic,
        dr_qos,
        nullptr,
        OpenDDS::DCPS::DEFAULT_STATUS_MASK);

      if (CORBA::is_nil(m_datareader)) {
        throw std::runtime_error("datareader == nullptr");
      }

      m_condition = m_datareader->get_statuscondition();
      m_condition->set_enabled_statuses(DDS::DATA_AVAILABLE_STATUS);
      m_waitset.attach_condition(m_condition);

      m_typed_datareader = DataReaderType::_narrow(m_datareader);
      if (m_typed_datareader == nullptr) {
        throw std::runtime_error("m_typed_datareader == nullptr");
      }

      //Again, RTI trick. No wonder
      //if (!m_condition_seq.ensure_length(2, 2)) {
      //  throw std::runtime_error("Error ensuring length of active_conditions_seq.");
      //}
    }

    if (!m_ec.no_waitset()) {
      DDS::Duration_t wait_timeout = {15, 0};
      m_waitset.wait(m_condition_seq, wait_timeout);
    }

    auto ret = m_typed_datareader->take(m_data_seq, m_sample_info_seq, DDS::LENGTH_UNLIMITED,
        DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE,
        DDS::ANY_INSTANCE_STATE);
    if (ret == DDS::RETCODE_OK) {
      lock();
      for (decltype(m_data_seq.length()) j = 0; j < m_data_seq.length(); ++j) {
        const auto & data = m_data_seq[j];
        if (m_sample_info_seq[j].valid_data) {
          if (m_prev_timestamp >= data.time_) {
            throw std::runtime_error(
                    "Data consistency violated. Received sample with not strictly older timestamp. "
                    "Time diff: " + std::to_string(
                      data.time_ - m_prev_timestamp) +
                    " Data Time: " +
                    std::to_string(data.time_)
            );
          }
          m_prev_timestamp = data.time_;
          update_lost_samples_counter(data.id_);
          add_latency_to_statistics(data.time_);
          increment_received();
        }
      }
      unlock();

      if (m_ec.roundtrip_mode() == ExperimentConfiguration::RoundTripMode::RELAY) {
        throw std::runtime_error("Round trip mode is not implemented for OpenDDS!");
      }

      m_typed_datareader->return_loan(m_data_seq,
        m_sample_info_seq);
    }
  }

  /// Returns the data received in bytes.
  std::size_t data_received()
  {
    return num_received_samples() * sizeof(DataType);
  }

private:
  /// Registers a topic to the participant. It makes sure that each topic is only registered once.
  void register_topic()
  {
    if (CORBA::is_nil(m_topic)) {

      DDS::ReturnCode_t retcode = Topic::get_type_support()->register_type(m_participant,Topic::topic_name().c_str());
      if (retcode != DDS::RETCODE_OK) {
        throw std::runtime_error("failed to register type");
      }
      m_topic = m_participant->create_topic(
        Topic::topic_name().c_str(),
        Topic::topic_name().c_str(),
        TOPIC_QOS_DEFAULT,
        nullptr,
        OpenDDS::DCPS::DEFAULT_STATUS_MASK);
      if (CORBA::is_nil(m_topic)) {
        throw std::runtime_error("topic == nullptr");
      }
    }
  }

  DDS::DomainParticipant_ptr m_participant;

  DDS::DataWriter_ptr m_datawriter;
  DDS::DataReader_ptr m_datareader;

  DDS::WaitSet m_waitset;
  DDS::StatusCondition_ptr m_condition;
  DDS::ConditionSeq m_condition_seq;

  DataReaderType * m_typed_datareader;
  DataWriterType * m_typed_datawriter;

  DataTypeSeq m_data_seq;
  DDS::SampleInfoSeq m_sample_info_seq;
  static DDS::Topic_ptr m_topic;
};

template<class Topic>
DDS::Topic_ptr OpenDDSCommunicator<Topic>::m_topic = nullptr;

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__CONNEXT_DDS_MICRO_COMMUNICATOR_HPP_
