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

#include "resource_manager.hpp"

#include <memory>
#include <string>

namespace performance_test
{

std::shared_ptr<rclcpp::Node> ResourceManager::ros2_node() const
{
  /* Temporarely commented out until ROS2 waitsets are available. As of now every ROS2 thread needs a node in the
   * current architecture.
   */

//    std::lock_guard<std::mutex> lock(m_global_mutex);
//    if(!m_node)
//    {
//      m_node = rclcpp::Node::make_shared("performance_test", "", m_ec.use_ros_shm());
//    }
//    return m_node;

  std::string rand_str;
  // if security is enabled
  if (m_ec.is_with_security()) {
    static uint32_t id = 0;
    rand_str = std::to_string(id++);
  } else {
    rand_str = std::to_string(std::rand());
  }

  auto options = rclcpp::NodeOptions()
    .use_intra_process_comms(m_ec.use_ros_shm());

  return rclcpp::Node::make_shared("performance_test" + rand_str, options);
}

#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
eprosima::fastrtps::Participant * ResourceManager::fastrtps_participant() const
{
  std::lock_guard<std::mutex> lock(m_global_mutex);

  eprosima::fastrtps::Participant * result = nullptr;
  eprosima::fastrtps::ParticipantAttributes PParam;

  eprosima::fastrtps::xmlparser::XMLProfileManager::loadDefaultXMLFile();
  eprosima::fastrtps::xmlparser::XMLProfileManager::getDefaultParticipantAttributes(PParam);
  PParam.rtps.sendSocketBufferSize = 1048576;
  PParam.rtps.listenSocketBufferSize = 4194304;
  PParam.rtps.builtin.use_SIMPLE_RTPSParticipantDiscoveryProtocol = true;
  PParam.rtps.builtin.use_SIMPLE_EndpointDiscoveryProtocol = true;
  PParam.rtps.builtin.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = true;
  PParam.rtps.builtin.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
  PParam.rtps.builtin.domainId = m_ec.dds_domain_id();
  PParam.rtps.builtin.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
  PParam.rtps.setName("performance_test_fastRTPS");

  if (!m_ec.use_single_participant()) {
    result = eprosima::fastrtps::Domain::createParticipant(PParam);
  } else {
    if (!m_fastrtps_participant) {
      m_fastrtps_participant = eprosima::fastrtps::Domain::createParticipant(PParam);
    }
    result = m_fastrtps_participant;
  }
  return result;
}
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
DDSDomainParticipant * ResourceManager::connext_DDS_micro_participant() const
{
  std::lock_guard<std::mutex> lock(m_global_mutex);

  if (!m_connext_dds_micro_participant) {
    DDSDomainParticipantFactory * factory = nullptr;
    DDS_DomainParticipantFactoryQos dpf_qos;
    DDS_DomainParticipantQos dp_qos;
    DPDE_DiscoveryPluginProperty dpde_properties;
    RTRegistry * registry = nullptr;
    UDP_InterfaceFactoryProperty * udp_property = nullptr;

    OSAPI_Log_set_verbosity(OSAPI_LOG_VERBOSITY_WARNING);

    factory = DDSDomainParticipantFactory::get_instance();
    registry = factory->get_registry();

    if (!registry->register_component("wh",
      WHSMHistoryFactory::get_interface(),
      nullptr, nullptr))
    {
      throw std::runtime_error("failed to register wh");
    }

    if (!registry->register_component("rh",
      RHSMHistoryFactory::get_interface(),
      nullptr, nullptr))
    {
      throw std::runtime_error("failed to register rh");
    }

    /* Configure UDP transport's allowed interfaces */
    if (!registry->unregister(NETIO_DEFAULT_UDP_NAME, nullptr, nullptr)) {
      throw std::runtime_error("failed to unregister udp");
    }

    udp_property = new UDP_InterfaceFactoryProperty();

    /* For additional allowed interface(s), increase maximum and length, and
       set interface below:
    */
    udp_property->allow_interface.maximum(2);
    udp_property->allow_interface.length(2);

    /* loopback interface */
    *udp_property->allow_interface.get_reference(0) = DDS_String_dup("lo");

    *udp_property->allow_interface.get_reference(1) = DDS_String_dup("eth0");

    if (!registry->register_component(NETIO_DEFAULT_UDP_NAME,
      UDPInterfaceFactory::get_interface(),
      &udp_property->_parent._parent,
      nullptr))
    {
      throw std::runtime_error("failed to register udp");
    }

    factory->get_qos(dpf_qos);
    dpf_qos.entity_factory.autoenable_created_entities = DDS_BOOLEAN_FALSE;
    factory->set_qos(dpf_qos);

    auto peer = "127.0.0.1"; /* default to loopback */

    if (!registry->register_component(
        "dpde",
        DPDEDiscoveryFactory::get_interface(),
        &dpde_properties._parent,
        nullptr))
    {
      throw std::runtime_error("failed to register dpde");
    }

    if (!dp_qos.discovery.discovery.name.set_name("dpde")) {
      throw std::runtime_error("failed to set discovery plugin name");
    }

    dp_qos.discovery.initial_peers.maximum(1);
    dp_qos.discovery.initial_peers.length(1);
    *dp_qos.discovery.initial_peers.get_reference(0) = DDS_String_dup(peer);

    /* if there are more remote or local endpoints, you need to increase these limits */
    dp_qos.resource_limits.max_destination_ports = 32;
    dp_qos.resource_limits.max_receive_ports = 32;
    dp_qos.resource_limits.local_topic_allocation = 10;
    dp_qos.resource_limits.local_type_allocation = 10;
    dp_qos.resource_limits.local_reader_allocation = 10;
    dp_qos.resource_limits.local_writer_allocation = 10;
    dp_qos.resource_limits.remote_participant_allocation = 8;
    dp_qos.resource_limits.remote_reader_allocation = 8;
    dp_qos.resource_limits.remote_writer_allocation = 8;

    dp_qos.resource_limits.local_subscriber_allocation =
      static_cast<decltype(dp_qos.resource_limits.local_subscriber_allocation)>(m_ec.
      number_of_subscribers());
    dp_qos.resource_limits.local_publisher_allocation =
      static_cast<decltype(dp_qos.resource_limits.local_publisher_allocation)>(m_ec.
      number_of_publishers());

    if (m_ec.no_micro_intra()) {
      REDA_StringSeq_set_maximum(&dp_qos.transports.enabled_transports, 1);
      REDA_StringSeq_set_length(&dp_qos.transports.enabled_transports, 1);
      *REDA_StringSeq_get_reference(&dp_qos.transports.enabled_transports, 0) = REDA_String_dup(
        NETIO_DEFAULT_UDP_NAME);
      /* Use only unicast for user-data traffic. */
      REDA_StringSeq_set_maximum(&dp_qos.user_traffic.enabled_transports, 1);
      REDA_StringSeq_set_length(&dp_qos.user_traffic.enabled_transports, 1);
      *REDA_StringSeq_get_reference(&dp_qos.user_traffic.enabled_transports, 0) = REDA_String_dup(
        "_udp://");
    }

    m_connext_dds_micro_participant = factory->create_participant(
      m_ec.dds_domain_id(),
      dp_qos,
      nullptr,
      DDS_STATUS_MASK_NONE);

    if (m_connext_dds_micro_participant == nullptr) {
      throw std::runtime_error("failed to create participant");
    }

    auto ret = m_connext_dds_micro_participant->enable();
    if (ret != DDS_RETCODE_OK) {
      throw std::runtime_error("Could not enable participant.");
    }
  }
  return m_connext_dds_micro_participant;
}

void ResourceManager::connext_dds_micro_publisher(
  DDSPublisher * & publisher,
  DDS_DataWriterQos & dw_qos) const
{
  auto participant = connext_DDS_micro_participant();
  std::lock_guard<std::mutex> lock(m_global_mutex);
  publisher = participant->create_publisher(
    DDS_PUBLISHER_QOS_DEFAULT, nullptr, DDS_STATUS_MASK_NONE);
  if (publisher == nullptr) {
    throw std::runtime_error("Pulisher is nullptr");
  }
  auto retcode = publisher->get_default_datawriter_qos(dw_qos);
  if (retcode != DDS_RETCODE_OK) {
    throw std::runtime_error("Failed to get default datawriter");
  }
}

void ResourceManager::connext_dds_micro_subscriber(
  DDSSubscriber * & subscriber,
  DDS_DataReaderQos & dr_qos) const
{
  auto participant = connext_DDS_micro_participant();
  std::lock_guard<std::mutex> lock(m_global_mutex);
  subscriber = participant->create_subscriber(
    DDS_SUBSCRIBER_QOS_DEFAULT, nullptr,
    DDS_STATUS_MASK_NONE);
  if (subscriber == nullptr) {
    throw std::runtime_error("m_subscriber == nullptr");
  }
  auto retcode = subscriber->get_default_datareader_qos(dr_qos);
  if (retcode != DDS_RETCODE_OK) {
    throw std::runtime_error("failed get_default_datareader_qos");
  }
}
#endif
}  // namespace performance_test
