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

#ifndef EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_
#define EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <string>
#include <fstream>

#include "qos_abstraction.hpp"
#include "communication_mean.hpp"
#include "../utilities/rt_enabler.hpp"

#ifdef ODB_FOR_SQL_ENABLED
  #include <odb/core.hxx>
#endif


namespace performance_test
{

/**
 * \brief Represents the configuration of an experiment.
 *
 * This experiment configuration could be created from various sources. At the moment, only
 * configuration by command line arguments are supported.
 */
#ifdef ODB_FOR_SQL_ENABLED
  #pragma db value(QOSAbstraction) definition
  #pragma db object
#endif
class ExperimentConfiguration
{
public:
  // Implementing the standard C++11 singleton pattern.
  /// The singleton instance getter.
  static ExperimentConfiguration & get()
  {
    static ExperimentConfiguration instance;

    return instance;
  }

  ExperimentConfiguration(ExperimentConfiguration const &) = delete;
  ExperimentConfiguration(ExperimentConfiguration &&) = delete;

  ExperimentConfiguration & operator=(ExperimentConfiguration const &) = delete;

  ExperimentConfiguration & operator=(ExperimentConfiguration &&) = delete;

  /// Specifies the selected roundtrip mode.
  enum RoundTripMode
  {
    NONE,  /// No roundtrip. Samples are only sent from sender to reciever.
    MAIN,  /// Sends packages to the relay and receives packages from the relay.
    RELAY  /// Relays packages from MAIN back to MAIN.
  };

  /**
   * \brief Derives an experiment configuration from command line arguments.
   * \param argc The argc parameter from the main function.
   * \param argv The argv parameter from the main function.
   */
  void setup(int argc, char ** argv);
  /// Returns if the experiment configuration is setup and ready to use.
  bool is_setup() const;

  /// \returns Returns the configured mean of communication. This will throw if the experiment
  /// configuration is not set up.
  CommunicationMean com_mean() const;
  /// \returns Returns the configured DDS domain ID. This will throw if the experiment
  /// configuration is not set up.
  uint32_t dds_domain_id() const;
  /// \returns Returns the configured QOS settings. This will throw if the experiment
  /// configuration is not set up.
  QOSAbstraction qos() const;
  /// \returns Returns the configured publishing rate. This will throw if the experiment
  /// configuration is not set up.
  uint32_t rate() const;
  /// \returns Returns the chosen topic name. This will throw if the experiment configuration is
  /// not set up.
  std::string topic_name() const;
  /// \returns Returns the time the application should run until it terminates [s]. This will
  /// throw if the experiment configuration is not set up.
  std::string db_name() const;
  uint64_t max_runtime() const;
  /// \returns Returns the configured number of publishers. This will throw if the experiment
  /// configuration is not set up.
  uint32_t number_of_publishers() const;
  /// \returns Returns the configured number of subscribers. This will throw if the experiment
  /// configuration is not set up.
  uint32_t number_of_subscribers() const;
  /// \returns Returns if memory operations should be logged.
  bool check_memory() const;
  /// \returns Returns if ROS shm should be used. This will throw if the experiment
  /// configuration is not set up.
  bool use_ros_shm() const;
  /// \returns Returns if only a single participant should be used. This will throw if
  /// the experiment configuration is not set up.
  bool use_single_participant() const;
  /// \returns Returns if no waitset should be used. Then the thread loop will just spin as fast
  /// as possible. This will throw if the experiment configuration is not set up.
  bool no_waitset() const;
  /// \returns Returns if Connext DSS Micro INTRA transport should be disabled. This will throw if
  /// the experiment configuration is not set up.
  bool no_micro_intra() const;
  /// \returns Returns if security is enabled for ROS2. This will throw if the configured mean
  ///  of communication is not ROS2
  /// \returns Returns if Drivepx RT is set or not. This will throw if the experiment configuration
  /// is not set up.
  bool is_drivepx_rt() const;
  bool is_with_security() const;
  /// \returns Returns the roundtrip mode.
  RoundTripMode roundtrip_mode() const;
  /// \returns Returns the publishing topic postfix
  std::string pub_topic_postfix() const;
  /// \returns Returns the subscribing topic postfix
  std::string sub_topic_postfix() const;
  /// \returns Returns the randomly generated unique ID of the experiment. This will throw if the
  /// experiment configuration is not set up.
  boost::uuids::uuid id() const;
  /// Logs \param msg to stdout and the configured log file. This will throw if the experiment
  /// configuration is not set up.
  void log(const std::string & msg) const;
  /// The configured logfile name. This will throw if the experiment configuration is not set up.
  std::string logfile_name() const;
  /// \return Returns true if the user requested the application to exit.
  bool exit_requested() const;

private:
  ExperimentConfiguration()
  : m_id(boost::uuids::random_generator()()),
    m_is_setup(false),
    m_dds_domain_id(),
    m_rate(),
    m_max_runtime(),
    m_number_of_publishers(),
    m_number_of_subscribers(),
    m_check_memory(false),
    m_use_ros_shm(false),
    m_use_single_participant(false),
    m_no_waitset(false),
    m_no_micro_intra(false),
    m_is_drivepx_rt(false),
    m_roundtrip_mode(RoundTripMode::NONE)
  {}

#ifdef ODB_FOR_SQL_ENABLED
  friend class odb::access;
#endif

  /// Throws #std::runtime_error if the experiment is not set up.
  void check_setup() const;

  /// Generates filename from the experiment configuration and opens a file accordingly. This will
  /// throw if the experiment configuration is not set up.
  void open_file();

#ifdef ODB_FOR_SQL_ENABLED
  friend odb::access;

  // Using the GUID of the experiment as ID.
  #pragma db id
#endif
  boost::uuids::uuid m_id;

  bool m_is_setup;

  std::string m_logfile;
  std::string m_final_logfile_name;

#ifdef ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  mutable std::ofstream m_os;

  CommunicationMean m_com_mean;
  uint32_t m_dds_domain_id;

  QOSAbstraction m_qos;

  uint32_t m_rate;
  std::string m_topic_name;
  std::string m_db_name;
  uint64_t m_max_runtime;

  uint32_t m_number_of_publishers;
  uint32_t m_number_of_subscribers;
  bool m_check_memory;
  bool m_use_ros_shm;
  bool m_use_single_participant;
  bool m_no_waitset;
  bool m_no_micro_intra;
  bool m_is_drivepx_rt;
  bool m_with_security;

  RoundTripMode m_roundtrip_mode;
};

/// Outstream operator for RoundTripMode.
std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration::RoundTripMode & e);


/// Outstream operator for ExperimentConfiguration.
std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration & e);
}  // namespace performance_test

#endif  // EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_
