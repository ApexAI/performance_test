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

#include "experiment_configuration.hpp"

#include <boost/program_options.hpp>

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <iomanip>
#include <exception>
#include <string>

#include "../utilities/rt_enabler.hpp"
#include "topics.hpp"

namespace performance_test
{

std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration & e)
{
  if (e.is_setup()) {
    return stream <<
           "Experiment id: " << e.id() <<
           "\nLogfile name: " << e.logfile_name() <<
           "\nCommunication mean: " << e.com_mean() <<
           "\nDDS domain id: " << e.dds_domain_id() <<
           "\nQOS: " << e.qos() <<
           "\nPublishing rate: " << e.rate() <<
           "\nTopic name: " << e.topic_name() <<
           "\nMaximum runtime (sec): " << e.max_runtime() <<
           "\nNumber of publishers: " << e.number_of_publishers() <<
           "\nNumber of subscribers:" << e.number_of_subscribers() <<
           "\nMemory check enabled: " << e.check_memory() <<
           "\nUse ros SHM: " << e.use_ros_shm() <<
           "\nUse single participant: " << e.use_single_participant() <<
           "\nNot using waitset: " << e.no_waitset() <<
           "\nNot using Connext DDS Micro INTRA: " << e.no_micro_intra();
  } else {
    return stream << "ERROR: Experiment is not yet setup!";
  }
}

void ExperimentConfiguration::setup(int argc, char ** argv)
{
  namespace po = ::boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "Print usage message.")("logfile,l", po::value<std::string>(),
    "Optionally specify a logfile.")("rate,r", po::value<uint32_t>()->default_value(1000),
    "The rate data should be published. Defaults to 1000 Hz. 0 means publish as fast as possible.")(
    "communication,c", po::value<std::string>()->required(),
    "Communication plugin to use (ROS2, FastRTPS, ConnextDDSMicro)")("topic,t",
    po::value<std::string>()->required(),
    "Topic to use. Use --topic_list to get a list.")("topic_list",
    "Prints list of available topics and exits.")("dds_domain_id",
    po::value<uint32_t>()->default_value(0), "Sets the DDS domain id.")("reliable",
    "Enable reliable QOS. Default is best effort.")("transient",
    "Enable transient QOS. Default is volatile.")("keep_last",
    "Enable keep last QOS. Default is keep all.")("history_depth",
    po::value<uint32_t>()->default_value(1000),
    "Set history depth QOS. Defaults to 1000.")("disable_async.",
    "Disables asyc. pub/sub.")("max_runtime",
    po::value<uint64_t>()->default_value(0),
    "Maximum number of seconds to run before exiting. Default (0) is to run forever.")(
    "num_pub_threads,p", po::value<uint32_t>()->default_value(1),
    "Maximum number of publisher threads.")("num_sub_threads,s",
    po::value<uint32_t>()->default_value(1),
    "Maximum number of subscriber threads.")("use_ros_shm",
    "Use Ros SHM support.")("check_memory",
    "Prints backtrace of all memory operations performed by the middleware. "
    "This will slow down the application!")("use_drive_px_rt",
    "Enable RT. Only the Drive PX has the right configuration to support this.")(
    "use_single_participant",
    "Uses only one participant per process. By default every thread has its own.")("no_waitset",
    "Disables the wait set for new data. The subscriber takes as fast as possible.")(
    "no_micro_intra", "Disables the Connext DDS Micro INTRA transport.")(
    "with_security", "Enables the security with ROS2")
  ;
  po::variables_map vm;
  po::store(parse_command_line(argc, argv, desc), vm);

  try {
    if (vm.count("topic_list")) {
      for (const auto & s : topics::supported_topic_names()) {
        std::cout << s << std::endl;
      }
      // Exiting as we just print out some information and not running the application.
      exit(0);
    }

    if (vm.count("help")) {
      std::cout << desc << "\n";
      exit(0);
    }

    if (!vm.count("topic")) {
      throw std::invalid_argument("--topic is required!");
    }
    m_topic_name = vm["topic"].as<std::string>();

    if (vm.count("rate")) {
      m_rate = vm["rate"].as<uint32_t>();
    }

    if (vm.count("check_memory")) {
      m_check_memory = true;
    }

    if (vm["communication"].as<std::string>() == "ROS2") {
      m_com_mean = CommunicationMean::ROS2;
    } else if (vm["communication"].as<std::string>() == "FastRTPS") {
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
      m_com_mean = CommunicationMean::FASTRTPS;
#else
      throw std::invalid_argument(
              "You must compile with FastRTPS support to enable it as communication mean.");
#endif
    } else if (vm["communication"].as<std::string>() == "ConnextDDSMicro") {
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
      m_com_mean = CommunicationMean::CONNEXTDDSMICRO;
#else
      throw std::invalid_argument(
              "You must compile with ConnextDDSMicro support to enable it as communication mean.");
#endif
    }

    if (vm.count("dds_domain_id")) {
      m_dds_domain_id = vm["dds_domain_id"].as<uint32_t>();
      if (m_com_mean == CommunicationMean::ROS2 && m_dds_domain_id != 0) {
        throw std::invalid_argument("ROS 2 does not support setting the domain ID.");
      }
    }

    if (vm.count("reliable")) {
      m_qos.reliability = QOSAbstraction::Reliability::RELIABLE;
    } else {
      m_qos.reliability = QOSAbstraction::Reliability::BEST_EFFORT;
    }
    if (vm.count("transient")) {
      m_qos.durability = QOSAbstraction::Durability::TRANSIENT_LOCAL;
    } else {
      m_qos.durability = QOSAbstraction::Durability::VOLATILE;
    }
    if (vm.count("keep_last")) {
      m_qos.history_kind = QOSAbstraction::HistoryKind::KEEP_LAST;
    } else {
      m_qos.history_kind = QOSAbstraction::HistoryKind::KEEP_ALL;
    }
    if (vm.count("history_depth")) {
      m_qos.history_depth = vm["history_depth"].as<uint32_t>();
    }
    if (vm.count("disable_async")) {
      if (m_com_mean == CommunicationMean::ROS2) {
        throw std::invalid_argument("ROS 2 does not support disabling async. publishing.");
      }
      m_qos.sync_pubsub = vm["disable_async"].as<uint32_t>();
    }

    m_max_runtime = vm["max_runtime"].as<uint64_t>();

    m_number_of_publishers = vm["num_pub_threads"].as<uint32_t>();
    m_number_of_subscribers = vm["num_sub_threads"].as<uint32_t>();

    if (m_number_of_publishers > 1) {
      throw std::invalid_argument("More than one publisher is not supported at the moment");
    }

    m_use_ros_shm = false;
    if (vm.count("use_ros_shm")) {
      if (m_com_mean != CommunicationMean::ROS2) {
        throw std::invalid_argument("Must use ROS2 for this option for ROS2 SHM!");
      }
      m_use_ros_shm = true;
    }
    if (vm.count("use_drive_px_rt")) {
      // Magic numbers for the drive px.
      proc_rt_init(62, 5);
    }
    m_use_single_participant = false;
    if (vm.count("use_single_participant")) {
      if (m_com_mean == CommunicationMean::ROS2) {
        throw std::invalid_argument("ROS2 does not support single participant mode!");
      } else {
        m_use_single_participant = true;
      }
    }
    m_no_waitset = false;
    if (vm.count("no_waitset")) {
      if (m_com_mean == CommunicationMean::ROS2) {
        throw std::invalid_argument("ROS2 does not support disabling the waitset!");
      } else {
        m_no_waitset = true;
      }
    }

    m_no_micro_intra = false;
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
    if (vm.count("no_micro_intra")) {
      if (m_com_mean != CommunicationMean::CONNEXTDDSMICRO) {
        throw std::invalid_argument(
                "Only Connext DDS Micro supports INTRA Transport and can therefore disable it.");
      } else {
        m_no_micro_intra = true;
      }
    }
#endif
    m_with_security = false;
    if (vm.count("with_security")) {
      if (m_com_mean != CommunicationMean::ROS2) {
        throw std::invalid_argument(
                "Only ROS2 supports security!");
      } else {
        m_with_security = true;
      }
    }
    m_is_setup = true;

    // Logfile needs to be opened at the end, as the experiment configuration influences the
    // filename.
    if (vm.count("logfile")) {
      m_logfile = vm["logfile"].as<std::string>();
      open_file();
    }
  } catch (const std::exception & e) {
    std::cerr << "ERROR: ";
    std::cerr << e.what() << std::endl;
    std::cerr << "Check below on how to use this tool:" << std::endl;
    std::cerr << desc << "\n";
    exit(1);
  }
}

bool ExperimentConfiguration::is_setup() const
{
  return m_is_setup;
}
CommunicationMean ExperimentConfiguration::com_mean() const
{
  check_setup();
  return m_com_mean;
}
uint32_t ExperimentConfiguration::dds_domain_id() const
{
  check_setup();
  return m_dds_domain_id;
}
QOSAbstraction ExperimentConfiguration::qos() const
{
  check_setup();
  return m_qos;
}
uint32_t ExperimentConfiguration::rate() const
{
  check_setup();
  return m_rate;
}
std::string ExperimentConfiguration::topic_name() const
{
  check_setup();
  return m_topic_name;
}
uint64_t ExperimentConfiguration::max_runtime() const
{
  check_setup();
  return m_max_runtime;
}

uint32_t ExperimentConfiguration::number_of_publishers() const
{
  check_setup();
  return m_number_of_publishers;
}
uint32_t ExperimentConfiguration::number_of_subscribers() const
{
  check_setup();
  return m_number_of_subscribers;
}

bool ExperimentConfiguration::check_memory() const
{
  check_setup();
  return m_check_memory;
}

bool ExperimentConfiguration::use_ros_shm() const
{
  check_setup();
  return m_use_ros_shm;
}

bool ExperimentConfiguration::use_single_participant() const
{
  check_setup();
  return m_use_single_participant;
}

bool ExperimentConfiguration::no_waitset() const
{
  check_setup();
  return m_no_waitset;
}

bool ExperimentConfiguration::no_micro_intra() const
{
  check_setup();
  return m_no_micro_intra;
}

bool ExperimentConfiguration::is_with_security() const
{
  check_setup();
  return m_with_security;
}


boost::uuids::uuid ExperimentConfiguration::id() const
{
  return m_id;
}

void ExperimentConfiguration::log(const std::string & msg) const
{
  std::cout << msg << std::endl;
  if (m_os.is_open()) {
    m_os << msg << std::endl;
  }
}

std::string ExperimentConfiguration::logfile_name() const
{
  return m_final_logfile_name;
}

void ExperimentConfiguration::check_setup() const
{
  if (!m_is_setup) {
    throw std::runtime_error("Experiment is not yet setup!");
  }
}

void ExperimentConfiguration::open_file()
{
  check_setup();
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << m_logfile.c_str() << "_" << m_topic_name << std::put_time(&tm, "_%d-%m-%Y_%H-%M-%S");
  m_final_logfile_name = oss.str();
  m_os.open(m_final_logfile_name, std::ofstream::out);
}

bool ExperimentConfiguration::exit_requested() const
{
  return !rclcpp::ok();
}

}  // namespace performance_test
