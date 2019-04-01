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

#ifndef EXPERIMENT_CONFIGURATION__COMMUNICATION_MEAN_HPP_
#define EXPERIMENT_CONFIGURATION__COMMUNICATION_MEAN_HPP_

namespace performance_test
{

/**
 * \brief Available means of communication.
 */
enum class CommunicationMean
{
  ROS2
#ifdef PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED
  , ROS2PollingSubscription
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
  , FASTRTPS
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  , CONNEXTDDSMICRO
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  , CYCLONEDDS
#endif

#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
  , OPENDDS
#endif
};

/// Outstream operator for CommunicationMean.
inline std::ostream & operator<<(std::ostream & stream, const CommunicationMean cm)
{
  if (cm == CommunicationMean::ROS2) {
    return stream << "ROS2";
#ifdef PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED
  } else if (cm == CommunicationMean::ROS2PollingSubscription) {
    return stream << "ROS2PollingSubscription";
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
  } else if (cm == CommunicationMean::FASTRTPS) {
    return stream << "FASTRTPS";
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  } else if (cm == CommunicationMean::CONNEXTDDSMICRO) {
    return stream << "CONNEXTDDSMICRO";
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  } else if (cm == CommunicationMean::CYCLONEDDS) {
    return stream << "CYCLONEDDS";
#endif

#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
  } else if (cm == CommunicationMean::OPENDDS) {
    return stream << "OpenDDS";
#endif
  } else {
    throw std::invalid_argument("Enum value not supported!");
  }
}

}  // namespace performance_test

#endif  // EXPERIMENT_CONFIGURATION__COMMUNICATION_MEAN_HPP_
