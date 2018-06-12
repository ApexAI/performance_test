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

#ifndef EXPERIMENT_CONFIGURATION__QOS_ABSTRACTION_HPP_
#define EXPERIMENT_CONFIGURATION__QOS_ABSTRACTION_HPP_

#include <ostream>

namespace performance_test
{

/**
 * \brief Holds limited QOS information in an DDS implementation independent way.
 */
struct QOSAbstraction
{
  QOSAbstraction()
  : reliability(),
    durability(),
    history_kind(),
    history_depth(),
    sync_pubsub(false) {}

  /// Abstract type for DDS reliability.
  enum class Reliability
  {
    BEST_EFFORT,
    RELIABLE
  };
  /// Abstract type for DDS durability.
  enum class Durability
  {
    VOLATILE,
    TRANSIENT_LOCAL
  };
  /// Abstract type for DDS history kind.
  enum class HistoryKind
  {
    KEEP_ALL,
    KEEP_LAST
  };

  /// Stored DDS reliability kind.
  Reliability reliability;
  /// Stored DDS durability kind.
  Durability durability;
  /// Stored DDS history kind.
  HistoryKind history_kind;
  /// Stored DDS history depth.
  uint32_t history_depth;
  /// Use synchronous pub/sub.
  bool sync_pubsub;
};

/// Outstream operator for QOSAbstraction::Reliability.
std::ostream & operator<<(std::ostream & stream, const QOSAbstraction::Reliability e);
/// Outstream operator for QOSAbstraction::Durability.
std::ostream & operator<<(std::ostream & stream, const QOSAbstraction::Durability e);
/// Outstream operator for QOSAbstraction::HistoryKind.
std::ostream & operator<<(std::ostream & stream, const QOSAbstraction::HistoryKind e);
/// Outstream operator for QOSAbstraction::QOSAbstraction.
std::ostream & operator<<(std::ostream & stream, const QOSAbstraction e);

}  // namespace performance_test

#endif  // EXPERIMENT_CONFIGURATION__QOS_ABSTRACTION_HPP_
