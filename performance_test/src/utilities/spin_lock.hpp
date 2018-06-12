// Copyright 2018 Apex.AI, Inc.
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


#ifndef UTILITIES__SPIN_LOCK_HPP_
#define UTILITIES__SPIN_LOCK_HPP_

#include <atomic>

namespace performance_test
{

/// A simple spinlock
class SpinLock
{
public:
  SpinLock()
  : m_lock(ATOMIC_FLAG_INIT) {}

  /// Locks the spinlock.
  inline void lock()
  {
    while (m_lock.test_and_set(std::memory_order_acquire)) {
      // spin
    }
  }

  /// Unlocks the spinlock.
  inline void unlock()
  {
    m_lock.clear(std::memory_order_release);
  }

private:
  std::atomic_flag m_lock;
};

}  // namespace performance_test

#endif  // UTILITIES__SPIN_LOCK_HPP_
