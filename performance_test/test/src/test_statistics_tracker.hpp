// Copyright 2017 Apex.AI, Inc.
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

#ifndef TEST_STATISTICS_TRACKER_HPP_
#define TEST_STATISTICS_TRACKER_HPP_

#include <limits>
#include "../../src/utilities/statistics_tracker.hpp"

TEST(performance_test, StatisticsTracker_init) {
  performance_test::StatisticsTracker st;

  ASSERT_FLOAT_EQ(st.mean(), 0.0);
  ASSERT_FLOAT_EQ(st.n(), 0.0);
  ASSERT_FLOAT_EQ(st.max(), std::numeric_limits<double>::lowest());
  ASSERT_FLOAT_EQ(st.min(), std::numeric_limits<double>::max());
}

TEST(performance_test, StatisticsTracker_single_sample) {
  performance_test::StatisticsTracker st;
  const auto sample = 5.345345;
  st.add_sample(sample);

  ASSERT_FLOAT_EQ(st.n(), 1.0);
  ASSERT_FLOAT_EQ(st.mean(), sample);
  ASSERT_FLOAT_EQ(st.max(), sample);
  ASSERT_FLOAT_EQ(st.min(), sample);
}


TEST(performance_test, StatisticsTracker_two_samples) {
  performance_test::StatisticsTracker st;

  const auto small = 2.235235;
  const auto big = 242.235253;

  st.add_sample(small);
  st.add_sample(big);

  const auto mean = (small + big) / 2.0;
  const auto variance = ((mean - small) * (mean - small) + (mean - big) * (mean - big)) / 2.0;

  ASSERT_FLOAT_EQ(st.min(), small);
  ASSERT_FLOAT_EQ(st.max(), big);
  ASSERT_FLOAT_EQ(st.n(), 2.0);
  ASSERT_FLOAT_EQ(st.mean(), mean);
  ASSERT_FLOAT_EQ(st.variance(), variance);
}

TEST(performance_test, StatisticsTracker_three_samples) {
  performance_test::StatisticsTracker st;

  const auto small = -2.235235;
  const auto mid = 26.235235;
  const auto big = 232.235253;

  st.add_sample(mid);
  st.add_sample(small);
  st.add_sample(big);

  const auto mean = (small + big + mid) / 3.0;
  const auto variance =
    ((mean - small) * (mean - small) + (mean - big) * (mean - big) + (mean - mid) * (mean - mid)) /
    3.0;

  ASSERT_FLOAT_EQ(st.min(), small);
  ASSERT_FLOAT_EQ(st.max(), big);
  ASSERT_FLOAT_EQ(st.n(), 3.0);
  ASSERT_FLOAT_EQ(st.mean(), mean);
  ASSERT_FLOAT_EQ(st.variance(), variance);
}

#endif  // TEST_STATISTICS_TRACKER_HPP_
