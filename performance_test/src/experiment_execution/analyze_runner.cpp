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
#include <cstddef>
#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "analyze_runner.hpp"

#include "analysis_result.hpp"

#include <odb/database.hxx>
#include <odb/transaction.hxx>
#include "experiment_configuration_odb.hpp"
#include "analysis_result_odb.hpp"

#include <memory>
#include <odb/database.hxx>
#include <odb/sqlite/database.hxx>
#include <odb/transaction.hxx>
#include <odb/schema-catalog.hxx>

inline std::auto_ptr<odb::database> create_database (int& argc, char* argv[])
{
  std::auto_ptr<odb::core::database> db
      (new odb::sqlite::database(argc, argv, false, SQLITE_OPEN_READWRITE |
                                                    SQLITE_OPEN_CREATE));
  {
    odb::core::connection_ptr c (db->connection ());
    c->execute ("PRAGMA foreign_keys=OFF");
    odb::core::transaction t (c->begin ());
    odb::core::schema_catalog::create_schema (*db);
    t.commit ();
    c->execute ("PRAGMA foreign_keys=ON");
  }
  return db;
}

namespace performance_test
{

AnalyzeRunner::AnalyzeRunner()
: m_ec(ExperimentConfiguration::get()),
  m_is_first_entry(true)
{
  std::stringstream os;
  os << m_ec;
  m_ec.log(os.str());

  // Reading optional environment variable for additional information.
  {
    const auto ptr = std::getenv("APEX_PERFORMANCE_TEST");
    if (ptr) {
      std::string env(ptr);
      boost::trim(env);
      m_ec.log(env);
    }
  }
  for (uint32_t i = 0; i < m_ec.number_of_publishers(); ++i) {
    m_pub_runners.push_back(DataRunnerFactory::get(m_ec.topic_name(), m_ec.com_mean(),
      RunType::PUBLISHER));
  }
  for (uint32_t i = 0; i < m_ec.number_of_subscribers(); ++i) {
    m_sub_runners.push_back(DataRunnerFactory::get(m_ec.topic_name(), m_ec.com_mean(),
      RunType::SUBSCRIBER));
  }
}

void AnalyzeRunner::run() const
{
  m_ec.log("---EXPERIMENT-START---");
  m_ec.log(AnalysisResult::csv_header(true));

  const auto experiment_start = boost::posix_time::microsec_clock::local_time();

  //TODO: this all should be provided in the command line:
  char *argv_db[] = {(char*)"./perf_test",
                     (char*)"--database",
                     (char*)"test_database"};
  int argc_db = 3;
  std::auto_ptr<odb::core::database> db (create_database (argc_db, argv_db));
  //odb::core::transaction t (db->begin());

  while (!check_exit(experiment_start)) {
    const auto loop_start = boost::posix_time::microsec_clock::local_time();

    sleep(1);

    std::for_each(m_pub_runners.begin(), m_pub_runners.end(), [](auto & a) {a->sync_reset();});
    std::for_each(m_sub_runners.begin(), m_sub_runners.end(), [](auto & a) {a->sync_reset();});

    /// Id drivepx_rt is set and this is the first loop, set the post RT init settings
    if (m_is_first_entry && m_ec.is_drivepx_rt()) {
      post_proc_rt_init();
      m_is_first_entry = false;
    }
    auto now = boost::posix_time::microsec_clock::local_time();
    auto loop_diff_start = now - loop_start;
    auto experiment_diff_start = now - experiment_start;

    analyze(loop_diff_start, experiment_diff_start, db);
  }
  //t.commit();
}

void AnalyzeRunner::analyze(
  const boost::posix_time::time_duration loop_diff_start,
  const boost::posix_time::time_duration experiment_diff_start,
  std::auto_ptr<odb::core::database> db) const
{
  std::vector<StatisticsTracker> latency_vec(m_sub_runners.size());
  std::transform(m_sub_runners.begin(), m_sub_runners.end(), latency_vec.begin(),
    [](const auto & a) {return a->latency_statistics();});

  std::vector<StatisticsTracker> ltr_pub_vec(m_pub_runners.size());
  std::transform(m_pub_runners.begin(), m_pub_runners.end(), ltr_pub_vec.begin(),
    [](const auto & a) {return a->loop_time_reserve_statistics();});

  std::vector<StatisticsTracker> ltr_sub_vec(m_sub_runners.size());
  std::transform(m_sub_runners.begin(), m_sub_runners.end(), ltr_sub_vec.begin(),
    [](const auto & a) {return a->loop_time_reserve_statistics();});

  uint64_t sum_received_samples = 0;
  for (auto e : m_sub_runners) {
    sum_received_samples += e->sum_received_samples();
  }

  uint64_t sum_sent_samples = 0;
  for (auto e : m_pub_runners) {
    sum_sent_samples += e->sum_sent_samples();
  }

  uint64_t sum_lost_samples = 0;
  for (auto e : m_sub_runners) {
    sum_lost_samples += e->sum_lost_samples();
  }

  uint64_t sum_data_received = 0;
  for (auto e : m_sub_runners) {
    sum_data_received += e->sum_data_received();
  }

  AnalysisResult result(
    experiment_diff_start,
    loop_diff_start,
    sum_received_samples,
    sum_sent_samples,
    sum_lost_samples,
    sum_data_received,
    StatisticsTracker(latency_vec),
    StatisticsTracker(ltr_pub_vec),
    StatisticsTracker(ltr_sub_vec)
  );

  m_ec.log(result.to_csv_string(true));

  //save values to sql database
  //db->persist(m_ec);
  //db->persist(result);


}

bool AnalyzeRunner::check_exit(boost::posix_time::ptime experiment_start) const
{
  if (m_ec.exit_requested()) {
    std::cout << "Caught signal. Exiting." << std::endl;
    return true;
  }

  if (m_ec.max_runtime() == 0) {
    // Run forever,
    return false;
  }

  const double runtime_sec =
      boost::posix_time::time_duration(boost::posix_time::microsec_clock::local_time() -
      experiment_start).seconds();

  if (runtime_sec > m_ec.max_runtime()) {
    std::cout << "Maximum runtime reached. Exiting." << std::endl;
    return true;
  } else {
    return false;
  }
}



}  // namespace performance_test
