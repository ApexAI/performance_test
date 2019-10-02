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
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <cstdlib>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "analyze_runner.hpp"

#include "analysis_result.hpp"

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #include <odb/database.hxx>
  #include <memory>
  #ifdef PERFORMANCE_TEST_ODB_SQLITE
    #include <odb/sqlite/database.hxx>
  #endif
  #ifdef PERFORMANCE_TEST_ODB_MYSQL
    #include <odb/mysql/database.hxx>
  #endif
  #ifdef PERFORMANCE_TEST_ODB_PGSQL
    #include <odb/pgsql/database.hxx>
  #endif
  #include <odb/transaction.hxx>
  #include <odb/schema-catalog.hxx>

  #include "experiment_configuration_odb.hpp"
  #include "analysis_result_odb.hpp"
#endif


namespace performance_test
{

AnalyzeRunner::AnalyzeRunner()
: m_ec(ExperimentConfiguration::get()),
  m_is_first_entry(true)
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  , m_db()
#endif
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

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
#ifdef PERFORMANCE_TEST_ODB_SQLITE
  m_db = std::unique_ptr<odb::core::database>(new odb::sqlite::database(
        m_ec.db_name(), SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE));
#elif PERFORMANCE_TEST_ODB_MYSQL
  m_db = std::unique_ptr<odb::core::database>(new odb::mysql::database(
        m_ec.db_user(), m_ec.db_password(), m_ec.db_name(), m_ec.db_host(), m_ec.db_port()));
#elif PERFORMANCE_TEST_ODB_PGSQL
  m_db = std::unique_ptr<odb::core::database>(new odb::pgsql::database(
        m_ec.db_user(), m_ec.db_password(), m_ec.db_name(), m_ec.db_host(), m_ec.db_port()));
#endif
  {
    #ifdef PERFORMANCE_TEST_ODB_SQLITE
    // Due to bugs in SQLite foreign key support for DDL statements,
    // we need to temporarily disable foreign keys while creating the database schema.
    odb::core::connection_ptr c(m_db->connection());
    c->execute("PRAGMA foreign_keys=OFF");
    #endif
    odb::core::transaction t(c->begin());
    try {
      m_db->query<ExperimentConfiguration>(false);
    } catch (const odb::exception & e) {
      odb::core::schema_catalog::create_schema(*m_db);
    }
    t.commit();
    #ifdef PERFORMANCE_TEST_ODB_SQLITE
    c->execute("PRAGMA foreign_keys=ON");
    #endif
  }
#endif
}

void AnalyzeRunner::run() const
{
  m_ec.log("---EXPERIMENT-START---");
  m_ec.log(AnalysisResult::csv_header(true));

  const auto experiment_start = std::chrono::steady_clock::now();

  #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  odb::core::transaction t(m_db->begin());
  #endif

  while (!check_exit(experiment_start)) {
    const auto loop_start = std::chrono::steady_clock::now();

    sleep(1);

    std::for_each(m_pub_runners.begin(), m_pub_runners.end(), [](auto & a) {a->sync_reset();});
    std::for_each(m_sub_runners.begin(), m_sub_runners.end(), [](auto & a) {a->sync_reset();});

    /// Id drivepx_rt is set and this is the first loop, set the post RT init settings
    if (m_is_first_entry && m_ec.is_drivepx_rt()) {
      post_proc_rt_init();
      m_is_first_entry = false;
    }

    auto now = std::chrono::steady_clock::now();
    auto loop_diff_start = now - loop_start;
    auto experiment_diff_start = now - experiment_start;
    analyze(loop_diff_start, experiment_diff_start);
  }

  #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  m_db->persist(m_ec);
  t.commit();
  #endif
}

void AnalyzeRunner::analyze(
  const std::chrono::nanoseconds loop_diff_start,
  const std::chrono::nanoseconds experiment_diff_start) const
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

  auto result = std::make_shared<AnalysisResult>(
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

  m_ec.log(result->to_csv_string(true));

  #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  result->set_configuration(&m_ec);
  m_ec.get_results().push_back(result);

  m_db->persist(result);
  #endif
}

bool AnalyzeRunner::check_exit(std::chrono::steady_clock::time_point experiment_start) const
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
    std::chrono::duration<double>(std::chrono::steady_clock::now() - experiment_start).count();

  if (runtime_sec > m_ec.max_runtime()) {
    std::cout << "Maximum runtime reached. Exiting." << std::endl;
    return true;
  } else {
    return false;
  }
}

}  // namespace performance_test
