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

#include <rclcpp/rclcpp.hpp>

#include "experiment_configuration/experiment_configuration.hpp"
#include "experiment_execution/analyze_runner.hpp"

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  char *argv_db[] = {(char*)"./perf_test",
                  (char*)"--database",
                  (char*)"test_database"};
  int argc_db = 3;

  try {
    std::auto_ptr<odb::core::database> db (create_database (argc_db, argv_db));

    auto & ec = performance_test::ExperimentConfiguration::get();
    ec.setup(argc, argv);

    performance_test::AnalyzeRunner ar;
    ar.run(db);
  }

  catch (const odb::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}

