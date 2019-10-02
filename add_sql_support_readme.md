# Save results to a SQL database

## Requirements
***ODB 2.5.0***
> Note: 2.5.0 is a beta version, but it is required if you are using Ubuntu 18.04 LTS and
gcc 7.4.0, if you are using earlier ubuntu/gcc versions, proceed to the installation of ODB 2.4.0
below.

Follow the instructions [here](https://www.codesynthesis.com/products/odb/doc/install-build2.xhtml)
 to install build2 toolchain, odb compiler and runtime libraries.

Required runtime libraries are: libodb, libodb-sqlite/libodb-pgsql/libodb-mysql (depending on which
 database you want to use) and libodb-boost.

Remember to install the correct gcc plugin, for example (for gcc 7):
```
sudo apt-get update
sudo apt-get install gcc-7-plugin-dev
```
Additionally, if you want to use MYSQL database, you will need to build a not yet released version,
so instead of executing:
```
bpkg build libodb-mysql
bpkg install libodb-mysql
```
you will need to execute:
```
bpkg build libodb-mysql@https://git.codesynthesis.com/odb/libodb-mysql.git#fix-bind-decl
bpkg install libodb-mysql
```
After installing all the libraries make sure to add `/usr/local/lib` to your PATH:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

***ODB 2.4.0:***
All the downloads can be found [here](https://www.codesynthesis.com/products/odb/download.xhtml).
* Odb compiler
* Common Runtime Library: libodb-2.4.0
* Database Runtime Library: libodb-sqlite-2.4.0/libodb-mysql-2.4.0/libodb-pgsql-2.4.0
* Profile Libraries: libodb-boost-2.4.0, libodb-qt-2.4.0

Follow the [instructions](https://www.codesynthesis.com/products/odb/doc/install-unix.xhtml) to
install all the components.

## How to build and run
```
source ros2_install_path/setup.bash
mkdir -p perf_test_ws/src
cd perf_test_ws/src
git clone https://github.com/ApexAI/performance_test.git
cd ..
colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release -DPERFORMANCE_TEST_ODB_FOR_SQL_ENABLED=ON
source install/setup.bash
ros2 run performance_test perf_test -c ROS2 -l log -t Array1k --max_runtime 10
```

The default name of the resulting database is "db_name", you can change it by using `--db_name`
argument in `ros2 run`. For MySQL or PostgreSQL databases, you can also specify `--db_user`,
`--db_password`, `--db_host` and `--db_port` to connect to your database.

The default database is SQLite but if you want to use MySQL or PostgreSQL instead, use
`-DPERFORMANCE_TEST_ODB_MYSQL=ON` or `-DPERFORMANCE_TEST_ODB_PGSQL=ON` and
disable the SQLite by adding `-DPERFORMANCE_TEST_ODB_SQLITE=OFF` option.

> All the necessary changes to add SQL database support to the performance_test tool were made by
following instructions from [ODB platform](https://www.codesynthesis.com/products/odb/). Please
refer to the [ODB manual](https://www.codesynthesis.com/products/odb/doc/odb-manual.pdf) for more information
 and implementation details.