## Performance Test Tool

### Purpose / Use Cases (Why implement this feature?)
This tool was designed to evaluate the performance of various means of communication
which either support the publish subscribe pattern directly or can be adapted to it.

### Design (How does it work?)
It runs publishers and subscribers, each one in one independent thread or process and records how fast
and with what latency data can be published and subscribed.

#### Assumptions / Known Limits
* Communication frameworks like DDS have a huge amount of settings. These are at the moment
hardcoded, aside form the common QOS settings which can be set on the command line.
* Only one publisher per topic is allowed, because the data verification logic does not support
matching data to the different publishers.
* Some communication plugins can get stuck in their internal loops if too much data is received.
But figuring out ways around such issues is one of the goals of this tool.
* ROS 2 msg files are not automatically converted to IDL files used by
the tool. But as ROS 2 will support IDL files in the near future, this issue
will be resolved.

#### Inputs / Outputs / API (How to use it?)
The tool has a fully documented command line interface which can be accessed by typing
`performance_test --help`.

To run more extensive tests, the following script can be used and adapted:
`helper_scripts/run_experiment.py`

The logfiles can be visualized using the following python script:
`helper_scripts/performance_test_file_reader.py`
Note that `pandas` is required to run this script.

##### CMAKE options

* `-DPERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED` Enable Connext DDS Micro support. Enabled by default.
* `-DPERFORMANCE_TEST_FASTRTPS_ENABLED` Enables FastRTPS support. Enabled by default.

> ROS 2 support is always enabled.

#### Inner-workings / Algorithms
The tool consists of the following modules:

##### Experiment configuration:
This modules is responsible for reading the experiment configuration like
what mean of communication to use, what QOS to use and how fast data should
be transferred.

##### Experiment execution:
Runs the experiment on the highest level. Responsible for setting up the experiment,
collecting experiment results regularly and outputting it to the user in form of command line
output and file output.

##### Data running:
Responsible for starting the threads effectively running the experiment and synchronizing data between
the experiment threads and the experiment execution.

##### Communication Abstractions
Plugins for the various means of communications which do the actual publishing and subscribing of data.
These are used by the data running module then.

##### Interface specification files and their generation
The ROS 2 MSG and IDL files and the machinery to generate sources from
them are located there.

##### Utilities
General helper files for real time and statistics are located there.

#### Error Detection and Handling
All errors are translated to exceptions. But exception are not handled and will
propagate up causing the application to terminate.

#### Security Considerations
<!-- Required -->
The security of this tool depends mostly on the tested means of communications and their
implementation in the plugins. Therefore this tool is not secure.

If you run the tool with the `helper_scripts/run_experiment.py` script is ran as root to
gain privileges to set real time priorities. If you are not in a secure network, you should not run this too,
or at least not as root.
#### References / External Links
<!-- Optional -->

#### Future Extensions / Unimplemented Parts
Possible additional communication which could be implemented are:
* ROS 1
* ROS 2 with wait sets
* Raw UDP communication

#### Related Issues
https://gitlab.apex.ai/ApexAI/grand_central/issues/957
https://gitlab.apex.ai/ApexAI/grand_central/issues/1305
https://gitlab.apex.ai/ApexAI/grand_central/issues/1382
https://gitlab.apex.ai/ApexAI/grand_central/issues/1305



