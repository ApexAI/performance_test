import launch
import launch.actions

TEST_PROC_PATH = os.path.join(
    ament_index_python.get_package_prefix('performance_test'),
    'lib/performance_test',
    'perf_test'
)

# This is necessary to get unbuffered output from the process under test
proc_env = os.environ.copy()
proc_env['PYTHONUNBUFFERED'] = '1'

perf_test_process_main = launch.actions.ExecuteProcess(
    cmd=[TEST_PROC_PATH, '-c', 'ROS2', '-t', 'Array1k', '--roundtrip_mode', 'Main'],
    env=proc_env,
)

perf_test_process_relay = launch.actions.ExecuteProcess(
    cmd=[TEST_PROC_PATH, '-c', 'ROS2', '-t', 'Array1k', '--roundtrip_mode', 'Relay'],
    env=proc_env,
)

def generate_test_description(ready_fn):

    return launch.LaunchDescription([
        perf_test_process_main,
        perf_test_process_relay,

        # Start tests right away - no need to wait for anything
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
])