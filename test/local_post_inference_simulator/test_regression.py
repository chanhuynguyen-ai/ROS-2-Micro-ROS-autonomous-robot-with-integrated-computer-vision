import os

# Must be set before rclpy.init() (triggered by importing ros_bridge below) or before any
# `ros2 run` subprocess starts: some sandboxes have a read-only/unwritable ~/.ros, which makes
# rclcpp/rclpy fail to start and silently makes nodes never come up (tests then skip with a
# misleading "not subscribing" message instead of surfacing the real cause).
os.environ.setdefault("ROS_LOG_DIR", "/tmp/avs_ros_logs")
os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)

import json
import pytest
import time
from tools.local_post_inference_simulator.backend.scenario_schema import ScenarioSchema, evaluate_assertions
from tools.local_post_inference_simulator.backend.ros_scenario_runner import ScenarioRunner
from tools.local_post_inference_simulator.backend.ros_bridge import get_bridge_node, shutdown_bridge

FIXTURES_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    "../../tools/local_post_inference_simulator/fixtures"
))

def get_scenario_files():
    files = []
    if os.path.exists(FIXTURES_DIR):
        for f in os.listdir(FIXTURES_DIR):
            if f.endswith(".json"):
                files.append(f)
    return sorted(files)

import subprocess
import signal

synthetic_node_process = None
ipm_node_process = None
control_node_process = None

NODE_LOG_DIR = os.environ["ROS_LOG_DIR"]

def _spawn(cmd, log_name, **kwargs):
    # Redirect stdout/stderr to a file instead of inheriting the caller's fds. If a caller
    # (e.g. `timeout N pytest ... | tail`) has piped stdout and this process outlives its
    # parent (ros2 run's actual node is a grandchild `terminate()` may not reach), an
    # inherited pipe fd is held open forever and the reader (`tail`) never sees EOF —
    # the whole pipeline hangs indefinitely even though pytest itself already exited.
    log_path = os.path.join(NODE_LOG_DIR, log_name)
    log_file = open(log_path, "w")
    return subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,  # own process group, so we can kill ros2 run's child node too
        **kwargs,
    )

def start_ros_nodes():
    global synthetic_node_process, ipm_node_process, control_node_process

    # Start synthetic inference node
    script_path = os.path.abspath(os.path.join(FIXTURES_DIR, "../ros2/synthetic_inference_node.py"))
    if os.path.exists(script_path):
        synthetic_node_process = _spawn(["python3", script_path], "synthetic_inference_node.log")

    # Start C++ nodes via ros2 run
    workspace_dir = os.path.abspath(os.path.join(FIXTURES_DIR, "../../../ros2_ws"))
    repo_root = os.path.dirname(workspace_dir)
    calibration_path = os.path.join(repo_root, "config", "calibration.json")
    setup_script = os.path.join(workspace_dir, "install_user/setup.bash")
    if not os.path.exists(setup_script):
        setup_script = os.path.join(workspace_dir, "install/setup.bash")

    if os.path.exists(setup_script):
        # publish_debug_centerline is set true at launch rather than relying on
        # ScenarioRunner's runtime `ros2 param set` (that call can PermissionError in
        # sandboxed environments; harmless for pass/fail, but leaves debug_centerline
        # missing from the report that Tầng 2 fixture investigation needs).
        cmd_ipm = (
            f"source {setup_script} && ros2 run avs_perception ipm_transform_node "
            f"--ros-args -p calibration_file_path:={calibration_path} "
            f"-p publish_debug_centerline:=true"
        )
        ipm_node_process = _spawn(cmd_ipm, "ipm_transform_node.log", shell=True, executable='/bin/bash')

        cmd_control = f"source {setup_script} && ros2 run avs_perception control_node"
        control_node_process = _spawn(cmd_control, "control_node.log", shell=True, executable='/bin/bash')

def _kill_group(proc, timeout=2.0):
    if not proc:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    try:
        os.killpg(pgid, signal.SIGTERM)
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(pgid, signal.SIGKILL)
        proc.wait(timeout=2.0)
    except ProcessLookupError:
        pass

def stop_ros_nodes():
    global synthetic_node_process, ipm_node_process, control_node_process

    _kill_group(synthetic_node_process)
    _kill_group(ipm_node_process)
    _kill_group(control_node_process)

@pytest.fixture(scope="module", autouse=True)
def setup_teardown_simulator():
    start_ros_nodes()
    # Give nodes time to initialize (ros2 run + sourcing setup.bash has noticeable startup latency)
    time.sleep(5.0)
    yield
    shutdown_bridge()
    stop_ros_nodes()

@pytest.mark.ros
@pytest.mark.parametrize("scenario_filename", get_scenario_files())
def test_scenario_regression(scenario_filename):
    # Load scenario
    path = os.path.join(FIXTURES_DIR, scenario_filename)
    with open(path, "r") as f:
        data = json.load(f)
    
    scenario = ScenarioSchema(**data)
    if not scenario.assertions:
        pytest.skip(f"Scenario '{scenario.name}' has no assertions defined.")

    # Check if ROS2 nodes are running. Set AVS_REQUIRE_LIVE_ROS=1 (e.g. in CI) to turn these
    # into hard failures instead of skips — a silent skip here previously masked a path bug
    # that made every one of these tests a no-op for an unknown length of time.
    require_live = os.environ.get("AVS_REQUIRE_LIVE_ROS") == "1"

    def _give_up(reason: str):
        if require_live:
            pytest.fail(reason)
        pytest.skip(reason)

    try:
        bridge = get_bridge_node()
        telemetry_ok = bridge.wait_for_telemetry_subscribers(timeout_sec=3.0)
        cmd_ok = bridge.wait_for_cmd_subscribers(timeout_sec=3.0)
    except Exception as e:
        _give_up(f"ROS2 not initialized or bridge failed: {e}")
        return

    if not (telemetry_ok and cmd_ok):
        _give_up(
            "Required ROS2 nodes are not subscribing "
            f"(telemetry_ok={telemetry_ok} [ipm_transform_node], cmd_ok={cmd_ok} [control_node]). "
            f"synthetic_node alive={synthetic_node_process.poll() is None if synthetic_node_process else None}, "
            f"ipm_node alive={ipm_node_process.poll() is None if ipm_node_process else None}, "
            f"control_node alive={control_node_process.poll() is None if control_node_process else None}."
        )

    # Run the scenario
    runner = ScenarioRunner()
    runner.load_scenario(scenario, mode="direct")
    runner.play()

    # Wait for completion (with timeout). Real-video fixtures (real_*.json)
    # replay 40-120 frames at ROS pace (~0.3-0.5s/frame), so scale the budget
    # with frame count instead of a flat 15s.
    timeout = max(15.0, 0.6 * len(scenario.frames))
    if not runner.wait_until_stopped(timeout=timeout):
        runner.stop()
        pytest.fail(f"Scenario {scenario.name} timed out after {timeout}s")

    report = runner.get_report()
    assert report is not None, "Report was not generated"

    # Evaluate assertions
    all_pass, results = evaluate_assertions(scenario, report)
    assert all_pass, f"Assertions failed: {results}"
