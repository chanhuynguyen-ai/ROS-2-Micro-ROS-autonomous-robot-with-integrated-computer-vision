import os

os.environ.setdefault("ROS_LOG_DIR", "/tmp/avs_ros_logs")
os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)

import json
import signal
import subprocess
import pytest
import time
from tools.local_post_inference_simulator.backend.scenario_schema import ScenarioSchema, evaluate_assertions
from tools.local_post_inference_simulator.backend.ros_scenario_runner import ScenarioRunner
from tools.local_post_inference_simulator.backend.ros_bridge import get_bridge_node, shutdown_bridge

# R3/R4 (T-junction detection + turn_left-blocked-by-solid) need the full ROS
# node state machine (t_junction_counter_ hysteresis, update_lane_state) which
# is not mirrored in decision_harness.py - these must run against the real
# control_node binary rather than a hand-rolled Python re-implementation.

FIXTURES_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    "../../tools/local_post_inference_simulator/fixtures"
))

NODE_LOG_DIR = os.environ["ROS_LOG_DIR"]

synthetic_node_process = None
ipm_node_process = None
control_node_process = None


def _spawn(cmd, log_name, **kwargs):
    log_path = os.path.join(NODE_LOG_DIR, log_name)
    log_file = open(log_path, "w")
    return subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        **kwargs,
    )


def start_ros_nodes():
    global synthetic_node_process, ipm_node_process, control_node_process

    script_path = os.path.abspath(os.path.join(FIXTURES_DIR, "../ros2/synthetic_inference_node.py"))
    if os.path.exists(script_path):
        synthetic_node_process = _spawn(["python3", script_path], "synthetic_inference_node.log")

    workspace_dir = os.path.abspath(os.path.join(FIXTURES_DIR, "../../../ros2_ws"))
    repo_root = os.path.dirname(workspace_dir)
    calibration_path = os.path.join(repo_root, "config", "calibration.json")
    setup_script = os.path.join(workspace_dir, "install_user/setup.bash")
    if not os.path.exists(setup_script):
        setup_script = os.path.join(workspace_dir, "install/setup.bash")

    if os.path.exists(setup_script):
        cmd_ipm = (
            f"source {setup_script} && ros2 run avs_perception ipm_transform_node "
            f"--ros-args -p calibration_file_path:={calibration_path} "
            f"-p publish_debug_centerline:=true"
        )
        ipm_node_process = _spawn(cmd_ipm, "ipm_transform_node.log", shell=True, executable="/bin/bash")

        cmd_control = f"source {setup_script} && ros2 run avs_perception control_node"
        control_node_process = _spawn(cmd_control, "control_node.log", shell=True, executable="/bin/bash")


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
    time.sleep(5.0)
    yield
    shutdown_bridge()
    stop_ros_nodes()


def _run_scenario(filename):
    path = os.path.join(FIXTURES_DIR, filename)
    with open(path, "r") as f:
        data = json.load(f)
    scenario = ScenarioSchema(**data)

    require_live = os.environ.get("AVS_REQUIRE_LIVE_ROS") == "1"

    def _give_up(reason):
        if require_live:
            pytest.fail(reason)
        pytest.skip(reason)

    try:
        bridge = get_bridge_node()
        telemetry_ok = bridge.wait_for_telemetry_subscribers(timeout_sec=3.0)
        cmd_ok = bridge.wait_for_cmd_subscribers(timeout_sec=3.0)
    except Exception as e:
        _give_up(f"ROS2 not initialized or bridge failed: {e}")
        return None, None

    if not (telemetry_ok and cmd_ok):
        _give_up("Required ROS2 nodes are not subscribing")
        return None, None

    runner = ScenarioRunner()
    runner.load_scenario(scenario, mode="direct")
    runner.play()

    if not runner.wait_until_stopped(timeout=15.0):
        runner.stop()
        pytest.fail(f"Scenario {scenario.name} timed out")

    report = runner.get_report()
    assert report is not None
    return scenario, report


def _hold_reasons(report):
    reasons = []
    for frame in report["frames"]:
        ls = (frame.get("outputs", {}) or {}).get("lane_state") or {}
        reasons.append(ls.get("hold_reason"))
    return reasons


@pytest.mark.ros
def test_t_junction_detected_purely_from_geometry_no_stopline():
    """R3: with no main-ahead and two widely-spread turn-lane candidates (no
    stop-line object anywhere in the scene), detect_t_junction's 3-frame
    hysteresis must confirm the T-junction and commit the turn - proving the
    detection is driven by lane geometry alone."""
    scenario, report = _run_scenario("t_junction_no_stopline.json")
    if scenario is None:
        pytest.skip("live ROS not available")

    reasons = _hold_reasons(report)
    # First two frames: geometry looks like a T-junction but hysteresis hasn't
    # confirmed it yet (t_junction_counter_ < 3).
    assert reasons[0] == "t_junction_pending"
    assert reasons[1] == "t_junction_pending"
    # Once confirmed (3rd frame), the turn commits and the hold clears.
    assert reasons[-1] == ""

    all_pass, results = evaluate_assertions(scenario, report)
    assert all_pass, f"Assertions failed: {results}"


@pytest.mark.ros
def test_t_junction_not_triggered_by_stopline_alone():
    """R3: a stop-line object must never trigger T-junction / turn-related
    hold state on its own. Same wide turn-lane geometry as the fixture above,
    but main-ahead is present (so detect_t_junction's first condition never
    holds) - hold_reason must never become t_junction_pending."""
    scenario, report = _run_scenario("t_junction_not_triggered_by_stopline.json")
    if scenario is None:
        pytest.skip("live ROS not available")

    reasons = _hold_reasons(report)
    assert all(r != "t_junction_pending" for r in reasons)

    all_pass, results = evaluate_assertions(scenario, report)
    assert all_pass, f"Assertions failed: {results}"


@pytest.mark.ros
def test_t_junction_turn_left_blocked_by_solid_marking():
    """R4: once a T-junction is confirmed, a solid marking blocking the
    turn-left path must force decision_state=BLOCKED / trajectory_kind=
    blocked_follow_main and keep the active trajectory on main - never commit
    the left-turn geometry."""
    scenario, report = _run_scenario("t_junction_turn_left_blocked_by_solid.json")
    if scenario is None:
        pytest.skip("live ROS not available")

    reasons = _hold_reasons(report)
    assert reasons[-1] == "blocked_by_marking"

    last_lane_state = (report["frames"][-1].get("outputs", {}) or {}).get("lane_state") or {}
    assert last_lane_state.get("blocked_by_marking") is True
    assert last_lane_state.get("trajectory_kind") == "blocked_follow_main"
    assert last_lane_state.get("selected_lane_id") == "tjlb_main_current"

    all_pass, results = evaluate_assertions(scenario, report)
    assert all_pass, f"Assertions failed: {results}"
