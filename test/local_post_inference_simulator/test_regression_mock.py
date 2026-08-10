import pytest
from tools.local_post_inference_simulator.backend.scenario_schema import ScenarioSchema, evaluate_assertions
from tools.local_post_inference_simulator.backend.ros_scenario_runner import ScenarioRunner


def test_assertions_evaluation_logic(monkeypatch):
    """
    Mock the ROS bridge and run the runner to verify that assertions
    evaluate correctly under simulated condition. No ROS graph required —
    kept in its own module (no autouse ROS-node fixture) so it always runs
    in CI regardless of whether a live ROS graph is available.
    """
    from unittest.mock import MagicMock
    import tools.local_post_inference_simulator.backend.ros_scenario_runner as runner_mod

    # Mock the ROS bridge
    mock_bridge = MagicMock()
    mock_bridge.wait_for_telemetry_subscribers.return_value = True
    mock_bridge.wait_for_route_intent_ack.return_value = True
    mock_bridge.wait_for_cmd_subscribers.return_value = True

    mock_bridge.synthetic_payload_pub = MagicMock()
    mock_bridge.synthetic_payload_pub.get_subscription_count.return_value = 1
    mock_bridge.cmd_pub = MagicMock()
    mock_bridge.cmd_pub.get_subscription_count.return_value = 1

    # Mock output data
    mock_outputs = {
        "telemetry_realworld": {"timestamp_ms": 100, "objects": []},
        "lane_state": {
            "trajectory_valid": True,
            "trajectory_kind": "follow_main",
            "selected_lane_id": "main_lane_1",
            "blocked_by_marking": False,
            "replan_reason": "none"
        },
        "control_error": {
            "epsilon_x_mm": 5.0,
            "theta_rad": 0.01
        }
    }
    mock_bridge.get_latest_outputs.return_value = mock_outputs
    monkeypatch.setattr(runner_mod, "get_bridge_node", lambda: mock_bridge)

    # Create scenario with assertions
    scenario_dict = {
        "name": "test_assertions",
        "canvas": {"width": 640, "height": 480},
        "calibration": {"source": "config/calibration.json"},
        "route_intent": {"intent": "follow_main", "seq": 1},
        "frames": [
            {
                "frame_id": 1,
                "objects": []
            }
        ],
        "assertions": {
            "expected_selected_lane": "main_lane_1",
            "max_lane_switch_count": 0,
            "max_jitter_epsilon_x_mm": 10.0,
            "max_jitter_theta_rad": 0.05,
            "expected_blocked_state": False,
            "expected_trajectory_kind": "follow_main"
        }
    }

    scenario = ScenarioSchema(**scenario_dict)

    # Run
    runner = ScenarioRunner()
    runner.load_scenario(scenario, mode="direct")
    runner.play()

    # Wait
    if not runner.wait_until_stopped(timeout=5.0):
        runner.stop()
        pytest.fail("Mock run timed out")

    report = runner.get_report()
    assert report is not None

    # Evaluate assertions
    all_pass, results = evaluate_assertions(scenario, report)
    assert all_pass, f"Assertions failed: {results}"
