import os
import json
import pytest
from unittest.mock import MagicMock, patch

from tools.local_post_inference_simulator.backend.scenario_schema import ScenarioSchema
from tools.local_post_inference_simulator.backend.ros_scenario_runner import ScenarioRunner


FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "tools", "local_post_inference_simulator", "fixtures")

def load_scenario_fixture(name: str) -> ScenarioSchema:
    path = os.path.join(FIXTURES_DIR, name)
    with open(path, "r") as f:
        data = json.load(f)
        return ScenarioSchema(**data)


@pytest.fixture
def mock_ros_bridge():
    """Mock the ROS bridge to avoid needing a live ROS graph for CI tests."""
    with patch("tools.local_post_inference_simulator.backend.ros_scenario_runner.get_bridge_node") as mock_get_bridge:
        mock_bridge = MagicMock()
        mock_get_bridge.return_value = mock_bridge
        
        mock_bridge.publish_telemetry = MagicMock()
        mock_bridge.wait_for_telemetry_subscribers.return_value = True
        mock_bridge.synthetic_payload_pub = MagicMock()
        mock_bridge.synthetic_payload_pub.get_subscription_count.return_value = 1
        mock_bridge.wait_for_route_intent_ack.return_value = True
        mock_bridge.wait_for_route_intent_state.return_value = True
        
        # Create fake output data mimicking control_node
        def mock_get_latest_outputs():
            return {
                "telemetry_realworld": {
                    "timestamp_ms": 100,
                    "objects": []
                },
                "lane_state": {
                    "decision_state": "follow_main",
                    "lane_state": "follow_main",
                    "route_intent": "follow_main",
                    "trajectory_valid": True,
                    "trajectory_kind": "follow_main",
                    "normalization_mode": "no_previous_passthrough",
                    "replan_reason": "first_valid_trajectory",
                    "control_source": "trajectory_manager",
                    "debug_trajectories": [
                        {
                            "stage": "committed",
                            "valid": True,
                            "trajectory_kind": "follow_main",
                            "confidence": 1.0,
                            "normalization_mode": "no_previous_passthrough",
                            "points": [[0.0, 100.0], [0.0, 500.0], [0.0, 900.0]],
                            "has_precomputed_control": True,
                        }
                    ],
                },
                "control_error": {
                    "epsilon_x_mm": 0.0,
                    "epsilon_y_mm": 500.0,
                    "theta_rad": 0.0,
                    "trajectory_valid": True
                }
            }
            
        mock_bridge.get_latest_outputs.side_effect = mock_get_latest_outputs
        
        yield mock_bridge


def test_scenario_runner_adapter_integration(mock_ros_bridge):
    """
    Test Phase 5 planning adapter integration in the simulator backend.
    Ensures ScenarioRunner can load a scene, run the step, and capture output
    planning metrics from the mock ROS bridge.
    """
    scenario = load_scenario_fixture("follow_main_straight.json")
    
    runner = ScenarioRunner()
    runner.load_scenario(scenario, mode="direct")
    
    # Run the step
    result = runner.step()
    
    # 1. Ensure bridge calls were made (adapter contract)
    mock_ros_bridge.publish_route_intent.assert_called_with("follow_main", seq=100)
    mock_ros_bridge.publish_telemetry.assert_called()
    mock_ros_bridge.wait_for_route_intent_ack.assert_called_with("follow_main", seq=100, timeout_sec=1.0)
    
    # 2. Assert step result structure holds planning metrics
    assert result["run_mode"] == "full"
    assert result["route_intent"]["intent"] == "follow_main"
    assert result["route_intent"]["seq"] == 100
    
    # 3. Verify lane_state fields are captured
    outputs = result["outputs"]
    lane_state = outputs["lane_state"]
    assert lane_state is not None
    assert lane_state["decision_state"] == "follow_main"
    assert lane_state["trajectory_valid"] is True
    assert lane_state["trajectory_kind"] == "follow_main"
    assert lane_state["replan_reason"] == "first_valid_trajectory"
    assert lane_state["normalization_mode"] == "no_previous_passthrough"
    
    # 4. Verify control_error fields are captured
    control_error = outputs["control_error"]
    assert control_error is not None
    assert control_error["trajectory_valid"] is True
    assert control_error["epsilon_y_mm"] == 500.0


def test_scenario_runner_handles_intent_sync_failure(mock_ros_bridge):
    """
    Test that if route_intent_ack times out (e.g. control_node is down),
    the step aborts early and reports the sync error without crashing.
    """
    scenario = load_scenario_fixture("follow_main_straight.json")
    
    runner = ScenarioRunner()
    runner.load_scenario(scenario, mode="direct")
    
    # Mock timeout
    mock_ros_bridge.wait_for_route_intent_ack.return_value = False
    
    result = runner.step()
    
    assert "error" in result
    assert "Timed out waiting for route intent" in result["error"]
    assert result["route_intent_sync"]["synced"] is False
    assert result["outputs"]["lane_state"] is None


def test_step_stores_lane_state_with_debug_trajectories_for_bev_overlay(mock_ros_bridge):
    """
    Regression guard for the debug trajectory BEV overlay (Phase 5): step() must
    surface lane_state (including debug_trajectories) into get_latest_ipm_output(),
    since that's what /api/ipm/bev reads to draw the candidate/normalized/committed
    overlay. If this regresses, the overlay silently draws nothing.
    """
    scenario = load_scenario_fixture("follow_main_straight.json")

    runner = ScenarioRunner()
    runner.load_scenario(scenario, mode="direct")
    runner.step()

    latest = runner.get_latest_ipm_output()
    assert latest["lane_state"] is not None
    assert latest["lane_state"]["control_source"] == "trajectory_manager"
    debug_trajectories = latest["lane_state"]["debug_trajectories"]
    assert len(debug_trajectories) == 1
    assert debug_trajectories[0]["stage"] == "committed"
    assert debug_trajectories[0]["points"] == [[0.0, 100.0], [0.0, 500.0], [0.0, 900.0]]


def test_step_ipm_reports_lane_state_none_not_missing_key(mock_ros_bridge):
    """
    step_ipm() (Phase 3, IPM-only) never runs control_node, so it must never
    surface a stale/mocked lane_state. get_latest_ipm_output()["lane_state"] must
    be explicitly None (not merely absent) so /api/ipm/bev's overlay can tell
    "no data because this used Step IPM" apart from a real lookup bug.
    """
    scenario = load_scenario_fixture("follow_main_straight.json")

    runner = ScenarioRunner()
    runner.load_scenario(scenario, mode="direct")
    runner.step_ipm()

    latest = runner.get_latest_ipm_output()
    assert "lane_state" in latest
    assert latest["lane_state"] is None


def test_running_frame_idx_reflects_frame_in_flight_not_next_frame(mock_ros_bridge):
    """
    Regression guard for the Frames panel "running" highlight (Phase 6). Naively
    reading current_frame_idx from get_status() during playback reports the frame
    *after* the one being processed, because step() increments current_frame_idx
    as soon as it finishes -- see review. running_frame_idx must instead equal the
    frame index that the just-completed step() actually processed.
    """
    scenario = load_scenario_fixture("follow_main_straight.json")

    runner = ScenarioRunner()
    runner.load_scenario(scenario, mode="direct")

    # get_status() only surfaces running_frame_idx while is_playing (matches the
    # frontend semantics: only highlight during Play, not after a manual Step).
    assert runner.get_status()["running_frame_idx"] is None

    runner.is_playing = True
    runner.step()

    assert runner.current_frame_idx == 1  # already advanced to the next frame
    assert runner.get_status()["running_frame_idx"] == 0  # frame actually processed

    runner.stop()
    assert runner.get_status()["running_frame_idx"] is None


def test_get_status_is_playing_and_running_frame_idx_never_contradict(mock_ros_bridge):
    """
    Regression guard: get_status() must derive is_playing and running_frame_idx
    from a single snapshot. Reading self.is_playing twice -- once (unlocked) to
    gate running_frame_idx, once more (inside _lock) for the response's
    is_playing field -- can race against stop()/pause() flipping the flag in
    between, producing a self-contradictory response like `is_playing: false`
    with a stale `running_frame_idx` still set. See review.
    """
    scenario = load_scenario_fixture("follow_main_straight.json")

    runner = ScenarioRunner()
    runner.load_scenario(scenario, mode="direct")
    runner.is_playing = True
    runner.step()
    assert runner._running_frame_idx == 0

    # Make every `self.is_playing` access return a different value in sequence,
    # simulating stop() flipping the flag concurrently. A correct get_status()
    # reads self.is_playing exactly once and reuses that snapshot for both
    # fields in the response, so it only ever sees the first value (True) here.
    reads = iter([True, False, False, False])

    class FlippingRunner(type(runner)):
        @property
        def is_playing(self):
            return next(reads, False)

        @is_playing.setter
        def is_playing(self, value):
            pass

    runner.__class__ = FlippingRunner

    status = runner.get_status()
    if status["is_playing"]:
        assert status["running_frame_idx"] is not None
    else:
        assert status["running_frame_idx"] is None
