import json
import os
import pytest
from decision_harness import (
    Point2D,
    TrajectoryKind,
    trajectory_kind_name,
    PathObservationBuilder,
    TrajectoryPlanner,
    TrajectoryNormalizer,
    TrajectoryManager,
    CommittedTrajectoryState,
    PlannedTrajectory,
    ManagerAction
)

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")

def load_fixture(name: str) -> list[dict]:
    path = os.path.join(FIXTURES_DIR, name)
    with open(path, "r") as f:
        return json.load(f)


def test_follow_main_straight():
    frames = load_fixture("follow_main_straight.json")
    assert len(frames) == 1
    
    # Frame 1: First valid trajectory commit
    telemetry = frames[0]
    obs = PathObservationBuilder.build(telemetry)
    assert len(obs.lanes) == 1
    assert obs.lanes[0].lane_id == "main_lane_1"
    
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    
    planned = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
    assert planned.valid
    assert planned.trajectory_kind == TrajectoryKind.FOLLOW_MAIN
    assert planned.target_lane_id == "main_lane_1"
    assert last_main_id[0] == "main_lane_1"
    
    normalized = TrajectoryNormalizer.normalize(planned, prev_state)
    assert normalized.normalization_mode == "no_previous_passthrough"
    
    consecutive_invalid = [0]
    decision = TrajectoryManager.update(normalized, prev_state, "follow_main", consecutive_invalid, 1)
    
    assert decision.action == ManagerAction.COMMIT_NEW
    assert decision.reason == "first_valid_trajectory"
    assert decision.next_state.trajectory.valid
    assert decision.next_state.remaining_s_mm > 3000.0
    
    # Frame 2: Stable straight driving (HOLD_CURRENT)
    prev_state2 = decision.next_state
    planned2 = TrajectoryPlanner.plan_follow_main(obs, prev_state2, last_main_id)
    normalized2 = TrajectoryNormalizer.normalize(planned2, prev_state2)
    assert normalized2.normalization_mode == "temporal_blend"
    
    decision2 = TrajectoryManager.update(normalized2, prev_state2, "follow_main", consecutive_invalid, 2)
    assert decision2.action == ManagerAction.HOLD_CURRENT
    assert decision2.reason == "deviation_below_threshold"
    assert decision2.next_state.replan_reason == "hold_due_to_low_deviation"


def test_follow_main_curve():
    frames = load_fixture("follow_main_curve.json")
    telemetry = frames[0]
    obs = PathObservationBuilder.build(telemetry)
    
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    
    # Frame 1: Commit new curve
    planned = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
    assert planned.valid
    normalized = TrajectoryNormalizer.normalize(planned, prev_state)
    
    consecutive_invalid = [0]
    decision = TrajectoryManager.update(normalized, prev_state, "follow_main", consecutive_invalid, 1)
    assert decision.action == ManagerAction.COMMIT_NEW
    
    # Frame 2: Blend next frame with curve
    prev_state2 = decision.next_state
    planned2 = TrajectoryPlanner.plan_follow_main(obs, prev_state2, last_main_id)
    normalized2 = TrajectoryNormalizer.normalize(planned2, prev_state2)
    assert normalized2.normalization_mode == "temporal_blend"
    
    # Check that points are blended properly
    assert len(normalized2.points) == len(planned2.points)


def test_follow_main_intersection_merge():
    frames = load_fixture("follow_main_intersection.json")
    telemetry = frames[0]
    obs = PathObservationBuilder.build(telemetry)
    
    assert len(obs.lanes) == 2  # main_current and main_ahead
    
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    
    planned = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
    assert planned.valid
    assert planned.target_lane_id == "main_current"
    # Merged path should contain waypoints from both lanes, length > 3000mm
    assert len(planned.points) > 5
    assert last_main_id[0] == "main_current"


def test_turn_right_closest_selection():
    frames = load_fixture("turn_right_two_lanes.json")
    telemetry = frames[0]
    obs = PathObservationBuilder.build(telemetry)
    
    # Verify that closest turn lane is selected for right turn
    selected = TrajectoryPlanner.select_turn_lane_obs(obs, is_turn_right=True, is_t_junction=False)
    assert selected is not None
    assert selected.lane_id == "turn_lane_closer"


def test_turn_left_farthest_selection():
    frames = load_fixture("turn_left_two_lanes.json")
    telemetry = frames[0]
    obs = PathObservationBuilder.build(telemetry)
    
    # Verify that farthest turn lane is selected for left turn
    selected = TrajectoryPlanner.select_turn_lane_obs(obs, is_turn_right=False, is_t_junction=False)
    assert selected is not None
    assert selected.lane_id == "turn_lane_further"


def test_lane_change_solid_blocked():
    frames = load_fixture("lane_change_solid_blocked.json")
    telemetry = frames[0]
    obs = PathObservationBuilder.build(telemetry)
    
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    
    # Try left lane change
    planned = TrajectoryPlanner.plan_lane_change_left(obs, prev_state, last_main_id)
    # Because of solid line, the planner should set blocked_by_marking = True and fallback to follow_main
    assert planned.blocked_by_marking
    assert planned.trajectory_kind == TrajectoryKind.FOLLOW_MAIN
    assert planned.target_lane_id == "main_lane"


def test_lane_change_dashed_allowed():
    frames = load_fixture("lane_change_dashed_allowed.json")
    telemetry = frames[0]
    obs = PathObservationBuilder.build(telemetry)
    
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    
    # Try left lane change
    planned = TrajectoryPlanner.plan_lane_change_left(obs, prev_state, last_main_id)
    # Because of dashed line, lane change is allowed and plans a transition
    assert not planned.blocked_by_marking
    assert planned.trajectory_kind == TrajectoryKind.LANE_CHANGE_LEFT
    assert planned.target_lane_id == "other_lane"


def test_dropout_and_recovery_sequence():
    frames = load_fixture("dropout_and_recovery.json")
    assert len(frames) == 7
    
    # Frame 1: Valid initial commit
    telemetry = frames[0]
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    
    planned = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
    normalized = TrajectoryNormalizer.normalize(planned, prev_state)
    consecutive_invalid = [0]
    decision = TrajectoryManager.update(normalized, prev_state, "follow_main", consecutive_invalid, 1)
    assert decision.action == ManagerAction.COMMIT_NEW
    assert decision.next_state.trajectory.valid
    
    # Frames 2 to 6: 5 frames of transient dropout (HOLD_CURRENT via planner memory fallback)
    current_state = decision.next_state
    for frame_idx in range(2, 7):
        telemetry = frames[frame_idx - 1]  # 0-indexed list
        obs = PathObservationBuilder.build(telemetry)
        
        # Planner falls back to holding previous committed trajectory during dropout
        planned = TrajectoryPlanner.plan_follow_main(obs, current_state, last_main_id)
        normalized = TrajectoryNormalizer.normalize(planned, current_state)
        
        decision = TrajectoryManager.update(normalized, current_state, "follow_main", consecutive_invalid, frame_idx)
        # Because the planner memory fallback succeeded, the candidate remains valid and identical
        assert decision.action == ManagerAction.HOLD_CURRENT
        assert decision.reason == "deviation_below_threshold"
        assert decision.next_state.replan_reason == "hold_due_to_low_deviation"
        assert decision.next_state.trajectory.valid  # INVARIANT: Trajectory remains valid during dropout!
        current_state = decision.next_state
        
    # Frame 7: Recovery (Valid lane returns)
    telemetry = frames[6]
    obs = PathObservationBuilder.build(telemetry)
    planned = TrajectoryPlanner.plan_follow_main(obs, current_state, last_main_id)
    normalized = TrajectoryNormalizer.normalize(planned, current_state)
    
    decision = TrajectoryManager.update(normalized, current_state, "follow_main", consecutive_invalid, 7)
    # Because deviation is 10mm (which is < 50mm), action is HOLD_CURRENT (deviation below threshold)
    assert decision.action == ManagerAction.HOLD_CURRENT
    assert decision.reason == "deviation_below_threshold"
    assert decision.next_state.dropout_hold_counter == 0
    assert decision.next_state.replan_reason == "hold_due_to_low_deviation"
    assert decision.next_state.trajectory.valid


def test_transient_dropout_hold_with_invalid_candidate():
    # Verify Case 2: New candidate is invalid (e.g. no memory fallback available or planning failed)
    prev_state = CommittedTrajectoryState()
    prev_state.trajectory = PlannedTrajectory(valid=True, points=[Point2D(0, 0), Point2D(0, 1000)])
    
    consecutive_invalid = [0]
    invalid_candidate = PlannedTrajectory(valid=False)
    
    # Frame 1 of invalid candidate -> HOLD_CURRENT with transient_dropout_hold
    decision = TrajectoryManager.update(invalid_candidate, prev_state, "follow_main", consecutive_invalid, 1)
    assert decision.action == ManagerAction.HOLD_CURRENT
    assert decision.reason == "transient_dropout_hold"
    assert decision.next_state.dropout_hold_counter == 1
    assert decision.next_state.replan_reason == "hold_due_to_dropout"
    assert decision.next_state.trajectory.valid  # INVARIANT: Stale path is still held


def test_persistent_dropout_clearing():
    # Simulate a 6th frame of dropout to verify persistent dropout recovery trigger
    prev_state = CommittedTrajectoryState()
    prev_state.trajectory = PlannedTrajectory(valid=True, points=[Point2D(0, 0), Point2D(0, 1000)])
    
    consecutive_invalid = [5]  # already had 5 invalid frames
    invalid_candidate = PlannedTrajectory(valid=False)
    
    decision = TrajectoryManager.update(invalid_candidate, prev_state, "follow_main", consecutive_invalid, 10)
    assert decision.action == ManagerAction.ENTER_RECOVERY
    assert decision.reason == "persistent_dropout_clear"
    assert not decision.next_state.trajectory.valid
    assert decision.next_state.replan_reason == "persistent_invalid_clear"


def test_precomputed_only_main_lane_no_crash():
    # Verify that a main lane with precomputed control and no waypoints does not crash the planner
    telemetry = {
        "timestamp_ms": 1000,
        "objects": [
            {
                "id": "precomputed_main",
                "label": 6,
                "class_name": "main-lane",
                "confidence": 0.9,
                "waypoints": [],
                "lookahead_x_mm": 0.0,
                "lookahead_d_mm": 800.0
            }
        ]
    }
    obs = PathObservationBuilder.build(telemetry)
    assert len(obs.lanes) == 1
    assert obs.lanes[0].has_precomputed_control
    
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    
    # This call must NOT crash and must return a valid precomputed plan
    planned = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
    assert planned.valid
    assert planned.has_precomputed_control
    assert planned.precomputed_lookahead_d_mm == 800.0
    assert planned.trajectory_kind == TrajectoryKind.FOLLOW_MAIN


def test_impossible_turn_transition_rejected():
    # Verify that an impossible turn transition (outside limits) falls back to FOLLOW_MAIN
    telemetry = {
        "timestamp_ms": 1000,
        "objects": [
            {
                "id": "main_lane",
                "label": 6,
                "class_name": "main-lane",
                "confidence": 0.9,
                "waypoints": [[0.0, 0.0], [0.0, 1000.0]]
            },
            {
                "id": "turn_lane_far_out",
                "label": 20,
                "class_name": "turn-lane",
                "confidence": 0.8,
                "waypoints": [[2000.0, 0.0], [2000.0, 1000.0]]  # lat_dist = 2000mm (outside [300, 1500] limit)
            }
        ]
    }
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    
    planned = TrajectoryPlanner.plan_turn_right(obs, prev_state, is_t=False, t_junction_pending=False, last_main_id=last_main_id)
    # The planner must reject the transition and fallback to FOLLOW_MAIN
    assert planned.valid
    assert planned.trajectory_kind == TrajectoryKind.FOLLOW_MAIN
    assert planned.target_lane_id == "main_lane"


def test_shared_lane_turn_transition_preserved():
    # Verify that an overlapping shared-lane turn transition (lat_dist < 300) is preserved as TURN
    telemetry = {
        "timestamp_ms": 1000,
        "objects": [
            {
                "id": "main_lane",
                "label": 6,
                "class_name": "main-lane",
                "confidence": 0.9,
                "waypoints": [[0.0, 0.0], [0.0, 1000.0]]
            },
            {
                "id": "turn_lane_overlapping",
                "label": 20,
                "class_name": "turn-lane",
                "confidence": 0.8,
                "waypoints": [[100.0, 0.0], [100.0, 1000.0]]  # lat_dist = 100mm (below 300mm limit, overlapping)
            }
        ]
    }
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    
    planned = TrajectoryPlanner.plan_turn_right(obs, prev_state, is_t=False, t_junction_pending=False, last_main_id=last_main_id)
    # The planner must preserve the transition as a valid TURN maneuver
    assert planned.valid
    assert planned.trajectory_kind == TrajectoryKind.TURN_RIGHT
    assert planned.target_lane_id == "turn_lane_overlapping"
    # INVARIANT: The geometry must preserve the current-lane prefix near ego (X = 0.0)
    assert abs(planned.points[0].x - 0.0) < 1.0
    # INVARIANT: The geometry must smoothly transition to the target turn lane at the end (X = 100.0)
    assert len(planned.points) >= 2
    assert abs(planned.points[-1].x - 100.0) < 1.0


