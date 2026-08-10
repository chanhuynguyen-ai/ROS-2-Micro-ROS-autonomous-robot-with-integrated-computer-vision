import json
import os
import re
from pathlib import Path

from decision_harness import (
    PathObservationBuilder,
    TrajectoryPlanner,
    CommittedTrajectoryState,
    TrajectoryKind,
)

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
REPO_ROOT = Path(__file__).resolve().parents[2]
CONTROL_NODE = REPO_ROOT / "ros2_ws/src/avs_perception/src/control_node.cpp"
INCLUDE_DIR = REPO_ROOT / "ros2_ws/src/avs_perception/include/avs_perception"
# Plan D (D4) split decision_trajectory_core.hpp into decision_*/path_observation/
# trajectory_*/control_error_projector headers; glob them all so a further split
# never silently drops code these structural regex checks look at.
CORE_HEADER_PATTERNS = ["decision_*.hpp", "trajectory_*.hpp", "path_observation.hpp", "control_error_projector.hpp", "legacy_lane_model.hpp"]


def load_fixture(name: str) -> list[dict]:
    path = os.path.join(FIXTURES_DIR, name)
    with open(path, "r") as f:
        return json.load(f)


def read_control_node() -> str:
    text = CONTROL_NODE.read_text(encoding="utf-8")
    for pattern in CORE_HEADER_PATTERNS:
        for header in sorted(INCLUDE_DIR.glob(pattern)):
            text += header.read_text(encoding="utf-8")
    return text


def _selected_ids_across_frames(fixture_name: str, is_turn_right: bool) -> list[str]:
    frames = load_fixture(fixture_name)
    ids = []
    for telemetry in frames:
        obs = PathObservationBuilder.build(telemetry)
        selected = TrajectoryPlanner.select_turn_lane_obs(obs, is_turn_right=is_turn_right, is_t_junction=False)
        assert selected is not None
        ids.append(selected.lane_id)
    return ids


def extract_function_body(source: str, signature_marker: str) -> str:
    """Extract a function body by brace-counting from its signature marker
    to the matching closing brace. Assumes signature_marker uniquely
    identifies the start of the function in the file."""
    start = source.index(signature_marker)
    brace_start = source.index("{", start)
    depth = 0
    i = brace_start
    while i < len(source):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                return source[start:i + 1]
        i += 1
    raise AssertionError(f"unbalanced braces extracting function at: {signature_marker!r}")


# ---------------------------------------------------------------------------
# R10: select_other_lane_obs/select_other_lane must pick the lane nearest to
# main (absolute lateral distance), not the one closest to an assumed 800mm
# lane width. Regression test for the approved Plan B fix.
# ---------------------------------------------------------------------------
def test_other_lane_nearest_absolute_selection():
    telemetry = {
        "timestamp_ms": 1000,
        "objects": [
            {
                "id": "main_lane",
                "label": 6,
                "class_name": "main-lane",
                "confidence": 0.9,
                "waypoints": [[0.0, 0.0], [0.0, 1000.0], [0.0, 2000.0]],
            },
            {
                "id": "other_near",
                "label": 7,
                "class_name": "other-lane",
                "confidence": 0.85,
                # 700mm to the right: nearer to main than the 800mm heuristic target
                "waypoints": [[700.0, 0.0], [700.0, 1000.0], [700.0, 2000.0]],
            },
            {
                "id": "other_far",
                "label": 7,
                "class_name": "other-lane",
                "confidence": 0.85,
                # 900mm to the right: under the old -abs(d-800) scoring this tied
                # with other_near; under nearest-absolute it must lose.
                "waypoints": [[900.0, 0.0], [900.0, 1000.0], [900.0, 2000.0]],
            },
        ],
    }
    obs = PathObservationBuilder.build(telemetry)
    selected = TrajectoryPlanner.select_other_lane_obs(obs, obs.lanes[0], is_left_change=False)
    assert selected is not None
    assert selected.lane_id == "other_near"


# ---------------------------------------------------------------------------
# R7: a solid marking that sits outside the lane-change corridor (ahead of the
# transition's y-range) must NOT block the lane change.
# ---------------------------------------------------------------------------
def test_lane_change_solid_outside_corridor_does_not_block():
    frames = load_fixture("lane_change_solid_outside_corridor.json")
    obs = PathObservationBuilder.build(frames[0])
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]

    planned = TrajectoryPlanner.plan_lane_change_left(obs, prev_state, last_main_id)
    assert not planned.blocked_by_marking
    assert planned.trajectory_kind == TrajectoryKind.LANE_CHANGE_LEFT
    assert planned.target_lane_id == "other_lane"


# ---------------------------------------------------------------------------
# marking_confidence_low: when no solid/dashed marking is detected at all in
# the corridor, the lane change stays allowed (default policy) but the plan
# must flag low confidence so it is visible in /avs/lane_state debug output.
# ---------------------------------------------------------------------------
def test_marking_confidence_low_when_no_marking_detected():
    frames = load_fixture("lane_change_no_marking_between_lanes.json")
    obs = PathObservationBuilder.build(frames[0])
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]

    planned = TrajectoryPlanner.plan_lane_change_left(obs, prev_state, last_main_id)
    assert not planned.blocked_by_marking
    assert planned.trajectory_kind == TrajectoryKind.LANE_CHANGE_LEFT
    assert planned.marking_confidence_low


def test_marking_confidence_low_false_when_solid_marking_detected():
    frames = load_fixture("lane_change_solid_blocked.json")
    obs = PathObservationBuilder.build(frames[0])
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]

    planned = TrajectoryPlanner.plan_lane_change_left(obs, prev_state, last_main_id)
    assert planned.blocked_by_marking
    # A marking WAS found (it's what caused the block) - confidence should not
    # be flagged low even though the resulting plan falls back to follow_main.
    assert not planned.marking_confidence_low


# ---------------------------------------------------------------------------
# R3: T-junction detection is purely geometric (no main-ahead + wide turn-lane
# spread + proximity) and must never reference stop-line.
# ---------------------------------------------------------------------------
def test_t_junction_detection_never_references_stop_line():
    source = read_control_node()
    body = extract_function_body(source, "bool detect_t_junction(const LaneCandidate* main_current,")
    assert "STOP_LINE" not in body
    assert "stop_line" not in body.lower()
    assert "main_current && !main_ahead" in body
    assert "max_turn_x - min_turn_x > 2000.0" in body
    assert "std::abs(main_end_y - avg_turn_start_y) < 1500.0" in body
    assert "t_junction_counter_ >= 3" in body


# ---------------------------------------------------------------------------
# Duplicate helper-pair consistency: select_turn_lane_obs/select_turn_lane and
# select_other_lane_obs/select_other_lane reimplement the same business rules
# (R1/R2/R10) against two different data representations (LaneObservation vs
# LaneCandidate). This locks their gate thresholds and scoring formulas
# together so an edit to one copy that's forgotten in the other fails CI
# instead of silently drifting.
# ---------------------------------------------------------------------------
def test_turn_lane_helper_pair_thresholds_match():
    source = read_control_node()
    obs_body = extract_function_body(
        source, "static const LaneObservation* select_turn_lane_obs(const PathObservationFrame& obs,"
    )
    cand_body = extract_function_body(
        source, "const LaneCandidate* select_turn_lane(const std::vector<LaneCandidate>& lanes,"
    )

    for body in (obs_body, cand_body):
        assert "avg_x < -200.0" in body
        assert "avg_x > 200.0" in body
        assert "return scored_lanes.front().second; // closest" in body
        assert "return scored_lanes.back().second;  // farthest" in body
        # R1/R2 near/far metric must be nearest-point distance, not average-x
        # (roadmap Phase 4: average-x is unreliable for strongly curved lanes).
        assert "double dist = std::sqrt(x*x + y*y);" in body or "double dist = std::sqrt(pt.x*pt.x + pt.y*pt.y);" in body
        assert "if (dist < min_dist) min_dist = dist;" in body
        assert "scored_lanes.push_back({min_dist, l});" in body


def test_other_lane_helper_pair_thresholds_match():
    source = read_control_node()
    obs_body = extract_function_body(
        source, "static const LaneObservation* select_other_lane_obs(const PathObservationFrame& obs,"
    )
    cand_body = extract_function_body(
        source, "const LaneCandidate* select_other_lane(const std::vector<LaneCandidate>& lanes,"
    )

    for body in (obs_body, cand_body):
        assert "lateral_dist > -200.0" in body
        assert "lateral_dist < 200.0" in body
        assert "30.0 * M_PI / 180.0" in body
        assert "abs_lat_dist < 400.0 || abs_lat_dist > 1400.0" in body
        assert "min_y > 1200.0" in body
        # R10: nearest-absolute scoring, no assumed lane-width bias
        assert "score = -abs_lat_dist - 1000.0 * diff_theta;" in body
        assert "800.0" not in re.sub(r"//.*", "", body)


# ---------------------------------------------------------------------------
# R1/R2: turn-lane selection (nearer for right, farther for left) must stay
# stable across consecutive frames with small position jitter, not flicker
# between the two candidates frame-to-frame.
# ---------------------------------------------------------------------------
def test_turn_right_two_lanes_same_side_picks_nearer_stably():
    ids = _selected_ids_across_frames("turn_right_two_lanes_same_side.json", is_turn_right=True)
    assert len(ids) == 5
    assert all(lane_id == "turn_lane_closer" for lane_id in ids)


def test_turn_left_two_lanes_same_side_picks_farther_stably():
    ids = _selected_ids_across_frames("turn_left_two_lanes_same_side.json", is_turn_right=False)
    assert len(ids) == 5
    assert all(lane_id == "turn_lane_further" for lane_id in ids)


def test_turn_lane_selection_stable_under_jitter():
    ids = _selected_ids_across_frames("turn_lane_selection_jitter.json", is_turn_right=True)
    assert len(ids) == 6
    assert all(lane_id == "turn_lane_closer" for lane_id in ids)


# ---------------------------------------------------------------------------
# R9: follow_main through an intersection must connect current main-lane to
# main-ahead with one smooth line only when main-ahead is actually visible.
# When only the current (abruptly-ending) main-lane is detected, the planner
# must not guess/extend a far-connect segment beyond it.
# ---------------------------------------------------------------------------
def test_follow_main_no_far_connect_without_main_ahead():
    telemetry = {
        "timestamp_ms": 1000,
        "objects": [
            {
                "id": "main_current",
                "label": 6,
                "class_name": "main-lane",
                "confidence": 0.9,
                # Lane ends abruptly at y=800mm (e.g. occluded/at the edge of FOV) -
                # no main-ahead object is present in this frame.
                "waypoints": [[0.0, 0.0], [0.0, 400.0], [0.0, 800.0]],
            }
        ],
    }
    obs = PathObservationBuilder.build(telemetry)
    assert len(obs.lanes) == 1

    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    planned = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)

    assert planned.valid
    assert planned.target_lane_id == "main_current"
    # Raw path must not extend past the current lane's own last waypoint (y=800mm) -
    # no guessed far-connect segment when main-ahead isn't visible.
    assert max(p.y for p in planned.points) <= 800.0 + 1e-6
