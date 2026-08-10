from __future__ import annotations

from math import atan2
from pathlib import Path

from decision_harness import (
    LABEL_DASHED_WHITE,
    LABEL_MAIN_LANE,
    LABEL_OTHER_LANE,
    LABEL_SOLID_WHITE,
    LABEL_TURN_LANE,
    build_trajectory_from_candidate_current,
    connect_two_lanes_smooth_current,
    control_error_from_trajectory_current,
    extract_lane_candidates,
    extract_marking_candidates,
    is_lane_change_blocked_by_solid_current,
    lane,
    marking,
    polygon_x_range,
    select_other_lane_current,
    select_turn_lane_current,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
CONTROL_NODE = REPO_ROOT / "ros2_ws/src/avs_perception/src/control_node.cpp"
INCLUDE_DIR = REPO_ROOT / "ros2_ws/src/avs_perception/include/avs_perception"
# Plan D (D4) split decision_trajectory_core.hpp into decision_*/path_observation/
# trajectory_*/control_error_projector headers; glob them all so a further split
# never silently drops code these structural regex checks look at.
CORE_HEADER_PATTERNS = ["decision_*.hpp", "trajectory_*.hpp", "path_observation.hpp", "control_error_projector.hpp", "legacy_lane_model.hpp"]


def read_control_node() -> str:
    text = CONTROL_NODE.read_text(encoding="utf-8")
    for pattern in CORE_HEADER_PATTERNS:
        for header in sorted(INCLUDE_DIR.glob(pattern)):
            text += header.read_text(encoding="utf-8")
    return text


def candidate_by_id(candidates, object_id: str):
    return next(c for c in candidates if c.raw_obj["id"] == object_id)


def test_phase1_uses_route_intent_topic_and_does_not_use_cmd_for_route_decisions():
    source = read_control_node()

    assert '"/avs/route_intent"' in source
    assert 'cmd == "turn"' not in source
    assert 'cmd == "lane_change"' not in source


def test_phase1_has_no_stop_line_gate_in_state_transition():
    source = read_control_node()
    transition_body = source.split("void update_lane_state", 1)[1].split(
        "void publish_control_error_from_trajectory", 1
    )[0]

    assert "stop_line_detected ||" not in transition_body
    assert "turn_cued" not in transition_body


def test_phase2_single_point_trajectory_is_invalid():
    main = lane("main", LABEL_MAIN_LANE, [(0.0, 100.0)])
    traj = build_trajectory_from_candidate_current(
        extract_lane_candidates({"objects": [main]})[0]
    )

    assert not traj.valid, "A controller trajectory needs at least two points."


def test_phase2_curvature_is_not_hardcoded_zero_in_control_error():
    curved = lane(
        "curved-main",
        LABEL_MAIN_LANE,
        [(-40.0, 100.0), (-10.0, 300.0), (60.0, 600.0), (160.0, 900.0)],
    )
    traj = build_trajectory_from_candidate_current(
        extract_lane_candidates({"objects": [curved]})[0]
    )
    error = control_error_from_trajectory_current(traj, 600.0)

    assert error["curvature_inv_mm"] != 0.0


def test_phase2_lookahead_distance_is_measured_from_vehicle_origin():
    visible_lane = lane(
        "visible-main",
        LABEL_MAIN_LANE,
        [(0.0, 300.0), (0.0, 900.0)],
    )
    traj = build_trajectory_from_candidate_current(
        extract_lane_candidates({"objects": [visible_lane]})[0]
    )
    error = control_error_from_trajectory_current(traj, 600.0)

    assert error["epsilon_x_mm"] == 0.0
    assert error["epsilon_y_mm"] == 600.0


def test_phase2_lookahead_only_lane_preserves_legacy_control_fields():
    legacy_lane = lane(
        "legacy-main",
        LABEL_MAIN_LANE,
        [],
        lookahead_x_mm=120.0,
        lookahead_d_mm=600.0,
    )
    legacy_lane["lookahead_theta_rad"] = 0.2
    legacy_lane["curvature_inv_mm"] = 0.001

    traj = build_trajectory_from_candidate_current(
        extract_lane_candidates({"objects": [legacy_lane]})[0]
    )
    error = control_error_from_trajectory_current(traj, 600.0)

    assert traj.valid
    assert traj.has_precomputed_control
    assert traj.points == []
    assert traj.trajectory_kind == "precomputed_main_lane"
    assert error["epsilon_x_mm"] == 120.0
    assert error["epsilon_y_mm"] == 600.0
    assert error["theta_rad"] == 0.2
    assert error["curvature_inv_mm"] == 0.001


def test_phase2_theta_is_angle_from_origin_to_lookahead_point():
    offset_lane = lane(
        "offset-main",
        LABEL_MAIN_LANE,
        [(200.0, 300.0), (200.0, 900.0)],
    )
    traj = build_trajectory_from_candidate_current(
        extract_lane_candidates({"objects": [offset_lane]})[0]
    )
    error = control_error_from_trajectory_current(traj, 600.0)

    assert error["epsilon_x_mm"] == 200.0
    assert error["epsilon_y_mm"] > 0.0
    assert round(error["theta_rad"], 6) == round(
        atan2(error["epsilon_x_mm"], error["epsilon_y_mm"]),
        6,
    )
    assert error["theta_rad"] != 0.0


def test_phase3_follow_main_connection_produces_one_active_trajectory():
    current = lane("main-current", LABEL_MAIN_LANE, [(0.0, 100.0), (0.0, 500.0)])
    ahead = lane("main-ahead", LABEL_MAIN_LANE, [(10.0, 900.0), (10.0, 1300.0)])
    lanes = extract_lane_candidates({"objects": [current, ahead]})

    traj = connect_two_lanes_smooth_current(lanes[0], lanes[1])

    assert traj.valid
    assert traj.source_labels == [LABEL_MAIN_LANE, LABEL_MAIN_LANE]
    assert len(traj.points) > len(current["waypoints"]) + len(ahead["waypoints"])


def test_phase4_turn_right_filters_to_right_side_before_near_far_selection():
    # Left turn-lane is closer to the vehicle, but turn_right must still select
    # the lane on the right side of the BEV frame.
    left_near = lane(
        "left-near",
        LABEL_TURN_LANE,
        [(-80.0, 80.0), (-220.0, 120.0), (-360.0, 160.0)],
    )
    right_far = lane(
        "right-far",
        LABEL_TURN_LANE,
        [(260.0, 180.0), (400.0, 220.0), (540.0, 260.0)],
    )
    lanes = extract_lane_candidates({"objects": [left_near, right_far]})

    selected = select_turn_lane_current(
        lanes,
        is_turn_right=True,
        is_t_junction=False,
    )

    assert selected is not None
    assert selected.raw_obj["id"] == "right-far"


def test_phase4_t_junction_signal_is_used_by_turn_lane_selection():
    source = read_control_node()
    function_body = source.split("const LaneCandidate* select_turn_lane", 1)[1].split(
        "const LaneCandidate* select_other_lane", 1
    )[0]

    assert function_body.count("is_t_junction") > 1


def test_phase5_select_other_lane_left_and_right_by_lateral_side():
    main = lane("main", LABEL_MAIN_LANE, [(0.0, 100.0), (0.0, 700.0)], lookahead_x_mm=0.0)
    left = lane("left", LABEL_OTHER_LANE, [(-240.0, 100.0), (-240.0, 700.0)], lookahead_x_mm=-240.0)
    right = lane("right", LABEL_OTHER_LANE, [(240.0, 100.0), (240.0, 700.0)], lookahead_x_mm=240.0)
    lanes = extract_lane_candidates({"objects": [main, left, right]})
    main_cand = candidate_by_id(lanes, "main")

    assert select_other_lane_current(lanes, main_cand, True).raw_obj["id"] == "left"
    assert select_other_lane_current(lanes, main_cand, False).raw_obj["id"] == "right"


def test_phase5_solid_polygon_between_lanes_blocks_lane_change():
    main = lane("main", LABEL_MAIN_LANE, [(0.0, 100.0), (0.0, 700.0)], lookahead_x_mm=0.0)
    target = lane("right", LABEL_OTHER_LANE, [(240.0, 100.0), (240.0, 700.0)], lookahead_x_mm=240.0)
    solid = marking(
        "solid-mid",
        LABEL_SOLID_WHITE,
        polygons_real_world=[[(115.0, 50.0), (125.0, 50.0), (125.0, 700.0), (115.0, 700.0)]],
    )
    telemetry = {"objects": [main, target, solid]}
    lanes = extract_lane_candidates(telemetry)
    markings = extract_marking_candidates(telemetry)

    assert polygon_x_range(solid) == (115.0, 125.0)
    assert is_lane_change_blocked_by_solid_current(lanes[0], lanes[1], markings)


def test_phase5_dashed_polygon_between_lanes_does_not_block_lane_change():
    main = lane("main", LABEL_MAIN_LANE, [(0.0, 100.0), (0.0, 700.0)], lookahead_x_mm=0.0)
    target = lane("right", LABEL_OTHER_LANE, [(240.0, 100.0), (240.0, 700.0)], lookahead_x_mm=240.0)
    dashed = marking(
        "dashed-mid",
        LABEL_DASHED_WHITE,
        polygons_real_world=[[(115.0, 50.0), (125.0, 50.0), (125.0, 700.0), (115.0, 700.0)]],
    )
    telemetry = {"objects": [main, target, dashed]}
    lanes = extract_lane_candidates(telemetry)
    markings = extract_marking_candidates(telemetry)

    assert not is_lane_change_blocked_by_solid_current(lanes[0], lanes[1], markings)


def test_debug_detection_flags_are_not_hardcoded_true():
    source = read_control_node()

    assert "true, // has_other (simplified)" not in source
    assert "true, // has_turn (simplified)" not in source


def test_lane_state_debug_keeps_legacy_lane_state_key():
    source = read_control_node()
    publish_body = source.split("void publish_lane_state", 1)[1].split(
        "// ── ROS2 interfaces", 1
    )[0]

    assert 'state_json["decision_state"]' in publish_body
    assert 'state_json["lane_state"]' in publish_body


def test_selected_lane_id_tolerates_non_string_json_ids():
    source = read_control_node()

    assert "std::string lane_id_string" in source
    assert "return id.dump();" in source
    assert 'raw_obj["id"].get<std::string>()' not in source


def test_control_error_does_not_depend_on_y_sorted_lower_bound_for_all_trajectories():
    source = read_control_node()

    assert "std::lower_bound(traj.points.begin(), traj.points.end(), lookahead_d_mm" not in source


def test_t_junction_detection_is_evaluated_once_per_telemetry_callback():
    source = read_control_node()
    telemetry_body = source.split("void telemetry_callback", 1)[1].split(
        "ActiveTrajectory connect_two_lanes_smooth", 1
    )[0]

    assert telemetry_body.count("detect_t_junction(") == 1
    assert "bool t_junction_pending = is_t_geom && !is_t;" in telemetry_body
    assert "plan_candidate_for_intent(" in telemetry_body
    assert "is_t, t_junction_pending" in telemetry_body


def test_t_junction_pending_holds_off_turn_commit():
    # An ambiguous T-junction geometry (is_t_geom true, is_t not yet confirmed)
    # must still hold the turn maneuver back to follow_main - this was a real
    # behavioral gate pre-Plan-A-refactor (state_ forced to FOLLOW_MAIN), not
    # just an artifact of the old switch-based code. The refactor threaded
    # t_junction_pending through plan_candidate_for_intent/plan_turn_right/
    # plan_turn_left as a dead parameter (never read by plan_turn_generic),
    # silently dropping the gate; it must be re-applied as an explicit fallback.
    source = read_control_node()
    telemetry_body = source.split("void telemetry_callback", 1)[1]

    assert "t_junction_pending && main_current" in telemetry_body
    assert 'hold_reason_ = "t_junction_pending"' in telemetry_body


def test_t_junction_turn_blocking_uses_confirmed_t_signal():
    source = read_control_node()
    telemetry_body = source.split("void telemetry_callback", 1)[1]

    assert "current_intent_ == RouteIntent::TURN_LEFT && is_t && planned_candidate.valid" in telemetry_body
    assert "is_t_geom && planned_candidate.valid" not in telemetry_body


def test_turn_solid_gate_uses_segment_intersection_not_wide_proximity():
    source = read_control_node()
    helper_body = source.split("bool is_turn_blocked_by_solid", 1)[1].split(
        "ActiveTrajectory build_trajectory_from_candidate", 1
    )[0]

    assert "segments_intersect" in helper_body
    assert "std::hypot(p.x - wx, p.y - wy) < 500.0" not in helper_body


def test_failed_transition_is_not_reported_as_successful_maneuver():
    source = read_control_node()
    transition_body = source.split("ActiveTrajectory transition_to_lane", 1)[1].split(
        "// ── Lane state transition logic", 1
    )[0]
    planner_body = source.split("class TrajectoryPlanner", 1)[1].split(
        "class TrajectoryNormalizer", 1
    )[0]

    assert 'invalid.trajectory_kind = "invalid_transition";' in transition_body
    assert "return traj;" not in transition_body.split("if (traj.points.size() < 2 || traj_target.points.size() < 2)", 1)[1].split("}", 1)[0]
    assert "plan = plan_follow_main(obs, prev_state, last_main_id);" in planner_body


def test_phase1_resume_command_restores_follow_main():
    source = read_control_node()
    cmd_body = source.split("void cmd_callback", 1)[1].split(
        "void telemetry_callback", 1
    )[0]
    assert "current_intent_ = RouteIntent::FOLLOW_MAIN;" in cmd_body
    assert 'cmd == "resume"' in cmd_body


def test_phase4_precomputed_turn_lane_selection():
    legacy_turn = {
        "id": "legacy-turn",
        "label": LABEL_TURN_LANE,
        "class_name": "turn-lane",
        "longitudinal_offset_mm": 500.0,
        "lookahead_theta_rad": 0.3,
    }
    lanes = extract_lane_candidates({"objects": [legacy_turn]})
    selected = select_turn_lane_current(
        lanes,
        is_turn_right=True,
        is_t_junction=False,
    )
    assert selected is not None
    assert selected.raw_obj["id"] == "legacy-turn"


def test_phase2_precomputed_turn_lane_preserves_legacy_fields():
    legacy_turn = {
        "id": "legacy-turn",
        "label": LABEL_TURN_LANE,
        "class_name": "turn-lane",
        "longitudinal_offset_mm": 500.0,
        "lookahead_theta_rad": -0.2,
    }
    traj = build_trajectory_from_candidate_current(
        extract_lane_candidates({"objects": [legacy_turn]})[0]
    )
    assert traj.valid
    assert traj.has_precomputed_control
    assert traj.points == []
    assert traj.trajectory_kind == "precomputed_turn_lane"
    
    error = control_error_from_trajectory_current(traj, 600.0)
    assert error["epsilon_x_mm"] == 0.0
    assert error["epsilon_y_mm"] == 500.0
    assert error["theta_rad"] == -0.2


def test_phase2_precomputed_turn_lane_prioritizes_longitudinal_offset():
    legacy_turn = {
        "id": "legacy-turn",
        "label": LABEL_TURN_LANE,
        "class_name": "turn-lane",
        "lookahead_x_mm": 100.0,
        "lookahead_d_mm": 600.0,
        "longitudinal_offset_mm": 350.0,
    }
    traj = build_trajectory_from_candidate_current(
        extract_lane_candidates({"objects": [legacy_turn]})[0]
    )
    assert traj.valid
    assert traj.has_precomputed_control
    assert traj.precomputed_epsilon_y_mm == 350.0  # Must prioritize longitudinal_offset_mm over lookahead_d_mm


def test_phase2_lookahead_does_not_interpolate_phantom_segment():
    offset_lane = lane(
        "offset-main",
        LABEL_MAIN_LANE,
        [(200.0, 300.0), (200.0, 900.0)],
    )
    traj = build_trajectory_from_candidate_current(
        extract_lane_candidates({"objects": [offset_lane]})[0]
    )
    # Distance to first point (200, 300) is sqrt(200^2 + 300^2) = 360.55 mm
    # If we request a lookahead of 150 mm, it falls in the gap.
    # It must return the first waypoint (200, 300) directly, NOT an interpolated point like (83.2, 124.8)
    error = control_error_from_trajectory_current(traj, 150.0)
    assert error["epsilon_x_mm"] == 200.0
    assert error["epsilon_y_mm"] == 300.0


def test_phase3_split_main_lanes_excludes_overlapping_fragments():
    source = read_control_node()
    assert "closest_max_y" in source
    assert "local_min_y >= min_ahead_start_y - 10.0" in source


def test_phase3_standalone_turn_preserves_target_preview_distance():
    source = read_control_node()
    assert "active_target_lane" in source
    assert 'active_target_lane->raw_obj.contains("lookahead_d_mm")' in source


def test_phase3_precomputed_turn_lanes_included_in_t_junction_detection():
    source = read_control_node()
    detect_t_body = source.split("bool detect_t_junction", 1)[1].split(
        "void telemetry_callback", 1
    )[0]
    assert "has_precomputed_control" in detect_t_body
    assert "precomputed_epsilon_x_mm" in detect_t_body
    assert "precomputed_epsilon_y_mm" in detect_t_body
