import os

import numpy as np

from tools.local_post_inference_simulator.backend.ipm_adapter import (
    AXIS_DESCRIPTION,
    TRAJECTORY_STAGE_STYLE,
    discard_stale_telemetry_realworld,
    load_calibration_status,
    matching_telemetry_realworld,
    render_bev_image,
    summarize_ipm_telemetry,
)


def sample_telemetry_realworld():
    return {
        "timestamp_ms": 42,
        "objects": [
            {
                "id": "main_lane_1",
                "track_id": "main_lane_1",
                "label": 6,
                "class_name": "main-lane",
                "prob": 1.0,
                "polygons": [[[250, 470], [390, 470], [360, 120], [280, 120]]],
                "polygons_real_world": [[[-120.0, 100.0], [120.0, 100.0], [90.0, 900.0], [-90.0, 900.0]]],
                "waypoints": [[0.0, 100.0], [0.0, 500.0], [0.0, 900.0]],
                "lookahead_d_mm": 120.0,
                "lookahead_x_mm": 0.0,
                "heading_angle_rad": 0.0,
                "polynomial": {"a3": 0.0, "a2": 0.0, "a1": 0.0, "a0": 0.0},
            },
            {
                "id": "solid_white_1",
                "track_id": "solid_white_1",
                "label": 16,
                "class_name": "solid-white",
                "prob": 0.95,
                "polygons_real_world": [[[120.0, 100.0], [90.0, 900.0]]],
            },
        ],
    }


def test_summarize_ipm_telemetry_extracts_world_geometry_and_axis_contract():
    summary = summarize_ipm_telemetry(sample_telemetry_realworld())

    assert summary["source_topic"] == "/avs/telemetry_realworld"
    assert summary["timestamp_ms"] == 42
    assert summary["axis"] == AXIS_DESCRIPTION
    assert summary["object_count"] == 2
    assert summary["lane_count"] == 1
    assert summary["waypoint_object_count"] == 1
    assert summary["world_bounds"] == {
        "min_x": -120.0,
        "max_x": 120.0,
        "min_y": 100.0,
        "max_y": 900.0,
    }

    main_lane = summary["objects"][0]
    assert main_lane["is_lane"] is True
    assert main_lane["polygon_count"] == 1
    assert main_lane["world_point_count"] == 4
    assert main_lane["waypoint_count"] == 3
    assert main_lane["control_fields"]["lookahead_d_mm"] == 120.0
    assert main_lane["control_fields"]["polynomial"]["a0"] == 0.0


def test_render_bev_image_returns_nonblank_debug_canvas():
    img = render_bev_image(sample_telemetry_realworld(), width=360, height=280)

    assert isinstance(img, np.ndarray)
    assert img.shape == (280, 360, 3)
    assert int(img.sum()) > 0


def sample_lane_state_with_debug_trajectories():
    return {
        "control_source": "trajectory_manager",
        "debug_trajectories": [
            {
                "stage": "candidate",
                "valid": True,
                "trajectory_kind": "follow_main",
                "confidence": 0.8,
                "normalization_mode": "no_previous_passthrough",
                "points": [[-40.0, 200.0], [-30.0, 500.0], [-20.0, 800.0]],
                "has_precomputed_control": False,
            },
            {
                "stage": "committed",
                "valid": True,
                "trajectory_kind": "follow_main",
                "confidence": 1.0,
                "normalization_mode": "no_previous_passthrough",
                "points": [[0.0, 200.0], [0.0, 500.0], [0.0, 800.0]],
                "has_precomputed_control": True,
            },
        ],
    }


def _count_pixels_near_color(img, bgr, tolerance=20):
    diff = np.abs(img.astype(int) - np.array(bgr))
    return int(np.all(diff <= tolerance, axis=-1).sum())


def test_render_bev_image_trajectory_overlay_only_draws_requested_stage():
    telemetry = sample_telemetry_realworld()
    lane_state = sample_lane_state_with_debug_trajectories()

    baseline = render_bev_image(telemetry, width=360, height=280)
    committed_only = render_bev_image(
        telemetry, width=360, height=280,
        lane_state=lane_state, show_committed_trajectory=True,
    )
    candidate_only = render_bev_image(
        telemetry, width=360, height=280,
        lane_state=lane_state, show_candidate_trajectory=True,
    )

    committed_color = TRAJECTORY_STAGE_STYLE["committed"]["color"]
    candidate_color = TRAJECTORY_STAGE_STYLE["candidate"]["color"]

    # Requested stage's color shows up that wasn't in the undecorated baseline.
    assert _count_pixels_near_color(committed_only, committed_color) > _count_pixels_near_color(baseline, committed_color)
    assert _count_pixels_near_color(candidate_only, candidate_color) > _count_pixels_near_color(baseline, candidate_color)

    # Toggling only "committed" must not also draw "candidate", and vice versa.
    assert _count_pixels_near_color(committed_only, candidate_color) == _count_pixels_near_color(baseline, candidate_color)
    assert _count_pixels_near_color(candidate_only, committed_color) == _count_pixels_near_color(baseline, committed_color)


def test_render_bev_image_trajectory_overlay_no_op_when_all_flags_off():
    telemetry = sample_telemetry_realworld()
    lane_state = sample_lane_state_with_debug_trajectories()

    baseline = render_bev_image(telemetry, width=360, height=280)
    with_unused_lane_state = render_bev_image(telemetry, width=360, height=280, lane_state=lane_state)

    assert np.array_equal(baseline, with_unused_lane_state)


def test_render_bev_image_trajectory_overlay_hints_when_no_debug_data():
    """
    Guards the Step-IPM-only workflow (Phase 3): control_node never ran, so
    lane_state is None. A checkbox being on with nothing to draw must not look
    identical to "trajectory overlay is broken" -- see review. The image should
    visibly differ (a hint is drawn) rather than silently no-op.
    """
    telemetry = sample_telemetry_realworld()

    baseline = render_bev_image(telemetry, width=360, height=280)
    checkbox_on_but_no_lane_state = render_bev_image(
        telemetry, width=360, height=280,
        lane_state=None, show_committed_trajectory=True,
    )

    assert not np.array_equal(baseline, checkbox_on_but_no_lane_state)


def test_render_bev_image_trajectory_overlay_hint_distinguishes_cause():
    """
    The "nothing was drawn" hint must not blanket-blame Step IPM for every
    no-data case -- see review. lane_state is None (Step IPM never ran
    control_node), lane_state present but missing the debug_trajectories field
    entirely, and debug_trajectories present but the requested stage has no
    valid points are three different root causes and must render three
    different hints (proxied here by distinct pixel output, since OCR-ing the
    hint text isn't practical in a unit test).

    Note: control_source == "direct_ipm" alone does NOT predict a missing
    debug_trajectories field -- confirmed live that control_node.cpp still
    runs the trajectory manager and populates it on effectively every decision
    cycle regardless of that bypass. The lane_state below is just a malformed
    payload missing the key, not a claim about when direct_ipm causes this.
    """
    telemetry = sample_telemetry_realworld()
    width, height = 360, 280

    no_lane_state = render_bev_image(
        telemetry, width=width, height=height,
        lane_state=None, show_committed_trajectory=True,
    )
    lane_state_without_debug_field = render_bev_image(
        telemetry, width=width, height=height,
        lane_state={"control_source": "trajectory_manager"}, show_committed_trajectory=True,
    )
    # debug_trajectories has "candidate"/"committed" but not "normalized".
    stage_not_present = render_bev_image(
        telemetry, width=width, height=height,
        lane_state=sample_lane_state_with_debug_trajectories(), show_normalized_trajectory=True,
    )

    assert not np.array_equal(no_lane_state, lane_state_without_debug_field)
    assert not np.array_equal(no_lane_state, stage_not_present)
    assert not np.array_equal(lane_state_without_debug_field, stage_not_present)


def test_load_calibration_status_validates_production_calibration_file():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    status = load_calibration_status("config/calibration.json", repo_root)

    assert status["exists"] is True
    assert status["valid"] is True
    assert len(status["homography_matrix"]) == 3
    assert status["axis"] == AXIS_DESCRIPTION


def test_matching_telemetry_realworld_rejects_stale_timestamp():
    outputs = {
        "telemetry_realworld": sample_telemetry_realworld(),
        "telemetry_realworld_time": 123.0,
    }

    assert matching_telemetry_realworld(outputs, timestamp_ms=42) == sample_telemetry_realworld()
    assert matching_telemetry_realworld(outputs, timestamp_ms=43) is None


def test_discard_stale_telemetry_realworld_removes_geometry_payload():
    outputs = {
        "telemetry_realworld": sample_telemetry_realworld(),
        "telemetry_realworld_time": 123.0,
        "lane_state": {"timestamp_ms": 42},
    }

    sanitized = discard_stale_telemetry_realworld(outputs)

    assert sanitized["telemetry_realworld"] is None
    assert sanitized["telemetry_realworld_time"] == 0.0
    assert sanitized["lane_state"] == {"timestamp_ms": 42}
