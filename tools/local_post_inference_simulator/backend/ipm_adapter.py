import json
import math
import os
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .mask_to_objects import CLASS_MAPPING

# Label ids come from config/label_mapping.json (via CLASS_MAPPING) so a class
# insertion in the model does not silently shift what counts as a lane here.
_LABEL_BY_NAME = {name: label for label, name in CLASS_MAPPING.items()}
TURN_LANE_LABEL = _LABEL_BY_NAME["turn-lane"]
LANE_LABELS = {_LABEL_BY_NAME[name] for name in ("main-lane", "other-lane", "turn-lane")}
DEFAULT_WORLD_BOUNDS = {"min_x": -500.0, "max_x": 500.0, "min_y": 0.0, "max_y": 1500.0}

AXIS_DESCRIPTION = {
    "units": "mm",
    "x_positive": "right_of_vehicle",
    "y_positive": "forward_from_vehicle",
    "origin": "vehicle_centerline_at_rear_axle",
}

# BGR colors, kept aligned with the inference/debug palette.
CLASS_COLORS = [
    (255, 0, 0),
    (0, 165, 255),
    (255, 127, 0),
    (120, 200, 0),
    (80, 80, 255),
    (128, 255, 255),
    (0, 255, 0),
    (0, 0, 255),
    (128, 128, 128),
    (60, 20, 220),
    (0, 0, 180),
    (50, 50, 150),
    (230, 100, 50),
    (0, 0, 255),
    (235, 206, 135),
    (180, 130, 70),
    (255, 255, 0),
    (0, 255, 255),
    (0, 255, 127),
    (0, 0, 128),
    (127, 0, 255),
    (255, 0, 255),
]


Point = Tuple[float, float]
Bounds = Dict[str, float]


def _finite_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _point_from_json(value: Any) -> Optional[Point]:
    if not isinstance(value, Sequence) or len(value) < 2:
        return None
    x = _finite_float(value[0])
    y = _finite_float(value[1])
    if x is None or y is None:
        return None
    return (x, y)


def _normalize_points(points: Any) -> List[List[float]]:
    if not isinstance(points, list):
        return []
    normalized: List[List[float]] = []
    for pt in points:
        pair = _point_from_json(pt)
        if pair is not None:
            normalized.append([pair[0], pair[1]])
    return normalized


def _normalize_polygons(polygons: Any) -> List[List[List[float]]]:
    if not isinstance(polygons, list):
        return []
    normalized: List[List[List[float]]] = []
    for poly in polygons:
        points = _normalize_points(poly)
        if points:
            normalized.append(points)
    return normalized


def _iter_polygon_points(polygons: Iterable[Iterable[Sequence[float]]]) -> Iterable[Point]:
    for poly in polygons:
        for pt in poly:
            pair = _point_from_json(pt)
            if pair is not None:
                yield pair


def _bounds_for_points(points: Iterable[Point]) -> Optional[Bounds]:
    pts = list(points)
    if not pts:
        return None
    xs = [pt[0] for pt in pts]
    ys = [pt[1] for pt in pts]
    return {
        "min_x": round(min(xs), 3),
        "max_x": round(max(xs), 3),
        "min_y": round(min(ys), 3),
        "max_y": round(max(ys), 3),
    }


def _merge_bounds(bounds_list: Iterable[Optional[Bounds]]) -> Optional[Bounds]:
    valid = [bounds for bounds in bounds_list if bounds is not None]
    if not valid:
        return None
    return {
        "min_x": round(min(bounds["min_x"] for bounds in valid), 3),
        "max_x": round(max(bounds["max_x"] for bounds in valid), 3),
        "min_y": round(min(bounds["min_y"] for bounds in valid), 3),
        "max_y": round(max(bounds["max_y"] for bounds in valid), 3),
    }


def summarize_ipm_telemetry(telemetry_realworld: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    telemetry = telemetry_realworld if isinstance(telemetry_realworld, dict) else {}
    objects = telemetry.get("objects", [])
    if not isinstance(objects, list):
        objects = []

    object_summaries: List[Dict[str, Any]] = []
    object_bounds: List[Optional[Bounds]] = []
    near_field_bounds: List[Optional[Bounds]] = []
    lane_count = 0
    waypoint_object_count = 0

    for obj in objects:
        if not isinstance(obj, dict):
            continue

        label = obj.get("label")
        try:
            label_int = int(label)
        except (TypeError, ValueError):
            label_int = -1

        world_polygons = _normalize_polygons(obj.get("polygons_real_world"))
        pixel_polygons = _normalize_polygons(obj.get("polygons"))
        waypoints = _normalize_points(obj.get("waypoints"))

        world_points = list(_iter_polygon_points(world_polygons))
        waypoint_points = [tuple(pt) for pt in waypoints]
        combined_world_points = world_points + waypoint_points
        world_bounds = _bounds_for_points(combined_world_points)
        if label_int != TURN_LANE_LABEL:
            near_field_bounds.append(world_bounds)
        pixel_bounds = _bounds_for_points(_iter_polygon_points(pixel_polygons))

        object_bounds.append(world_bounds)
        is_lane = label_int in LANE_LABELS
        if is_lane:
            lane_count += 1
        if waypoints:
            waypoint_object_count += 1

        control_fields: Dict[str, Any] = {}
        for key in (
            "lateral_offset_mm",
            "longitudinal_offset_mm",
            "heading_angle_rad",
            "curvature_inv_mm",
            "lookahead_d_mm",
            "lookahead_x_mm",
            "lookahead_theta_rad",
            "polynomial",
            "debug_centerline",
        ):
            if key in obj:
                control_fields[key] = obj[key]

        object_summaries.append(
            {
                "id": obj.get("id"),
                "track_id": obj.get("track_id"),
                "label": label_int,
                "class_name": obj.get("class_name"),
                "confidence": obj.get("prob"),
                "is_lane": is_lane,
                "polygon_count": len(world_polygons),
                "world_point_count": len(world_points),
                "waypoint_count": len(waypoints),
                "world_bounds": world_bounds,
                "pixel_bounds": pixel_bounds,
                "polygons_real_world": world_polygons,
                "waypoints": waypoints,
                "control_fields": control_fields,
            }
        )

    return {
        "source_topic": "/avs/telemetry_realworld",
        "timestamp_ms": telemetry.get("timestamp_ms"),
        "axis": AXIS_DESCRIPTION,
        "object_count": len(object_summaries),
        "lane_count": lane_count,
        "waypoint_object_count": waypoint_object_count,
        "world_bounds": _merge_bounds(object_bounds),
        "near_field_bounds": _merge_bounds(near_field_bounds),
        "objects": object_summaries,
    }


def matching_telemetry_realworld(outputs: Dict[str, Any], timestamp_ms: int) -> Optional[Dict[str, Any]]:
    trw = outputs.get("telemetry_realworld")
    if trw is not None and trw.get("timestamp_ms", -1) == timestamp_ms:
        return trw
    return None


def discard_stale_telemetry_realworld(outputs: Dict[str, Any]) -> Dict[str, Any]:
    sanitized = dict(outputs)
    sanitized["telemetry_realworld"] = None
    sanitized["telemetry_realworld_time"] = 0.0
    return sanitized


def resolve_calibration_path(source: str, repo_root: str) -> str:
    if os.path.isabs(source):
        if source == "/workspace":
            return repo_root
        workspace_prefix = "/workspace/"
        if source.startswith(workspace_prefix) and not os.path.exists(source):
            return os.path.abspath(os.path.join(repo_root, source[len(workspace_prefix):]))
        return source
    return os.path.abspath(os.path.join(repo_root, source))


def load_calibration_status(source: str, repo_root: str) -> Dict[str, Any]:
    resolved_path = resolve_calibration_path(source, repo_root)
    status: Dict[str, Any] = {
        "source": source,
        "resolved_path": resolved_path,
        "exists": os.path.exists(resolved_path),
        "valid": False,
        "axis": AXIS_DESCRIPTION,
    }

    if not status["exists"]:
        status["error"] = "calibration_file_not_found"
        return status

    try:
        with open(resolved_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:
        status["error"] = f"calibration_parse_error: {exc}"
        return status

    matrix = data.get("homography_matrix")
    matrix_valid = (
        isinstance(matrix, list)
        and len(matrix) == 3
        and all(isinstance(row, list) and len(row) == 3 for row in matrix)
    )
    if not matrix_valid:
        status["error"] = "missing_or_invalid_homography_matrix"
        return status

    try:
        status["homography_matrix"] = [[float(value) for value in row] for row in matrix]
    except (TypeError, ValueError):
        status["error"] = "non_numeric_homography_matrix"
        return status

    status["valid"] = True
    for key in ("image_size", "pixel_points", "world_points", "calibrated_at"):
        if key in data:
            status[key] = data[key]
    return status


def _expanded_bounds(bounds: Optional[Bounds]) -> Bounds:
    expanded = dict(bounds or DEFAULT_WORLD_BOUNDS)
    expanded["min_x"] = min(expanded["min_x"], 0.0)
    expanded["max_x"] = max(expanded["max_x"], 0.0)
    expanded["min_y"] = min(expanded["min_y"], 0.0)
    expanded["max_y"] = max(expanded["max_y"], 0.0)

    pad_x = max(100.0, (expanded["max_x"] - expanded["min_x"]) * 0.08)
    pad_y = max(100.0, (expanded["max_y"] - expanded["min_y"]) * 0.08)
    expanded["min_x"] -= pad_x
    expanded["max_x"] += pad_x
    expanded["min_y"] -= pad_y
    expanded["max_y"] += pad_y

    if abs(expanded["max_x"] - expanded["min_x"]) < 1e-6:
        expanded["min_x"] -= 500.0
        expanded["max_x"] += 500.0
    if abs(expanded["max_y"] - expanded["min_y"]) < 1e-6:
        expanded["min_y"] -= 500.0
        expanded["max_y"] += 500.0

    return expanded


# Debug trajectory stages published by control_node in /avs/lane_state
# (see json_from_planned_trajectory in control_node.cpp). Colors chosen to be
# visually distinct from CLASS_COLORS and from each other (BGR).
TRAJECTORY_STAGE_STYLE = {
    "candidate": {"color": (0, 165, 255), "thickness": 2, "dash": True},     # orange, dashed
    "normalized": {"color": (255, 0, 220), "thickness": 2, "dash": True},    # magenta, dashed
    "committed": {"color": (0, 255, 0), "thickness": 3, "dash": False},      # green, solid
}


def _draw_dashed_polyline(img: np.ndarray, pts_px: List[Tuple[int, int]], color, thickness: int, dash_len: int = 10, gap_len: int = 6) -> None:
    for i in range(len(pts_px) - 1):
        x1, y1 = pts_px[i]
        x2, y2 = pts_px[i + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len < 1e-6:
            continue
        n_steps = max(1, int(seg_len // (dash_len + gap_len)) + 1)
        for s in range(n_steps):
            t0 = min(1.0, (s * (dash_len + gap_len)) / seg_len)
            t1 = min(1.0, (s * (dash_len + gap_len) + dash_len) / seg_len)
            if t0 >= 1.0:
                break
            p0 = (int(round(x1 + (x2 - x1) * t0)), int(round(y1 + (y2 - y1) * t0)))
            p1 = (int(round(x1 + (x2 - x1) * t1)), int(round(y1 + (y2 - y1) * t1)))
            cv2.line(img, p0, p1, color, thickness, cv2.LINE_AA)


def draw_debug_trajectories(
    img: np.ndarray,
    world_to_px,
    lane_state: Optional[Dict[str, Any]],
    show_candidate: bool = False,
    show_normalized: bool = False,
    show_committed: bool = False,
) -> None:
    """Overlay control_node's candidate/normalized/committed debug trajectories
    (from /avs/lane_state -> debug_trajectories) onto a BEV image already
    rendered in world coordinates. Points are consumed as-is (world mm),
    matching the axis convention used elsewhere in this module. This never
    recomputes planning; it only visualizes what control_node already
    published, per docs/local_post_inference_simulator/plan.md ("không copy
    logic planner sang simulator")."""
    stages_wanted = {
        "candidate": show_candidate,
        "normalized": show_normalized,
        "committed": show_committed,
    }
    if not any(stages_wanted.values()):
        return

    debug_trajectories = lane_state.get("debug_trajectories") if isinstance(lane_state, dict) else None
    drawn_any = False

    if isinstance(debug_trajectories, list):
        for traj in debug_trajectories:
            if not isinstance(traj, dict):
                continue
            stage = traj.get("stage")
            if stage not in stages_wanted or not stages_wanted[stage]:
                continue
            points = _normalize_points(traj.get("points"))
            if len(points) < 2:
                continue
            style = TRAJECTORY_STAGE_STYLE[stage]
            pts_px = [world_to_px(pt[0], pt[1]) for pt in points]
            if style["dash"]:
                _draw_dashed_polyline(img, pts_px, style["color"], style["thickness"])
            else:
                cv2.polylines(img, [np.array(pts_px, dtype=np.int32)], isClosed=False, color=style["color"], thickness=style["thickness"], lineType=cv2.LINE_AA)
            for px, py in pts_px:
                cv2.circle(img, (px, py), 2, style["color"], -1, cv2.LINE_AA)

            label_bits = [stage, traj.get("trajectory_kind", "")]
            if not traj.get("valid", True):
                label_bits.append("INVALID")
            label_text = " ".join(b for b in label_bits if b)
            anchor = pts_px[-1]
            cv2.putText(img, label_text, (anchor[0] + 6, anchor[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, style["color"], 1, cv2.LINE_AA)
            drawn_any = True

    if drawn_any:
        return

    # A checkbox is on but nothing was plotted. Distinguish *why*, so the hint
    # doesn't blame "you're using Step IPM" when the real cause is e.g. the
    # selected stage/points being genuinely absent for this frame -- see review.
    if lane_state is None:
        # /api/scenarios/step_ipm never runs control_node, so there is no
        # /avs/lane_state at all for this frame.
        hint = "debug_trajectories: khong co (dang dung Step IPM, control_node chua chay - hay dung Step/Play)"
    elif not isinstance(debug_trajectories, list):
        # lane_state exists but the field itself is missing/malformed. Note:
        # control_source == "direct_ipm" (final control bypasses the trajectory
        # manager for that frame's steering) is NOT a reliable predictor of this
        # -- control_node.cpp still runs TrajectoryPlanner/Normalizer/Manager and
        # populates debug_trajectories on effectively every decision cycle
        # regardless of that bypass; confirmed live against turn_right_two_lanes
        # (control_source: direct_ipm, debug_trajectories: 3 stages present).
        hint = "debug_trajectories: khong co trong lane_state nay (kiem tra log control_node)"
    else:
        # debug_trajectories is present, but none of the enabled stage(s) had
        # >=2 valid points in this frame -- not a missing-data-source problem.
        hint = "debug_trajectories: khong co diem hop le cho stage da chon trong frame nay"

    cv2.putText(img, hint, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)


def render_bev_image(
    telemetry_realworld: Optional[Dict[str, Any]],
    width: int = 720,
    height: int = 520,
    show_slices: bool = False,
    show_raw_midpoints: bool = False,
    show_filtered_midpoints: bool = False,
    lane_state: Optional[Dict[str, Any]] = None,
    show_candidate_trajectory: bool = False,
    show_normalized_trajectory: bool = False,
    show_committed_trajectory: bool = False,
) -> np.ndarray:
    width = max(240, min(int(width), 2000))
    height = max(240, min(int(height), 2000))

    summary = summarize_ipm_telemetry(telemetry_realworld)
    # Scale to near-field lanes/markings so turn-lane geometry (which can span
    # several meters) doesn't crush close-range lanes down to a few pixels.
    # Turn-lane objects still draw at their true world coordinates and are
    # simply clipped at the canvas edge if they extend past this view.
    bounds = _expanded_bounds(summary.get("near_field_bounds") or summary["world_bounds"])
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (18, 20, 22)

    margin = 46
    usable_w = max(1, width - margin * 2)
    usable_h = max(1, height - margin * 2)
    span_x = bounds["max_x"] - bounds["min_x"]
    span_y = bounds["max_y"] - bounds["min_y"]
    scale = min(usable_w / span_x, usable_h / span_y)

    def world_to_px(x: float, y: float) -> Tuple[int, int]:
        px = margin + int(round((x - bounds["min_x"]) * scale))
        py = height - margin - int(round((y - bounds["min_y"]) * scale))
        return px, py

    def draw_line_world(p0: Point, p1: Point, color: Tuple[int, int, int], thickness: int = 1) -> None:
        cv2.line(img, world_to_px(p0[0], p0[1]), world_to_px(p1[0], p1[1]), color, thickness, cv2.LINE_AA)

    grid_step = 250.0
    grid_color = (42, 47, 52)
    x_start = math.floor(bounds["min_x"] / grid_step) * grid_step
    x_end = math.ceil(bounds["max_x"] / grid_step) * grid_step
    y_start = math.floor(bounds["min_y"] / grid_step) * grid_step
    y_end = math.ceil(bounds["max_y"] / grid_step) * grid_step

    x = x_start
    while x <= x_end:
        draw_line_world((x, bounds["min_y"]), (x, bounds["max_y"]), grid_color)
        x += grid_step

    y = y_start
    while y <= y_end:
        draw_line_world((bounds["min_x"], y), (bounds["max_x"], y), grid_color)
        y += grid_step

    if bounds["min_y"] <= 0.0 <= bounds["max_y"]:
        draw_line_world((bounds["min_x"], 0.0), (bounds["max_x"], 0.0), (80, 80, 220), 2)
    if bounds["min_x"] <= 0.0 <= bounds["max_x"]:
        draw_line_world((0.0, bounds["min_y"]), (0.0, bounds["max_y"]), (80, 180, 80), 2)

    origin = world_to_px(0.0, 0.0)
    cv2.circle(img, origin, 5, (240, 240, 240), -1, cv2.LINE_AA)
    cv2.putText(img, "X+ right", (width - 128, height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 230), 1, cv2.LINE_AA)
    cv2.putText(img, "Y+ forward", (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 230, 190), 1, cv2.LINE_AA)

    overlay = img.copy()
    has_world_points = summary["world_bounds"] is not None
    for obj in summary["objects"]:
        label = int(obj.get("label", -1))
        color = CLASS_COLORS[label % len(CLASS_COLORS)] if label >= 0 else (200, 200, 200)

        for poly in obj["polygons_real_world"]:
            pts = np.array([world_to_px(pt[0], pt[1]) for pt in poly], dtype=np.int32)
            if len(pts) >= 3:
                cv2.fillPoly(overlay, [pts], color)
                cv2.polylines(img, [pts], isClosed=True, color=color, thickness=2, lineType=cv2.LINE_AA)
            elif len(pts) >= 2:
                cv2.polylines(img, [pts], isClosed=False, color=color, thickness=2, lineType=cv2.LINE_AA)

        waypoints = obj["waypoints"]
        if waypoints:
            wp_pts = np.array([world_to_px(pt[0], pt[1]) for pt in waypoints], dtype=np.int32)
            if len(wp_pts) >= 2:
                cv2.polylines(img, [wp_pts], isClosed=False, color=(245, 245, 245), thickness=2, lineType=cv2.LINE_AA)
            for px, py in wp_pts:
                cv2.circle(img, (int(px), int(py)), 3, (255, 255, 255), -1, cv2.LINE_AA)

        debug_centerline = obj.get("control_fields", {}).get("debug_centerline")
        if debug_centerline:
            if show_slices and "slices" in debug_centerline:
                for slice_item in debug_centerline["slices"]:
                    if "y" in slice_item:
                        px1, py1 = world_to_px(slice_item["x_left"], slice_item["y"])
                        px2, py2 = world_to_px(slice_item["x_right"], slice_item["y"])
                        cv2.line(img, (int(px1), int(py1)), (int(px2), int(py2)), (100, 100, 100), 1, cv2.LINE_AA)
                    elif "x" in slice_item:
                        px1, py1 = world_to_px(slice_item["x"], slice_item["y_bottom"])
                        px2, py2 = world_to_px(slice_item["x"], slice_item["y_top"])
                        cv2.line(img, (int(px1), int(py1)), (int(px2), int(py2)), (100, 100, 100), 1, cv2.LINE_AA)
            if show_raw_midpoints and "raw_midpoints" in debug_centerline:
                for pt in debug_centerline["raw_midpoints"]:
                    px, py = world_to_px(pt[0], pt[1])
                    cv2.circle(img, (int(px), int(py)), 3, (0, 0, 255), -1, cv2.LINE_AA)
            if show_filtered_midpoints and "filtered_midpoints" in debug_centerline:
                for pt in debug_centerline["filtered_midpoints"]:
                    px, py = world_to_px(pt[0], pt[1])
                    cv2.circle(img, (int(px), int(py)), 4, (0, 200, 255), -1, cv2.LINE_AA)

        class_name = obj.get("class_name")
        label = obj.get("label")
        label_text = str(class_name or (str(label) if label is not None else ""))
        anchor = None
        if waypoints:
            anchor = world_to_px(waypoints[0][0], waypoints[0][1])
        elif obj["polygons_real_world"] and obj["polygons_real_world"][0]:
            pt = obj["polygons_real_world"][0][0]
            anchor = world_to_px(pt[0], pt[1])
        if anchor and label_text:
            cv2.putText(img, label_text, (anchor[0] + 6, anchor[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (235, 235, 235), 1, cv2.LINE_AA)

    cv2.addWeighted(overlay, 0.22, img, 0.78, 0, img)

    draw_debug_trajectories(
        img,
        world_to_px,
        lane_state,
        show_candidate=show_candidate_trajectory,
        show_normalized=show_normalized_trajectory,
        show_committed=show_committed_trajectory,
    )

    if not has_world_points:
        cv2.putText(img, "No /avs/telemetry_realworld world points", (28, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 210, 210), 1, cv2.LINE_AA)

    return img
