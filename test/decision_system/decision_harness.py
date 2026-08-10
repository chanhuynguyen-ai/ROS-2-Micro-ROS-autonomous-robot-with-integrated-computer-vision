from __future__ import annotations

from dataclasses import dataclass
from math import atan2, sqrt
from typing import Any


LABEL_DASHED_WHITE = 0
LABEL_DASHED_YELLOW = 1
LABEL_DOUBLE_SOLID_WHITE = 2
LABEL_MAIN_LANE = 6
LABEL_OTHER_LANE = 7
LABEL_SOLID_WHITE = 16
LABEL_SOLID_YELLOW = 17
LABEL_STOP_LINE = 19
LABEL_TURN_LANE = 20

SOLID_LABELS = {
    LABEL_DOUBLE_SOLID_WHITE,
    LABEL_SOLID_WHITE,
    LABEL_SOLID_YELLOW,
}


@dataclass
class LaneCandidate:
    label: int
    raw_obj: dict[str, Any]


@dataclass
class MarkingCandidate:
    label: int
    raw_obj: dict[str, Any]


@dataclass
class ActiveTrajectory:
    points: list[tuple[float, float]]
    source_labels: list[int]
    trajectory_kind: str
    valid: bool
    has_precomputed_control: bool = False
    precomputed_epsilon_x_mm: float = 0.0
    precomputed_epsilon_y_mm: float = 0.0
    precomputed_theta_rad: float = 0.0
    precomputed_curvature_inv_mm: float = 0.0
    precomputed_lookahead_d_mm: float = 0.0


def lane(
    object_id: str,
    label: int,
    points: list[tuple[float, float]],
    *,
    lookahead_x_mm: float | None = None,
    lookahead_d_mm: float = 600.0,
) -> dict[str, Any]:
    obj: dict[str, Any] = {
        "id": object_id,
        "label": label,
        "class_name": {
            LABEL_MAIN_LANE: "main-lane",
            LABEL_OTHER_LANE: "other-lane",
            LABEL_TURN_LANE: "turn-lane",
        }.get(label, "unknown"),
        "waypoints": [[x, y] for x, y in points],
        "lookahead_d_mm": lookahead_d_mm,
    }
    if lookahead_x_mm is not None:
        obj["lookahead_x_mm"] = lookahead_x_mm
    return obj


def marking(
    object_id: str,
    label: int,
    *,
    waypoints: list[tuple[float, float]] | None = None,
    polygons_real_world: list[list[tuple[float, float]]] | None = None,
    lookahead_x_mm: float | None = None,
) -> dict[str, Any]:
    obj: dict[str, Any] = {
        "id": object_id,
        "label": label,
        "class_name": {
            LABEL_DASHED_WHITE: "dashed-white",
            LABEL_DASHED_YELLOW: "dashed-yellow",
            LABEL_DOUBLE_SOLID_WHITE: "double-solid-white",
            LABEL_SOLID_WHITE: "solid-white",
            LABEL_SOLID_YELLOW: "solid-yellow",
            LABEL_STOP_LINE: "stop-line",
        }.get(label, "unknown"),
    }
    if waypoints is not None:
        obj["waypoints"] = [[x, y] for x, y in waypoints]
    if polygons_real_world is not None:
        obj["polygons_real_world"] = [
            [[x, y] for x, y in polygon] for polygon in polygons_real_world
        ]
    if lookahead_x_mm is not None:
        obj["lookahead_x_mm"] = lookahead_x_mm
    return obj


def extract_lane_candidates(telemetry: dict[str, Any]) -> list[LaneCandidate]:
    candidates: list[LaneCandidate] = []
    for obj in telemetry.get("objects", []):
        label = obj.get("label", -1)
        if label in {LABEL_MAIN_LANE, LABEL_OTHER_LANE, LABEL_TURN_LANE}:
            candidates.append(LaneCandidate(label=label, raw_obj=obj))
    return candidates


def extract_marking_candidates(telemetry: dict[str, Any]) -> list[MarkingCandidate]:
    candidates: list[MarkingCandidate] = []
    for obj in telemetry.get("objects", []):
        label = obj.get("label", -1)
        if label in {
            LABEL_DASHED_WHITE,
            LABEL_DASHED_YELLOW,
            LABEL_DOUBLE_SOLID_WHITE,
            LABEL_SOLID_WHITE,
            LABEL_SOLID_YELLOW,
            LABEL_STOP_LINE,
        }:
            candidates.append(MarkingCandidate(label=label, raw_obj=obj))
    return candidates


def _representative_x(obj: dict[str, Any]) -> float:
    if "lookahead_x_mm" in obj:
        return float(obj["lookahead_x_mm"])
    if obj.get("waypoints"):
        return float(obj["waypoints"][0][0])
    return 0.0


def select_other_lane_current(
    lanes: list[LaneCandidate],
    main_lane: LaneCandidate | None,
    is_left_change: bool,
) -> LaneCandidate | None:
    other_lanes = [cand for cand in lanes if cand.label == LABEL_OTHER_LANE]
    if not other_lanes:
        return None
        
    if main_lane is not None:
        main_x = _representative_x(main_lane.raw_obj)
    else:
        main_x = 0.0
        
    best: LaneCandidate | None = None
    best_diff = float("inf")
    for cand in other_lanes:
        other_x = _representative_x(cand.raw_obj)
        is_left = other_x < main_x
        if is_left == is_left_change:
            diff = abs(other_x - main_x)
            if diff < best_diff:
                best = cand
                best_diff = diff
    return best


def select_turn_lane_current(
    lanes: list[LaneCandidate],
    *,
    is_turn_right: bool,
    is_t_junction: bool,
) -> LaneCandidate | None:
    turns = [cand for cand in lanes if cand.label == LABEL_TURN_LANE]
    if not turns:
        return None

    # First pass: identify if any candidate is on the strict correct side
    correct_side_exists = False
    for cand in turns:
        avg_x = 0.0
        if cand.raw_obj.get("waypoints"):
            sum_x = 0.0
            count = 0
            for x, _ in cand.raw_obj["waypoints"]:
                sum_x += float(x)
                count += 1
            if count > 0:
                avg_x = sum_x / count
        elif "longitudinal_offset_mm" in cand.raw_obj or "lookahead_d_mm" in cand.raw_obj or "lookahead_theta_rad" in cand.raw_obj or "lookahead_x_mm" in cand.raw_obj:
            if "lookahead_x_mm" in cand.raw_obj:
                avg_x = float(cand.raw_obj["lookahead_x_mm"])
            elif "lookahead_theta_rad" in cand.raw_obj:
                avg_x = float(cand.raw_obj["lookahead_theta_rad"])
            else:
                avg_x = 1.0 if is_turn_right else -1.0
        
        if is_turn_right and avg_x >= 0.0:
            correct_side_exists = True
        if not is_turn_right and avg_x <= 0.0:
            correct_side_exists = True

    scored: list[tuple[float, LaneCandidate]] = []
    for cand in turns:
        min_dist = float("inf")
        sum_x = 0.0
        count = 0
        avg_x_is_rad = False
        if cand.raw_obj.get("waypoints"):
            for x, y in cand.raw_obj["waypoints"]:
                sum_x += float(x)
                count += 1
                min_dist = min(min_dist, sqrt(float(x) * float(x) + float(y) * float(y)))
            if count == 0:
                continue
            avg_x = sum_x / count
        elif "longitudinal_offset_mm" in cand.raw_obj or "lookahead_d_mm" in cand.raw_obj or "lookahead_theta_rad" in cand.raw_obj or "lookahead_x_mm" in cand.raw_obj:
            min_dist = float(cand.raw_obj.get("longitudinal_offset_mm", cand.raw_obj.get("lookahead_d_mm", 1000.0)))
            if "lookahead_x_mm" in cand.raw_obj:
                avg_x = float(cand.raw_obj["lookahead_x_mm"])
            elif "lookahead_theta_rad" in cand.raw_obj:
                avg_x = float(cand.raw_obj["lookahead_theta_rad"])
                avg_x_is_rad = True
            else:
                avg_x = 1.0 if is_turn_right else -1.0
        else:
            continue

        if not is_t_junction:
            if is_turn_right and avg_x < 0:
                continue
            if not is_turn_right and avg_x > 0:
                continue
        else:
            if correct_side_exists:
                if is_turn_right and avg_x < 0.0:
                    continue
                if not is_turn_right and avg_x > 0.0:
                    continue
            else:
                if avg_x_is_rad:
                    if is_turn_right and avg_x < 0.0:
                        continue
                    if not is_turn_right and avg_x > 0.0:
                        continue
                else:
                    if is_turn_right and avg_x < -200.0:
                        continue
                    if not is_turn_right and avg_x > 200.0:
                        continue
        scored.append((min_dist, cand))

    if not scored:
        return None

    scored.sort(key=lambda item: item[0])
    if is_turn_right:
        return scored[0][1]
    else:
        return scored[-1][1]


def is_lane_change_blocked_by_solid_current(
    main_lane: LaneCandidate | None,
    target_lane: LaneCandidate | None,
    markings: list[MarkingCandidate],
) -> bool:
    if main_lane is None or target_lane is None:
        return False

    main_x = _representative_x(main_lane.raw_obj)
    target_x = _representative_x(target_lane.raw_obj)
    min_x, max_x = sorted((main_x, target_x))

    for mark in markings:
        if mark.label not in SOLID_LABELS:
            continue
        if "lookahead_x_mm" in mark.raw_obj:
            mark_x = float(mark.raw_obj["lookahead_x_mm"])
            if min_x < mark_x < max_x:
                return True
        elif mark.raw_obj.get("waypoints"):
            is_between = False
            for wp in mark.raw_obj["waypoints"]:
                mark_x = float(wp[0])
                mark_y = float(wp[1])
                if min_x < mark_x < max_x and -100.0 <= mark_y <= 1500.0:
                    is_between = True
                    break
            if is_between:
                return True
        elif mark.raw_obj.get("polygons_real_world"):
            is_between = False
            for poly in mark.raw_obj["polygons_real_world"]:
                for pt in poly:
                    mark_x = float(pt[0])
                    mark_y = float(pt[1])
                    if min_x < mark_x < max_x and -100.0 <= mark_y <= 1500.0:
                        is_between = True
                        break
                if is_between:
                    break
            if is_between:
                return True
    return False


def build_trajectory_from_candidate_current(cand: LaneCandidate) -> ActiveTrajectory:
    points = [
        (float(x), float(y))
        for x, y in cand.raw_obj.get("waypoints", [])
    ]
    if cand.label != LABEL_TURN_LANE:
        points.sort(key=lambda point: point[1])
    elif points:
        front_dist = points[0][0] * points[0][0] + points[0][1] * points[0][1]
        back_dist = points[-1][0] * points[-1][0] + points[-1][1] * points[-1][1]
        if back_dist < front_dist:
            points.reverse()

    filtered: list[tuple[float, float]] = []
    for point in points:
        if not filtered:
            filtered.append(point)
            continue
        dx = point[0] - filtered[-1][0]
        dy = point[1] - filtered[-1][1]
        if sqrt(dx * dx + dy * dy) > 10.0:
            filtered.append(point)

    traj = ActiveTrajectory(
        points=filtered,
        source_labels=[cand.label],
        trajectory_kind="turn_lane" if cand.label == LABEL_TURN_LANE else "main_lane",
        valid=len(filtered) >= 2,
    )
    if not traj.valid and cand.label == LABEL_TURN_LANE and "longitudinal_offset_mm" in cand.raw_obj:
        traj.has_precomputed_control = True
        traj.precomputed_epsilon_x_mm = 0.0
        traj.precomputed_epsilon_y_mm = float(cand.raw_obj["longitudinal_offset_mm"])
        traj.precomputed_theta_rad = float(cand.raw_obj.get("lookahead_theta_rad", 0.0))
        traj.precomputed_curvature_inv_mm = float(cand.raw_obj.get("curvature_inv_mm", 0.0))
        traj.precomputed_lookahead_d_mm = float(cand.raw_obj.get("lookahead_d_mm", traj.precomputed_epsilon_y_mm))
        traj.trajectory_kind = "precomputed_turn_lane"
        traj.valid = True
    elif not traj.valid and "lookahead_x_mm" in cand.raw_obj and "lookahead_d_mm" in cand.raw_obj:
        traj.has_precomputed_control = True
        traj.precomputed_epsilon_x_mm = float(cand.raw_obj["lookahead_x_mm"])
        traj.precomputed_epsilon_y_mm = float(cand.raw_obj["lookahead_d_mm"])
        traj.precomputed_theta_rad = float(
            cand.raw_obj.get(
                "lookahead_theta_rad",
                atan2(traj.precomputed_epsilon_x_mm, traj.precomputed_epsilon_y_mm),
            )
        )
        traj.precomputed_curvature_inv_mm = float(cand.raw_obj.get("curvature_inv_mm", 0.0))
        traj.precomputed_lookahead_d_mm = float(cand.raw_obj["lookahead_d_mm"])
        traj.trajectory_kind = "precomputed_turn_lane" if cand.label == LABEL_TURN_LANE else "precomputed_main_lane"
        traj.valid = True
    return traj


def _synthesize_precomputed_points(traj: ActiveTrajectory) -> None:
    if traj.valid and traj.has_precomputed_control and not traj.points:
        traj.points = [
            (0.0, 0.0),
            (traj.precomputed_epsilon_x_mm * 0.5, traj.precomputed_epsilon_y_mm * 0.5),
            (traj.precomputed_epsilon_x_mm, traj.precomputed_epsilon_y_mm)
        ]

def connect_two_lanes_smooth_current(
    current_lane: LaneCandidate,
    target_lane: LaneCandidate,
) -> ActiveTrajectory:
    traj = build_trajectory_from_candidate_current(current_lane)
    target = build_trajectory_from_candidate_current(target_lane)
    
    _synthesize_precomputed_points(traj)
    _synthesize_precomputed_points(target)
    
    if len(traj.points) < 2 or len(target.points) < 2:
        if len(traj.points) < 2 and len(target.points) >= 2:
            return target
        if len(traj.points) >= 2 and len(target.points) < 2:
            return traj
        return target if len(target.points) > len(traj.points) else traj

    p0 = traj.points[-1]
    p3 = target.points[0]
    prev = traj.points[-2]
    nxt = target.points[1]

    dx0 = p0[0] - prev[0]
    dy0 = p0[1] - prev[1]
    len0 = sqrt(dx0 * dx0 + dy0 * dy0) or 1.0
    dx0 /= len0
    dy0 /= len0

    dx3 = nxt[0] - p3[0]
    dy3 = nxt[1] - p3[1]
    len3 = sqrt(dx3 * dx3 + dy3 * dy3) or 1.0
    dx3 /= len3
    dy3 /= len3

    dist = sqrt((p3[0] - p0[0]) ** 2 + (p3[1] - p0[1]) ** 2)
    scale = dist / 3.0
    p1 = (p0[0] + dx0 * scale, p0[1] + dy0 * scale)
    p2 = (p3[0] - dx3 * scale, p3[1] - dy3 * scale)

    num_samples = max(10, int(dist / 50.0))
    points = list(traj.points)
    for i in range(1, num_samples):
        t = i / num_samples
        u = 1.0 - t
        w0 = u * u * u
        w1 = 3.0 * u * u * t
        w2 = 3.0 * u * t * t
        w3 = t * t * t
        bx = w0 * p0[0] + w1 * p1[0] + w2 * p2[0] + w3 * p3[0]
        by = w0 * p0[1] + w1 * p1[1] + w2 * p2[1] + w3 * p3[1]
        points.append((bx, by))
    points.extend(target.points)

    return ActiveTrajectory(
        points=points,
        source_labels=[current_lane.label, target_lane.label],
        trajectory_kind="follow_main_connected",
        valid=True,
    )


def control_error_from_trajectory_current(
    traj: ActiveTrajectory,
    lookahead_d_mm: float,
) -> dict[str, float]:
    if not traj.valid:
        return {
            "epsilon_x_mm": 0.0,
            "epsilon_y_mm": 0.0,
            "theta_rad": 0.0,
            "curvature_inv_mm": 0.0,
        }
    if traj.has_precomputed_control:
        return {
            "epsilon_x_mm": traj.precomputed_epsilon_x_mm,
            "epsilon_y_mm": traj.precomputed_epsilon_y_mm,
            "theta_rad": traj.precomputed_theta_rad,
            "curvature_inv_mm": traj.precomputed_curvature_inv_mm,
        }
    if not traj.points:
        return {
            "epsilon_x_mm": 0.0,
            "epsilon_y_mm": 0.0,
            "theta_rad": 0.0,
            "curvature_inv_mm": 0.0,
        }

    first_pt = traj.points[0]
    initial_dist = sqrt(first_pt[0] * first_pt[0] + first_pt[1] * first_pt[1])

    if initial_dist >= lookahead_d_mm:
        target = first_pt
        target_idx = 0
    else:
        target_idx = len(traj.points) - 1
        target = traj.points[target_idx]
        cumulative_dist = initial_dist
        prev = first_pt
        for idx in range(1, len(traj.points)):
            point = traj.points[idx]
            dx = point[0] - prev[0]
            dy = point[1] - prev[1]
            segment_len = sqrt(dx * dx + dy * dy)
            if cumulative_dist + segment_len >= lookahead_d_mm and segment_len > 1e-6:
                ratio = (lookahead_d_mm - cumulative_dist) / segment_len
                ratio = max(0.0, min(1.0, ratio))
                target = (prev[0] + ratio * dx, prev[1] + ratio * dy)
                target_idx = idx
                break
            cumulative_dist += segment_len
            prev = point

    theta = 0.0
    if abs(target[1]) > 1e-3 or abs(target[0]) > 1e-3:
        theta = atan2(target[0], target[1])

    curvature = 0.0
    if len(traj.points) >= 3:
        c_idx = target_idx
        if c_idx == 0:
            c_idx = 1
        if c_idx == len(traj.points) - 1:
            c_idx = len(traj.points) - 2

        p1 = traj.points[c_idx - 1]
        p2 = traj.points[c_idx]
        p3 = traj.points[c_idx + 1]
        a = sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
        b = sqrt((p3[0] - p2[0]) ** 2 + (p3[1] - p2[1]) ** 2)
        c = sqrt((p3[0] - p1[0]) ** 2 + (p3[1] - p1[1]) ** 2)
        if a > 0 and b > 0 and c > 0:
            cross = (p2[0] - p1[0]) * (p3[1] - p2[1]) - (p2[1] - p1[1]) * (p3[0] - p2[0])
            curvature = 2.0 * cross / (a * b * c)

    return {
        "epsilon_x_mm": target[0],
        "epsilon_y_mm": target[1],
        "theta_rad": theta,
        "curvature_inv_mm": curvature,
    }


def polygon_x_range(obj: dict[str, Any]) -> tuple[float, float] | None:
    xs: list[float] = []
    for polygon in obj.get("polygons_real_world", []):
        for x, _ in polygon:
            xs.append(float(x))
    if not xs:
        return None
    return min(xs), max(xs)


# ==============================================================================
# New Class-Based Trajectory Planning & Memory Architecture (Phase 11)
# ==============================================================================
from enum import Enum
import math

class Point2D:
    def __init__(self, x: float, y: float):
        self._x = float(x)
        self._y = float(y)

    @property
    def x(self) -> float:
        return self._x

    @x.setter
    def x(self, val: float):
        self._x = float(val)

    @property
    def y(self) -> float:
        return self._y

    @y.setter
    def y(self, val: float):
        self._y = float(val)

    @property
    def epsilon_x_mm(self) -> float:
        return self._x

    @epsilon_x_mm.setter
    def epsilon_x_mm(self, val: float):
        self._x = float(val)

    @property
    def epsilon_y_mm(self) -> float:
        return self._y

    @epsilon_y_mm.setter
    def epsilon_y_mm(self, val: float):
        self._y = float(val)

    def __getitem__(self, idx: int) -> float:
        if idx == 0:
            return self._x
        elif idx == 1:
            return self._y
        raise IndexError("Point2D index out of range")

    def __len__(self) -> int:
        return 2

    def __repr__(self):
        return f"Point2D(x={self._x}, y={self._y})"

    def __eq__(self, other):
        if not isinstance(other, Point2D):
            return False
        return self._x == other._x and self._y == other._y


class TrajectoryKind(Enum):
    UNKNOWN = 0
    FOLLOW_MAIN = 1
    TURN_RIGHT = 2
    TURN_LEFT = 3
    LANE_CHANGE_LEFT = 4
    LANE_CHANGE_RIGHT = 5


def trajectory_kind_name(kind: TrajectoryKind) -> str:
    return {
        TrajectoryKind.FOLLOW_MAIN: "follow_main",
        TrajectoryKind.TURN_RIGHT: "turn_right",
        TrajectoryKind.TURN_LEFT: "turn_left",
        TrajectoryKind.LANE_CHANGE_LEFT: "lane_change_left",
        TrajectoryKind.LANE_CHANGE_RIGHT: "lane_change_right",
    }.get(kind, "unknown")


def route_intent_name(intent: "RouteIntent") -> str:
    return {
        RouteIntent.FOLLOW_MAIN: "follow_main",
        RouteIntent.TURN_RIGHT: "turn_right",
        RouteIntent.TURN_LEFT: "turn_left",
        RouteIntent.LANE_CHANGE_LEFT: "lane_change_left",
        RouteIntent.LANE_CHANGE_RIGHT: "lane_change_right",
        RouteIntent.LEGACY_TURN: "legacy_turn",
        RouteIntent.LEGACY_LANE_CHANGE: "legacy_lane_change",
    }.get(intent, "unknown")


def route_intent_from_trajectory_kind(kind: TrajectoryKind) -> "RouteIntent":
    return {
        TrajectoryKind.TURN_RIGHT: RouteIntent.TURN_RIGHT,
        TrajectoryKind.TURN_LEFT: RouteIntent.TURN_LEFT,
        TrajectoryKind.LANE_CHANGE_LEFT: RouteIntent.LANE_CHANGE_LEFT,
        TrajectoryKind.LANE_CHANGE_RIGHT: RouteIntent.LANE_CHANGE_RIGHT,
    }.get(kind, RouteIntent.FOLLOW_MAIN)


class RouteIntent(Enum):
    FOLLOW_MAIN = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2
    LANE_CHANGE_LEFT = 3
    LANE_CHANGE_RIGHT = 4
    LEGACY_TURN = 5
    LEGACY_LANE_CHANGE = 6


@dataclass
class LaneObservation:
    lane_id: str = ""
    class_name: str = ""
    label: int = -1
    points: list[Point2D] = None
    confidence: float = 0.0
    heading_hint: float = 0.0
    has_precomputed_control: bool = False
    precomputed_epsilon_x_mm: float = 0.0
    precomputed_epsilon_y_mm: float = 0.0
    precomputed_theta_rad: float = 0.0
    precomputed_curvature_inv_mm: float = 0.0
    precomputed_lookahead_d_mm: float = 0.0
    raw_obj: dict[str, Any] = None

    def __post_init__(self):
        if self.points is None:
            self.points = []


@dataclass
class MarkingObservation:
    marking_id: str = ""
    class_name: str = ""
    label: int = -1
    points: list[Point2D] = None
    confidence: float = 1.0
    raw_obj: dict[str, Any] = None

    def __post_init__(self):
        if self.points is None:
            self.points = []


@dataclass
class PathObservationFrame:
    lanes: list[LaneObservation] = None
    markings: list[MarkingObservation] = None
    timestamp_ms: int = 0

    def __post_init__(self):
        if self.lanes is None:
            self.lanes = []
        if self.markings is None:
            self.markings = []


@dataclass
class PlannedTrajectory:
    points: list[Point2D] = None
    source_lane_ids: list[str] = None
    target_lane_id: str = ""
    trajectory_kind: TrajectoryKind = TrajectoryKind.UNKNOWN
    confidence: float = 0.0
    valid: bool = False
    blocked_by_marking: bool = False
    marking_confidence_low: bool = False
    normalization_mode: str = "none"
    has_precomputed_control: bool = False
    precomputed_epsilon_x_mm: float = 0.0
    precomputed_epsilon_y_mm: float = 0.0
    precomputed_theta_rad: float = 0.0
    precomputed_curvature_inv_mm: float = 0.0
    precomputed_lookahead_d_mm: float = 0.0

    def __post_init__(self):
        if self.points is None:
            self.points = []
        if self.source_lane_ids is None:
            self.source_lane_ids = []


@dataclass
class CommittedTrajectoryState:
    trajectory: PlannedTrajectory = None
    progress_s_mm: float = 0.0
    remaining_s_mm: float = 0.0
    last_good_frame: int = 0
    dropout_hold_counter: int = 0
    # Separate from dropout_hold_counter (transient-invalid dropout / maneuver-fallback
    # hold) - see control_node.cpp's CommittedTrajectoryState for why.
    low_conf_hold_counter: int = 0
    replan_reason: str = "none"
    committed_intent: RouteIntent = RouteIntent.FOLLOW_MAIN
    committed_intent_seq: int = 0

    def __post_init__(self):
        if self.trajectory is None:
            self.trajectory = PlannedTrajectory()


class ManagerAction(Enum):
    HOLD_CURRENT = 0
    UPDATE_CURRENT = 1
    COMMIT_NEW = 2
    ENTER_RECOVERY = 3
    ENTER_BLOCKED = 4


@dataclass
class PathDeviationMetrics:
    lateral_rms_mm: float = 0.0
    heading_rms_rad: float = 0.0
    curvature_max_delta: float = 0.0
    overlap_ratio: float = 1.0
    topology_changed: bool = False


class PathObservationBuilder:
    @staticmethod
    def build(telemetry: dict[str, Any]) -> PathObservationFrame:
        frame = PathObservationFrame()
        frame.timestamp_ms = telemetry.get("timestamp_ms", 0)

        if "objects" not in telemetry or not isinstance(telemetry["objects"], list):
            return frame

        for obj in telemetry["objects"]:
            label = obj.get("label", -1)
            class_name = obj.get("class_name", "")
            
            # Resolve ID
            id_str = ""
            if "id" in obj:
                id_val = obj["id"]
                if isinstance(id_val, str):
                    id_str = id_val
                elif id_val is not None:
                    id_str = str(id_val)
            if not id_str and "track_id" in obj:
                tid_val = obj["track_id"]
                if isinstance(tid_val, str):
                    id_str = tid_val
                elif tid_val is not None:
                    id_str = str(tid_val)
            if not id_str:
                id_str = f"obj_{label}_{len(frame.lanes) + len(frame.markings)}"

            if label in {LABEL_MAIN_LANE, LABEL_OTHER_LANE, LABEL_TURN_LANE}:
                lane_obs = LaneObservation(
                    lane_id=id_str,
                    class_name=class_name,
                    label=label,
                    raw_obj=obj
                )
                
                raw_points = []
                if "waypoints" in obj and isinstance(obj["waypoints"], list):
                    for pt in obj["waypoints"]:
                        if isinstance(pt, list) and len(pt) >= 2:
                            raw_points.append(Point2D(pt[0], pt[1]))

                # Sort points
                if label != LABEL_TURN_LANE:
                    raw_points.sort(key=lambda p: p.y)
                else:
                    if raw_points:
                        dist_front = raw_points[0].x**2 + raw_points[0].y**2
                        dist_back = raw_points[-1].x**2 + raw_points[-1].y**2
                        if dist_back < dist_front:
                            raw_points.reverse()

                # Remove duplicates (>10mm)
                if raw_points:
                    lane_obs.points.append(raw_points[0])
                    for i in range(1, len(raw_points)):
                        dist = math.sqrt((raw_points[i].x - lane_obs.points[-1].x)**2 + 
                                         (raw_points[i].y - lane_obs.points[-1].y)**2)
                        if dist > 10.0:
                            lane_obs.points.append(raw_points[i])

                # Precomputed control fields compatibility
                if len(lane_obs.points) < 2 and label == LABEL_TURN_LANE and "longitudinal_offset_mm" in obj:
                    lane_obs.has_precomputed_control = True
                    lane_obs.precomputed_epsilon_x_mm = 0.0
                    lane_obs.precomputed_epsilon_y_mm = float(obj["longitudinal_offset_mm"])
                    lane_obs.precomputed_theta_rad = float(obj.get("lookahead_theta_rad", 0.0))
                    lane_obs.precomputed_curvature_inv_mm = float(obj.get("curvature_inv_mm", 0.0))
                    lane_obs.precomputed_lookahead_d_mm = float(obj.get("lookahead_d_mm", lane_obs.precomputed_epsilon_y_mm))
                elif len(lane_obs.points) < 2 and "lookahead_x_mm" in obj and "lookahead_d_mm" in obj:
                    lane_obs.has_precomputed_control = True
                    lane_obs.precomputed_epsilon_x_mm = float(obj["lookahead_x_mm"])
                    lane_obs.precomputed_epsilon_y_mm = float(obj["lookahead_d_mm"])
                    lane_obs.precomputed_theta_rad = float(obj.get(
                        "lookahead_theta_rad",
                        math.atan2(lane_obs.precomputed_epsilon_x_mm, lane_obs.precomputed_epsilon_y_mm)
                    ))
                    lane_obs.precomputed_curvature_inv_mm = float(obj.get("curvature_inv_mm", 0.0))
                    lane_obs.precomputed_lookahead_d_mm = float(obj["lookahead_d_mm"])

                # Metadata & confidence
                if len(lane_obs.points) >= 2:
                    lane_obs.heading_hint = math.atan2(lane_obs.points[1].x - lane_obs.points[0].x, 
                                                       lane_obs.points[1].y - lane_obs.points[0].y)
                    dx = lane_obs.points[-1].x - lane_obs.points[0].x
                    dy = lane_obs.points[-1].y - lane_obs.points[0].y
                    total_len = math.sqrt(dx*dx + dy*dy)
                    len_factor = min(1.0, total_len / 5000.0)
                    pts_factor = min(1.0, len(lane_obs.points) / 10.0)
                    lane_obs.confidence = 0.5 * len_factor + 0.5 * pts_factor
                elif lane_obs.has_precomputed_control:
                    lane_obs.confidence = 1.0;
                    lane_obs.heading_hint = lane_obs.precomputed_theta_rad
                else:
                    lane_obs.confidence = 0.0

                lane_obs.confidence = float(obj.get("confidence", lane_obs.confidence))
                frame.lanes.append(lane_obs)

            elif label in {0, 1, 2, 13, 14, 16}:
                marking_obs = MarkingObservation(
                    marking_id=id_str,
                    class_name=class_name,
                    label=label,
                    raw_obj=obj
                )

                raw_points = []
                if "waypoints" in obj and isinstance(obj["waypoints"], list):
                    for pt in obj["waypoints"]:
                        if isinstance(pt, list) and len(pt) >= 2:
                            raw_points.append(Point2D(pt[0], pt[1]))

                raw_points.sort(key=lambda p: p.y)

                if raw_points:
                    marking_obs.points.append(raw_points[0])
                    for i in range(1, len(raw_points)):
                        dist = math.sqrt((raw_points[i].x - marking_obs.points[-1].x)**2 + 
                                         (raw_points[i].y - marking_obs.points[-1].y)**2)
                        if dist > 10.0:
                            marking_obs.points.append(raw_points[i])

                marking_obs.confidence = float(obj.get("confidence", 1.0))
                frame.markings.append(marking_obs)

        return frame


class TrajectoryPlanner:
    @staticmethod
    def plan_follow_main(obs: PathObservationFrame, 
                         prev_state: CommittedTrajectoryState, 
                         last_main_id: list[str]) -> PlannedTrajectory:
        plan = PlannedTrajectory(trajectory_kind=TrajectoryKind.FOLLOW_MAIN)
        
        cur_lane = TrajectoryPlanner.select_main_current(obs, last_main_id[0])
        
        if cur_lane:
            last_main_id[0] = cur_lane.lane_id
            plan.target_lane_id = cur_lane.lane_id
            plan.source_lane_ids.append(f"{cur_lane.label}:{cur_lane.lane_id}")
            
            ahead_lane = TrajectoryPlanner.select_main_ahead(obs, cur_lane)
            
            raw_path = []
            if ahead_lane:
                plan.source_lane_ids.append(f"{ahead_lane.label}:{ahead_lane.lane_id}")
                raw_path = TrajectoryPlanner.merge_lanes(cur_lane, ahead_lane)
            else:
                raw_path = cur_lane.points
                
            plan.points = TrajectoryPlanner.resample_path(raw_path, 100.0)
            plan.confidence = cur_lane.confidence
            plan.valid = len(plan.points) >= 2
            
            if cur_lane.has_precomputed_control:
                plan.has_precomputed_control = True
                plan.precomputed_epsilon_x_mm = cur_lane.precomputed_epsilon_x_mm
                plan.precomputed_epsilon_y_mm = cur_lane.precomputed_epsilon_y_mm
                plan.precomputed_theta_rad = cur_lane.precomputed_theta_rad
                plan.precomputed_curvature_inv_mm = cur_lane.precomputed_curvature_inv_mm
                plan.precomputed_lookahead_d_mm = cur_lane.precomputed_lookahead_d_mm
                if not plan.valid:
                    plan.valid = True
                    plan.confidence = cur_lane.confidence
        else:
            if prev_state.trajectory.valid and prev_state.trajectory.trajectory_kind == TrajectoryKind.FOLLOW_MAIN:
                plan.points = prev_state.trajectory.points
                plan.target_lane_id = prev_state.trajectory.target_lane_id
                plan.source_lane_ids = prev_state.trajectory.source_lane_ids
                plan.confidence = max(0.05, prev_state.trajectory.confidence * 0.8)  # mirror: memory-hold decay floor (sync with trajectory_planner.hpp)
                plan.valid = True
                
                plan.has_precomputed_control = prev_state.trajectory.has_precomputed_control
                plan.precomputed_epsilon_x_mm = prev_state.trajectory.precomputed_epsilon_x_mm
                plan.precomputed_epsilon_y_mm = prev_state.trajectory.precomputed_epsilon_y_mm
                plan.precomputed_theta_rad = prev_state.trajectory.precomputed_theta_rad
                plan.precomputed_curvature_inv_mm = prev_state.trajectory.precomputed_curvature_inv_mm
                plan.precomputed_lookahead_d_mm = prev_state.trajectory.precomputed_lookahead_d_mm
            else:
                plan.valid = False
                plan.confidence = 0.0
                last_main_id[0] = ""
                
        return plan

    @staticmethod
    def plan_turn_right(obs: PathObservationFrame, 
                        prev_state: CommittedTrajectoryState, 
                        is_t: bool, 
                        t_junction_pending: bool, 
                        last_main_id: list[str]) -> PlannedTrajectory:
        return TrajectoryPlanner.plan_turn_generic(obs, prev_state, True, is_t, last_main_id)

    @staticmethod
    def plan_turn_left(obs: PathObservationFrame, 
                       prev_state: CommittedTrajectoryState, 
                       is_t: bool, 
                       t_junction_pending: bool, 
                       last_main_id: list[str]) -> PlannedTrajectory:
        return TrajectoryPlanner.plan_turn_generic(obs, prev_state, False, is_t, last_main_id)

    @staticmethod
    def plan_turn_generic(obs: PathObservationFrame, 
                          prev_state: CommittedTrajectoryState, 
                          is_right_turn: bool, 
                          is_t: bool, 
                          last_main_id: list[str]) -> PlannedTrajectory:
        plan = PlannedTrajectory(trajectory_kind=TrajectoryKind.TURN_RIGHT if is_right_turn else TrajectoryKind.TURN_LEFT)
        
        selected_turn = TrajectoryPlanner.select_turn_lane_obs(obs, is_right_turn, is_t)
        cur_main = TrajectoryPlanner.select_main_current(obs, last_main_id[0])
        if cur_main:
            last_main_id[0] = cur_main.lane_id

        if selected_turn and selected_turn.has_precomputed_control:
            plan.points = list(selected_turn.points)
            if not plan.points:
                plan.points = [
                    Point2D(0.0, 0.0),
                    Point2D(selected_turn.precomputed_epsilon_x_mm * 0.5, selected_turn.precomputed_epsilon_y_mm * 0.5),
                    Point2D(selected_turn.precomputed_epsilon_x_mm, selected_turn.precomputed_epsilon_y_mm)
                ]
            plan.target_lane_id = selected_turn.lane_id
            if cur_main:
                plan.source_lane_ids.append(f"{cur_main.label}:{cur_main.lane_id}")
            plan.source_lane_ids.append(f"{selected_turn.label}:{selected_turn.lane_id}")
            plan.confidence = selected_turn.confidence
            plan.valid = True
            plan.has_precomputed_control = True
            plan.precomputed_epsilon_x_mm = selected_turn.precomputed_epsilon_x_mm
            plan.precomputed_epsilon_y_mm = selected_turn.precomputed_epsilon_y_mm
            plan.precomputed_theta_rad = selected_turn.precomputed_theta_rad
            plan.precomputed_curvature_inv_mm = selected_turn.precomputed_curvature_inv_mm
            plan.precomputed_lookahead_d_mm = selected_turn.precomputed_lookahead_d_mm
            return plan

        if cur_main and selected_turn:
            transition_pts = TrajectoryPlanner.plan_transition(
                cur_main.points, selected_turn.points, TrajectoryPlanner.TURN_MAX_HEADING_DIFF_RAD)
            if transition_pts:
                plan.points = TrajectoryPlanner.resample_path(transition_pts, 100.0)
                plan.target_lane_id = selected_turn.lane_id
                plan.source_lane_ids.append(f"{cur_main.label}:{cur_main.lane_id}")
                plan.source_lane_ids.append(f"{selected_turn.label}:{selected_turn.lane_id}")
                plan.confidence = selected_turn.confidence
                plan.valid = len(plan.points) >= 2
                
            if not plan.valid:
                plan = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
                plan.trajectory_kind = TrajectoryKind.FOLLOW_MAIN
        elif selected_turn:
            plan.points = TrajectoryPlanner.resample_path(selected_turn.points, 100.0)
            plan.target_lane_id = selected_turn.lane_id
            plan.source_lane_ids.append(f"{selected_turn.label}:{selected_turn.lane_id}")
            plan.confidence = selected_turn.confidence
            plan.valid = len(plan.points) >= 2
        elif cur_main:
            # Turn-lane momentarily lost. Prefer holding the previous committed turn
            # trajectory (memory fallback, mirrors plan_follow_main's own no-detection
            # fallback) over silently downgrading to follow_main, so a brief perception
            # dropout does not discard an in-progress maneuver.
            expected_kind = TrajectoryKind.TURN_RIGHT if is_right_turn else TrajectoryKind.TURN_LEFT
            if prev_state.trajectory.valid and prev_state.trajectory.trajectory_kind == expected_kind:
                plan.points = list(prev_state.trajectory.points)
                plan.target_lane_id = prev_state.trajectory.target_lane_id
                plan.source_lane_ids = list(prev_state.trajectory.source_lane_ids)
                plan.confidence = max(0.05, prev_state.trajectory.confidence * 0.8)  # mirror: memory-hold decay floor (sync with trajectory_planner.hpp)
                plan.valid = True

                plan.has_precomputed_control = prev_state.trajectory.has_precomputed_control
                plan.precomputed_epsilon_x_mm = prev_state.trajectory.precomputed_epsilon_x_mm
                plan.precomputed_epsilon_y_mm = prev_state.trajectory.precomputed_epsilon_y_mm
                plan.precomputed_theta_rad = prev_state.trajectory.precomputed_theta_rad
                plan.precomputed_curvature_inv_mm = prev_state.trajectory.precomputed_curvature_inv_mm
                plan.precomputed_lookahead_d_mm = prev_state.trajectory.precomputed_lookahead_d_mm
            else:
                plan = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
                plan.trajectory_kind = TrajectoryKind.FOLLOW_MAIN
        else:
            plan.valid = False
            plan.confidence = 0.0

        return plan

    @staticmethod
    def plan_lane_change_left(obs: PathObservationFrame, 
                               prev_state: CommittedTrajectoryState, 
                               last_main_id: list[str]) -> PlannedTrajectory:
        return TrajectoryPlanner.plan_lane_change_generic(obs, prev_state, True, last_main_id)

    @staticmethod
    def plan_lane_change_right(obs: PathObservationFrame, 
                                prev_state: CommittedTrajectoryState, 
                                last_main_id: list[str]) -> PlannedTrajectory:
        return TrajectoryPlanner.plan_lane_change_generic(obs, prev_state, False, last_main_id)

    @staticmethod
    def plan_lane_change_generic(obs: PathObservationFrame, 
                                 prev_state: CommittedTrajectoryState, 
                                 is_left_change: bool, 
                                 last_main_id: list[str]) -> PlannedTrajectory:
        plan = PlannedTrajectory(trajectory_kind=TrajectoryKind.LANE_CHANGE_LEFT if is_left_change else TrajectoryKind.LANE_CHANGE_RIGHT)
        
        cur_main = TrajectoryPlanner.select_main_current(obs, last_main_id[0])
        if cur_main:
            last_main_id[0] = cur_main.lane_id
            
        target_other = TrajectoryPlanner.select_other_lane_obs(obs, cur_main, is_left_change)
        
        if cur_main and target_other:
            confidence_out: dict = {}
            blocked = TrajectoryPlanner.is_lane_change_blocked_by_solid_obs(cur_main, target_other, obs.markings, confidence_out)
            if blocked:
                plan = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
                plan.blocked_by_marking = True
                plan.trajectory_kind = TrajectoryKind.FOLLOW_MAIN
            else:
                plan.marking_confidence_low = confidence_out.get("marking_confidence_low", False)
                transition_pts = TrajectoryPlanner.plan_transition(cur_main.points, target_other.points)
                if transition_pts:
                    plan.points = TrajectoryPlanner.resample_path(transition_pts, 100.0)
                    plan.target_lane_id = target_other.lane_id
                    plan.source_lane_ids.append(f"{cur_main.label}:{cur_main.lane_id}")
                    plan.source_lane_ids.append(f"{target_other.label}:{target_other.lane_id}")
                    plan.confidence = target_other.confidence
                    plan.valid = len(plan.points) >= 2
                    
                if not plan.valid:
                    plan = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
                    plan.trajectory_kind = TrajectoryKind.FOLLOW_MAIN
        elif cur_main:
            # Target other lane momentarily lost. Prefer holding the previous committed
            # lane-change trajectory (memory fallback, mirrors plan_follow_main's own
            # no-detection fallback and plan_turn_generic's analogous fallback) over
            # silently downgrading to follow_main on a brief perception dropout.
            expected_kind = TrajectoryKind.LANE_CHANGE_LEFT if is_left_change else TrajectoryKind.LANE_CHANGE_RIGHT
            if prev_state.trajectory.valid and prev_state.trajectory.trajectory_kind == expected_kind:
                plan.points = list(prev_state.trajectory.points)
                plan.target_lane_id = prev_state.trajectory.target_lane_id
                plan.source_lane_ids = list(prev_state.trajectory.source_lane_ids)
                plan.confidence = max(0.05, prev_state.trajectory.confidence * 0.8)  # mirror: memory-hold decay floor (sync with trajectory_planner.hpp)
                plan.valid = True

                plan.has_precomputed_control = prev_state.trajectory.has_precomputed_control
                plan.precomputed_epsilon_x_mm = prev_state.trajectory.precomputed_epsilon_x_mm
                plan.precomputed_epsilon_y_mm = prev_state.trajectory.precomputed_epsilon_y_mm
                plan.precomputed_theta_rad = prev_state.trajectory.precomputed_theta_rad
                plan.precomputed_curvature_inv_mm = prev_state.trajectory.precomputed_curvature_inv_mm
                plan.precomputed_lookahead_d_mm = prev_state.trajectory.precomputed_lookahead_d_mm
            else:
                plan = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
                plan.trajectory_kind = TrajectoryKind.FOLLOW_MAIN
        elif target_other:
            plan.points = TrajectoryPlanner.resample_path(target_other.points, 100.0)
            plan.target_lane_id = target_other.lane_id
            plan.source_lane_ids.append(f"{target_other.label}:{target_other.lane_id}")
            plan.confidence = target_other.confidence
            plan.valid = len(plan.points) >= 2
        else:
            plan.valid = False
            plan.confidence = 0.0

        return plan

    @staticmethod
    def plan_candidate_for_intent(obs: PathObservationFrame,
                                   intent: RouteIntent,
                                   prev_state: CommittedTrajectoryState,
                                   is_t: bool,
                                   t_junction_pending: bool,
                                   last_main_id: list[str]) -> PlannedTrajectory:
        if intent == RouteIntent.TURN_RIGHT:
            return TrajectoryPlanner.plan_turn_right(obs, prev_state, is_t, t_junction_pending, last_main_id)
        if intent == RouteIntent.TURN_LEFT:
            return TrajectoryPlanner.plan_turn_left(obs, prev_state, is_t, t_junction_pending, last_main_id)
        if intent == RouteIntent.LANE_CHANGE_LEFT:
            return TrajectoryPlanner.plan_lane_change_left(obs, prev_state, last_main_id)
        if intent == RouteIntent.LANE_CHANGE_RIGHT:
            return TrajectoryPlanner.plan_lane_change_right(obs, prev_state, last_main_id)
        return TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)

    @staticmethod
    def get_lane_heading_obs(lane: LaneObservation) -> float:
        if len(lane.points) < 2:
            return lane.precomputed_theta_rad if lane.has_precomputed_control else 0.0
        end_idx = min(len(lane.points) - 1, 3)
        dx = lane.points[end_idx].x - lane.points[0].x
        dy = lane.points[end_idx].y - lane.points[0].y
        return math.atan2(dx, dy)

    @staticmethod
    def select_other_lane_obs(obs: PathObservationFrame, 
                              main_lane: LaneObservation | None, 
                              is_left_change: bool) -> LaneObservation | None:
        other_lanes = [l for l in obs.lanes if l.label == LABEL_OTHER_LANE or l.class_name == "other-lane"]
        if not other_lanes:
            return None

        main_x = 0.0
        main_heading = 0.0
        
        if main_lane:
            if main_lane.points:
                main_x = sum(p.x for p in main_lane.points) / len(main_lane.points)
                main_heading = TrajectoryPlanner.get_lane_heading_obs(main_lane)
            elif main_lane.has_precomputed_control:
                main_x = main_lane.precomputed_epsilon_x_mm
                main_heading = main_lane.precomputed_theta_rad

        best_cand = None
        best_score = -1e9

        for l in other_lanes:
            other_x = 0.0
            other_heading = 0.0
            min_y = 0.0
            
            if l.points:
                other_x = sum(p.x for p in l.points) / len(l.points)
                other_heading = TrajectoryPlanner.get_lane_heading_obs(l)
                min_y = min(p.y for p in l.points)
            elif l.has_precomputed_control:
                other_x = l.precomputed_epsilon_x_mm
                other_heading = l.precomputed_theta_rad
                min_y = 0.0
            else:
                continue

            lateral_dist = other_x - main_x
            
            # hard filters
            if is_left_change and lateral_dist > -200.0: continue
            if not is_left_change and lateral_dist < 200.0: continue
            
            diff_theta = abs(other_heading - main_heading)
            while diff_theta > math.pi: diff_theta -= 2.0 * math.pi
            while diff_theta < -math.pi: diff_theta += 2.0 * math.pi
            diff_theta = abs(diff_theta)
            if diff_theta > (30.0 * math.pi / 180.0): continue
            
            abs_lat_dist = abs(lateral_dist)
            if abs_lat_dist < 400.0 or abs_lat_dist > 1400.0: continue
            if min_y > 1200.0: continue

            # scoring: nearest lateral distance to main wins (decision_sys.md: "chọn
            # lane gần main nhất"), the 400-1400mm gate above excludes implausible slivers
            score = -abs_lat_dist - 1000.0 * diff_theta
            if score > best_score:
                best_score = score
                best_cand = l
                
        return best_cand

    @staticmethod
    def is_lane_change_blocked_by_solid_obs(main_lane: LaneObservation | None,
                                            target_lane: LaneObservation | None,
                                            markings: list[MarkingObservation],
                                            confidence_out: dict | None = None) -> bool:
        if confidence_out is not None:
            confidence_out["marking_confidence_low"] = False
        if not main_lane or not target_lane:
            return False

        def get_x(l):
            if l.raw_obj and "lookahead_x_mm" in l.raw_obj:
                return float(l.raw_obj["lookahead_x_mm"])
            if l.points:
                return l.points[0].x
            if l.has_precomputed_control:
                return l.precomputed_epsilon_x_mm
            return 0.0

        main_x = get_x(main_lane)
        target_x = get_x(target_lane)
        min_x, max_x = sorted((main_x, target_x))

        p0_y = 0.0
        p3_y = 2000.0
        
        main_wps = sorted(main_lane.points, key=lambda p: p.y)
        target_wps = sorted(target_lane.points, key=lambda p: p.y)

        cum_dist = 0.0
        if main_wps:
            p0_y = main_wps[0].y
            for i in range(1, len(main_wps)):
                dx = main_wps[i].x - main_wps[i-1].x
                dy = main_wps[i].y - main_wps[i-1].y
                cum_dist += math.hypot(dx, dy)
                if cum_dist >= 300.0:
                    p0_y = main_wps[i].y
                    break
                    
        cum_dist = 0.0
        if target_wps:
            p3_y = target_wps[-1].y
            for i in range(1, len(target_wps)):
                dx = target_wps[i].x - target_wps[i-1].x
                dy = target_wps[i].y - target_wps[i-1].y
                cum_dist += math.hypot(dx, dy)
                if cum_dist >= 1200.0:
                    p3_y = target_wps[i].y
                    break

        y_min = min(p0_y, p3_y) - 100.0
        y_max = max(p0_y, p3_y) + 300.0

        any_marking_in_corridor = False
        for m in markings:
            is_solid = m.label in {LABEL_DOUBLE_SOLID_WHITE, LABEL_SOLID_WHITE, LABEL_SOLID_YELLOW}
            is_dashed = m.label in {LABEL_DASHED_WHITE, LABEL_DASHED_YELLOW}
            if is_solid or is_dashed:
                is_between = False
                raw = m.raw_obj or {}
                if "lookahead_x_mm" in raw and "waypoints" not in raw and "polygons_real_world" not in raw:
                    mark_x = float(raw["lookahead_x_mm"])
                    mark_y = 600.0
                    is_between = (min_x < mark_x < max_x and y_min <= mark_y <= y_max)
                elif "waypoints" in raw and raw["waypoints"]:
                    for wp in raw["waypoints"]:
                        mark_x = float(wp[0])
                        mark_y = float(wp[1])
                        if min_x < mark_x < max_x and y_min <= mark_y <= y_max:
                            is_between = True
                            break
                elif "polygons_real_world" in raw and raw["polygons_real_world"]:
                    for poly in raw["polygons_real_world"]:
                        if isinstance(poly, list):
                            for pt in poly:
                                if isinstance(pt, list) and len(pt) >= 2:
                                    mark_x = float(pt[0])
                                    mark_y = float(pt[1])
                                    if min_x < mark_x < max_x and y_min <= mark_y <= y_max:
                                        is_between = True
                                        break
                        if is_between:
                            break

                if is_between:
                    any_marking_in_corridor = True
                    if is_solid:
                        return True
        if confidence_out is not None and not any_marking_in_corridor:
            confidence_out["marking_confidence_low"] = True
        return False

    @staticmethod
    def select_turn_lane_obs(obs: PathObservationFrame, 
                             is_turn_right: bool, 
                             is_t_junction: bool) -> LaneObservation | None:
        turn_lanes = [l for l in obs.lanes if l.label == LABEL_TURN_LANE]
        if not turn_lanes:
            return None

        # identify if any candidate is on the strict correct side
        correct_side_exists = False
        for l in turn_lanes:
            avg_x = 0.0
            if l.points:
                avg_x = sum(p.x for p in l.points) / len(l.points)
            elif l.has_precomputed_control:
                avg_x = l.precomputed_epsilon_x_mm
            if is_turn_right and avg_x >= 0.0: correct_side_exists = True
            if not is_turn_right and avg_x <= 0.0: correct_side_exists = True

        scored_lanes = []
        for l in turn_lanes:
            min_dist = 1e9
            avg_x = 0.0
            
            if l.points:
                avg_x = sum(p.x for p in l.points) / len(l.points)
                for pt in l.points:
                    dist = math.sqrt(pt.x**2 + pt.y**2)
                    if dist < min_dist:
                        min_dist = dist
            elif l.has_precomputed_control:
                min_dist = l.precomputed_lookahead_d_mm
                avg_x = l.precomputed_epsilon_x_mm
            else:
                continue

            if not is_t_junction:
                if is_turn_right and avg_x < 0: continue
                if not is_turn_right and avg_x > 0: continue
            else:
                if correct_side_exists:
                    if is_turn_right and avg_x < 0.0: continue
                    if not is_turn_right and avg_x > 0.0: continue
                else:
                    if is_turn_right and avg_x < -200.0: continue
                    if not is_turn_right and avg_x > 200.0: continue

            scored_lanes.append((min_dist, l))

        if not scored_lanes:
            return None

        scored_lanes.sort(key=lambda x: x[0])
        if is_turn_right:
            return scored_lanes[0][1] # closest
        else:
            return scored_lanes[-1][1] # farthest

    # Mirrors trajectory_planner.hpp kLaneChangeMaxHeadingDiffRad / kTurnMaxHeadingDiffRad
    LANE_CHANGE_MAX_HEADING_DIFF_RAD = 40.0 * math.pi / 180.0
    TURN_MAX_HEADING_DIFF_RAD = 110.0 * math.pi / 180.0

    @staticmethod
    def plan_transition(current_pts: list[Point2D], target_pts: list[Point2D],
                        max_heading_diff_rad: float = LANE_CHANGE_MAX_HEADING_DIFF_RAD) -> list[Point2D]:
        if len(current_pts) < 2 or len(target_pts) < 2:
            return []

        cur_heading = math.atan2(current_pts[1].x - current_pts[0].x, current_pts[1].y - current_pts[0].y)
        target_heading = math.atan2(target_pts[1].x - target_pts[0].x, target_pts[1].y - target_pts[0].y)
        
        cur_x = current_pts[0].x
        target_x = target_pts[0].x
        lat_dist = abs(target_x - cur_x)
        
        heading_diff = abs(target_heading - cur_heading)
        while heading_diff > math.pi: heading_diff -= 2.0 * math.pi
        while heading_diff < -math.pi: heading_diff += 2.0 * math.pi
        heading_diff = abs(heading_diff)

        if lat_dist > 1500.0 or heading_diff > max_heading_diff_rad:
            return []

        P0 = current_pts[0]
        p_prev = P0
        cum_dist = 0.0
        split_idx_current = 0
        for i in range(1, len(current_pts)):
            cum_dist += math.hypot(current_pts[i].x - current_pts[i-1].x, current_pts[i].y - current_pts[i-1].y)
            if cum_dist >= 300.0:
                P0 = current_pts[i]
                p_prev = current_pts[i-1]
                split_idx_current = i
                break
        if split_idx_current == 0 and len(current_pts) > 1:
            split_idx_current = 1
            P0 = current_pts[1]
            p_prev = current_pts[0]

        P3 = target_pts[-1]
        p_next = P3
        cum_dist = 0.0
        split_idx_target = len(target_pts) - 1
        for i in range(1, len(target_pts)):
            cum_dist += math.hypot(target_pts[i].x - target_pts[i-1].x, target_pts[i].y - target_pts[i-1].y)
            if cum_dist >= 1200.0:
                P3 = target_pts[i]
                p_next = target_pts[i+1] if i + 1 < len(target_pts) else P3
                split_idx_target = i
                break
        if split_idx_target == len(target_pts) - 1 and len(target_pts) > 1:
            split_idx_target = len(target_pts) // 2
            if split_idx_target == 0: split_idx_target = 1
            P3 = target_pts[split_idx_target]
            p_next = target_pts[split_idx_target+1] if split_idx_target+1 < len(target_pts) else P3

        dx0 = P0.x - p_prev.x
        dy0 = P0.y - p_prev.y
        len0 = math.sqrt(dx0*dx0 + dy0*dy0)
        if len0 < 1e-3: dx0, dy0 = 0.0, 1.0
        else: dx0, dy0 = dx0 / len0, dy0 / len0
        
        dx3 = p_next.x - P3.x
        dy3 = p_next.y - P3.y
        len3 = math.sqrt(dx3*dx3 + dy3*dy3)
        if len3 < 1e-3: dx3, dy3 = 0.0, 1.0
        else: dx3, dy3 = dx3 / len3, dy3 / len3

        dist = math.sqrt((P3.x - P0.x)**2 + (P3.y - P0.y)**2)
        scale = dist / 3.0
        
        P1 = Point2D(P0.x + dx0 * scale, P0.y + dy0 * scale)
        P2 = Point2D(P3.x - dx3 * scale, P3.y - dy3 * scale)

        result = []
        for i in range(split_idx_current + 1):
            result.append(current_pts[i])

        num_samples = max(10, int(dist / 50.0))
        for i in range(1, num_samples):
            t = i / num_samples
            u = 1.0 - t
            w0 = u**3
            w1 = 3.0 * u**2 * t
            w2 = 3.0 * u * t**2
            w3 = t**3
            bx = w0*P0.x + w1*P1.x + w2*P2.x + w3*P3.x
            by = w0*P0.y + w1*P1.y + w2*P2.y + w3*P3.y
            result.append(Point2D(bx, by))

        for i in range(split_idx_target, len(target_pts)):
            result.append(target_pts[i])

        return result

    @staticmethod
    def select_main_current(obs: PathObservationFrame, last_main_id: str) -> LaneObservation | None:
        main_lanes = [l for l in obs.lanes if l.label == LABEL_MAIN_LANE or l.class_name == "main-lane"]
        if not main_lanes:
            return None
            
        min_ys = [l.points[0].y for l in main_lanes if l.points]
        min_start_y = min(min_ys) if min_ys else 1e9
        
        best_lane = None
        best_score = 1e9
        
        for l in main_lanes:
            has_wps = bool(l.points)
            if has_wps:
                start_x = l.points[0].x
                start_y = l.points[0].y
            elif l.has_precomputed_control:
                start_x = 0.0
                start_y = 0.0
            else:
                continue
                
            score = abs(start_x) + 0.5 * start_y
            if not has_wps:
                score += 5000.0
                
            if last_main_id and l.lane_id == last_main_id:
                if start_y - min_start_y <= 600.0:
                    score -= 1500.0
                    
            if score < best_score:
                best_score = score
                best_lane = l
                
        return best_lane if best_lane else main_lanes[0]

    @staticmethod
    def select_main_ahead(obs: PathObservationFrame, cur_lane: LaneObservation) -> LaneObservation | None:
        if not cur_lane or len(cur_lane.points) < 2:
            return None
            
        cur_end_x = cur_lane.points[-1].x
        cur_end_y = cur_lane.points[-1].y
        cur_prev_x = cur_lane.points[-2].x
        cur_prev_y = cur_lane.points[-2].y
        cur_theta = math.atan2(cur_end_x - cur_prev_x, cur_end_y - cur_prev_y)
        
        best_ahead = None
        best_ahead_y = 1e9
        
        for l in obs.lanes:
            is_main = (l.label == LABEL_MAIN_LANE or l.class_name == "main-lane")
            if l == cur_lane or not is_main or len(l.points) < 2:
                continue
                
            ahead_start_x = l.points[0].x
            ahead_start_y = l.points[0].y
            ahead_next_x = l.points[1].x
            ahead_next_y = l.points[1].y
            ahead_theta = math.atan2(ahead_next_x - ahead_start_x, ahead_next_y - ahead_start_y)
            
            long_gap = ahead_start_y - cur_end_y
            if long_gap < -500.0 or long_gap > 2000.0: continue
            
            lat_jump = abs(ahead_start_x - cur_end_x)
            if lat_jump > 400.0: continue
            
            diff_theta = abs(ahead_theta - cur_theta)
            while diff_theta > math.pi: diff_theta -= 2.0 * math.pi
            while diff_theta < -math.pi: diff_theta += 2.0 * math.pi
            diff_theta = abs(diff_theta)
            if diff_theta > (30.0 * math.pi / 180.0): continue
            
            if ahead_start_y < best_ahead_y:
                best_ahead_y = ahead_start_y
                best_ahead = l
                
        return best_ahead

    @staticmethod
    def merge_lanes(cur: LaneObservation, ahead: LaneObservation) -> list[Point2D]:
        merged = list(cur.points)
        if not ahead.points:
            return merged
            
        end_y = cur.points[-1].y if cur.points else -1e9
        for pt in ahead.points:
            if pt.y > end_y + 10.0:
                merged.append(pt)
        return merged

    @staticmethod
    def resample_path(points: list[Point2D], step_mm: float, start_offset_mm: float = 0.0) -> list[Point2D]:
        resampled = []
        if not points:
            return resampled
        if len(points) == 1:
            resampled.append(points[0])
            return resampled

        if start_offset_mm <= 0.0:
            resampled.append(points[0])
        accumulated_dist = 0.0

        next_idx = 1
        current_s = 0.0 if start_offset_mm <= 0.0 else (start_offset_mm - step_mm)

        while next_idx < len(points):
            p0 = points[next_idx - 1]
            p1 = points[next_idx]
            seg_len = math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2)

            if seg_len < 1e-3:
                next_idx += 1
                continue

            target_s = current_s + step_mm
            if accumulated_dist + seg_len >= target_s:
                ratio = (target_s - accumulated_dist) / seg_len
                interpolated = Point2D(p0.x + ratio * (p1.x - p0.x), p0.y + ratio * (p1.y - p0.y))
                resampled.append(interpolated)
                current_s = target_s
            else:
                accumulated_dist += seg_len
                next_idx += 1

        if resampled:
            dist = math.sqrt((points[-1].x - resampled[-1].x)**2 + (points[-1].y - resampled[-1].y)**2)
            if dist > 10.0:
                resampled.append(points[-1])

        return resampled

    @staticmethod
    def project_point_to_path(query: Point2D, path: list[Point2D]) -> float:
        if len(path) < 2:
            return 0.0
        best_dist_sq = float("inf")
        best_s = 0.0
        cum = 0.0
        for i in range(1, len(path)):
            a = path[i - 1]
            b = path[i]
            seg_len = math.sqrt((b.x - a.x)**2 + (b.y - a.y)**2)
            if seg_len > 1e-6:
                t = ((query.x - a.x) * (b.x - a.x) + (query.y - a.y) * (b.y - a.y)) / (seg_len * seg_len)
                t = max(0.0, min(1.0, t))
                px = a.x + t * (b.x - a.x)
                py = a.y + t * (b.y - a.y)
                d2 = (px - query.x)**2 + (py - query.y)**2
                if d2 < best_dist_sq:
                    best_dist_sq = d2
                    best_s = cum + t * seg_len
                cum += seg_len
        return max(0.0, best_s)


class TrajectoryNormalizer:
    @staticmethod
    def normalize(current_candidate: PlannedTrajectory,
                  previous_state: CommittedTrajectoryState,
                  continuity_heading_jump_rad: float = 0.35,
                  continuity_lateral_jump_mm: float = 300.0) -> PlannedTrajectory:
        normalized = current_candidate

        if current_candidate.blocked_by_marking:
            normalized.normalization_mode = "blocked_passthrough"
            return normalized

        if not previous_state.trajectory.valid or not previous_state.trajectory.points:
            normalized.normalization_mode = "no_previous_passthrough"
            return normalized

        if not current_candidate.valid or not current_candidate.points:
            normalized.normalization_mode = "invalid_candidate_passthrough"
            return normalized

        # Blending across a topology change by array index is not meaningful geometry
        # and produces a kink - passthrough the fresh candidate instead.
        if current_candidate.trajectory_kind != previous_state.trajectory.trajectory_kind:
            normalized.normalization_mode = "kind_mismatch_passthrough"
            return normalized

        prev_pts_raw = previous_state.trajectory.points
        cur_pts = current_candidate.points

        # Align the previous committed path onto the candidate's arc-length origin.
        progress_offset = TrajectoryPlanner.project_point_to_path(cur_pts[0], prev_pts_raw)
        prev_pts = TrajectoryPlanner.resample_path(prev_pts_raw, 100.0, progress_offset)

        if len(prev_pts) < 2:
            normalized.normalization_mode = "kind_mismatch_passthrough"
            return normalized

        blended_pts = []
        common_size = min(len(prev_pts), len(cur_pts))

        C = current_candidate.confidence
        L_trans = 3000.0

        w_cur_max = 0.2 + 0.7 * C
        w_cur_min = 0.05 + 0.15 * C

        for i in range(common_size):
            s = i * 100.0
            alpha = min(1.0, s / L_trans)
            w_cur = w_cur_min + alpha * (w_cur_max - w_cur_min)
            w_prev = 1.0 - w_cur

            pt = Point2D(
                w_prev * prev_pts[i].x + w_cur * cur_pts[i].x,
                w_prev * prev_pts[i].y + w_cur * cur_pts[i].y
            )
            blended_pts.append(pt)

        if len(cur_pts) > common_size:
            for i in range(common_size, len(cur_pts)):
                blended_pts.append(cur_pts[i])
        elif len(prev_pts) > common_size:
            for i in range(common_size, len(prev_pts)):
                blended_pts.append(prev_pts[i])

        seam_index, has_violation = TrajectoryNormalizer._find_continuity_violation(
            blended_pts, continuity_lateral_jump_mm, continuity_heading_jump_rad)
        if has_violation:
            bridged = TrajectoryNormalizer._bridge_local_discontinuity(blended_pts, seam_index)
            _, recheck_violation = TrajectoryNormalizer._find_continuity_violation(
                bridged, continuity_lateral_jump_mm, continuity_heading_jump_rad) if len(bridged) >= 2 else (0, True)
            if len(bridged) >= 2 and not recheck_violation:
                normalized.points = bridged
                normalized.valid = len(normalized.points) >= 2
                normalized.normalization_mode = "temporal_blend_bridged"
                return normalized
            normalized.points = blended_pts
            normalized.valid = False
            normalized.normalization_mode = "continuity_guard_invalid"
            return normalized

        normalized.points = blended_pts
        normalized.valid = len(normalized.points) >= 2
        normalized.normalization_mode = "temporal_blend"

        return normalized

    @staticmethod
    def _find_continuity_violation(points: list[Point2D], max_lateral_jump_mm: float,
                                    max_heading_jump_rad: float) -> tuple[int, bool]:
        for i in range(2, len(points)):
            h1 = math.atan2(points[i-1].x - points[i-2].x, points[i-1].y - points[i-2].y)
            h2 = math.atan2(points[i].x - points[i-1].x, points[i].y - points[i-1].y)
            dh = h2 - h1
            while dh > math.pi:
                dh -= 2.0 * math.pi
            while dh < -math.pi:
                dh += 2.0 * math.pi

            dx = points[i-1].x - points[i-2].x
            dy = points[i-1].y - points[i-2].y
            seg_len = math.sqrt(dx*dx + dy*dy)
            lateral_jump_mm = 0.0
            if seg_len > 1e-6:
                nx = -dy / seg_len
                ny = dx / seg_len
                lateral_jump_mm = abs((points[i].x - points[i-1].x) * nx +
                                       (points[i].y - points[i-1].y) * ny)

            if abs(dh) > max_heading_jump_rad or lateral_jump_mm > max_lateral_jump_mm:
                return i, True
        return 0, False

    @staticmethod
    def _bridge_local_discontinuity(points: list[Point2D], seam_index: int) -> list[Point2D]:
        if len(points) < 4 or seam_index < 1 or seam_index >= len(points):
            return points

        anchor_dist_mm = 300.0

        before_idx = seam_index - 1
        cum = 0.0
        while before_idx > 0:
            cum += math.hypot(points[before_idx].x - points[before_idx - 1].x,
                               points[before_idx].y - points[before_idx - 1].y)
            if cum >= anchor_dist_mm:
                break
            before_idx -= 1

        after_idx = seam_index
        cum = 0.0
        while after_idx + 1 < len(points):
            cum += math.hypot(points[after_idx + 1].x - points[after_idx].x,
                               points[after_idx + 1].y - points[after_idx].y)
            if cum >= anchor_dist_mm:
                break
            after_idx += 1

        if after_idx <= before_idx + 1:
            return points

        P0 = points[before_idx]
        p_prev = points[before_idx - 1] if before_idx > 0 else points[before_idx]
        P3 = points[after_idx]
        p_next = points[after_idx + 1] if after_idx + 1 < len(points) else points[after_idx]

        dx0, dy0 = P0.x - p_prev.x, P0.y - p_prev.y
        len0 = math.sqrt(dx0*dx0 + dy0*dy0)
        if len0 < 1e-3:
            dx0, dy0 = 0.0, 1.0
        else:
            dx0, dy0 = dx0 / len0, dy0 / len0

        dx3, dy3 = p_next.x - P3.x, p_next.y - P3.y
        len3 = math.sqrt(dx3*dx3 + dy3*dy3)
        if len3 < 1e-3:
            dx3, dy3 = 0.0, 1.0
        else:
            dx3, dy3 = dx3 / len3, dy3 / len3

        dist = math.hypot(P3.x - P0.x, P3.y - P0.y)
        scale = dist / 3.0
        P1 = Point2D(P0.x + dx0 * scale, P0.y + dy0 * scale)
        P2 = Point2D(P3.x - dx3 * scale, P3.y - dy3 * scale)

        result = []
        for i in range(before_idx + 1):
            result.append(points[i])

        num_samples = max(6, int(dist / 50.0))
        for i in range(1, num_samples):
            t = i / num_samples
            u = 1.0 - t
            w0, w1, w2, w3 = u*u*u, 3.0*u*u*t, 3.0*u*t*t, t*t*t
            result.append(Point2D(
                w0*P0.x + w1*P1.x + w2*P2.x + w3*P3.x,
                w0*P0.y + w1*P1.y + w2*P2.y + w3*P3.y
            ))

        for i in range(after_idx, len(points)):
            result.append(points[i])

        return result


class TrajectoryManager:
    class Decision:
        def __init__(self, action: ManagerAction, reason: str, next_state: CommittedTrajectoryState):
            self.action = action
            self.reason = reason
            self.next_state = next_state

    # Every literal ever assigned to next_state.replan_reason below - kept in sync with
    # control_node.cpp's kValidReplanReasons by test_plan_c_stability.py.
    kValidReplanReasons = (
        "recovery", "hold_due_to_dropout", "persistent_invalid_clear", "first_commit",
        "intent_change", "hold_maneuver_fallback", "hold_due_to_low_deviation", "soft_update",
        "low_confidence_deviation_hold", "persistent_low_confidence_deviation",
        "topology_changed", "excessive_deviation", "blocked_by_marking",
    )

    @staticmethod
    def update(normalized_candidate: PlannedTrajectory,
               previous_state: CommittedTrajectoryState,
               current_intent: str | RouteIntent,
               consecutive_invalid_frames: list[int],
               current_frame: int,
               current_intent_seq: int = 0,
               maneuver_dropout_hold_frames: int = 5,
               replan_lateral_rms_mm: float = 800.0,
               hold_lateral_rms_mm: float = 50.0,
               min_overlap_ratio: float = 0.5,
               replan_min_confidence: float = 0.5,
               low_conf_hold_frames: int = 5,
               hold_min_remaining_s_mm: float = 500.0,
               min_path_length_mm: float = 200.0) -> Decision:
        if isinstance(current_intent, str):
            intent_lookup = {
                "follow_main": RouteIntent.FOLLOW_MAIN,
                "turn_right": RouteIntent.TURN_RIGHT,
                "turn_left": RouteIntent.TURN_LEFT,
                "lane_change_left": RouteIntent.LANE_CHANGE_LEFT,
                "lane_change_right": RouteIntent.LANE_CHANGE_RIGHT,
            }
            current_intent_enum = intent_lookup.get(current_intent, RouteIntent.FOLLOW_MAIN)
        else:
            current_intent_enum = current_intent

        next_state = CommittedTrajectoryState(
            trajectory=previous_state.trajectory,
            progress_s_mm=previous_state.progress_s_mm,
            remaining_s_mm=previous_state.remaining_s_mm,
            last_good_frame=previous_state.last_good_frame,
            dropout_hold_counter=previous_state.dropout_hold_counter,
            low_conf_hold_counter=previous_state.low_conf_hold_counter,
            replan_reason=previous_state.replan_reason,
            committed_intent=previous_state.committed_intent,
            committed_intent_seq=previous_state.committed_intent_seq,
        )
        
        is_intent_changed = False
        if previous_state.trajectory.valid:
            if (
                previous_state.committed_intent != current_intent_enum
                or previous_state.committed_intent_seq != current_intent_seq
            ):
                is_intent_changed = True

        # Case 1: Both old and new trajectories are invalid
        if not previous_state.trajectory.valid and not normalized_candidate.valid:
            next_state.progress_s_mm = 0.0
            next_state.remaining_s_mm = 0.0
            next_state.trajectory = PlannedTrajectory(valid=False)
            next_state.dropout_hold_counter = 0
            next_state.low_conf_hold_counter = 0
            next_state.replan_reason = "recovery"
            next_state.committed_intent = RouteIntent.FOLLOW_MAIN
            next_state.committed_intent_seq = 0
            consecutive_invalid_frames[0] = 0
            return TrajectoryManager.Decision(ManagerAction.ENTER_RECOVERY, "no_valid_trajectory", next_state)

        # Case 2: New candidate is invalid
        if not normalized_candidate.valid:
            consecutive_invalid_frames[0] += 1
            if consecutive_invalid_frames[0] <= 5:
                next_state.dropout_hold_counter = consecutive_invalid_frames[0]
                next_state.low_conf_hold_counter = 0
                next_state.replan_reason = "hold_due_to_dropout"
                return TrajectoryManager.Decision(ManagerAction.HOLD_CURRENT, "transient_dropout_hold", next_state)
            else:
                next_state.progress_s_mm = 0.0
                next_state.remaining_s_mm = 0.0
                next_state.trajectory = PlannedTrajectory(valid=False)
                next_state.dropout_hold_counter = consecutive_invalid_frames[0]
                next_state.low_conf_hold_counter = 0
                next_state.replan_reason = "persistent_invalid_clear"
                next_state.committed_intent = RouteIntent.FOLLOW_MAIN
                next_state.committed_intent_seq = 0
                return TrajectoryManager.Decision(ManagerAction.ENTER_RECOVERY, "persistent_dropout_clear", next_state)

        # Case 3: New candidate is valid, but previous state was invalid
        if not previous_state.trajectory.valid:
            next_state.trajectory = normalized_candidate
            next_state.progress_s_mm = 0.0
            next_state.remaining_s_mm = TrajectoryManager.calculate_path_length(normalized_candidate.points)
            next_state.last_good_frame = current_frame
            next_state.dropout_hold_counter = 0
            next_state.low_conf_hold_counter = 0
            next_state.replan_reason = "first_commit"
            next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind)
            next_state.committed_intent_seq = 0 if next_state.committed_intent == RouteIntent.FOLLOW_MAIN else current_intent_seq
            consecutive_invalid_frames[0] = 0
            return TrajectoryManager.Decision(ManagerAction.COMMIT_NEW, "first_valid_trajectory", next_state)

        # Case 4: Both are valid
        consecutive_invalid_frames[0] = 0
        next_state.dropout_hold_counter = 0
        next_state.low_conf_hold_counter = 0

        if is_intent_changed:
            next_state.trajectory = normalized_candidate
            next_state.progress_s_mm = 0.0
            next_state.remaining_s_mm = TrajectoryManager.calculate_path_length(normalized_candidate.points)
            next_state.last_good_frame = current_frame
            next_state.replan_reason = "intent_change"
            next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind)
            next_state.committed_intent_seq = 0 if next_state.committed_intent == RouteIntent.FOLLOW_MAIN else current_intent_seq
            return TrajectoryManager.Decision(
                ManagerAction.COMMIT_NEW,
                f"intent_changed_to_{route_intent_name(current_intent_enum)}",
                next_state,
            )

        # Priority: mid-maneuver perception dropout to a follow_main fallback gets a
        # bounded grace period regardless of deviation metrics - checked before the
        # composite deviation policy below, not as part of it (see control_node.cpp).
        prev_is_maneuver = previous_state.trajectory.trajectory_kind in {
            TrajectoryKind.LANE_CHANGE_LEFT, TrajectoryKind.LANE_CHANGE_RIGHT,
            TrajectoryKind.TURN_LEFT, TrajectoryKind.TURN_RIGHT
        }
        candidate_is_follow_main = normalized_candidate.trajectory_kind == TrajectoryKind.FOLLOW_MAIN
        allow_maneuver_fallback = normalized_candidate.blocked_by_marking

        if (prev_is_maneuver and candidate_is_follow_main and not allow_maneuver_fallback
                and previous_state.dropout_hold_counter < maneuver_dropout_hold_frames):
            next_state.trajectory = previous_state.trajectory
            next_state.progress_s_mm = previous_state.progress_s_mm
            next_state.remaining_s_mm = previous_state.remaining_s_mm
            next_state.last_good_frame = current_frame
            next_state.dropout_hold_counter = previous_state.dropout_hold_counter + 1
            next_state.replan_reason = "hold_maneuver_fallback"
            next_state.committed_intent = previous_state.committed_intent
            next_state.committed_intent_seq = previous_state.committed_intent_seq
            return TrajectoryManager.Decision(ManagerAction.HOLD_CURRENT, "hold_maneuver_fallback", next_state)

        # Priority: a candidate flagged blocked_by_marking is a deterministic rule-based
        # decision, not a noisy perception guess - it must commit on its own topology
        # change regardless of confidence (see control_node.cpp for the geometry-derived
        # low-confidence T-junction scenario this guards against).
        if (normalized_candidate.blocked_by_marking
                and normalized_candidate.trajectory_kind != previous_state.trajectory.trajectory_kind):
            next_state.trajectory = normalized_candidate
            next_state.progress_s_mm = 0.0
            next_state.remaining_s_mm = TrajectoryManager.calculate_path_length(normalized_candidate.points)
            next_state.last_good_frame = current_frame
            next_state.replan_reason = "blocked_by_marking"
            next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind)
            next_state.committed_intent_seq = 0 if next_state.committed_intent == RouteIntent.FOLLOW_MAIN else current_intent_seq
            return TrajectoryManager.Decision(ManagerAction.COMMIT_NEW, "blocked_by_marking_replan", next_state)

        metrics = TrajectoryManager.calculate_path_deviation_metrics(
            previous_state.trajectory, normalized_candidate, min_path_length_mm)

        # A recognized maneuver->follow_main transition with its grace period already
        # exhausted should still fall through to the ordinary hold/update decision below
        # (which commits the candidate either way) rather than being treated as a fresh
        # topology-change replan.
        is_recognized_maneuver_transition = prev_is_maneuver and candidate_is_follow_main and not allow_maneuver_fallback
        replan_needed = ((metrics.topology_changed and not is_recognized_maneuver_transition)
                         or metrics.overlap_ratio < min_overlap_ratio
                         or metrics.lateral_rms_mm > replan_lateral_rms_mm)

        if replan_needed:
            if normalized_candidate.confidence < replan_min_confidence:
                if previous_state.low_conf_hold_counter < low_conf_hold_frames:
                    next_state.trajectory = previous_state.trajectory
                    next_state.progress_s_mm = previous_state.progress_s_mm
                    next_state.remaining_s_mm = previous_state.remaining_s_mm
                    next_state.last_good_frame = current_frame
                    next_state.low_conf_hold_counter = previous_state.low_conf_hold_counter + 1
                    next_state.replan_reason = "low_confidence_deviation_hold"
                    next_state.committed_intent = previous_state.committed_intent
                    next_state.committed_intent_seq = previous_state.committed_intent_seq
                    return TrajectoryManager.Decision(ManagerAction.HOLD_CURRENT, "low_confidence_deviation_hold", next_state)

                next_state.progress_s_mm = 0.0
                next_state.remaining_s_mm = 0.0
                next_state.trajectory = PlannedTrajectory(valid=False)
                next_state.dropout_hold_counter = 0
                next_state.low_conf_hold_counter = 0
                next_state.replan_reason = "persistent_low_confidence_deviation"
                next_state.committed_intent = RouteIntent.FOLLOW_MAIN
                next_state.committed_intent_seq = 0
                return TrajectoryManager.Decision(ManagerAction.ENTER_RECOVERY, "persistent_low_confidence_deviation", next_state)

            next_state.trajectory = normalized_candidate
            next_state.progress_s_mm = 0.0
            next_state.remaining_s_mm = TrajectoryManager.calculate_path_length(normalized_candidate.points)
            next_state.last_good_frame = current_frame
            next_state.replan_reason = "topology_changed" if metrics.topology_changed else "excessive_deviation"
            next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind)
            next_state.committed_intent_seq = 0 if next_state.committed_intent == RouteIntent.FOLLOW_MAIN else current_intent_seq
            reason = "topology_changed_replan" if metrics.topology_changed else "excessive_deviation_replan"
            return TrajectoryManager.Decision(ManagerAction.COMMIT_NEW, reason, next_state)

        if metrics.lateral_rms_mm < hold_lateral_rms_mm and previous_state.remaining_s_mm > hold_min_remaining_s_mm:
            same_kind = normalized_candidate.trajectory_kind == previous_state.trajectory.trajectory_kind

            if same_kind:
                next_state.trajectory = normalized_candidate
                prev_id = previous_state.trajectory.target_lane_id
                cand_id = normalized_candidate.target_lane_id
                prev_has_stable = prev_id and not prev_id.startswith("obj_")
                cand_has_stable = cand_id and not cand_id.startswith("obj_")
                if prev_has_stable and not cand_has_stable:
                    next_state.trajectory.target_lane_id = prev_id
                if not next_state.trajectory.source_lane_ids:
                    next_state.trajectory.source_lane_ids = previous_state.trajectory.source_lane_ids
                    
                next_state.progress_s_mm = previous_state.progress_s_mm
                next_state.remaining_s_mm = TrajectoryManager.calculate_path_length(normalized_candidate.points)
                next_state.committed_intent = previous_state.committed_intent
                next_state.committed_intent_seq = previous_state.committed_intent_seq
            else:
                # Any active maneuver-fallback grace period was already handled (and
                # returned) above, before the composite deviation policy ran.
                next_state.trajectory = normalized_candidate
                next_state.progress_s_mm = previous_state.progress_s_mm
                next_state.remaining_s_mm = TrajectoryManager.calculate_path_length(normalized_candidate.points)
                next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind)
                next_state.committed_intent_seq = 0 if next_state.committed_intent == RouteIntent.FOLLOW_MAIN else current_intent_seq

            next_state.last_good_frame = current_frame
            next_state.replan_reason = "hold_due_to_low_deviation"
            return TrajectoryManager.Decision(ManagerAction.HOLD_CURRENT, "deviation_below_threshold", next_state)
        else:
            next_state.trajectory = normalized_candidate
            next_state.remaining_s_mm = TrajectoryManager.calculate_path_length(normalized_candidate.points)
            next_state.last_good_frame = current_frame
            next_state.replan_reason = "soft_update"
            next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind)
            next_state.committed_intent_seq = 0 if next_state.committed_intent == RouteIntent.FOLLOW_MAIN else current_intent_seq
            return TrajectoryManager.Decision(ManagerAction.UPDATE_CURRENT, "soft_update_path", next_state)

    @staticmethod
    def calculate_path_length(pts: list[Point2D]) -> float:
        length = 0.0
        for i in range(1, len(pts)):
            length += math.sqrt((pts[i].x - pts[i-1].x)**2 + (pts[i].y - pts[i-1].y)**2)
        return length

    @staticmethod
    def calculate_path_deviation_metrics(prev_traj: PlannedTrajectory, cur_traj: PlannedTrajectory,
                                         min_path_length_mm: float) -> PathDeviationMetrics:
        m = PathDeviationMetrics()
        m.topology_changed = (prev_traj.trajectory_kind != cur_traj.trajectory_kind)

        prev_pts_raw = prev_traj.points
        cur_pts = cur_traj.points

        if len(prev_pts_raw) < 2 or len(cur_pts) < 2:
            m.overlap_ratio = 0.0
            return m

        # Align prev onto cur's arc-length origin before comparing extents - see
        # control_node.cpp's calculate_path_deviation_metrics for why (raw full-length
        # comparison flags plain topology-agreeing continuations as low overlap).
        progress_offset = TrajectoryPlanner.project_point_to_path(cur_pts[0], prev_pts_raw)
        len_prev_raw = TrajectoryManager.calculate_path_length(prev_pts_raw)
        len_prev = max(0.0, len_prev_raw - progress_offset)
        len_cur = TrajectoryManager.calculate_path_length(cur_pts)
        max_len = max(len_prev, len_cur)
        m.overlap_ratio = (min(len_prev, len_cur) / max_len) if max_len > 1e-6 else 1.0

        if len_prev < min_path_length_mm or len_cur < min_path_length_mm:
            return m

        prev_pts = TrajectoryPlanner.resample_path(prev_pts_raw, 100.0, progress_offset)

        common_size = min(len(prev_pts), len(cur_pts))
        if common_size < 2:
            return m

        lateral_sq_sum = 0.0
        heading_sq_sum = 0.0
        max_curv_delta = 0.0

        def turning_angle(a: Point2D, b: Point2D, c: Point2D) -> float:
            h1 = math.atan2(b.x - a.x, b.y - a.y)
            h2 = math.atan2(c.x - b.x, c.y - b.y)
            d = h2 - h1
            while d > math.pi:
                d -= 2.0 * math.pi
            while d < -math.pi:
                d += 2.0 * math.pi
            return d

        for i in range(common_size):
            lateral_diff_mm = math.hypot(prev_pts[i].x - cur_pts[i].x, prev_pts[i].y - cur_pts[i].y)
            lateral_sq_sum += lateral_diff_mm * lateral_diff_mm

            if i > 0:
                h_prev = math.atan2(prev_pts[i].x - prev_pts[i-1].x, prev_pts[i].y - prev_pts[i-1].y)
                h_cur = math.atan2(cur_pts[i].x - cur_pts[i-1].x, cur_pts[i].y - cur_pts[i-1].y)
                dh = h_cur - h_prev
                while dh > math.pi:
                    dh -= 2.0 * math.pi
                while dh < -math.pi:
                    dh += 2.0 * math.pi
                heading_sq_sum += dh * dh

            if i > 1:
                curv_prev = turning_angle(prev_pts[i-2], prev_pts[i-1], prev_pts[i])
                curv_cur = turning_angle(cur_pts[i-2], cur_pts[i-1], cur_pts[i])
                curv_delta = abs(curv_cur - curv_prev)
                if curv_delta > max_curv_delta:
                    max_curv_delta = curv_delta

        m.lateral_rms_mm = math.sqrt(lateral_sq_sum / common_size)
        m.heading_rms_rad = math.sqrt(heading_sq_sum / (common_size - 1)) if common_size > 1 else 0.0
        m.curvature_max_delta = max_curv_delta

        return m
