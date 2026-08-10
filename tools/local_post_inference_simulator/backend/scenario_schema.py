from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any

class CanvasSchema(BaseModel):
    width: int = 640
    height: int = 480

class CalibrationSchema(BaseModel):
    source: str = "config/calibration.json"

class RouteIntentSchema(BaseModel):
    intent: str = "follow_main"
    seq: int = 1

class ObjectSchema(BaseModel):
    id: str
    class_name: str
    label: int
    shape: str = "polygon"  # "polygon", "polyline", etc.
    points_px: List[List[float]]  # List of [x, y] coordinates in pixel space
    confidence: float = 1.0
    # Real inference output can carry several mask contours per object
    # ("polygons": [c1, c2, ...] in /avs/telemetry, all consumed by the IPM).
    # Scenarios captured from real video (telemetry_to_scenario.py) store ALL
    # contours here verbatim, with points_px kept as the first contour for the
    # canvas editor. When present, this field takes precedence over points_px
    # in the telemetry payload so replay matches production exactly.
    polygons_px: Optional[List[List[List[float]]]] = None

class FrameSchema(BaseModel):
    frame_id: int
    route_intent: Optional[RouteIntentSchema] = None
    objects: List[ObjectSchema]

class AssertionsSchema(BaseModel):
    expected_selected_lane: Optional[str] = None
    max_lane_switch_count: Optional[int] = None
    max_jitter_epsilon_x_mm: Optional[float] = None
    max_jitter_theta_rad: Optional[float] = None
    expected_blocked_state: Optional[bool] = None
    expected_trajectory_kind: Optional[str] = None
    max_replan_count: Optional[int] = None
    max_invalid_frame_count: Optional[int] = None
    expected_control_source: Optional[str] = None

class ScenarioSchema(BaseModel):
    name: str
    canvas: CanvasSchema = Field(default_factory=CanvasSchema)
    calibration: CalibrationSchema = Field(default_factory=CalibrationSchema)
    route_intent: RouteIntentSchema = Field(default_factory=RouteIntentSchema)
    frames: List[FrameSchema]
    assertions: Optional[AssertionsSchema] = None

def evaluate_assertions(scenario: ScenarioSchema, report: Dict[str, Any]) -> tuple[bool, Dict[str, Any]]:
    assertions_results = {}
    all_pass = True

    if not scenario.assertions:
        return True, {}

    metrics = report.get("metrics", {})
    frames = report.get("frames", [])
    
    last_frame = frames[-1] if frames else {}
    last_outputs = last_frame.get("outputs", {}) or {}
    last_lane_state = last_outputs.get("lane_state") or {}
    last_control_error = last_outputs.get("control_error") or {}

    # Helper to check assertion
    def check_assert(name, actual, expected, compare_fn, is_max=False):
        nonlocal all_pass
        if expected is not None:
            passed = compare_fn(actual, expected)
            res = {"expected": expected, "actual": actual, "passed": passed} if not is_max else {"expected_max": expected, "actual": actual, "passed": passed}
            assertions_results[name] = res
            if not passed:
                all_pass = False

    # 1. expected_selected_lane
    check_assert(
        "expected_selected_lane",
        last_lane_state.get("selected_lane_id"),
        scenario.assertions.expected_selected_lane,
        lambda a, e: a == e
    )

    # 2. max_lane_switch_count
    check_assert(
        "max_lane_switch_count",
        metrics.get("selected_lane_switch_count", 0),
        scenario.assertions.max_lane_switch_count,
        lambda a, e: a <= e,
        is_max=True
    )

    # 3. max_jitter_epsilon_x_mm
    check_assert(
        "max_jitter_epsilon_x_mm",
        metrics.get("jitter_epsilon_x_mm", 0.0),
        scenario.assertions.max_jitter_epsilon_x_mm,
        lambda a, e: a <= e,
        is_max=True
    )

    # 4. max_jitter_theta_rad
    check_assert(
        "max_jitter_theta_rad",
        metrics.get("jitter_theta_rad", 0.0),
        scenario.assertions.max_jitter_theta_rad,
        lambda a, e: a <= e,
        is_max=True
    )

    # 5. expected_blocked_state
    check_assert(
        "expected_blocked_state",
        last_lane_state.get("blocked_by_marking"),
        scenario.assertions.expected_blocked_state,
        lambda a, e: a == e
    )

    # 6. expected_trajectory_kind
    check_assert(
        "expected_trajectory_kind",
        last_lane_state.get("trajectory_kind"),
        scenario.assertions.expected_trajectory_kind,
        lambda a, e: a == e
    )

    # 7. max_replan_count
    check_assert(
        "max_replan_count",
        metrics.get("replan_count", 0),
        scenario.assertions.max_replan_count,
        lambda a, e: a <= e,
        is_max=True
    )

    # 8. max_invalid_frame_count
    check_assert(
        "max_invalid_frame_count",
        metrics.get("invalid_frame_count", 0),
        scenario.assertions.max_invalid_frame_count,
        lambda a, e: a <= e,
        is_max=True
    )

    # 9. expected_control_source
    check_assert(
        "expected_control_source",
        last_lane_state.get("control_source"),
        scenario.assertions.expected_control_source,
        lambda a, e: a == e
    )

    return all_pass, assertions_results
