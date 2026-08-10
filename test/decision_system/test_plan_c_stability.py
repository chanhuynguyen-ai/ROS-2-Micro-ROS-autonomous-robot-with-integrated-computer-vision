import math
import re
from pathlib import Path

from decision_harness import (
    Point2D,
    TrajectoryKind,
    RouteIntent,
    PlannedTrajectory,
    CommittedTrajectoryState,
    TrajectoryPlanner,
    TrajectoryNormalizer,
    TrajectoryManager,
    ManagerAction,
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


def extract_function_body(source: str, signature_marker: str) -> str:
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


def straight_line(start_y: float, count: int, step_mm: float = 100.0, x: float = 0.0) -> list[Point2D]:
    return [Point2D(x, start_y + i * step_mm) for i in range(count)]


def make_trajectory(points: list[Point2D], kind: TrajectoryKind = TrajectoryKind.FOLLOW_MAIN,
                     confidence: float = 0.9, target_lane_id: str = "main_lane") -> PlannedTrajectory:
    return PlannedTrajectory(
        points=points,
        target_lane_id=target_lane_id,
        trajectory_kind=kind,
        confidence=confidence,
        valid=True,
    )


# ---------------------------------------------------------------------------
# C1: progress estimation via arc-length projection.
# ---------------------------------------------------------------------------
def test_progress_estimation_on_straight():
    # Committed path passes through y = -1200 .. 3800 in vehicle frame - i.e. ego
    # (at the vehicle-frame origin) has already advanced 1200mm along it.
    committed = straight_line(-1200.0, 51)
    ego_query = Point2D(0.0, 0.0)
    s = TrajectoryPlanner.project_point_to_path(ego_query, committed)
    assert abs(s - 1200.0) <= 100.0


# ---------------------------------------------------------------------------
# C2: blend must align previous/candidate by arc-length, not raw array index.
# Uses a mildly curved path (x = k*y^2) so that blending mismatched arc-length
# positions (the old index-based bug) produces a visibly off-curve point,
# while correct s-alignment keeps every blended point on the true curve.
# ---------------------------------------------------------------------------
def test_blend_aligned_not_by_index():
    k = 0.00005

    def curve_x(y: float) -> float:
        return k * y * y

    prev_pts = [Point2D(curve_x(i * 100.0), i * 100.0) for i in range(30)]  # y: 0..2900
    cur_pts = [Point2D(curve_x(500.0 + i * 100.0), 500.0 + i * 100.0) for i in range(20)]  # y: 500..2400

    prev_state = CommittedTrajectoryState(
        trajectory=make_trajectory(prev_pts, confidence=0.9),
    )
    candidate = make_trajectory(cur_pts, confidence=0.3)

    normalized = TrajectoryNormalizer.normalize(candidate, prev_state)
    assert normalized.normalization_mode in ("temporal_blend", "temporal_blend_bridged")

    for pt in normalized.points[: min(len(prev_pts), len(cur_pts))]:
        expected_x = curve_x(pt.y)
        assert abs(pt.x - expected_x) < 10.0


# ---------------------------------------------------------------------------
# Small frame-to-frame lateral jitter must be damped by the blend, not passed
# straight through, and must not snap discontinuously.
# ---------------------------------------------------------------------------
def test_small_jitter_smoothed():
    n = 20
    prev_pts = straight_line(0.0, n, x=0.0)
    jitter = [30.0 if i % 2 == 0 else -25.0 for i in range(n)]
    cur_pts = [Point2D(jitter[i], i * 100.0) for i in range(n)]

    prev_state = CommittedTrajectoryState(trajectory=make_trajectory(prev_pts, confidence=0.9))
    candidate = make_trajectory(cur_pts, confidence=0.9)

    normalized = TrajectoryNormalizer.normalize(candidate, prev_state)
    common = min(len(prev_pts), len(cur_pts))
    for i in range(common):
        assert abs(normalized.points[i].x) < abs(jitter[i])


# ---------------------------------------------------------------------------
# A topology change (different trajectory_kind) must never blindly append the
# previous path's tail - it must passthrough the fresh candidate untouched.
# ---------------------------------------------------------------------------
def test_no_blind_tail_append_on_topology_change():
    prev_pts = straight_line(0.0, 40, x=0.0)  # long FOLLOW_MAIN history
    cur_pts = [Point2D(50.0 * i, i * 150.0) for i in range(10)]  # short TURN_RIGHT candidate

    prev_state = CommittedTrajectoryState(
        trajectory=make_trajectory(prev_pts, kind=TrajectoryKind.FOLLOW_MAIN),
    )
    candidate = make_trajectory(cur_pts, kind=TrajectoryKind.TURN_RIGHT, confidence=0.9)

    normalized = TrajectoryNormalizer.normalize(candidate, prev_state)
    assert normalized.normalization_mode == "kind_mismatch_passthrough"
    # Passthrough must be exactly the candidate - none of the previous path's
    # (much longer) tail may be appended onto it.
    assert normalized.points == cur_pts
    assert len(normalized.points) == len(cur_pts)


# ---------------------------------------------------------------------------
# Blend weighting must scale with candidate confidence: a low-confidence
# candidate should pull the blended path noticeably less toward itself than
# a high-confidence one, for the same raw deviation.
# ---------------------------------------------------------------------------
def test_blend_respects_confidence():
    prev_pts = straight_line(0.0, 15, x=0.0)
    cur_pts = straight_line(0.0, 15, x=100.0)

    prev_state = CommittedTrajectoryState(trajectory=make_trajectory(prev_pts, confidence=0.9))

    low_conf = TrajectoryNormalizer.normalize(make_trajectory(cur_pts, confidence=0.1), prev_state)
    high_conf = TrajectoryNormalizer.normalize(make_trajectory(cur_pts, confidence=0.9), prev_state)

    assert low_conf.points[0].x < high_conf.points[0].x


# ---------------------------------------------------------------------------
# A single outlier point that creates a hard local kink must be bridged back
# into a smooth seam by the post-blend continuity guard.
# ---------------------------------------------------------------------------
def test_post_blend_continuity_guard_bridges_successfully():
    # A genuine corner (clean straight segment, then a sharp ~40deg bend into
    # another clean straight segment) - the realistic case this guard exists
    # for, with enough clean geometry on both sides of the seam to refit.
    n = 30
    prev_pts = straight_line(0.0, n, x=0.0)
    cur_pts = straight_line(0.0, 16, x=0.0)
    angle = 0.7
    for i in range(16, n):
        d = (i - 15) * 100.0
        cur_pts.append(Point2D(d * math.sin(angle), 1500.0 + d * math.cos(angle)))

    prev_state = CommittedTrajectoryState(trajectory=make_trajectory(prev_pts, confidence=0.9))
    candidate = make_trajectory(cur_pts, confidence=0.9)

    normalized = TrajectoryNormalizer.normalize(candidate, prev_state)
    assert normalized.normalization_mode == "temporal_blend_bridged"
    assert normalized.valid

    _, has_violation = TrajectoryNormalizer._find_continuity_violation(normalized.points, 300.0, 0.35)
    assert not has_violation


def test_post_blend_continuity_guard_invalid_when_unbridgeable():
    # A single-point outlier spike in the middle of an otherwise straight path:
    # the violation is detected exactly at the spike, so the seam anchor itself
    # sits on the spike - there isn't enough clean geometry to refit around it,
    # and the guard must fall back to invalid rather than silently keep the kink.
    n = 30
    prev_pts = straight_line(0.0, n, x=0.0)
    cur_pts = straight_line(0.0, n, x=0.0)
    cur_pts[15] = Point2D(800.0, cur_pts[15].y)

    prev_state = CommittedTrajectoryState(trajectory=make_trajectory(prev_pts, confidence=0.9))
    candidate = make_trajectory(cur_pts, confidence=0.9)

    normalized = TrajectoryNormalizer.normalize(candidate, prev_state)
    assert normalized.normalization_mode == "continuity_guard_invalid"
    assert not normalized.valid


# ---------------------------------------------------------------------------
# C3: composite deviation metrics must separate lateral vs heading deviation -
# a zigzag candidate close to the previous path's lateral position but with
# strongly alternating heading must register a large heading_rms even though
# lateral_rms stays small.
# ---------------------------------------------------------------------------
def test_deviation_metrics_composite():
    n = 10
    prev_pts = straight_line(0.0, n, x=0.0)
    cur_pts = [Point2D(40.0 if i % 2 == 0 else -40.0, i * 100.0) for i in range(n)]

    prev_traj = make_trajectory(prev_pts)
    cur_traj = make_trajectory(cur_pts)

    metrics = TrajectoryManager.calculate_path_deviation_metrics(prev_traj, cur_traj, 200.0)
    assert metrics.lateral_rms_mm < 60.0
    assert metrics.heading_rms_rad > 0.5


# ---------------------------------------------------------------------------
# Large deviation with a low-confidence candidate must not be committed - it
# should hold on the previous trajectory, then fall into recovery once the
# low-confidence grace window is exhausted.
# ---------------------------------------------------------------------------
def test_low_confidence_high_deviation_holds():
    prev_pts = straight_line(0.0, 20, x=0.0)
    cur_pts = straight_line(0.0, 20, x=900.0)  # 900mm > default replan_lateral_rms_mm (800)

    state = CommittedTrajectoryState(
        trajectory=make_trajectory(prev_pts),
        remaining_s_mm=TrajectoryManager.calculate_path_length(prev_pts),
        committed_intent=RouteIntent.FOLLOW_MAIN,
    )
    candidate = make_trajectory(cur_pts, confidence=0.1)
    invalid_counter = [0]

    for expected_counter in range(1, 6):
        decision = TrajectoryManager.update(
            candidate, state, RouteIntent.FOLLOW_MAIN, invalid_counter, expected_counter,
        )
        assert decision.action == ManagerAction.HOLD_CURRENT
        assert decision.reason == "low_confidence_deviation_hold"
        assert decision.next_state.low_conf_hold_counter == expected_counter
        state = decision.next_state

    final = TrajectoryManager.update(candidate, state, RouteIntent.FOLLOW_MAIN, invalid_counter, 6)
    assert final.action == ManagerAction.ENTER_RECOVERY
    assert final.reason == "persistent_low_confidence_deviation"


# ---------------------------------------------------------------------------
# A genuine topology change (not the maneuver-fallback special case) with a
# high-confidence candidate must commit immediately with a clear reason.
# ---------------------------------------------------------------------------
def test_high_confidence_topology_change_commits():
    prev_pts = straight_line(0.0, 20, x=0.0)
    cur_pts = [Point2D(30.0 * i, i * 150.0) for i in range(15)]

    # committed_intent/seq already match the current intent, so this exercises the
    # deviation-metrics topology_changed path specifically, not the (earlier
    # checked, higher priority) is_intent_changed path.
    state = CommittedTrajectoryState(
        trajectory=make_trajectory(prev_pts, kind=TrajectoryKind.FOLLOW_MAIN),
        remaining_s_mm=TrajectoryManager.calculate_path_length(prev_pts),
        committed_intent=RouteIntent.TURN_RIGHT,
        committed_intent_seq=1,
    )
    candidate = make_trajectory(cur_pts, kind=TrajectoryKind.TURN_RIGHT, confidence=0.9)
    invalid_counter = [0]

    decision = TrajectoryManager.update(
        candidate, state, RouteIntent.TURN_RIGHT, invalid_counter, 1, current_intent_seq=1,
    )
    assert decision.action == ManagerAction.COMMIT_NEW
    assert decision.reason == "topology_changed_replan"
    assert decision.next_state.replan_reason == "topology_changed"


# ---------------------------------------------------------------------------
# When the committed path is nearly exhausted (remaining_s_mm below the hold
# floor), the manager must not hold onto it even if deviation is tiny - it
# should accept the fresh candidate instead of freezing on a path that's
# about to run out.
# ---------------------------------------------------------------------------
def test_hold_window_shrinks_with_remaining_distance():
    prev_pts = straight_line(0.0, 10, x=0.0)
    cur_pts = straight_line(0.0, 10, x=5.0)  # negligible deviation

    state = CommittedTrajectoryState(
        trajectory=make_trajectory(prev_pts),
        remaining_s_mm=300.0,  # below default hold_min_remaining_s_mm (500)
        committed_intent=RouteIntent.FOLLOW_MAIN,
    )
    candidate = make_trajectory(cur_pts, confidence=0.9)
    invalid_counter = [0]

    decision = TrajectoryManager.update(candidate, state, RouteIntent.FOLLOW_MAIN, invalid_counter, 1)
    assert decision.action != ManagerAction.HOLD_CURRENT
    assert decision.reason == "soft_update_path"


# ---------------------------------------------------------------------------
# Regression (pre-existing Phase 6 behavior): sustained perception dropout
# past the hold window must clear into recovery.
# ---------------------------------------------------------------------------
def test_dropout_hold_then_recovery():
    prev_pts = straight_line(0.0, 10, x=0.0)
    state = CommittedTrajectoryState(trajectory=make_trajectory(prev_pts))
    invalid_candidate = PlannedTrajectory(valid=False)
    invalid_counter = [0]

    for frame in range(1, 6):
        decision = TrajectoryManager.update(
            invalid_candidate, state, RouteIntent.FOLLOW_MAIN, invalid_counter, frame,
        )
        assert decision.action == ManagerAction.HOLD_CURRENT
        state = decision.next_state

    final = TrajectoryManager.update(invalid_candidate, state, RouteIntent.FOLLOW_MAIN, invalid_counter, 6)
    assert final.action == ManagerAction.ENTER_RECOVERY
    assert final.reason == "persistent_dropout_clear"


# ---------------------------------------------------------------------------
# Small frame-to-frame noise on an otherwise stable path must not trigger a
# replan storm - only the very first frame should COMMIT_NEW.
# ---------------------------------------------------------------------------
def test_no_replan_storm_on_noise():
    n = 15
    base_pts = straight_line(0.0, n, x=0.0)

    state = CommittedTrajectoryState(trajectory=PlannedTrajectory(valid=False))
    invalid_counter = [0]
    commit_count = 0

    for frame in range(1, 31):
        noisy_pts = [Point2D(base_pts[i].x + ((-1) ** (i + frame)) * 20.0, base_pts[i].y) for i in range(n)]
        candidate = make_trajectory(noisy_pts, confidence=0.9)
        decision = TrajectoryManager.update(
            candidate, state, RouteIntent.FOLLOW_MAIN, invalid_counter, frame,
        )
        if decision.action == ManagerAction.COMMIT_NEW:
            commit_count += 1
        state = decision.next_state

    assert commit_count == 1


# ---------------------------------------------------------------------------
# Enum-listed replan reasons: every literal ever assigned to
# next_state.replan_reason inside TrajectoryManager::update (C++) must appear
# in kValidReplanReasons, and vice versa - so a new reason string can't sneak
# in unlisted, and the harness's mirrored list can't silently drift.
# ---------------------------------------------------------------------------
def test_replan_reason_enum_matches_control_node():
    source = read_control_node()
    body = extract_function_body(source, "static Decision update(const PlannedTrajectory& normalized_candidate,")

    literals = set(re.findall(r'replan_reason\s*=\s*"([^"]+)"', body))
    for a, b in re.findall(r'replan_reason\s*=\s*metrics\.topology_changed\s*\?\s*"([^"]+)"\s*:\s*"([^"]+)"', body):
        literals.add(a)
        literals.add(b)

    valid = set(TrajectoryManager.kValidReplanReasons)
    assert literals, "expected to find at least one replan_reason literal in control_node.cpp"
    assert literals <= valid, f"unlisted replan_reason literal(s) in control_node.cpp: {literals - valid}"
    assert valid <= literals, f"stale kValidReplanReasons entries not used in control_node.cpp: {valid - literals}"


def test_replan_reason_enum_matches_harness():
    text = (Path(__file__).resolve().parent / "decision_harness.py").read_text()

    literals = set(re.findall(r'replan_reason\s*=\s*"([^"]+)"', text))
    for b in re.findall(r'replan_reason\s*=\s*"[^"]+"\s+if\s+.+?\s+else\s+"([^"]+)"', text):
        literals.add(b)

    valid = set(TrajectoryManager.kValidReplanReasons)
    assert literals, "expected to find at least one replan_reason literal in decision_harness.py"
    assert literals <= valid, f"unlisted replan_reason literal(s) in decision_harness.py: {literals - valid}"
    assert valid <= literals, f"stale kValidReplanReasons entries not used in decision_harness.py: {valid - literals}"
