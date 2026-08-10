import json
import os

from decision_harness import (
    LABEL_OTHER_LANE,
    RouteIntent,
    TrajectoryKind,
    trajectory_kind_name,
    route_intent_name,
    PathObservationBuilder,
    TrajectoryPlanner,
    TrajectoryNormalizer,
    CommittedTrajectoryState,
    PlannedTrajectory,
    Point2D,
    TrajectoryManager,
    ManagerAction,
)

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")


def load_fixture(name: str) -> list[dict]:
    path = os.path.join(FIXTURES_DIR, name)
    with open(path, "r") as f:
        return json.load(f)


def _turn_longitudinal_offset(obs, is_right: bool) -> float:
    turn_lane = TrajectoryPlanner.select_turn_lane_obs(obs, is_turn_right=is_right, is_t_junction=False)
    if turn_lane is None:
        return float("inf")
    raw = turn_lane.raw_obj or {}
    if "longitudinal_offset_mm" in raw:
        return float(raw["longitudinal_offset_mm"])
    if "lookahead_d_mm" in raw:
        return float(raw["lookahead_d_mm"])
    if turn_lane.points:
        return min(point.y for point in turn_lane.points)
    return float("inf")


def _commit_follow_main_baseline(telemetry: dict, frame_idx: int = 1):
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()
    last_main_id = [""]
    planned = TrajectoryPlanner.plan_follow_main(obs, prev_state, last_main_id)
    normalized = TrajectoryNormalizer.normalize(planned, prev_state)
    decision = TrajectoryManager.update(
        normalized,
        prev_state,
        RouteIntent.FOLLOW_MAIN,
        [0],
        frame_idx,
        current_intent_seq=0,
        maneuver_dropout_hold_frames=5,
    )
    return decision.next_state, last_main_id


def _replay_plan_a_sequence(
    frames: list[dict],
    *,
    current_intent: RouteIntent,
    current_intent_seq: int = 1,
    initial_state: CommittedTrajectoryState | None = None,
    initial_last_main_id: list[str] | None = None,
    turn_proximity_mm: float = 500.0,
    maneuver_dropout_hold_frames: int = 5,
    intent_abort_frames: int = 30,
):
    state = initial_state or CommittedTrajectoryState()
    last_main_id = list(initial_last_main_id or [""])
    consecutive_invalid = [0]
    maneuver_dropout_counter = 0
    blocked_intent_counter = 0
    maneuver_target_seen = False
    target_seen_streak = 0
    intent_arm_seen_frames = 5  # mirrors control_node kIntentArmSeenFrames
    history = []

    for frame_idx, telemetry in enumerate(frames, start=1):
        obs = PathObservationBuilder.build(telemetry)
        candidate_last_main_id = list(last_main_id)
        follow_main_last_main_id = list(last_main_id)

        follow_main_candidate = TrajectoryPlanner.plan_follow_main(obs, state, follow_main_last_main_id)
        planned_candidate = TrajectoryPlanner.plan_candidate_for_intent(
            obs,
            current_intent,
            state,
            is_t=False,
            t_junction_pending=False,
            last_main_id=candidate_last_main_id,
        )
        candidate_kind = trajectory_kind_name(planned_candidate.trajectory_kind)

        maneuver_pending = current_intent != RouteIntent.FOLLOW_MAIN
        committed_maneuver_active = state.committed_intent != RouteIntent.FOLLOW_MAIN
        blocked_maneuver = False
        should_use_follow_main_fallback = False
        force_follow_main_commit = False
        hold_reason = ""

        if current_intent in {RouteIntent.TURN_RIGHT, RouteIntent.TURN_LEFT}:
            # A committed maneuver counts as armed regardless of the sighting
            # streak (mirrors control_node).
            if state.committed_intent == current_intent:
                maneuver_target_seen = True
            is_right = current_intent == RouteIntent.TURN_RIGHT
            turn_offset = _turn_longitudinal_offset(obs, is_right=is_right)
            if turn_offset == float("inf"):
                target_seen_streak = 0
                # Mirrors update_lane_state: dropout only counts toward the abort
                # once the turn-lane has been seen stably (intent_arm_seen_frames
                # consecutive frames) for this intent; a pending intent whose
                # target never appeared, or only flickered, waits indefinitely.
                if maneuver_target_seen:
                    maneuver_dropout_counter += 1
            else:
                maneuver_dropout_counter = 0
                target_seen_streak += 1
                if target_seen_streak >= intent_arm_seen_frames:
                    maneuver_target_seen = True
            if maneuver_pending and not committed_maneuver_active and turn_offset >= turn_proximity_mm:
                should_use_follow_main_fallback = True
                hold_reason = "turn_not_in_range"
            if maneuver_pending and committed_maneuver_active and maneuver_dropout_counter > maneuver_dropout_hold_frames:
                should_use_follow_main_fallback = True
                force_follow_main_commit = True
                hold_reason = "maneuver_dropout_hold_exceeded"
            if maneuver_dropout_counter > intent_abort_frames:
                # Mirrors update_lane_state: gated on current_intent (pending intent),
                # not on the instantaneous committed trajectory kind, so this keeps
                # advancing even while already follow-main-fallback'd (hold exceeded).
                current_intent = RouteIntent.FOLLOW_MAIN
                current_intent_seq = 0
                maneuver_dropout_counter = 0
                blocked_intent_counter = 0
                maneuver_target_seen = False
                target_seen_streak = 0
                should_use_follow_main_fallback = True
                force_follow_main_commit = True
                hold_reason = "intent_aborted_dropout"
        elif current_intent in {RouteIntent.LANE_CHANGE_LEFT, RouteIntent.LANE_CHANGE_RIGHT}:
            if state.committed_intent == current_intent:
                maneuver_target_seen = True
            is_left = current_intent == RouteIntent.LANE_CHANGE_LEFT
            main_lane = TrajectoryPlanner.select_main_current(obs, last_main_id[0] if last_main_id else "")
            target_other = TrajectoryPlanner.select_other_lane_obs(obs, main_lane, is_left)
            if target_other is None:
                target_seen_streak = 0
                if maneuver_target_seen:
                    maneuver_dropout_counter += 1
            else:
                maneuver_dropout_counter = 0
                target_seen_streak += 1
                if target_seen_streak >= intent_arm_seen_frames:
                    maneuver_target_seen = True
            if maneuver_pending and not committed_maneuver_active and target_other is None:
                # Mirrors the turn "not in range yet" hold: while no candidate other-lane
                # is visible at all, keep the manager on FOLLOW_MAIN instead of forcing a
                # COMMIT_NEW/"intent_change" replan every single frame.
                should_use_follow_main_fallback = True
                hold_reason = "lane_change_target_not_detected"
            if planned_candidate.blocked_by_marking:
                blocked_maneuver = True
                should_use_follow_main_fallback = True
                force_follow_main_commit = True
                hold_reason = "blocked_by_marking"
                blocked_intent_counter += 1
            else:
                blocked_intent_counter = 0
            if maneuver_pending and committed_maneuver_active and target_other is None and maneuver_dropout_counter > maneuver_dropout_hold_frames:
                should_use_follow_main_fallback = True
                force_follow_main_commit = True
                hold_reason = "maneuver_dropout_hold_exceeded"
            if maneuver_dropout_counter > intent_abort_frames:
                current_intent = RouteIntent.FOLLOW_MAIN
                current_intent_seq = 0
                maneuver_dropout_counter = 0
                blocked_intent_counter = 0
                maneuver_target_seen = False
                target_seen_streak = 0
                should_use_follow_main_fallback = True
                force_follow_main_commit = True
                hold_reason = "intent_aborted_dropout"
        else:
            maneuver_dropout_counter = 0
            blocked_intent_counter = 0
            maneuver_target_seen = False
            target_seen_streak = 0

        if blocked_intent_counter > intent_abort_frames:
            current_intent = RouteIntent.FOLLOW_MAIN
            current_intent_seq = 0
            blocked_intent_counter = 0
            maneuver_dropout_counter = 0
            maneuver_target_seen = False
            target_seen_streak = 0
            should_use_follow_main_fallback = True
            force_follow_main_commit = True
            hold_reason = "intent_aborted_blocked"

        if should_use_follow_main_fallback:
            planned_for_manager = follow_main_candidate
            last_main_id = follow_main_last_main_id
        else:
            planned_for_manager = planned_candidate
            last_main_id = candidate_last_main_id

        normalized_candidate = TrajectoryNormalizer.normalize(planned_for_manager, state)
        manager_intent = current_intent
        manager_intent_seq = current_intent_seq
        if force_follow_main_commit or (should_use_follow_main_fallback and not committed_maneuver_active):
            manager_intent = RouteIntent.FOLLOW_MAIN
            manager_intent_seq = 0

        decision = TrajectoryManager.update(
            normalized_candidate,
            state,
            manager_intent,
            consecutive_invalid,
            frame_idx,
            current_intent_seq=manager_intent_seq,
            maneuver_dropout_hold_frames=maneuver_dropout_hold_frames,
        )
        state = decision.next_state

        history.append(
            {
                "frame": frame_idx,
                "candidate_trajectory_kind": candidate_kind,
                "planned_trajectory_kind": trajectory_kind_name(planned_for_manager.trajectory_kind),
                "manager_action": decision.action.name,
                "manager_reason": decision.reason,
                "hold_reason": hold_reason,
                "route_intent": route_intent_name(current_intent),
                "intent_seq": current_intent_seq,
                "trajectory_kind": trajectory_kind_name(state.trajectory.trajectory_kind),
                "replan_reason": state.replan_reason,
                "maneuver_dropout_counter": maneuver_dropout_counter,
                "committed_intent": route_intent_name(state.committed_intent),
            }
        )

    return history


# These tests verify plan_candidate_for_intent (Plan A Step 1) dispatches to
# exactly the same planner, with exactly the same result, as calling the
# planner directly. Turn/lane-change geometry correctness itself is already
# covered by test_phase11_pipeline.py and is out of scope here (Plan B).


def test_plan_candidate_for_intent_follow_main_matches_direct_call():
    telemetry = load_fixture("follow_main_straight.json")[0]
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()

    direct = TrajectoryPlanner.plan_follow_main(obs, prev_state, [""])
    dispatched = TrajectoryPlanner.plan_candidate_for_intent(
        obs, RouteIntent.FOLLOW_MAIN, prev_state, is_t=False, t_junction_pending=False,
        last_main_id=[""],
    )

    assert dispatched.trajectory_kind == direct.trajectory_kind == TrajectoryKind.FOLLOW_MAIN
    assert dispatched.valid == direct.valid
    assert dispatched.target_lane_id == direct.target_lane_id
    assert dispatched.points == direct.points


def test_plan_candidate_for_intent_turn_right_matches_direct_call():
    telemetry = load_fixture("turn_right_two_lanes.json")[0]
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()

    direct = TrajectoryPlanner.plan_turn_right(obs, prev_state, is_t=False, t_junction_pending=False,
                                                last_main_id=[""])
    dispatched = TrajectoryPlanner.plan_candidate_for_intent(
        obs, RouteIntent.TURN_RIGHT, prev_state, is_t=False, t_junction_pending=False,
        last_main_id=[""],
    )

    assert dispatched.trajectory_kind == direct.trajectory_kind
    assert dispatched.valid == direct.valid
    assert dispatched.target_lane_id == direct.target_lane_id
    assert dispatched.points == direct.points


def test_plan_candidate_for_intent_turn_left_matches_direct_call():
    telemetry = load_fixture("turn_left_two_lanes.json")[0]
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()

    direct = TrajectoryPlanner.plan_turn_left(obs, prev_state, is_t=False, t_junction_pending=False,
                                               last_main_id=[""])
    dispatched = TrajectoryPlanner.plan_candidate_for_intent(
        obs, RouteIntent.TURN_LEFT, prev_state, is_t=False, t_junction_pending=False,
        last_main_id=[""],
    )

    assert dispatched.trajectory_kind == direct.trajectory_kind
    assert dispatched.valid == direct.valid
    assert dispatched.target_lane_id == direct.target_lane_id
    assert dispatched.points == direct.points


def test_plan_candidate_for_intent_lane_change_left_matches_direct_call():
    telemetry = load_fixture("lane_change_dashed_allowed.json")[0]
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()

    direct = TrajectoryPlanner.plan_lane_change_left(obs, prev_state, [""])
    dispatched = TrajectoryPlanner.plan_candidate_for_intent(
        obs, RouteIntent.LANE_CHANGE_LEFT, prev_state, is_t=False, t_junction_pending=False,
        last_main_id=[""],
    )

    assert dispatched.trajectory_kind == direct.trajectory_kind == TrajectoryKind.LANE_CHANGE_LEFT
    assert dispatched.valid == direct.valid
    assert dispatched.target_lane_id == direct.target_lane_id == "other_lane"
    assert dispatched.points == direct.points


def test_plan_candidate_for_intent_lane_change_right_matches_direct_call():
    telemetry = load_fixture("lane_change_dashed_allowed.json")[0]
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()

    direct = TrajectoryPlanner.plan_lane_change_right(obs, prev_state, [""])
    dispatched = TrajectoryPlanner.plan_candidate_for_intent(
        obs, RouteIntent.LANE_CHANGE_RIGHT, prev_state, is_t=False, t_junction_pending=False,
        last_main_id=[""],
    )

    assert dispatched.trajectory_kind == direct.trajectory_kind
    assert dispatched.valid == direct.valid
    assert dispatched.target_lane_id == direct.target_lane_id
    assert dispatched.points == direct.points


def test_plan_candidate_for_intent_lane_change_blocked_by_solid_matches_direct_call():
    telemetry = load_fixture("lane_change_solid_blocked.json")[0]
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()

    direct = TrajectoryPlanner.plan_lane_change_left(obs, prev_state, [""])
    dispatched = TrajectoryPlanner.plan_candidate_for_intent(
        obs, RouteIntent.LANE_CHANGE_LEFT, prev_state, is_t=False, t_junction_pending=False,
        last_main_id=[""],
    )

    # Candidate is still produced every frame even though the maneuver is
    # blocked by a solid marking; the manager/state-machine layer (Plan A
    # Step 2) decides whether/how to react. Planner-level fallback here is
    # follow_main with blocked_by_marking=True.
    assert dispatched.blocked_by_marking == direct.blocked_by_marking is True
    assert dispatched.trajectory_kind == direct.trajectory_kind == TrajectoryKind.FOLLOW_MAIN


def test_plan_candidate_for_intent_legacy_and_unknown_fallback_to_follow_main():
    telemetry = load_fixture("follow_main_straight.json")[0]
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()

    for intent in (RouteIntent.LEGACY_TURN, RouteIntent.LEGACY_LANE_CHANGE):
        dispatched = TrajectoryPlanner.plan_candidate_for_intent(
            obs, intent, prev_state, is_t=False, t_junction_pending=False, last_main_id=[""],
        )
        assert dispatched.trajectory_kind == TrajectoryKind.FOLLOW_MAIN


def test_plan_candidate_for_intent_does_not_mutate_caller_last_main_id_state():
    # Guard against Step 1 regressing the real decision flow: a debug/candidate
    # call must operate on its own scratch copy of last_main_id, never the
    # live one shared with the real follow_main/turn/lane_change dispatch.
    telemetry = load_fixture("turn_right_two_lanes.json")[0]
    obs = PathObservationBuilder.build(telemetry)
    prev_state = CommittedTrajectoryState()

    live_last_main_id = ["main_lane"]
    scratch_copy = list(live_last_main_id)

    TrajectoryPlanner.plan_candidate_for_intent(
        obs, RouteIntent.TURN_RIGHT, prev_state, is_t=False, t_junction_pending=False,
        last_main_id=scratch_copy,
    )

    assert live_last_main_id == ["main_lane"]


def test_new_seq_same_intent_triggers_replan():
    prev_state = CommittedTrajectoryState(
        trajectory=PlannedTrajectory(
            points=[Point2D(0.0, 0.0), Point2D(50.0, 1000.0)],
            target_lane_id="turn_lane",
            trajectory_kind=TrajectoryKind.TURN_RIGHT,
            valid=True,
        ),
        committed_intent=RouteIntent.TURN_RIGHT,
        committed_intent_seq=1,
    )
    candidate = PlannedTrajectory(
        points=[Point2D(0.0, 0.0), Point2D(120.0, 950.0)],
        target_lane_id="turn_lane",
        trajectory_kind=TrajectoryKind.TURN_RIGHT,
        valid=True,
    )

    decision = TrajectoryManager.update(
        candidate,
        prev_state,
        RouteIntent.TURN_RIGHT,
        [0],
        10,
        current_intent_seq=2,
        maneuver_dropout_hold_frames=5,
    )

    assert decision.action == ManagerAction.COMMIT_NEW
    assert decision.reason == "intent_changed_to_turn_right"
    assert decision.next_state.replan_reason == "intent_change"
    assert decision.next_state.committed_intent == RouteIntent.TURN_RIGHT
    assert decision.next_state.committed_intent_seq == 2


def test_maneuver_fallback_holds_for_multiple_frames_before_follow_main():
    prev_state = CommittedTrajectoryState(
        trajectory=PlannedTrajectory(
            points=[Point2D(0.0, 0.0), Point2D(80.0, 1200.0)],
            target_lane_id="turn_lane",
            trajectory_kind=TrajectoryKind.TURN_RIGHT,
            valid=True,
        ),
        dropout_hold_counter=0,
        replan_reason="soft_update",
        committed_intent=RouteIntent.TURN_RIGHT,
        committed_intent_seq=3,
    )
    fallback_candidate = PlannedTrajectory(
        points=[Point2D(0.0, 0.0), Point2D(0.0, 1200.0)],
        target_lane_id="main_lane",
        trajectory_kind=TrajectoryKind.FOLLOW_MAIN,
        valid=True,
    )

    first = TrajectoryManager.update(
        fallback_candidate,
        prev_state,
        RouteIntent.TURN_RIGHT,
        [0],
        20,
        current_intent_seq=3,
        maneuver_dropout_hold_frames=3,
    )
    second = TrajectoryManager.update(
        fallback_candidate,
        first.next_state,
        RouteIntent.TURN_RIGHT,
        [0],
        21,
        current_intent_seq=3,
        maneuver_dropout_hold_frames=3,
    )
    third = TrajectoryManager.update(
        fallback_candidate,
        second.next_state,
        RouteIntent.TURN_RIGHT,
        [0],
        22,
        current_intent_seq=3,
        maneuver_dropout_hold_frames=3,
    )
    fourth = TrajectoryManager.update(
        fallback_candidate,
        third.next_state,
        RouteIntent.TURN_RIGHT,
        [0],
        23,
        current_intent_seq=3,
        maneuver_dropout_hold_frames=3,
    )

    assert first.action == ManagerAction.HOLD_CURRENT
    assert first.reason == "hold_maneuver_fallback"
    assert first.next_state.trajectory.trajectory_kind == TrajectoryKind.TURN_RIGHT
    assert first.next_state.dropout_hold_counter == 1

    assert second.action == ManagerAction.HOLD_CURRENT
    assert second.next_state.dropout_hold_counter == 2

    assert third.action == ManagerAction.HOLD_CURRENT
    assert third.next_state.dropout_hold_counter == 3

    assert fourth.next_state.trajectory.trajectory_kind == TrajectoryKind.FOLLOW_MAIN
    assert fourth.next_state.committed_intent == RouteIntent.FOLLOW_MAIN
    assert fourth.next_state.committed_intent_seq == 0


# ── Plan A Step 2 (A3): planner-level memory fallback on transient dropout ──
# These verify plan_turn_generic / plan_lane_change_generic hold the previously
# committed maneuver trajectory (decayed confidence) instead of silently
# downgrading to follow_main the instant the turn/other lane is not detected
# for a single frame. This matters because TrajectoryManager.update would
# otherwise see a large path_diff between the held turn/lane-change geometry
# and a fresh follow_main geometry and force an immediate "excessive_deviation"
# replan, discarding the in-progress maneuver on the very first dropout frame.

_TURN_RIGHT_TELEMETRY = {
    "timestamp_ms": 1000,
    "objects": [
        {
            "id": "main_lane",
            "label": 6,
            "class_name": "main-lane",
            "confidence": 0.9,
            "waypoints": [[0.0, 0.0], [0.0, 1000.0]],
        },
        {
            "id": "turn_lane_overlapping",
            "label": 20,
            "class_name": "turn-lane",
            "confidence": 0.8,
            "waypoints": [[100.0, 0.0], [100.0, 1000.0]],
        },
    ],
}

_MAIN_LANE_ONLY_TELEMETRY = {
    "timestamp_ms": 1100,
    "objects": [
        {
            "id": "main_lane",
            "label": 6,
            "class_name": "main-lane",
            "confidence": 0.9,
            "waypoints": [[0.0, 0.0], [0.0, 1000.0]],
        },
    ],
}


def test_plan_turn_right_holds_committed_trajectory_on_transient_dropout():
    obs_frame1 = PathObservationBuilder.build(_TURN_RIGHT_TELEMETRY)
    prev_state = CommittedTrajectoryState()
    committed = TrajectoryPlanner.plan_turn_right(
        obs_frame1, prev_state, is_t=False, t_junction_pending=False, last_main_id=[""],
    )
    assert committed.valid
    assert committed.trajectory_kind == TrajectoryKind.TURN_RIGHT

    committed_state = CommittedTrajectoryState()
    committed_state.trajectory = committed

    obs_frame2 = PathObservationBuilder.build(_MAIN_LANE_ONLY_TELEMETRY)
    held = TrajectoryPlanner.plan_turn_right(
        obs_frame2, committed_state, is_t=False, t_junction_pending=False, last_main_id=[""],
    )

    assert held.valid
    assert held.trajectory_kind == TrajectoryKind.TURN_RIGHT
    assert held.points == committed.points
    assert held.target_lane_id == committed.target_lane_id
    assert held.confidence == committed.confidence * 0.8


def test_plan_lane_change_left_holds_committed_trajectory_on_transient_dropout():
    telemetry_frame1 = load_fixture("lane_change_dashed_allowed.json")[0]
    obs_frame1 = PathObservationBuilder.build(telemetry_frame1)
    prev_state = CommittedTrajectoryState()
    committed = TrajectoryPlanner.plan_lane_change_left(obs_frame1, prev_state, [""])
    assert committed.trajectory_kind == TrajectoryKind.LANE_CHANGE_LEFT
    assert committed.valid

    committed_state = CommittedTrajectoryState()
    committed_state.trajectory = committed

    # Drop the "other-lane" object to simulate a transient perception dropout,
    # keep only the main-lane object.
    main_only_objects = [
        obj for obj in telemetry_frame1["objects"] if obj["label"] != LABEL_OTHER_LANE
    ]
    telemetry_frame2 = {"timestamp_ms": telemetry_frame1["timestamp_ms"] + 100, "objects": main_only_objects}
    obs_frame2 = PathObservationBuilder.build(telemetry_frame2)

    held = TrajectoryPlanner.plan_lane_change_left(obs_frame2, committed_state, [""])

    assert held.valid
    assert held.trajectory_kind == TrajectoryKind.LANE_CHANGE_LEFT
    assert held.points == committed.points
    assert held.target_lane_id == committed.target_lane_id
    assert held.confidence == committed.confidence * 0.8


def test_plan_turn_right_falls_back_to_follow_main_without_matching_memory():
    # No previously committed TURN trajectory to hold -> falls back to follow_main,
    # same as before this change (regression guard for the "no memory" branch).
    obs_frame = PathObservationBuilder.build(_MAIN_LANE_ONLY_TELEMETRY)
    prev_state = CommittedTrajectoryState()  # empty / invalid, no turn memory

    plan = TrajectoryPlanner.plan_turn_right(
        obs_frame, prev_state, is_t=False, t_junction_pending=False, last_main_id=[""],
    )
    assert plan.valid
    assert plan.trajectory_kind == TrajectoryKind.FOLLOW_MAIN
    assert plan.target_lane_id == "main_lane"


def test_pending_turn_intent_plans_turn_candidate_every_frame():
    frames = load_fixture("turn_intent_far_lane.json")
    baseline_state, last_main_id = _commit_follow_main_baseline(frames[0])

    history = _replay_plan_a_sequence(
        frames,
        current_intent=RouteIntent.TURN_RIGHT,
        current_intent_seq=1,
        initial_state=baseline_state,
        initial_last_main_id=last_main_id,
    )

    for frame in history:
        assert frame["candidate_trajectory_kind"] == "turn_right"
        assert frame["hold_reason"] == "turn_not_in_range"
        assert frame["trajectory_kind"] == "follow_main"
        assert frame["route_intent"] == "turn_right"
        assert frame["manager_action"] == "HOLD_CURRENT"


def test_pending_lane_change_intent_holds_instead_of_replanning_every_frame():
    # Regression guard: lane-change intent latched but no other-lane object ever
    # detected must settle into HOLD_CURRENT/follow_main, mirroring the turn's
    # "not in range yet" hold above. Before this fix, the manager saw
    # committed_intent(FOLLOW_MAIN) != manager_intent(LANE_CHANGE_LEFT) forever and
    # forced a COMMIT_NEW/"intent_change" replan on every single frame instead.
    main_only_frame = {
        "timestamp_ms": 1000,
        "objects": [
            {
                "id": "main_lane",
                "label": 6,
                "class_name": "main-lane",
                "confidence": 0.9,
                "waypoints": [[0.0, 0.0], [0.0, 1000.0], [0.0, 2000.0]],
            },
        ],
    }
    frames = [dict(main_only_frame, timestamp_ms=1000 + i * 100) for i in range(6)]

    history = _replay_plan_a_sequence(
        frames,
        current_intent=RouteIntent.LANE_CHANGE_LEFT,
        current_intent_seq=1,
    )

    assert history[0]["manager_action"] == "COMMIT_NEW"
    for frame in history[1:]:
        assert frame["hold_reason"] == "lane_change_target_not_detected"
        assert frame["trajectory_kind"] == "follow_main"
        assert frame["route_intent"] == "lane_change_left"
        assert frame["manager_action"] == "HOLD_CURRENT"


def test_turn_commits_when_in_range():
    frames = load_fixture("turn_intent_far_lane.json")
    commit_frame = {
        "timestamp_ms": 1300,
        "objects": [
            {
                "id": "main_lane",
                "label": 6,
                "class_name": "main-lane",
                "confidence": 0.9,
                "waypoints": [[0.0, 0.0], [0.0, 1000.0], [0.0, 2000.0], [0.0, 3000.0]],
            },
            {
                "id": "turn_lane_right",
                "label": 20,
                "class_name": "turn-lane",
                "confidence": 0.85,
                "waypoints": [],
                "longitudinal_offset_mm": 420.0,
                "lookahead_x_mm": 500.0,
                "lookahead_d_mm": 420.0,
                "lookahead_theta_rad": 0.5,
            },
        ],
    }
    baseline_state, last_main_id = _commit_follow_main_baseline(frames[0])

    history = _replay_plan_a_sequence(
        frames[:2] + [commit_frame],
        current_intent=RouteIntent.TURN_RIGHT,
        current_intent_seq=7,
        initial_state=baseline_state,
        initial_last_main_id=last_main_id,
    )

    assert history[-1]["candidate_trajectory_kind"] == "turn_right"
    assert history[-1]["manager_action"] == "COMMIT_NEW"
    assert history[-1]["trajectory_kind"] == "turn_right"
    assert history[-1]["replan_reason"] == "intent_change"


def test_turn_lane_dropout_keeps_intent_within_hold_window():
    frames = load_fixture("turn_dropout_relatch.json")
    history = _replay_plan_a_sequence(
        frames[:6],
        current_intent=RouteIntent.TURN_RIGHT,
        current_intent_seq=9,
    )

    assert history[0]["trajectory_kind"] == "turn_right"
    for frame in history[1:5]:
        assert frame["route_intent"] == "turn_right"
        assert frame["trajectory_kind"] == "turn_right"
    assert history[5]["trajectory_kind"] == "turn_right"
    assert all(frame["trajectory_kind"] != "follow_main" for frame in history[:6])


def test_turn_lane_dropout_beyond_window_falls_back_but_relatches():
    frames = load_fixture("turn_dropout_relatch.json")
    history = _replay_plan_a_sequence(
        frames,
        current_intent=RouteIntent.TURN_RIGHT,
        current_intent_seq=10,
    )

    assert history[0]["trajectory_kind"] == "turn_right"
    assert history[12]["hold_reason"] == "maneuver_dropout_hold_exceeded"
    assert history[12]["trajectory_kind"] == "follow_main"
    assert history[12]["route_intent"] == "turn_right"
    assert history[-1]["candidate_trajectory_kind"] == "turn_right"
    assert history[-1]["manager_action"] == "COMMIT_NEW"
    assert history[-1]["trajectory_kind"] == "turn_right"


_MAIN_ONLY_FRAME = {
    "timestamp_ms": 1000,
    "objects": [
        {
            "id": "main_lane",
            "label": 6,
            "class_name": "main-lane",
            "confidence": 0.9,
            "waypoints": [[0.0, 0.0], [0.0, 1000.0], [0.0, 2000.0]],
        },
    ],
}

# Turn-lane visible on the right but still far away (beyond turn_proximity_mm),
# so the intent is latched and the target counts as "seen" without committing.
_FAR_RIGHT_TURN_LANE = {
    "id": "turn_lane_far_right",
    "label": 20,
    "class_name": "turn-lane",
    "confidence": 0.9,
    "longitudinal_offset_mm": 2000.0,
    "waypoints": [[400.0, 2000.0], [900.0, 2050.0]],
}

_LEFT_OTHER_LANE = {
    "id": "other_lane_left",
    "label": 7,
    "class_name": "other-lane",
    "confidence": 0.9,
    "waypoints": [[-700.0, 0.0], [-700.0, 1000.0], [-700.0, 2000.0]],
}


def _main_frame(timestamp_ms: int, extra_objects: list[dict] = ()):  # noqa: B006
    frame = dict(_MAIN_ONLY_FRAME, timestamp_ms=timestamp_ms)
    frame["objects"] = list(_MAIN_ONLY_FRAME["objects"]) + list(extra_objects)
    return frame


def test_turn_intent_never_seen_target_stays_latched():
    # Real-robot regression (2026-07-10): the operator presses turn_left/right
    # on the dashboard several seconds before the intersection is visible. At
    # 14 FPS the old behavior expired the pending intent after
    # intent_abort_frames (~2s), so by the time the turn-lane appeared the
    # intent was gone and no replan ever happened. A pending intent whose
    # target has never been detected must now wait indefinitely under the
    # "turn_not_in_range" hold.
    frames = [_main_frame(1000 + i * 100) for i in range(40)]

    history = _replay_plan_a_sequence(
        frames,
        current_intent=RouteIntent.TURN_RIGHT,
        current_intent_seq=13,
        intent_abort_frames=20,
    )

    for frame in history:
        assert frame["route_intent"] == "turn_right"
        assert frame["trajectory_kind"] == "follow_main"
        assert frame["hold_reason"] == "turn_not_in_range"
        assert frame["maneuver_dropout_counter"] == 0


def test_lane_change_intent_never_seen_target_stays_latched():
    frames = [_main_frame(1000 + i * 100) for i in range(40)]

    history = _replay_plan_a_sequence(
        frames,
        current_intent=RouteIntent.LANE_CHANGE_LEFT,
        current_intent_seq=14,
        intent_abort_frames=20,
    )

    for frame in history:
        assert frame["route_intent"] == "lane_change_left"
        assert frame["trajectory_kind"] == "follow_main"
        assert frame["hold_reason"] == "lane_change_target_not_detected"
        assert frame["maneuver_dropout_counter"] == 0


def test_turn_intent_survives_flickering_target_detection():
    # Real-robot regression (2026-07-10, second failure mode): while the robot
    # approaches the intersection, turn-lane detection flickers frame-to-frame
    # (seen/unseen alternating). A brief glimpse must NOT arm the dropout-abort
    # clock - only a stable sighting (kIntentArmSeenFrames consecutive frames)
    # does. Alternating detection therefore never aborts the latched intent.
    frames = []
    for i in range(60):
        extra = [_FAR_RIGHT_TURN_LANE] if i % 2 == 0 else []
        frames.append(_main_frame(1000 + i * 100, extra))

    history = _replay_plan_a_sequence(
        frames,
        current_intent=RouteIntent.TURN_RIGHT,
        current_intent_seq=13,
        intent_abort_frames=20,
    )

    for frame in history:
        assert frame["route_intent"] == "turn_right"
        assert frame["trajectory_kind"] == "follow_main"
        assert frame["maneuver_dropout_counter"] == 0


def test_turn_intent_dropout_after_target_seen_clears_intent():
    # The dropout abort still protects against a target that was seen and then
    # lost: once the turn-lane has appeared for this intent, losing it for more
    # than intent_abort_frames clears the latched intent.
    seen = [_main_frame(1000 + i * 100, [_FAR_RIGHT_TURN_LANE]) for i in range(5)]
    lost = [_main_frame(1500 + i * 100) for i in range(23)]
    frames = seen + lost

    history = _replay_plan_a_sequence(
        frames,
        current_intent=RouteIntent.TURN_RIGHT,
        current_intent_seq=13,
        intent_abort_frames=20,
    )

    for frame in history[:5]:
        assert frame["route_intent"] == "turn_right"
        assert frame["hold_reason"] == "turn_not_in_range"
        assert frame["trajectory_kind"] == "follow_main"
    # Frames 5..24: target lost, counter climbing 1..20 - intent still latched.
    for frame in history[5:25]:
        assert frame["route_intent"] == "turn_right"
    # Frame 25: counter hits 21 > 20 - intent aborted.
    assert history[25]["hold_reason"] == "intent_aborted_dropout"
    assert history[25]["route_intent"] == "follow_main"
    assert history[-1]["route_intent"] == "follow_main"
    assert history[-1]["trajectory_kind"] == "follow_main"


def test_lane_change_intent_dropout_after_target_seen_clears_intent():
    seen = [_main_frame(1000 + i * 100, [_LEFT_OTHER_LANE]) for i in range(5)]
    lost = [_main_frame(1500 + i * 100) for i in range(23)]
    frames = seen + lost

    history = _replay_plan_a_sequence(
        frames,
        current_intent=RouteIntent.LANE_CHANGE_LEFT,
        current_intent_seq=14,
        intent_abort_frames=20,
    )

    for frame in history[:5]:
        assert frame["route_intent"] == "lane_change_left"
    for frame in history[5:25]:
        assert frame["route_intent"] == "lane_change_left"
    assert history[25]["hold_reason"] == "intent_aborted_dropout"
    assert history[25]["route_intent"] == "follow_main"
    assert history[-1]["route_intent"] == "follow_main"
    assert history[-1]["trajectory_kind"] == "follow_main"


def test_intent_abort_after_blocked_timeout():
    frames = load_fixture("lane_change_blocked_timeout.json")
    baseline_state, last_main_id = _commit_follow_main_baseline(frames[0])

    history = _replay_plan_a_sequence(
        frames,
        current_intent=RouteIntent.LANE_CHANGE_LEFT,
        current_intent_seq=11,
        initial_state=baseline_state,
        initial_last_main_id=last_main_id,
        intent_abort_frames=20,
    )

    assert history[0]["candidate_trajectory_kind"] == "follow_main"
    assert history[0]["hold_reason"] == "blocked_by_marking"
    assert history[0]["trajectory_kind"] == "follow_main"
    assert history[20]["hold_reason"] == "intent_aborted_blocked"
    assert history[20]["route_intent"] == "follow_main"
    assert history[-1]["trajectory_kind"] == "follow_main"


def test_single_active_trajectory_invariant_for_plan_a_sequences():
    far_frames = load_fixture("turn_intent_far_lane.json")
    dropout_frames = load_fixture("turn_dropout_relatch.json")
    blocked_frames = load_fixture("lane_change_blocked_timeout.json")

    baseline_state, last_main_id = _commit_follow_main_baseline(far_frames[0])
    histories = [
        _replay_plan_a_sequence(
            far_frames,
            current_intent=RouteIntent.TURN_RIGHT,
            current_intent_seq=12,
            initial_state=baseline_state,
            initial_last_main_id=last_main_id,
        ),
        _replay_plan_a_sequence(
            dropout_frames,
            current_intent=RouteIntent.TURN_RIGHT,
            current_intent_seq=13,
        ),
        _replay_plan_a_sequence(
            blocked_frames,
            current_intent=RouteIntent.LANE_CHANGE_LEFT,
            current_intent_seq=14,
            initial_state=baseline_state,
            initial_last_main_id=last_main_id,
        ),
    ]

    for history in histories:
        for frame in history:
            assert frame["trajectory_kind"] in {
                "follow_main",
                "turn_right",
                "turn_left",
                "lane_change_left",
                "lane_change_right",
            }
