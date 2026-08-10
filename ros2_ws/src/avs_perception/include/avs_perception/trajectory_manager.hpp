#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "avs_perception/decision_types.hpp"
#include "avs_perception/trajectory_planner.hpp"

class TrajectoryManager {
public:
    struct Decision {
        ManagerAction action;
        std::string reason;
        CommittedTrajectoryState next_state;
    };

    // Every literal ever assigned to next_state.replan_reason below - kept in sync by
    // test_plan_c_stability.py so a new reason string can't be added without updating
    // this list (and, symmetrically, a stale entry here gets caught as unused).
    static constexpr std::array<const char*, 13> kValidReplanReasons = {
        "recovery", "hold_due_to_dropout", "persistent_invalid_clear", "first_commit",
        "intent_change", "hold_maneuver_fallback", "hold_due_to_low_deviation", "soft_update",
        "low_confidence_deviation_hold", "persistent_low_confidence_deviation",
        "topology_changed", "excessive_deviation", "blocked_by_marking",
    };

    static Decision update(const PlannedTrajectory& normalized_candidate,
                           const CommittedTrajectoryState& previous_state,
                           RouteIntent current_intent,
                           uint64_t current_intent_seq,
                           int maneuver_dropout_hold_frames,
                           int& consecutive_invalid_frames,
                           uint64_t current_frame,
                           double replan_lateral_rms_mm = 800.0,
                           double hold_lateral_rms_mm = 50.0,
                           double min_overlap_ratio = 0.5,
                           double replan_min_confidence = 0.5,
                           int low_conf_hold_frames = 5,
                           double hold_min_remaining_s_mm = 500.0,
                           double min_path_length_mm = 200.0) {
        Decision d;
        d.next_state = previous_state;

        bool is_intent_changed = false;
        if (previous_state.trajectory.valid &&
            (previous_state.committed_intent != current_intent ||
             previous_state.committed_intent_seq != current_intent_seq)) {
            is_intent_changed = true;
        }

        // ── Case 1: Both old and new trajectories are invalid ──
        if (!previous_state.trajectory.valid && !normalized_candidate.valid) {
            d.action = ManagerAction::ENTER_RECOVERY;
            d.reason = "no_valid_trajectory";
            d.next_state.progress_s_mm = 0.0;
            d.next_state.remaining_s_mm = 0.0;
            d.next_state.trajectory.valid = false;
            d.next_state.trajectory.points.clear();
            d.next_state.trajectory.source_lane_ids.clear();
            d.next_state.trajectory.target_lane_id.clear();
            d.next_state.trajectory.trajectory_kind = TrajectoryKind::UNKNOWN;
            d.next_state.trajectory.confidence = 0.0;
            d.next_state.trajectory.normalization_mode = "none";
            d.next_state.dropout_hold_counter = 0;
            d.next_state.low_conf_hold_counter = 0;
            d.next_state.replan_reason = "recovery";
            d.next_state.committed_intent = RouteIntent::FOLLOW_MAIN;
            d.next_state.committed_intent_seq = 0;
            consecutive_invalid_frames = 0;
            return d;
        }

        // ── Case 2: New candidate is invalid ──
        if (!normalized_candidate.valid) {
            consecutive_invalid_frames++;
            // Single dropout-window source (Plan C C3): the transient
            // invalid-candidate hold reuses the same maneuver_dropout_hold_frames
            // budget as the maneuver-missing branch above, instead of a separate
            // hard-coded window. Default (5) keeps behavior identical.
            if (consecutive_invalid_frames <= maneuver_dropout_hold_frames) {
                // Transient dropout: HOLD the previous committed state
                d.action = ManagerAction::HOLD_CURRENT;
                d.reason = "transient_dropout_hold";
                d.next_state.dropout_hold_counter = consecutive_invalid_frames;
                d.next_state.low_conf_hold_counter = 0;
                d.next_state.replan_reason = "hold_due_to_dropout";
            } else {
                // Persistent dropout: Clear and enter recovery
                d.action = ManagerAction::ENTER_RECOVERY;
                d.reason = "persistent_dropout_clear";
                d.next_state.progress_s_mm = 0.0;
                d.next_state.remaining_s_mm = 0.0;
                d.next_state.trajectory.valid = false;
                d.next_state.trajectory.points.clear();
                d.next_state.trajectory.source_lane_ids.clear();
                d.next_state.trajectory.target_lane_id.clear();
                d.next_state.trajectory.trajectory_kind = TrajectoryKind::UNKNOWN;
                d.next_state.trajectory.confidence = 0.0;
                d.next_state.trajectory.normalization_mode = "none";
                d.next_state.dropout_hold_counter = consecutive_invalid_frames;
                d.next_state.low_conf_hold_counter = 0;
                d.next_state.replan_reason = "persistent_invalid_clear";
                d.next_state.committed_intent = RouteIntent::FOLLOW_MAIN;
                d.next_state.committed_intent_seq = 0;
            }
            return d;
        }

        // ── Case 3: New candidate is valid, but previous state was invalid ──
        if (!previous_state.trajectory.valid) {
            d.action = ManagerAction::COMMIT_NEW;
            d.reason = "first_valid_trajectory";
            d.next_state.trajectory = normalized_candidate;
            d.next_state.progress_s_mm = 0.0;
            d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            d.next_state.last_good_frame = current_frame;
            d.next_state.dropout_hold_counter = 0;
            d.next_state.low_conf_hold_counter = 0;
            d.next_state.replan_reason = "first_commit";
            d.next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind);
            d.next_state.committed_intent_seq =
                (d.next_state.committed_intent == RouteIntent::FOLLOW_MAIN) ? 0 : current_intent_seq;
            consecutive_invalid_frames = 0;
            return d;
        }

        // ── Case 4: Both are valid ──
        consecutive_invalid_frames = 0;
        d.next_state.dropout_hold_counter = 0;
        d.next_state.low_conf_hold_counter = 0;

        // Check if intent changed (e.g. follow_main -> turn)
        if (is_intent_changed) {
            d.action = ManagerAction::COMMIT_NEW;
            d.reason = std::string("intent_changed_to_") + route_intent_name(current_intent);
            d.next_state.trajectory = normalized_candidate;
            d.next_state.progress_s_mm = 0.0;
            d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            d.next_state.last_good_frame = current_frame;
            d.next_state.replan_reason = "intent_change";
            d.next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind);
            d.next_state.committed_intent_seq =
                (d.next_state.committed_intent == RouteIntent::FOLLOW_MAIN) ? 0 : current_intent_seq;
            return d;
        }

        // Priority: mid-maneuver perception dropout to a follow_main fallback gets a
        // bounded grace period regardless of deviation metrics - a turn/lane-change in
        // progress vs. a fresh follow_main candidate looks like a topology change (and
        // often a large deviation) by construction, but discarding the maneuver on the
        // very first dropout frame would defeat the whole point of holding it. This is
        // checked before the composite deviation policy below, not as part of it.
        bool prev_is_maneuver = (previous_state.trajectory.trajectory_kind == TrajectoryKind::LANE_CHANGE_LEFT ||
                                 previous_state.trajectory.trajectory_kind == TrajectoryKind::LANE_CHANGE_RIGHT ||
                                 previous_state.trajectory.trajectory_kind == TrajectoryKind::TURN_LEFT ||
                                 previous_state.trajectory.trajectory_kind == TrajectoryKind::TURN_RIGHT);
        bool candidate_is_follow_main = (normalized_candidate.trajectory_kind == TrajectoryKind::FOLLOW_MAIN);
        bool allow_maneuver_fallback = normalized_candidate.blocked_by_marking;
        bool is_recognized_maneuver_transition = prev_is_maneuver && candidate_is_follow_main && !allow_maneuver_fallback;

        if (is_recognized_maneuver_transition &&
            previous_state.dropout_hold_counter < maneuver_dropout_hold_frames) {
            d.action = ManagerAction::HOLD_CURRENT;
            d.reason = "hold_maneuver_fallback";
            d.next_state.trajectory = previous_state.trajectory;
            d.next_state.progress_s_mm = previous_state.progress_s_mm;
            d.next_state.remaining_s_mm = previous_state.remaining_s_mm;
            d.next_state.last_good_frame = current_frame;
            d.next_state.dropout_hold_counter = previous_state.dropout_hold_counter + 1;
            d.next_state.replan_reason = "hold_maneuver_fallback";
            d.next_state.committed_intent = previous_state.committed_intent;
            d.next_state.committed_intent_seq = previous_state.committed_intent_seq;
            return d;
        }

        // Priority: a candidate flagged blocked_by_marking is a deterministic rule-based
        // decision (a solid marking/rule forbids the maneuver), not a noisy perception
        // guess - it must commit on its own topology change regardless of confidence,
        // unlike the generic composite-deviation branch below. A short lane segment
        // right at a T-junction can legitimately compute a low geometry-derived
        // confidence (see plan_follow_main's len_factor/pts_factor), which would
        // otherwise trip the confidence gate and strand the vehicle on its old
        // (now-forbidden) trajectory_kind.
        if (normalized_candidate.blocked_by_marking &&
            normalized_candidate.trajectory_kind != previous_state.trajectory.trajectory_kind) {
            d.action = ManagerAction::COMMIT_NEW;
            d.reason = "blocked_by_marking_replan";
            d.next_state.trajectory = normalized_candidate;
            d.next_state.progress_s_mm = 0.0;
            d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            d.next_state.last_good_frame = current_frame;
            d.next_state.replan_reason = "blocked_by_marking";
            d.next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind);
            d.next_state.committed_intent_seq =
                (d.next_state.committed_intent == RouteIntent::FOLLOW_MAIN) ? 0 : current_intent_seq;
            return d;
        }

        // Compare new candidate and previous path to prevent jittery replan. Metrics are
        // arc-length aligned (see calculate_path_deviation_metrics), so this is meaningful
        // even across differing path lengths - unlike the old index-based path_diff.
        PathDeviationMetrics metrics = calculate_path_deviation_metrics(
            previous_state.trajectory, normalized_candidate, min_path_length_mm);

        // A recognized maneuver->follow_main transition with its grace period already
        // exhausted should still fall through to the ordinary hold/update decision below
        // (which commits the candidate either way) rather than being treated as a fresh
        // topology-change replan - that would just re-litigate a transition this dedicated
        // mechanism already decided to accept.
        bool replan_needed = (metrics.topology_changed && !is_recognized_maneuver_transition) ||
                              metrics.overlap_ratio < min_overlap_ratio ||
                              metrics.lateral_rms_mm > replan_lateral_rms_mm;

        // A follow_main candidate built from a lane actually seen this frame is
        // never worth holding a stale path over. Steering hard makes the fresh
        // candidate deviate a lot from what was committed (the vehicle frame
        // rotated) while the lane segment still in view gets short and curved,
        // which drives its geometry-derived confidence under
        // replan_min_confidence - so the low-confidence branch below would
        // freeze the old, now-off-lane path for low_conf_hold_frames right when
        // the vehicle most needs the current one. That gate is meant for noisy
        // guesses, not for a lane the camera is looking straight at.
        bool fresh_follow_main_observation =
            normalized_candidate.trajectory_kind == TrajectoryKind::FOLLOW_MAIN &&
            normalized_candidate.from_direct_observation;

        if (replan_needed) {
            if (normalized_candidate.confidence < replan_min_confidence &&
                !fresh_follow_main_observation) {
                // A big/topology deviation from a low-confidence candidate isn't trustworthy
                // enough to snap to yet - hold through it for a bounded number of frames
                // using its own dedicated low_conf_hold_counter (separate from
                // dropout_hold_counter, which belongs to transient-invalid dropout and
                // maneuver-fallback holds - sharing one counter would let those unrelated
                // hold reasons silently steal from this policy's grace period), then give up.
                if (previous_state.low_conf_hold_counter < low_conf_hold_frames) {
                    d.action = ManagerAction::HOLD_CURRENT;
                    d.reason = "low_confidence_deviation_hold";
                    d.next_state.trajectory = previous_state.trajectory;
                    d.next_state.progress_s_mm = previous_state.progress_s_mm;
                    d.next_state.remaining_s_mm = previous_state.remaining_s_mm;
                    d.next_state.last_good_frame = current_frame;
                    d.next_state.low_conf_hold_counter = previous_state.low_conf_hold_counter + 1;
                    d.next_state.replan_reason = "low_confidence_deviation_hold";
                    d.next_state.committed_intent = previous_state.committed_intent;
                    d.next_state.committed_intent_seq = previous_state.committed_intent_seq;
                    return d;
                }

                d.action = ManagerAction::ENTER_RECOVERY;
                d.reason = "persistent_low_confidence_deviation";
                d.next_state.progress_s_mm = 0.0;
                d.next_state.remaining_s_mm = 0.0;
                d.next_state.trajectory.valid = false;
                d.next_state.trajectory.points.clear();
                d.next_state.trajectory.source_lane_ids.clear();
                d.next_state.trajectory.target_lane_id.clear();
                d.next_state.trajectory.trajectory_kind = TrajectoryKind::UNKNOWN;
                d.next_state.trajectory.confidence = 0.0;
                d.next_state.trajectory.normalization_mode = "none";
                d.next_state.dropout_hold_counter = 0;
                d.next_state.low_conf_hold_counter = 0;
                d.next_state.replan_reason = "persistent_low_confidence_deviation";
                d.next_state.committed_intent = RouteIntent::FOLLOW_MAIN;
                d.next_state.committed_intent_seq = 0;
                return d;
            }

            d.action = ManagerAction::COMMIT_NEW;
            d.reason = metrics.topology_changed ? "topology_changed_replan" : "excessive_deviation_replan";
            d.next_state.trajectory = normalized_candidate;
            d.next_state.progress_s_mm = 0.0;
            d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            d.next_state.last_good_frame = current_frame;
            d.next_state.replan_reason = metrics.topology_changed ? "topology_changed" : "excessive_deviation";
            d.next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind);
            d.next_state.committed_intent_seq =
                (d.next_state.committed_intent == RouteIntent::FOLLOW_MAIN) ? 0 : current_intent_seq;
            return d;
        }

        if (metrics.lateral_rms_mm < hold_lateral_rms_mm &&
            previous_state.remaining_s_mm > hold_min_remaining_s_mm) {
            d.action = ManagerAction::HOLD_CURRENT;
            d.reason = "deviation_below_threshold";

            bool same_kind = (normalized_candidate.trajectory_kind == previous_state.trajectory.trajectory_kind);

            if (same_kind) {
                // Refresh low-deviation geometry so committed follow-main paths keep extending
                // forward, but preserve stronger ID metadata when the latest frame degraded to
                // a synthesized obj_* identifier.
                d.next_state.trajectory = normalized_candidate;

                bool previous_has_stable_id = !previous_state.trajectory.target_lane_id.empty() &&
                                              previous_state.trajectory.target_lane_id.rfind("obj_", 0) != 0;
                bool candidate_has_stable_id = !normalized_candidate.target_lane_id.empty() &&
                                               normalized_candidate.target_lane_id.rfind("obj_", 0) != 0;

                if (previous_has_stable_id && !candidate_has_stable_id) {
                    d.next_state.trajectory.target_lane_id = previous_state.trajectory.target_lane_id;
                }

                if (d.next_state.trajectory.source_lane_ids.empty()) {
                    d.next_state.trajectory.source_lane_ids = previous_state.trajectory.source_lane_ids;
                }

                d.next_state.progress_s_mm = previous_state.progress_s_mm;
                d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
                d.next_state.committed_intent = previous_state.committed_intent;
                d.next_state.committed_intent_seq = previous_state.committed_intent_seq;
            } else {
                // Any active maneuver-fallback grace period was already handled (and
                // returned) above, before the composite deviation policy ran - reaching
                // here means either this wasn't that case, or the grace period is
                // exhausted, so just commit the candidate as usual.
                d.next_state.trajectory = normalized_candidate;
                d.next_state.progress_s_mm = previous_state.progress_s_mm;
                d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
                d.next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind);
                d.next_state.committed_intent_seq =
                    (d.next_state.committed_intent == RouteIntent::FOLLOW_MAIN) ? 0 : current_intent_seq;
            }

            d.next_state.last_good_frame = current_frame;
            d.next_state.replan_reason = "hold_due_to_low_deviation";
        } else {
            d.action = ManagerAction::UPDATE_CURRENT;
            d.reason = "soft_update_path";
            d.next_state.trajectory = normalized_candidate;
            d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            d.next_state.last_good_frame = current_frame;
            d.next_state.replan_reason = "soft_update";
            d.next_state.committed_intent = route_intent_from_trajectory_kind(normalized_candidate.trajectory_kind);
            d.next_state.committed_intent_seq =
                (d.next_state.committed_intent == RouteIntent::FOLLOW_MAIN) ? 0 : current_intent_seq;
        }

        return d;
    }

private:
    static double calculate_path_length(const std::vector<Point2D>& pts) {
        double len = 0.0;
        for (size_t i = 1; i < pts.size(); ++i) {
            len += std::sqrt(std::pow(pts[i].x - pts[i-1].x, 2) + std::pow(pts[i].y - pts[i-1].y, 2));
        }
        return len;
    }

    // Aligns prev onto cur's arc-length origin (same technique as
    // TrajectoryNormalizer::normalize) before comparing, so the metrics are meaningful
    // even when the vehicle has moved along the path or the two arrays have different
    // lengths - the old index-based calculate_path_deviation assumed index i meant the
    // same physical point in both arrays, which is only true by coincidence.
    static PathDeviationMetrics calculate_path_deviation_metrics(const PlannedTrajectory& prev_traj,
                                                                  const PlannedTrajectory& cur_traj,
                                                                  double min_path_length_mm) {
        PathDeviationMetrics m;
        m.topology_changed = (prev_traj.trajectory_kind != cur_traj.trajectory_kind);

        const auto& prev_pts_raw = prev_traj.points;
        const auto& cur_pts = cur_traj.points;

        if (prev_pts_raw.size() < 2 || cur_pts.size() < 2) {
            m.overlap_ratio = 0.0;
            return m;
        }

        // Align prev onto cur's arc-length origin *before* comparing extents: prev_traj is
        // whatever was committed last frame (often planned from farther back / including
        // already-passed geometry), so comparing its raw full length against cur's fresh,
        // possibly shorter view (e.g. reduced FOV/occlusion, or approaching a real lane
        // end) would flag plain topology-agreeing continuations as low overlap.
        double progress_offset = TrajectoryPlanner::project_point_to_path(cur_pts.front(), prev_pts_raw);
        double len_prev_raw = calculate_path_length(prev_pts_raw);
        double len_prev = std::max(0.0, len_prev_raw - progress_offset);
        double len_cur = calculate_path_length(cur_pts);
        double max_len = std::max(len_prev, len_cur);
        m.overlap_ratio = (max_len > 1e-6) ? (std::min(len_prev, len_cur) / max_len) : 1.0;

        if (len_prev < min_path_length_mm || len_cur < min_path_length_mm) {
            return m;
        }

        std::vector<Point2D> prev_pts = TrajectoryPlanner::resample_path(prev_pts_raw, 100.0, progress_offset);

        size_t common_size = std::min(prev_pts.size(), cur_pts.size());
        if (common_size < 2) {
            return m;
        }

        double lateral_sq_sum = 0.0;
        double heading_sq_sum = 0.0;
        double max_curv_delta = 0.0;

        for (size_t i = 0; i < common_size; ++i) {
            double lateral_diff_mm = std::hypot(prev_pts[i].x - cur_pts[i].x, prev_pts[i].y - cur_pts[i].y);
            lateral_sq_sum += lateral_diff_mm * lateral_diff_mm;

            if (i > 0) {
                double h_prev = std::atan2(prev_pts[i].x - prev_pts[i-1].x, prev_pts[i].y - prev_pts[i-1].y);
                double h_cur = std::atan2(cur_pts[i].x - cur_pts[i-1].x, cur_pts[i].y - cur_pts[i-1].y);
                double dh = h_cur - h_prev;
                while (dh > M_PI) dh -= 2.0 * M_PI;
                while (dh < -M_PI) dh += 2.0 * M_PI;
                heading_sq_sum += dh * dh;
            }

            if (i > 1) {
                auto turning_angle = [](const Point2D& a, const Point2D& b, const Point2D& c) {
                    double h1 = std::atan2(b.x - a.x, b.y - a.y);
                    double h2 = std::atan2(c.x - b.x, c.y - b.y);
                    double d = h2 - h1;
                    while (d > M_PI) d -= 2.0 * M_PI;
                    while (d < -M_PI) d += 2.0 * M_PI;
                    return d;
                };
                double curv_prev = turning_angle(prev_pts[i-2], prev_pts[i-1], prev_pts[i]);
                double curv_cur = turning_angle(cur_pts[i-2], cur_pts[i-1], cur_pts[i]);
                double curv_delta = std::abs(curv_cur - curv_prev);
                if (curv_delta > max_curv_delta) max_curv_delta = curv_delta;
            }
        }

        m.lateral_rms_mm = std::sqrt(lateral_sq_sum / common_size);
        m.heading_rms_rad = (common_size > 1) ? std::sqrt(heading_sq_sum / (common_size - 1)) : 0.0;
        m.curvature_max_delta = max_curv_delta;

        return m;
    }
};
