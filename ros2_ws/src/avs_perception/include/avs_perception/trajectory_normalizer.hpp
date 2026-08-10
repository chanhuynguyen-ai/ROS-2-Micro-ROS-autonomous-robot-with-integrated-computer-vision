#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "avs_perception/decision_types.hpp"

class TrajectoryNormalizer {
public:
    // Turn connectors are short (~1-2m), so the distance-ramped blend below
    // never settles - by the time it reaches the end of the connector,
    // w_cur is already well past its near-vehicle floor, letting the shape
    // wobble frame to frame. Turn kinds use this fixed weight on the
    // previous path instead, uniformly along the whole path.
    static inline double turn_blend_prev_weight = 0.75;

    // Weight the previous path keeps for FOLLOW_MAIN. Default 0.0: publish the
    // observed main-lane waypoints as-is.
    //
    // The blend below mixes raw x/y coordinates, but prev's coordinates are in
    // the PREVIOUS frame's vehicle frame and cur's are in the current one.
    // project_point_to_path only recovers how far the vehicle advanced ALONG
    // the path - a scalar - so nothing corrects the frame's rotation or lateral
    // shift. While the vehicle is steering hard those differ a lot, and at the
    // old near-vehicle weight (w_prev ~= 0.8, the stretch Pure Pursuit takes
    // its lookahead from) the stale, wrongly-rotated geometry dominated and
    // dragged the published path off the lane. Following the waypoints directly
    // makes the path a function of the current observation alone.
    //
    // Raise this if IPM noise makes the path visibly jitter; it trades that
    // jitter back for the lag described above.
    static inline double follow_main_blend_prev_weight = 0.0;

    static PlannedTrajectory normalize(const PlannedTrajectory& current_candidate,
                                       const CommittedTrajectoryState& previous_state,
                                       double continuity_heading_jump_rad = 0.35,
                                       double continuity_lateral_jump_mm = 300.0) {
        PlannedTrajectory normalized = current_candidate;

        // If the maneuver is blocked by a marking, skip blending to immediately return to the lane.
        if (current_candidate.blocked_by_marking) {
            normalized.normalization_mode = "blocked_passthrough";
            return normalized;
        }

        // If the previous committed trajectory is invalid or empty, we cannot blend.
        // Return the current candidate directly.
        if (!previous_state.trajectory.valid || previous_state.trajectory.points.empty()) {
            normalized.normalization_mode = "no_previous_passthrough";
            return normalized;
        }

        // If the current candidate is invalid or empty, return it directly.
        if (!current_candidate.valid || current_candidate.points.empty()) {
            normalized.normalization_mode = "invalid_candidate_passthrough";
            return normalized;
        }

        // Blending across a topology change (e.g. a turn transition vs. a straight
        // follow-main tail) by array index is not meaningful geometry and produces a
        // kink - passthrough the fresh candidate instead of blending unrelated shapes.
        if (current_candidate.trajectory_kind != previous_state.trajectory.trajectory_kind) {
            normalized.normalization_mode = "kind_mismatch_passthrough";
            return normalized;
        }

        const auto& prev_pts_raw = previous_state.trajectory.points;
        const auto& cur_pts = current_candidate.points;

        // Align the previous committed path onto the candidate's arc-length origin:
        // find where the vehicle (start of the fresh candidate) currently projects onto
        // the previously-committed path, then resample prev starting from that progress
        // instead of from its own index 0. Without this, index i in prev and cur refer to
        // different physical points once the vehicle has moved or curvature differs.
        double progress_offset = TrajectoryPlanner::project_point_to_path(cur_pts.front(), prev_pts_raw);
        std::vector<Point2D> prev_pts = TrajectoryPlanner::resample_path(prev_pts_raw, 100.0, progress_offset);

        if (prev_pts.size() < 2) {
            normalized.normalization_mode = "kind_mismatch_passthrough";
            return normalized;
        }

        std::vector<Point2D> blended_pts;
        size_t common_size = std::min(prev_pts.size(), cur_pts.size());
        blended_pts.reserve(std::max(prev_pts.size(), cur_pts.size()));

        // Fixed, uniform weights for the kinds where the distance ramp is a poor
        // fit; lane-change keeps the ramp it was tuned for.
        bool has_fixed_weight = true;
        double fixed_w_cur = 0.0;
        switch (current_candidate.trajectory_kind) {
            case TrajectoryKind::TURN_LEFT:
            case TrajectoryKind::TURN_RIGHT:
                // See turn_blend_prev_weight: turn connectors are too short for
                // the ramp to settle before the path ends.
                fixed_w_cur = 1.0 - turn_blend_prev_weight;
                break;
            case TrajectoryKind::FOLLOW_MAIN:
                // See follow_main_blend_prev_weight: the ramp blends across
                // vehicle frames it never reconciles, which is what pulled the
                // path off the lane during hard steering.
                fixed_w_cur = 1.0 - follow_main_blend_prev_weight;
                break;
            default:
                has_fixed_weight = false;
                break;
        }

        double C = current_candidate.confidence;
        double L_trans = 3000.0; // 3 meters transition length

        // Confidence-based bounds for new path weight
        double w_cur_max = 0.2 + 0.7 * C;
        double w_cur_min = 0.05 + 0.15 * C;

        for (size_t i = 0; i < common_size; ++i) {
            double w_cur;
            if (has_fixed_weight) {
                w_cur = fixed_w_cur;
            } else {
                double s = i * 100.0; // both paths are resampled/aligned at 100mm steps
                double alpha = std::min(1.0, s / L_trans);
                w_cur = w_cur_min + alpha * (w_cur_max - w_cur_min);
            }
            double w_prev = 1.0 - w_cur;

            Point2D pt;
            pt.x = w_prev * prev_pts[i].x + w_cur * cur_pts[i].x;
            pt.y = w_prev * prev_pts[i].y + w_cur * cur_pts[i].y;
            blended_pts.push_back(pt);
        }

        // If the fresh candidate sees farther than the previous committed path, append
        // its unblended tail (fresh perception, safe to extend into). If instead the
        // previous path was longer (this frame's lane view shrank - noise/occlusion,
        // range-dependent IPM error, etc.), do NOT append the leftover prev_pts tail:
        // it is unblended geometry from an earlier frame no longer supported by the
        // current candidate, and stitching it on produces a stale tail projecting past
        // where the lane is presently observed. Let the committed path shrink instead.
        if (cur_pts.size() > common_size) {
            for (size_t i = common_size; i < cur_pts.size(); ++i) {
                blended_pts.push_back(cur_pts[i]);
            }
        }

        // Post-blend continuity guard: a correctly-aligned, same-kind blend shouldn't
        // contain a sharp local kink. If one slipped through anyway, try a local Bezier
        // bridge (same control-point technique as plan_transition) around the seam; if
        // that still can't resolve it, invalidate rather than publish a kinked path -
        // the existing dropout/recovery policy in TrajectoryManager already handles an
        // invalid candidate correctly.
        size_t seam_index = 0;
        if (find_continuity_violation(blended_pts, continuity_lateral_jump_mm,
                                       continuity_heading_jump_rad, seam_index)) {
            std::vector<Point2D> bridged = bridge_local_discontinuity(blended_pts, seam_index);
            size_t recheck_seam = 0;
            if (bridged.size() >= 2 &&
                !find_continuity_violation(bridged, continuity_lateral_jump_mm,
                                            continuity_heading_jump_rad, recheck_seam)) {
                normalized.points = std::move(bridged);
                normalized.valid = (normalized.points.size() >= 2);
                normalized.normalization_mode = "temporal_blend_bridged";
                return normalized;
            }
            normalized.points = std::move(blended_pts);
            normalized.valid = false;
            normalized.normalization_mode = "continuity_guard_invalid";
            return normalized;
        }

        normalized.points = std::move(blended_pts);
        normalized.valid = (normalized.points.size() >= 2);
        normalized.normalization_mode = "temporal_blend";

        return normalized;
    }

private:
    // True if a local kink exists between consecutive segments of `points`: either the
    // heading changes abruptly, or the next point lands far off to the side of the line
    // extended from the previous segment. Reports the index of the second point of the
    // first violating pair via `seam_index`.
    static bool find_continuity_violation(const std::vector<Point2D>& points,
                                           double max_lateral_jump_mm,
                                           double max_heading_jump_rad,
                                           size_t& seam_index) {
        for (size_t i = 2; i < points.size(); ++i) {
            double h1 = std::atan2(points[i-1].x - points[i-2].x, points[i-1].y - points[i-2].y);
            double h2 = std::atan2(points[i].x - points[i-1].x, points[i].y - points[i-1].y);
            double dh = h2 - h1;
            while (dh > M_PI) dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;

            double dx = points[i-1].x - points[i-2].x;
            double dy = points[i-1].y - points[i-2].y;
            double seg_len = std::sqrt(dx*dx + dy*dy);
            double lateral_jump_mm = 0.0;
            if (seg_len > 1e-6) {
                double nx = -dy / seg_len;
                double ny = dx / seg_len;
                lateral_jump_mm = std::abs((points[i].x - points[i-1].x) * nx +
                                            (points[i].y - points[i-1].y) * ny);
            }

            if (std::abs(dh) > max_heading_jump_rad || lateral_jump_mm > max_lateral_jump_mm) {
                seam_index = i;
                return true;
            }
        }
        return false;
    }

    // Refits the local seam at `seam_index` using the same cubic-Bezier bridge
    // technique as plan_transition (control points at chord/3 along each side's
    // tangent), but on a single already-blended array around one seam index instead
    // of joining two separate lane arrays.
    static std::vector<Point2D> bridge_local_discontinuity(const std::vector<Point2D>& points,
                                                            size_t seam_index) {
        if (points.size() < 4 || seam_index < 1 || seam_index >= points.size()) {
            return points;
        }

        const double anchor_dist_mm = 300.0;

        size_t before_idx = seam_index - 1;
        double cum = 0.0;
        while (before_idx > 0) {
            cum += std::hypot(points[before_idx].x - points[before_idx - 1].x,
                               points[before_idx].y - points[before_idx - 1].y);
            if (cum >= anchor_dist_mm) break;
            before_idx--;
        }

        size_t after_idx = seam_index;
        cum = 0.0;
        while (after_idx + 1 < points.size()) {
            cum += std::hypot(points[after_idx + 1].x - points[after_idx].x,
                               points[after_idx + 1].y - points[after_idx].y);
            if (cum >= anchor_dist_mm) break;
            after_idx++;
        }

        if (after_idx <= before_idx + 1) {
            return points; // window too narrow to bridge meaningfully
        }

        Point2D P0 = points[before_idx];
        Point2D p_prev = points[before_idx > 0 ? before_idx - 1 : before_idx];
        Point2D P3 = points[after_idx];
        Point2D p_next = points[after_idx + 1 < points.size() ? after_idx + 1 : after_idx];

        double dx0 = P0.x - p_prev.x, dy0 = P0.y - p_prev.y;
        double len0 = std::sqrt(dx0*dx0 + dy0*dy0);
        if (len0 < 1e-3) { dx0 = 0; dy0 = 1.0; } else { dx0 /= len0; dy0 /= len0; }

        double dx3 = p_next.x - P3.x, dy3 = p_next.y - P3.y;
        double len3 = std::sqrt(dx3*dx3 + dy3*dy3);
        if (len3 < 1e-3) { dx3 = 0; dy3 = 1.0; } else { dx3 /= len3; dy3 /= len3; }

        double dist = std::hypot(P3.x - P0.x, P3.y - P0.y);
        double scale = dist / 3.0;
        Point2D P1 = { P0.x + dx0 * scale, P0.y + dy0 * scale };
        Point2D P2 = { P3.x - dx3 * scale, P3.y - dy3 * scale };

        std::vector<Point2D> result;
        result.reserve(points.size());
        for (size_t i = 0; i <= before_idx; ++i) result.push_back(points[i]);

        int num_samples = std::max(6, static_cast<int>(dist / 50.0));
        for (int i = 1; i < num_samples; ++i) {
            double t = static_cast<double>(i) / num_samples;
            double u = 1.0 - t;
            double w0 = u*u*u, w1 = 3.0*u*u*t, w2 = 3.0*u*t*t, w3 = t*t*t;
            result.push_back({ w0*P0.x + w1*P1.x + w2*P2.x + w3*P3.x,
                                w0*P0.y + w1*P1.y + w2*P2.y + w3*P3.y });
        }

        for (size_t i = after_idx; i < points.size(); ++i) result.push_back(points[i]);
        return result;
    }
};
