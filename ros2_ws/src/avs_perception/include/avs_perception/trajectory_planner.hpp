#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "avs_perception/label_mapping.hpp"
#include "avs_perception/decision_types.hpp"

// Mirrors gtest's FRIEND_TEST from gtest_prod.h, defined locally so this
// production header (and control_node.cpp, which includes it) doesn't pick up
// a hard dependency on gtest just to let test code befriend private helpers.
#ifndef FRIEND_TEST
#define FRIEND_TEST(test_case_name, test_name) friend class test_case_name##_##test_name##_Test
#endif

using namespace avs_perception;

class TrajectoryPlanner {
public:
    // ── Turn connector shape knobs (tunable at runtime) ──────────────────
    // These two shape the Bezier connector that joins the vehicle's current
    // path to the turn lane. They are plain settable statics rather than
    // constexpr so the shape can be swept offline (tools/turn_bulge_sweep)
    // and driven live from a ROS parameter without changing call sites.
    // Process-wide and read on the planning path only - set them at startup
    // or from the same thread that calls the planner.

    // plan_transition's default Bezier handle length (dist/3, the standard
    // 1/3 rule) is tuned for lane-change's shallow heading changes. Applied
    // unscaled to a turn's much larger heading change, the handles are too
    // short to bulge the curve outward, so it cuts close to the inside corner
    // instead of swinging toward the middle of the intersection. Turn-only:
    // lane-change and the follow_main gap-bridge keep the unscaled default.
    static inline double turn_bezier_handle_scale_mult = 1.5;

    // Same, for right turns, where the comment above reads as the argument
    // AGAINST scaling: at the unscaled 1.0 the curve "cuts close to the inside
    // corner", which is precisely what a right turn wants and what 1.5 was
    // written to prevent. Lengthening the handles holds the vehicle's entry
    // heading further into the junction before the curve bends, so the arc
    // leaves the vehicle wider - the second cause of the wide right-hand arc,
    // alongside the outward belly (turn_lateral_bulge_mult_right).
    //
    // Measured over run21-25, right turns emit arcs of 609-943mm where the
    // junction fits 310-613mm, and lowering turn_latch_min_radius_mm to 400 (run25)
    // moved the emitted radius by 7mm - the width is built into the connector's
    // shape here, not clamped in by the latch.
    //
    // Live-tunable, and can go below 1.0 to tighten further.
    //
    // 1.0 here with turn_lateral_bulge_mult_right at 0.0 is not a guess: it is
    // exactly the turn connector as it stood before 360c6b0 (2026-08-04), which
    // called plan_transition(from, turn_pts, kTurnMaxHeadingDiffRad) with no
    // scaling and no bulge at all. Both widening knobs were added that same day
    // for the left turn's swing through the junction and applied to both
    // directions without the asymmetry being considered. Right turns are
    // therefore restored to the shape they had before, while left turns keep
    // the 1.5/0.355 tuning.
    // 1.25 after driving it (run28 + user review): 1.0 restored the pre-widening
    // shape and did fix the wide arc - emitted radius fell from a median of
    // 741mm to 475-588mm, inside the 310-613mm the junction actually fits, and
    // the turn released at 92.8 degrees without the run-out skip having to
    // intervene. But on the mat it now hugs the corner too closely to drive, so
    // the useful value sits between the two: more curve into the junction than
    // 1.0 gives, well short of the left turn's 1.5 swing.
    static inline double turn_bezier_handle_scale_mult_right = 1.25;

    // Depth of the turn connector's belly, as a fraction of the P0-P3 chord
    // length, pushed toward the outside of the turn (the side the entry
    // tangent leans to). Adds curve depth without lengthening the Bezier
    // handles, which fold the path back on itself past ~2.5x on their own.
    // Larger = the path holds the vehicle's heading longer and swings wider
    // through the intersection before bending; smaller = a tighter line
    // closer to the straight chord. See tools/turn_bulge_sweep/README.md for
    // the measured shape at each value and how to re-sweep.
    static inline double turn_lateral_bulge_mult = 0.355;

    // Same, for right turns, which want the opposite shape. Swinging wide is
    // right for a left turn: it crosses the intersection, so there is room to
    // hold heading and bend late. A right turn is taken from the right lane
    // around the near corner - there is no room outside it, and the outward
    // belly makes the connector leave the vehicle pointing slightly AWAY from
    // the turn. Measured on the vehicle (run24, right turns): the emitted path
    // asked for theta_rad of -0.05 to -0.12 while the intent was a right turn,
    // and the controller answered with up to 0.50 rad/s of left rotation before
    // any right rotation began - the vehicle steering out of the corner it was
    // trying to enter, then arriving short of the angle it needed.
    //
    // 0.0 removes the outward belly and leaves the plain Bezier. Negative
    // values put the belly on the far side instead, leaning the path into the
    // turn from the moment it leaves the vehicle - the "hugs the inside corner"
    // shape plan_transition's comment describes, which is what a right turn
    // actually wants. Live-tunable, so the sign can be explored on the vehicle.
    static inline double turn_lateral_bulge_mult_right = 0.0;

    static PlannedTrajectory plan_follow_main(const PathObservationFrame& obs,
                                              const CommittedTrajectoryState& prev_state,
                                              std::string& last_main_id) {
        PlannedTrajectory plan;
        plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
        
        const LaneObservation* cur_lane = select_main_current(obs, last_main_id);
        
        if (cur_lane) {
            last_main_id = cur_lane->lane_id;
            plan.from_direct_observation = true;
            plan.target_lane_id = cur_lane->lane_id;
            plan.source_lane_ids.push_back(std::to_string(cur_lane->label) + ":" + cur_lane->lane_id);
            
            const LaneObservation* ahead_lane = select_main_ahead(obs, cur_lane);
            
            std::vector<Point2D> raw_path;
            if (ahead_lane) {
                plan.source_lane_ids.push_back(std::to_string(ahead_lane->label) + ":" + ahead_lane->lane_id);
                raw_path = merge_lanes(*cur_lane, *ahead_lane);
            } else {
                raw_path = cur_lane->points;
            }

            plan.confidence = cur_lane->confidence;

            // The path IS the observed centreline. ipm_transform_node already
            // samples the lane centre at 100mm (extract_centerline_waypoints_*,
            // polynomial-fitted), so raw_path is a dense curve through the lane
            // centre and needs nothing added to it - resample_path below keeps
            // its 100mm spacing, and every observed waypoint stays on the path.
            //
            // Do NOT anchor this at the vehicle in the ordinary case. Bridging
            // runs the path through plan_transition, which DISCARDS the first
            // ~1200mm of the lane's own waypoints and substitutes Bezier samples
            // shaped by the vehicle's current heading. That makes the near field
            // - the stretch the controller actually steers on - a synthetic
            // curve that cuts across the lane centre and moves whenever the
            // vehicle is off-centre, instead of the fixed centreline it should
            // be. The vehicle's offset belongs in the control error, not in the
            // path's shape.
            //
            // The one case that still bridges is the mid-intersection gap, where
            // the lane only reappears well past the vehicle: there are no
            // waypoints near the vehicle to follow, and a path floating out
            // there collapses the manager's overlap metric against the committed
            // path (see the deviation metrics in TrajectoryManager).
            //
            // bridge_gap_to_lane, not plan_transition: the connector spans only
            // the empty stretch in front of the vehicle and every observed
            // waypoint survives. plan_transition would eat the first 1200mm of
            // the lane - or half of it when the observation is shorter than that,
            // which is the normal case for a lane just acquired coming out of a
            // junction - and hand the controller a Bezier through the near field
            // instead of the centreline.
            if (raw_path.size() >= 2 && raw_path.front().y > kBridgeMinLaneStartYMm) {
                std::vector<Point2D> bridged = bridge_gap_to_lane(raw_path);
                if (bridged.size() >= 2) {
                    raw_path = std::move(bridged);
                    plan.confidence = std::max(plan.confidence, geometry_confidence(raw_path));
                }
            }

            plan.points = resample_path(raw_path, 100.0);
            plan.valid = (plan.points.size() >= 2);
            
            if (cur_lane->has_precomputed_control) {
                plan.has_precomputed_control = true;
                plan.precomputed_epsilon_x_mm = cur_lane->precomputed_epsilon_x_mm;
                plan.precomputed_epsilon_y_mm = cur_lane->precomputed_epsilon_y_mm;
                plan.precomputed_theta_rad = cur_lane->precomputed_theta_rad;
                plan.precomputed_curvature_inv_mm = cur_lane->precomputed_curvature_inv_mm;
                plan.precomputed_lookahead_d_mm = cur_lane->precomputed_lookahead_d_mm;
                
                if (!plan.valid) {
                    plan.valid = true;
                    plan.confidence = cur_lane->confidence;
                }
            }
        } else {
            if (prev_state.trajectory.valid && prev_state.trajectory.trajectory_kind == TrajectoryKind::FOLLOW_MAIN) {
                // Main lane dropped out this frame (common exactly when the
                // vehicle is steering hard - the lane briefly leaves the BEV
                // window/blurs). Replaying prev_state's points unchanged would
                // freeze them in the OLD vehicle-frame, so as the vehicle keeps
                // moving they read as a straight line running off the actual
                // (now-curved) lane. Re-anchor at the vehicle instead, same
                // vehicle-origin bridge used for the intersection gap case
                // above, treating the held path as the best known lane target
                // rather than a fixed path to replay verbatim.
                const std::vector<Point2D> ego_stub = {{0.0, 0.0}, {0.0, 100.0}};
                std::vector<Point2D> bridged = plan_transition(ego_stub, prev_state.trajectory.points);
                plan.points = bridged.size() >= 2 ? resample_path(bridged, 100.0)
                                                   : prev_state.trajectory.points;
                plan.target_lane_id = prev_state.trajectory.target_lane_id;
                plan.source_lane_ids = prev_state.trajectory.source_lane_ids;
                // Floor the memory-hold decay: an unbounded *0.8 per frame underflows
                // to denormals (1e-323 observed live) after minutes of holding.
                plan.confidence = std::max(0.05, prev_state.trajectory.confidence * 0.8);
                plan.valid = true;
                
                plan.has_precomputed_control = prev_state.trajectory.has_precomputed_control;
                plan.precomputed_epsilon_x_mm = prev_state.trajectory.precomputed_epsilon_x_mm;
                plan.precomputed_epsilon_y_mm = prev_state.trajectory.precomputed_epsilon_y_mm;
                plan.precomputed_theta_rad = prev_state.trajectory.precomputed_theta_rad;
                plan.precomputed_curvature_inv_mm = prev_state.trajectory.precomputed_curvature_inv_mm;
                plan.precomputed_lookahead_d_mm = prev_state.trajectory.precomputed_lookahead_d_mm;
            } else {
                plan.valid = false;
                plan.confidence = 0.0;
                last_main_id = "";
            }
        }
        
        return plan;
    }

    static PlannedTrajectory plan_turn_right(const PathObservationFrame& obs, 
                                             const CommittedTrajectoryState& prev_state,
                                             bool is_t,
                                             bool t_junction_pending,
                                             std::string& last_main_id) {
        return plan_turn_generic(obs, prev_state, true, is_t, last_main_id);
    }
    
    static PlannedTrajectory plan_turn_left(const PathObservationFrame& obs, 
                                            const CommittedTrajectoryState& prev_state,
                                            bool is_t,
                                            bool t_junction_pending,
                                            std::string& last_main_id) {
        return plan_turn_generic(obs, prev_state, false, is_t, last_main_id);
    }

    static PlannedTrajectory plan_turn_generic(const PathObservationFrame& obs,
                                               const CommittedTrajectoryState& prev_state,
                                               bool is_right_turn,
                                               bool is_t,
                                               std::string& last_main_id) {
        PlannedTrajectory plan;
        plan.trajectory_kind = is_right_turn ? TrajectoryKind::TURN_RIGHT : TrajectoryKind::TURN_LEFT;

        // 1. Select the turn lane
        const LaneObservation* selected_turn = select_turn_lane_obs(obs, is_right_turn, is_t);

        // 2. Reuse main-lane selection logic for turn transitions to avoid picking wrong main lane segment
        const LaneObservation* cur_main = select_main_current(obs, last_main_id);
        if (cur_main) {
            last_main_id = cur_main->lane_id;
        }

        // 3. Preserve precomputed turn lanes when no waypoint path is available or as legacy fallback
        if (selected_turn && selected_turn->has_precomputed_control) {
            plan.points = selected_turn->points; // may be empty
            if (plan.points.empty()) {
                plan.points = {
                    { 0.0, 0.0 },
                    { selected_turn->precomputed_epsilon_x_mm * 0.5, selected_turn->precomputed_epsilon_y_mm * 0.5 },
                    { selected_turn->precomputed_epsilon_x_mm, selected_turn->precomputed_epsilon_y_mm }
                };
            }
            plan.target_lane_id = selected_turn->lane_id;
            if (cur_main) {
                plan.source_lane_ids.push_back(std::to_string(cur_main->label) + ":" + cur_main->lane_id);
            }
            plan.source_lane_ids.push_back(std::to_string(selected_turn->label) + ":" + selected_turn->lane_id);
            plan.confidence = selected_turn->confidence;
            plan.valid = true; // explicitly mark valid
            
            // Populate precomputed control fields
            plan.has_precomputed_control = true;
            plan.precomputed_epsilon_x_mm = selected_turn->precomputed_epsilon_x_mm;
            plan.precomputed_epsilon_y_mm = selected_turn->precomputed_epsilon_y_mm;
            plan.precomputed_theta_rad = selected_turn->precomputed_theta_rad;
            plan.precomputed_curvature_inv_mm = selected_turn->precomputed_curvature_inv_mm;
            plan.precomputed_lookahead_d_mm = selected_turn->precomputed_lookahead_d_mm;
            
            return plan;
        }

        if (cur_main && selected_turn) {
            // Approaching the junction the ego main lane leaves the BEV window and
            // select_main_current latches onto the main-lane segment on the far
            // side of the intersection. Transitioning out of *that* builds a path
            // running from across the junction back to the turn lane - geometry
            // the vehicle is nowhere near, and it is what makes the turn path jump
            // away just as the turn becomes due. Same remedy as plan_follow_main's
            // intersection bridge: a main lane that no longer starts at the
            // vehicle is not the ego lane, so anchor the turn at the vehicle and
            // plan straight from there into the turn lane.
            bool main_is_ego_lane = cur_main->points.size() >= 2 &&
                                    cur_main->points.front().y <= kTurnAnchorMaxLaneStartYMm;
            // The far-side main lane's absolute position is untrustworthy as an
            // anchor (it's nowhere near the vehicle), but it is still the best
            // available estimate of the true main-lane heading - anchor at the
            // vehicle while keeping that heading, instead of assuming the
            // vehicle points dead straight ahead.
            std::vector<Point2D> transition_from =
                main_is_ego_lane ? cur_main->points
                                  : ego_stub_towards_heading(get_path_heading_pts(cur_main->points));

            const double bulge_mult =
                is_right_turn ? turn_lateral_bulge_mult_right : turn_lateral_bulge_mult;
            const double handle_mult =
                is_right_turn ? turn_bezier_handle_scale_mult_right
                              : turn_bezier_handle_scale_mult;
            std::vector<Point2D> transition_pts = plan_transition(transition_from, selected_turn->points, kTurnMaxHeadingDiffRad, handle_mult, bulge_mult);
            if (!transition_pts.empty()) {
                plan.points = resample_path(transition_pts, 100.0);
                plan.target_lane_id = selected_turn->lane_id;
                if (main_is_ego_lane) {
                    plan.source_lane_ids.push_back(std::to_string(cur_main->label) + ":" + cur_main->lane_id);
                }
                plan.source_lane_ids.push_back(std::to_string(selected_turn->label) + ":" + selected_turn->lane_id);
                plan.confidence = selected_turn->confidence;
                plan.valid = (plan.points.size() >= 2);
            }
            
            // If transition plan failed or is invalid, fallback to follow main
            if (!plan.valid) {
                plan = plan_follow_main(obs, prev_state, last_main_id);
                plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN; // Mark it as fallback follow_main
            }
        } else if (selected_turn) {
            // Standalone turn lane: no main-lane observation this frame. This is
            // typically the last frame or two before the turn-lane itself leaves
            // view too - main-lane detection breaks down right at the junction
            // edge, just ahead of the turn-lane disappearing - which makes this
            // exactly the path TrajectoryLatch freezes from. Route it through the
            // same bulged connector as the paired branch above (anchored at the
            // vehicle, since there is no main-lane path to anchor from) instead of
            // using selected_turn->points directly, which carried no bulge at all
            // and was silently handing the latch an unbulged path to freeze.
            //
            // There is no main-lane observation THIS frame to take a heading
            // from, so fall back to the last committed trajectory's own heading
            // (the best available memory of which way the main lane was
            // pointing) rather than assuming the vehicle is dead straight ahead.
            double ego_heading = (prev_state.trajectory.valid && prev_state.trajectory.points.size() >= 2)
                                      ? get_path_heading_pts(prev_state.trajectory.points)
                                      : 0.0;
            const std::vector<Point2D> ego_stub = ego_stub_towards_heading(ego_heading);
            const double bulge_mult =
                is_right_turn ? turn_lateral_bulge_mult_right : turn_lateral_bulge_mult;
            const double handle_mult =
                is_right_turn ? turn_bezier_handle_scale_mult_right
                              : turn_bezier_handle_scale_mult;
            std::vector<Point2D> transition_pts = plan_transition(
                ego_stub, selected_turn->points, kTurnMaxHeadingDiffRad,
                handle_mult, bulge_mult);
            // Even if the Bezier connector rejects the geometry outright, still
            // guarantee the path touches the vehicle rather than handing back
            // selected_turn->points completely disconnected from it.
            plan.points = resample_path(
                transition_pts.empty() ? bridge_from_vehicle(selected_turn->points) : transition_pts, 100.0);
            plan.target_lane_id = selected_turn->lane_id;
            plan.source_lane_ids.push_back(std::to_string(selected_turn->label) + ":" + selected_turn->lane_id);
            plan.confidence = selected_turn->confidence;
            plan.valid = (plan.points.size() >= 2);
        } else if (cur_main) {
            // Turn-lane momentarily lost. Prefer holding the previous committed turn
            // trajectory (memory fallback, mirrors plan_follow_main's own no-detection
            // fallback above) over silently downgrading to follow_main, so a brief
            // perception dropout does not discard an in-progress maneuver.
            TrajectoryKind expected_kind = is_right_turn ? TrajectoryKind::TURN_RIGHT : TrajectoryKind::TURN_LEFT;
            if (prev_state.trajectory.valid && prev_state.trajectory.trajectory_kind == expected_kind) {
                plan.points = prev_state.trajectory.points;
                plan.target_lane_id = prev_state.trajectory.target_lane_id;
                plan.source_lane_ids = prev_state.trajectory.source_lane_ids;
                // Floor the memory-hold decay: an unbounded *0.8 per frame underflows
                // to denormals (1e-323 observed live) after minutes of holding.
                plan.confidence = std::max(0.05, prev_state.trajectory.confidence * 0.8);
                plan.valid = true;

                plan.has_precomputed_control = prev_state.trajectory.has_precomputed_control;
                plan.precomputed_epsilon_x_mm = prev_state.trajectory.precomputed_epsilon_x_mm;
                plan.precomputed_epsilon_y_mm = prev_state.trajectory.precomputed_epsilon_y_mm;
                plan.precomputed_theta_rad = prev_state.trajectory.precomputed_theta_rad;
                plan.precomputed_curvature_inv_mm = prev_state.trajectory.precomputed_curvature_inv_mm;
                plan.precomputed_lookahead_d_mm = prev_state.trajectory.precomputed_lookahead_d_mm;
            } else {
                // No matching turn memory to fall back on: fallback to follow main
                plan = plan_follow_main(obs, prev_state, last_main_id);
                plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
            }
        } else {
            // Recovery / invalid
            plan.valid = false;
            plan.confidence = 0.0;
        }

        return plan;
    }

    static PlannedTrajectory plan_lane_change_left(const PathObservationFrame& obs,
                                                    const CommittedTrajectoryState& prev_state,
                                                    std::string& last_main_id) {
        return plan_lane_change_generic(obs, prev_state, true, last_main_id);
    }
    
    static PlannedTrajectory plan_lane_change_right(const PathObservationFrame& obs, 
                                                     const CommittedTrajectoryState& prev_state,
                                                     std::string& last_main_id) {
        return plan_lane_change_generic(obs, prev_state, false, last_main_id);
    }

    static PlannedTrajectory plan_lane_change_generic(const PathObservationFrame& obs,
                                                      const CommittedTrajectoryState& prev_state,
                                                      bool is_left_change,
                                                      std::string& last_main_id) {
        PlannedTrajectory plan;
        plan.trajectory_kind = is_left_change ? TrajectoryKind::LANE_CHANGE_LEFT : TrajectoryKind::LANE_CHANGE_RIGHT;

        // 1. Select the current main lane
        const LaneObservation* cur_main = select_main_current(obs, last_main_id);
        if (cur_main) {
            last_main_id = cur_main->lane_id;
        }

        // 2. Select the target other lane
        const LaneObservation* target_other = select_other_lane_obs(obs, cur_main, is_left_change);

        if (cur_main && target_other) {
            // 3. Check if lane change is blocked by a solid marking
            bool marking_confidence_low = false;
            bool blocked = is_lane_change_blocked_by_solid_obs(cur_main, target_other, obs.markings, &marking_confidence_low);
            if (blocked) {
                // If blocked, plan follow main but set blocked_by_marking = true
                plan = plan_follow_main(obs, prev_state, last_main_id);
                plan.blocked_by_marking = true;
                plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
            } else {
                plan.marking_confidence_low = marking_confidence_low;
                // Plan transition
                std::vector<Point2D> transition_pts = plan_transition(cur_main->points, target_other->points);
                if (!transition_pts.empty()) {
                    plan.points = resample_path(transition_pts, 100.0);
                    plan.target_lane_id = target_other->lane_id;
                    plan.source_lane_ids.push_back(std::to_string(cur_main->label) + ":" + cur_main->lane_id);
                    plan.source_lane_ids.push_back(std::to_string(target_other->label) + ":" + target_other->lane_id);
                    plan.confidence = target_other->confidence;
                    plan.valid = (plan.points.size() >= 2);
                }
                
                // If transition planning failed, fallback to follow main
                if (!plan.valid) {
                    plan = plan_follow_main(obs, prev_state, last_main_id);
                    plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
                }
            }
        } else if (cur_main) {
            // Target other lane momentarily lost. Prefer holding the previous committed
            // lane-change trajectory (memory fallback, mirrors plan_follow_main's own
            // no-detection fallback and plan_turn_generic's analogous fallback) over
            // silently downgrading to follow_main on a brief perception dropout.
            TrajectoryKind expected_kind = is_left_change ? TrajectoryKind::LANE_CHANGE_LEFT : TrajectoryKind::LANE_CHANGE_RIGHT;
            if (prev_state.trajectory.valid && prev_state.trajectory.trajectory_kind == expected_kind) {
                plan.points = prev_state.trajectory.points;
                plan.target_lane_id = prev_state.trajectory.target_lane_id;
                plan.source_lane_ids = prev_state.trajectory.source_lane_ids;
                // Floor the memory-hold decay: an unbounded *0.8 per frame underflows
                // to denormals (1e-323 observed live) after minutes of holding.
                plan.confidence = std::max(0.05, prev_state.trajectory.confidence * 0.8);
                plan.valid = true;

                plan.has_precomputed_control = prev_state.trajectory.has_precomputed_control;
                plan.precomputed_epsilon_x_mm = prev_state.trajectory.precomputed_epsilon_x_mm;
                plan.precomputed_epsilon_y_mm = prev_state.trajectory.precomputed_epsilon_y_mm;
                plan.precomputed_theta_rad = prev_state.trajectory.precomputed_theta_rad;
                plan.precomputed_curvature_inv_mm = prev_state.trajectory.precomputed_curvature_inv_mm;
                plan.precomputed_lookahead_d_mm = prev_state.trajectory.precomputed_lookahead_d_mm;
            } else {
                // No matching lane-change memory to fall back on: fallback to follow main
                plan = plan_follow_main(obs, prev_state, last_main_id);
                plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
            }
        } else if (target_other) {
            // Only other lane is detected
            plan.points = resample_path(target_other->points, 100.0);
            plan.target_lane_id = target_other->lane_id;
            plan.source_lane_ids.push_back(std::to_string(target_other->label) + ":" + target_other->lane_id);
            plan.confidence = target_other->confidence;
            plan.valid = (plan.points.size() >= 2);
        } else {
            // Recovery / invalid
            plan.valid = false;
            plan.confidence = 0.0;
        }

        return plan;
    }

    // Plan A Step 1: central dispatcher mapping route intent -> planner. Callers
    // that only need this for debug/observation purposes should pass a scratch
    // copy of last_main_id so the debug call cannot perturb the real committed
    // last_main_id used by the active decision flow.
    static PlannedTrajectory plan_candidate_for_intent(const PathObservationFrame& obs,
                                                        RouteIntent intent,
                                                        const CommittedTrajectoryState& prev_state,
                                                        bool is_t,
                                                        bool t_junction_pending,
                                                        std::string& last_main_id) {
        switch (intent) {
            case RouteIntent::TURN_RIGHT:
                return plan_turn_right(obs, prev_state, is_t, t_junction_pending, last_main_id);
            case RouteIntent::TURN_LEFT:
                return plan_turn_left(obs, prev_state, is_t, t_junction_pending, last_main_id);
            case RouteIntent::LANE_CHANGE_LEFT:
                return plan_lane_change_left(obs, prev_state, last_main_id);
            case RouteIntent::LANE_CHANGE_RIGHT:
                return plan_lane_change_right(obs, prev_state, last_main_id);
            case RouteIntent::FOLLOW_MAIN:
            case RouteIntent::LEGACY_TURN:
            case RouteIntent::LEGACY_LANE_CHANGE:
            default:
                return plan_follow_main(obs, prev_state, last_main_id);
        }
    }

private:
    // Test-only access to private selection helpers (mirrors what the retired
    // Python harness exercised directly); see test/decision_trajectory_test.cpp.
    FRIEND_TEST(TurnBulgeAsymmetry, RightTurnDoesNotLeaveTheVehicleAimedAwayFromTheTurn);
    FRIEND_TEST(TurnBulgeAsymmetry, RightTurnSitsBetweenTheTightAndWideShapes);
    FRIEND_TEST(PlanBLaneRules, OtherLaneNearestAbsoluteSelection);
    FRIEND_TEST(PlanFLegality, FilteredFrameRemovesLaneChangeTarget);
    FRIEND_TEST(PlanBLaneRules, TurnRightTwoLanesSameSidePicksNearerStably);
    FRIEND_TEST(PlanBLaneRules, TurnLeftTwoLanesSameSidePicksFartherStably);
    FRIEND_TEST(PlanBLaneRules, TurnLaneSelectionStableUnderJitter);

    static double get_lane_heading_obs(const LaneObservation& lane) {
        if (lane.points.size() < 2) {
            return lane.has_precomputed_control ? lane.precomputed_theta_rad : 0.0;
        }
        size_t end_idx = std::min(lane.points.size() - 1, size_t(3));
        double dx = lane.points[end_idx].x - lane.points.front().x;
        double dy = lane.points[end_idx].y - lane.points.front().y;
        return std::atan2(dx, dy);
    }

    // Same heading estimate as get_lane_heading_obs, but for a raw point list
    // (e.g. a committed trajectory's points) rather than a LaneObservation.
    static double get_path_heading_pts(const std::vector<Point2D>& pts) {
        if (pts.size() < 2) return 0.0;
        size_t end_idx = std::min(pts.size() - 1, size_t(3));
        double dx = pts[end_idx].x - pts.front().x;
        double dy = pts[end_idx].y - pts.front().y;
        return std::atan2(dx, dy);
    }

    // A vehicle-anchored stub oriented along a known heading, for use as the
    // "current_pts" side of plan_transition when the vehicle's own lane isn't
    // directly observable this frame but a heading estimate is available from
    // elsewhere (a main lane seen further out, or the last committed path).
    // Using a fixed straight-ahead stub here regardless of the real lane
    // direction is what let the turn/lane-change connector diverge from the
    // actual main-lane heading right after leaving the vehicle.
    static std::vector<Point2D> ego_stub_towards_heading(double heading_rad, double length_mm = 100.0) {
        return { {0.0, 0.0}, {length_mm * std::sin(heading_rad), length_mm * std::cos(heading_rad)} };
    }

    // Guarantees vehicle connectivity as an absolute last resort: a straight
    // bridge from the vehicle position to the target path's own first point,
    // used only when plan_transition's Bezier connector rejects the geometry
    // entirely (empty result) and there is nothing better to hand back than
    // the target lane's raw points.
    static std::vector<Point2D> bridge_from_vehicle(const std::vector<Point2D>& target_pts) {
        if (target_pts.empty()) return target_pts;
        std::vector<Point2D> bridged;
        bridged.reserve(target_pts.size() + 1);
        bridged.push_back({0.0, 0.0});
        bridged.insert(bridged.end(), target_pts.begin(), target_pts.end());
        return bridged;
    }

    static const LaneObservation* select_other_lane_obs(const PathObservationFrame& obs,
                                                        const LaneObservation* main_lane,
                                                        bool is_left_change) {
        std::vector<const LaneObservation*> other_lanes;
        for (const auto& l : obs.lanes) {
            if (l.label == LABEL_OTHER_LANE || l.class_name == "other-lane") {
                other_lanes.push_back(&l);
            }
        }
        if (other_lanes.empty()) return nullptr;

        double main_x = 0.0;
        double main_heading = 0.0;
        
        if (main_lane) {
            if (!main_lane->points.empty()) {
                double sum_x = 0.0;
                for (const auto& pt : main_lane->points) {
                    sum_x += pt.x;
                }
                main_x = sum_x / main_lane->points.size();
                main_heading = get_lane_heading_obs(*main_lane);
            } else if (main_lane->has_precomputed_control) {
                main_x = main_lane->precomputed_epsilon_x_mm;
                main_heading = main_lane->precomputed_theta_rad;
            }
        }

        const LaneObservation* best_cand = nullptr;
        double best_score = -1e9;

        for (const auto* l : other_lanes) {
            double other_x = 0.0;
            double other_heading = 0.0;
            double min_y = 0.0;
            
            if (!l->points.empty()) {
                double sum_x = 0.0;
                double local_min_y = 1e9;
                for (const auto& pt : l->points) {
                    sum_x += pt.x;
                    if (pt.y < local_min_y) local_min_y = pt.y;
                }
                other_x = sum_x / l->points.size();
                other_heading = get_lane_heading_obs(*l);
                min_y = local_min_y;
            } else if (l->has_precomputed_control) {
                other_x = l->precomputed_epsilon_x_mm;
                other_heading = l->precomputed_theta_rad;
                min_y = 0.0;
            } else {
                continue;
            }
            
            double lateral_dist = other_x - main_x;
            
            // ── Gating (Hard Filters) ──
            // 1. Side Gate
            if (is_left_change && lateral_dist > -200.0) continue;
            if (!is_left_change && lateral_dist < 200.0) continue;
            
            // 2. Parallelism Gate (heading difference < 30 degrees)
            double diff_theta = std::abs(other_heading - main_heading);
            while (diff_theta > M_PI) diff_theta -= 2.0 * M_PI;
            while (diff_theta < -M_PI) diff_theta += 2.0 * M_PI;
            diff_theta = std::abs(diff_theta);
            if (diff_theta > (30.0 * M_PI / 180.0)) continue;
            
            // 3. Distance Gate (400mm to 1400mm)
            double abs_lat_dist = std::abs(lateral_dist);
            if (abs_lat_dist < 400.0 || abs_lat_dist > 1400.0) continue;
            
            // 4. Corridor Overlap Gate
            if (min_y > 1200.0) continue;

            // ── Scoring ──
            // Nearest lateral distance to main wins (decision_sys.md: "chọn lane gần
            // main nhất"), not closest-to-an-assumed-lane-width; the 400-1400mm gate
            // above already excludes implausibly close slivers.
            double score = -abs_lat_dist - 1000.0 * diff_theta;
            if (score > best_score) {
                best_score = score;
                best_cand = l;
            }
        }

        return best_cand;
    }

    static bool is_lane_change_blocked_by_solid_obs(const LaneObservation* main_lane,
                                                    const LaneObservation* target_lane,
                                                    const std::vector<MarkingObservation>& markings,
                                                    bool* marking_confidence_low = nullptr) {
        if (marking_confidence_low) *marking_confidence_low = false;
        if (!main_lane || !target_lane) return false;

        auto get_x = [](const LaneObservation* l) {
            if (l->raw_obj.contains("lookahead_x_mm")) return l->raw_obj["lookahead_x_mm"].get<double>();
            if (!l->points.empty()) return l->points[0].x;
            if (l->has_precomputed_control) return l->precomputed_epsilon_x_mm;
            return 0.0;
        };
        double main_x = get_x(main_lane);
        double target_x = get_x(target_lane);

        double min_x = std::min(main_x, target_x);
        double max_x = std::max(main_x, target_x);

        double p0_y = 0.0;
        double p3_y = 2000.0;
        
        std::vector<Point2D> main_wps = main_lane->points;
        std::sort(main_wps.begin(), main_wps.end(), [](const Point2D& a, const Point2D& b) {
            return a.y < b.y;
        });

        std::vector<Point2D> target_wps = target_lane->points;
        std::sort(target_wps.begin(), target_wps.end(), [](const Point2D& a, const Point2D& b) {
            return a.y < b.y;
        });

        double cum_dist = 0.0;
        if (!main_wps.empty()) {
            p0_y = main_wps[0].y;
            for (size_t i = 1; i < main_wps.size(); ++i) {
                double dx = main_wps[i].x - main_wps[i-1].x;
                double dy = main_wps[i].y - main_wps[i-1].y;
                cum_dist += std::hypot(dx, dy);
                if (cum_dist >= 300.0) {
                    p0_y = main_wps[i].y;
                    break;
                }
            }
        }
        
        cum_dist = 0.0;
        if (!target_wps.empty()) {
            p3_y = target_wps.back().y;
            for (size_t i = 1; i < target_wps.size(); ++i) {
                double dx = target_wps[i].x - target_wps[i-1].x;
                double dy = target_wps[i].y - target_wps[i-1].y;
                cum_dist += std::hypot(dx, dy);
                if (cum_dist >= 1200.0) {
                    p3_y = target_wps[i].y;
                    break;
                }
            }
        }
        
        double y_min = std::min(p0_y, p3_y) - 100.0;
        double y_max = std::max(p0_y, p3_y) + 300.0;

        bool any_marking_in_corridor = false;
        for (const auto& m : markings) {
            bool is_solid = (m.label == LABEL_DOUBLE_SOLID_WHITE || m.label == LABEL_SOLID_WHITE || m.label == LABEL_SOLID_YELLOW);
            bool is_dashed = (m.label == LABEL_DASHED_WHITE || m.label == LABEL_DASHED_YELLOW);
            if (is_solid || is_dashed) {
                bool is_between = false;
                if (m.raw_obj.contains("lookahead_x_mm") && !m.raw_obj.contains("waypoints") && !m.raw_obj.contains("polygons_real_world")) {
                    double mark_x = m.raw_obj["lookahead_x_mm"].get<double>();
                    double mark_y = 600.0;
                    is_between = (mark_x > min_x && mark_x < max_x && mark_y >= y_min && mark_y <= y_max);
                } else if (m.raw_obj.contains("waypoints") && !m.raw_obj["waypoints"].empty()) {
                    for (const auto& wp : m.raw_obj["waypoints"]) {
                        double mark_x = wp[0].get<double>();
                        double mark_y = wp[1].get<double>();
                        if (mark_x > min_x && mark_x < max_x && mark_y >= y_min && mark_y <= y_max) {
                            is_between = true;
                            break;
                        }
                    }
                } else if (m.raw_obj.contains("polygons_real_world") && !m.raw_obj["polygons_real_world"].empty()) {
                    for (const auto& poly : m.raw_obj["polygons_real_world"]) {
                        if (poly.is_array()) {
                            for (const auto& pt : poly) {
                                if (pt.is_array() && pt.size() >= 2) {
                                    double mark_x = pt[0].get<double>();
                                    double mark_y = pt[1].get<double>();
                                    if (mark_x > min_x && mark_x < max_x && mark_y >= y_min && mark_y <= y_max) {
                                        is_between = true;
                                        break;
                                    }
                                }
                            }
                        }
                        if (is_between) break;
                    }
                }

                if (is_between) {
                    any_marking_in_corridor = true;
                    if (is_solid) return true;
                }
            }
        }
        // No dashed OR solid marking detected at all in the corridor: perception
        // may simply have missed the marking. Default policy stays "allowed",
        // but flag low confidence so it is visible in debug output.
        if (marking_confidence_low && !any_marking_in_corridor) {
            *marking_confidence_low = true;
        }
        return false;
    }
    
private:
    static const LaneObservation* select_turn_lane_obs(const PathObservationFrame& obs,
                                                       bool is_turn_right,
                                                       bool is_t_junction) {
        std::vector<const LaneObservation*> turn_lanes;
        for (const auto& l : obs.lanes) {
            if (l.label == LABEL_TURN_LANE) turn_lanes.push_back(&l);
        }
        
        if (turn_lanes.empty()) return nullptr;

        // First pass: identify if any candidate is on the strict correct side
        bool correct_side_exists = false;
        for (const auto* l : turn_lanes) {
            double avg_x = 0.0;
            if (!l->points.empty()) {
                double sum_x = 0;
                for (const auto& pt : l->points) {
                    sum_x += pt.x;
                }
                avg_x = sum_x / l->points.size();
            } else if (l->has_precomputed_control) {
                avg_x = l->precomputed_epsilon_x_mm;
            }
            if (is_turn_right && avg_x >= 0.0) correct_side_exists = true;
            if (!is_turn_right && avg_x <= 0.0) correct_side_exists = true;
        }

        std::vector<std::pair<double, const LaneObservation*>> scored_lanes;
        for (const auto* l : turn_lanes) {
            double min_dist = 1e9;
            double avg_x = 0.0;
            
            if (!l->points.empty()) {
                double sum_x = 0;
                for (const auto& pt : l->points) {
                    sum_x += pt.x;
                    double dist = std::sqrt(pt.x*pt.x + pt.y*pt.y);
                    if (dist < min_dist) min_dist = dist;
                }
                avg_x = sum_x / l->points.size();
            } else if (l->has_precomputed_control) {
                min_dist = l->precomputed_lookahead_d_mm;
                avg_x = l->precomputed_epsilon_x_mm;
            } else {
                continue;
            }

            if (!is_t_junction) {
                if (is_turn_right && avg_x < 0) continue;
                if (!is_turn_right && avg_x > 0) continue;
            } else {
                if (correct_side_exists) {
                    if (is_turn_right && avg_x < 0.0) continue;
                    if (!is_turn_right && avg_x > 0.0) continue;
                } else {
                    if (is_turn_right && avg_x < -200.0) continue;
                    if (!is_turn_right && avg_x > 200.0) continue;
                }
            }

            scored_lanes.push_back({min_dist, l});
        }
        
        if (scored_lanes.empty()) return nullptr;

        std::sort(scored_lanes.begin(), scored_lanes.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        if (is_turn_right) {
            return scored_lanes.front().second; // closest
        } else {
            return scored_lanes.back().second;  // farthest
        }
    }

    // Max heading divergence accepted between the current lane and the
    // transition target. Lane changes connect nearly-parallel lanes, so a
    // tight gate rejects garbage matches. Turn lanes are transverse by
    // definition (a real 90-degree turn-lane observed near the corner reads
    // 60-90 degrees in vehicle frame), so the turn planner passes the wider
    // gate - the cubic Bezier below is tangent-constrained at both ends and
    // handles large heading changes by design.
    static constexpr double kLaneChangeMaxHeadingDiffRad = 40.0 * M_PI / 180.0;
    static constexpr double kTurnMaxHeadingDiffRad = 110.0 * M_PI / 180.0;

    // turn_bezier_handle_scale_mult / turn_lateral_bulge_mult live in the
    // public section at the top of the class - they are runtime-settable.

    // A main-lane observation whose first waypoint is farther ahead than this is
    // treated as resuming across an intersection gap: plan_follow_main bridges
    // from the vehicle origin so the committed path stays anchored to the vehicle.
    // Ordinary lanes enter the BEV window near y=0 and stay untouched.
    static constexpr double kBridgeMinLaneStartYMm = 600.0;

    // The same question asked for a turn, where the wrong answer costs far more.
    // Bridging a follow_main path from a lane that starts 600mm out merely begins
    // the path a little ahead of the vehicle. Anchoring a *turn* on the far-side
    // main lane instead builds a connector that runs across the junction, down
    // the far lane and then hooks back to the turn marking - a path the vehicle
    // is nowhere near, aimed initially away from the turn.
    //
    // 600mm was too loose to catch it. Measured with a turn intent live
    // (2026-08-05, run15/run16): the first waypoint of the emitted path is
    // bimodal - genuine ego lanes start at a median of 100mm, the frames that
    // hook backwards start at 500-600mm with a median of exactly 600, sitting
    // right on the boundary the test allowed through. 13-14% of frames were
    // affected. 300mm separates the two clusters with room on both sides.
    static constexpr double kTurnAnchorMaxLaneStartYMm = 300.0;

    // Same convention as PathObservationBuilder's lane confidence (straight-line
    // extent / 5000mm and point count / 10, averaged) so a bridged path is rated
    // like any observed lane path of the same span.
    static double geometry_confidence(const std::vector<Point2D>& pts) {
        if (pts.size() < 2) return 0.0;
        double dx = pts.back().x - pts.front().x;
        double dy = pts.back().y - pts.front().y;
        double len_factor = std::min(1.0, std::sqrt(dx * dx + dy * dy) / 5000.0);
        double pts_factor = std::min(1.0, static_cast<double>(pts.size()) / 10.0);
        return 0.5 * len_factor + 0.5 * pts_factor;
    }

    static std::vector<Point2D> plan_transition(const std::vector<Point2D>& current_pts,
                                                const std::vector<Point2D>& target_pts,
                                                double max_heading_diff_rad = kLaneChangeMaxHeadingDiffRad,
                                                double handle_scale_mult = 1.0,
                                                double lateral_bulge_mult = 0.0) {
        if (current_pts.size() < 2 || target_pts.size() < 2) {
            return {};
        }

        // Safety guard: check if target lane is too far or heading is too divergent
        double cur_heading = 0.0;
        if (current_pts.size() >= 2) {
            cur_heading = std::atan2(current_pts[1].x - current_pts[0].x, current_pts[1].y - current_pts[0].y);
        }
        double target_heading = 0.0;
        if (target_pts.size() >= 2) {
            target_heading = std::atan2(target_pts[1].x - target_pts[0].x, target_pts[1].y - target_pts[0].y);
        }
        
        double cur_x = current_pts.front().x;
        double target_x = target_pts.front().x;
        double lat_dist = std::abs(target_x - cur_x);
        double heading_diff = std::abs(target_heading - cur_heading);
        while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
        heading_diff = std::abs(heading_diff);

        if (lat_dist > 1500.0 || heading_diff > max_heading_diff_rad) {
            return {};
        }

        // Find P0: ~300mm along the current lane
        Point2D P0 = current_pts.front();
        Point2D p_prev = P0;
        double cum_dist = 0.0;
        size_t split_idx_current = 0;
        for (size_t i = 1; i < current_pts.size(); ++i) {
            cum_dist += std::hypot(current_pts[i].x - current_pts[i-1].x, current_pts[i].y - current_pts[i-1].y);
            if (cum_dist >= 300.0) {
                P0 = current_pts[i];
                p_prev = current_pts[i-1];
                split_idx_current = i;
                break;
            }
        }
        if (split_idx_current == 0 && current_pts.size() > 1) {
            split_idx_current = 1;
            P0 = current_pts[1];
            p_prev = current_pts[0];
        }

        // Find P3: ~1200mm along the target lane
        Point2D P3 = target_pts.back();
        Point2D p_next = P3;
        cum_dist = 0.0;
        size_t split_idx_target = target_pts.size() - 1;
        for (size_t i = 1; i < target_pts.size(); ++i) {
            cum_dist += std::hypot(target_pts[i].x - target_pts[i-1].x, target_pts[i].y - target_pts[i-1].y);
            if (cum_dist >= 1200.0) {
                P3 = target_pts[i];
                p_next = (i + 1 < target_pts.size()) ? target_pts[i+1] : P3;
                split_idx_target = i;
                break;
            }
        }
        if (split_idx_target == target_pts.size() - 1 && target_pts.size() > 1) {
            split_idx_target = target_pts.size() / 2;
            if (split_idx_target == 0) split_idx_target = 1;
            P3 = target_pts[split_idx_target];
            p_next = (split_idx_target + 1 < target_pts.size()) ? target_pts[split_idx_target+1] : P3;
        }

        double dx0 = P0.x - p_prev.x;
        double dy0 = P0.y - p_prev.y;
        double len0 = std::sqrt(dx0*dx0 + dy0*dy0);
        if (len0 < 1e-3) { dx0 = 0; dy0 = 1.0; }
        else { dx0 /= len0; dy0 /= len0; }
        
        double dx3 = p_next.x - P3.x;
        double dy3 = p_next.y - P3.y;
        double len3 = std::sqrt(dx3*dx3 + dy3*dy3);
        if (len3 < 1e-3) { dx3 = 0; dy3 = 1.0; }
        else { dx3 /= len3; dy3 /= len3; }

        double dist = std::sqrt((P3.x - P0.x)*(P3.x - P0.x) + (P3.y - P0.y)*(P3.y - P0.y));
        double scale = dist / 3.0 * handle_scale_mult;
        
        Point2D P1 = { P0.x + dx0 * scale, P0.y + dy0 * scale };
        Point2D P2 = { P3.x - dx3 * scale, P3.y - dy3 * scale };

        if (lateral_bulge_mult != 0.0) {
            // Deepen the curve on the side a natural circular arc through
            // (P0, d0) -> (P3, d3) bulges to - decoupled from tangent-handle
            // length so it can add curve depth without pushing the handles
            // into the unstable, self-crossing region that pure handle
            // scaling runs into.
            double chord_x = P3.x - P0.x;
            double chord_y = P3.y - P0.y;
            double chord_len = std::hypot(chord_x, chord_y);
            if (chord_len > 1e-3) {
                double perp_x = -chord_y / chord_len;
                double perp_y = chord_x / chord_len;
                // The arc's belly sits on the side the ENTRY tangent leans off
                // the chord (for an arc, entry and exit tangents lean to
                // opposite sides and the curve bulges toward the entry side).
                // That keeps the vehicle's current heading for longer and does
                // the bending late, swinging out through the middle of the
                // intersection. Reinforcing the target's own side instead
                // (chord_x's sign) inverts the belly: the path veers toward
                // the turn the instant it leaves the vehicle and hugs the
                // inside corner, which is what this fixes.
                double side = perp_x * dx0 + perp_y * dy0;
                if (std::abs(side) < 1e-6) {
                    // Entry tangent runs along the chord (nothing to lean).
                    // The exit tangent leans to the far side of the belly, so
                    // its dot product picks the same side with a sign flip.
                    side = -(perp_x * dx3 + perp_y * dy3);
                }
                // Both tangents collinear with the chord means a straight
                // line - no side to bulge to, so leave the handles alone
                // rather than inventing an arbitrary swerve.
                if (std::abs(side) > 1e-6) {
                    if (side < 0.0) {
                        perp_x = -perp_x;
                        perp_y = -perp_y;
                    }
                    // A negative multiplier asks for the inverted belly the
                    // comment above describes - leaning into the turn straight
                    // off the vehicle rather than holding heading and bending
                    // late. Right turns want that; they have no room outside
                    // the corner to swing into.
                    if (lateral_bulge_mult < 0.0) {
                        perp_x = -perp_x;
                        perp_y = -perp_y;
                    }
                    double bulge = chord_len * std::abs(lateral_bulge_mult);
                    P1.x += perp_x * bulge;
                    P1.y += perp_y * bulge;
                    P2.x += perp_x * bulge;
                    P2.y += perp_y * bulge;
                }
            }
        }

        std::vector<Point2D> result;
        for (size_t i = 0; i <= split_idx_current; ++i) {
            result.push_back(current_pts[i]);
        }

        int num_samples = std::max(10, static_cast<int>(dist / 50.0));
        for (int i = 1; i < num_samples; ++i) {
            double t = static_cast<double>(i) / num_samples;
            double u = 1.0 - t;
            double w0 = u * u * u;
            double w1 = 3.0 * u * u * t;
            double w2 = 3.0 * u * t * t;
            double w3 = t * t * t;
            double bx = w0*P0.x + w1*P1.x + w2*P2.x + w3*P3.x;
            double by = w0*P0.y + w1*P1.y + w2*P2.y + w3*P3.y;
            result.push_back({bx, by});
        }

        for (size_t i = split_idx_target; i < target_pts.size(); ++i) {
            result.push_back(target_pts[i]);
        }

        return result;
    }

    static const LaneObservation* select_main_current(const PathObservationFrame& obs, const std::string& last_main_id) {
        std::vector<const LaneObservation*> main_lanes;
        for (const auto& l : obs.lanes) {
            if (l.label == LABEL_MAIN_LANE || l.class_name == "main-lane") {
                // Plan E2: reject geometrically implausible candidates before
                // scoring. A blown-up projection (start behind the vehicle or
                // reaching beyond perception range) would otherwise win the
                // |start_x| + 0.5*start_y score with its huge negative start_y.
                if (!l.points.empty() &&
                    (l.points.front().y < kMinPlausibleLaneStartYMm ||
                     l.points.back().y > kMaxPlausibleLaneEndYMm)) {
                    continue;
                }
                main_lanes.push_back(&l);
            }
        }

        if (main_lanes.empty()) return nullptr;
        
        double min_start_y = 1e9;
        for (const auto* l : main_lanes) {
            if (!l->points.empty()) {
                min_start_y = std::min(min_start_y, l->points.front().y);
            }
        }
        
        const LaneObservation* best_lane = nullptr;
        double best_score = 1e9;
        
        for (const auto* l : main_lanes) {
            double start_x = 0.0;
            double start_y = 0.0;
            bool has_wps = !l->points.empty();
            
            if (has_wps) {
                start_x = l->points.front().x;
                start_y = l->points.front().y;
            } else if (l->has_precomputed_control) {
                start_x = 0.0;
                start_y = 0.0;
            } else {
                continue;
            }
            
            double score = std::abs(start_x) + 0.5 * start_y;
            if (!has_wps) {
                score += 5000.0;
            }
            
            if (!last_main_id.empty() && l->lane_id == last_main_id) {
                if (start_y - min_start_y <= 600.0) {
                    score -= 1500.0;
                }
            }
            
            if (score < best_score) {
                best_score = score;
                best_lane = l;
            }
        }
        
        return best_lane ? best_lane : main_lanes.front();
    }
    
    static const LaneObservation* select_main_ahead(const PathObservationFrame& obs, const LaneObservation* cur_lane) {
        if (!cur_lane || cur_lane->points.size() < 2) return nullptr;
        
        double cur_end_x = cur_lane->points.back().x;
        double cur_end_y = cur_lane->points.back().y;
        double cur_prev_x = cur_lane->points[cur_lane->points.size() - 2].x;
        double cur_prev_y = cur_lane->points[cur_lane->points.size() - 2].y;
        double cur_theta = std::atan2(cur_end_x - cur_prev_x, cur_end_y - cur_prev_y);
        
        const LaneObservation* best_ahead = nullptr;
        double best_ahead_y = 1e9;
        
        for (const auto& l : obs.lanes) {
            bool is_main = (l.label == LABEL_MAIN_LANE || l.class_name == "main-lane");
            if (&l == cur_lane || !is_main || l.points.size() < 2) continue;
            
            double ahead_start_x = l.points.front().x;
            double ahead_start_y = l.points.front().y;
            double ahead_next_x = l.points[1].x;
            double ahead_next_y = l.points[1].y;
            double ahead_theta = std::atan2(ahead_next_x - ahead_start_x, ahead_next_y - ahead_start_y);
            
            double long_gap = ahead_start_y - cur_end_y;
            if (long_gap < -500.0 || long_gap > 2000.0) continue;
            
            double lat_jump = std::abs(ahead_start_x - cur_end_x);
            if (lat_jump > 400.0) continue;
            
            double diff_theta = std::abs(ahead_theta - cur_theta);
            while (diff_theta > M_PI) diff_theta -= 2.0 * M_PI;
            while (diff_theta < -M_PI) diff_theta += 2.0 * M_PI;
            diff_theta = std::abs(diff_theta);
            if (diff_theta > (30.0 * M_PI / 180.0)) continue;
            
            if (ahead_start_y < best_ahead_y) {
                best_ahead_y = ahead_start_y;
                best_ahead = &l;
            }
        }
        
        return best_ahead;
    }
    
    static std::vector<Point2D> merge_lanes(const LaneObservation& cur, const LaneObservation& ahead) {
        std::vector<Point2D> merged = cur.points;
        if (ahead.points.empty()) return merged;
        
        double end_y = cur.points.empty() ? -1e9 : cur.points.back().y;
        for (const auto& pt : ahead.points) {
            if (pt.y > end_y + 10.0) {
                merged.push_back(pt);
            }
        }
        return merged;
    }
    
public:
    // Join the vehicle to a lane whose waypoints only begin some distance
    // ahead, without giving up a single one of them.
    //
    // plan_transition cannot be used for this. It splits the target at ~1200mm
    // and throws everything before that away (and when the observation is
    // shorter than 1200mm its fallback throws away *half* of it), replacing the
    // lane's own centreline with Bezier samples shaped by the vehicle heading.
    // That is deliberate for a lane change, where easing across into the target
    // over a distance is the point. For follow-main it is wrong: the observed
    // waypoints ARE the lane centre, and the only thing missing is the stretch
    // between the vehicle and where they start.
    //
    // So the connector spans exactly that gap - vehicle origin to the first
    // waypoint - and every waypoint is then appended untouched. The connector
    // leaves the vehicle along its own heading and arrives along the lane's
    // initial tangent, so the join carries no kink.
    //
    // Returns empty when the lane is too far off to the side or points back at
    // the vehicle; the caller then keeps the raw observation rather than
    // following a fabricated swerve.
    static std::vector<Point2D> bridge_gap_to_lane(const std::vector<Point2D>& lane_pts) {
        if (lane_pts.size() < 2) return {};

        const Point2D P0{0.0, 0.0};
        const Point2D P3 = lane_pts.front();

        double gap = std::hypot(P3.x - P0.x, P3.y - P0.y);
        if (gap < 1e-3) return lane_pts;  // already at the vehicle: nothing to span

        double lane_dx = lane_pts[1].x - lane_pts[0].x;
        double lane_dy = lane_pts[1].y - lane_pts[0].y;
        double lane_len = std::hypot(lane_dx, lane_dy);
        if (lane_len < 1e-6) return {};
        lane_dx /= lane_len;
        lane_dy /= lane_len;

        // The vehicle heading is 0 by construction of the vehicle frame, so the
        // lane tangent's own angle is the heading difference.
        double heading_diff = std::abs(std::atan2(lane_dx, lane_dy));
        if (heading_diff > kTurnMaxHeadingDiffRad) return {};
        if (std::abs(P3.x) > 1500.0) return {};

        // Handles a third of the gap each: the standard cubic that reproduces a
        // circular arc closely over a short span, and short enough that it
        // cannot loop back on itself when the gap is small.
        double scale = gap / 3.0;
        const Point2D P1{P0.x, P0.y + scale};                    // leave along vehicle heading
        const Point2D P2{P3.x - lane_dx * scale, P3.y - lane_dy * scale};  // arrive along the lane

        std::vector<Point2D> out;
        int samples = std::max(4, static_cast<int>(gap / 50.0));
        for (int i = 0; i < samples; ++i) {
            double t = static_cast<double>(i) / samples;
            double u = 1.0 - t;
            double w0 = u * u * u;
            double w1 = 3.0 * u * u * t;
            double w2 = 3.0 * u * t * t;
            double w3 = t * t * t;
            out.push_back({w0 * P0.x + w1 * P1.x + w2 * P2.x + w3 * P3.x,
                           w0 * P0.y + w1 * P1.y + w2 * P2.y + w3 * P3.y});
        }
        // t == 1 is P3, which is lane_pts.front() - appended by the loop below
        // rather than sampled, so the first waypoint appears exactly once.
        out.insert(out.end(), lane_pts.begin(), lane_pts.end());
        return out;
    }

    // Resamples at arc-length positions start_offset_mm + k*step_mm (k=0,1,2,...).
    // start_offset_mm=0.0 (default) reproduces the original behavior exactly: points.front()
    // is emitted as-is, and subsequent samples start at step_mm. A positive start_offset_mm
    // instead begins sampling at that arc-length position (used to align a resample against
    // another path's progress, see project_point_to_path).
    // Public: also used by TrajectoryNormalizer/TrajectoryManager for arc-length alignment.
    static std::vector<Point2D> resample_path(const std::vector<Point2D>& points, double step_mm,
                                               double start_offset_mm = 0.0) {
        std::vector<Point2D> resampled;
        if (points.empty()) return resampled;
        if (points.size() == 1) {
            resampled.push_back(points.front());
            return resampled;
        }

        if (start_offset_mm <= 0.0) {
            resampled.push_back(points.front());
        }
        double accumulated_dist = 0.0;

        size_t next_idx = 1;
        double current_s = (start_offset_mm <= 0.0) ? 0.0 : (start_offset_mm - step_mm);

        while (next_idx < points.size()) {
            const auto& p0 = points[next_idx - 1];
            const auto& p1 = points[next_idx];
            double seg_len = std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2));

            if (seg_len < 1e-3) {
                next_idx++;
                continue;
            }

            double target_s = current_s + step_mm;
            if (accumulated_dist + seg_len >= target_s) {
                double ratio = (target_s - accumulated_dist) / seg_len;
                Point2D interpolated;
                interpolated.x = p0.x + ratio * (p1.x - p0.x);
                interpolated.y = p0.y + ratio * (p1.y - p0.y);
                resampled.push_back(interpolated);
                current_s = target_s;
            } else {
                accumulated_dist += seg_len;
                next_idx++;
            }
        }

        if (resampled.size() > 0) {
            double dist = std::sqrt(std::pow(points.back().x - resampled.back().x, 2) +
                                    std::pow(points.back().y - resampled.back().y, 2));
            if (dist > 10.0) {
                resampled.push_back(points.back());
            }
        }

        return resampled;
    }

    // Arc-length position (>=0, clamped to the path's own length) of the point on `path`
    // nearest to `query`, via standard point-to-segment projection. Used to find where the
    // vehicle currently sits along a previously-committed trajectory (progress alignment).
    static double project_point_to_path(const Point2D& query, const std::vector<Point2D>& path) {
        if (path.size() < 2) return 0.0;
        double best_dist_sq = std::numeric_limits<double>::max();
        double best_s = 0.0;
        double cum = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            const auto& a = path[i - 1];
            const auto& b = path[i];
            double seg_len = std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
            if (seg_len > 1e-6) {
                double t = ((query.x - a.x) * (b.x - a.x) + (query.y - a.y) * (b.y - a.y)) / (seg_len * seg_len);
                t = std::clamp(t, 0.0, 1.0);
                double px = a.x + t * (b.x - a.x);
                double py = a.y + t * (b.y - a.y);
                double d2 = std::pow(px - query.x, 2) + std::pow(py - query.y, 2);
                if (d2 < best_dist_sq) {
                    best_dist_sq = d2;
                    best_s = cum + t * seg_len;
                }
                cum += seg_len;
            }
        }
        return std::max(0.0, best_s);
    }
};
