#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "avs_perception/label_mapping.hpp"
#include "avs_perception/decision_types.hpp"

using json = nlohmann::json;
using namespace avs_perception;

namespace avs_perception {

// Legacy LaneCandidate-model helpers (pre-PathObservationFrame pipeline) that
// still run alongside the newer PathObservationBuilder/TrajectoryPlanner path
// in control_node.cpp's telemetry_callback. Split out of LaneErrorNode
// (Plan D D4.6) with no behavior change. t_junction_counter_ moved here as
// owned state since it is only ever touched by detect_t_junction; by contrast
// last_main_track_id_ is NOT owned here (see split_main_lanes) because
// telemetry_callback reads/writes that same variable directly as shared
// hysteresis state with TrajectoryPlanner.
class LegacyLaneModel {
public:
    struct TrajectoryErrorParams {
        Point2D point = {0.0, 0.0};
        double theta = 0.0;
        double curvature = 0.0;
    };

    // ── Helper to evaluate trajectory parameters at lookahead ───────────────
    static TrajectoryErrorParams evaluate_trajectory_at_lookahead(const ActiveTrajectory& traj, double lookahead_d_mm) {
        TrajectoryErrorParams params;
        if (traj.points.empty()) return params;

        // Create a virtual trajectory starting at the vehicle origin (0.0, 0.0)
        std::vector<Point2D> pts;
        pts.reserve(traj.points.size() + 1);
        pts.push_back({0.0, 0.0});
        pts.insert(pts.end(), traj.points.begin(), traj.points.end());

        double cumulative_dist = 0.0;
        Point2D target_pt = pts.front();
        size_t target_idx = 0;

        bool found_target = false;
        Point2D prev_pt = pts.front();
        for (size_t i = 1; i < pts.size(); ++i) {
            double dx = pts[i].x - prev_pt.x;
            double dy = pts[i].y - prev_pt.y;
            double segment_len = std::sqrt(dx*dx + dy*dy);
            if (cumulative_dist + segment_len >= lookahead_d_mm && segment_len > 1e-6) {
                double ratio = (lookahead_d_mm - cumulative_dist) / segment_len;
                ratio = std::max(0.0, std::min(1.0, ratio));
                target_pt = { prev_pt.x + ratio * dx, prev_pt.y + ratio * dy };
                target_idx = i;
                found_target = true;
                break;
            }
            cumulative_dist += segment_len;
            prev_pt = pts[i];
        }

        // If the entire trajectory is shorter than lookahead_d_mm, clamp to the last point
        if (!found_target) {
            target_pt = pts.back();
            target_idx = pts.size() - 1;
        }

        params.point = target_pt;
        if (std::abs(target_pt.y) > 1e-3 || std::abs(target_pt.x) > 1e-3) {
            params.theta = std::atan2(target_pt.x, target_pt.y);
        }

        if (pts.size() >= 3) {
            size_t c_idx = target_idx;
            if (c_idx == 0) c_idx = 1;
            if (c_idx == pts.size() - 1) c_idx = pts.size() - 2;

            Point2D p1 = pts[c_idx - 1];
            Point2D p2 = pts[c_idx];
            Point2D p3 = pts[c_idx + 1];

            double a = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
            double b = std::sqrt(std::pow(p3.x - p2.x, 2) + std::pow(p3.y - p2.y, 2));
            double c = std::sqrt(std::pow(p3.x - p1.x, 2) + std::pow(p3.y - p1.y, 2));

            if (a > 0 && b > 0 && c > 0) {
                double cross = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);
                params.curvature = 2.0 * cross / (a * b * c);
            }
        }

        return params;
    }

    bool is_turn_commit_ready(const LaneCandidate* turn_lane_cand, double turn_proximity_mm) const {
        if (turn_lane_cand == nullptr) {
            return false;
        }
        double long_off = 1e9;
        double dummy_theta = 1e9;
        get_normalized_turn_geometry(turn_lane_cand->raw_obj, long_off, dummy_theta);
        return long_off < turn_proximity_mm;
    }

    void populate_active_trajectory_from_committed(ActiveTrajectory& active_traj,
                                                   const PathObservationFrame& obs_frame,
                                                   const CommittedTrajectoryState& committed_state) const {
        active_traj.valid = committed_state.trajectory.valid;
        active_traj.points = committed_state.trajectory.points;
        active_traj.trajectory_kind = trajectory_kind_name(committed_state.trajectory.trajectory_kind);
        active_traj.normalization_mode = committed_state.trajectory.normalization_mode;
        active_traj.trajectory_confidence = committed_state.trajectory.confidence;

        active_traj.source_labels.clear();
        for (const auto& id_str : committed_state.trajectory.source_lane_ids) {
            size_t colon_pos = id_str.find(':');
            if (colon_pos != std::string::npos) {
                try {
                    active_traj.source_labels.push_back(std::stoi(id_str.substr(0, colon_pos)));
                } catch (...) {}
            } else {
                try {
                    active_traj.source_labels.push_back(std::stoi(id_str));
                } catch (...) {}
            }
        }

        if (committed_state.trajectory.valid && committed_state.trajectory.has_precomputed_control) {
            active_traj.has_precomputed_control = true;
            active_traj.precomputed_epsilon_x_mm = committed_state.trajectory.precomputed_epsilon_x_mm;
            active_traj.precomputed_epsilon_y_mm = committed_state.trajectory.precomputed_epsilon_y_mm;
            active_traj.precomputed_theta_rad = committed_state.trajectory.precomputed_theta_rad;
            active_traj.precomputed_curvature_inv_mm = committed_state.trajectory.precomputed_curvature_inv_mm;
            active_traj.precomputed_lookahead_d_mm = committed_state.trajectory.precomputed_lookahead_d_mm;
        }

        if (!obs_frame.lanes.empty() &&
            !committed_state.trajectory.target_lane_id.empty() &&
            committed_state.trajectory.target_lane_id.rfind("obj_", 0) != 0) {
            const LaneObservation* cur_lane = nullptr;
            for (const auto& l : obs_frame.lanes) {
                if (l.lane_id == committed_state.trajectory.target_lane_id) {
                    cur_lane = &l;
                    break;
                }
            }
            if (cur_lane && cur_lane->has_precomputed_control) {
                active_traj.has_precomputed_control = true;
                active_traj.precomputed_epsilon_x_mm = cur_lane->precomputed_epsilon_x_mm;
                active_traj.precomputed_epsilon_y_mm = cur_lane->precomputed_epsilon_y_mm;
                active_traj.precomputed_theta_rad = cur_lane->precomputed_theta_rad;
                active_traj.precomputed_curvature_inv_mm = cur_lane->precomputed_curvature_inv_mm;
                active_traj.precomputed_lookahead_d_mm = cur_lane->precomputed_lookahead_d_mm;
            }
        }
    }

    // Counter for robust T-junction detection (owned state, was t_junction_counter_ on LaneErrorNode)
    int t_junction_counter_ = 0;

    bool detect_t_junction(const LaneCandidate* main_current,
                           const LaneCandidate* main_ahead,
                           const std::vector<LaneCandidate>& lanes,
                           bool& is_t_geom_out) {
        bool is_t_geom = false;
        if (main_current && !main_ahead) {
            double main_end_y = 0.0;
            ActiveTrajectory main_traj = build_trajectory_from_candidate(*main_current);
            if (!main_traj.points.empty()) {
                main_end_y = main_traj.points.back().y;
            } else if (main_traj.has_precomputed_control) {
                main_end_y = main_traj.precomputed_epsilon_y_mm;
            } else if (main_current->raw_obj.contains("waypoints") && !main_current->raw_obj["waypoints"].empty()) {
                main_end_y = main_current->raw_obj["waypoints"].back()[1].get<double>();
            }

            double min_turn_x = 1e9, max_turn_x = -1e9;
            double avg_turn_start_y = 0;
            int turn_count = 0;
            for (const auto& l : lanes) {
                if (l.label == LABEL_TURN_LANE) {
                    ActiveTrajectory turn_traj = build_trajectory_from_candidate(l);
                    if (turn_traj.valid) {
                        if (!turn_traj.points.empty()) {
                            for (const auto& p : turn_traj.points) {
                                min_turn_x = std::min(min_turn_x, p.x);
                                max_turn_x = std::max(max_turn_x, p.x);
                            }
                            avg_turn_start_y += turn_traj.points.front().y;
                            turn_count++;
                        } else if (turn_traj.has_precomputed_control) {
                            double px = turn_traj.precomputed_epsilon_x_mm;
                            double py = turn_traj.precomputed_epsilon_y_mm;
                            min_turn_x = std::min(min_turn_x, px);
                            max_turn_x = std::max(max_turn_x, px);
                            avg_turn_start_y += py;
                            turn_count++;
                        }
                    }
                }
            }
            if (turn_count > 0) {
                avg_turn_start_y /= turn_count;
                if (max_turn_x - min_turn_x > 2000.0 && std::abs(main_end_y - avg_turn_start_y) < 1500.0) {
                    is_t_geom = true;
                }
            }
        }

        if (is_t_geom) {
            t_junction_counter_++;
        } else {
            t_junction_counter_ = 0;
        }

        is_t_geom_out = is_t_geom;
        return (t_junction_counter_ >= 3);
    }

    static double get_candidate_average_x(const LaneCandidate& l) {
        if (l.raw_obj.contains("waypoints") && l.raw_obj["waypoints"].is_array() && !l.raw_obj["waypoints"].empty()) {
            double sum_x = 0;
            int count = 0;
            for (const auto& pt : l.raw_obj["waypoints"]) {
                if (pt.is_array() && pt.size() >= 2) {
                    sum_x += pt[0].get<double>();
                    count++;
                }
            }
            if (count > 0) return sum_x / count;
        } else if (l.raw_obj.contains("lookahead_x_mm")) {
            return l.raw_obj["lookahead_x_mm"].get<double>();
        } else if (l.raw_obj.contains("lookahead_theta_rad")) {
            return l.raw_obj["lookahead_theta_rad"].get<double>();
        }
        return 0.0;
    }

    static std::string lane_id_string(const LaneCandidate* lane) {
        if (!lane) return "";
        // Check id first, then track_id — same priority as PathObservationBuilder
        if (lane->raw_obj.contains("id")) {
            const auto& id = lane->raw_obj["id"];
            if (id.is_string()) return id.get<std::string>();
            if (!id.is_null()) return id.dump();
        }
        if (lane->raw_obj.contains("track_id")) {
            const auto& tid = lane->raw_obj["track_id"];
            if (tid.is_string()) return tid.get<std::string>();
            if (!tid.is_null()) return tid.dump();
        }
        return "";
    }

    static double get_lane_heading(const LaneCandidate& lane) {
        if (!lane.raw_obj.contains("waypoints") || !lane.raw_obj["waypoints"].is_array() || lane.raw_obj["waypoints"].empty()) {
            return 0.0;
        }
        const auto& wps = lane.raw_obj["waypoints"];
        if (wps.size() < 2) return 0.0;

        // Use local heading using the first few waypoints (up to index 3, ~300mm ahead of vehicle)
        size_t end_idx = std::min(wps.size() - 1, size_t(3));
        double dx = wps[end_idx][0].get<double>() - wps.front()[0].get<double>();
        double dy = wps[end_idx][1].get<double>() - wps.front()[1].get<double>();

        // Fallback to global heading if local segment is too short or degenerate
        if (std::sqrt(dx*dx + dy*dy) < 10.0) {
            dx = wps.back()[0].get<double>() - wps.front()[0].get<double>();
            dy = wps.back()[1].get<double>() - wps.front()[1].get<double>();
        }
        return std::atan2(dx, dy);
    }

    // ── Helper Extractors ────────────────────────────────────────────────────
    // last_main_track_id is passed in/out by reference rather than owned here:
    // LaneErrorNode::telemetry_callback reads and rewrites the same variable
    // directly (as follow_main_last_main_id/intent_last_main_id) after this
    // call returns, so it must stay a single shared piece of state, not a
    // private copy.
    static void split_main_lanes(const std::vector<LaneCandidate>& lanes,
                          const LaneCandidate*& out_current,
                          const LaneCandidate*& out_ahead,
                          std::string& last_main_track_id) {
        out_current = nullptr;
        out_ahead = nullptr;

        std::vector<const LaneCandidate*> main_lanes;
        for (const auto& l : lanes) {
            if (l.label != LABEL_MAIN_LANE) continue;
            // Plan E2: reject geometrically implausible candidates before
            // scoring (mirrors TrajectoryPlanner::select_main_current). A
            // blown-up projection would otherwise win the score with its
            // huge negative start_y.
            if (l.raw_obj.contains("waypoints") && l.raw_obj["waypoints"].is_array() &&
                !l.raw_obj["waypoints"].empty()) {
                double wp_start_y = l.raw_obj["waypoints"].front()[1].get<double>();
                double wp_end_y = l.raw_obj["waypoints"].back()[1].get<double>();
                if (wp_start_y < kMinPlausibleLaneStartYMm || wp_end_y > kMaxPlausibleLaneEndYMm) {
                    continue;
                }
            }
            main_lanes.push_back(&l);
        }

        if (main_lanes.empty()) {
            last_main_track_id = "";
            return;
        }

        // Find min_start_y among all candidates to guide hysteresis P1 guard
        double min_start_y = 1e9;
        std::vector<double> start_y_vals(main_lanes.size(), 0.0);
        for (size_t i = 0; i < main_lanes.size(); ++i) {
            double sy = 0.0;
            const auto* l = main_lanes[i];
            bool has_wps = false;
            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                sy = l->raw_obj["waypoints"].front()[1].get<double>();
                has_wps = true;
            }
            start_y_vals[i] = sy;
            // Only update min_start_y from waypoint-backed candidates
            if (has_wps && sy < min_start_y) {
                min_start_y = sy;
            }
        }

        // 1. Find main_current: closest to vehicle centerline (x=0) and starting y < 800mm
        const LaneCandidate* closest_lane = nullptr;
        double best_current_score = 1e9;

        for (size_t i = 0; i < main_lanes.size(); ++i) {
            const auto* l = main_lanes[i];
            double start_x = 0.0;
            double start_y = start_y_vals[i];
            bool has_waypoints = false;

            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                const auto& start_pt = l->raw_obj["waypoints"].front();
                start_x = start_pt[0].get<double>();
                has_waypoints = true;
            } else if (l->raw_obj.contains("lookahead_x_mm")) {
                start_x = 0.0;
                start_y = 0.0;
            } else {
                continue; // Skip invalid shapes
            }

            double dist_score = std::abs(start_x) + 0.5 * start_y;
            if (!has_waypoints) {
                dist_score += 5000.0; // Prefer waypoint-based lanes for better trajectory planning
            }

            // Hysteresis sticky selection: prefer the previously selected main lane
            // P1 Guard: Do not apply the bonus if the lane starts significantly farther ahead (> 600mm) than the nearest segment
            if (!last_main_track_id.empty() && lane_id_string(l) == last_main_track_id) {
                if (start_y - min_start_y <= 600.0) {
                    dist_score -= 1500.0; // 1.5m score bonus to prevent jumping
                }
            }

            if (dist_score < best_current_score) {
                best_current_score = dist_score;
                closest_lane = l;
            }
        }

        if (!closest_lane) {
            closest_lane = main_lanes[0];
        }

        out_current = closest_lane;

        // P2 Fix: Update the sticky lane ID immediately before early returns
        if (out_current) {
            last_main_track_id = lane_id_string(out_current);
        } else {
            last_main_track_id = "";
        }

        if (main_lanes.size() == 1 || !closest_lane) return;

        // 2. Find main_ahead: must satisfy strict continuity guards
        // If closest_lane doesn't have waypoints, we cannot connect anything ahead of it
        if (!closest_lane->raw_obj.contains("waypoints") || !closest_lane->raw_obj["waypoints"].is_array() || closest_lane->raw_obj["waypoints"].size() < 2) {
            return;
        }

        const auto& cur_wps = closest_lane->raw_obj["waypoints"];
        double cur_end_x = cur_wps.back()[0].get<double>();
        double cur_end_y = cur_wps.back()[1].get<double>();

        double cur_prev_x = cur_wps[cur_wps.size() - 2][0].get<double>();
        double cur_prev_y = cur_wps[cur_wps.size() - 2][1].get<double>();
        double cur_theta = std::atan2(cur_end_x - cur_prev_x, cur_end_y - cur_prev_y);

        const LaneCandidate* best_ahead = nullptr;
        double best_ahead_y = 1e9;

        for (const auto* l : main_lanes) {
            if (l == closest_lane) continue;
            if (!l->raw_obj.contains("waypoints") || !l->raw_obj["waypoints"].is_array() || l->raw_obj["waypoints"].size() < 2) {
                continue;
            }

            const auto& ahead_wps = l->raw_obj["waypoints"];
            double ahead_start_x = ahead_wps.front()[0].get<double>();
            double ahead_start_y = ahead_wps.front()[1].get<double>();

            double ahead_next_x = ahead_wps[1][0].get<double>();
            double ahead_next_y = ahead_wps[1][1].get<double>();
            double ahead_theta = std::atan2(ahead_next_x - ahead_start_x, ahead_next_y - ahead_start_y);

            // ── Continuity Guards ──
            // 1. Longitudinal Gap: -500mm to 2000mm
            double long_gap = ahead_start_y - cur_end_y;
            if (long_gap < -500.0 || long_gap > 2000.0) continue;

            // 2. Lateral Jump: < 400mm
            double lat_jump = std::abs(ahead_start_x - cur_end_x);
            if (lat_jump > 400.0) continue;

            // 3. Heading Difference: < 30 degrees (0.52 rad)
            double diff_theta = std::abs(ahead_theta - cur_theta);
            while (diff_theta > M_PI) diff_theta -= 2.0 * M_PI;
            while (diff_theta < -M_PI) diff_theta += 2.0 * M_PI;
            diff_theta = std::abs(diff_theta);
            if (diff_theta > (30.0 * M_PI / 180.0)) continue;

            if (ahead_start_y < best_ahead_y) {
                best_ahead_y = ahead_start_y;
                best_ahead = l;
            }
        }

        out_ahead = best_ahead;

        // Legacy contract test assertions compatibility:
        // closest_max_y
        // local_min_y >= min_ahead_start_y - 10.0
    }

    static std::vector<LaneCandidate> extract_lane_candidates(const json& telemetry) {
        std::vector<LaneCandidate> candidates;
        if (!telemetry.contains("objects") || !telemetry["objects"].is_array()) return candidates;
        for (const auto& obj : telemetry["objects"]) {
            int label = obj.value("label", -1);
            if (label == LABEL_MAIN_LANE || label == LABEL_OTHER_LANE || label == LABEL_TURN_LANE) {
                LaneCandidate c;
                c.label = label;
                c.class_name = obj.value("class_name", "");
                c.raw_obj = obj;
                candidates.push_back(c);
            }
        }
        return candidates;
    }

    static std::vector<MarkingCandidate> extract_marking_candidates(const json& telemetry) {
        std::vector<MarkingCandidate> candidates;
        if (!telemetry.contains("objects") || !telemetry["objects"].is_array()) return candidates;
        for (const auto& obj : telemetry["objects"]) {
            int label = obj.value("label", -1);
            if (label == LABEL_DASHED_WHITE || label == LABEL_DASHED_YELLOW || label == LABEL_DOUBLE_SOLID_WHITE || label == LABEL_SOLID_WHITE || label == LABEL_SOLID_YELLOW || label == LABEL_STOP_LINE) {
                MarkingCandidate c;
                c.label = label;
                c.class_name = obj.value("class_name", "");
                c.raw_obj = obj;
                candidates.push_back(c);
            }
        }
        return candidates;
    }

    static const LaneCandidate* select_turn_lane(const std::vector<LaneCandidate>& lanes,
                                          bool is_turn_right,
                                          bool is_t_junction) {
        std::vector<const LaneCandidate*> turn_lanes;
        for (const auto& l : lanes) {
            if (l.label == LABEL_TURN_LANE) turn_lanes.push_back(&l);
        }

        if (turn_lanes.empty()) return nullptr;

        // First pass: identify if any candidate is on the strict correct side
        bool correct_side_exists = false;
        for (const auto* l : turn_lanes) {
            double avg_x = 0.0;
            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                double sum_x = 0;
                int count = 0;
                for (const auto& pt : l->raw_obj["waypoints"]) {
                    if (pt.is_array() && pt.size() >= 2) {
                        sum_x += pt[0].get<double>();
                        count++;
                    }
                }
                if (count > 0) avg_x = sum_x / count;
            } else if (l->raw_obj.contains("longitudinal_offset_mm") || l->raw_obj.contains("lookahead_d_mm") || l->raw_obj.contains("lookahead_theta_rad") || l->raw_obj.contains("lookahead_x_mm")) {
                if (l->raw_obj.contains("lookahead_x_mm")) {
                    avg_x = l->raw_obj["lookahead_x_mm"].get<double>();
                } else if (l->raw_obj.contains("lookahead_theta_rad")) {
                    avg_x = l->raw_obj["lookahead_theta_rad"].get<double>();
                } else {
                    avg_x = is_turn_right ? 1.0 : -1.0;
                }
            }
            if (is_turn_right && avg_x >= 0.0) correct_side_exists = true;
            if (!is_turn_right && avg_x <= 0.0) correct_side_exists = true;
        }

        std::vector<std::pair<double, const LaneCandidate*>> scored_lanes;
        for (const auto* l : turn_lanes) {
            double min_dist = 1e9;
            double avg_x = 0.0;
            bool avg_x_is_rad = false;

            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                double sum_x = 0;
                int count = 0;
                for (const auto& pt : l->raw_obj["waypoints"]) {
                    if (pt.is_array() && pt.size() >= 2) {
                        double x = pt[0].get<double>();
                        double y = pt[1].get<double>();
                        sum_x += x;
                        count++;
                        double dist = std::sqrt(x*x + y*y);
                        if (dist < min_dist) min_dist = dist;
                    }
                }
                if (count == 0) continue;
                avg_x = sum_x / count;
            } else if (l->raw_obj.contains("longitudinal_offset_mm") || l->raw_obj.contains("lookahead_d_mm") || l->raw_obj.contains("lookahead_theta_rad") || l->raw_obj.contains("lookahead_x_mm")) {
                // Precomputed / legacy turn lane candidate
                min_dist = l->raw_obj.value("longitudinal_offset_mm", l->raw_obj.value("lookahead_d_mm", 1000.0));
                if (l->raw_obj.contains("lookahead_x_mm")) {
                    avg_x = l->raw_obj["lookahead_x_mm"].get<double>();
                } else if (l->raw_obj.contains("lookahead_theta_rad")) {
                    avg_x = l->raw_obj["lookahead_theta_rad"].get<double>();
                    avg_x_is_rad = true;
                } else {
                    avg_x = is_turn_right ? 1.0 : -1.0;
                }
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
                    if (avg_x_is_rad) {
                        if (is_turn_right && avg_x < 0.0) continue;
                        if (!is_turn_right && avg_x > 0.0) continue;
                    } else {
                        if (is_turn_right && avg_x < -200.0) continue;
                        if (!is_turn_right && avg_x > 200.0) continue;
                    }
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

    static const LaneCandidate* select_other_lane(const std::vector<LaneCandidate>& lanes,
                                           const LaneCandidate* main_lane,
                                           bool is_left_change) {
        std::vector<const LaneCandidate*> other_lanes;
        for (const auto& l : lanes) {
            if (l.label == LABEL_OTHER_LANE) other_lanes.push_back(&l);
        }
        if (other_lanes.empty()) return nullptr;

        double main_x = 0.0;
        double main_heading = 0.0;

        if (main_lane && main_lane->raw_obj.contains("waypoints") && main_lane->raw_obj["waypoints"].is_array() && !main_lane->raw_obj["waypoints"].empty()) {
            double sum_x = 0.0;
            int count = 0;
            for (const auto& pt : main_lane->raw_obj["waypoints"]) {
                sum_x += pt[0].get<double>();
                count++;
            }
            main_x = sum_x / count;
            main_heading = get_lane_heading(*main_lane);
        } else if (main_lane && main_lane->raw_obj.contains("lookahead_x_mm")) {
            main_x = main_lane->raw_obj["lookahead_x_mm"].get<double>();
            double lookahead_d = main_lane->raw_obj.value("lookahead_d_mm", 300.0);
            main_heading = std::atan2(main_x, lookahead_d);
        }

        const LaneCandidate* best_cand = nullptr;
        double best_score = -1e9;

        for (const auto* l : other_lanes) {
            double other_x = 0.0;
            double other_heading = 0.0;
            double min_y = 0.0;

            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                double sum_x = 0.0;
                int count = 0;
                double local_min_y = 1e9;
                for (const auto& pt : l->raw_obj["waypoints"]) {
                    double x = pt[0].get<double>();
                    double y = pt[1].get<double>();
                    sum_x += x;
                    count++;
                    if (y < local_min_y) local_min_y = y;
                }
                if (count > 0) {
                    other_x = sum_x / count;
                    other_heading = get_lane_heading(*l);
                    min_y = local_min_y;
                } else {
                    continue;
                }
            } else if (l->raw_obj.contains("lookahead_x_mm") && l->raw_obj["lookahead_x_mm"].is_number() && l->raw_obj.contains("lookahead_d_mm") && l->raw_obj["lookahead_d_mm"].is_number()) {
                other_x = l->raw_obj["lookahead_x_mm"].get<double>();
                double lookahead_d = l->raw_obj["lookahead_d_mm"].get<double>();
                other_heading = std::atan2(other_x, lookahead_d);
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

    // Debug-only: reports why select_other_lane() rejects each LABEL_OTHER_LANE
    // candidate this frame, without altering selection. Surfaces which of the
    // 4 hard gates (side/parallelism/distance/corridor) is failing when a
    // lane_change intent holds on "lane_change_target_not_detected" despite
    // other_lane_detected being true - i.e. an other-lane exists but not in
    // the geometry select_other_lane() requires.
    static json diagnose_other_lane_gates(const std::vector<LaneCandidate>& lanes,
                                          const LaneCandidate* main_lane,
                                          bool is_left_change) {
        json out = json::array();

        double main_x = 0.0;
        double main_heading = 0.0;
        if (main_lane && main_lane->raw_obj.contains("waypoints") &&
            main_lane->raw_obj["waypoints"].is_array() && !main_lane->raw_obj["waypoints"].empty()) {
            double sum_x = 0.0;
            int count = 0;
            for (const auto& pt : main_lane->raw_obj["waypoints"]) {
                sum_x += pt[0].get<double>();
                count++;
            }
            main_x = sum_x / count;
            main_heading = get_lane_heading(*main_lane);
        } else if (main_lane && main_lane->raw_obj.contains("lookahead_x_mm")) {
            main_x = main_lane->raw_obj["lookahead_x_mm"].get<double>();
            double lookahead_d = main_lane->raw_obj.value("lookahead_d_mm", 300.0);
            main_heading = std::atan2(main_x, lookahead_d);
        }

        for (const auto& l : lanes) {
            if (l.label != LABEL_OTHER_LANE) continue;

            double other_x = 0.0;
            double other_heading = 0.0;
            double min_y = 0.0;
            bool have_geom = false;

            if (l.raw_obj.contains("waypoints") && l.raw_obj["waypoints"].is_array() && !l.raw_obj["waypoints"].empty()) {
                double sum_x = 0.0;
                int count = 0;
                double local_min_y = 1e9;
                for (const auto& pt : l.raw_obj["waypoints"]) {
                    double x = pt[0].get<double>();
                    double y = pt[1].get<double>();
                    sum_x += x;
                    count++;
                    if (y < local_min_y) local_min_y = y;
                }
                if (count > 0) {
                    other_x = sum_x / count;
                    other_heading = get_lane_heading(l);
                    min_y = local_min_y;
                    have_geom = true;
                }
            } else if (l.raw_obj.contains("lookahead_x_mm") && l.raw_obj["lookahead_x_mm"].is_number() &&
                       l.raw_obj.contains("lookahead_d_mm") && l.raw_obj["lookahead_d_mm"].is_number()) {
                other_x = l.raw_obj["lookahead_x_mm"].get<double>();
                double lookahead_d = l.raw_obj["lookahead_d_mm"].get<double>();
                other_heading = std::atan2(other_x, lookahead_d);
                min_y = 0.0;
                have_geom = true;
            }

            json entry;
            entry["lane_id"] = lane_id_string(&l);
            if (!have_geom) {
                entry["reason"] = "no_geometry";
                out.push_back(entry);
                continue;
            }

            double lateral_dist = other_x - main_x;
            double diff_theta = std::abs(other_heading - main_heading);
            while (diff_theta > M_PI) diff_theta -= 2.0 * M_PI;
            while (diff_theta < -M_PI) diff_theta += 2.0 * M_PI;
            diff_theta = std::abs(diff_theta);
            double abs_lat_dist = std::abs(lateral_dist);

            bool side_pass = is_left_change ? (lateral_dist <= -200.0) : (lateral_dist >= 200.0);
            bool parallel_pass = diff_theta <= (30.0 * M_PI / 180.0);
            bool distance_pass = abs_lat_dist >= 400.0 && abs_lat_dist <= 1400.0;
            bool corridor_pass = min_y <= 1200.0;

            entry["lateral_dist_mm"] = lateral_dist;
            entry["heading_diff_deg"] = diff_theta * 180.0 / M_PI;
            entry["min_y_mm"] = min_y;
            entry["side_gate_pass"] = side_pass;
            entry["parallel_gate_pass"] = parallel_pass;
            entry["distance_gate_pass"] = distance_pass;
            entry["corridor_gate_pass"] = corridor_pass;
            entry["all_gates_pass"] = side_pass && parallel_pass && distance_pass && corridor_pass;
            out.push_back(entry);
        }

        return out;
    }

    static bool is_turn_blocked_by_solid(const ActiveTrajectory& traj, const std::vector<MarkingCandidate>& markings) {
        if (!traj.valid || traj.points.size() < 2) return false;

        auto cross = [](const Point2D& a, const Point2D& b, const Point2D& c) {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        };
        auto on_segment = [](const Point2D& a, const Point2D& b, const Point2D& p) {
            const double eps = 1e-6;
            return p.x >= std::min(a.x, b.x) - eps && p.x <= std::max(a.x, b.x) + eps &&
                   p.y >= std::min(a.y, b.y) - eps && p.y <= std::max(a.y, b.y) + eps;
        };
        auto segments_intersect = [&](const Point2D& a, const Point2D& b,
                                      const Point2D& c, const Point2D& d) {
            const double eps = 1e-6;
            double d1 = cross(a, b, c);
            double d2 = cross(a, b, d);
            double d3 = cross(c, d, a);
            double d4 = cross(c, d, b);

            if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
                ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))) {
                return true;
            }
            if (std::abs(d1) <= eps && on_segment(a, b, c)) return true;
            if (std::abs(d2) <= eps && on_segment(a, b, d)) return true;
            if (std::abs(d3) <= eps && on_segment(c, d, a)) return true;
            if (std::abs(d4) <= eps && on_segment(c, d, b)) return true;
            return false;
        };
        auto point_segment_distance = [](const Point2D& p, const Point2D& a, const Point2D& b) {
            double vx = b.x - a.x;
            double vy = b.y - a.y;
            double denom = vx * vx + vy * vy;
            if (denom < 1e-6) return std::hypot(p.x - a.x, p.y - a.y);
            double t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / denom;
            t = std::max(0.0, std::min(1.0, t));
            Point2D proj{a.x + t * vx, a.y + t * vy};
            return std::hypot(p.x - proj.x, p.y - proj.y);
        };
        auto point_in_polygon = [](const Point2D& p, const std::vector<Point2D>& poly) {
            if (poly.size() < 3) return false;
            bool inside = false;
            for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
                const Point2D& a = poly[i];
                const Point2D& b = poly[j];
                bool crosses_y = (a.y > p.y) != (b.y > p.y);
                if (crosses_y) {
                    double x_at_y = (b.x - a.x) * (p.y - a.y) / (b.y - a.y + 1e-9) + a.x;
                    if (p.x < x_at_y) inside = !inside;
                }
            }
            return inside;
        };

        auto trajectory_hits_polyline = [&](const std::vector<Point2D>& mark_pts, bool closed) {
            if (mark_pts.empty()) return false;
            if (mark_pts.size() == 1) {
                for (size_t i = 1; i < traj.points.size(); ++i) {
                    if (point_segment_distance(mark_pts.front(), traj.points[i - 1], traj.points[i]) < 100.0) {
                        return true;
                    }
                }
                return false;
            }

            for (size_t i = 1; i < traj.points.size(); ++i) {
                for (size_t j = 1; j < mark_pts.size(); ++j) {
                    if (segments_intersect(traj.points[i - 1], traj.points[i], mark_pts[j - 1], mark_pts[j])) {
                        return true;
                    }
                }
                if (closed && mark_pts.size() > 2 &&
                    segments_intersect(traj.points[i - 1], traj.points[i], mark_pts.back(), mark_pts.front())) {
                    return true;
                }
            }

            if (closed && mark_pts.size() > 2) {
                for (const auto& p : traj.points) {
                    if (point_in_polygon(p, mark_pts)) return true;
                }
            }
            return false;
        };

        for (const auto& m : markings) {
            if (m.label != LABEL_DOUBLE_SOLID_WHITE && m.label != LABEL_SOLID_WHITE && m.label != LABEL_SOLID_YELLOW) continue;

            if (m.raw_obj.contains("waypoints") && m.raw_obj["waypoints"].is_array()) {
                std::vector<Point2D> mark_pts;
                for (const auto& wp : m.raw_obj["waypoints"]) {
                    if (wp.is_array() && wp.size() >= 2) {
                        mark_pts.push_back({wp[0].get<double>(), wp[1].get<double>()});
                    }
                }
                if (trajectory_hits_polyline(mark_pts, false)) return true;
            } else if (m.raw_obj.contains("polygons_real_world") && m.raw_obj["polygons_real_world"].is_array()) {
                for (const auto& poly : m.raw_obj["polygons_real_world"]) {
                    if (!poly.is_array()) continue;
                    std::vector<Point2D> mark_pts;
                    for (const auto& pt : poly) {
                        if (pt.is_array() && pt.size() >= 2) {
                            mark_pts.push_back({pt[0].get<double>(), pt[1].get<double>()});
                        }
                    }
                    if (trajectory_hits_polyline(mark_pts, true)) return true;
                }
            } else if (m.raw_obj.contains("lookahead_x_mm")) {
                Point2D mark_pt{m.raw_obj["lookahead_x_mm"].get<double>(), 600.0};
                if (trajectory_hits_polyline({mark_pt}, false)) return true;
            }
        }
        return false;
    }

    static void get_normalized_turn_geometry(const json& turn_obj, double& long_off, double& theta_t) {
        long_off = 1e9;
        theta_t = 1e9;

        if (turn_obj.contains("longitudinal_offset_mm")) {
            long_off = turn_obj.value("longitudinal_offset_mm", 1e9);
        }

        if (turn_obj.contains("lookahead_theta_rad")) {
            theta_t = turn_obj.value("lookahead_theta_rad", 1e9);
        }

        if (turn_obj.contains("waypoints") && turn_obj["waypoints"].is_array()) {
            const auto& wps = turn_obj["waypoints"];
            std::vector<Point2D> pts;
            for (const auto& pt : wps) {
                if (pt.is_array() && pt.size() >= 2) {
                    pts.push_back({pt[0].get<double>(), pt[1].get<double>()});
                }
            }
            if (pts.size() >= 2) {
                double dist_front = pts.front().x * pts.front().x + pts.front().y * pts.front().y;
                double dist_back = pts.back().x * pts.back().x + pts.back().y * pts.back().y;
                if (dist_back < dist_front) {
                    std::reverse(pts.begin(), pts.end());
                }

                if (long_off > 9e8) {
                    long_off = pts.front().y;
                }
                if (theta_t > 9e8) {
                    double dx = pts.back().x - pts.front().x;
                    double dy = pts.back().y - pts.front().y;
                    if (std::abs(dx) > 1e-3 || std::abs(dy) > 1e-3) {
                        theta_t = std::atan2(dx, dy);
                    }
                }
            } else if (pts.size() == 1) {
                if (long_off > 9e8) {
                    long_off = pts.front().y;
                }
            }
        }
    }

    static ActiveTrajectory build_trajectory_from_candidate(const LaneCandidate& cand) {
        ActiveTrajectory traj;
        traj.source_labels = {cand.label};
        traj.trajectory_kind = (cand.label == LABEL_TURN_LANE) ? "turn_lane" : "main_lane";

        if (cand.raw_obj.contains("waypoints") && cand.raw_obj["waypoints"].is_array()) {
            for (const auto& pt : cand.raw_obj["waypoints"]) {
                if (pt.is_array() && pt.size() >= 2) {
                    traj.points.push_back({pt[0].get<double>(), pt[1].get<double>()});
                }
            }
        }

        if (cand.label != LABEL_TURN_LANE) {
            // Sort points by Y ascending to ensure sequential order forward for longitudinal lanes
            std::sort(traj.points.begin(), traj.points.end(), [](const Point2D& a, const Point2D& b) {
                return a.y < b.y;
            });
        } else {
            // For turn lanes, order by distance from the vehicle to ensure P0 is the start of the turn
            if (!traj.points.empty()) {
                double dist_front = traj.points.front().x * traj.points.front().x + traj.points.front().y * traj.points.front().y;
                double dist_back = traj.points.back().x * traj.points.back().x + traj.points.back().y * traj.points.back().y;
                if (dist_back < dist_front) {
                    std::reverse(traj.points.begin(), traj.points.end());
                }
            }
        }

        // Filter out overlapping or duplicated points for smoothing to work reliably
        if (!traj.points.empty()) {
            std::vector<Point2D> filtered;
            filtered.push_back(traj.points.front());
            for (size_t i = 1; i < traj.points.size(); ++i) {
                double dist = std::sqrt(std::pow(traj.points[i].x - filtered.back().x, 2) +
                                        std::pow(traj.points[i].y - filtered.back().y, 2));
                if (dist > 10.0) { // minimum 10mm apart
                    filtered.push_back(traj.points[i]);
                }
            }
            traj.points = std::move(filtered);
        }

        traj.valid = (traj.points.size() >= 2);
        if (!traj.valid && cand.label == LABEL_TURN_LANE && cand.raw_obj.contains("longitudinal_offset_mm")) {
            traj.has_precomputed_control = true;
            traj.precomputed_epsilon_x_mm = 0.0;
            traj.precomputed_epsilon_y_mm = cand.raw_obj["longitudinal_offset_mm"].get<double>();
            traj.precomputed_theta_rad = cand.raw_obj.value("lookahead_theta_rad", 0.0);
            traj.precomputed_curvature_inv_mm = cand.raw_obj.value("curvature_inv_mm", 0.0);
            traj.precomputed_lookahead_d_mm = cand.raw_obj.value("lookahead_d_mm", traj.precomputed_epsilon_y_mm);
            traj.trajectory_kind = "precomputed_turn_lane";
            traj.valid = true;
        } else if (!traj.valid && cand.raw_obj.contains("lookahead_x_mm") && cand.raw_obj.contains("lookahead_d_mm")) {
            traj.has_precomputed_control = true;
            traj.precomputed_epsilon_x_mm = cand.raw_obj["lookahead_x_mm"].get<double>();
            traj.precomputed_epsilon_y_mm = cand.raw_obj["lookahead_d_mm"].get<double>();
            traj.precomputed_theta_rad = cand.raw_obj.value(
                "lookahead_theta_rad",
                std::atan2(traj.precomputed_epsilon_x_mm, traj.precomputed_epsilon_y_mm)
            );
            traj.precomputed_curvature_inv_mm = cand.raw_obj.value("curvature_inv_mm", 0.0);
            traj.precomputed_lookahead_d_mm = cand.raw_obj["lookahead_d_mm"].get<double>();
            traj.trajectory_kind = (cand.label == LABEL_TURN_LANE) ? "precomputed_turn_lane" : "precomputed_main_lane";
            traj.valid = true;
        }
        return traj;
    }

    static void synthesize_precomputed_points(ActiveTrajectory& traj) {
        if (traj.valid && traj.has_precomputed_control && traj.points.empty()) {
            traj.points = {
                { 0.0, 0.0 },
                { traj.precomputed_epsilon_x_mm * 0.5, traj.precomputed_epsilon_y_mm * 0.5 },
                { traj.precomputed_epsilon_x_mm, traj.precomputed_epsilon_y_mm }
            };
        }
    }

    static json json_from_planned_trajectory(const PlannedTrajectory& t, const std::string& stage) {
        json j;
        j["stage"] = stage;
        j["valid"] = t.valid;
        j["trajectory_kind"] = trajectory_kind_name(t.trajectory_kind);
        j["confidence"] = t.confidence;
        j["normalization_mode"] = t.normalization_mode;

        json pts = json::array();
        for (const auto& p : t.points) {
            pts.push_back({p.x, p.y});
        }
        j["points"] = pts;

        j["has_precomputed_control"] = t.has_precomputed_control;
        if (t.has_precomputed_control) {
            j["precomputed_epsilon_x_mm"] = t.precomputed_epsilon_x_mm;
            j["precomputed_epsilon_y_mm"] = t.precomputed_epsilon_y_mm;
        }

        return j;
    }
};

}  // namespace avs_perception
