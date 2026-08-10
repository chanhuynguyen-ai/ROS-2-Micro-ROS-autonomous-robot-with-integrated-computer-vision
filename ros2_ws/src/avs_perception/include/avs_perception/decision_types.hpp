#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

// Plan E2: geometry sanity bounds for lane candidates. A main-lane candidate
// whose waypoints start behind the vehicle (y < 0) or reach beyond any
// plausible perception range only exists because upstream projection blew up
// (see bev_region.hpp); selection must never score such a candidate. Shared
// by TrajectoryPlanner::select_main_current and LegacyLaneModel::split_main_lanes.
inline constexpr double kMinPlausibleLaneStartYMm = 0.0;
inline constexpr double kMaxPlausibleLaneEndYMm = 10000.0;

enum class RouteIntent {
    FOLLOW_MAIN,
    TURN_RIGHT,
    TURN_LEFT,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT,
    LEGACY_TURN,
    LEGACY_LANE_CHANGE
};

static const char* route_intent_name(RouteIntent intent) {
    switch (intent) {
        case RouteIntent::FOLLOW_MAIN: return "follow_main";
        case RouteIntent::TURN_RIGHT: return "turn_right";
        case RouteIntent::TURN_LEFT: return "turn_left";
        case RouteIntent::LANE_CHANGE_LEFT: return "lane_change_left";
        case RouteIntent::LANE_CHANGE_RIGHT: return "lane_change_right";
        case RouteIntent::LEGACY_TURN: return "legacy_turn";
        case RouteIntent::LEGACY_LANE_CHANGE: return "legacy_lane_change";
    }
    return "unknown";
}

static bool is_maneuver_intent(RouteIntent intent) {
    return intent == RouteIntent::TURN_RIGHT ||
           intent == RouteIntent::TURN_LEFT ||
           intent == RouteIntent::LANE_CHANGE_LEFT ||
           intent == RouteIntent::LANE_CHANGE_RIGHT;
}

enum class DecisionState {
    FOLLOW_MAIN,
    TURN_RIGHT,
    TURN_LEFT,
    LANE_CHANGE,
    BLOCKED,
    RECOVERY
};

static const char* decision_state_name(DecisionState s) {
    switch (s) {
        case DecisionState::FOLLOW_MAIN: return "FOLLOW_MAIN";
        case DecisionState::TURN_RIGHT: return "TURN_RIGHT";
        case DecisionState::TURN_LEFT: return "TURN_LEFT";
        case DecisionState::LANE_CHANGE: return "LANE_CHANGE";
        case DecisionState::BLOCKED: return "BLOCKED";
        case DecisionState::RECOVERY: return "RECOVERY";
    }
    return "UNKNOWN";
}

static const char* legacy_lane_state_name(DecisionState s) {
    switch (s) {
        case DecisionState::FOLLOW_MAIN: return "FOLLOW_MAIN";
        case DecisionState::TURN_RIGHT:  return "TURNING";
        case DecisionState::TURN_LEFT:   return "TURNING";
        case DecisionState::LANE_CHANGE: return "LANE_CHANGE";
        case DecisionState::BLOCKED:     return "FOLLOW_MAIN";
        case DecisionState::RECOVERY:    return "FOLLOW_MAIN";
        default:                         return "FOLLOW_MAIN";
    }
}

struct LaneCandidate {
    int label = -1;
    std::string class_name;
    // waypoints, polynomial, offsets, lookahead fields, bbox/range etc.
    json raw_obj;
};

struct MarkingCandidate {
    int label = -1;
    std::string class_name;
    // polygon/waypoints/range
    json raw_obj;
};

// ─────────────────────────────────────────────────────────────────────────────
// Point and Trajectory
// ─────────────────────────────────────────────────────────────────────────────
struct Point2D {
    union {
        double x; // lateral (x_mm)
        double x_mm;
    };
    union {
        double y; // forward (y_mm)
        double y_mm;
    };
};

enum class TrajectoryKind {
    FOLLOW_MAIN,
    TURN_RIGHT,
    TURN_LEFT,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT,
    BLOCKED_FOLLOW_MAIN,
    UNKNOWN
};

static const char* trajectory_kind_name(TrajectoryKind kind) {
    switch (kind) {
        case TrajectoryKind::FOLLOW_MAIN: return "follow_main";
        case TrajectoryKind::TURN_RIGHT: return "turn_right";
        case TrajectoryKind::TURN_LEFT: return "turn_left";
        case TrajectoryKind::LANE_CHANGE_LEFT: return "lane_change_left";
        case TrajectoryKind::LANE_CHANGE_RIGHT: return "lane_change_right";
        case TrajectoryKind::BLOCKED_FOLLOW_MAIN: return "blocked_follow_main";
        case TrajectoryKind::UNKNOWN: return "unknown";
    }
    return "unknown";
}

static RouteIntent route_intent_from_trajectory_kind(TrajectoryKind kind) {
    switch (kind) {
        case TrajectoryKind::TURN_RIGHT:
            return RouteIntent::TURN_RIGHT;
        case TrajectoryKind::TURN_LEFT:
            return RouteIntent::TURN_LEFT;
        case TrajectoryKind::LANE_CHANGE_LEFT:
            return RouteIntent::LANE_CHANGE_LEFT;
        case TrajectoryKind::LANE_CHANGE_RIGHT:
            return RouteIntent::LANE_CHANGE_RIGHT;
        case TrajectoryKind::FOLLOW_MAIN:
        case TrajectoryKind::BLOCKED_FOLLOW_MAIN:
        case TrajectoryKind::UNKNOWN:
        default:
            return RouteIntent::FOLLOW_MAIN;
    }
}

static TrajectoryKind string_to_trajectory_kind(const std::string& kind_str, RouteIntent current_intent) {
    if (kind_str == "follow_main" || kind_str == "fallback_follow_main" ||
        kind_str == "pending_t_junction_follow_main" || kind_str == "main_lane" ||
        kind_str == "precomputed_main_lane" || kind_str == "follow_main_connected") {
        return TrajectoryKind::FOLLOW_MAIN;
    } else if (kind_str == "turn_right" || kind_str == "turn_right_connected" || kind_str == "turn_right_standalone") {
        return TrajectoryKind::TURN_RIGHT;
    } else if (kind_str == "turn_left" || kind_str == "turn_left_connected" || kind_str == "turn_left_standalone") {
        return TrajectoryKind::TURN_LEFT;
    } else if (kind_str == "lane_change_left") {
        return TrajectoryKind::LANE_CHANGE_LEFT;
    } else if (kind_str == "lane_change_right") {
        return TrajectoryKind::LANE_CHANGE_RIGHT;
    } else if (kind_str == "blocked_follow_main") {
        return TrajectoryKind::BLOCKED_FOLLOW_MAIN;
    } else if (kind_str == "turn_lane" || kind_str == "precomputed_turn_lane") {
        if (current_intent == RouteIntent::TURN_LEFT) return TrajectoryKind::TURN_LEFT;
        if (current_intent == RouteIntent::TURN_RIGHT) return TrajectoryKind::TURN_RIGHT;
        return TrajectoryKind::UNKNOWN;
    } else if (kind_str == "transition") {
        if (current_intent == RouteIntent::LANE_CHANGE_LEFT) return TrajectoryKind::LANE_CHANGE_LEFT;
        if (current_intent == RouteIntent::LANE_CHANGE_RIGHT) return TrajectoryKind::LANE_CHANGE_RIGHT;
        if (current_intent == RouteIntent::TURN_LEFT) return TrajectoryKind::TURN_LEFT;
        if (current_intent == RouteIntent::TURN_RIGHT) return TrajectoryKind::TURN_RIGHT;
        return TrajectoryKind::UNKNOWN;
    }
    return TrajectoryKind::UNKNOWN;
}

struct LaneObservation {
    std::string lane_id;
    std::string class_name;
    int label = -1;
    std::vector<Point2D> points;
    double confidence = 0.0;
    double heading_hint = 0.0;
    double curvature_hint = 0.0;
    bool has_precomputed_control = false;
    double precomputed_epsilon_x_mm = 0.0;
    double precomputed_epsilon_y_mm = 0.0;
    double precomputed_theta_rad = 0.0;
    double precomputed_curvature_inv_mm = 0.0;
    double precomputed_lookahead_d_mm = 0.0;
    json raw_obj;
};

struct MarkingObservation {
    std::string marking_id;
    std::string class_name;
    int label = -1;
    std::vector<Point2D> points;
    double confidence = 0.0;
    json raw_obj;
};

struct PathObservationFrame {
    std::vector<LaneObservation> lanes;
    std::vector<MarkingObservation> markings;
    uint64_t timestamp_ms = 0;
};

struct PlannedTrajectory {
    std::vector<Point2D> points;
    std::vector<std::string> source_lane_ids;
    std::string target_lane_id;
    TrajectoryKind trajectory_kind = TrajectoryKind::UNKNOWN;
    double confidence = 0.0;
    bool valid = false;
    bool blocked_by_marking = false;
    bool marking_confidence_low = false;
    // True when this plan's geometry came from a lane actually observed in the
    // current frame, false when it is a memory hold replaying an earlier one.
    // Downstream policy that would rather trust fresh perception than freeze an
    // aging path (TrajectoryManager's low-confidence hold) keys off this.
    bool from_direct_observation = false;
    std::string normalization_mode = "none";

    // Precomputed control fields
    bool has_precomputed_control = false;
    double precomputed_epsilon_x_mm = 0.0;
    double precomputed_epsilon_y_mm = 0.0;
    double precomputed_theta_rad = 0.0;
    double precomputed_curvature_inv_mm = 0.0;
    double precomputed_lookahead_d_mm = 0.0;
};

struct CommittedTrajectoryState {
    PlannedTrajectory trajectory;
    double progress_s_mm = 0.0;
    double remaining_s_mm = 0.0;
    uint64_t last_good_frame = 0;
    int dropout_hold_counter = 0;
    // Separate from dropout_hold_counter (transient-invalid dropout / maneuver-fallback
    // hold) so a low-confidence-deviation hold streak gets its own dedicated budget
    // instead of inheriting whatever count those unrelated hold reasons left behind.
    int low_conf_hold_counter = 0;
    std::string replan_reason = "none";
    RouteIntent committed_intent = RouteIntent::FOLLOW_MAIN;
    uint64_t committed_intent_seq = 0;
};

struct ActiveTrajectory {
    std::vector<Point2D> points;
    std::vector<int> source_labels;
    std::string trajectory_kind = "unknown";
    std::string normalization_mode = "none";
    double trajectory_confidence = 0.0;
    bool valid = false;
    bool has_precomputed_control = false;
    double precomputed_epsilon_x_mm = 0.0;
    double precomputed_epsilon_y_mm = 0.0;
    double precomputed_theta_rad = 0.0;
    double precomputed_curvature_inv_mm = 0.0;
    double precomputed_lookahead_d_mm = 0.0;

    std::vector<json> debug_trajectories;
};

enum class ManagerAction {
    HOLD_CURRENT,
    UPDATE_CURRENT,
    COMMIT_NEW,
    ENTER_RECOVERY,
    ENTER_BLOCKED
};

// Composite replacement for the old index-based mean abs-X deviation: aligns the two
// paths by arc-length (same technique as TrajectoryNormalizer) before comparing, and
// reports enough signal (lateral, heading, curvature, overlap, topology) for the
// manager's replan/hold/update policy to distinguish "same shape, small jitter" from
// "topology changed" or "candidate diverged from committed geometry".
struct PathDeviationMetrics {
    double lateral_rms_mm = 0.0;
    double heading_rms_rad = 0.0;
    double curvature_max_delta = 0.0;
    double overlap_ratio = 1.0;
    bool topology_changed = false;
};
