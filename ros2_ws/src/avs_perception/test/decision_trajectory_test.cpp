// gtest port of test/decision_system/test_plan_b_lane_rules.py (Plan B: lane
// selection rules). Calls the real decision/trajectory classes directly from
// the split avs_perception/decision_*/path_observation/trajectory_* headers -
// no Python mirror involved.
#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "avs_perception/label_mapping.hpp"
#include "avs_perception/decision_types.hpp"
#include "avs_perception/path_observation.hpp"
#include "avs_perception/trajectory_planner.hpp"
#include "avs_perception/trajectory_normalizer.hpp"
#include "avs_perception/trajectory_manager.hpp"
#include "avs_perception/trajectory_latch.hpp"
#include "avs_perception/ego_motion.hpp"

using namespace avs_perception;  // generated LABEL_* constants

namespace {

namespace fs = std::filesystem;

fs::path repo_root() {
    // <repo>/ros2_ws/src/avs_perception/test/decision_trajectory_test.cpp
    return fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path().parent_path();
}

json load_fixture(const std::string& name) {
    fs::path path = repo_root() / "test/decision_system/fixtures" / name;
    std::ifstream f(path);
    json data;
    f >> data;
    return data;
}

// Structural assertions in this file check text that used to live entirely in
// decision_trajectory_core.hpp but is now spread across the split
// decision_*/path_observation/trajectory_*/control_error_projector headers
// (Plan D D4). Concatenate all of them plus control_node.cpp so a header
// split never silently breaks these checks.
std::string read_control_node() {
    fs::path control_node = repo_root() / "ros2_ws/src/avs_perception/src/control_node.cpp";
    fs::path include_dir = repo_root() / "ros2_ws/src/avs_perception/include/avs_perception";
    std::ostringstream out;
    out << std::ifstream(control_node).rdbuf();
    std::vector<fs::path> core_headers;
    for (const auto& entry : fs::directory_iterator(include_dir)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("decision_", 0) == 0 || name.rfind("trajectory_", 0) == 0 ||
            name == "path_observation.hpp" || name == "control_error_projector.hpp" ||
            name == "legacy_lane_model.hpp") {
            core_headers.push_back(entry.path());
        }
    }
    std::sort(core_headers.begin(), core_headers.end());
    for (const auto& header : core_headers) {
        out << std::ifstream(header).rdbuf();
    }
    return out.str();
}

std::string extract_function_body(const std::string& source, const std::string& signature_marker) {
    size_t start = source.find(signature_marker);
    if (start == std::string::npos) {
        ADD_FAILURE() << "signature marker not found: " << signature_marker;
        return "";
    }
    size_t brace_start = source.find('{', start);
    int depth = 0;
    for (size_t i = brace_start; i < source.size(); ++i) {
        if (source[i] == '{') depth++;
        else if (source[i] == '}') {
            depth--;
            if (depth == 0) return source.substr(start, i + 1 - start);
        }
    }
    ADD_FAILURE() << "unbalanced braces extracting function at: " << signature_marker;
    return "";
}

}  // namespace

// R10: select_other_lane_obs must pick the lane nearest to main (absolute
// lateral distance), not the one closest to an assumed 800mm lane width.
TEST(PlanBLaneRules, OtherLaneNearestAbsoluteSelection) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_lane"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 0.9},
             {"waypoints", json::array({{0.0, 0.0}, {0.0, 1000.0}, {0.0, 2000.0}})}},
            {{"id", "other_near"}, {"label", 7}, {"class_name", "other-lane"}, {"confidence", 0.85},
             {"waypoints", json::array({{700.0, 0.0}, {700.0, 1000.0}, {700.0, 2000.0}})}},
            {{"id", "other_far"}, {"label", 7}, {"class_name", "other-lane"}, {"confidence", 0.85},
             {"waypoints", json::array({{900.0, 0.0}, {900.0, 1000.0}, {900.0, 2000.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    const LaneObservation* selected = TrajectoryPlanner::select_other_lane_obs(obs, &obs.lanes[0], false);
    ASSERT_NE(selected, nullptr);
    EXPECT_EQ(selected->lane_id, "other_near");
}

// R7: a solid marking outside the lane-change corridor must not block the change.
TEST(PlanBLaneRules, LaneChangeSolidOutsideCorridorDoesNotBlock) {
    json frames = load_fixture("lane_change_solid_outside_corridor.json");
    PathObservationFrame obs = PathObservationBuilder::build(frames[0]);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;

    PlannedTrajectory planned = TrajectoryPlanner::plan_lane_change_left(obs, prev_state, last_main_id);
    EXPECT_FALSE(planned.blocked_by_marking);
    EXPECT_EQ(planned.trajectory_kind, TrajectoryKind::LANE_CHANGE_LEFT);
    EXPECT_EQ(planned.target_lane_id, "other_lane");
}

// marking_confidence_low: no marking detected in corridor -> change stays
// allowed but flagged low-confidence.
TEST(PlanBLaneRules, MarkingConfidenceLowWhenNoMarkingDetected) {
    json frames = load_fixture("lane_change_no_marking_between_lanes.json");
    PathObservationFrame obs = PathObservationBuilder::build(frames[0]);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;

    PlannedTrajectory planned = TrajectoryPlanner::plan_lane_change_left(obs, prev_state, last_main_id);
    EXPECT_FALSE(planned.blocked_by_marking);
    EXPECT_EQ(planned.trajectory_kind, TrajectoryKind::LANE_CHANGE_LEFT);
    EXPECT_TRUE(planned.marking_confidence_low);
}

TEST(PlanBLaneRules, MarkingConfidenceLowFalseWhenSolidMarkingDetected) {
    json frames = load_fixture("lane_change_solid_blocked.json");
    PathObservationFrame obs = PathObservationBuilder::build(frames[0]);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;

    PlannedTrajectory planned = TrajectoryPlanner::plan_lane_change_left(obs, prev_state, last_main_id);
    EXPECT_TRUE(planned.blocked_by_marking);
    EXPECT_FALSE(planned.marking_confidence_low);
}

// R3: T-junction detection is purely geometric and must never reference stop-line.
TEST(PlanBLaneRules, TJunctionDetectionNeverReferencesStopLine) {
    std::string source = read_control_node();
    std::string body = extract_function_body(source, "bool detect_t_junction(const LaneCandidate* main_current,");
    EXPECT_EQ(body.find("STOP_LINE"), std::string::npos);
    std::string lower = body;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    EXPECT_EQ(lower.find("stop_line"), std::string::npos);
    EXPECT_NE(body.find("main_current && !main_ahead"), std::string::npos);
    EXPECT_NE(body.find("max_turn_x - min_turn_x > 2000.0"), std::string::npos);
    EXPECT_NE(body.find("std::abs(main_end_y - avg_turn_start_y) < 1500.0"), std::string::npos);
    EXPECT_NE(body.find("t_junction_counter_ >= 3"), std::string::npos);
}

// Duplicate helper-pair consistency: select_turn_lane_obs/select_turn_lane
// reimplement the same business rules (R1/R2) against two different data
// representations (LaneObservation vs LaneCandidate). Locks their gate
// thresholds together so an edit to one copy forgotten in the other fails CI.
TEST(PlanBLaneRules, TurnLaneHelperPairThresholdsMatch) {
    std::string source = read_control_node();
    std::string obs_body = extract_function_body(
        source, "static const LaneObservation* select_turn_lane_obs(const PathObservationFrame& obs,");
    std::string cand_body = extract_function_body(
        source, "const LaneCandidate* select_turn_lane(const std::vector<LaneCandidate>& lanes,");

    for (const std::string& body : {obs_body, cand_body}) {
        EXPECT_NE(body.find("avg_x < -200.0"), std::string::npos);
        EXPECT_NE(body.find("avg_x > 200.0"), std::string::npos);
        EXPECT_NE(body.find("return scored_lanes.front().second; // closest"), std::string::npos);
        EXPECT_NE(body.find("return scored_lanes.back().second;  // farthest"), std::string::npos);
        // R1/R2 near/far metric must be nearest-point distance, not average-x.
        bool has_dist_form = body.find("double dist = std::sqrt(x*x + y*y);") != std::string::npos ||
                              body.find("double dist = std::sqrt(pt.x*pt.x + pt.y*pt.y);") != std::string::npos;
        EXPECT_TRUE(has_dist_form);
        EXPECT_NE(body.find("if (dist < min_dist) min_dist = dist;"), std::string::npos);
        EXPECT_NE(body.find("scored_lanes.push_back({min_dist, l});"), std::string::npos);
    }
}

TEST(PlanBLaneRules, OtherLaneHelperPairThresholdsMatch) {
    std::string source = read_control_node();
    std::string obs_body = extract_function_body(
        source, "static const LaneObservation* select_other_lane_obs(const PathObservationFrame& obs,");
    std::string cand_body = extract_function_body(
        source, "const LaneCandidate* select_other_lane(const std::vector<LaneCandidate>& lanes,");

    for (const std::string& body : {obs_body, cand_body}) {
        EXPECT_NE(body.find("lateral_dist > -200.0"), std::string::npos);
        EXPECT_NE(body.find("lateral_dist < 200.0"), std::string::npos);
        EXPECT_NE(body.find("30.0 * M_PI / 180.0"), std::string::npos);
        EXPECT_NE(body.find("abs_lat_dist < 400.0 || abs_lat_dist > 1400.0"), std::string::npos);
        EXPECT_NE(body.find("min_y > 1200.0"), std::string::npos);
        // R10: nearest-absolute scoring, no assumed lane-width bias.
        EXPECT_NE(body.find("score = -abs_lat_dist - 1000.0 * diff_theta;"), std::string::npos);
        std::string no_comments = std::regex_replace(body, std::regex("//.*"), "");
        EXPECT_EQ(no_comments.find("800.0"), std::string::npos);
    }
}

// R1/R2: turn-lane selection must stay stable across consecutive frames with
// small position jitter, not flicker between candidates frame-to-frame.
TEST(PlanBLaneRules, TurnRightTwoLanesSameSidePicksNearerStably) {
    json frames = load_fixture("turn_right_two_lanes_same_side.json");
    ASSERT_EQ(frames.size(), 5u);
    for (const auto& telemetry : frames) {
        PathObservationFrame obs = PathObservationBuilder::build(telemetry);
        const LaneObservation* selected = TrajectoryPlanner::select_turn_lane_obs(obs, true, false);
        ASSERT_NE(selected, nullptr);
        EXPECT_EQ(selected->lane_id, "turn_lane_closer");
    }
}

TEST(PlanBLaneRules, TurnLeftTwoLanesSameSidePicksFartherStably) {
    json frames = load_fixture("turn_left_two_lanes_same_side.json");
    ASSERT_EQ(frames.size(), 5u);
    for (const auto& telemetry : frames) {
        PathObservationFrame obs = PathObservationBuilder::build(telemetry);
        const LaneObservation* selected = TrajectoryPlanner::select_turn_lane_obs(obs, false, false);
        ASSERT_NE(selected, nullptr);
        EXPECT_EQ(selected->lane_id, "turn_lane_further");
    }
}

TEST(PlanBLaneRules, TurnLaneSelectionStableUnderJitter) {
    json frames = load_fixture("turn_lane_selection_jitter.json");
    ASSERT_EQ(frames.size(), 6u);
    for (const auto& telemetry : frames) {
        PathObservationFrame obs = PathObservationBuilder::build(telemetry);
        const LaneObservation* selected = TrajectoryPlanner::select_turn_lane_obs(obs, true, false);
        ASSERT_NE(selected, nullptr);
        EXPECT_EQ(selected->lane_id, "turn_lane_closer");
    }
}

// At the junction mouth the ego main lane has slid out of the BEV window and
// the only main-lane left is the segment across the intersection. Transitioning
// out of *that* builds a turn path starting far ahead of the vehicle, so the
// controller steers toward the far side instead of into the turn. The turn must
// instead be planned from the vehicle.
TEST(PlanBLaneRules, TurnFromFarSideMainLaneAnchorsAtVehicle) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            // Main lane resumes only past the junction (starts 2.2 m ahead).
            {{"id", "main_far"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 0.9},
             {"waypoints", json::array({{0.0, 2200.0}, {0.0, 2800.0}, {0.0, 3400.0}})}},
            {{"id", "turn_r"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 0.85},
             {"waypoints", json::array({{600.0, 900.0}, {1100.0, 1000.0}, {1600.0, 1050.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);

    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned =
        TrajectoryPlanner::plan_turn_generic(obs, prev_state, true, false, last_main_id);

    ASSERT_TRUE(planned.valid);
    ASSERT_GE(planned.points.size(), 2u);
    // The path must begin next to the vehicle, not out at the far-side lane.
    EXPECT_LT(planned.points.front().y, 600.0);
    // And the far-side main lane must not be claimed as a source lane.
    for (const auto& id : planned.source_lane_ids) {
        EXPECT_EQ(id.find("main_far"), std::string::npos);
    }
}

// The same planner path with a genuine ego main lane must keep using it - the
// bridge above is a fallback, not a replacement for the normal transition.
TEST(PlanBLaneRules, TurnFromEgoMainLaneStillTransitionsFromIt) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_ego"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 0.9},
             {"waypoints", json::array({{0.0, 100.0}, {0.0, 700.0}, {0.0, 1300.0}})}},
            {{"id", "turn_r"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 0.85},
             {"waypoints", json::array({{600.0, 900.0}, {1100.0, 1000.0}, {1600.0, 1050.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);

    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned =
        TrajectoryPlanner::plan_turn_generic(obs, prev_state, true, false, last_main_id);

    ASSERT_TRUE(planned.valid);
    bool cites_ego_main = false;
    for (const auto& id : planned.source_lane_ids) {
        if (id.find("main_ego") != std::string::npos) cites_ego_main = true;
    }
    EXPECT_TRUE(cites_ego_main);
}

// R9: follow_main through an intersection must not guess/extend a far-connect
// segment beyond the current lane when main-ahead isn't visible.
TEST(PlanBLaneRules, FollowMainNoFarConnectWithoutMainAhead) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_current"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 0.9},
             {"waypoints", json::array({{0.0, 0.0}, {0.0, 400.0}, {0.0, 800.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    ASSERT_EQ(obs.lanes.size(), 1u);

    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_follow_main(obs, prev_state, last_main_id);

    EXPECT_TRUE(planned.valid);
    EXPECT_EQ(planned.target_lane_id, "main_current");
    double max_y = -1e18;
    for (const auto& p : planned.points) max_y = std::max(max_y, p.y);
    EXPECT_LE(max_y, 800.0 + 1e-6);
}

// ─── Plan E: IPM horizon clipping & far-field robustness ────────────────────
// Numbers below come from the crossroads fixture diagnosis
// (docs/plans/plan_E_ipm_horizon_and_far_field.md §1): the calibration
// homography puts the horizon at v ≈ 151–159px, and pixels above it
// back-projected to ±hundreds of meters before Plan E1 clipped them.

#include "avs_perception/bev_region.hpp"

namespace {

// config/calibration.json homography (frozen copy for the tests; the real
// file is runtime-loaded so tests must not depend on its presence).
const double kCrossroadsH[3][3] = {
    {-0.696146323238539, -0.04708639180695328, 238.6975847215506},
    {-0.03667146499054984, 0.7466169069491455, -509.15437782954604},
    {8.268867302536152e-05, -0.006619077183420509, 1.0},
};

}  // namespace

TEST(BevRegion, RejectsAboveHorizon) {
    avs_perception::BevRegion region;  // defaults: margin 10px
    // Horizon at u=320 is v ≈ 155.08 for this H.
    double v_h = avs_perception::BevRegion::horizon_v(kCrossroadsH, 320.0);
    EXPECT_NEAR(v_h, 155.08, 0.05);
    // Above the horizon and inside the margin band: rejected.
    EXPECT_FALSE(region.accepts_pixel(kCrossroadsH, 320.0, 150.0));
    EXPECT_FALSE(region.accepts_pixel(kCrossroadsH, 320.0, 160.0));
    // Safely below horizon + margin: accepted.
    EXPECT_TRUE(region.accepts_pixel(kCrossroadsH, 320.0, 170.0));
    EXPECT_TRUE(region.accepts_pixel(kCrossroadsH, 320.0, 453.0));
}

TEST(BevRegion, RejectsBehindVehicleAndFarField) {
    avs_perception::BevRegion region;  // defaults: y [0, 8000], |x| <= 4000
    EXPECT_FALSE(region.accepts_world(0.0, -500.0));     // behind the vehicle
    EXPECT_FALSE(region.accepts_world(0.0, 9000.0));     // beyond lookahead
    EXPECT_FALSE(region.accepts_world(4500.0, 500.0));   // too far lateral
    EXPECT_TRUE(region.accepts_world(100.0, 500.0));
    EXPECT_TRUE(region.accepts_world(-3999.0, 7999.0));
}

// End-to-end on the actual crossroads geometry: the far patch of main-lane_2
// (pixel v=45..152, entirely above the horizon) blew up to y=-19930mm before
// E1; the clip must delete it. The near main-lane_1 (v=287..453) must survive
// intact with all world points in the valid region.
TEST(BevRegion, CrossroadsFarPatchRejected) {
    avs_perception::BevRegion region;
    // main-lane_2 polygon from crossroads.json (all v <= 152 < horizon).
    std::vector<avs_perception::BevPoint> far_patch =
        {{321, 152}, {332, 45}, {395, 46}, {408, 152}};
    EXPECT_TRUE(region.clip_and_project(kCrossroadsH, far_patch).empty());

    // main-lane_1 polygon (v=287..453, fully below horizon): kept, valid.
    std::vector<avs_perception::BevPoint> near_patch =
        {{312, 453}, {323, 287}, {402, 287}, {416, 453}};
    auto world = region.clip_and_project(kCrossroadsH, near_patch);
    ASSERT_GE(world.size(), 4u);
    double min_y = 1e18, max_y = -1e18;
    for (const auto& p : world) {
        EXPECT_TRUE(region.accepts_world(p.x, p.y));
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    // Pre-E1 run measured main-lane_1 world y in [92, 357].
    EXPECT_NEAR(min_y, 92.0, 5.0);
    EXPECT_NEAR(max_y, 357.0, 5.0);
}

// The regression breaker found while landing E1: a sparse 4-corner polygon
// straddling the horizon (follow_main_straight has corners at v=120 above and
// v=470 below). Dropping the two above-horizon corners would leave a 2-point
// sliver and kill the lane's waypoints; edge clipping must instead keep the
// whole below-horizon part, interpolated at the horizon boundary.
TEST(BevRegion, StraddlingPolygonKeepsValidPart) {
    avs_perception::BevRegion region;
    std::vector<avs_perception::BevPoint> straddling =
        {{250, 470}, {280, 120}, {360, 120}, {390, 470}};
    auto world = region.clip_and_project(kCrossroadsH, straddling);
    ASSERT_GE(world.size(), 4u) << "valid part of the polygon was lost";
    double max_y = -1e18;
    for (const auto& p : world) {
        EXPECT_TRUE(region.accepts_world(p.x, p.y));
        max_y = std::max(max_y, p.y);
    }
    // The interpolated far edge must reach deep into the BEV (several meters),
    // not stop at the near corners.
    EXPECT_GT(max_y, 3000.0);
}

// Structural check: the waypoint hard-cap and the BEV clip must stay wired
// into ipm_transform_node.cpp (same style as the helper-pair checks above —
// the node is not directly unit-testable without ROS).
TEST(BevRegion, WaypointHardCapWiredIntoIpmNode) {
    fs::path node_path = repo_root() / "ros2_ws/src/avs_perception/src/ipm_transform_node.cpp";
    std::ostringstream buf;
    buf << std::ifstream(node_path).rdbuf();
    const std::string source = buf.str();

    EXPECT_NE(source.find("bev_region_.clip_and_project("), std::string::npos)
        << "projection loop no longer clips against BevRegion";

    // Cap must guard both smooth-waypoint regeneration loops (turn-lane x-sweep
    // and main/other-lane y-sweep).
    size_t count = 0;
    for (size_t pos = source.find("smooth_wps.size() < kMaxWaypointsPerObject");
         pos != std::string::npos;
         pos = source.find("smooth_wps.size() < kMaxWaypointsPerObject", pos + 1)) {
        count++;
    }
    EXPECT_GE(count, 4u) << "waypoint cap missing from a regeneration loop";
}

// Plan E2: a main-lane candidate whose waypoints start behind the vehicle
// (blown-up projection) must lose to the clean near lane, even though its
// huge negative start_y would win the |start_x| + 0.5*start_y score.
TEST(PlanE, SelectMainRejectsNegativeStartY) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            // Poisoned far patch: starts 19.9m behind the vehicle (real values
            // from the pre-E1 crossroads run).
            {{"id", "main_far"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{211.0, -19900.0}, {199.2, -19800.0}, {-7.6, -700.0}})}},
            {{"id", "main_near"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{18.6, 100.0}, {23.6, 200.0}, {28.7, 300.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_follow_main(obs, prev_state, last_main_id);

    EXPECT_TRUE(planned.valid);
    EXPECT_EQ(planned.target_lane_id, "main_near");
    for (const auto& p : planned.points) {
        EXPECT_GE(p.y, 0.0) << "follow_main path still contains behind-vehicle points";
    }
}

// Plan E counterfactual (§1.5): with the clean near main lane and the real
// (already valid) turn-lane_3 geometry from the crossroads fixture, a
// turn_right intent must produce a TURN_RIGHT candidate, not the follow_main
// fallback that the poisoned main-lane_2 selection used to force.
TEST(PlanE, CrossroadsTurnRightPlansTurnCandidate) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main-lane_1"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{18.6, 100.0}, {23.6, 200.0}, {28.7, 300.0}})}},
            {{"id", "turn-lane_3"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{100.0, 517.7}, {200.0, 632.6}, {300.0, 747.6}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_candidate_for_intent(
        obs, RouteIntent::TURN_RIGHT, prev_state, /*is_t=*/false,
        /*t_junction_pending=*/false, last_main_id);

    EXPECT_TRUE(planned.valid);
    EXPECT_EQ(planned.trajectory_kind, TrajectoryKind::TURN_RIGHT)
        << "turn_right fell back to follow_main";
}

TEST(PlanTransition, TurnConnectsToTransverseTurnLane) {
    // Geometry captured live on the Pi (2026-07-10): a real turn-lane runs
    // crosswise, ~63 degrees off the ego main-lane heading. The old shared
    // 40-degree plan_transition heading gate rejected it, so turn_right
    // silently fell back to follow_main and route intents never produced a
    // turn plan on the robot.
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_lane_1114"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{0.0, 100.0}, {-17.0, 200.0}})}},
            {{"id", "turn_lane_1209"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{400.0, 843.0}, {500.0, 918.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_candidate_for_intent(
        obs, RouteIntent::TURN_RIGHT, prev_state, /*is_t=*/false,
        /*t_junction_pending=*/false, last_main_id);

    EXPECT_TRUE(planned.valid);
    EXPECT_EQ(planned.trajectory_kind, TrajectoryKind::TURN_RIGHT)
        << "transverse turn-lane rejected by the heading gate";
    EXPECT_GE(planned.points.size(), 2u);
}

TEST(PlanTransition, TurnHandleWideningBulgesTowardIntersection) {
    // Regression guard for the 2026-08 turn-sharpness fix: plan_turn_generic's
    // call into plan_transition now passes kTurnBezierHandleScaleMult so the
    // connecting curve swings toward the intersection instead of cutting the
    // inside corner. Uses the same live-captured geometry as
    // TurnConnectsToTransverseTurnLane above and checks the resulting path
    // bulges past the unscaled 1/3-rule handle would have produced.
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_lane_1114"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{0.0, 100.0}, {17.0, 200.0}})}},
            {{"id", "turn_lane_1209"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{-400.0, 843.0}, {-500.0, 918.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned_raw = TrajectoryPlanner::plan_candidate_for_intent(
        obs, RouteIntent::TURN_LEFT, prev_state, /*is_t=*/false,
        /*t_junction_pending=*/false, last_main_id);
    // Mirrored to a LEFT turn 2026-08-06: the outward belly is a left-turn
    // shape now. A left turn crosses the intersection and has room to swing
    // into; a right turn takes the near corner from the right lane and has
    // none, where the same shape steered the vehicle out of the corner it was
    // entering (see TurnBulgeAsymmetry). Mirroring the fixture keeps every
    // guard here - depth, no fold-back, belly side, bounded drift - pointed at
    // the direction where the shape is still wanted, rather than weakening it.
    PlannedTrajectory planned = planned_raw;
    for (auto& q : planned.points) q.x = -q.x;

    ASSERT_TRUE(planned.valid);
    ASSERT_GE(planned.points.size(), 3u);

    // Max perpendicular distance of any path point from the chord between
    // the path's first and last point - the "bulge" the widened handle
    // produces.
    const Point2D& a = planned.points.front();
    const Point2D& b = planned.points.back();
    double chord_len = std::hypot(b.x - a.x, b.y - a.y);
    ASSERT_GT(chord_len, 1.0);
    // Signed perpendicular offset from the chord: positive = the side the
    // entry tangent leans to (outside of the turn, toward the middle of the
    // intersection), negative = the inside corner the turn lane sits on.
    double max_bulge = 0.0;
    double inside_bulge = 0.0;
    double min_x = std::numeric_limits<double>::infinity();
    for (const auto& p : planned.points) {
        double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        max_bulge = std::max(max_bulge, cross / chord_len);
        inside_bulge = std::min(inside_bulge, cross / chord_len);
        min_x = std::min(min_x, p.x);
    }

    // Measured for this fixture: ~117mm with no widening at all, ~148mm at
    // handle_scale_mult=1.5 alone, ~356mm with the current handle_scale_mult
    // =1.5 + lateral_bulge_mult=0.40 combined. Threshold sits well above the
    // handle-only figure so a regression back to relying on handle scale
    // alone fails this test, but low enough that ordinary re-tuning of
    // turn_lateral_bulge_mult (tools/turn_bulge_sweep) doesn't trip it.
    EXPECT_GT(max_bulge, 250.0);

    // The lateral bulge is a second, independent knob precisely because
    // handle_scale_mult alone folds the path back on itself (non-monotonic
    // forward progress) past ~2.5-2.6x on this fixture. The bulge knob has
    // its own fold cliff: re-swept 2026-08-04 in the flipped belly direction
    // (tools/turn_bulge_sweep), the path folds at handle=1.5/bulge=0.80 and
    // is clean at 0.75. Anything at or past that cliff is unusable no matter
    // how the shape looks on the dashboard, so this check stays even though
    // the shipped default (0.40) sits far below it.
    for (size_t i = 1; i < planned.points.size(); ++i) {
        ASSERT_GE(planned.points[i].y, planned.points[i - 1].y - 1e-6)
            << "path folded back on itself at point " << i;
    }

    // Regression guard for the bulge's perpendicular SIDE (flipped 2026-08-04
    // on user review of the on-vehicle path): the belly must sit on the side
    // the entry tangent leans to, i.e. the outside of the turn. The path holds
    // the vehicle's heading, swings out through the middle of the
    // intersection, and does its bending late. The previous rule reinforced
    // the target lane's own side instead, so the path veered toward the turn
    // the instant it left the vehicle and hugged the inside corner.
    EXPECT_GT(inside_bulge, -50.0)
        << "belly leaked onto the inside corner instead of staying outside"
        << " (inside_bulge=" << inside_bulge << ")";

    // Outside belly on a right turn means the path does drift left of the
    // vehicle before turning - that is the intended shape, not the old sign
    // bug - but the drift stays bounded (~126mm at the shipped 0.40, ~290mm
    // at 0.78). Both bounds together pin the belly's side AND its depth.
    EXPECT_LT(min_x, -50.0)
        << "path never swung outward at all (min_x=" << min_x << ")";
    EXPECT_GT(min_x, -350.0)
        << "outward swing far exceeds the tuned depth (min_x=" << min_x << ")";
}

TEST(PlanTransition, StandaloneTurnLaneAlsoBulges) {
    // Regression guard: the standalone-turn-lane branch of plan_turn_generic
    // (no main-lane observation this frame) used to hand back
    // selected_turn->points untouched - no plan_transition call, no bulge at
    // all. That branch fires right at the edge of the junction, exactly the
    // frame or two before the turn-lane itself leaves view and
    // TrajectoryLatch freezes the path, so an unbulged path there meant the
    // frozen/replayed turn inherited no bulge either. Same telemetry as
    // TurnHandleWideningBulgesTowardIntersection above but with the
    // main-lane object dropped so select_main_current returns null.
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "turn_lane_1209"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{-400.0, 843.0}, {-500.0, 918.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned_raw = TrajectoryPlanner::plan_candidate_for_intent(
        obs, RouteIntent::TURN_LEFT, prev_state, /*is_t=*/false,
        /*t_junction_pending=*/false, last_main_id);
    // Mirrored to a LEFT turn 2026-08-06 for the same reason as the test above:
    // the outward belly is now a left-turn shape. What this test uniquely
    // guards - that the standalone-turn-lane branch bulges at all rather than
    // handing back selected_turn->points untouched - is unchanged.
    PlannedTrajectory planned = planned_raw;
    for (auto& q : planned.points) q.x = -q.x;

    ASSERT_TRUE(planned.valid);
    ASSERT_GE(planned.points.size(), 3u);

    const Point2D& a = planned.points.front();
    const Point2D& b = planned.points.back();
    double chord_len = std::hypot(b.x - a.x, b.y - a.y);
    ASSERT_GT(chord_len, 1.0);
    double max_bulge = 0.0;
    double inside_bulge = 0.0;
    double min_x = std::numeric_limits<double>::infinity();
    for (const auto& p : planned.points) {
        double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        max_bulge = std::max(max_bulge, cross / chord_len);
        inside_bulge = std::min(inside_bulge, cross / chord_len);
        min_x = std::min(min_x, p.x);
    }
    EXPECT_GT(max_bulge, 100.0)
        << "standalone-turn-lane branch should bulge like the paired branch, not hand back a raw straight path";

    for (size_t i = 1; i < planned.points.size(); ++i) {
        ASSERT_GE(planned.points[i].y, planned.points[i - 1].y - 1e-6)
            << "path folded back on itself at point " << i;
    }

    // Same belly-side guard as TurnHandleWideningBulgesTowardIntersection
    // above: the belly belongs on the outside of the turn (entry-tangent
    // side), not on the inside corner. The ego_stub anchor here has no
    // lateral offset at all (entry x=0 exactly), so the outward swing is
    // entirely the connector's doing, not pre-existing lane geometry.
    EXPECT_GT(inside_bulge, -50.0)
        << "belly leaked onto the inside corner instead of staying outside"
        << " (inside_bulge=" << inside_bulge << ")";
    EXPECT_LT(min_x, -20.0)
        << "path never swung outward at all (min_x=" << min_x << ")";
    EXPECT_GT(min_x, -350.0)
        << "outward swing far exceeds the tuned depth (min_x=" << min_x << ")";
}

// Regression guard for the 2026-08 "path ignores main-lane direction" fix:
// when the main lane IS observed this frame but starts far from the vehicle
// (past kBridgeMinLaneStartYMm, i.e. resuming across an intersection gap),
// plan_turn_generic used to anchor the connector with a straight
// dead-ahead ego_stub, discarding the far lane's real heading entirely. It
// should instead anchor at the vehicle but keep that lane's own heading.
TEST(PlanTransition, TurnAnchorFollowsFarMainLaneHeading) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            // Starts at y=700 (> kBridgeMinLaneStartYMm=600) so this is the
            // "far side" case, not the near-vehicle case. Heading is a clean
            // ~26.57 degrees off dead-ahead (dx=50 over dy=100).
            {{"id", "main_lane_far"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{700.0, 700.0}, {750.0, 800.0}})}},
            {{"id", "turn_lane_1209"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{400.0, 843.0}, {500.0, 918.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_candidate_for_intent(
        obs, RouteIntent::TURN_RIGHT, prev_state, /*is_t=*/false,
        /*t_junction_pending=*/false, last_main_id);

    ASSERT_TRUE(planned.valid);
    ASSERT_EQ(planned.trajectory_kind, TrajectoryKind::TURN_RIGHT);
    ASSERT_GE(planned.points.size(), 3u);

    // The path must still touch the vehicle...
    EXPECT_NEAR(planned.points.front().x, 0.0, 1e-6);
    EXPECT_NEAR(planned.points.front().y, 0.0, 1e-6);

    // ...and its very first leg must head off along the far main lane's own
    // ~26.57 degree heading (tan ~= 0.5), not straight up dead-ahead (which
    // would put x2 at ~0 regardless of y2).
    const Point2D& p1 = planned.points[1];
    ASSERT_GT(p1.y, 1.0);
    double initial_slope = p1.x / p1.y;
    EXPECT_NEAR(initial_slope, 0.5, 0.05)
        << "connector anchor ignored the far main lane's heading (p1=" << p1.x << "," << p1.y << ")";
}

// Regression guard, standalone branch: with no main-lane observation at all
// this frame, the ego anchor's heading should come from the last committed
// trajectory (the best available memory of the true lane direction) rather
// than always assuming dead-ahead.
TEST(PlanTransition, StandaloneTurnAnchorFollowsCommittedHeading) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "turn_lane_1209"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{400.0, 843.0}, {500.0, 918.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    prev_state.trajectory.valid = true;
    prev_state.trajectory.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    // Same ~26.57 degree heading as the far-main-lane fixture above.
    prev_state.trajectory.points = {{0.0, 0.0}, {50.0, 100.0}};
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_candidate_for_intent(
        obs, RouteIntent::TURN_RIGHT, prev_state, /*is_t=*/false,
        /*t_junction_pending=*/false, last_main_id);

    ASSERT_TRUE(planned.valid);
    ASSERT_GE(planned.points.size(), 3u);

    EXPECT_NEAR(planned.points.front().x, 0.0, 1e-6);
    EXPECT_NEAR(planned.points.front().y, 0.0, 1e-6);

    const Point2D& p1 = planned.points[1];
    ASSERT_GT(p1.y, 1.0);
    double initial_slope = p1.x / p1.y;
    EXPECT_NEAR(initial_slope, 0.5, 0.05)
        << "standalone anchor ignored the last committed trajectory's heading (p1=" << p1.x << "," << p1.y << ")";
}

// Regression guard: even when plan_transition's Bezier connector rejects the
// geometry outright (empty result), the standalone branch must still hand
// back a path anchored at the vehicle instead of the target lane's raw,
// disconnected points - this is the direct fix for the "cyan path floats
// near the turn lane, never reaching the car" symptom.
TEST(PlanTransition, StandaloneTurnBridgesToVehicleWhenTransitionRejected) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            // Heading here is ~135 degrees off dead-ahead (dx=100 over
            // dy=-100), well past kTurnMaxHeadingDiffRad (110 degrees), so
            // plan_transition must reject it (return {}).
            {{"id", "turn_lane_reverse"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{100.0, 50.0}, {200.0, -50.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_candidate_for_intent(
        obs, RouteIntent::TURN_RIGHT, prev_state, /*is_t=*/false,
        /*t_junction_pending=*/false, last_main_id);

    ASSERT_TRUE(planned.valid);
    ASSERT_GE(planned.points.size(), 2u);
    EXPECT_NEAR(planned.points.front().x, 0.0, 1e-6)
        << "rejected transition left the path disconnected from the vehicle";
    EXPECT_NEAR(planned.points.front().y, 0.0, 1e-6)
        << "rejected transition left the path disconnected from the vehicle";
}

TEST(PlanTransition, LaneChangeStillRejectsDivergentHeading) {
    // The tight 40-degree gate must keep protecting lane changes: an
    // other-lane whose heading diverges ~63 degrees from the ego lane is a
    // garbage match, not a parallel lane to merge into.
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_lane"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{0.0, 100.0}, {-17.0, 200.0}, {-30.0, 300.0}})}},
            {{"id", "other_lane"}, {"label", 7}, {"class_name", "other-lane"}, {"confidence", 1.0},
             {"waypoints", json::array({{-700.0, 100.0}, {-600.0, 175.0}, {-500.0, 250.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_candidate_for_intent(
        obs, RouteIntent::LANE_CHANGE_LEFT, prev_state, /*is_t=*/false,
        /*t_junction_pending=*/false, last_main_id);

    EXPECT_NE(planned.trajectory_kind, TrajectoryKind::LANE_CHANGE_LEFT)
        << "divergent-heading other-lane must not be accepted for lane change";
}

// ─── Intersection gap traversal: vehicle-anchored bridge ────────────────────
// Mid-crossroads the ego main-lane falls behind the BEV window (y_min = 0) and
// the only observation left is the main-lane resuming across the gap, starting
// 1-3m ahead of the vehicle. Unbridged, that candidate floats ahead of the
// vehicle: its overlap against the stale committed path collapses, its
// short-segment geometric confidence sits under replan_min_confidence, and
// after low_conf_hold_frames the manager clears to RECOVERY - the "path lost
// in the middle of the intersection" failure observed on the robot.

namespace {

json gap_traversal_frame(const std::vector<double>& near_ys, double far_start_y) {
    json objs = json::array();
    if (!near_ys.empty()) {
        json wps = json::array();
        for (double y : near_ys) wps.push_back(json::array({0.0, y}));
        objs.push_back({{"id", "main_near"}, {"label", 6}, {"class_name", "main-lane"},
                        {"waypoints", wps}});
    }
    json far_wps = json::array();
    for (int i = 0; i < 5; ++i) far_wps.push_back(json::array({0.0, far_start_y + 600.0 * i}));
    objs.push_back({{"id", "main_far"}, {"label", 6}, {"class_name", "main-lane"},
                    {"waypoints", far_wps}});
    // No explicit "confidence" key: the builder's geometric confidence applies,
    // exactly like real IPM output for a far short segment.
    return json{{"timestamp_ms", 1000}, {"objects", objs}};
}

}  // namespace

TEST(FollowMainBridge, FarLaneOnlyAnchorsPathAtVehicle) {
    PathObservationFrame obs = PathObservationBuilder::build(gap_traversal_frame({}, 1800.0));
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_follow_main(obs, prev_state, last_main_id);

    ASSERT_TRUE(planned.valid);
    ASSERT_GE(planned.points.size(), 2u);
    EXPECT_NEAR(planned.points.front().x, 0.0, 1.0);
    EXPECT_NEAR(planned.points.front().y, 0.0, 1.0);
    EXPECT_GE(planned.points.back().y, 4000.0);
    // The anchored path must clear the manager's replan_min_confidence (0.5)
    // gate, or it can never replace a stale committed path.
    EXPECT_GE(planned.confidence, 0.5);
}

// Largest distance from `p` to the polyline through `poly`.
static double distance_to_polyline(const Point2D& p, const std::vector<Point2D>& poly) {
    double best = std::numeric_limits<double>::max();
    for (size_t i = 1; i < poly.size(); ++i) {
        const Point2D& a = poly[i - 1];
        const Point2D& b = poly[i];
        double dx = b.x - a.x, dy = b.y - a.y;
        double len_sq = dx * dx + dy * dy;
        double t = (len_sq > 1e-9) ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        best = std::min(best, std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy)));
    }
    return best;
}

// 2026-08-04: the path IS the observed lane centreline. Anchoring it at the
// vehicle runs it through plan_transition, which discards the first ~1200mm of
// the lane's own waypoints and substitutes Bezier samples shaped by the
// vehicle's heading - so the stretch the controller steers on became a
// synthetic curve cutting across the centre, moving with the vehicle's offset.
// Only the mid-intersection gap (lane starting past kBridgeMinLaneStartYMm)
// still bridges; see FollowMainBridge.FarLaneOnlyAnchorsPathAtVehicle.
TEST(FollowMainBridge, NearLaneFollowsItsWaypointsExactly) {
    // A curving lane starting right at the vehicle - a Bezier bridge would cut
    // this curve, a centreline-following path will not.
    std::vector<Point2D> lane = {{0.0, 100.0}, {60.0, 600.0}, {240.0, 1100.0}, {540.0, 1600.0}};
    json waypoints = json::array();
    for (const auto& p : lane) waypoints.push_back(json::array({p.x, p.y}));

    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_near"}, {"label", 6}, {"class_name", "main-lane"},
             {"waypoints", waypoints}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_follow_main(obs, prev_state, last_main_id);

    ASSERT_TRUE(planned.valid);
    EXPECT_NEAR(planned.points.front().x, lane.front().x, 1.0) << "path must start at the first waypoint";
    EXPECT_NEAR(planned.points.front().y, lane.front().y, 1.0);
    for (const auto& p : planned.points) {
        EXPECT_LT(distance_to_polyline(p, lane), 1.0)
            << "every path point must lie on the observed centreline";
    }
}

// The vehicle has yawed hard off the lane, so the observed centreline sits off
// to the side and runs away from the vehicle's heading. The path must still be
// that centreline - the offset is the controller's job (epsilon_x), not a
// reason to reshape the path.
TEST(FollowMainBridge, YawedVehicleStillGetsTheUnalteredCentreline) {
    std::vector<Point2D> lane = {{400.0, 300.0}, {900.0, 800.0}, {1400.0, 1300.0}};
    json waypoints = json::array();
    for (const auto& p : lane) waypoints.push_back(json::array({p.x, p.y}));

    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_yawed"}, {"label", 6}, {"class_name", "main-lane"},
             {"waypoints", waypoints}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_follow_main(obs, prev_state, last_main_id);

    ASSERT_TRUE(planned.valid);
    ASSERT_GE(planned.points.size(), 2u);
    EXPECT_NEAR(planned.points.front().x, lane.front().x, 1.0)
        << "path must start at the lane's own first waypoint, not be dragged to the vehicle";
    EXPECT_NEAR(planned.points.front().y, lane.front().y, 1.0);
    for (const auto& p : planned.points) {
        EXPECT_LT(distance_to_polyline(p, lane), 1.0)
            << "vehicle offset must not bend the published centreline";
    }
}

// 2026-08-04: a momentary main-lane dropout (no lane observation at all this
// frame - common right when the vehicle steers hard) used to replay
// prev_state's points verbatim, frozen in the OLD vehicle-frame. As the
// vehicle kept moving that read as a straight line running off the actual
// (now-curved) lane. The held path must be re-anchored at the vehicle each
// frame, same as the intersection-gap bridge above.
TEST(FollowMainBridge, MainLaneDropoutReanchorsHeldPathAtVehicle) {
    PathObservationFrame empty_obs = PathObservationBuilder::build(
        json{{"timestamp_ms", 1000}, {"objects", json::array()}});

    CommittedTrajectoryState prev_state;
    prev_state.trajectory.valid = true;
    prev_state.trajectory.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    prev_state.trajectory.confidence = 0.9;
    // A path curving away from the vehicle's current heading, as if captured
    // a moment ago before the vehicle turned - the stale near-vehicle start
    // no longer matches where the vehicle actually is/points now.
    prev_state.trajectory.points = {{300.0, 100.0}, {600.0, 500.0}, {900.0, 1000.0}};

    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_follow_main(empty_obs, prev_state, last_main_id);

    ASSERT_TRUE(planned.valid);
    ASSERT_GE(planned.points.size(), 2u);
    EXPECT_NEAR(planned.points.front().x, 0.0, 1.0)
        << "held path must start back at the vehicle, not the stale old path's start";
    EXPECT_NEAR(planned.points.front().y, 0.0, 1.0);
}

TEST(FollowMainBridge, IntersectionGapTraversalKeepsPath) {
    // Straight crossing, gap wider than select_main_ahead's 2000mm merge gate.
    // Frames 1-3: ego lane shrinks as the vehicle reaches the intersection edge;
    // frames 4-9: only the far lane, approaching as the vehicle traverses.
    std::vector<json> frames = {
        gap_traversal_frame({100, 600, 1100, 1600, 2000}, 4200.0),
        gap_traversal_frame({100, 600, 1100, 1600}, 3800.0),
        gap_traversal_frame({100, 550, 1000}, 3200.0),
        gap_traversal_frame({}, 2400.0),
        gap_traversal_frame({}, 1800.0),
        gap_traversal_frame({}, 1200.0),
        gap_traversal_frame({}, 700.0),
        gap_traversal_frame({}, 300.0),
        gap_traversal_frame({}, 100.0),
    };

    CommittedTrajectoryState committed;
    std::string last_main_id;
    int consecutive_invalid = 0;
    uint64_t frame_no = 0;

    for (size_t fi = 0; fi < frames.size(); ++fi) {
        PathObservationFrame obs = PathObservationBuilder::build(frames[fi]);
        PlannedTrajectory cand = TrajectoryPlanner::plan_follow_main(obs, committed, last_main_id);
        PlannedTrajectory normalized = TrajectoryNormalizer::normalize(cand, committed);
        TrajectoryManager::Decision decision = TrajectoryManager::update(
            normalized, committed, RouteIntent::FOLLOW_MAIN, 0,
            /*maneuver_dropout_hold_frames=*/5, consecutive_invalid, ++frame_no);
        committed = decision.next_state;

        EXPECT_NE(decision.action, ManagerAction::ENTER_RECOVERY) << "frame " << fi + 1;
        ASSERT_TRUE(committed.trajectory.valid) << "frame " << fi + 1;
        if (fi >= 3) {
            ASSERT_GE(committed.trajectory.points.size(), 2u) << "frame " << fi + 1;
            // Anchored at the vehicle, not floating across the gap...
            EXPECT_LE(committed.trajectory.points.front().y, 600.0) << "frame " << fi + 1;
            // ...and actually reaching the far lane, not a stale pre-gap stub.
            EXPECT_GE(committed.trajectory.points.back().y, 2000.0) << "frame " << fi + 1;
        }
    }
}

// Safety net approved alongside the bridge: hold windows widened 5 -> 10 so a
// flickering far lane gets ~0.7s of grace at 14 FPS before recovery. Pins the
// declared ROS parameter defaults in control_node.cpp.
TEST(FollowMainBridge, HoldWindowDefaultsWidened) {
    std::string source = read_control_node();
    EXPECT_NE(source.find("declare_parameter<int>(\"maneuver_dropout_hold_frames\", 10)"),
              std::string::npos);
    EXPECT_NE(source.find("declare_parameter<int>(\"low_conf_hold_frames\", 10)"),
              std::string::npos);
}

// ─── 2026-07-15 control-error stability fixes ───────────────────────────────
// Live diagnosis on the robot: dashboard route intents carry seq >= 1 while the
// manager stores committed FOLLOW_MAIN with seq 0, so every frame looked like
// an intent change (COMMIT_NEW "intent_change" replan storm, hysteresis dead),
// and long memory-holds decayed confidence into denormals (1e-323 observed).

// Memory-hold confidence decay is floored so holding for minutes can't
// underflow to denormals.
TEST(FollowMainHold, ConfidenceDecayFloored) {
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array()},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);

    CommittedTrajectoryState prev_state;
    prev_state.trajectory.valid = true;
    prev_state.trajectory.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    prev_state.trajectory.points = {{0.0, 100.0}, {0.0, 600.0}, {0.0, 1100.0}};
    prev_state.trajectory.target_lane_id = "main_lane_1";
    prev_state.trajectory.confidence = 1e-300;

    std::string last_main_id = "main_lane_1";
    PlannedTrajectory planned = TrajectoryPlanner::plan_follow_main(obs, prev_state, last_main_id);

    ASSERT_TRUE(planned.valid);
    EXPECT_GE(planned.confidence, 0.05);
}

// Dashboard bug: when the fresh candidate sees less far than the previously
// committed path (lane view shrank this frame - occlusion, range-dependent
// IPM noise, etc.), TrajectoryNormalizer::normalize used to splice the
// leftover, unblended tail of the *previous* path onto the result. That tail
// was geometry from an earlier frame no longer supported by current
// perception, so the committed - and therefore control-facing - path grew a
// stale segment projecting past where the lane is actually observed
// (reported via the dashboard's active_trajectory_points overshooting the
// main-lane polygon at the far end). The committed path must instead shrink
// to the candidate's own extent.
TEST(TrajectoryNormalizerBlend, ShrinkingCandidateDropsStalePrevTail) {
    CommittedTrajectoryState prev_state;
    prev_state.trajectory.valid = true;
    prev_state.trajectory.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    prev_state.trajectory.confidence = 0.8;
    prev_state.trajectory.points = {{0.0, 0.0}, {0.0, 3000.0}};

    PlannedTrajectory cand;
    cand.valid = true;
    cand.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    cand.confidence = 0.8;
    cand.points = TrajectoryPlanner::resample_path({{0.0, 0.0}, {0.0, 1000.0}}, 100.0);

    PlannedTrajectory normalized = TrajectoryNormalizer::normalize(cand, prev_state);

    ASSERT_TRUE(normalized.valid);
    EXPECT_EQ(normalized.normalization_mode, "temporal_blend");
    EXPECT_LE(normalized.points.back().y, 1000.0 + 1.0)
        << "must not carry the previous path's unblended 1000-3000mm tail forward";
}

// 2026-08-04: steering hard makes the fresh follow_main candidate deviate a
// lot from what was committed (the vehicle frame rotated) while the lane
// segment still in view is short and curved, so its geometry-derived
// confidence falls under replan_min_confidence. The low-confidence branch then
// froze the old, now-off-lane path for low_conf_hold_frames - exactly when the
// vehicle needed the current one. A lane the camera is looking straight at is
// not the noisy guess that gate was built for.
TEST(TrajectoryManagerHold, FreshFollowMainObservationIsNotHeldOnLowConfidence) {
    CommittedTrajectoryState prev_state;
    prev_state.trajectory.valid = true;
    prev_state.trajectory.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    prev_state.trajectory.confidence = 0.9;
    prev_state.trajectory.points = TrajectoryPlanner::resample_path({{0.0, 0.0}, {0.0, 3000.0}}, 100.0);
    prev_state.remaining_s_mm = 3000.0;

    // Fresh observation, far enough off the committed path to trip replan_needed,
    // with the low confidence a short curved segment gets.
    PlannedTrajectory cand;
    cand.valid = true;
    cand.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    cand.confidence = 0.2;
    cand.from_direct_observation = true;
    cand.points = TrajectoryPlanner::resample_path({{0.0, 0.0}, {1500.0, 3000.0}}, 100.0);

    int consecutive_invalid = 0;
    TrajectoryManager::Decision d = TrajectoryManager::update(
        cand, prev_state, RouteIntent::FOLLOW_MAIN, 0, /*maneuver_dropout_hold_frames=*/5,
        consecutive_invalid, /*current_frame=*/1);

    EXPECT_NE(d.next_state.replan_reason, "low_confidence_deviation_hold")
        << "a directly observed lane must not be held behind the stale path";
    EXPECT_EQ(d.action, ManagerAction::COMMIT_NEW);
    ASSERT_FALSE(d.next_state.trajectory.points.empty());
    EXPECT_GT(d.next_state.trajectory.points.back().x, 1000.0)
        << "committed path must be the fresh observation";
}

// The same gate must still protect everything it was built for: a low-confidence
// candidate that is NOT a direct follow_main observation still gets held.
TEST(TrajectoryManagerHold, LowConfidenceHoldStillAppliesWithoutDirectObservation) {
    CommittedTrajectoryState prev_state;
    prev_state.trajectory.valid = true;
    prev_state.trajectory.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    prev_state.trajectory.confidence = 0.9;
    prev_state.trajectory.points = TrajectoryPlanner::resample_path({{0.0, 0.0}, {0.0, 3000.0}}, 100.0);
    prev_state.remaining_s_mm = 3000.0;

    PlannedTrajectory cand;
    cand.valid = true;
    cand.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    cand.confidence = 0.2;
    cand.from_direct_observation = false;  // memory hold, not a fresh lane
    cand.points = TrajectoryPlanner::resample_path({{0.0, 0.0}, {1500.0, 3000.0}}, 100.0);

    int consecutive_invalid = 0;
    TrajectoryManager::Decision d = TrajectoryManager::update(
        cand, prev_state, RouteIntent::FOLLOW_MAIN, 0, 5, consecutive_invalid, 1);

    EXPECT_EQ(d.action, ManagerAction::HOLD_CURRENT);
    EXPECT_EQ(d.next_state.replan_reason, "low_confidence_deviation_hold");
}

// 2026-08-04: the blend mixes raw coordinates from two different vehicle
// frames - project_point_to_path only recovers travel ALONG the path, never
// the frame's rotation. While the vehicle steers hard those frames diverge,
// and at the old near-vehicle weight (w_prev ~= 0.8) the stale geometry
// dominated the stretch Pure Pursuit reads, dragging the path off the lane.
// FOLLOW_MAIN now publishes the observed waypoints as they are.
TEST(TrajectoryNormalizerBlend, FollowMainPublishesObservedWaypointsUnblended) {
    CommittedTrajectoryState prev_state;
    prev_state.trajectory.valid = true;
    prev_state.trajectory.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    prev_state.trajectory.confidence = 1.0;
    // Stale path from before the vehicle turned: straight ahead.
    prev_state.trajectory.points = TrajectoryPlanner::resample_path({{0.0, 0.0}, {0.0, 3000.0}}, 100.0);

    PlannedTrajectory cand;
    cand.valid = true;
    cand.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    cand.confidence = 1.0;
    cand.from_direct_observation = true;
    // Fresh observation after the vehicle swung: the lane now runs off to the
    // side in the current vehicle frame.
    cand.points = TrajectoryPlanner::resample_path({{0.0, 0.0}, {900.0, 3000.0}}, 100.0);

    PlannedTrajectory normalized = TrajectoryNormalizer::normalize(cand, prev_state);

    ASSERT_TRUE(normalized.valid);
    ASSERT_EQ(normalized.points.size(), cand.points.size());
    for (size_t i = 0; i < cand.points.size(); ++i) {
        EXPECT_NEAR(normalized.points[i].x, cand.points[i].x, 1e-6)
            << "point " << i << " must be the observed waypoint, not a blend toward the stale path";
        EXPECT_NEAR(normalized.points[i].y, cand.points[i].y, 1e-6);
    }
}

// The knob is still there for the noise/lag tradeoff, so verify it actually
// takes effect rather than being dead configuration.
TEST(TrajectoryNormalizerBlend, FollowMainBlendWeightKnobStillApplies) {
    CommittedTrajectoryState prev_state;
    prev_state.trajectory.valid = true;
    prev_state.trajectory.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    prev_state.trajectory.confidence = 1.0;
    prev_state.trajectory.points = TrajectoryPlanner::resample_path({{0.0, 0.0}, {0.0, 3000.0}}, 100.0);

    PlannedTrajectory cand;
    cand.valid = true;
    cand.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
    cand.confidence = 1.0;
    cand.points = TrajectoryPlanner::resample_path({{200.0, 0.0}, {200.0, 3000.0}}, 100.0);

    double saved = TrajectoryNormalizer::follow_main_blend_prev_weight;
    TrajectoryNormalizer::follow_main_blend_prev_weight = 0.5;
    PlannedTrajectory normalized = TrajectoryNormalizer::normalize(cand, prev_state);
    TrajectoryNormalizer::follow_main_blend_prev_weight = saved;

    ASSERT_TRUE(normalized.valid);
    ASSERT_GE(normalized.points.size(), 30u);
    // Uniform 50/50, near the vehicle and far along alike - no distance ramp.
    EXPECT_NEAR(normalized.points.front().x, 100.0, 1.0);
    EXPECT_NEAR(normalized.points[29].x, 100.0, 1.0);
}

// 2026-08-04: turn connectors are short enough that the distance-ramped
// blend above never settles - by the far end of a ~1-2m connector w_cur is
// already well past its near-vehicle floor, so the shape still wobbles
// frame to frame. Turn kinds use a fixed turn_blend_prev_weight instead,
// applied uniformly along the whole path (no ramp toward the far end).
TEST(TrajectoryNormalizerBlend, TurnKindUsesFixedWeightInsteadOfDistanceRamp) {
    CommittedTrajectoryState prev_state;
    prev_state.trajectory.valid = true;
    prev_state.trajectory.trajectory_kind = TrajectoryKind::TURN_RIGHT;
    prev_state.trajectory.confidence = 1.0;
    prev_state.trajectory.points = {{0.0, 0.0}, {0.0, 3000.0}};

    PlannedTrajectory cand;
    cand.valid = true;
    cand.trajectory_kind = TrajectoryKind::TURN_RIGHT;
    cand.confidence = 1.0;
    // Parallel line offset by a constant 200mm from prev, so any point-wise
    // weight change (ramped vs. fixed) shows up directly as a change in x.
    cand.points = TrajectoryPlanner::resample_path({{200.0, 0.0}, {200.0, 3000.0}}, 100.0);

    double saved_weight = TrajectoryNormalizer::turn_blend_prev_weight;
    TrajectoryNormalizer::turn_blend_prev_weight = 0.75;
    PlannedTrajectory normalized = TrajectoryNormalizer::normalize(cand, prev_state);
    TrajectoryNormalizer::turn_blend_prev_weight = saved_weight;

    ASSERT_TRUE(normalized.valid);
    ASSERT_GE(normalized.points.size(), 30u);
    // w_cur = 0.25 everywhere: blended.x = prev.x + 0.25 * (cand.x - prev.x).
    EXPECT_NEAR(normalized.points.front().x, 50.0, 1.0)
        << "near-vehicle point should already sit at the fixed 25% weight";
    EXPECT_NEAR(normalized.points[29].x, 50.0, 1.0)
        << "far end (~2.9m in) must stay at the same 25% weight, not ramp toward the candidate";
}

// Pins the call-site seq normalization in control_node.cpp: FOLLOW_MAIN must
// always reach the manager with seq 0, matching the manager's own
// committed_intent_seq normalization.
TEST(FollowMainSeq, ManagerSeqNormalizedForFollowMain) {
    std::string source = read_control_node();
    size_t guard = source.find("if (manager_intent == RouteIntent::FOLLOW_MAIN)");
    ASSERT_NE(guard, std::string::npos);
    EXPECT_NE(source.find("manager_intent_seq = 0;", guard), std::string::npos);
}

// Pins the direct-IPM bypass span guard in control_node.cpp: poly(lookahead_d)
// is only trusted when the lane's observed waypoint span covers the lookahead
// distance (mid-gap extrapolation produced 13-15m epsilon_x on real video).
TEST(DirectIpm, BypassRequiresLookaheadWithinWaypointSpan) {
    std::string source = read_control_node();
    EXPECT_NE(source.find("direct_lookahead_within_span(*main_current)"), std::string::npos);
    std::string body = extract_function_body(source, "static bool direct_lookahead_within_span");
    EXPECT_NE(body.find("d_la >= y_first && d_la <= y_last"), std::string::npos);
}

// ─────────────────────────────────────────────────────────────────────────────
// Plan F: solid-yellow legality gate (docs/plans/plan_F_solid_yellow_legality_gate.md)
// ─────────────────────────────────────────────────────────────────────────────
#include "avs_perception/lane_legality.hpp"

namespace {

json lane_obj(const std::string& id, int label, const std::string& cls,
              std::initializer_list<std::pair<double, double>> wps) {
    json waypoints = json::array();
    for (const auto& [x, y] : wps) waypoints.push_back({x, y});
    return {{"id", id}, {"label", label}, {"class_name", cls}, {"confidence", 0.9},
            {"waypoints", waypoints}};
}

json yellow_obj(const std::string& id, std::initializer_list<std::pair<double, double>> wps) {
    return lane_obj(id, LABEL_SOLID_YELLOW, "solid-yellow", wps);
}

json frame_of(std::initializer_list<json> objects) {
    json arr = json::array();
    for (const auto& obj : objects) arr.push_back(obj);
    return {{"timestamp_ms", 1000}, {"objects", arr}};
}

LaneLegality verdict_of(const LaneLegalityReport& report, const std::string& id) {
    auto it = report.lane_verdicts.find(id);
    return it == report.lane_verdicts.end() ? LaneLegality::UNKNOWN : it->second;
}

}  // namespace

// §7.1: longitudinal yellow -> lane on the right LEGAL, lane on the left ILLEGAL.
TEST(PlanFLegality, VerticalYellowRightLegalLeftIllegal) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("right", LABEL_MAIN_LANE, "main-lane", {{400.0, 200.0}, {400.0, 1200.0}, {400.0, 2200.0}}),
        lane_obj("left", LABEL_OTHER_LANE, "other-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_TRUE(report.yellow_visible);
    EXPECT_EQ(verdict_of(report, "right"), LaneLegality::LEGAL);
    EXPECT_EQ(verdict_of(report, "left"), LaneLegality::ILLEGAL);
}

// §7.2: transverse yellow -> lane below LEGAL, lane above ILLEGAL.
TEST(PlanFLegality, HorizontalYellowBelowLegalAboveIllegal) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{-1500.0, 2000.0}, {0.0, 2000.0}, {1500.0, 2000.0}}),
        lane_obj("below", LABEL_MAIN_LANE, "main-lane", {{300.0, 500.0}, {300.0, 1000.0}, {300.0, 1500.0}}),
        lane_obj("above", LABEL_OTHER_LANE, "other-lane", {{300.0, 2500.0}, {300.0, 3000.0}, {300.0, 3500.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_EQ(verdict_of(report, "below"), LaneLegality::LEGAL);
    EXPECT_EQ(verdict_of(report, "above"), LaneLegality::ILLEGAL);
}

// §7.3: 45-degree yellow (outside the beta band) -> oriented away from the
// vehicle, right side of travel direction is legal.
TEST(PlanFLegality, DiagonalYellowCorrectSides) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {1000.0, 1000.0}, {2000.0, 2000.0}}),
        lane_obj("right_below", LABEL_MAIN_LANE, "main-lane", {{1000.0, 200.0}, {1400.0, 600.0}, {1800.0, 1000.0}}),
        lane_obj("left_above", LABEL_OTHER_LANE, "other-lane", {{200.0, 1000.0}, {600.0, 1400.0}, {1000.0, 1800.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_EQ(verdict_of(report, "right_below"), LaneLegality::LEGAL);
    EXPECT_EQ(verdict_of(report, "left_above"), LaneLegality::ILLEGAL);
}

// §7.4: near-transverse yellow dipping slightly downward would flip under a
// naive "flip when d.y < 0" convention; the beta band must keep "below = legal".
TEST(PlanFLegality, NearHorizontalDippingYellowStillBelowLegal) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{-1500.0, 2050.0}, {-500.0, 2017.0}, {500.0, 1983.0}, {1500.0, 1950.0}}),
        lane_obj("below", LABEL_MAIN_LANE, "main-lane", {{300.0, 500.0}, {300.0, 1000.0}, {300.0, 1500.0}}),
        lane_obj("above", LABEL_OTHER_LANE, "other-lane", {{300.0, 2500.0}, {300.0, 3000.0}, {300.0, 3500.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_EQ(verdict_of(report, "below"), LaneLegality::LEGAL);
    EXPECT_EQ(verdict_of(report, "above"), LaneLegality::ILLEGAL);
}

// §7.5: PathObservationBuilder sorts marking waypoints by y, which zigzags a
// transverse yellow; the gate's PCA re-sort must still produce the right verdict.
TEST(PlanFLegality, ZigzagSortedHorizontalYellowStillCorrect) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{-1200.0, 2003.0}, {-400.0, 1998.0}, {400.0, 2004.0}, {1200.0, 1997.0}}),
        lane_obj("below", LABEL_MAIN_LANE, "main-lane", {{300.0, 500.0}, {300.0, 1000.0}, {300.0, 1500.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    ASSERT_EQ(obs.markings.size(), 1u);
    // Builder really did scramble the x-order (sorted by noisy y).
    EXPECT_NE(obs.markings[0].points.front().x, -1200.0);
    auto report = gate.evaluate(obs);
    EXPECT_EQ(verdict_of(report, "below"), LaneLegality::LEGAL);
}

// §7.6: curved yellow - the signed test follows the nearest segment, so a lane
// hugging the concave side stays on its true side along the whole arc.
TEST(PlanFLegality, CurvedYellowUsesNearestSegment) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {150.0, 1000.0}, {550.0, 2000.0}, {1150.0, 2900.0}}),
        lane_obj("right_of_curve", LABEL_MAIN_LANE, "main-lane", {{700.0, 300.0}, {900.0, 1100.0}, {1300.0, 2000.0}}),
        lane_obj("left_of_curve", LABEL_OTHER_LANE, "other-lane", {{-600.0, 300.0}, {-400.0, 1100.0}, {-100.0, 2000.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_EQ(verdict_of(report, "right_of_curve"), LaneLegality::LEGAL);
    EXPECT_EQ(verdict_of(report, "left_of_curve"), LaneLegality::ILLEGAL);
}

// §7.7: a lane entirely outside the yellow's projection span (+ext) must not
// be judged by extrapolation - verdict UNKNOWN, never filtered.
TEST(PlanFLegality, LaneBeyondYellowSpanIsUnknown) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 500.0}, {0.0, 1000.0}}),
        lane_obj("far_left", LABEL_OTHER_LANE, "other-lane", {{-400.0, 2000.0}, {-400.0, 2500.0}, {-400.0, 3000.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_EQ(verdict_of(report, "far_left"), LaneLegality::UNKNOWN);
}

// §7.8: crossroad with the current road's longitudinal yellow and the
// destination road's transverse yellow: each lane is ruled by its nearest
// applicable reference; the opposite-direction turn-lane goes ILLEGAL.
TEST(PlanFLegality, TwoYellowsCrossroadTurnLanes) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl_current", {{-300.0, 0.0}, {-300.0, 750.0}, {-300.0, 1500.0}}),
        yellow_obj("yl_dest", {{-2000.0, 2500.0}, {0.0, 2500.0}, {2000.0, 2500.0}}),
        lane_obj("turn_correct", LABEL_TURN_LANE, "turn-lane", {{300.0, 2100.0}, {1000.0, 2100.0}, {1700.0, 2100.0}}),
        lane_obj("turn_opposite", LABEL_TURN_LANE, "turn-lane", {{300.0, 2900.0}, {1000.0, 2900.0}, {1700.0, 2900.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_EQ(verdict_of(report, "turn_correct"), LaneLegality::LEGAL);
    EXPECT_EQ(verdict_of(report, "turn_opposite"), LaneLegality::ILLEGAL);
}

// §7.9: dead-zone - a lane hugging the yellow keeps its previous settled verdict.
TEST(PlanFLegality, DeadZoneKeepsPreviousVerdict) {
    LaneLegalityGate gate;
    json f1 = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("lane", LABEL_MAIN_LANE, "main-lane", {{400.0, 200.0}, {400.0, 1200.0}, {400.0, 2200.0}}),
    });
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(f1)), "lane"),
              LaneLegality::LEGAL);
    json f2 = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("lane", LABEL_MAIN_LANE, "main-lane", {{50.0, 200.0}, {50.0, 1200.0}, {50.0, 2200.0}}),
    });
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(f2)), "lane"),
              LaneLegality::LEGAL);
}

// §7.10: yellow dropout - held reference keeps verdicts alive through the hold
// window, then decays to permissive UNKNOWN (stale verdicts must not filter forever).
TEST(PlanFLegality, YellowDropoutHoldThenPermissive) {
    LaneLegalityGate gate;
    json with_yellow = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("left", LABEL_OTHER_LANE, "other-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    json without_yellow = frame_of({
        lane_obj("left", LABEL_OTHER_LANE, "other-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(with_yellow)), "left"),
              LaneLegality::ILLEGAL);
    for (int i = 1; i <= gate.params().yellow_hold_frames; ++i) {
        auto report = gate.evaluate(PathObservationBuilder::build(without_yellow));
        EXPECT_FALSE(report.yellow_visible);
        EXPECT_EQ(report.yellow_age_frames, i);
        EXPECT_EQ(verdict_of(report, "left"), LaneLegality::ILLEGAL) << "frame age " << i;
    }
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(without_yellow)), "left"),
              LaneLegality::UNKNOWN);
}

// §7.11: a settled verdict only flips after verdict_flip_frames consistent frames.
TEST(PlanFLegality, VerdictFlipNeedsStableFrames) {
    LaneLegalityGate gate;
    json legal_frame = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("lane", LABEL_MAIN_LANE, "main-lane", {{400.0, 200.0}, {400.0, 1200.0}, {400.0, 2200.0}}),
    });
    json illegal_frame = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("lane", LABEL_MAIN_LANE, "main-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(legal_frame)), "lane"),
              LaneLegality::LEGAL);
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(illegal_frame)), "lane"),
              LaneLegality::LEGAL);  // 1 frame of disagreement: hold
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(illegal_frame)), "lane"),
              LaneLegality::ILLEGAL);  // 2nd consistent frame: flip
}

// §7.12: lane id churn - a lane appearing under a new id gets a fresh
// geometric verdict immediately, and the exempt id does not leak onto it.
TEST(PlanFLegality, IdChurnFreshVerdictAndNoExemptLeak) {
    LaneLegalityGate gate;
    json f1 = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("id_a", LABEL_MAIN_LANE, "main-lane", {{400.0, 200.0}, {400.0, 1200.0}, {400.0, 2200.0}}),
    });
    gate.evaluate(PathObservationBuilder::build(f1));
    json f2 = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("id_b", LABEL_MAIN_LANE, "main-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    PathObservationFrame obs2 = PathObservationBuilder::build(f2);
    auto report2 = gate.evaluate(obs2);
    EXPECT_EQ(verdict_of(report2, "id_b"), LaneLegality::ILLEGAL);
    PathObservationFrame filtered = LaneLegalityGate::filter(obs2, report2, "id_a");
    EXPECT_TRUE(filtered.lanes.empty());
}

// §7.14 (F1 slice): filter removes non-exempt ILLEGAL lanes, keeps the exempt
// current lane and legal lanes; markings pass through untouched.
TEST(PlanFLegality, FilterKeepsExemptAndLegal) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("cur_main", LABEL_MAIN_LANE, "main-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
        lane_obj("bad_other", LABEL_OTHER_LANE, "other-lane", {{-900.0, 200.0}, {-900.0, 1200.0}, {-900.0, 2200.0}}),
        lane_obj("good_other", LABEL_OTHER_LANE, "other-lane", {{500.0, 200.0}, {500.0, 1200.0}, {500.0, 2200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    auto report = gate.evaluate(obs);
    PathObservationFrame filtered = LaneLegalityGate::filter(obs, report, "cur_main");
    ASSERT_EQ(filtered.lanes.size(), 2u);
    EXPECT_EQ(filtered.lanes[0].lane_id, "cur_main");
    EXPECT_EQ(filtered.lanes[1].lane_id, "good_other");
    EXPECT_EQ(filtered.markings.size(), obs.markings.size());
}

// §7.15 (F1 slice): with the illegal left other-lane filtered out, the planner
// has no lane-change target left (raw frame still finds it - locks that the
// gate, not the planner, is what removes it).
TEST(PlanFLegality, FilteredFrameRemovesLaneChangeTarget) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{-350.0, 0.0}, {-350.0, 1500.0}, {-350.0, 3000.0}}),
        lane_obj("cur_main", LABEL_MAIN_LANE, "main-lane", {{0.0, 200.0}, {0.0, 1200.0}, {0.0, 2200.0}}),
        lane_obj("left_other", LABEL_OTHER_LANE, "other-lane", {{-800.0, 200.0}, {-800.0, 1200.0}, {-800.0, 2200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    ASSERT_NE(TrajectoryPlanner::select_other_lane_obs(obs, &obs.lanes[0], true), nullptr);
    auto report = gate.evaluate(obs);
    PathObservationFrame filtered = LaneLegalityGate::filter(obs, report, "cur_main");
    const LaneObservation* cur = nullptr;
    for (const auto& l : filtered.lanes) if (l.lane_id == "cur_main") cur = &l;
    ASSERT_NE(cur, nullptr);
    EXPECT_EQ(TrajectoryPlanner::select_other_lane_obs(filtered, cur, true), nullptr);
}

// Legacy candidate list must be filtered with the same verdicts/ids so the
// state machine and direct-IPM fallback share the planner's world-view.
TEST(PlanFLegality, LegacyFilterMatchesObservationFilter) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("cur_main", LABEL_MAIN_LANE, "main-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
        lane_obj("good_other", LABEL_OTHER_LANE, "other-lane", {{500.0, 200.0}, {500.0, 1200.0}, {500.0, 2200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    auto report = gate.evaluate(obs);

    std::vector<LaneCandidate> legacy;
    for (const auto& obj : telemetry["objects"]) {
        int label = obj.value("label", -1);
        if (label != LABEL_MAIN_LANE && label != LABEL_OTHER_LANE && label != LABEL_TURN_LANE) continue;
        LaneCandidate c;
        c.label = label;
        c.class_name = obj.value("class_name", "");
        c.raw_obj = obj;
        legacy.push_back(c);
    }
    ASSERT_EQ(legacy.size(), 2u);

    std::vector<LaneCandidate> kept = LaneLegalityGate::filter_legacy(legacy, report, "");
    ASSERT_EQ(kept.size(), 1u);
    EXPECT_EQ(kept[0].raw_obj.value("id", ""), "good_other");

    std::vector<LaneCandidate> kept_exempt = LaneLegalityGate::filter_legacy(legacy, report, "cur_main");
    EXPECT_EQ(kept_exempt.size(), 2u);
}

// Bug fix (lane_change_and_turn path-never-changes): a turn-lane crossing the
// very divider it's judged against is classified ILLEGAL by design - that
// must not mean "invisible to the turn selector," or turn intents can never
// commit. filter()/filter_legacy() must keep turn-lane candidates regardless
// of verdict; only the classification/report is unaffected.
TEST(PlanFLegality, TurnLaneExemptFromFilterEvenWhenIllegal) {
    LaneLegalityGate gate;
    json both = frame_of({
        yellow_obj("yl_current", {{-300.0, 0.0}, {-300.0, 750.0}, {-300.0, 1500.0}}),
        yellow_obj("yl_dest", {{-2000.0, 2500.0}, {0.0, 2500.0}, {2000.0, 2500.0}}),
        lane_obj("turn_opposite", LABEL_TURN_LANE, "turn-lane", {{300.0, 2900.0}, {1000.0, 2900.0}, {1700.0, 2900.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(both);
    auto report = gate.evaluate(obs);
    ASSERT_EQ(verdict_of(report, "turn_opposite"), LaneLegality::ILLEGAL);

    PathObservationFrame filtered = LaneLegalityGate::filter(obs, report, "");
    bool found = false;
    for (const auto& l : filtered.lanes) if (l.lane_id == "turn_opposite") found = true;
    EXPECT_TRUE(found) << "turn-lane must survive filter() even when classified ILLEGAL";

    std::vector<LaneCandidate> legacy;
    for (const auto& o : both["objects"]) {
        if (o.value("label", -1) != LABEL_TURN_LANE) continue;
        LaneCandidate c;
        c.label = o.value("label", -1);
        c.class_name = o.value("class_name", "");
        c.raw_obj = o;
        legacy.push_back(c);
    }
    ASSERT_EQ(legacy.size(), 1u);
    std::vector<LaneCandidate> kept = LaneLegalityGate::filter_legacy(legacy, report, "");
    EXPECT_EQ(kept.size(), 1u) << "turn-lane must survive filter_legacy() even when classified ILLEGAL";
}

// Auto-return target selection: laterally nearest LEGAL main/other lane,
// never a turn-lane, never the excluded (current) lane.
TEST(PlanFLegality, NearestLegalLaneSelection) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("cur_main", LABEL_MAIN_LANE, "main-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
        lane_obj("near_other", LABEL_OTHER_LANE, "other-lane", {{450.0, 200.0}, {450.0, 1200.0}, {450.0, 2200.0}}),
        lane_obj("far_other", LABEL_OTHER_LANE, "other-lane", {{1200.0, 200.0}, {1200.0, 1200.0}, {1200.0, 2200.0}}),
        lane_obj("legal_turn", LABEL_TURN_LANE, "turn-lane", {{300.0, 2100.0}, {1000.0, 2100.0}, {1700.0, 2100.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    auto report = gate.evaluate(obs);
    const LaneObservation* target = LaneLegalityGate::nearest_legal_lane(obs, report, "cur_main");
    ASSERT_NE(target, nullptr);
    EXPECT_EQ(target->lane_id, "near_other");
}

// Kill-switch: with the gate disabled every verdict is UNKNOWN and the filter
// passes the frame through bit-for-bit.
TEST(PlanFLegality, DisabledGateIsTransparent) {
    LaneLegalityGate::Params params;
    params.enabled = false;
    LaneLegalityGate gate(params);
    json telemetry = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("left", LABEL_OTHER_LANE, "other-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    auto report = gate.evaluate(obs);
    EXPECT_TRUE(report.lane_verdicts.empty());
    PathObservationFrame filtered = LaneLegalityGate::filter(obs, report, "");
    EXPECT_EQ(filtered.lanes.size(), obs.lanes.size());
}

// Codex review fix: partial multi-yellow dropout - losing one of two
// references keeps the other's held verdicts alive through the hold window.
TEST(PlanFLegality, PartialYellowDropoutHoldsMissingRef) {
    LaneLegalityGate gate;
    json both = frame_of({
        yellow_obj("yl_current", {{-300.0, 0.0}, {-300.0, 750.0}, {-300.0, 1500.0}}),
        yellow_obj("yl_dest", {{-2000.0, 2500.0}, {0.0, 2500.0}, {2000.0, 2500.0}}),
        lane_obj("turn_opposite", LABEL_TURN_LANE, "turn-lane", {{300.0, 2900.0}, {1000.0, 2900.0}, {1700.0, 2900.0}}),
    });
    json dest_occluded = frame_of({
        yellow_obj("yl_current", {{-300.0, 0.0}, {-300.0, 750.0}, {-300.0, 1500.0}}),
        lane_obj("turn_opposite", LABEL_TURN_LANE, "turn-lane", {{300.0, 2900.0}, {1000.0, 2900.0}, {1700.0, 2900.0}}),
    });
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(both)), "turn_opposite"),
              LaneLegality::ILLEGAL);
    for (int i = 0; i < 3; ++i) {
        auto report = gate.evaluate(PathObservationBuilder::build(dest_occluded));
        // yl_current is fresh, so the frame counts as yellow-visible, but the
        // held yl_dest must still rule the transverse turn-lane.
        EXPECT_TRUE(report.yellow_visible);
        EXPECT_EQ(verdict_of(report, "turn_opposite"), LaneLegality::ILLEGAL) << "frame " << i;
    }
}

// Codex review fix: a lane that leaves the marking's applicability span while
// yellow stays visible must decay to permissive UNKNOWN (via the flip
// hysteresis), not stay filtered on stale memory forever.
TEST(PlanFLegality, LaneLeavingSpanDecaysToUnknown) {
    LaneLegalityGate gate;
    json near_frame = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 500.0}, {0.0, 1000.0}}),
        lane_obj("lane", LABEL_OTHER_LANE, "other-lane", {{-400.0, 200.0}, {-400.0, 700.0}, {-400.0, 1200.0}}),
    });
    json far_frame = frame_of({
        yellow_obj("yl", {{0.0, 0.0}, {0.0, 500.0}, {0.0, 1000.0}}),
        lane_obj("lane", LABEL_OTHER_LANE, "other-lane", {{-400.0, 2200.0}, {-400.0, 2700.0}, {-400.0, 3200.0}}),
    });
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(near_frame)), "lane"),
              LaneLegality::ILLEGAL);
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(far_frame)), "lane"),
              LaneLegality::ILLEGAL);  // 1st out-of-span frame: hysteresis holds
    EXPECT_EQ(verdict_of(gate.evaluate(PathObservationBuilder::build(far_frame)), "lane"),
              LaneLegality::UNKNOWN);  // 2nd frame: decayed, lane no longer filtered
}

// Codex review fix: a low-confidence solid-yellow detection must not build a
// reference (misdetections must not start filtering lanes).
TEST(PlanFLegality, LowConfidenceYellowIgnored) {
    LaneLegalityGate gate;
    json yl = yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}});
    yl["confidence"] = 0.1;
    json telemetry = frame_of({
        yl,
        lane_obj("left", LABEL_OTHER_LANE, "other-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_FALSE(report.yellow_visible);
    EXPECT_EQ(verdict_of(report, "left"), LaneLegality::UNKNOWN);
}

// Codex review fix: polygon-only solid-yellow (thick mask rectangle, no
// waypoints) is collapsed to a centerline - the cross-thickness edges of the
// polygon must not flip signs near the marking.
TEST(PlanFLegality, PolygonFallbackRectangleCenterline) {
    LaneLegalityGate gate;
    json yl = {{"id", "yl"}, {"label", 17}, {"class_name", "solid-yellow"}, {"confidence", 0.9},
               {"polygons_real_world", json::array({json::array({
                   {-40.0, 0.0}, {40.0, 0.0}, {40.0, 750.0}, {40.0, 1500.0},
                   {40.0, 2250.0}, {40.0, 3000.0}, {-40.0, 3000.0}, {-40.0, 2250.0},
                   {-40.0, 1500.0}, {-40.0, 750.0}})})}};
    json telemetry = frame_of({
        yl,
        lane_obj("right", LABEL_MAIN_LANE, "main-lane", {{400.0, 200.0}, {400.0, 1200.0}, {400.0, 2200.0}}),
        lane_obj("left", LABEL_OTHER_LANE, "other-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_TRUE(report.yellow_visible);
    EXPECT_EQ(verdict_of(report, "right"), LaneLegality::LEGAL);
    EXPECT_EQ(verdict_of(report, "left"), LaneLegality::ILLEGAL);
}

// Codex review fix: non-finite telemetry points are dropped at ingestion and
// never poison PCA/sorting/verdicts.
TEST(PlanFLegality, NanPointsAreIgnored) {
    LaneLegalityGate gate;
    json yl = yellow_obj("yl", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}});
    json telemetry = frame_of({
        yl,
        lane_obj("right", LABEL_MAIN_LANE, "main-lane", {{400.0, 200.0}, {400.0, 1200.0}, {400.0, 2200.0}}),
    });
    // Inject a NaN waypoint into the lane after construction.
    telemetry["objects"][1]["waypoints"].push_back(
        {std::numeric_limits<double>::quiet_NaN(), 1800.0});
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_EQ(verdict_of(report, "right"), LaneLegality::LEGAL);
}

// ── Dashed-yellow soft gate (user update 2026-07-18) ─────────────────────────

// Dashed-yellow divides directions exactly like solid-yellow for path drawing:
// by default (no lane_change intent) the far-side lane is filtered.
TEST(PlanFLegality, DashedYellowGatesLikeSolidByDefault) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        lane_obj("dy", LABEL_DASHED_YELLOW, "dashed-yellow", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("right", LABEL_MAIN_LANE, "main-lane", {{400.0, 200.0}, {400.0, 1200.0}, {400.0, 2200.0}}),
        lane_obj("left", LABEL_OTHER_LANE, "other-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    auto report = gate.evaluate(obs);
    EXPECT_TRUE(report.yellow_visible);
    EXPECT_EQ(verdict_of(report, "right"), LaneLegality::LEGAL);
    EXPECT_EQ(verdict_of(report, "left"), LaneLegality::ILLEGAL);
    EXPECT_TRUE(report.soft_illegal.at("left"));
    PathObservationFrame filtered = LaneLegalityGate::filter(obs, report, "");
    ASSERT_EQ(filtered.lanes.size(), 1u);
    EXPECT_EQ(filtered.lanes[0].lane_id, "right");
}

// With allow_soft_illegal (lane_change intent active) the dashed-side lane
// survives filtering, but a solid-side lane never does.
TEST(PlanFLegality, AllowSoftKeepsDashedSideNeverSolidSide) {
    LaneLegalityGate gate;
    // Dashed-yellow on the left of the vehicle, solid-yellow further left:
    // lane A is across the dashed only (soft), lane B is across the solid (hard).
    json telemetry = frame_of({
        lane_obj("dy", LABEL_DASHED_YELLOW, "dashed-yellow", {{-300.0, 0.0}, {-300.0, 1500.0}, {-300.0, 3000.0}}),
        yellow_obj("sy", {{-1400.0, 0.0}, {-1400.0, 1500.0}, {-1400.0, 3000.0}}),
        lane_obj("across_dashed", LABEL_OTHER_LANE, "other-lane", {{-700.0, 200.0}, {-700.0, 1200.0}, {-700.0, 2200.0}}),
        lane_obj("across_solid", LABEL_OTHER_LANE, "other-lane", {{-1800.0, 200.0}, {-1800.0, 1200.0}, {-1800.0, 2200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    auto report = gate.evaluate(obs);
    EXPECT_EQ(verdict_of(report, "across_dashed"), LaneLegality::ILLEGAL);
    EXPECT_TRUE(report.soft_illegal.at("across_dashed"));
    EXPECT_EQ(verdict_of(report, "across_solid"), LaneLegality::ILLEGAL);
    EXPECT_FALSE(report.soft_illegal.at("across_solid"));

    PathObservationFrame strict = LaneLegalityGate::filter(obs, report, "");
    EXPECT_TRUE(strict.lanes.empty());
    PathObservationFrame lane_change_view =
        LaneLegalityGate::filter(obs, report, "", /*allow_soft_illegal=*/true);
    ASSERT_EQ(lane_change_view.lanes.size(), 1u);
    EXPECT_EQ(lane_change_view.lanes[0].lane_id, "across_dashed");
}

// Kill-switch: with dashed_yellow_enabled=false a dashed-yellow builds no
// reference and the far-side lane stays UNKNOWN.
TEST(PlanFLegality, DashedYellowDisabledParam) {
    LaneLegalityGate::Params params;
    params.dashed_yellow_enabled = false;
    LaneLegalityGate gate(params);
    json telemetry = frame_of({
        lane_obj("dy", LABEL_DASHED_YELLOW, "dashed-yellow", {{0.0, 0.0}, {0.0, 1500.0}, {0.0, 3000.0}}),
        lane_obj("left", LABEL_OTHER_LANE, "other-lane", {{-400.0, 200.0}, {-400.0, 1200.0}, {-400.0, 2200.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_FALSE(report.yellow_visible);
    EXPECT_EQ(verdict_of(report, "left"), LaneLegality::UNKNOWN);
}

// Softness follows the ruling (nearest applicable) reference when both a
// solid and a dashed yellow could judge the same lane.
TEST(PlanFLegality, SoftnessFollowsNearestRulingReference) {
    LaneLegalityGate gate;
    // Lane sits left of both, but the dashed-yellow is much nearer.
    json telemetry = frame_of({
        lane_obj("dy", LABEL_DASHED_YELLOW, "dashed-yellow", {{-300.0, 0.0}, {-300.0, 1500.0}, {-300.0, 3000.0}}),
        yellow_obj("sy", {{2000.0, 0.0}, {2000.0, 1500.0}, {2000.0, 3000.0}}),
        lane_obj("lane", LABEL_OTHER_LANE, "other-lane", {{-750.0, 200.0}, {-750.0, 1200.0}, {-750.0, 2200.0}}),
    });
    auto report = gate.evaluate(PathObservationBuilder::build(telemetry));
    EXPECT_EQ(verdict_of(report, "lane"), LaneLegality::ILLEGAL);
    EXPECT_TRUE(report.soft_illegal.at("lane"));
}

// ─── Plan F F2/F3: control_node wiring + auto-return (§7 items 13-22) ────────

// §5 item 4b: the gate must produce one filtered world-view BEFORE any lane
// consumer - legacy intent resolution, split_main_lanes, select_turn_lane,
// update_lane_state, planner - with the followed lane exempt. Pinned on the
// control_node source like the other wiring tests in this file.
TEST(PlanFLegality, ControlNodeFiltersBeforeAllConsumers) {
    std::string source = read_control_node();
    size_t build_pos = source.find("PathObservationFrame obs_frame = PathObservationBuilder::build(telemetry);");
    size_t eval_pos = source.find("legality_gate_.evaluate(obs_frame)");
    size_t filter_pos = source.find("LaneLegalityGate::filter(");
    size_t legacy_filter_pos = source.find("LaneLegalityGate::filter_legacy(");
    size_t legacy_resolve_pos = source.find("Resolve legacy directionless intents");
    size_t split_pos = source.find("LegacyLaneModel::split_main_lanes(");
    size_t update_pos = source.find("update_lane_state(lanes,");
    ASSERT_NE(build_pos, std::string::npos);
    ASSERT_NE(eval_pos, std::string::npos);
    ASSERT_NE(filter_pos, std::string::npos);
    ASSERT_NE(legacy_filter_pos, std::string::npos);
    ASSERT_NE(legacy_resolve_pos, std::string::npos);
    ASSERT_NE(split_pos, std::string::npos);
    ASSERT_NE(update_pos, std::string::npos);
    EXPECT_LT(build_pos, eval_pos);
    EXPECT_LT(eval_pos, filter_pos);
    EXPECT_LT(filter_pos, legacy_resolve_pos) << "obs filter must run before legacy intent resolution";
    EXPECT_LT(legacy_filter_pos, legacy_resolve_pos) << "legacy filter must run before legacy intent resolution";
    EXPECT_LT(build_pos, update_pos) << "obs_frame build moved before update_lane_state";
    // Exempt lane and soft flag are passed on both paths.
    EXPECT_NE(source.find("obs_frame, last_legality_report_, last_main_track_id_, legality_allow_soft_"),
              std::string::npos);
    EXPECT_NE(source.find("lanes, last_legality_report_, last_main_track_id_, legality_allow_soft_"),
              std::string::npos);
}

// §5 item 4e: parameter defaults pinned (kill-switches + tuning knobs).
TEST(PlanFLegality, ControlNodeParamDefaults) {
    std::string source = read_control_node();
    EXPECT_NE(source.find("declare_parameter<bool>(\"legality_gate_enabled\", true)"), std::string::npos);
    EXPECT_NE(source.find("declare_parameter<bool>(\"legality_return_enabled\", true)"), std::string::npos);
    EXPECT_NE(source.find("declare_parameter<bool>(\"legality_dashed_yellow_enabled\", true)"), std::string::npos);
    EXPECT_NE(source.find("declare_parameter<double>(\"legality_margin_mm\", 100.0)"), std::string::npos);
    EXPECT_NE(source.find("declare_parameter<int>(\"legality_yellow_hold_frames\", 10)"), std::string::npos);
    EXPECT_NE(source.find("declare_parameter<int>(\"legality_return_debounce_frames\", 5)"), std::string::npos);
    EXPECT_NE(source.find("declare_parameter<double>(\"legality_beta_deg\", 20.0)"), std::string::npos);
    // Return kill-switch actually gates the auto-return eligibility.
    EXPECT_NE(source.find("legality_gate_enabled_ && legality_return_enabled_"), std::string::npos);
}

// §7.21 (wiring half): a real intent from /avs/route_intent always beats the
// internal override - the callback clears the flag and the debounce.
TEST(PlanFLegality, ControlNodeRealIntentClearsOverride) {
    std::string source = read_control_node();
    size_t cb = source.find("void route_intent_callback");
    ASSERT_NE(cb, std::string::npos);
    size_t cb_end = source.find("void cmd_callback", cb);
    ASSERT_NE(cb_end, std::string::npos);
    EXPECT_NE(source.find("legality_return_active_ = false;", cb), std::string::npos);
    EXPECT_LT(source.find("legality_return_active_ = false;", cb), cb_end);
    EXPECT_LT(source.find("legality_auto_return_.reset();", cb), cb_end);
}

// §7.18: followed lane stably ILLEGAL (dashed-yellow divider, so the crossing
// back is not marking-blocked) for the full debounce, legal lane on the right
// -> the auto-return decision fires with the right direction, the internal
// LANE_CHANGE_RIGHT intent produces a lane-change candidate the manager
// commits, and the NEXT frame with the same latched intent does not re-replan
// on intent_change (codex #2).
TEST(PlanFLegality, AutoReturnTriggersAndManagerCommitsAndHolds) {
    LaneLegalityGate gate;
    LegalityAutoReturn auto_return;
    json telemetry = frame_of({
        lane_obj("dy", LABEL_DASHED_YELLOW, "dashed-yellow", {{300.0, 0.0}, {300.0, 700.0}, {300.0, 1400.0}}),
        lane_obj("main_A", LABEL_MAIN_LANE, "main-lane", {{0.0, 200.0}, {0.0, 700.0}, {0.0, 1200.0}}),
        lane_obj("other_B", LABEL_OTHER_LANE, "other-lane", {{900.0, 200.0}, {900.0, 700.0}, {900.0, 1200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    const int debounce = 5;

    LegalityAutoReturn::Decision dec;
    LaneLegalityReport report;
    for (int f = 0; f < debounce; ++f) {
        report = gate.evaluate(obs);
        PathObservationFrame filtered =
            LaneLegalityGate::filter(obs, report, "main_A", /*allow_soft_illegal=*/false);
        dec = auto_return.step(/*eligible=*/true, filtered, report, "main_A", debounce);
        if (f < debounce - 1) {
            EXPECT_FALSE(dec.trigger) << "frame " << f;
            EXPECT_EQ(auto_return.streak(), f + 1);
        }
    }
    ASSERT_TRUE(dec.trigger);
    EXPECT_FALSE(dec.go_left) << "legal lane sits to the right";
    EXPECT_EQ(dec.target_lane_id, "other_B");

    // The internal intent flows through the untouched planner/manager path.
    RouteIntent intent = RouteIntent::LANE_CHANGE_RIGHT;
    PathObservationFrame plan_frame =
        LaneLegalityGate::filter(obs, report, "main_A", /*allow_soft_illegal=*/true);
    CommittedTrajectoryState committed;
    int consecutive_invalid = 0;
    uint64_t frame_no = 0;
    std::string last_main_id = "main_A";

    PlannedTrajectory cand = TrajectoryPlanner::plan_candidate_for_intent(
        plan_frame, intent, committed, /*is_t=*/false, /*t_junction_pending=*/false, last_main_id);
    ASSERT_TRUE(cand.valid);
    EXPECT_EQ(cand.trajectory_kind, TrajectoryKind::LANE_CHANGE_RIGHT);
    EXPECT_FALSE(cand.blocked_by_marking) << "dashed-yellow must not block the return";
    PlannedTrajectory normalized = TrajectoryNormalizer::normalize(cand, committed);
    TrajectoryManager::Decision d1 = TrajectoryManager::update(
        normalized, committed, intent, 1, /*maneuver_dropout_hold_frames=*/10,
        consecutive_invalid, ++frame_no);
    committed = d1.next_state;
    EXPECT_EQ(committed.committed_intent, intent);
    EXPECT_EQ(committed.trajectory.trajectory_kind, TrajectoryKind::LANE_CHANGE_RIGHT);

    PlannedTrajectory cand2 = TrajectoryPlanner::plan_candidate_for_intent(
        plan_frame, intent, committed, false, false, last_main_id);
    PlannedTrajectory normalized2 = TrajectoryNormalizer::normalize(cand2, committed);
    TrajectoryManager::Decision d2 = TrajectoryManager::update(
        normalized2, committed, intent, 1, 10, consecutive_invalid, ++frame_no);
    EXPECT_NE(d2.next_state.replan_reason, "intent_change")
        << "latched override must not look like a fresh intent every frame";
    EXPECT_EQ(d2.next_state.trajectory.trajectory_kind, TrajectoryKind::LANE_CHANGE_RIGHT);

    // While the override intent is active the node calls step(eligible=false):
    // the debounce must stay disarmed instead of accumulating a second trigger.
    dec = auto_return.step(/*eligible=*/false, plan_frame, report, "main_A", debounce);
    EXPECT_FALSE(dec.trigger);
    EXPECT_EQ(auto_return.streak(), 0);
}

// §7.19: ILLEGAL for debounce-1 frames, then legal again -> streak resets, no
// auto-return; a later ILLEGAL run starts counting from zero.
TEST(PlanFLegality, AutoReturnNotTriggeredWhenIllegalClearsBeforeDebounce) {
    LegalityAutoReturn auto_return;
    json telemetry = frame_of({
        lane_obj("main_A", LABEL_MAIN_LANE, "main-lane", {{0.0, 200.0}, {0.0, 700.0}, {0.0, 1200.0}}),
        lane_obj("other_B", LABEL_OTHER_LANE, "other-lane", {{900.0, 200.0}, {900.0, 700.0}, {900.0, 1200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    LaneLegalityReport illegal_report;
    illegal_report.lane_verdicts["main_A"] = LaneLegality::ILLEGAL;
    illegal_report.lane_verdicts["other_B"] = LaneLegality::LEGAL;
    LaneLegalityReport legal_report;
    legal_report.lane_verdicts["main_A"] = LaneLegality::LEGAL;
    legal_report.lane_verdicts["other_B"] = LaneLegality::LEGAL;

    for (int f = 0; f < 4; ++f) {
        EXPECT_FALSE(auto_return.step(true, obs, illegal_report, "main_A", 5).trigger);
    }
    EXPECT_FALSE(auto_return.step(true, obs, legal_report, "main_A", 5).trigger);
    EXPECT_EQ(auto_return.streak(), 0);
    for (int f = 0; f < 4; ++f) {
        EXPECT_FALSE(auto_return.step(true, obs, illegal_report, "main_A", 5).trigger)
            << "streak must restart from zero after the legal frame";
    }
}

// §7.20 + §7.22: while a committed turn/lane-change is active - or the return
// kill-switch is off - the node passes eligible=false; the decision never
// fires and the debounce never accumulates.
TEST(PlanFLegality, AutoReturnSuppressedWhenIneligible) {
    LegalityAutoReturn auto_return;
    json telemetry = frame_of({
        lane_obj("main_A", LABEL_MAIN_LANE, "main-lane", {{0.0, 200.0}, {0.0, 700.0}, {0.0, 1200.0}}),
        lane_obj("other_B", LABEL_OTHER_LANE, "other-lane", {{900.0, 200.0}, {900.0, 700.0}, {900.0, 1200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    LaneLegalityReport illegal_report;
    illegal_report.lane_verdicts["main_A"] = LaneLegality::ILLEGAL;
    illegal_report.lane_verdicts["other_B"] = LaneLegality::LEGAL;

    for (int f = 0; f < 20; ++f) {
        EXPECT_FALSE(auto_return.step(/*eligible=*/false, obs, illegal_report, "main_A", 5).trigger);
        EXPECT_EQ(auto_return.streak(), 0);
    }
}

// No legal lane anywhere: the decision must never trigger (we never trade the
// exempt path we have for nothing), but the streak stays exposed for debug.
TEST(PlanFLegality, AutoReturnHoldsLaneWhenNoLegalTarget) {
    LegalityAutoReturn auto_return;
    json telemetry = frame_of({
        lane_obj("main_A", LABEL_MAIN_LANE, "main-lane", {{0.0, 200.0}, {0.0, 700.0}, {0.0, 1200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    LaneLegalityReport report;
    report.lane_verdicts["main_A"] = LaneLegality::ILLEGAL;

    for (int f = 0; f < 12; ++f) {
        EXPECT_FALSE(auto_return.step(true, obs, report, "main_A", 5).trigger) << "frame " << f;
    }
    EXPECT_EQ(auto_return.streak(), 12);
}

// Solid-yellow divider: the return decision still fires, but the resulting
// internal lane-change is stopped by the existing solid-marking gate
// (is_lane_change_blocked_by_solid_obs treats solid-yellow as blocking, and
// Plan F §3 deliberately leaves that gate untouched). Pins that no planned
// path ever crosses the solid line - the blocked machinery holds follow_main.
TEST(PlanFLegality, AutoReturnAcrossSolidYellowStaysBlocked) {
    LaneLegalityGate gate;
    json telemetry = frame_of({
        yellow_obj("sy", {{300.0, 0.0}, {300.0, 700.0}, {300.0, 1400.0}}),
        lane_obj("main_A", LABEL_MAIN_LANE, "main-lane", {{0.0, 200.0}, {0.0, 700.0}, {0.0, 1200.0}}),
        lane_obj("other_B", LABEL_OTHER_LANE, "other-lane", {{900.0, 200.0}, {900.0, 700.0}, {900.0, 1200.0}}),
    });
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    LaneLegalityReport report = gate.evaluate(obs);
    EXPECT_EQ(verdict_of(report, "main_A"), LaneLegality::ILLEGAL);
    EXPECT_EQ(verdict_of(report, "other_B"), LaneLegality::LEGAL);

    LegalityAutoReturn auto_return;
    LegalityAutoReturn::Decision dec;
    PathObservationFrame filtered = LaneLegalityGate::filter(obs, report, "main_A", true);
    for (int f = 0; f < 5; ++f) {
        dec = auto_return.step(true, filtered, report, "main_A", 5);
    }
    ASSERT_TRUE(dec.trigger);

    CommittedTrajectoryState committed;
    std::string last_main_id = "main_A";
    PlannedTrajectory cand = TrajectoryPlanner::plan_candidate_for_intent(
        filtered, RouteIntent::LANE_CHANGE_RIGHT, committed, false, false, last_main_id);
    EXPECT_TRUE(cand.blocked_by_marking)
        << "crossing back over a solid-yellow must stay blocked by the marking gate";
    EXPECT_NE(cand.trajectory_kind, TrajectoryKind::LANE_CHANGE_RIGHT);
}

// ── Frozen turn execution geometry (trajectory_latch.hpp) ───────────────────
// The latch replays a path captured in the vehicle frame of the latch frame, so
// every frame it must hand back what is left of it in the *current* vehicle
// frame. Point2D is (x = lateral, y = forward).
//
// Trajectories here follow the planners' convention: they do not contain the
// vehicle origin, because LegacyLaneModel::evaluate_trajectory_at_lookahead
// prepends (0,0) to everything it measures. followed_length() below mirrors
// that, and is the only length that means "distance the vehicle still has to
// travel" - raw path_length() of a re-expressed path always omits the leading
// stretch between the vehicle and the first remaining point.

namespace {

// Quarter circle of radius R turning right (+x), starting at the vehicle origin
// heading forward (+y). Arc length from the start to angle a is R*a.
std::vector<Point2D> quarter_turn_right(double radius, int steps) {
    std::vector<Point2D> pts;
    for (int i = 0; i <= steps; ++i) {
        double a = (M_PI / 2.0) * static_cast<double>(i) / static_cast<double>(steps);
        pts.push_back({radius * (1.0 - std::cos(a)), radius * std::sin(a)});
    }
    return pts;
}

// The near end of a turn marking as the camera actually delivers it: an arc of
// radius R covering only `span_rad` of the full 90 degrees.
std::vector<Point2D> partial_turn_right(double radius, double span_rad, int steps) {
    std::vector<Point2D> pts;
    for (int i = 0; i <= steps; ++i) {
        double a = span_rad * static_cast<double>(i) / static_cast<double>(steps);
        pts.push_back({radius * (1.0 - std::cos(a)), radius * std::sin(a)});
    }
    return pts;
}

double followed_length(const std::vector<Point2D>& pts) {
    std::vector<Point2D> with_origin;
    with_origin.push_back({0.0, 0.0});
    with_origin.insert(with_origin.end(), pts.begin(), pts.end());
    return TrajectoryLatch::path_length(with_origin);
}

}  // namespace

// --- extend_to_turn_angle -------------------------------------------------
// The camera only ever hands over the near end of the turn marking, so a
// latched turn typically encodes 40-60 of the 90 degrees. Replaying it leaves
// the vehicle half-turned, where the new road is still too oblique to be
// labelled main-lane and perception never recovers.

namespace {
constexpr double kDeg = M_PI / 180.0;
constexpr double kRight90 = 90.0 * kDeg;
// The production defaults these tests pin against.
constexpr double kRunout = 700.0;
constexpr double kMinR = 800.0;
constexpr double kMaxR = 4000.0;
constexpr double kMinSpan = 15.0 * kDeg;
}  // namespace

TEST(TurnLatchExtension, ShortTurnIsGrownToTheFullTurnAngle) {
    // 50 degrees observed of a 1500mm-radius junction.
    std::vector<Point2D> obs = partial_turn_right(1500.0, 50.0 * kDeg, 15);
    ASSERT_NEAR(TrajectoryLatch::terminal_heading_rad(obs) / kDeg, 50.0, 2.0);

    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        obs, kRight90, kRunout, kMinR, kMaxR, kMinSpan);

    EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(ext) / kDeg, 90.0, 0.5);
    EXPECT_GT(TrajectoryLatch::path_length(ext), TrajectoryLatch::path_length(obs));
}

TEST(TurnLatchExtension, ExtensionKeepsTheRadiusOfTheObservedMarking) {
    const double r = 1500.0;
    std::vector<Point2D> obs = partial_turn_right(r, 50.0 * kDeg, 15);
    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        obs, kRight90, 0.0, kMinR, kMaxR, kMinSpan);

    // Continuing the same arc to 90 degrees must land on the same circle, i.e.
    // at (r, r) for an arc that starts at the origin heading straight ahead.
    // Tight bounds on purpose: a biased radius estimate lands the exit a long
    // way off the new lane, which is the failure this feature exists to fix.
    EXPECT_NEAR(ext.back().x, r, 15.0);
    EXPECT_NEAR(ext.back().y, r, 15.0);
}

TEST(TurnLatchExtension, StraightRunoutIsAppendedAfterTheTurn) {
    std::vector<Point2D> obs = partial_turn_right(1500.0, 50.0 * kDeg, 15);
    std::vector<Point2D> with = TrajectoryLatch::extend_to_turn_angle(
        obs, kRight90, kRunout, kMinR, kMaxR, kMinSpan);
    std::vector<Point2D> without = TrajectoryLatch::extend_to_turn_angle(
        obs, kRight90, 0.0, kMinR, kMaxR, kMinSpan);

    EXPECT_NEAR(TrajectoryLatch::path_length(with) - TrajectoryLatch::path_length(without),
                kRunout, 1.0);
    // The run-out must be straight, so it must not rotate the vehicle further:
    // this is the stretch pure pursuit flattens once it is shorter than the
    // lookahead, and it only costs nothing if there is no turning left in it.
    EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(with) / kDeg, 90.0, 0.5);
}

TEST(TurnLatchExtension, AlreadyCompleteTurnGainsOnlyTheRunout) {
    std::vector<Point2D> obs = partial_turn_right(1500.0, 90.0 * kDeg, 30);
    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        obs, kRight90, kRunout, kMinR, kMaxR, kMinSpan);
    // No arc left to add - only the straight run-out that keeps the lookahead
    // clamp away from the curved part.
    EXPECT_NEAR(TrajectoryLatch::path_length(ext) - TrajectoryLatch::path_length(obs),
                kRunout, kRunout * 0.1);
    EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(ext) / kDeg, 90.0, 0.5);
}

TEST(TurnLatchExtension, StraightPathIsNeverBentIntoATurn) {
    std::vector<Point2D> straight = {{0.0, 100.0}, {0.0, 600.0}, {0.0, 1100.0}, {0.0, 1600.0}};
    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        straight, kRight90, kRunout, kMinR, kMaxR, kMinSpan);
    EXPECT_EQ(ext.size(), straight.size());

    // A barely-curving path is still not a turn - below the observed-span floor
    // the geometry is noise, not evidence of a junction arc.
    std::vector<Point2D> nearly = partial_turn_right(1500.0, 8.0 * kDeg, 10);
    EXPECT_EQ(TrajectoryLatch::extend_to_turn_angle(
                  nearly, kRight90, kRunout, kMinR, kMaxR, kMinSpan).size(),
              nearly.size());
}

TEST(TurnLatchExtension, PathCurvingAgainstTheIntentIsLeftAlone) {
    // A left-curving observation must not be dragged round to a right turn.
    std::vector<Point2D> left;
    for (const auto& p : partial_turn_right(1500.0, 50.0 * kDeg, 15)) {
        left.push_back({-p.x, p.y});
    }
    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        left, kRight90, kRunout, kMinR, kMaxR, kMinSpan);
    EXPECT_EQ(ext.size(), left.size());
}

TEST(TurnLatchExtension, LeftTurnsExtendTheOtherWay) {
    std::vector<Point2D> left;
    for (const auto& p : partial_turn_right(1500.0, 50.0 * kDeg, 15)) {
        left.push_back({-p.x, p.y});
    }
    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        left, -kRight90, kRunout, kMinR, kMaxR, kMinSpan);
    EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(ext) / kDeg, -90.0, 0.5);
}

TEST(TurnLatchExtension, RadiusIsClampedSoATightObservationCannotSpin) {
    // A 200mm-radius blob would otherwise curl the extension into a hairpin.
    std::vector<Point2D> tight = partial_turn_right(200.0, 50.0 * kDeg, 15);
    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        tight, kRight90, 0.0, kMinR, kMaxR, kMinSpan);
    double added = TrajectoryLatch::path_length(ext) - TrajectoryLatch::path_length(tight);
    // 40 degrees still to turn at the 800mm floor.
    EXPECT_NEAR(added, kMinR * 40.0 * kDeg, 120.0);
}

TEST(TurnLatchExtension, ExtendedTurnReplaysAllTheWayToNinetyDegrees) {
    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        partial_turn_right(1500.0, 50.0 * kDeg, 15), kRight90, kRunout, kMinR, kMaxR, kMinSpan);
    double total = TrajectoryLatch::path_length(ext);

    // Replay it the way control_node does and track how far the vehicle has
    // rotated: the residual turn left in the path must drain to zero, meaning
    // the vehicle has taken up the whole 90 degrees by the time it releases.
    double last_residual = 1e9;
    for (double s = 0.0; s < total; s += 100.0) {
        std::vector<Point2D> rem = TrajectoryLatch::re_express(ext, s);
        if (rem.size() < 2) break;
        double residual = TrajectoryLatch::terminal_heading_rad(rem);
        EXPECT_LE(residual, last_residual + 1e-6);  // monotonically consumed
        last_residual = residual;
    }
    EXPECT_NEAR(last_residual / kDeg, 0.0, 1.0);
}

// A turn marking whose far end flares back out, as the real ones do once the
// camera is looking along it: the heading climbs past the junction angle and
// then unwinds over the last couple of hundred millimetres. Taken from the
// vehicle log of 2026-08-05 (a left turn, mirrored here to the right so it
// shares the kRight90 target with the rest of these tests) where the observed
// heading ran 63 -> 79 -> 87 -> 81 -> 63 degrees over the final 300mm.
namespace {
std::vector<Point2D> turn_with_unwinding_tail() {
    const double profile_deg[] = {9.0,  18.0, 27.0, 36.0, 45.0, 54.0,
                                  63.0, 79.0, 87.0, 81.0, 63.0};
    std::vector<Point2D> pts;
    double x = 0.0, y = 0.0;
    for (double h_deg : profile_deg) {
        double h = h_deg * kDeg;
        x += 100.0 * std::sin(h);
        y += 100.0 * std::cos(h);
        pts.push_back({x, y});
    }
    return pts;
}
}  // namespace

namespace {
// Build a path whose segment headings follow a given profile, 100mm apart.
std::vector<Point2D> path_from_headings(const std::vector<double>& deg) {
    std::vector<Point2D> pts;
    double x = 0.0, y = 0.0;
    for (double d : deg) {
        double h = d * kDeg;
        x += 100.0 * std::sin(h);
        y += 100.0 * std::cos(h);
        pts.push_back({x, y});
    }
    return pts;
}
}  // namespace

TEST(TurnLatchTrim, FlaredTipIsCutBackToTheFurthestTurnedSegment) {
    // Heading profile of a left turn logged on the vehicle 2026-08-05: it reaches
    // -76 degrees and then unwinds to -66 over the last 200mm, because the far
    // end of an IPM observation is its least reliable part.
    std::vector<Point2D> obs = path_from_headings(
        {17, 12, 8, 0, -4, -10, -16, -21, -27, -34, -42, -52, -63, -68, -76, -72, -66});
    ASSERT_NEAR(TrajectoryLatch::terminal_heading_rad(obs) / kDeg, -66.0, 1.0);

    std::vector<Point2D> cut = TrajectoryLatch::trim_flared_tip(obs, -kRight90);

    EXPECT_EQ(cut.size(), obs.size() - 2);
    EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(cut) / kDeg, -76.0, 1.0)
        << "the observation's real span was understated by the flared tip";
}

TEST(TurnLatchTrim, MonotoneTailIsLeftAlone) {
    // The one logged turn whose tail did not unwind - and the only one that got
    // extended before this trim existed. Nothing here should be removed.
    std::vector<Point2D> obs = path_from_headings(
        {19, 15, 9, 4, -2, -8, -14, -23, -33, -47, -62, -71, -72});
    EXPECT_EQ(TrajectoryLatch::trim_flared_tip(obs, -kRight90).size(), obs.size());
}

TEST(TurnLatchTrim, SamplingJitterAtTheTipIsNotMistakenForAFlare) {
    // A couple of degrees of wobble is noise, not the marking turning back.
    // Trimming on that would eat the end of every path.
    std::vector<Point2D> obs =
        path_from_headings({5, 12, 20, 29, 39, 50, 62, 71, 79, 84, 83, 84});
    EXPECT_EQ(TrajectoryLatch::trim_flared_tip(obs, kRight90).size(), obs.size());
}

TEST(TurnLatchTrim, RefusesToTrimWhenNothingUsefulWouldBeLeft) {
    // A path that turns early then runs back: trimming to the peak would leave
    // too little to fit a circle through, so it is handed back whole and the
    // guards downstream deal with it.
    std::vector<Point2D> obs = path_from_headings({40, 30, 20, 10, 0, -10, -20});
    EXPECT_EQ(TrajectoryLatch::trim_flared_tip(obs, kRight90).size(), obs.size());
}

TEST(TurnLatchExtension, FlaredObservationsCanBeExtendedOnceTheTipIsTrimmed) {
    // End to end on the same logged profile: before the trim the contradictory
    // tangent made extend_to_turn_angle hand the path straight back, which on the
    // vehicle left five of seven left turns with no extension and no run-out.
    std::vector<Point2D> obs = path_from_headings(
        {17, 12, 8, 0, -4, -10, -16, -21, -27, -34, -42, -52, -63, -68, -76, -72, -66});
    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        obs, -kRight90, kRunout, 350.0, kMaxR, kMinSpan);

    EXPECT_GT(TrajectoryLatch::path_length(ext), TrajectoryLatch::path_length(obs));
    EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(ext) / kDeg, -90.0, 1.0);
}

TEST(TurnLatchExtension, LeastSquaresFitSurvivesNoiseThatBreaksAThreePointFit) {
    // A known 1000mm arc, sampled and then jittered sideways the way real lane
    // centreline waypoints are. The least-squares fit uses every sample, so the
    // noise averages down; a circumcircle through three picked samples cannot
    // average anything and follows whichever three it was handed.
    // The claim is about spread, not about any one draw: on a single noise
    // realisation the circumcircle can land closer by luck, and asserting one
    // draw would be testing that luck. So both fits are run over the same 200
    // realisations and their RMS radius error compared.
    const double kTrueR = 1000.0;
    auto lcg = [](uint32_t& s) {  // deterministic, no <random> dependency
        s = s * 1664525u + 1013904223u;
        return (static_cast<double>(s >> 8) / 16777216.0) * 2.0 - 1.0;  // [-1, 1)
    };
    uint32_t seed = 20260805u;
    double ls_sq = 0.0, three_sq = 0.0;
    int trials = 200, ls_failures = 0;

    for (int t = 0; t < trials; ++t) {
        std::vector<Point2D> obs = partial_turn_right(kTrueR, 60.0 * kDeg, 12);
        for (auto& p : obs) p.x += 12.0 * lcg(seed);

        Point2D centre{0.0, 0.0};
        double radius = 0.0;
        if (!TrajectoryLatch::fit_circle_ls(obs, 0, obs.size() - 1, centre, radius)) {
            ++ls_failures;
            continue;
        }
        ls_sq += (radius - kTrueR) * (radius - kTrueR);

        // Circumcircle through first / middle / last, the fit this replaced.
        size_t n = obs.size() - 1, mid = n / 2;
        const Point2D &a = obs[0], &b = obs[mid], &c = obs[n];
        double det = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        ASSERT_GT(std::abs(det), 1e-9);
        double a2 = a.x * a.x + a.y * a.y, b2 = b.x * b.x + b.y * b.y,
               c2 = c.x * c.x + c.y * c.y;
        Point2D c3{(a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / det,
                   (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / det};
        double r3 = std::hypot(c.x - c3.x, c.y - c3.y);
        three_sq += (r3 - kTrueR) * (r3 - kTrueR);
    }

    ASSERT_EQ(ls_failures, 0);
    double ls_rms = std::sqrt(ls_sq / trials);
    double three_rms = std::sqrt(three_sq / trials);
    EXPECT_LT(ls_rms, three_rms * 0.8)
        << "least squares RMS " << ls_rms << "mm vs three-point " << three_rms << "mm";
    EXPECT_LT(ls_rms, 120.0) << "least squares RMS radius error " << ls_rms << "mm";

    // Under even noise the gain is only about 1.4x. The gain that matters is
    // against a single displaced endpoint - the flared far end of a real turn
    // marking - because the circumcircle is forced through that point while least
    // squares lets the other eleven outvote it. This is the shape that produced
    // the 38-degree tangent error on the vehicle.
    std::vector<Point2D> flared = partial_turn_right(kTrueR, 60.0 * kDeg, 12);
    flared.back().x -= 90.0;  // far end flares out of the arc

    Point2D fc{0.0, 0.0};
    double fr = 0.0;
    ASSERT_TRUE(TrajectoryLatch::fit_circle_ls(flared, 0, flared.size() - 1, fc, fr));

    size_t n = flared.size() - 1, mid = n / 2;
    const Point2D &a = flared[0], &b = flared[mid], &c = flared[n];
    double det = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    ASSERT_GT(std::abs(det), 1e-9);
    double a2 = a.x * a.x + a.y * a.y, b2 = b.x * b.x + b.y * b.y,
           c2 = c.x * c.x + c.y * c.y;
    Point2D c3{(a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / det,
               (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / det};
    double r3 = std::hypot(c.x - c3.x, c.y - c3.y);

    EXPECT_LT(std::abs(fr - kTrueR), std::abs(r3 - kTrueR) * 0.5)
        << "one flared endpoint: least squares " << fr << "mm, three-point " << r3
        << "mm, true " << kTrueR << "mm";
}

TEST(TurnLatchExtension, FitIsRejectedRatherThanGuessedOnCollinearSamples) {
    std::vector<Point2D> line;
    for (int i = 1; i <= 8; ++i) line.push_back({0.0, 100.0 * i});
    Point2D centre{0.0, 0.0};
    double radius = 0.0;
    EXPECT_FALSE(TrajectoryLatch::fit_circle_ls(line, 0, line.size() - 1, centre, radius));
}

TEST(TurnLatchExtension, TailThatIsNotAnArcIsNotExtended) {
    // What the fit guard is actually for: refusing to continue an arc through a
    // tail that is not one. These headings step between roughly straight and
    // fully turned rather than sweeping, so no circle passes near all of them.
    // Residual 64mm, which is where the one genuinely non-circular tail in the
    // 2026-08-05 capture set landed (64.5mm) - against 2.8-12.7mm for the sound
    // ones, so this is the shape of failure the threshold is placed against.
    // Nothing here says where the marking was going, so the observation is handed
    // back untouched: no invented arc, no run-out welded onto a guessed heading.
    std::vector<Point2D> obs = path_from_headings(
        {0, -85, -2, -86, -4, -88, -6, -90, -8, -90});

    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        obs, -kRight90, kRunout, 350.0, kMaxR, kMinSpan);

    EXPECT_EQ(ext.size(), obs.size());
    EXPECT_NEAR(TrajectoryLatch::path_length(ext),
                TrajectoryLatch::path_length(obs), 1.0);
}

TEST(TurnLatchExtension, CleanArcClearsTheResidualGuardAtVehicleCurvature) {
    // The other half of the guard's job - not blocking sound observations. These
    // are the radii the vehicle actually turns at, where the guard this replaced
    // rejected 7 of 8 left turns (see trajectory_latch.hpp). A clean arc at any
    // of them must extend, or the latch is back to freezing half a junction.
    for (double radius : {350.0, 400.0, 500.0, 600.0}) {
        std::vector<Point2D> obs = partial_turn_right(radius, 60.0 * kDeg, 12);
        std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
            obs, kRight90, kRunout, 350.0, kMaxR, kMinSpan);
        EXPECT_GT(TrajectoryLatch::path_length(ext),
                  TrajectoryLatch::path_length(obs))
            << "radius " << radius << " should extend";
        EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(ext) / kDeg, 90.0, 2.0)
            << "radius " << radius;
    }
}

TEST(TurnLatchExtension, ObservationThatAlreadyReachedTheTargetStillGetsItsRunout) {
    // Logged left turn of 2026-08-05. A near-straight entry followed by a very
    // sharp bend: once the flared tip is trimmed the marking has been seen all
    // the way round to -91 degrees, so there is no arc left to add. The run-out
    // still belongs, pointing along the completed turn rather than past it - a
    // latched path that stops dead at the last observed point leaves the vehicle
    // with nothing to track through the rest of the junction.
    std::vector<Point2D> obs = path_from_headings(
        {-17, 14, 15, 16, 15, 13, 10, 2, -14, -41, -64, -77, -83, -88, -91, -84, -75});

    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        obs, -kRight90, kRunout, 350.0, kMaxR, kMinSpan);

    // Leaves along the turn, never past it: the run-out is clamped to the target
    // rather than carrying the observation's slight overshoot outward.
    EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(ext) / kDeg, -90.0, 2.0);
    EXPECT_GT(TrajectoryLatch::path_length(ext), TrajectoryLatch::path_length(obs));
}

TEST(TurnLatchExtension, ExitHeadingNeverContradictsTheObservation) {
    // The general invariant behind the guard, over the whole family of shapes
    // these tests use: whatever the extension does, the path must leave in a
    // direction the observation supports - somewhere between where the marking
    // was last seen heading and the turn target, never outside that span. An
    // exit beyond the target is the signature of a fit that has taken over.
    struct Case {
        const char* name;
        std::vector<Point2D> obs;
    };
    std::vector<Case> cases = {
        {"clean 50 degree arc", partial_turn_right(1500.0, 50.0 * kDeg, 15)},
        {"clean 30 degree arc", partial_turn_right(1500.0, 30.0 * kDeg, 10)},
        {"tight observation", partial_turn_right(200.0, 50.0 * kDeg, 15)},
        {"already complete", partial_turn_right(1500.0, 90.0 * kDeg, 30)},
        {"flared tail", turn_with_unwinding_tail()},
    };
    for (const Case& c : cases) {
        double psi = TrajectoryLatch::terminal_heading_rad(c.obs);
        std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
            c.obs, kRight90, kRunout, kMinR, kMaxR, kMinSpan);
        double exit = TrajectoryLatch::terminal_heading_rad(ext);
        EXPECT_GE(exit, std::min(psi, kRight90) - 1e-6) << c.name;
        EXPECT_LE(exit, kRight90 + 1e-6) << c.name;
    }
}

TEST(TurnLatchExtension, GuardStillLetsRealisticSamplingNoiseThrough) {
    // The guard must not be so tight that ordinary waypoint jitter stops turns
    // being completed - that would silently give back the short-turn bug this
    // extension was written to fix. 100mm samples on a 1500mm arc with 12mm of
    // lateral noise still has to reach the full 90 degrees.
    std::vector<Point2D> obs = partial_turn_right(1500.0, 50.0 * kDeg, 15);
    const double jitter[] = {4.0, -9.0, 7.0, -3.0, 11.0, -6.0, 2.0, -12.0,
                             8.0, -5.0, 6.0, -2.0, 9.0,  -7.0, 3.0, -10.0};
    for (size_t i = 0; i < obs.size(); ++i) obs[i].x += jitter[i % 16];

    std::vector<Point2D> ext = TrajectoryLatch::extend_to_turn_angle(
        obs, kRight90, kRunout, kMinR, kMaxR, kMinSpan);

    EXPECT_GT(ext.size(), obs.size());
    EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(ext) / kDeg, 90.0, 0.5);
}

TEST(TurnLatch, PathLengthMatchesArcLength) {
    std::vector<Point2D> straight = {{0.0, 0.0}, {0.0, 500.0}, {0.0, 1200.0}};
    EXPECT_NEAR(TrajectoryLatch::path_length(straight), 1200.0, 1e-6);

    double r = 1000.0;
    EXPECT_NEAR(TrajectoryLatch::path_length(quarter_turn_right(r, 512)), M_PI / 2.0 * r, 1.0);

    EXPECT_NEAR(TrajectoryLatch::path_length({}), 0.0, 1e-9);
    EXPECT_NEAR(TrajectoryLatch::path_length({{0.0, 0.0}}), 0.0, 1e-9);
}

TEST(TurnLatch, ZeroProgressLeavesGeometryUnchanged) {
    // At progress 0 the vehicle sits at the path start heading along it, which is
    // exactly the frame the path was captured in - nothing may move. The leading
    // point coincides with the vehicle and is dropped, which loses no geometry:
    // the lookahead evaluator puts the origin back.
    std::vector<Point2D> pts = {{0.0, 0.0}, {0.0, 500.0}, {200.0, 1000.0}};
    std::vector<Point2D> out = TrajectoryLatch::re_express(pts, 0.0);

    ASSERT_EQ(out.size(), 2u);
    for (size_t i = 0; i < out.size(); ++i) {
        EXPECT_NEAR(out[i].x, pts[i + 1].x, 1e-6) << "point " << i;
        EXPECT_NEAR(out[i].y, pts[i + 1].y, 1e-6) << "point " << i;
    }
    EXPECT_NEAR(followed_length(out), TrajectoryLatch::path_length(pts), 1e-6);
}

TEST(TurnLatch, AdvancingConsumesPathFromTheFront) {
    // 2000mm straight ahead. Progress lands exactly on a vertex, the case that
    // used to emit a duplicate point at the origin.
    std::vector<Point2D> pts;
    for (int i = 0; i <= 20; ++i) pts.push_back({0.0, 100.0 * i});
    double total = TrajectoryLatch::path_length(pts);
    ASSERT_NEAR(total, 2000.0, 1e-6);

    std::vector<Point2D> out = TrajectoryLatch::re_express(pts, 800.0);
    ASSERT_FALSE(out.empty());
    EXPECT_NEAR(followed_length(out), total - 800.0, 1e-6);
    EXPECT_GT(out.front().y, 0.0) << "leading point must be ahead of the vehicle";

    // Same again off-vertex, to pin the interpolation.
    out = TrajectoryLatch::re_express(pts, 850.0);
    ASSERT_FALSE(out.empty());
    EXPECT_NEAR(followed_length(out), total - 850.0, 1e-6);
    EXPECT_GT(out.front().y, 0.0);
}

TEST(TurnLatch, RemainingPathIsRotatedIntoTheCurrentVehicleFrame) {
    // The rotation is what makes an open-loop turn work at all: a quarter turn
    // taken halfway through leaves the vehicle 45 deg into the corner, and the
    // rest of the path has to be re-expressed relative to *that* heading. Skip it
    // and the controller is handed geometry off by up to 90 deg.
    double r = 1000.0;
    std::vector<Point2D> pts = quarter_turn_right(r, 512);
    double half = TrajectoryLatch::path_length(pts) / 2.0;

    std::vector<Point2D> out = TrajectoryLatch::re_express(pts, half);
    ASSERT_GE(out.size(), 2u);

    // Still the same right turn: the end of a remaining 45 deg arc of radius r
    // sits at (r*(1-cos45), r*sin45) in the new frame.
    EXPECT_NEAR(out.back().x, r * (1.0 - std::cos(M_PI / 4.0)), 5.0);
    EXPECT_NEAR(out.back().y, r * std::sin(M_PI / 4.0), 5.0);
    EXPECT_GT(out.back().x, 0.0) << "curvature must survive the transform";

    // The vehicle is on the path, so the immediate heading is straight ahead.
    EXPECT_NEAR(std::atan2(out.front().x, out.front().y), 0.0, 0.02);
}

TEST(TurnLatch, ConsumedPathReportsEmptyRatherThanTruncatingEarly) {
    std::vector<Point2D> pts = {{0.0, 0.0}, {0.0, 500.0}, {0.0, 1000.0}};

    EXPECT_TRUE(TrajectoryLatch::re_express(pts, 1500.0).empty()) << "past the end";
    EXPECT_TRUE(TrajectoryLatch::re_express(pts, 1000.0).empty()) << "exactly at the end";

    // 300mm still to go, but only one point left ahead. That is a followable path
    // once the origin is prepended - the caller must not treat it as finished, or
    // every turn loses its last stretch.
    std::vector<Point2D> tail = TrajectoryLatch::re_express(pts, 700.0);
    ASSERT_EQ(tail.size(), 1u);
    EXPECT_NEAR(followed_length(tail), 300.0, 1e-6);
}

TEST(TurnLatch, DegenerateInputsAreRejected) {
    EXPECT_TRUE(TrajectoryLatch::re_express({}, 0.0).empty());
    EXPECT_TRUE(TrajectoryLatch::re_express({{0.0, 0.0}}, 0.0).empty());

    // Repeated points carry no heading; they must not yield a NaN rotation.
    std::vector<Point2D> dup = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 600.0}};
    std::vector<Point2D> out = TrajectoryLatch::re_express(dup, 0.0);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_FALSE(std::isnan(out.front().x));
    EXPECT_FALSE(std::isnan(out.front().y));
    EXPECT_NEAR(followed_length(out), 600.0, 1e-6);
}

TEST(TurnLatch, ReplayedTurnStaysConsistentAcrossFrames) {
    // Multi-frame replay: stepping the latch must monotonically shorten what is
    // left and never flip the turn's direction. A single-frame check would miss
    // an accumulating sign or origin error - the class of bug that only shows up
    // as the vehicle drifting out of the corner halfway through.
    double r = 900.0;
    std::vector<Point2D> pts = quarter_turn_right(r, 512);
    double total = TrajectoryLatch::path_length(pts);

    double previous_remaining = total + 1.0;
    for (double s = 0.0; s < total - 50.0; s += 50.0) {
        std::vector<Point2D> out = TrajectoryLatch::re_express(pts, s);
        ASSERT_FALSE(out.empty()) << "progress " << s;

        double remaining = followed_length(out);
        EXPECT_NEAR(remaining, total - s, 1e-6) << "progress " << s;
        EXPECT_LT(remaining, previous_remaining) << "progress " << s;
        previous_remaining = remaining;

        EXPECT_GT(out.back().x, 0.0) << "turn flipped direction at progress " << s;
        EXPECT_NEAR(std::atan2(out.front().x, out.front().y), 0.0, 0.05) << "progress " << s;
    }

    // And it ends: the replay terminates instead of looping forever.
    EXPECT_TRUE(TrajectoryLatch::re_express(pts, total).empty());
}

// --- heading_at -----------------------------------------------------------
// How far the vehicle has rotated since the latch closed. The latched path is
// captured in the vehicle frame of that moment, where the vehicle heading is 0,
// so the path tangent at progress_mm *is* the rotation so far.

TEST(LatchHeadingAt, TracksArcProgress) {
    const double R = 1500.0;
    std::vector<Point2D> arc = quarter_turn_right(R, 90);

    // Arc length to angle a is R*a, so a quarter of the way round is R*pi/8.
    EXPECT_NEAR(TrajectoryLatch::heading_at(arc, 0.0) / kDeg, 0.0, 2.0);
    EXPECT_NEAR(TrajectoryLatch::heading_at(arc, R * M_PI / 8.0) / kDeg, 22.5, 2.0);
    EXPECT_NEAR(TrajectoryLatch::heading_at(arc, R * M_PI / 4.0) / kDeg, 45.0, 2.0);
}

TEST(LatchHeadingAt, SaturatesAtTerminalHeadingPastTheEnd) {
    std::vector<Point2D> arc = quarter_turn_right(1500.0, 90);
    EXPECT_NEAR(TrajectoryLatch::heading_at(arc, 1e6) / kDeg,
                TrajectoryLatch::terminal_heading_rad(arc) / kDeg, 1e-6);
}

TEST(LatchHeadingAt, DegenerateShortPathReportsNoRotation) {
    EXPECT_DOUBLE_EQ(TrajectoryLatch::heading_at({}, 500.0), 0.0);
    EXPECT_DOUBLE_EQ(TrajectoryLatch::heading_at({{0.0, 0.0}}, 500.0), 0.0);
}

// The rotation reported over a real latched path - the camera's partial arc
// carried out to the full turn - rises to the target and then holds flat
// through the straight run-out, which is what makes the run-out a settled
// window for perception rather than more turning.
TEST(LatchHeadingAt, RisesToTargetThenHoldsThroughRunout) {
    std::vector<Point2D> obs = partial_turn_right(1500.0, 50.0 * kDeg, 20);
    std::vector<Point2D> latched = TrajectoryLatch::extend_to_turn_angle(
        obs, kRight90, kRunout, kMinR, kMaxR, 15.0 * kDeg);
    double total = TrajectoryLatch::path_length(latched);

    double prev = -1e9;
    for (double s = 0.0; s <= total - kRunout; s += 100.0) {
        double h = TrajectoryLatch::heading_at(latched, s) / kDeg;
        EXPECT_GE(h, prev - 1.0) << "rotation went backwards at s=" << s;
        EXPECT_LE(h, 91.0);
        prev = h;
    }
    // Anywhere inside the run-out the vehicle is already fully turned.
    EXPECT_NEAR(TrajectoryLatch::heading_at(latched, total - kRunout / 2.0) / kDeg, 90.0, 1.5);
}

// --- turn_complete --------------------------------------------------------
// Release gate. Both halves are needed: alignment alone fires at the start of
// the turn on the old lane, progress alone fires at the end into an empty frame.

namespace {
constexpr double kSpanFrac = 0.7;                  // production default
constexpr double kMaxLaneHeading = 25.0 * kDeg;    // production default

bool release(double turned_deg, double target_deg, bool lane, double lane_deg) {
    return TrajectoryLatch::turn_complete(turned_deg * kDeg, target_deg * kDeg,
                                          kSpanFrac, lane, lane_deg * kDeg,
                                          kMaxLaneHeading);
}
}  // namespace

// Why the span gate is fed measured odom yaw and not heading_at(path, progress).
//
// heading_at returns the angle of whichever single 100mm segment of the observed
// lane centreline progress_mm happens to land in. On a real path that reading is
// not monotone and is not bounded by the path's own turn: logged on the vehicle
// 2026-08-05, a latched left turn whose whole frozen path spans 61.8 degrees had
// heading_at report 70.1 at the moment the latch released.
//
// A gate thresholding that value is therefore unsound - it can open on rotation
// the frozen path does not contain at all. Here the spike clears 0.7 x 90 = 63
// while the entire path, ridden to its very end, never would.
TEST(LatchReleaseGate, ChordSpikeOpensTheGateOnRotationThePathNeverContains) {
    const double kPathSpanDeg = 61.8;   // terminal_heading_rad of the whole path
    const double kChordSpikeDeg = 70.1; // what heading_at reported mid-path
    const double kAlignedLaneDeg = 10.0;

    EXPECT_TRUE(release(kChordSpikeDeg, 90.0, true, kAlignedLaneDeg));
    EXPECT_FALSE(release(kPathSpanDeg, 90.0, true, kAlignedLaneDeg))
        << "riding the frozen path to its end never reaches the span threshold, "
           "so any release driven by it came from chord noise";
}

// The measured-yaw feed cannot produce that spike: it is a rotation the vehicle
// either performed or did not. Replaying the odom column of a logged turn, the
// gate stays shut until the vehicle has genuinely turned far enough, and once it
// opens it stays open - no flicker back and forth.
TEST(LatchReleaseGate, MeasuredYawDrivesTheGateMonotonically) {
    // Odom yaw change per frame through the logged right turn of 2026-08-05,
    // with the old lane swinging away from the vehicle as it goes.
    const double measured_deg[] = {1.5,  7.9,  21.9, 28.1, 32.3, 37.4,
                                   41.1, 41.9, 61.8, 72.8, 84.3, 90.3, 96.2};
    bool opened = false;
    for (double turned : measured_deg) {
        // Lane alignment improves as the vehicle comes round onto the new road.
        double lane_deg = std::max(0.0, 90.0 - turned) * 0.3;
        bool r = release(turned, 90.0, true, lane_deg);
        if (opened) EXPECT_TRUE(r) << "gate flickered shut again at " << turned;
        if (r) opened = true;
        if (turned < 63.0) EXPECT_FALSE(r) << "opened at only " << turned << " degrees";
    }
    EXPECT_TRUE(opened) << "the turn completed without the gate ever opening";
}

// The case that rules out releasing on lane visibility alone. The latch closes
// exactly when the turn-lane leaves view, and at that moment the old main lane
// running straight through the junction is both visible and perfectly aligned.
TEST(LatchReleaseGate, OldLaneStraightAheadAtTurnStartDoesNotRelease) {
    EXPECT_FALSE(release(0.0, 90.0, true, 0.0));
    EXPECT_FALSE(release(5.0, 90.0, true, 0.0));
    EXPECT_FALSE(release(30.0, 90.0, true, 2.0));
}

// The case that rules out releasing on turn progress alone.
TEST(LatchReleaseGate, FullyTurnedButNoLaneDoesNotRelease) {
    EXPECT_FALSE(release(90.0, 90.0, false, 0.0));
}

// Late in the turn the lane the vehicle came from has swung round to lie across
// the vehicle, so it cannot be mistaken for the new road.
TEST(LatchReleaseGate, ObliqueLaneLateInTurnDoesNotRelease) {
    EXPECT_FALSE(release(80.0, 90.0, true, 85.0));
    EXPECT_FALSE(release(80.0, 90.0, true, -60.0));
}

TEST(LatchReleaseGate, TurnedFarEnoughWithAlignedRoadAheadReleases) {
    EXPECT_TRUE(release(75.0, 90.0, true, 5.0));
    EXPECT_TRUE(release(90.0, 90.0, true, -20.0));
}

TEST(LatchReleaseGate, SpanGateIsAFractionOfTheTarget) {
    // 0.7 * 90 = 63 degrees.
    EXPECT_FALSE(release(62.0, 90.0, true, 0.0));
    EXPECT_TRUE(release(64.0, 90.0, true, 0.0));
}

// Left turns carry negative headings throughout; the gate compares magnitudes
// so it must behave identically mirrored.
TEST(LatchReleaseGate, LeftTurnMirrorsRight) {
    EXPECT_FALSE(release(-30.0, -90.0, true, 0.0));
    EXPECT_TRUE(release(-75.0, -90.0, true, -5.0));
    EXPECT_FALSE(release(-80.0, -90.0, true, -85.0));
}

TEST(LatchReleaseGate, NoTargetNeverReleases) {
    EXPECT_FALSE(release(90.0, 0.0, true, 0.0));
}

// Walking a real latched turn frame by frame: the gate must stay shut for the
// whole approach even with an aligned lane in view every single frame, and open
// only once the rotation is genuinely there. A single-frame check cannot show
// this - the failure being guarded against is precisely an early frame firing.
TEST(LatchReleaseGate, StaysShutAcrossTheWholeApproachWithALaneAlwaysInView) {
    std::vector<Point2D> obs = partial_turn_right(1500.0, 50.0 * kDeg, 20);
    std::vector<Point2D> latched = TrajectoryLatch::extend_to_turn_angle(
        obs, kRight90, kRunout, kMinR, kMaxR, 15.0 * kDeg);
    double total = TrajectoryLatch::path_length(latched);

    bool released = false;
    double released_at_deg = 0.0;
    for (double s = 0.0; s <= total; s += 50.0) {
        double turned = TrajectoryLatch::heading_at(latched, s);
        if (TrajectoryLatch::turn_complete(turned, kRight90, kSpanFrac, true, 0.0,
                                           kMaxLaneHeading)) {
            released = true;
            released_at_deg = turned / kDeg;
            break;
        }
    }
    ASSERT_TRUE(released) << "gate never opened on a completed turn";
    EXPECT_GE(released_at_deg, 63.0);
}

// --- bridge_gap_to_lane ---------------------------------------------------
// Spanning the empty stretch in front of the vehicle when the lane is only
// picked up further ahead - typically the frames just after a turn. The whole
// point is that it adds geometry without removing any: plan_transition, which
// used to serve this, splits the target at 1200mm and discards everything
// before it, and halves the observation outright when it is shorter than that.

TEST(BridgeGapToLane, KeepsEveryWaypointAndStartsAtTheVehicle) {
    std::vector<Point2D> lane = {{0.0, 1200.0}, {100.0, 1600.0}, {250.0, 2000.0}};
    std::vector<Point2D> bridged = TrajectoryPlanner::bridge_gap_to_lane(lane);

    ASSERT_GE(bridged.size(), lane.size());
    EXPECT_NEAR(bridged.front().x, 0.0, 1e-9);
    EXPECT_NEAR(bridged.front().y, 0.0, 1e-9);

    // Every observed waypoint survives, in order, byte for byte.
    size_t tail = bridged.size() - lane.size();
    for (size_t i = 0; i < lane.size(); ++i) {
        EXPECT_DOUBLE_EQ(bridged[tail + i].x, lane[i].x);
        EXPECT_DOUBLE_EQ(bridged[tail + i].y, lane[i].y);
    }
}

// A kink at the join would put a curvature spike exactly where the controller
// hands over from connector to centreline.
TEST(BridgeGapToLane, ArrivesAlongTheLaneTangent) {
    std::vector<Point2D> lane = {{300.0, 1200.0}, {500.0, 1600.0}, {700.0, 2000.0}};
    std::vector<Point2D> bridged = TrajectoryPlanner::bridge_gap_to_lane(lane);
    ASSERT_GE(bridged.size(), 3u);

    size_t j = bridged.size() - lane.size();  // index of lane.front() in the result
    ASSERT_GE(j, 1u);
    double conn = std::atan2(bridged[j].x - bridged[j - 1].x, bridged[j].y - bridged[j - 1].y);
    double lane_dir = std::atan2(lane[1].x - lane[0].x, lane[1].y - lane[0].y);
    EXPECT_NEAR(conn / kDeg, lane_dir / kDeg, 5.0) << "connector must arrive along the lane";
}

TEST(BridgeGapToLane, LaneAlreadyAtTheVehicleIsReturnedUnchanged) {
    std::vector<Point2D> lane = {{0.0, 0.0}, {0.0, 500.0}, {0.0, 1000.0}};
    EXPECT_EQ(TrajectoryPlanner::bridge_gap_to_lane(lane).size(), lane.size());
}

TEST(BridgeGapToLane, RefusesGeometryItCannotHonestlySpan) {
    // Runs back towards the vehicle: no connector, keep the raw observation.
    EXPECT_TRUE(TrajectoryPlanner::bridge_gap_to_lane(
        {{0.0, 1200.0}, {0.0, 700.0}, {0.0, 200.0}}).empty());
    // Off to the side beyond any plausible lane offset.
    EXPECT_TRUE(TrajectoryPlanner::bridge_gap_to_lane(
        {{2200.0, 1200.0}, {2200.0, 1600.0}}).empty());
    // Nothing to bridge to.
    EXPECT_TRUE(TrajectoryPlanner::bridge_gap_to_lane({{0.0, 1200.0}}).empty());
}

// End to end through the planner: the short, far lane that a freshly completed
// turn hands over. This is the case plan_transition halved.
TEST(FollowMainBridge, ShortFarLaneKeepsItsCentreline) {
    // Sampled at 100mm along a gentle curve, the way ipm_transform_node's
    // extract_centerline_waypoints_y actually delivers a centreline. Total span
    // is under 1200mm, so this is exactly the observation plan_transition's
    // fallback used to cut in half.
    std::vector<Point2D> lane;
    for (int i = 0; i <= 8; ++i) {
        double y = 1200.0 + i * 100.0;
        lane.push_back({0.0002 * (y - 1200.0) * (y - 1200.0), y});
    }
    json waypoints = json::array();
    for (const auto& p : lane) waypoints.push_back(json::array({p.x, p.y}));

    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_far_short"}, {"label", 6}, {"class_name", "main-lane"},
             {"waypoints", waypoints}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_follow_main(obs, prev_state, last_main_id);

    ASSERT_TRUE(planned.valid);
    EXPECT_NEAR(planned.points.front().x, 0.0, 1.0) << "path must start at the vehicle";
    EXPECT_NEAR(planned.points.front().y, 0.0, 1.0);

    // No waypoint was dropped: each one still lies on the published path.
    for (const auto& wp : lane) {
        EXPECT_LT(distance_to_polyline(wp, planned.points), 1.0)
            << "waypoint (" << wp.x << "," << wp.y << ") was discarded from the path";
    }

    // And past the gap the path is the centreline, not a Bezier through it.
    for (const auto& p : planned.points) {
        if (p.y < lane.front().y) continue;
        EXPECT_LT(distance_to_polyline(p, lane), 1.0)
            << "path left the centreline at y=" << p.y;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// EgoMotion - re-expressing remembered paths in the current vehicle frame.
//
// The world model these tests use: world axes are the vehicle axes at yaw 0
// (X to the right, Y forward), and ROS yaw psi is CCW-positive, so the vehicle's
// forward direction is (-sin psi, cos psi). Converting a world point into the
// vehicle frame at yaw psi is therefore a rotation by -psi, which is exactly
// rotate_into_frame(p, frame_delta_from_ros_yaw(psi, 0)). That identity is what
// makes the compensation correct, so it is worth stating explicitly.
// ─────────────────────────────────────────────────────────────────────────────

namespace {

// A world point as seen from a vehicle sitting at the world origin at ROS yaw
// psi. Written independently of EgoMotion so the tests below are checking the
// header against geometry, not against itself.
Point2D world_to_vehicle(const Point2D& p, double psi) {
    return {p.x * std::cos(psi) + p.y * std::sin(psi),
            -p.x * std::sin(psi) + p.y * std::cos(psi)};
}

}  // namespace

TEST(EgoMotion, ZeroDeltaLeavesPathUntouched) {
    std::vector<Point2D> pts = {{0.0, 0.0}, {50.0, 500.0}, {-30.0, 1200.0}};
    auto out = EgoMotion::rotate_into_frame(pts, 0.0);
    ASSERT_EQ(out.size(), pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        EXPECT_NEAR(out[i].x, pts[i].x, 1e-9);
        EXPECT_NEAR(out[i].y, pts[i].y, 1e-9);
    }
}

TEST(EgoMotion, QuarterTurnMapsAxes) {
    // delta = +90deg: a point straight ahead lands on the left, a point on the
    // right lands straight ahead.
    std::vector<Point2D> pts = {{0.0, 1000.0}, {1000.0, 0.0}};
    auto out = EgoMotion::rotate_into_frame(pts, M_PI / 2.0);
    EXPECT_NEAR(out[0].x, -1000.0, 1e-6);
    EXPECT_NEAR(out[0].y, 0.0, 1e-6);
    EXPECT_NEAR(out[1].x, 0.0, 1e-6);
    EXPECT_NEAR(out[1].y, 1000.0, 1e-6);
}

TEST(EgoMotion, YawFromQuaternionMatchesRotation) {
    for (double psi : {0.0, 0.5, -0.9, 2.5, -3.0}) {
        double z = std::sin(psi / 2.0);
        double w = std::cos(psi / 2.0);
        EXPECT_NEAR(EgoMotion::yaw_from_quaternion(z, w), psi, 1e-9) << "psi=" << psi;
    }
}

TEST(EgoMotion, DeltaWrapsAcrossPi) {
    // Crossing +pi: 3.0 -> -3.0 rad is ROS yaw continuing to INCREASE by 0.283
    // rad and wrapping round, i.e. a small further LEFT turn - not a 6 rad
    // lurch back to the right. Getting this wrong once per lap would fling the
    // remembered path most of a full turn sideways.
    double d = EgoMotion::frame_delta_from_ros_yaw(-3.0, 3.0);
    EXPECT_NEAR(std::abs(d), 2.0 * M_PI - 6.0, 1e-9);
    EXPECT_LT(d, 0.0) << "a left ego turn must give a negative frame delta";

    // And the other way across -pi.
    double d2 = EgoMotion::frame_delta_from_ros_yaw(3.0, -3.0);
    EXPECT_NEAR(d2, -d, 1e-9);
}

TEST(EgoMotion, SignAgreesWithPathHeadingConvention) {
    // The trap this whole header exists around. A straight-ahead path
    // remembered at yaw 0; the vehicle then turns LEFT by 20 deg (ROS yaw
    // +20deg). From the new frame that path now runs off to the RIGHT, and
    // TrajectoryLatch::terminal_heading_rad - atan2(dx, dy), right-positive -
    // must report +20 deg.
    std::vector<Point2D> straight = {{0.0, 0.0}, {0.0, 500.0}, {0.0, 1000.0}};
    double delta = EgoMotion::frame_delta_from_ros_yaw(20.0 * M_PI / 180.0, 0.0);
    auto out = EgoMotion::rotate_into_frame(straight, delta);

    double heading = TrajectoryLatch::terminal_heading_rad(out);
    EXPECT_NEAR(heading * 180.0 / M_PI, 20.0, 1e-6)
        << "a left ego turn must swing the remembered path right by the same angle";

    // A right ego turn is the mirror image.
    double delta_r = EgoMotion::frame_delta_from_ros_yaw(-20.0 * M_PI / 180.0, 0.0);
    auto out_r = EgoMotion::rotate_into_frame(straight, delta_r);
    EXPECT_NEAR(TrajectoryLatch::terminal_heading_rad(out_r) * 180.0 / M_PI, -20.0, 1e-6);
}

TEST(EgoMotion, HeldPathStaysOnLaneAcrossAFastTurn) {
    // The failure the user reported, reproduced as a multi-frame replay: the
    // vehicle spins (pure rotation, so this isolates the heading term the
    // existing along-path correction cannot express) while the planner holds a
    // remembered path instead of replanning. The lane is fixed in the world.
    //
    // 12 deg per frame at ~11 FPS is about 130 deg/s - a hard turn into a
    // turn-lane, which is where the drift was seen.
    std::vector<Point2D> lane_world;
    for (int i = 0; i <= 20; ++i) lane_world.push_back({0.0, 100.0 * i});

    const double step = 12.0 * M_PI / 180.0;
    double psi = 0.0;

    // What the planner captured on frame 0.
    std::vector<Point2D> held;
    std::vector<Point2D> uncompensated;
    for (const auto& p : lane_world) {
        held.push_back(world_to_vehicle(p, psi));
        uncompensated.push_back(world_to_vehicle(p, psi));
    }

    for (int frame = 1; frame <= 5; ++frame) {
        double psi_new = psi + step;  // turning left
        held = EgoMotion::rotate_into_frame(
            held, EgoMotion::frame_delta_from_ros_yaw(psi_new, psi));
        psi = psi_new;

        // Ground truth: the same world lane seen from the vehicle now.
        std::vector<Point2D> truth;
        for (const auto& p : lane_world) truth.push_back(world_to_vehicle(p, psi));

        for (const auto& p : held) {
            EXPECT_LT(distance_to_polyline(p, truth), 1.0)
                << "compensated path left the lane on frame " << frame;
        }
    }

    // And the drift the compensation removes: after 60 deg the un-rotated
    // memory is most of a metre off the lane at the far end. Anything under
    // this and the test would not be proving much.
    std::vector<Point2D> truth_final;
    for (const auto& p : lane_world) truth_final.push_back(world_to_vehicle(p, psi));
    EXPECT_GT(distance_to_polyline(uncompensated.back(), truth_final), 500.0)
        << "fixture is too gentle to demonstrate the drift";
}

TEST(TurnLatchReExpress, MeasuredOverRotationTiltsTheEmittedPathBack) {
    // The open-loop hole this closes. The frozen path is replayed as though the
    // vehicle were sitting on it heading along its tangent; when the vehicle has
    // actually rotated further than that, the emitted path has to lean back by
    // the difference, or it asks for the same curvature that caused the
    // over-rotation. Measured 2026-08-05 (run13): frozen paths carrying exactly
    // 90 degrees produced 67, 83 and 214 degrees of real rotation.
    std::vector<Point2D> path = partial_turn_right(800.0, 90.0 * kDeg, 40);
    const double progress = 300.0;

    std::vector<Point2D> open_loop = TrajectoryLatch::re_express(path, progress);
    ASSERT_GE(open_loop.size(), 2u);
    double h_open = std::atan2(open_loop[1].x - open_loop[0].x,
                               open_loop[1].y - open_loop[0].y);

    // Over-rotated by 20 degrees: the path ahead must read 20 degrees less
    // turned, so the controller stops adding the rotation that is already there.
    double over = 20.0 * kDeg;
    std::vector<Point2D> corrected = TrajectoryLatch::re_express(path, progress, over);
    ASSERT_EQ(corrected.size(), open_loop.size());
    double h_corr = std::atan2(corrected[1].x - corrected[0].x,
                               corrected[1].y - corrected[0].y);
    EXPECT_NEAR((h_corr - h_open) / kDeg, -20.0, 0.5);

    // Under-rotated tilts the other way by the same construction.
    std::vector<Point2D> under = TrajectoryLatch::re_express(path, progress, -over);
    double h_under = std::atan2(under[1].x - under[0].x, under[1].y - under[0].y);
    EXPECT_NEAR((h_under - h_open) / kDeg, 20.0, 0.5);

    // Shape is preserved - this rotates the frame, it does not reshape the path.
    EXPECT_NEAR(TrajectoryLatch::path_length(corrected),
                TrajectoryLatch::path_length(open_loop), 1.0);
}

TEST(TurnLatchReExpress, ZeroCorrectionIsExactlyTheOpenLoopBehaviour) {
    // The parameter defaults to zero and the disable path must be bit-for-bit
    // the old behaviour, so a bad odometry reading can be switched out of the
    // loop on the vehicle without changing anything else.
    std::vector<Point2D> path = partial_turn_right(800.0, 90.0 * kDeg, 40);
    for (double progress : {0.0, 250.0, 700.0}) {
        std::vector<Point2D> a = TrajectoryLatch::re_express(path, progress);
        std::vector<Point2D> b = TrajectoryLatch::re_express(path, progress, 0.0);
        ASSERT_EQ(a.size(), b.size());
        for (size_t i = 0; i < a.size(); ++i) {
            EXPECT_DOUBLE_EQ(a[i].x, b[i].x);
            EXPECT_DOUBLE_EQ(a[i].y, b[i].y);
        }
    }
}

TEST(PlanTurnAnchor, FarSideMainLaneDoesNotAnchorTheTurn) {
    // Geometry logged off the vehicle 2026-08-05 (run16). Approaching the
    // junction the ego main lane leaves the BEV window and select_main_current
    // latches onto the segment across the intersection - here starting 600mm
    // ahead and running away to the right while the turn marking sits left.
    // Anchoring the turn on that produced a connector that ran across the
    // junction, down the far lane, and only then hooked back to the marking:
    // out to 1076mm and back to 552mm, aimed initially away from the turn.
    json telemetry = {
        {"timestamp_ms", 1000},
        {"objects", json::array({
            {{"id", "main_far"}, {"label", 6}, {"class_name", "main-lane"}, {"confidence", 0.9},
             {"waypoints", json::array({{184.0, 600.0}, {300.0, 900.0},
                                        {420.0, 1200.0}, {540.0, 1500.0}})}},
            {{"id", "turn_left_lane"}, {"label", 20}, {"class_name", "turn-lane"}, {"confidence", 0.85},
             {"waypoints", json::array({{0.0, 800.0}, {-200.0, 1000.0},
                                        {-450.0, 1100.0}, {-700.0, 1150.0}})}},
        })},
    };
    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;

    PlannedTrajectory planned =
        TrajectoryPlanner::plan_turn_left(obs, prev_state, false, false, last_main_id);
    ASSERT_TRUE(planned.valid);
    ASSERT_GE(planned.points.size(), 4u);

    // Anchored at the vehicle, not out where the far lane happens to start.
    EXPECT_LT(planned.points.front().y, 300.0);

    // And it goes somewhere and stays there: the point farthest from the vehicle
    // is the end of the path, not a corner it turns round and comes back from.
    size_t farthest = 0;
    double best = 0.0;
    for (size_t i = 0; i < planned.points.size(); ++i) {
        double d = std::hypot(planned.points[i].x, planned.points[i].y);
        if (d > best) { best = d; farthest = i; }
    }
    double tail = std::hypot(planned.points.back().x, planned.points.back().y);
    EXPECT_LT(best - tail, 300.0)
        << "path doubles back: farthest " << best << "mm at index " << farthest
        << " of " << planned.points.size() - 1 << ", ends at " << tail << "mm";
}

TEST(TurnLatchSkipLead, LeadPushesTheProjectedAngleTowardsTheTarget) {
    // The run-out skip compares |turned + lead| against the target, where lead
    // extrapolates measured yaw over one frame. turned is vehicle-frame
    // (right-positive) and yaw_rate is ROS yaw (CCW-positive, so left-positive):
    // adding the raw rate would move the projection AWAY from the target and
    // fire the skip later than reading the frame alone, which is the opposite of
    // the point. Converting through the helper is what makes the sign right, and
    // nothing else in the build catches getting it backwards.
    const double lead_s = 0.1;

    // Left turn: vehicle-frame angle is negative and growing more negative,
    // ROS yaw rate is positive.
    double turned_left = -80.0 * M_PI / 180.0;
    double rate_left = 60.0 * M_PI / 180.0;   // deg/s, CCW
    double lead_left = EgoMotion::frame_delta_from_ros_yaw(rate_left * lead_s, 0.0);
    EXPECT_GT(std::abs(turned_left + lead_left), std::abs(turned_left))
        << "lead must project a left turn further into the turn, not out of it";

    // Right turn: mirror image, and the same expression must still hold.
    double turned_right = 80.0 * M_PI / 180.0;
    double rate_right = -60.0 * M_PI / 180.0;  // deg/s, CW
    double lead_right = EgoMotion::frame_delta_from_ros_yaw(rate_right * lead_s, 0.0);
    EXPECT_GT(std::abs(turned_right + lead_right), std::abs(turned_right))
        << "lead must project a right turn further into the turn, not out of it";

    // Sized as advertised: 60 deg/s over 0.1s is 6 degrees of projection.
    EXPECT_NEAR(std::abs(lead_left) * 180.0 / M_PI, 6.0, 1e-9);

    // A turn that is barely moving gains almost nothing, so the lead cannot
    // trip the skip on a turn that was never about to reach the target.
    double lead_slow = EgoMotion::frame_delta_from_ros_yaw(
        (5.0 * M_PI / 180.0) * lead_s, 0.0);
    EXPECT_LT(std::abs(lead_slow) * 180.0 / M_PI, 1.0);
}

TEST(TurnLatchRunoutCorrection, ReferenceOnTheRunOutIsTheTargetHeading) {
    // The correction is only sound where the tangent it measures against carries
    // no min_radius distortion. Build a latch path the way extend_to_turn_angle
    // does - a 90 degree arc followed by a straight run-out - and check that
    // heading_at reads the target on the run-out but something short of it part
    // way round the arc. That difference is the whole reason this is gated on
    // progress: on the arc, "turned - assumed" mixes real drift with the clamp,
    // on the run-out it is the over-rotation alone.
    const double R = 800.0;
    const double runout_mm = 700.0;
    std::vector<Point2D> path;
    for (int i = 0; i <= 90; ++i) {
        double a = i * M_PI / 180.0;
        path.push_back({R * (1.0 - std::cos(a)), R * std::sin(a)});   // right turn
    }
    Point2D end = path.back();
    for (int i = 1; i <= 7; ++i) {
        path.push_back({end.x + i * 100.0, end.y});                   // straight, heading +90
    }
    double arc_len = R * M_PI / 2.0;

    double on_runout = TrajectoryLatch::heading_at(path, arc_len + runout_mm / 2.0);
    EXPECT_NEAR(on_runout * 180.0 / M_PI, 90.0, 2.0)
        << "run-out tangent must be the target heading, so the correction there "
           "measures over-rotation and nothing else";

    double mid_arc = TrajectoryLatch::heading_at(path, arc_len / 2.0);
    EXPECT_LT(std::abs(mid_arc * 180.0 / M_PI), 60.0)
        << "mid-arc tangent is far from target, which is why the same subtraction "
           "there does not mean over-rotation";

    // And the gate itself: progress past length - runout selects the run-out.
    double total = arc_len + 700.0;
    EXPECT_GE(arc_len + runout_mm / 2.0, total - runout_mm);
    EXPECT_LT(arc_len / 2.0, total - runout_mm);
}

TEST(TurnBulgeAsymmetry, RightTurnDoesNotLeaveTheVehicleAimedAwayFromTheTurn) {
    // The outward belly is right for a left turn and wrong for a right one. A
    // left turn crosses the intersection, so holding heading and bending late
    // uses room that is actually there; a right turn is taken from the right
    // lane around the near corner, where the same shape aims the connector away
    // from the turn before it comes back. Measured on the vehicle (run24): right
    // turns published theta_rad of -0.05 to -0.12 - a left-hand heading under a
    // right-turn intent - and the controller answered with 0.50 rad/s of left
    // rotation before any right rotation began.
    //
    // Locks the asymmetry itself, not a number, so re-tuning either multiplier
    // keeps working and collapsing them back into one does not.
    // How far LEFT of the vehicle the path strays, which on a right turn is the
    // drift that has to be undone before any right rotation can start. Measured
    // over the whole path rather than a fixed distance: plan_transition keeps
    // the entry path's own points as a prefix, so a window near the vehicle
    // sits in geometry the bulge never touches and reads identical for every
    // multiplier.
    auto leftmost = [](const std::vector<Point2D>& pts) {
        double m = 0.0;
        for (const auto& p : pts) m = std::min(m, p.x);
        return m;   // <= 0; more negative = strays further from a right turn
    };

    // Straight ego stub, and a turn lane peeling off to the right.
    std::vector<Point2D> ego;
    for (int i = 0; i <= 10; ++i) ego.push_back({0.0, i * 100.0});
    std::vector<Point2D> target_right;
    for (int i = 0; i <= 10; ++i) {
        double a = i * (M_PI / 2.0) / 10.0;
        target_right.push_back({600.0 * (1.0 - std::cos(a)) + 200.0,
                                600.0 * std::sin(a) + 800.0});
    }

    std::vector<Point2D> outward = TrajectoryPlanner::plan_transition(
        ego, target_right, 110.0 * M_PI / 180.0, 1.5, 0.355);
    std::vector<Point2D> plain = TrajectoryPlanner::plan_transition(
        ego, target_right, 110.0 * M_PI / 180.0, 1.5, 0.0);
    std::vector<Point2D> inward = TrajectoryPlanner::plan_transition(
        ego, target_right, 110.0 * M_PI / 180.0, 1.5, -0.355);
    ASSERT_GE(outward.size(), 4u);
    ASSERT_GE(plain.size(), 4u);
    ASSERT_GE(inward.size(), 4u);

    double x_out = leftmost(outward);
    double x_plain = leftmost(plain);
    double x_in = leftmost(inward);

    // Ordering is the property that matters, not any single number: each step
    // toward a negative multiplier must leave the path straying LESS far from
    // the turn. Re-tuning either multiplier keeps this true; collapsing the two
    // back into one value does not.
    EXPECT_GT(x_plain, x_out)
        << "removing the outward belly must reduce the leftward stray on a right "
           "turn (out=" << x_out << " plain=" << x_plain << ")";
    EXPECT_GE(x_in, x_plain)
        << "a negative multiplier must not stray further left than none at all "
           "(plain=" << x_plain << " in=" << x_in << ")";

    // The outward shape really does stray left on a right turn - that is the
    // behaviour measured on the vehicle, and what the default now avoids.
    EXPECT_LT(x_out, -20.0)
        << "fixture does not exercise the outward belly at all (x_out=" << x_out << ")";
}

TEST(TurnBulgeAsymmetry, RightTurnSitsBetweenTheTightAndWideShapes) {
    // The right-turn connector has to live between two shapes that were both
    // driven and both wrong.
    //
    // The pre-360c6b0 geometry (handle 1.0, no bulge) fixed the wide arc on the
    // measurements - emitted radius fell from a 741mm median to 475-588mm, and
    // run28 released a right turn at 92.8 degrees with no run-out skip needed -
    // but on the mat it hugs the corner too tightly to actually drive round.
    // The left-turn tuning (1.5 / 0.355) swings out through the middle of the
    // junction, which a right turn taken from the right lane has no room for.
    //
    // So this pins the ordering rather than any single value: the right-turn
    // defaults must curve into the junction more than the tight shape does, and
    // less than the left-turn tuning does. Re-tuning within that band keeps
    // passing; collapsing the directions back onto one value does not.
    std::vector<Point2D> ego;
    for (int i = 0; i <= 10; ++i) ego.push_back({0.0, i * 100.0});
    std::vector<Point2D> target_right;
    for (int i = 0; i <= 10; ++i) {
        double a = i * (M_PI / 2.0) / 10.0;
        target_right.push_back({600.0 * (1.0 - std::cos(a)) + 200.0,
                                600.0 * std::sin(a) + 800.0});
    }
    const double kMaxHeading = 110.0 * M_PI / 180.0;

    std::vector<Point2D> tight = TrajectoryPlanner::plan_transition(
        ego, target_right, kMaxHeading, 1.0, 0.0);
    std::vector<Point2D> current = TrajectoryPlanner::plan_transition(
        ego, target_right, kMaxHeading,
        TrajectoryPlanner::turn_bezier_handle_scale_mult_right,
        TrajectoryPlanner::turn_lateral_bulge_mult_right);
    std::vector<Point2D> wide = TrajectoryPlanner::plan_transition(
        ego, target_right, kMaxHeading,
        TrajectoryPlanner::turn_bezier_handle_scale_mult,
        TrajectoryPlanner::turn_lateral_bulge_mult);
    ASSERT_GE(tight.size(), 4u);
    ASSERT_GE(current.size(), 4u);
    ASSERT_GE(wide.size(), 4u);

    // How far into the turn the path has moved by a fixed distance ahead of the
    // vehicle. A longer Bezier handle holds the entry heading further before
    // bending, so it reads SMALLER here - that is the wide shape. Measured at
    // 800mm, inside the stretch the controller actually steers on.
    auto offset_at = [](const std::vector<Point2D>& pts, double y_at) {
        for (size_t i = 1; i < pts.size(); ++i) {
            if (pts[i].y >= y_at) {
                double span = pts[i].y - pts[i - 1].y;
                double f = span > 1e-9 ? (y_at - pts[i - 1].y) / span : 0.0;
                return pts[i - 1].x + f * (pts[i].x - pts[i - 1].x);
            }
        }
        return pts.back().x;
    };
    double x_tight = offset_at(tight, 800.0);
    double x_cur = offset_at(current, 800.0);
    double x_wide = offset_at(wide, 800.0);

    EXPECT_LT(x_cur, x_tight)
        << "right turn must not cut into the corner as hard as the bare tight "
           "shape (tight=" << x_tight << " current=" << x_cur << ")";
    EXPECT_GT(x_cur, x_wide)
        << "right turn must still commit to the turn earlier than the left-turn "
           "tuning does (current=" << x_cur << " wide=" << x_wide << ")";
}
