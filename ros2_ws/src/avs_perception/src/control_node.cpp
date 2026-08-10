/**
 * control_node.cpp
 *
 * AVS Lane Error Publisher — Computes and publishes the 3 control error
 * parameters in the vehicle frame coordinate system:
 *
 *   Vehicle Frame (origin O = bottom-center of camera frame projected to ground):
 *     X — lateral  (right = positive, left = negative)
 *     Y — forward  (ahead = positive)
 *
 *   Control errors published:
 *     epsilon_x_mm   : lateral deviation  = x-coordinate of look-ahead waypoint
 *     epsilon_y_mm   : longitudinal deviation = y-coordinate of look-ahead waypoint
 *     theta_rad      : heading error = angle of line (O → waypoint) from Y-axis
 *                      = atan2(epsilon_x, epsilon_y)
 *
 * Lane selection state (which lane's waypoints serve as setpoint to origin O):
 *   FOLLOW_MAIN  : main-lane centerline is the setpoint
 *   LANE_CHANGE  : other-lane centerline is the setpoint
 *   TURNING      : turn-lane centerline is the setpoint
 *
 * Subscriptions:
 *   /avs/telemetry_realworld  (std_msgs/String JSON) — pre-computed look-ahead errors
 *   /avs/cmd                  (std_msgs/String JSON) — {"cmd": "lane_change"|"turn"|"resume"}
 *
 * Publications:
 *   /avs/control_error        (std_msgs/String JSON) — {epsilon_x_mm, epsilon_y_mm, theta_rad, ...}
 *   /avs/lane_state           (std_msgs/String JSON) — current lane selection state
 */

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <vector>
#include <limits>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nlohmann/json.hpp"
#include "avs_perception/label_mapping.hpp"
#include "avs_perception/decision_types.hpp"
#include "avs_perception/path_observation.hpp"
#include "avs_perception/trajectory_planner.hpp"
#include "avs_perception/trajectory_normalizer.hpp"
#include "avs_perception/trajectory_manager.hpp"
#include "avs_perception/trajectory_latch.hpp"
#include "avs_perception/ego_motion.hpp"
#include "avs_perception/legacy_lane_model.hpp"
#include "avs_perception/lane_legality.hpp"

using json = nlohmann::json;
using namespace avs_perception;

// ─────────────────────────────────────────────────────────────────────────────
// LaneErrorNode
// ─────────────────────────────────────────────────────────────────────────────
class LaneErrorNode : public rclcpp::Node {
public:
    LaneErrorNode() : Node("control_node") {
        // ── Declare turn trigger thresholds ──────────────────────────────────
        // These determine when to switch to turn-lane errors.
        // The actual PD controller is a separate node.
        this->declare_parameter<double>("turn_proximity_mm",  500.0);
        this->declare_parameter<double>("turn_done_mm",       200.0);
        this->declare_parameter<double>("theta_done_rad",     0.1);
        this->declare_parameter<int>("maneuver_dropout_hold_frames", 10);
        this->declare_parameter<int>("intent_abort_frames", 30);
        this->declare_parameter<double>("maneuver_max_duration_s", 10.0);
        // Frozen turn execution: how far the latched path is carried past what
        // the camera saw. 90 degrees because every junction on the track is
        // square; min_observed_span is the floor below which the observation is
        // noise rather than an arc, and the radius bounds stop a degenerate
        // detection curling the extension into a hairpin or a near-straight line.
        this->declare_parameter<double>("turn_latch_target_heading_deg", 90.0);
        this->declare_parameter<double>("turn_latch_runout_mm", 700.0);
        // How far ahead the run-out skip extrapolates measured yaw. Sized to one
        // median gap between telemetry frames during a latch (0.075s measured
        // over run19-21, 0.198s at the 99th percentile), because that gap is what
        // the skip is late by - see update_turn_latch. Zero restores deciding on
        // the frame's own reading.
        this->declare_parameter<double>("turn_latch_skip_lead_s", 0.1);
        // Floor on the fitted radius. Real markings on this map fit tighter than
        // this - six turns logged 2026-08-05 came out at 362-459mm - so the floor
        // genuinely reshapes them, and it has to: this value must stay above the
        // pure-pursuit lookahead, which ipm_transform_node publishes at 600mm.
        // Once the path's radius of curvature drops below the lookahead the
        // controller's geometry degenerates - the aim point sits across the
        // circle rather than along it - and steering saturates. Tried at 350.0 on
        // the vehicle for exactly that reason and it was much worse: 90-degree
        // junctions came out at 154 degrees of rotation. Do not lower this
        // without lowering the lookahead with it.
        this->declare_parameter<double>("turn_latch_min_radius_mm", 800.0);
        // Junction turns on this map fit circles of roughly 1000mm; 4000.0 let a
        // gently-curved observation extrapolate into a 4.7m sweep the vehicle rode
        // for 18 seconds (measured 2026-08-05). This is the backstop on a bad fit,
        // not a description of any real marking, so it sits just clear of what a
        // real junction needs.
        this->declare_parameter<double>("turn_latch_max_radius_mm", 2000.0);
        // How much of the turn must be seen before its radius is worth
        // extrapolating. At 15.0 a 20.6-degree observation was extended to a full
        // 90 - but 20 degrees of arc cannot distinguish the start of a tight
        // junction turn from a gentle bend, and the extension inherits whichever
        // it guesses. Every turn that latched cleanly on the vehicle carried
        // 48-70 degrees, so 40.0 keeps those and rejects the ambiguous ones.
        this->declare_parameter<double>("turn_latch_min_observed_span_deg", 40.0);
        // Fraction of the turn that must be measured before the latch may be
        // released. 0.7 opened the span gate at 63 degrees of a 90-degree turn,
        // and the lane gate is satisfiable there by the road being turned into,
        // seen obliquely - so every turn was cut 27 degrees short. Measured
        // 2026-08-05 (run15): the gate read 59.5, 59.5, 61.3, 62.1 and 62.9
        // degrees at release, having consumed only 15-43% of the latched path,
        // and the vehicle then followed whatever lane it saw from 27 degrees off
        // - or, pointing into the corner with no lane in view at all, stopped.
        this->declare_parameter<double>("turn_latch_release_min_span_frac", 0.9);
        this->declare_parameter<double>("turn_latch_release_max_lane_heading_deg", 25.0);
        // Consecutive frames the turn-lane must be absent before freezing the
        // turn. See update_turn_latch for the measurements: real losses run five
        // frames or more, the spurious ones one to three, so 4 separates them
        // without spending much of the junction waiting.
        this->declare_parameter<int>("turn_latch_enter_dropout_frames", 4);
        // Cap on the measured-yaw correction applied to the replayed turn path.
        // Zero disables it and restores pure open-loop replay.
        //
        // Off by default: as built this fights min_radius_mm rather than tracking
        // error. The latched path is clamped to an 800mm radius while junctions
        // fit at 310-613mm, so a vehicle driving the real junction rotates faster
        // per millimetre than the path it is replaying says it should, and the
        // error this feeds back is dominated by that mismatch, not by drift.
        // Measured 2026-08-05 (run14) at 40 degrees: right turns read a median
        // +18.9 degrees of phantom over-rotation, were tilted back for it, and
        // came out at 61-85 degrees instead of 90. Left turns improved, but by
        // accident - their observations often exceed 90 degrees and so are never
        // extended, and the correction happened to cancel that over-curl. Making
        // this useful needs a reference that does not inherit the radius clamp.
        this->declare_parameter<double>("turn_latch_heading_correction_max_deg", 0.0);
        // Same feedback, allowed only on the run-out where the reference is the
        // target heading rather than a min_radius-clamped tangent. Sized to cover
        // the over-rotation actually seen there (+3 to +56 degrees over run21-23,
        // median 18) without letting one wild yaw reading throw the path around.
        this->declare_parameter<double>("turn_latch_runout_correction_max_deg", 25.0);
        // twist.linear.x -> mm/s. Measured on the vehicle 2026-08-05: integrating
        // |twist.linear.x| over a 48m drive matches the distance pose.position
        // travels to within 1.0001, and that position is in metres, so linear.x is
        // m/s exactly as nav_msgs/Odometry specifies. The previous 2500.0 was an
        // uncalibrated guess and ran progress_mm 2.5x fast: heading_at() then read
        // 63 degrees turned while the vehicle had really turned 25, which is also
        // where the old lane sits relative to the vehicle, so both release gates
        // opened together and every turn was abandoned a quarter of the way in.
        // Every latched turn's length is divided by this, so it stays a parameter:
        // raise it and turns end short, lower it and they run long. Calibrate
        // against turn_latch_heading_turned_deg in /avs/lane_state - it should
        // read close to turn_latch_target_heading_deg when the latch releases.
        this->declare_parameter<double>("odom_speed_scale", 1000.0);
        // How long the straight stub handed over at latch release may keep the
        // vehicle going when perception delivers no main lane at all. The stub
        // exists to cover the few frames it takes to label the new road; past
        // that it is the vehicle driving blind, so it expires into recovery.
        this->declare_parameter<int>("post_latch_stub_max_frames", 15);
        // Ego-motion compensation. Paths held across frames are expressed in the
        // vehicle frame of the frame that built them; while the vehicle yaws,
        // that frame is no longer the current one and the remembered geometry
        // drifts off the lane. See ego_motion.hpp. Applied to the published path
        // only - never to committed_state_, which the manager's thresholds are
        // measured against.
        this->declare_parameter<bool>("ego_yaw_compensation_enabled", true);
        // Latency compensation: the observation itself is one pipeline age old
        // (~150-250ms here), so even a perfectly fresh path describes where the
        // lane was, not where it is. Clamped, because extrapolating a yaw rate
        // across a long gap is a guess that grows worse with the gap - and off
        // by default for the same reason, since it moves epsilon_x_mm/theta_rad
        // by an amount no measurement on this vehicle has confirmed.
        this->declare_parameter<bool>("latency_compensation_enabled", false);
        this->declare_parameter<double>("latency_compensation_max_s", 0.4);
        // Shape of the Bezier connector into a turn lane: how far the curve's
        // belly swings toward the outside of the turn, and how long the
        // tangent handles are. Defaults come from the planner header so there
        // is one source of truth. tools/turn_bulge_sweep measures what each
        // value does to the path - do NOT raise the bulge past the fold cliff
        // it reports (0.80 at handle 1.5) or the path stops moving forward.
        this->declare_parameter<double>("turn_bezier_handle_scale_mult",
                                        TrajectoryPlanner::turn_bezier_handle_scale_mult);
        // Right turns want the UNSCALED handle (1.0), which cuts close to the
        // inside corner - the shape the 1.5 above was written to prevent, and
        // the one a near-corner right turn needs. See the header.
        this->declare_parameter<double>("turn_bezier_handle_scale_mult_right",
                                        TrajectoryPlanner::turn_bezier_handle_scale_mult_right);
        this->declare_parameter<double>("turn_lateral_bulge_mult",
                                        TrajectoryPlanner::turn_lateral_bulge_mult);
        // Right turns take the near corner from the right lane and have no room
        // to swing outside it. 0.0 = plain Bezier, negative = belly inverted so
        // the path leans into the turn immediately. See the header for the
        // measurement that motivated splitting this from the left-turn value.
        this->declare_parameter<double>("turn_lateral_bulge_mult_right",
                                        TrajectoryPlanner::turn_lateral_bulge_mult_right);
        // Fixed prev-frame weight the normalizer uses for turn trajectories
        // (uniform along the whole connector, replacing the distance ramp used
        // for follow_main/lane_change - see TrajectoryNormalizer::normalize).
        this->declare_parameter<double>("turn_blend_prev_weight",
                                        TrajectoryNormalizer::turn_blend_prev_weight);
        // Same knob for follow_main. Default 0.0 publishes the observed lane
        // waypoints directly; raise it to trade path jitter for lag.
        this->declare_parameter<double>("follow_main_blend_prev_weight",
                                        TrajectoryNormalizer::follow_main_blend_prev_weight);
        // Plan C: normalizer post-blend continuity guard thresholds.
        this->declare_parameter<double>("continuity_heading_jump_rad", 0.35);
        this->declare_parameter<double>("continuity_lateral_jump_mm", 300.0);
        // Plan C: manager composite-deviation replan/hold policy thresholds.
        this->declare_parameter<double>("replan_lateral_rms_mm", 800.0);
        this->declare_parameter<double>("hold_lateral_rms_mm", 50.0);
        this->declare_parameter<double>("min_overlap_ratio", 0.5);
        this->declare_parameter<double>("replan_min_confidence", 0.5);
        this->declare_parameter<int>("low_conf_hold_frames", 10);
        this->declare_parameter<double>("hold_min_remaining_s_mm", 500.0);
        this->declare_parameter<double>("min_path_length_mm", 200.0);
        // Plan F: solid/dashed-yellow legality gate + auto-return.
        this->declare_parameter<bool>("legality_gate_enabled", true);
        this->declare_parameter<bool>("legality_return_enabled", true);
        this->declare_parameter<bool>("legality_dashed_yellow_enabled", true);
        this->declare_parameter<double>("legality_margin_mm", 100.0);
        this->declare_parameter<int>("legality_yellow_hold_frames", 10);
        this->declare_parameter<int>("legality_return_debounce_frames", 5);
        this->declare_parameter<double>("legality_beta_deg", 20.0);

        turn_proximity_mm_ = this->get_parameter("turn_proximity_mm").as_double();
        turn_done_mm_      = this->get_parameter("turn_done_mm").as_double();
        theta_done_rad_    = this->get_parameter("theta_done_rad").as_double();
        maneuver_dropout_hold_frames_ = this->get_parameter("maneuver_dropout_hold_frames").as_int();
        intent_abort_frames_          = this->get_parameter("intent_abort_frames").as_int();
        maneuver_max_duration_s_      = this->get_parameter("maneuver_max_duration_s").as_double();
        turn_latch_target_heading_deg_    = this->get_parameter("turn_latch_target_heading_deg").as_double();
        turn_latch_runout_mm_             = this->get_parameter("turn_latch_runout_mm").as_double();
        turn_latch_skip_lead_s_           = this->get_parameter("turn_latch_skip_lead_s").as_double();
        turn_latch_min_radius_mm_         = this->get_parameter("turn_latch_min_radius_mm").as_double();
        turn_latch_max_radius_mm_         = this->get_parameter("turn_latch_max_radius_mm").as_double();
        turn_latch_min_observed_span_deg_ = this->get_parameter("turn_latch_min_observed_span_deg").as_double();
        turn_latch_release_min_span_frac_ = this->get_parameter("turn_latch_release_min_span_frac").as_double();
        turn_latch_release_max_lane_heading_deg_ = this->get_parameter("turn_latch_release_max_lane_heading_deg").as_double();
        turn_latch_enter_dropout_frames_  = this->get_parameter("turn_latch_enter_dropout_frames").as_int();
        turn_latch_heading_correction_max_deg_ = this->get_parameter("turn_latch_heading_correction_max_deg").as_double();
        turn_latch_runout_correction_max_deg_ = this->get_parameter("turn_latch_runout_correction_max_deg").as_double();
        odom_speed_scale_                 = this->get_parameter("odom_speed_scale").as_double();
        post_latch_stub_max_frames_       = this->get_parameter("post_latch_stub_max_frames").as_int();
        ego_yaw_compensation_enabled_     = this->get_parameter("ego_yaw_compensation_enabled").as_bool();
        latency_compensation_enabled_     = this->get_parameter("latency_compensation_enabled").as_bool();
        latency_compensation_max_s_       = this->get_parameter("latency_compensation_max_s").as_double();
        // These two live on the planner itself rather than in a member, so the
        // assignment here IS the plumbing - no call site passes them.
        TrajectoryPlanner::turn_bezier_handle_scale_mult = this->get_parameter("turn_bezier_handle_scale_mult").as_double();
        TrajectoryPlanner::turn_bezier_handle_scale_mult_right = this->get_parameter("turn_bezier_handle_scale_mult_right").as_double();
        TrajectoryPlanner::turn_lateral_bulge_mult       = this->get_parameter("turn_lateral_bulge_mult").as_double();
        TrajectoryPlanner::turn_lateral_bulge_mult_right = this->get_parameter("turn_lateral_bulge_mult_right").as_double();
        TrajectoryNormalizer::turn_blend_prev_weight     = this->get_parameter("turn_blend_prev_weight").as_double();
        TrajectoryNormalizer::follow_main_blend_prev_weight = this->get_parameter("follow_main_blend_prev_weight").as_double();
        continuity_heading_jump_rad_ = this->get_parameter("continuity_heading_jump_rad").as_double();
        continuity_lateral_jump_mm_  = this->get_parameter("continuity_lateral_jump_mm").as_double();
        replan_lateral_rms_mm_   = this->get_parameter("replan_lateral_rms_mm").as_double();
        hold_lateral_rms_mm_     = this->get_parameter("hold_lateral_rms_mm").as_double();
        min_overlap_ratio_       = this->get_parameter("min_overlap_ratio").as_double();
        replan_min_confidence_   = this->get_parameter("replan_min_confidence").as_double();
        low_conf_hold_frames_    = this->get_parameter("low_conf_hold_frames").as_int();
        hold_min_remaining_s_mm_ = this->get_parameter("hold_min_remaining_s_mm").as_double();
        min_path_length_mm_      = this->get_parameter("min_path_length_mm").as_double();

        // ── Publishers ───────────────────────────────────────────────────────
        control_error_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/avs/control_error", 10);
        lane_state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/avs/lane_state", 10);
        route_intent_ack_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/avs/route_intent_ack", 10);

        // ── Subscribers ──────────────────────────────────────────────────────
        telemetry_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/telemetry_realworld", 10,
            std::bind(&LaneErrorNode::telemetry_callback, this, std::placeholders::_1)
        );
        route_intent_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/route_intent", 10,
            std::bind(&LaneErrorNode::route_intent_callback, this, std::placeholders::_1)
        );
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/cmd", 10,
            std::bind(&LaneErrorNode::cmd_callback, this, std::placeholders::_1)
        );
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_raw", 10,
            std::bind(&LaneErrorNode::odom_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "LaneErrorNode started. Initial state: FOLLOW_MAIN");
        RCLCPP_INFO(this->get_logger(), "Subscribing: /avs/telemetry_realworld, /avs/route_intent, /avs/cmd, /odom_raw");
        RCLCPP_INFO(this->get_logger(), "Publishing:  /avs/control_error, /avs/lane_state, /avs/route_intent_ack");
    }

private:
    // ── Route intent callback ────────────────────────────────────────────────
    void publish_route_intent_ack(const json& intent_json, const std::string& requested_intent,
                                  bool accepted, const std::string& reason = "") {
        json ack;
        ack["intent"] = route_intent_name(current_intent_);
        ack["pending_intent"] = route_intent_name(current_intent_);
        ack["requested_intent"] = requested_intent;
        ack["accepted"] = accepted;
        if (!reason.empty()) {
            ack["reason"] = reason;
        }
        ack["seq"] = current_intent_seq_;
        if (intent_json.contains("source")) {
            ack["source"] = intent_json["source"];
        }

        std_msgs::msg::String ack_msg;
        ack_msg.data = ack.dump();
        route_intent_ack_pub_->publish(ack_msg);
    }

    void route_intent_callback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            json intent_json = json::parse(msg->data);
            if (intent_json.contains("intent")) {
                std::string intent_str = intent_json["intent"].get<std::string>();
                bool accepted = true;
                std::string reason;
                RouteIntent next_intent = current_intent_;
                if (intent_str == "follow_main") {
                    next_intent = RouteIntent::FOLLOW_MAIN;
                } else if (intent_str == "turn_right") {
                    next_intent = RouteIntent::TURN_RIGHT;
                } else if (intent_str == "turn_left") {
                    next_intent = RouteIntent::TURN_LEFT;
                } else if (intent_str == "lane_change_left") {
                    next_intent = RouteIntent::LANE_CHANGE_LEFT;
                } else if (intent_str == "lane_change_right") {
                    next_intent = RouteIntent::LANE_CHANGE_RIGHT;
                } else if (intent_str == "straight") {
                    next_intent = RouteIntent::FOLLOW_MAIN;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Received legacy intent 'straight', mapping to FOLLOW_MAIN");
                } else {
                    accepted = false;
                    reason = "unrecognized_intent";
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Received unrecognized intent '%s', ignoring", intent_str.c_str());
                }

                if (accepted) {
                    current_intent_ = next_intent;
                    current_intent_seq_ = intent_json.contains("seq")
                        ? intent_json["seq"].get<uint64_t>()
                        : next_route_intent_seq_++;
                    current_intent_age_frames_ = 0;
                    blocked_intent_counter_ = 0;
                    maneuver_dropout_counter_ = 0;
                    maneuver_target_seen_since_intent_ = false;
                    target_seen_streak_ = 0;
                    hold_reason_.clear();
                    // Plan F: a real route intent always beats the internal
                    // legality-return override.
                    legality_return_active_ = false;
                    legality_auto_return_.reset();
                }
                publish_route_intent_ack(intent_json, intent_str, accepted, reason);
            } else {
                publish_route_intent_ack(intent_json, "", false, "missing_intent");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "route_intent_callback parse error: %s. Ignoring message", e.what());
        }
    }

    // ── External command callback ────────────────────────────────────────────
    void cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            json cmd_json = json::parse(msg->data);
            std::string cmd = cmd_json.value("cmd", "");

            if (cmd == "arm" || cmd == "disarm" || cmd == "resume") {
                RCLCPP_INFO(this->get_logger(), "CMD: System command received: %s", cmd.c_str());
                if (cmd == "resume") {
                    current_intent_ = RouteIntent::FOLLOW_MAIN;
                    current_intent_seq_ = next_route_intent_seq_++;
                    current_intent_age_frames_ = 0;
                }
            } else if (cmd == "reset") {
                RCLCPP_INFO(this->get_logger(), "CMD: Reset command received. Resetting hysteresis and memory.");
                committed_state_ = CommittedTrajectoryState{};
                consecutive_invalid_frames_ = 0;
                current_intent_ = RouteIntent::FOLLOW_MAIN;
                current_intent_seq_ = 0;
                current_intent_age_frames_ = 0;
                last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
                last_main_track_id_ = "";
                state_ = DecisionState::FOLLOW_MAIN;
                legacy_model_.t_junction_counter_ = 0;
                frame_count_ = 0;
                blocked_intent_counter_ = 0;
                maneuver_dropout_counter_ = 0;
                maneuver_target_seen_since_intent_ = false;
                target_seen_streak_ = 0;
                hold_reason_.clear();
                legality_gate_ = LaneLegalityGate{};
                last_legality_report_ = LaneLegalityReport{};
                legality_auto_return_.reset();
                illegal_current_streak_ = 0;
                legality_return_active_ = false;
            } else if (cmd == "tur" "n") {
                current_intent_ = RouteIntent::LEGACY_TURN;
                current_intent_seq_ = next_route_intent_seq_++;
                current_intent_age_frames_ = 0;
                maneuver_target_seen_since_intent_ = false;
                target_seen_streak_ = 0;
                RCLCPP_INFO(this->get_logger(), "CMD: Legacy turn command received. Arming legacy turn intent.");
            } else if (cmd == "lane_chang" "e") {
                current_intent_ = RouteIntent::LEGACY_LANE_CHANGE;
                current_intent_seq_ = next_route_intent_seq_++;
                current_intent_age_frames_ = 0;
                maneuver_target_seen_since_intent_ = false;
                target_seen_streak_ = 0;
                RCLCPP_INFO(this->get_logger(), "CMD: Legacy lane_change command received. Arming legacy lane change intent.");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "cmd_callback parse error: %s", e.what());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_speed_mms_ = std::abs(msg->twist.twist.linear.x) * odom_speed_scale_;

        // Yaw from the orientation quaternion rather than twist.angular.z - see
        // EgoMotion::yaw_from_quaternion for why the dimensionless source is the
        // safe one on this vehicle.
        double yaw = EgoMotion::yaw_from_quaternion(msg->pose.pose.orientation.z,
                                                    msg->pose.pose.orientation.w);
        auto now = this->get_clock()->now();
        if (has_yaw_ && last_odom_time_.nanoseconds() > 0) {
            double odom_dt = (now - last_odom_time_).seconds();
            // Ignore duplicated stamps and long gaps: a rate differenced across
            // either is noise, not motion.
            if (odom_dt > 1e-3 && odom_dt < 1.0) {
                double rate = EgoMotion::wrap_pi(yaw - current_yaw_rad_) / odom_dt;
                // /odom_raw runs at ~11Hz, so a raw difference is coarse; damp
                // it. Only latency compensation reads this rate - the
                // frame-to-frame memory rotation uses absolute yaw and needs no
                // filtering at all.
                yaw_rate_rps_ = 0.5 * yaw_rate_rps_ + 0.5 * rate;
            }
        }
        if (!has_yaw_) {
            // First reading: there is no stale geometry to correct against it.
            committed_yaw_rad_ = yaw;
            has_yaw_ = true;
        }
        current_yaw_rad_ = yaw;
        last_odom_time_ = now;
    }

    // ── Frozen turn execution lifecycle ──────────────────────────────────────
    // Latches the committed turn path when perception loses the turn-lane at the
    // junction, then advances along it on odometry until it is consumed. The
    // geometry itself lives in trajectory_latch.hpp; this only owns the state
    // machine. Freeze is deliberately hard: a turn-lane that flickers back mid-
    // junction is ignored, because re-latching onto a partial re-detection is
    // exactly the jitter this mechanism exists to avoid.
    // Direction of a lane relative to the vehicle's own heading, from the chord
    // across its waypoints. Chord rather than first segment: the near end of a
    // freshly acquired lane is the noisiest part of the projection, and what
    // matters here is only whether the road as a whole runs with the vehicle or
    // across it.
    static double lane_heading_rad(const LaneCandidate& l) {
        if (l.raw_obj.contains("waypoints") && l.raw_obj["waypoints"].is_array() &&
            l.raw_obj["waypoints"].size() >= 2) {
            const auto& wps = l.raw_obj["waypoints"];
            double dx = wps.back()[0].get<double>() - wps.front()[0].get<double>();
            double dy = wps.back()[1].get<double>() - wps.front()[1].get<double>();
            if (std::hypot(dx, dy) > 1e-6) return std::atan2(dx, dy);
        }
        if (l.raw_obj.contains("lookahead_theta_rad")) {
            return l.raw_obj["lookahead_theta_rad"].get<double>();
        }
        return 0.0;
    }

    void update_turn_latch(const LaneCandidate* turn_lane_cand, const LaneCandidate* main_current,
                           double dt, const rclcpp::Time& now) {
        if (turn_latch_active_) {
            turn_latch_progress_mm_ += current_speed_mms_ * dt;
            // How far the vehicle has actually rotated since the latch closed,
            // measured, not inferred.
            //
            // This used to read heading_at(path, progress_mm) - the tangent of
            // the frozen path at the point the vehicle is assumed to have
            // reached. Two things are wrong with that, both measured on the
            // vehicle 2026-08-05. It inherits every error in progress_mm, so the
            // 2.5x odom_speed_scale bug made it report 63 degrees turned at 25
            // degrees of real rotation. And heading_at returns the chord angle of
            // one 100mm segment of an observed lane centreline, which on real
            // paths swings +-25 degrees frame to frame (logged: 12.5, 4.7, 1.6,
            // 26.3, 25.8, 19.4, 12.5, 4.3, 34.4, 61.6 degrees over consecutive
            // frames of one turn), so the span gate opened at essentially random
            // moments.
            //
            // Odom yaw has neither problem: it is a direct measurement, it is
            // monotone through a turn, and being an angle it carries no distance
            // scale to get wrong. Sign per EgoMotion::frame_delta_from_ros_yaw -
            // ROS yaw is CCW-positive, this frame's path angles are right-turn
            // positive, hence the flip that helper applies.
            //
            // If /odom_raw stops arriving this holds at whatever it last read and
            // the span gate simply stops opening, leaving the path-consumed and
            // deadline releases to end the turn. That is the safe direction: this
            // gate may only ever release earlier than those, never later.
            turn_latch_heading_turned_rad_ =
                EgoMotion::frame_delta_from_ros_yaw(current_yaw_rad_, turn_latch_start_yaw_rad_);

            // How far the replay's own assumption has drifted from the vehicle.
            // re_express emits the remaining path into the frame of a vehicle
            // sitting exactly on the path at progress_mm, heading along its
            // tangent there; the difference between that tangent and the yaw
            // actually measured is the error the open-loop replay is blind to.
            // Feeding it back tilts the emitted path the other way, so a vehicle
            // that has over-rotated is handed a path that steers it less, not one
            // drawn as though the over-rotation never happened.
            //
            // Clamped because this is the only place odometry can steer directly:
            // a wild yaw reading should degrade the turn, not invert it. Setting
            // the limit to zero restores pure open-loop replay.
            double target = (turn_latch_kind_ == TrajectoryKind::TURN_RIGHT ? 1.0 : -1.0) *
                            turn_latch_target_heading_deg_ * M_PI / 180.0;

            // Stop turning once the vehicle has measurably made the turn, even
            // though the frozen path still has arc left in it. The path is built
            // at min_radius_mm - 800mm, forced up from the 310-613mm junctions
            // really fit at, because a radius under the 600mm pure-pursuit
            // lookahead saturates steering. The vehicle therefore rotates faster
            // per millimetre than the arc it is replaying says it should, and
            // driving that arc to its end overshoots.
            //
            // It only overshoots when the release gate has not fired, which needs
            // both enough rotation and a lane ahead to hand over to - so the case
            // this catches is precisely the one that hurts: rotation complete, no
            // road acquired yet, and the arc still turning the vehicle away from
            // where the road is. Measured 2026-08-05 (run17): a left turn drove
            // 2175 of 2191mm and came out at 118 degrees, pointing 28 degrees off
            // the new road with nothing in view, whereupon the post-latch stub
            // drove it straight for another second.
            //
            // Skipping to the run-out rather than releasing keeps the two-gate
            // release intact - the latch still will not hand over to an empty
            // frame - while making what is left of it straight.
            // Read one frame ahead before deciding. This test can only run when a
            // telemetry frame arrives, and frames are far apart next to how fast
            // the vehicle turns: measured over run19-21, the gap between frames
            // during a latch runs 0.075s at the median and 0.198s at the 99th
            // percentile, while yaw rate reaches 110 deg/s. So the angle can
            // cross the target and be well past it by the time anything looks.
            // Measured on the one turn of run21 that went wrong: consecutive
            // frames read 89.4 then 106.6 degrees across a 0.27s gap, and the
            // skip - correct on its own terms, firing at the first frame past
            // target - fired 17 degrees late and the turn ended at 119.8.
            //
            // Extrapolating over one median frame keeps the decision on the
            // frame where the crossing actually happens. It cannot fire the skip
            // early on a turn that was never going to reach the target: at the
            // rates those turns run, the lead is worth two or three degrees, and
            // the three good turns of run21 released at 81-84 degrees having
            // never come near this test at all.
            // Through the helper, not by multiplying: yaw_rate_rps_ is ROS yaw
            // (CCW positive, so left turns raise it) while every angle in this
            // function is vehicle-frame (right positive). Adding the raw rate
            // would subtract the lead and fire the skip later than it does now.
            double lead_rad = EgoMotion::frame_delta_from_ros_yaw(
                yaw_rate_rps_ * turn_latch_skip_lead_s_, 0.0);
            double turned_soon = std::abs(turn_latch_heading_turned_rad_ + lead_rad);
            if (turned_soon >= std::abs(target)) {
                double runout_start = std::max(0.0, turn_latch_length_mm_ - turn_latch_runout_mm_);
                if (turn_latch_progress_mm_ < runout_start) {
                    RCLCPP_INFO(this->get_logger(),
                        "Turn measured complete at %.0f deg (%.0f projected) with %.0fmm of "
                        "arc unspent - skipping to run-out.",
                        turn_latch_heading_turned_rad_ * 180.0 / M_PI,
                        turned_soon * 180.0 / M_PI,
                        runout_start - turn_latch_progress_mm_);
                    turn_latch_progress_mm_ = runout_start;
                }
            }

            // How far the replay's own assumption has drifted from the vehicle.
            // re_express emits the remaining path into the frame of a vehicle
            // sitting exactly on the path at progress_mm, heading along its
            // tangent there; the difference between that tangent and the yaw
            // actually measured is the error the open-loop replay is blind to.
            // Feeding it back tilts the emitted path the other way, so a vehicle
            // that has over-rotated is handed a path that steers it less, not one
            // drawn as though the over-rotation never happened.
            //
            // Allowed on the run-out and nowhere else. On the arc the tangent
            // inherits the min_radius_mm clamp - the frozen arc is drawn at
            // 609-1078mm (measured run21-23) where real junctions fit 310-613mm,
            // so the vehicle rotates more per millimetre than the arc says and
            // "turned - assumed" is positive whatever the vehicle does. Closing
            // the loop there reads that as over-rotation and fights the clamp
            // rather than the drift; run14 measured it steering right turns out
            // at 61-85 degrees instead of 90. The run-out has no such problem:
            // it is straight at the target heading, so the tangent IS the target
            // and the difference is the over-rotation itself, with no radius in
            // it to be wrong.
            //
            // This is where the error lives now. All seven turns that reached the
            // run-out in run21-23 overshot, +3 to +56 degrees, while the eleven
            // that released before it landed at a median of 6 - because the
            // run-out publishes zero error by construction and the vehicle coasts
            // on whatever wheel differential it was left with, unobserved.
            bool on_runout =
                turn_latch_progress_mm_ >= std::max(0.0, turn_latch_length_mm_ - turn_latch_runout_mm_);
            double limit = (on_runout ? turn_latch_runout_correction_max_deg_
                                      : turn_latch_heading_correction_max_deg_) * M_PI / 180.0;
            double assumed = TrajectoryLatch::heading_at(turn_latch_path_, turn_latch_progress_mm_);
            turn_latch_heading_error_rad_ = std::max(
                -limit, std::min(limit,
                                 TrajectoryLatch::wrap_pi(turn_latch_heading_turned_rad_ - assumed)));

            bool lane_present = (main_current != nullptr);
            double lane_heading = lane_present ? lane_heading_rad(*main_current) : 0.0;

            if (turn_latch_progress_mm_ >= turn_latch_length_mm_) {
                release_turn_latch("latch_path_consumed");
            } else if (TrajectoryLatch::turn_complete(
                           turn_latch_heading_turned_rad_, target,
                           turn_latch_release_min_span_frac_, lane_present, lane_heading,
                           turn_latch_release_max_lane_heading_deg_ * M_PI / 180.0)) {
                // Turned far enough and a road is lined up ahead: hand the
                // vehicle back to perception now rather than running the
                // remaining open-loop metres blind past it.
                RCLCPP_INFO(this->get_logger(),
                    "Frozen turn handing over: %.0f deg turned, new lane %.0f deg off, "
                    "%.0f/%.0fmm consumed.",
                    turn_latch_heading_turned_rad_ * 180.0 / M_PI,
                    lane_heading * 180.0 / M_PI,
                    turn_latch_progress_mm_, turn_latch_length_mm_);
                release_turn_latch("latch_new_lane_acquired");
            } else if ((now - turn_latch_start_time_).seconds() > turn_latch_deadline_s_) {
                RCLCPP_WARN(this->get_logger(),
                    "Frozen turn exceeded %.1fs at %.0f/%.0fmm - releasing. Check /odom_raw is publishing.",
                    turn_latch_deadline_s_, turn_latch_progress_mm_, turn_latch_length_mm_);
                release_turn_latch("latch_timeout");
            }
            return;
        }

        bool turn_intent = (current_intent_ == RouteIntent::TURN_LEFT ||
                            current_intent_ == RouteIntent::TURN_RIGHT);
        if (!turn_intent) {
            turn_latch_enter_dropout_ = 0;
            turn_latch_fresh_obs_.clear();
            return;
        }
        if (turn_lane_cand != nullptr) {
            turn_latch_enter_dropout_ = 0;
            // Keep the newest committed path that was backed by a visible turn
            // marking. The dropout window below deliberately spends four frames
            // before freezing, and over those frames the committed path keeps
            // being soft-updated against whatever lane fragments remain once the
            // marking is gone - which is noise, by definition. Latching the path
            // as it stands at the end of the wait measurably wrecks it: latching
            // immediately (run10) produced observed spans of 70-86 degrees, never
            // above 90; latching four frames later (run11) produced 38-146
            // degrees with 4 of 11 past 90, and a path already curved 146 degrees
            // is not extended at all, so the vehicle simply drove that curve and
            // over-rotated. Freezing this snapshot instead keeps the wait without
            // paying for it in geometry.
            if (committed_state_.committed_intent == current_intent_ &&
                committed_state_.trajectory.valid) {
                turn_latch_fresh_obs_ = committed_state_.trajectory.points;
            }
            return;
        }

        // Require the turn-lane to stay gone, not merely blink. Latching is
        // irreversible for the rest of the turn, so paying for it with one
        // dropped frame trades a whole junction of live tracking for a frozen
        // path - and left turns drop frames constantly. Measured over one full
        // map (2026-08-05, run10): with a left-turn intent the turn-lane was
        // visible 73% of frames, and of the gaps, 29 lasted a single frame and 8
        // lasted two or three, against 14 real losses of five frames or more.
        // Right turns saw one single-frame gap all run - which is the whole
        // reason they already worked, not anything about their geometry. So the
        // left turns were latching after 20-312mm and replaying 61-184 degrees
        // open-loop while the marking they had given up on was still in view for
        // 47-91% of the frozen frames.
        //
        // A budget already existed for exactly this - maneuver_dropout_hold_frames,
        // 10 - and the latch was pre-empting it at frame one. This counter is
        // separate rather than reusing that one because maneuver_dropout_counter_
        // is maintained in update_lane_state, which runs after this function, and
        // is reset from several other places; sharing it would make latch entry
        // depend on call order.
        if (++turn_latch_enter_dropout_ <= turn_latch_enter_dropout_frames_) return;

        // Only latch a turn the manager has already committed. That commit is
        // gated on turn_proximity_mm, so a turn-lane lost while the junction is
        // still far away - a transient occlusion, not an arrival - keeps
        // following main instead of steering into a turn that isn't there yet.
        if (committed_state_.committed_intent != current_intent_) return;
        if (!committed_state_.trajectory.valid) return;

        // The snapshot taken while the marking was still visible, not the path as
        // it stands now - see the comment where it is captured. Falling back to
        // the live path keeps the old behaviour for the one case the snapshot
        // cannot cover: an intent whose very first frame already has no marking.
        const std::vector<Point2D>& observed =
            turn_latch_fresh_obs_.empty() ? committed_state_.trajectory.points
                                          : turn_latch_fresh_obs_;
        double observed_len = TrajectoryLatch::path_length(observed);
        if (observed_len <= 0.0) return;

        // The camera only ever delivers the near end of the turn marking, so the
        // committed path typically carries 40-60 of the junction's 90 degrees.
        // Replaying it verbatim leaves the vehicle half-turned, where the new road
        // is still too oblique to be labelled main-lane and perception never
        // recovers. Continue the marking's own arc to the full turn instead.
        double target = (current_intent_ == RouteIntent::TURN_RIGHT ? 1.0 : -1.0) *
                        turn_latch_target_heading_deg_ * M_PI / 180.0;
        // Drop the flared tip here rather than only inside extend_to_turn_angle,
        // so the span and length reported on /avs/lane_state describe the same
        // path the extension actually worked from. Reporting the raw terminal
        // heading made the logs unreadable: a turn whose usable span was 50-odd
        // degrees showed observed_span=9.2, which reads like a path that should
        // have been rejected by the 40-degree floor and was not. The call inside
        // extend_to_turn_angle stays - it is the one that matters for
        // correctness, and trimming an already-trimmed path is a no-op.
        // A copy, not a write-back: committed_state_ is the datum every replan and
        // hold threshold is measured against, and reshaping it here would move
        // those thresholds' input without moving the thresholds.
        std::vector<Point2D> usable = TrajectoryLatch::trim_flared_tip(observed, target);
        observed_len = TrajectoryLatch::path_length(usable);
        if (observed_len <= 0.0) return;

        turn_latch_observed_span_rad_ = TrajectoryLatch::terminal_heading_rad(usable);
        std::vector<Point2D> path = TrajectoryLatch::extend_to_turn_angle(
            usable, target, turn_latch_runout_mm_,
            turn_latch_min_radius_mm_, turn_latch_max_radius_mm_,
            turn_latch_min_observed_span_deg_ * M_PI / 180.0);

        double len = TrajectoryLatch::path_length(path);

        turn_latch_active_ = true;
        turn_latch_path_ = std::move(path);
        turn_latch_length_mm_ = len;
        turn_latch_extension_mm_ = len - observed_len;
        turn_latch_extended_span_rad_ = TrajectoryLatch::terminal_heading_rad(turn_latch_path_);
        turn_latch_progress_mm_ = 0.0;
        turn_latch_heading_turned_rad_ = 0.0;
        // Datum for the measured span gate above. Whatever /odom_raw reads now is
        // zero rotation by definition, so a stale or offset yaw cannot bias it.
        //
        // This was briefly taken at commit time instead, to fold in the approach
        // swing that happens before the marking leaves view. Measured on the
        // vehicle across run19 and run20 (9 latched turns), that made the gate
        // read an error of -66 to +9 degrees against yaw integrated
        // independently from /odom_raw - a 76 degree spread, random in sign on
        // both turn directions, because how much of the approach the snapshot
        // caught depended on wherever the manager happened to commit. The turns
        // came out anywhere from 70 to 154 degrees of real rotation with only
        // one of nine inside 90 +- 10. The commit instant is not a repeatable
        // reference; the latch instant is.
        turn_latch_start_yaw_rad_ = current_yaw_rad_;
        turn_latch_kind_ = committed_state_.trajectory.trajectory_kind;
        turn_latch_confidence_ = committed_state_.trajectory.confidence;
        turn_latch_start_time_ = now;
        turn_latch_deadline_s_ = latch_deadline_s(len);
        RCLCPP_INFO(this->get_logger(),
            "Turn-lane lost with %s committed - latching %.0fmm (%.0f observed + %.0f extension), "
            "%.0f -> %.0f deg, deadline %.1fs.",
            trajectory_kind_name(turn_latch_kind_), len, observed_len, turn_latch_extension_mm_,
            turn_latch_observed_span_rad_ * 180.0 / M_PI,
            turn_latch_extended_span_rad_ * 180.0 / M_PI, turn_latch_deadline_s_);
    }

    // The latch is released by distance travelled; the clock is only a backstop
    // for odometry going silent mid-turn. A fixed budget cannot serve both a
    // 1.7m and a 4.2m path, so scale it by how long this path should take at the
    // speed we entered the turn at, doubled for slowdowns, and floor the speed so
    // a stationary start cannot produce an unbounded deadline.
    double latch_deadline_s(double length_mm) const {
        double speed = std::max(std::abs(current_speed_mms_), kLatchDeadlineMinSpeedMms);
        double budget = 2.0 * length_mm / speed;
        return std::max(maneuver_max_duration_s_, std::min(budget, kLatchDeadlineMaxS));
    }

    void release_turn_latch(const char* reason) {
        RCLCPP_INFO(this->get_logger(), "Frozen turn released (%s) - intent -> FOLLOW_MAIN", reason);
        turn_latch_active_ = false;
        turn_latch_path_.clear();
        turn_latch_length_mm_ = 0.0;
        turn_latch_progress_mm_ = 0.0;
        turn_latch_kind_ = TrajectoryKind::UNKNOWN;
        turn_latch_confidence_ = 0.0;
        current_intent_ = RouteIntent::FOLLOW_MAIN;
        current_intent_seq_ = 0;
        current_intent_age_frames_ = 0;
        last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
        maneuver_dropout_counter_ = 0;
        blocked_intent_counter_ = 0;
        turn_latch_enter_dropout_ = 0;
        turn_latch_fresh_obs_.clear();
        turn_latch_heading_error_rad_ = 0.0;
        maneuver_target_seen_since_intent_ = false;
        target_seen_streak_ = 0;
        // Never release into a void. By the time the latch ends, whatever is in
        // committed_state_ is the last re-expressed tail of the latched path -
        // often a point or two, sometimes nothing - and plan_follow_main's
        // dropout branch needs >= 2 points to bridge from. Without them the
        // trajectory comes back invalid, the manager enters recovery and the
        // vehicle has no path at all for as long as perception takes to pick up
        // the new road. The latch ends aligned with that road (its run-out is
        // straight by construction), so continuing straight ahead is the honest
        // continuation of the geometry just being followed, not a guess. A real
        // lane observed this frame replaces it before it is ever used.
        if (committed_state_.trajectory.points.size() < 2) {
            std::vector<Point2D> stub;
            for (double y = 100.0; y <= kPostLatchStubLengthMm + 1e-9; y += 100.0) {
                stub.push_back({0.0, y});
            }
            committed_state_.trajectory.points = std::move(stub);
            committed_state_.trajectory.valid = true;
            committed_state_.trajectory.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
            committed_state_.trajectory.blocked_by_marking = false;
            committed_state_.trajectory.has_precomputed_control = false;
            committed_state_.trajectory.from_direct_observation = false;
            // Low enough that any real observation outranks it in the manager's
            // replan arithmetic - this is a bridge, not a path worth defending.
            committed_state_.trajectory.confidence = kPostLatchStubConfidence;
            committed_state_.trajectory.normalization_mode = "post_latch_stub";
            committed_state_.trajectory.target_lane_id = "";
            committed_state_.progress_s_mm = 0.0;
            committed_state_.remaining_s_mm = kPostLatchStubLengthMm;
            committed_state_.committed_intent = RouteIntent::FOLLOW_MAIN;
            committed_state_.committed_intent_seq = 0;
            committed_state_.replan_reason = "post_latch_stub";
            // Start the clock. Without it the stub is immortal: plan_follow_main's
            // dropout branch re-anchors the previous path whenever no lane is
            // observed and returns it *valid*, so a straight stub regenerates
            // itself as a straight path every frame and the manager never sees a
            // dropout to recover from. The vehicle then drives straight out of
            // the road with a path on screen the whole way.
            post_latch_stub_frames_ = 0;
        }

        // Not hold_reason_: update_turn_latch runs before telemetry_callback
        // clears that for the frame, so a reason stored there would be wiped
        // before anyone saw it - and exempting it from the clear would make it
        // stick forever instead. This has its own lifetime.
        turn_latch_release_reason_ = reason;
    }

    // ── Telemetry callback: build trajectory and publish errors ─────────────
    void telemetry_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Track telemetry timing for debug/runtime bookkeeping.
        double dt = 0.033;
        auto now = this->get_clock()->now();
        if (last_telemetry_time_.nanoseconds() > 0) {
            dt = (now - last_telemetry_time_).seconds();
        }
        last_telemetry_time_ = now;

        // Reload thresholds in case they were updated at runtime
        turn_proximity_mm_ = this->get_parameter("turn_proximity_mm").as_double();
        turn_done_mm_      = this->get_parameter("turn_done_mm").as_double();
        theta_done_rad_    = this->get_parameter("theta_done_rad").as_double();
        maneuver_dropout_hold_frames_ = this->get_parameter("maneuver_dropout_hold_frames").as_int();
        intent_abort_frames_          = this->get_parameter("intent_abort_frames").as_int();
        maneuver_max_duration_s_      = this->get_parameter("maneuver_max_duration_s").as_double();
        turn_latch_target_heading_deg_    = this->get_parameter("turn_latch_target_heading_deg").as_double();
        turn_latch_runout_mm_             = this->get_parameter("turn_latch_runout_mm").as_double();
        turn_latch_skip_lead_s_           = this->get_parameter("turn_latch_skip_lead_s").as_double();
        turn_latch_min_radius_mm_         = this->get_parameter("turn_latch_min_radius_mm").as_double();
        turn_latch_max_radius_mm_         = this->get_parameter("turn_latch_max_radius_mm").as_double();
        turn_latch_min_observed_span_deg_ = this->get_parameter("turn_latch_min_observed_span_deg").as_double();
        turn_latch_release_min_span_frac_ = this->get_parameter("turn_latch_release_min_span_frac").as_double();
        turn_latch_release_max_lane_heading_deg_ = this->get_parameter("turn_latch_release_max_lane_heading_deg").as_double();
        turn_latch_enter_dropout_frames_  = this->get_parameter("turn_latch_enter_dropout_frames").as_int();
        turn_latch_heading_correction_max_deg_ = this->get_parameter("turn_latch_heading_correction_max_deg").as_double();
        turn_latch_runout_correction_max_deg_ = this->get_parameter("turn_latch_runout_correction_max_deg").as_double();
        odom_speed_scale_                 = this->get_parameter("odom_speed_scale").as_double();
        post_latch_stub_max_frames_       = this->get_parameter("post_latch_stub_max_frames").as_int();
        ego_yaw_compensation_enabled_     = this->get_parameter("ego_yaw_compensation_enabled").as_bool();
        latency_compensation_enabled_     = this->get_parameter("latency_compensation_enabled").as_bool();
        latency_compensation_max_s_       = this->get_parameter("latency_compensation_max_s").as_double();
        // These two live on the planner itself rather than in a member, so the
        // assignment here IS the plumbing - no call site passes them.
        TrajectoryPlanner::turn_bezier_handle_scale_mult = this->get_parameter("turn_bezier_handle_scale_mult").as_double();
        TrajectoryPlanner::turn_bezier_handle_scale_mult_right = this->get_parameter("turn_bezier_handle_scale_mult_right").as_double();
        TrajectoryPlanner::turn_lateral_bulge_mult       = this->get_parameter("turn_lateral_bulge_mult").as_double();
        TrajectoryPlanner::turn_lateral_bulge_mult_right = this->get_parameter("turn_lateral_bulge_mult_right").as_double();
        TrajectoryNormalizer::turn_blend_prev_weight     = this->get_parameter("turn_blend_prev_weight").as_double();
        TrajectoryNormalizer::follow_main_blend_prev_weight = this->get_parameter("follow_main_blend_prev_weight").as_double();
        continuity_heading_jump_rad_ = this->get_parameter("continuity_heading_jump_rad").as_double();
        continuity_lateral_jump_mm_  = this->get_parameter("continuity_lateral_jump_mm").as_double();
        replan_lateral_rms_mm_   = this->get_parameter("replan_lateral_rms_mm").as_double();
        hold_lateral_rms_mm_     = this->get_parameter("hold_lateral_rms_mm").as_double();
        min_overlap_ratio_       = this->get_parameter("min_overlap_ratio").as_double();
        replan_min_confidence_   = this->get_parameter("replan_min_confidence").as_double();
        low_conf_hold_frames_    = this->get_parameter("low_conf_hold_frames").as_int();
        hold_min_remaining_s_mm_ = this->get_parameter("hold_min_remaining_s_mm").as_double();
        min_path_length_mm_      = this->get_parameter("min_path_length_mm").as_double();
        legality_gate_enabled_          = this->get_parameter("legality_gate_enabled").as_bool();
        legality_return_enabled_        = this->get_parameter("legality_return_enabled").as_bool();
        legality_dashed_yellow_enabled_ = this->get_parameter("legality_dashed_yellow_enabled").as_bool();
        legality_margin_mm_             = this->get_parameter("legality_margin_mm").as_double();
        legality_yellow_hold_frames_    = this->get_parameter("legality_yellow_hold_frames").as_int();
        legality_return_debounce_frames_ = this->get_parameter("legality_return_debounce_frames").as_int();
        legality_beta_deg_              = this->get_parameter("legality_beta_deg").as_double();

        try {
            json telemetry = json::parse(msg->data);
            latest_telemetry_timestamp_ms_ = telemetry.value("timestamp_ms", static_cast<uint64_t>(0));
            // Age of the camera frame this observation came from, stamped by
            // ncnn_inference_node. Missing -> 0 -> latency compensation is a
            // no-op, which is the right fallback.
            latest_output_age_ms_ = telemetry.value("output_age_ms", 0.0);
            std::vector<LaneCandidate> lanes = LegacyLaneModel::extract_lane_candidates(telemetry);
            std::vector<MarkingCandidate> markings = LegacyLaneModel::extract_marking_candidates(telemetry);

            // ── Plan F: yellow-marking legality gate ────────────────────────
            // One filtered world-view per frame: evaluate + filter BEFORE any
            // lane consumer (legacy intent resolution, split_main_lanes,
            // select_turn_lane, update_lane_state, planner, direct-IPM
            // fallback) so an ILLEGAL lane simply does not exist downstream.
            // The lane currently followed (last_main_track_id_) is exempt -
            // never drop the active path; it goes through auto-return instead.
            PathObservationFrame obs_frame = PathObservationBuilder::build(telemetry);
            {
                LaneLegalityGate::Params gp = legality_gate_.params();
                gp.enabled = legality_gate_enabled_;
                gp.margin_mm = legality_margin_mm_;
                gp.yellow_hold_frames = legality_yellow_hold_frames_;
                gp.beta_deg = legality_beta_deg_;
                gp.dashed_yellow_enabled = legality_dashed_yellow_enabled_;
                legality_gate_.set_params(gp);
            }
            last_legality_report_ = legality_gate_.evaluate(obs_frame);
            // Dashed-yellow (soft) ILLEGAL lanes stay visible while a
            // lane-change intent or committed lane-change maneuver is active:
            // deliberately crossing a dashed yellow is allowed. LEGACY_LANE_CHANGE
            // counts too - it resolves to LANE_CHANGE_* from these very lanes.
            legality_allow_soft_ =
                current_intent_ == RouteIntent::LANE_CHANGE_LEFT ||
                current_intent_ == RouteIntent::LANE_CHANGE_RIGHT ||
                current_intent_ == RouteIntent::LEGACY_LANE_CHANGE ||
                committed_state_.committed_intent == RouteIntent::LANE_CHANGE_LEFT ||
                committed_state_.committed_intent == RouteIntent::LANE_CHANGE_RIGHT;
            if (legality_gate_enabled_) {
                obs_frame = LaneLegalityGate::filter(
                    obs_frame, last_legality_report_, last_main_track_id_, legality_allow_soft_);
                lanes = LaneLegalityGate::filter_legacy(
                    lanes, last_legality_report_, last_main_track_id_, legality_allow_soft_);
            }

            // Resolve legacy directionless intents from /avs/cmd
            if (current_intent_ == RouteIntent::LEGACY_TURN) {
                for (const auto& l : lanes) {
                    if (l.label == LABEL_TURN_LANE) {
                        double avg_x = LegacyLaneModel::get_candidate_average_x(l);
                        if (avg_x > 0.0) {
                            current_intent_ = RouteIntent::TURN_RIGHT;
                            RCLCPP_INFO(this->get_logger(), "Resolved LEGACY_TURN to TURN_RIGHT based on perception");
                            break;
                        } else if (avg_x < 0.0) {
                            current_intent_ = RouteIntent::TURN_LEFT;
                            RCLCPP_INFO(this->get_logger(), "Resolved LEGACY_TURN to TURN_LEFT based on perception");
                            break;
                        }
                    }
                }
            } else if (current_intent_ == RouteIntent::LEGACY_LANE_CHANGE) {
                for (const auto& l : lanes) {
                    if (l.label == LABEL_OTHER_LANE) {
                        double avg_x = LegacyLaneModel::get_candidate_average_x(l);
                        if (avg_x > 0.0) {
                            current_intent_ = RouteIntent::LANE_CHANGE_RIGHT;
                            RCLCPP_INFO(this->get_logger(), "Resolved LEGACY_LANE_CHANGE to LANE_CHANGE_RIGHT based on perception");
                            break;
                        } else if (avg_x < 0.0) {
                            current_intent_ = RouteIntent::LANE_CHANGE_LEFT;
                            RCLCPP_INFO(this->get_logger(), "Resolved LEGACY_LANE_CHANGE to LANE_CHANGE_LEFT based on perception");
                            break;
                        }
                    }
                }
            }

            // ── Collect lane objects by label ───────────────────────────────
            const LaneCandidate* main_current = nullptr;
            const LaneCandidate* main_ahead = nullptr;
            LegacyLaneModel::split_main_lanes(lanes, main_current, main_ahead, last_main_track_id_);

            const LaneCandidate* other_lane_cand = nullptr;
            const LaneCandidate* turn_lane_cand = nullptr;
            bool stop_line_detected = false;

            for (const auto& l : lanes) {
                if (l.label == LABEL_OTHER_LANE) other_lane_cand = &l;
            }
            for (const auto& m : markings) {
                if (m.label == LABEL_STOP_LINE) stop_line_detected = true;
            }

            bool is_t_geom = false;
            bool is_t = legacy_model_.detect_t_junction(main_current, main_ahead, lanes, is_t_geom);
            bool t_junction_pending = is_t_geom && !is_t;

            // Select the turn lane candidate based on the active turning intent.
            // current_intent_ is the sole source of truth here (Plan A4) - state_ only
            // reflects last frame's committed trajectory and can lag behind a fresh
            // intent change mid-maneuver, which would pick the wrong-side lane.
            bool is_right_turn = (current_intent_ == RouteIntent::TURN_RIGHT);
            turn_lane_cand = LegacyLaneModel::select_turn_lane(lanes, is_right_turn, is_t);

            // ── Frozen turn execution: latch / advance / release ─────────────
            // Runs before update_lane_state so the frame-count dropout and abort
            // accounting in there can be skipped for as long as the latch owns
            // the maneuver's lifetime.
            update_turn_latch(turn_lane_cand, main_current, dt, now);

            // ── State transition logic ──────────────────────────────────────
            update_lane_state(lanes, markings, main_current, turn_lane_cand, stop_line_detected, is_t);
            // update_lane_state() may itself abort a latched maneuver this frame
            // (turn/lane-change target gone > intent_abort_frames_) and set
            // hold_reason_ = "intent_aborted_dropout" while resetting current_intent_
            // to FOLLOW_MAIN. Once that happens, maneuver_pending below is false, so
            // none of the branches that follow would ever re-derive that reason -
            // clearing it unconditionally would silently lose it for this frame.
            if (hold_reason_ != "intent_aborted_dropout") {
                hold_reason_.clear();
            }

            // ── Plan F auto-return (F3) ─────────────────────────────────────
            // The internal override lives exactly as long as the lane-change
            // intent it set: once that intent is gone (maneuver completed,
            // aborted, or replaced by a real /avs/route_intent - the callback
            // clears the flag as well), the override is over.
            if (legality_return_active_ &&
                current_intent_ != RouteIntent::LANE_CHANGE_LEFT &&
                current_intent_ != RouteIntent::LANE_CHANGE_RIGHT) {
                legality_return_active_ = false;
            }
            bool return_eligible = legality_gate_enabled_ && legality_return_enabled_ &&
                !legality_return_active_ &&
                current_intent_ == RouteIntent::FOLLOW_MAIN &&
                !is_maneuver_intent(committed_state_.committed_intent);
            LegalityAutoReturn::Decision return_decision = legality_auto_return_.step(
                return_eligible, obs_frame, last_legality_report_, last_main_track_id_,
                legality_return_debounce_frames_);
            illegal_current_streak_ = legality_auto_return_.streak();
            if (return_decision.trigger) {
                // Same reachability bar as a user-requested lane change: only
                // fire the internal intent when select_other_lane already sees
                // a candidate on that side, otherwise keep the (exempt) lane
                // and let the debounce re-arm - never trade a real path for a
                // "lane_change_target_not_detected" hold we created ourselves.
                const LaneCandidate* return_target =
                    LegacyLaneModel::select_other_lane(lanes, main_current, return_decision.go_left);
                if (return_target != nullptr) {
                    current_intent_ = return_decision.go_left ? RouteIntent::LANE_CHANGE_LEFT
                                                              : RouteIntent::LANE_CHANGE_RIGHT;
                    current_intent_seq_ = next_route_intent_seq_++;
                    current_intent_age_frames_ = 0;
                    blocked_intent_counter_ = 0;
                    maneuver_dropout_counter_ = 0;
                    maneuver_target_seen_since_intent_ = false;
                    target_seen_streak_ = 0;
                    legality_return_active_ = true;
                    RCLCPP_WARN(this->get_logger(),
                        "Legality auto-return: followed lane ILLEGAL %d frames, internal intent -> %s (target %s)",
                        legality_return_debounce_frames_, route_intent_name(current_intent_),
                        return_decision.target_lane_id.c_str());
                }
            }

            if (current_intent_ == RouteIntent::FOLLOW_MAIN) {
                current_intent_age_frames_ = 0;
            } else {
                current_intent_age_frames_++;
            }

            std::string follow_main_last_main_id = last_main_track_id_;
            PlannedTrajectory follow_main_candidate =
                TrajectoryPlanner::plan_follow_main(obs_frame, committed_state_, follow_main_last_main_id);

            // Post-latch stub lifetime. from_direct_observation is the exact
            // signal wanted here: it is true only when plan_follow_main built
            // the path from a lane it actually saw this frame, and false for
            // every memory replay - including the stub regenerating itself.
            bool post_latch_stub_expired = false;
            if (post_latch_stub_frames_ >= 0) {
                if (follow_main_candidate.from_direct_observation) {
                    RCLCPP_INFO(this->get_logger(),
                        "New road acquired %d frames after the turn - stub handed over.",
                        post_latch_stub_frames_);
                    post_latch_stub_frames_ = -1;
                } else if (++post_latch_stub_frames_ > post_latch_stub_max_frames_) {
                    post_latch_stub_expired = true;
                    post_latch_stub_frames_ = -1;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "No main lane since the turn ended (%d/%d frames) - following a "
                        "straight stub, not the lane centre.",
                        post_latch_stub_frames_, post_latch_stub_max_frames_);
                }
            }

            std::string intent_last_main_id = last_main_track_id_;
            PlannedTrajectory planned_candidate = TrajectoryPlanner::plan_candidate_for_intent(
                obs_frame, current_intent_, committed_state_, is_t, t_junction_pending, intent_last_main_id);
            last_candidate_trajectory_kind_ = trajectory_kind_name(planned_candidate.trajectory_kind);

            ActiveTrajectory active_traj;
            bool blocked_by_marking = false;
            std::string selected_lane_id = "";
            const LaneCandidate* active_target_lane = main_current;

            bool maneuver_pending = is_maneuver_intent(current_intent_);
            bool committed_maneuver_active = is_maneuver_intent(committed_state_.committed_intent);
            bool should_use_follow_main_fallback = false;
            bool blocked_maneuver = false;
            bool force_follow_main_commit = false;

            if (current_intent_ == RouteIntent::TURN_LEFT && is_t && planned_candidate.valid) {
                ActiveTrajectory tentative_turn;
                tentative_turn.points = planned_candidate.points;
                tentative_turn.valid = true;
                if (LegacyLaneModel::is_turn_blocked_by_solid(tentative_turn, markings)) {
                    blocked_maneuver = true;
                }
            }

            if ((current_intent_ == RouteIntent::LANE_CHANGE_LEFT ||
                 current_intent_ == RouteIntent::LANE_CHANGE_RIGHT) &&
                planned_candidate.blocked_by_marking) {
                blocked_maneuver = true;
            }

            // Geometry looks like a possible T-junction but isn't confirmed yet
            // (is_t_geom true, is_t false): hold off on committing a turn until the
            // T-junction signal is confirmed, regardless of hold/commit state.
            if ((current_intent_ == RouteIntent::TURN_RIGHT || current_intent_ == RouteIntent::TURN_LEFT) &&
                t_junction_pending && main_current != nullptr) {
                should_use_follow_main_fallback = true;
                force_follow_main_commit = true;
                hold_reason_ = "t_junction_pending";
            }

            if (maneuver_pending && !committed_maneuver_active &&
                (current_intent_ == RouteIntent::TURN_RIGHT || current_intent_ == RouteIntent::TURN_LEFT) &&
                !legacy_model_.is_turn_commit_ready(turn_lane_cand, turn_proximity_mm_)) {
                should_use_follow_main_fallback = true;
                hold_reason_ = "turn_not_in_range";
            }

            if (maneuver_pending && !committed_maneuver_active &&
                (current_intent_ == RouteIntent::LANE_CHANGE_LEFT || current_intent_ == RouteIntent::LANE_CHANGE_RIGHT) &&
                !blocked_maneuver &&
                LegacyLaneModel::select_other_lane(lanes, main_current, current_intent_ == RouteIntent::LANE_CHANGE_LEFT) == nullptr) {
                // Mirrors the turn's "not in range yet" hold above: while no candidate
                // other-lane is visible at all, keep the manager on FOLLOW_MAIN instead
                // of letting it see committed_intent(FOLLOW_MAIN) != manager_intent
                // (LANE_CHANGE_*) every frame, which would force a COMMIT_NEW/"intent_change"
                // replan on every single frame instead of settling into HOLD_CURRENT.
                should_use_follow_main_fallback = true;
                hold_reason_ = "lane_change_target_not_detected";
                lane_change_gate_debug_ = LegacyLaneModel::diagnose_other_lane_gates(
                    lanes, main_current, current_intent_ == RouteIntent::LANE_CHANGE_LEFT);
            } else {
                lane_change_gate_debug_ = json::array();
            }

            if (maneuver_pending && committed_maneuver_active &&
                maneuver_dropout_counter_ > maneuver_dropout_hold_frames_) {
                should_use_follow_main_fallback = true;
                force_follow_main_commit = true;
                if (hold_reason_.empty()) {
                    hold_reason_ = "maneuver_dropout_hold_exceeded";
                }
            }

            if (blocked_maneuver) {
                // The only thing allowed to cut a frozen turn short: a solid
                // marking forbidding it is a rule, not a perception guess, so it
                // outranks the latch's "run the path to the end" contract.
                if (turn_latch_active_) {
                    release_turn_latch("latch_blocked_by_marking");
                }
                blocked_by_marking = true;
                should_use_follow_main_fallback = true;
                force_follow_main_commit = true;
                hold_reason_ = "blocked_by_marking";
                blocked_intent_counter_++;
            } else {
                blocked_intent_counter_ = 0;
            }

            if (blocked_intent_counter_ > intent_abort_frames_) {
                current_intent_ = RouteIntent::FOLLOW_MAIN;
                current_intent_seq_ = 0;
                current_intent_age_frames_ = 0;
                blocked_intent_counter_ = 0;
                maneuver_dropout_counter_ = 0;
                maneuver_target_seen_since_intent_ = false;
                target_seen_streak_ = 0;
                should_use_follow_main_fallback = true;
                force_follow_main_commit = true;
                hold_reason_ = "intent_aborted_blocked";
            }

            if (should_use_follow_main_fallback) {
                planned_candidate = follow_main_candidate;
                last_main_track_id_ = follow_main_last_main_id;
                if (blocked_by_marking && planned_candidate.trajectory_kind == TrajectoryKind::FOLLOW_MAIN) {
                    planned_candidate.trajectory_kind = TrajectoryKind::BLOCKED_FOLLOW_MAIN;
                    // Mirrors plan_lane_change_generic's own blocked_by_marking flag (set on
                    // the struct field, not just the local var) so the normalizer's
                    // blocked_passthrough guard and the manager's blocked-by-rule commit
                    // path both recognize this the same way regardless of which maneuver
                    // triggered the block.
                    planned_candidate.blocked_by_marking = true;
                }
            } else {
                last_main_track_id_ = intent_last_main_id;
            }

            PlannedTrajectory normalized_candidate = TrajectoryNormalizer::normalize(
                planned_candidate, committed_state_,
                continuity_heading_jump_rad_, continuity_lateral_jump_mm_);
            active_traj.debug_trajectories.push_back(LegacyLaneModel::json_from_planned_trajectory(planned_candidate, "candidate"));
            active_traj.debug_trajectories.push_back(LegacyLaneModel::json_from_planned_trajectory(normalized_candidate, "normalized"));

            frame_count_++;

            RouteIntent manager_intent = current_intent_;
            uint64_t manager_intent_seq = current_intent_seq_;
            if (force_follow_main_commit ||
                (should_use_follow_main_fallback && !committed_maneuver_active)) {
                manager_intent = RouteIntent::FOLLOW_MAIN;
                manager_intent_seq = 0;
            }
            // The manager always stores committed FOLLOW_MAIN with seq 0 (see
            // committed_intent_seq normalization in trajectory_manager.hpp), while
            // route intents arrive with seq >= 1. Passing the route seq through for
            // FOLLOW_MAIN reads as an intent change on every single frame, forcing a
            // COMMIT_NEW "intent_change" replan each frame and disabling all
            // hold/hysteresis in the manager.
            if (manager_intent == RouteIntent::FOLLOW_MAIN) {
                manager_intent_seq = 0;
            }

            // Snapshot for the ego-motion bookkeeping below: the manager's HOLD
            // branches assign previous_state.trajectory verbatim, so geometry
            // that comes out identical to this is replayed memory, and geometry
            // that differs was refreshed from the current observation. Comparing
            // the points is exact and survives reason strings being renamed.
            std::vector<Point2D> geometry_before_update = committed_state_.trajectory.points;

            TrajectoryManager::Decision decision = TrajectoryManager::update(
                normalized_candidate,
                committed_state_,
                manager_intent,
                manager_intent_seq,
                maneuver_dropout_hold_frames_,
                consecutive_invalid_frames_,
                frame_count_,
                replan_lateral_rms_mm_,
                hold_lateral_rms_mm_,
                min_overlap_ratio_,
                replan_min_confidence_,
                low_conf_hold_frames_,
                hold_min_remaining_s_mm_,
                min_path_length_mm_
            );
            committed_state_ = decision.next_state;

            // Frozen turn execution overrides whatever the manager decided. While
            // latched, perception cannot see the maneuver at all, so the manager
            // is only ever choosing among follow_main paths leading across the
            // junction - following any of them means driving straight through the
            // turn. The latched path is replayed instead, re-expressed into the
            // vehicle frame implied by how far along it the vehicle has travelled.
            if (turn_latch_active_) {
                std::vector<Point2D> remaining = TrajectoryLatch::re_express(
                    turn_latch_path_, turn_latch_progress_mm_, turn_latch_heading_error_rad_);
                // One point is enough: publish_control_error_from_trajectory
                // prepends the vehicle origin, so a lone point is still a real
                // segment to aim at. Requiring two would drop the last stretch of
                // every turn - the very truncation this mechanism prevents.
                if (!remaining.empty()) {
                    committed_state_.trajectory.points = remaining;
                    committed_state_.trajectory.valid = true;
                    committed_state_.trajectory.trajectory_kind = turn_latch_kind_;
                    committed_state_.trajectory.confidence = turn_latch_confidence_;
                    committed_state_.trajectory.blocked_by_marking = false;
                    committed_state_.trajectory.normalization_mode = "turn_latch";
                    // A stale precomputed epsilon would bypass the path entirely in
                    // publish_control_error_from_trajectory - the replayed geometry
                    // must be what the lookahead is evaluated against.
                    committed_state_.trajectory.has_precomputed_control = false;
                    committed_state_.progress_s_mm = turn_latch_progress_mm_;
                    committed_state_.remaining_s_mm = turn_latch_length_mm_ - turn_latch_progress_mm_;
                    committed_state_.committed_intent = current_intent_;
                    committed_state_.committed_intent_seq = current_intent_seq_;
                    committed_state_.replan_reason = "turn_latch";
                    hold_reason_ = "turn_latch_active";
                } else {
                    // Nothing left to follow even though the progress integral has
                    // not reached the recorded length (degenerate tail segments).
                    release_turn_latch("latch_path_consumed");
                }
            }

            // The stub ran out of frames without perception ever producing a
            // lane. Everything downstream of here has been steering on geometry
            // that describes nothing observed, so stop asserting a path exists:
            // recovery is the honest state, and it also breaks the regeneration
            // loop, since plan_follow_main's dropout branch requires a valid
            // previous trajectory to replay.
            if (post_latch_stub_expired) {
                RCLCPP_WARN(this->get_logger(),
                    "No main lane for %d frames after the turn - dropping the stub and "
                    "entering recovery.", post_latch_stub_max_frames_);
                committed_state_.trajectory.valid = false;
                committed_state_.trajectory.points.clear();
                committed_state_.trajectory.source_lane_ids.clear();
                committed_state_.trajectory.target_lane_id.clear();
                committed_state_.trajectory.trajectory_kind = TrajectoryKind::UNKNOWN;
                committed_state_.trajectory.confidence = 0.0;
                committed_state_.trajectory.normalization_mode = "none";
                committed_state_.progress_s_mm = 0.0;
                committed_state_.remaining_s_mm = 0.0;
                committed_state_.replan_reason = "post_latch_stub_expired";
                hold_reason_ = "post_latch_stub_expired";
            }

            // ── Ego-motion: track which frame the committed geometry lives in ─
            // Every path here is expressed in the vehicle frame of the frame
            // that produced it. That is exact while it is used in the same frame
            // it was made, and wrong the moment it is carried forward - which
            // happens on every HOLD. committed_yaw_rad_ is the yaw the geometry
            // currently in committed_state_ was built at, so the gap between it
            // and current_yaw_rad_ is exactly how stale that heading is.
            //
            // This deliberately does NOT rewrite committed_state_. An earlier
            // revision rotated the memory in place, which looked like it fixed
            // every replay site at once - but committed_state_ is also the
            // reference the manager measures the fresh candidate against
            // (lateral_rms_mm, overlap_ratio, topology_changed), and every one
            // of those thresholds was tuned against UNROTATED memory. Rotating
            // it changed when the system switches paths: held geometry started
            // looking plausible enough to hold far longer after a turn, and a
            // wrong-signed yaw source could inflate the deviation instead and
            // throw the turn away mid-manoeuvre. The correction belongs on the
            // published path only, where it cannot perturb any decision.
            bool geometry_is_fresh =
                turn_latch_active_ ||  // re_express rebuilds the arc every frame
                !committed_state_.trajectory.valid ||
                committed_state_.trajectory.points.size() != geometry_before_update.size();
            if (!geometry_is_fresh) {
                for (size_t i = 0; i < geometry_before_update.size(); ++i) {
                    if (committed_state_.trajectory.points[i].x != geometry_before_update[i].x ||
                        committed_state_.trajectory.points[i].y != geometry_before_update[i].y) {
                        geometry_is_fresh = true;
                        break;
                    }
                }
            }
            if (has_yaw_ && geometry_is_fresh) committed_yaw_rad_ = current_yaw_rad_;

            // ── Publish-time correction (rotation only, on a COPY) ────────────
            // committed_state_ stays exactly as the manager left it so the next
            // frame's decisions see the same reference they always have.
            CommittedTrajectoryState published_state = committed_state_;

            ego_yaw_delta_rad_ = 0.0;
            if (ego_yaw_compensation_enabled_ && has_yaw_ &&
                published_state.trajectory.points.size() >= 2) {
                ego_yaw_delta_rad_ =
                    EgoMotion::frame_delta_from_ros_yaw(current_yaw_rad_, committed_yaw_rad_);
                if (std::abs(ego_yaw_delta_rad_) > 1e-4) {
                    published_state.trajectory.points = EgoMotion::rotate_into_frame(
                        published_state.trajectory.points, ego_yaw_delta_rad_);
                }
            }

            // ── Latency compensation ─────────────────────────────────────────
            // Even freshly refreshed geometry describes where the lane was
            // output_age_ms ago (~150-250ms on the Pi), not where it is, and the
            // vehicle is steered from it right now. Extrapolating the observed
            // yaw rate over that age is a guess, which is why it now defaults
            // OFF: unlike the rotation above it changes epsilon_x_mm/theta_rad
            // by an amount nothing has measured. Turn it on deliberately.
            latency_yaw_rad_ = 0.0;
            if (latency_compensation_enabled_ && has_yaw_ &&
                published_state.trajectory.points.size() >= 2) {
                // Clamped: a yaw rate held constant over a long gap stops being
                // an estimate. Absent output_age_ms -> 0 -> no compensation.
                double age_s = std::min(latest_output_age_ms_ / 1000.0, latency_compensation_max_s_);
                latency_yaw_rad_ = EgoMotion::frame_delta_from_ros_yaw(yaw_rate_rps_ * age_s, 0.0);
                published_state.trajectory.points = EgoMotion::rotate_into_frame(
                    published_state.trajectory.points, latency_yaw_rad_);
            }

            active_traj.debug_trajectories.push_back(LegacyLaneModel::json_from_planned_trajectory(published_state.trajectory, "committed"));

            legacy_model_.populate_active_trajectory_from_committed(active_traj, obs_frame, published_state);
            selected_lane_id = committed_state_.trajectory.target_lane_id;

            if (!selected_lane_id.empty()) {
                for (const auto& l : lanes) {
                    if (LegacyLaneModel::lane_id_string(&l) == selected_lane_id) {
                        active_target_lane = &l;
                        break;
                    }
                }
            }

            // The manager's own verdict does not apply while the latch is driving:
            // it saw no maneuver candidate and may well have entered recovery, but
            // committed_state_ has just been overridden with a valid latched path.
            if (!turn_latch_active_ &&
                (decision.action == ManagerAction::ENTER_RECOVERY || !committed_state_.trajectory.valid)) {
                state_ = DecisionState::RECOVERY;
            } else if (blocked_by_marking) {
                state_ = DecisionState::BLOCKED;
            } else {
                switch (committed_state_.trajectory.trajectory_kind) {
                    case TrajectoryKind::TURN_RIGHT:
                        state_ = DecisionState::TURN_RIGHT;
                        break;
                    case TrajectoryKind::TURN_LEFT:
                        state_ = DecisionState::TURN_LEFT;
                        break;
                    case TrajectoryKind::LANE_CHANGE_LEFT:
                    case TrajectoryKind::LANE_CHANGE_RIGHT:
                        state_ = DecisionState::LANE_CHANGE;
                        break;
                    case TrajectoryKind::FOLLOW_MAIN:
                    case TrajectoryKind::BLOCKED_FOLLOW_MAIN:
                    case TrajectoryKind::UNKNOWN:
                    default:
                        state_ = DecisionState::FOLLOW_MAIN;
                        break;
                }
            }

            // ── Extract and publish control errors ──────────────────────────
            if (active_traj.valid) {
                double lookahead_d = 600.0;
                if (active_target_lane && active_target_lane->raw_obj.contains("lookahead_d_mm")) {
                    lookahead_d = active_target_lane->raw_obj["lookahead_d_mm"].get<double>();
                } else if (main_current && main_current->raw_obj.contains("lookahead_d_mm")) {
                    lookahead_d = main_current->raw_obj["lookahead_d_mm"].get<double>();
                }

                // Hybrid Control Policy: for straight-line following states (FOLLOW_MAIN, BLOCKED, RECOVERY),
                // prioritize direct polynomial lookahead from IPM to completely eliminate lateral drift bias,
                // BUT only if the upcoming connected trajectory does not diverge significantly in lateral position (< 100mm),
                // heading angle (< 0.05 rad / ~3 degrees), and curvature (< 1e-5) from the local polynomial.
                // This ensures we keep stable direct IPM lookahead on segmented straight roads, while immediately yielding to
                // the connected trajectory at any turn entry, gentle curve, or junction bend to steer early and prevent understeer.
                bool use_direct_lookahead = false;
                if (!maneuver_pending && !turn_latch_active_ &&
                    (state_ == DecisionState::FOLLOW_MAIN || state_ == DecisionState::BLOCKED || state_ == DecisionState::RECOVERY)) {
                    if (main_current && main_current->raw_obj.contains("lookahead_x_mm") && main_current->raw_obj.contains("lookahead_d_mm") &&
                        direct_lookahead_within_span(*main_current)) {
                        if (!main_ahead) {
                            use_direct_lookahead = true; // No continuation: safely use stable direct IPM
                        } else {
                            // Evaluate the connected trajectory's parameters at the lookahead distance
                            LegacyLaneModel::TrajectoryErrorParams traj_params = LegacyLaneModel::evaluate_trajectory_at_lookahead(active_traj, lookahead_d);
                            
                            double direct_x = main_current->raw_obj["lookahead_x_mm"].get<double>();
                            double direct_d = main_current->raw_obj["lookahead_d_mm"].get<double>();
                            double direct_theta = main_current->raw_obj.value("lookahead_theta_rad", std::atan2(direct_x, direct_d));
                            double direct_curvature = main_current->raw_obj.value("curvature_inv_mm", 0.0);
                            
                            bool lateral_match = std::abs(traj_params.point.x - direct_x) < 100.0;
                            bool heading_match = std::abs(traj_params.theta - direct_theta) < 0.05; // ~3 degrees
                            bool curvature_match = std::abs(traj_params.curvature - direct_curvature) < 1e-5;
                            
                            if (lateral_match && heading_match && curvature_match) {
                                use_direct_lookahead = true; // Segmented straight road: use stable direct IPM
                            }
                        }
                    }
                }

                if (use_direct_lookahead) {
                    ActiveTrajectory direct_traj = active_traj;
                    direct_traj.has_precomputed_control = true;
                    direct_traj.precomputed_epsilon_x_mm = main_current->raw_obj["lookahead_x_mm"].get<double>();
                    direct_traj.precomputed_epsilon_y_mm = main_current->raw_obj["lookahead_d_mm"].get<double>();
                    direct_traj.precomputed_theta_rad = main_current->raw_obj.value("lookahead_theta_rad",
                        std::atan2(direct_traj.precomputed_epsilon_x_mm, direct_traj.precomputed_epsilon_y_mm));
                    direct_traj.precomputed_curvature_inv_mm = main_current->raw_obj.value("curvature_inv_mm", 0.0);
                    direct_traj.precomputed_lookahead_d_mm = direct_traj.precomputed_epsilon_y_mm;
                    publish_control_error_from_trajectory(direct_traj, lookahead_d, "direct_ipm");
                } else {
                    publish_control_error_from_trajectory(active_traj, lookahead_d, "trajectory_manager");
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[%s] Target trajectory invalid — publishing invalid control error.",
                    decision_state_name(state_));
                // Publish invalid control error with safe default errors (trajectory_valid: false)
                publish_control_error_from_trajectory(active_traj, 600.0, "trajectory_manager");
            }

            // ── Always publish lane state ───────────────────────────────────
            bool has_main_raw = (main_current != nullptr || main_ahead != nullptr);
            bool has_other_raw = false;
            bool has_turn_raw = false;
            for (const auto& l : lanes) {
                if (l.label == LABEL_OTHER_LANE) has_other_raw = true;
                if (l.label == LABEL_TURN_LANE) has_turn_raw = true;
            }

            publish_lane_state(
                has_main_raw,
                has_other_raw,
                has_turn_raw,
                stop_line_detected,
                blocked_by_marking,
                active_traj,
                selected_lane_id
            );

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "telemetry_callback error: %s", e.what());
        }
    }

    ActiveTrajectory connect_two_lanes_smooth(const LaneCandidate& current_lane, const LaneCandidate& ahead_lane) {
        ActiveTrajectory traj = LegacyLaneModel::build_trajectory_from_candidate(current_lane);
        ActiveTrajectory traj_ahead = LegacyLaneModel::build_trajectory_from_candidate(ahead_lane);

        LegacyLaneModel::synthesize_precomputed_points(traj);
        LegacyLaneModel::synthesize_precomputed_points(traj_ahead);
        
        if (traj.points.size() < 2 || traj_ahead.points.size() < 2) {
            if (traj.points.size() < 2 && traj_ahead.points.size() >= 2) {
                return traj_ahead;
            }
            if (traj.points.size() >= 2 && traj_ahead.points.size() < 2) {
                return traj;
            }
            return (traj_ahead.points.size() > traj.points.size()) ? traj_ahead : traj;
        }
        
        // Safety guard: if they are too far apart laterally or longitudinally, do not connect
        Point2D P0 = traj.points.back();
        Point2D P3 = traj_ahead.points.front();
        double gap_y = P3.y - P0.y;
        double jump_x = std::abs(P3.x - P0.x);
        if (gap_y < -500.0 || gap_y > 2500.0 || jump_x > 500.0) {
            RCLCPP_WARN(this->get_logger(), "Geometric continuity check failed in connect_two_lanes_smooth. Gap Y: %.1f, Jump X: %.1f. Aborting connection.", gap_y, jump_x);
            return traj;
        }
        
        traj.has_precomputed_control = false;
        
        Point2D p_prev = traj.points[traj.points.size() - 2];
        double dx0 = P0.x - p_prev.x;
        double dy0 = P0.y - p_prev.y;
        double len0 = std::sqrt(dx0*dx0 + dy0*dy0);
        if (len0 < 1e-3) len0 = 1.0;
        dx0 /= len0; dy0 /= len0;
        
        Point2D p_next = traj_ahead.points[1];
        double dx3 = p_next.x - P3.x;
        double dy3 = p_next.y - P3.y;
        double len3 = std::sqrt(dx3*dx3 + dy3*dy3);
        if (len3 < 1e-3) len3 = 1.0;
        dx3 /= len3; dy3 /= len3;
        
        double dist = std::sqrt((P3.x - P0.x)*(P3.x - P0.x) + (P3.y - P0.y)*(P3.y - P0.y));
        double scale = dist / 3.0;
        
        Point2D P1 = { P0.x + dx0 * scale, P0.y + dy0 * scale };
        Point2D P2 = { P3.x - dx3 * scale, P3.y - dy3 * scale };
        
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
            traj.points.push_back({bx, by});
        }
        
        for (const auto& pt : traj_ahead.points) {
            traj.points.push_back(pt);
        }
        
        traj.source_labels.push_back(ahead_lane.label);
        traj.trajectory_kind = "follow_main_connected";
        return traj;
    }

    ActiveTrajectory transition_to_lane(const LaneCandidate& current_lane, const LaneCandidate& target_lane) {
        ActiveTrajectory traj = LegacyLaneModel::build_trajectory_from_candidate(current_lane);
        ActiveTrajectory traj_target = LegacyLaneModel::build_trajectory_from_candidate(target_lane);

        LegacyLaneModel::synthesize_precomputed_points(traj);
        LegacyLaneModel::synthesize_precomputed_points(traj_target);
        
        if (traj_target.valid && traj_target.has_precomputed_control) {
            return traj_target;
        }
        
        if (traj.points.size() < 2 || traj_target.points.size() < 2) {
            ActiveTrajectory invalid;
            invalid.source_labels = traj.source_labels;
            invalid.source_labels.push_back(target_lane.label);
            invalid.trajectory_kind = "invalid_transition";
            invalid.valid = false;
            return invalid;
        }

        // Safety guard: if target lane is too far laterally (e.g. > 1500mm) or heading is too divergent, abort transition
        double cur_x = traj.points.front().x;
        double target_x = traj_target.points.front().x;
        double lat_dist = std::abs(target_x - cur_x);
        double cur_heading = LegacyLaneModel::get_lane_heading(current_lane);
        double target_heading = LegacyLaneModel::get_lane_heading(target_lane);
        double heading_diff = std::abs(target_heading - cur_heading);
        while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
        heading_diff = std::abs(heading_diff);

        if (lat_dist < 300.0 || lat_dist > 1500.0 || heading_diff > (40.0 * M_PI / 180.0)) {
            RCLCPP_WARN(this->get_logger(), "Transition safety check failed! Lateral distance: %.1f, Heading diff: %.2f rad. Staying in current lane.", lat_dist, heading_diff);
            return traj;
        }

        Point2D P0 = traj.points.front();
        Point2D p_prev = P0;
        double cum_dist = 0.0;
        size_t split_idx_current = 0;
        for (size_t i = 1; i < traj.points.size(); ++i) {
            cum_dist += std::hypot(traj.points[i].x - traj.points[i-1].x, traj.points[i].y - traj.points[i-1].y);
            if (cum_dist >= 300.0) {
                P0 = traj.points[i];
                p_prev = traj.points[i-1];
                split_idx_current = i;
                break;
            }
        }
        if (split_idx_current == 0 && traj.points.size() > 1) {
            split_idx_current = 1;
            P0 = traj.points[1];
            p_prev = traj.points[0];
        }

        Point2D P3 = traj_target.points.back();
        Point2D p_next = P3;
        cum_dist = 0.0;
        size_t split_idx_target = traj_target.points.size() - 1;
        for (size_t i = 1; i < traj_target.points.size(); ++i) {
            cum_dist += std::hypot(traj_target.points[i].x - traj_target.points[i-1].x, traj_target.points[i].y - traj_target.points[i-1].y);
            if (cum_dist >= 1200.0) {
                P3 = traj_target.points[i];
                p_next = (i + 1 < traj_target.points.size()) ? traj_target.points[i+1] : P3;
                split_idx_target = i;
                break;
            }
        }
        if (split_idx_target == traj_target.points.size() - 1 && traj_target.points.size() > 1) {
            split_idx_target = traj_target.points.size() / 2;
            if (split_idx_target == 0) split_idx_target = 1;
            P3 = traj_target.points[split_idx_target];
            p_next = (split_idx_target + 1 < traj_target.points.size()) ? traj_target.points[split_idx_target+1] : P3;
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
        double scale = dist / 3.0;
        
        Point2D P1 = { P0.x + dx0 * scale, P0.y + dy0 * scale };
        Point2D P2 = { P3.x - dx3 * scale, P3.y - dy3 * scale };

        ActiveTrajectory result;
        result.source_labels = traj.source_labels;
        result.source_labels.push_back(target_lane.label);

        for (size_t i = 0; i <= split_idx_current; ++i) {
            result.points.push_back(traj.points[i]);
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
            result.points.push_back({bx, by});
        }

        for (size_t i = split_idx_target; i < traj_target.points.size(); ++i) {
            result.points.push_back(traj_target.points[i]);
        }

        result.trajectory_kind = "transition";
        result.valid = (result.points.size() >= 2);
        return result;
    }

    // ── Lane state transition logic ──────────────────────────────────────────
    void update_lane_state(const std::vector<LaneCandidate>& lanes,
                           const std::vector<MarkingCandidate>& markings,
                           const LaneCandidate* main_current,
                           const LaneCandidate* turn_lane_cand,
                           bool stop_line_detected,
                           bool is_t) {
        
        // State machine's only role now is maneuver completion and intent timeout.
        // Feasibility and holding are handled by TrajectoryManager and maneuver fallbacks.
        last_processed_intent_ = current_intent_;

        // Gated on current_intent_ (the latched pending intent), not state_: state_ only
        // reflects the last committed trajectory kind and can fall back to FOLLOW_MAIN
        // while the maneuver intent is still latched (hold window exceeded). Gating on
        // state_ here would freeze maneuver_dropout_counter_ forever once that fallback
        // happens, so intent_abort_frames_ would never be reached.
        // Frozen turn execution owns the maneuver's lifetime while it runs: its
        // exit is distance-based (plus a wall-clock backstop in
        // update_turn_latch), so neither the frame-count dropout budget nor the
        // completion test may fire underneath it. In particular a turn-lane that
        // flickers back mid-junction must not complete the turn early - the latch
        // is frozen hard by design.
        if (turn_latch_active_) {
            // update_turn_latch() drives this - nothing to account for here.
        }
        else if (current_intent_ == RouteIntent::TURN_RIGHT || current_intent_ == RouteIntent::TURN_LEFT) {
            // A committed maneuver counts as armed regardless of the sighting
            // streak: once the manager has committed the turn, losing the target
            // must feed the dropout fallback/abort accounting immediately.
            if (committed_state_.committed_intent == current_intent_) {
                maneuver_target_seen_since_intent_ = true;
            }
            if (turn_lane_cand != nullptr) {
                maneuver_dropout_counter_ = 0;
                target_seen_streak_++;
                if (target_seen_streak_ >= kIntentArmSeenFrames) {
                    maneuver_target_seen_since_intent_ = true;
                }
                double theta_t = 1e9;
                double long_off = 1e9;
                LegacyLaneModel::get_normalized_turn_geometry(turn_lane_cand->raw_obj, long_off, theta_t);

                bool heading_ok = (std::abs(theta_t) < theta_done_rad_);
                bool past_turn = (long_off < -turn_done_mm_);

                if (heading_ok && past_turn) {
                    current_intent_ = RouteIntent::FOLLOW_MAIN;
                    current_intent_seq_ = 0;
                    current_intent_age_frames_ = 0;
                    last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
                    blocked_intent_counter_ = 0;
                    maneuver_target_seen_since_intent_ = false;
                    target_seen_streak_ = 0;
                    hold_reason_.clear();
                    RCLCPP_INFO(this->get_logger(), "Turn complete. Intent → FOLLOW_MAIN");
                }
            } else {
                target_seen_streak_ = 0;
                // Dropout abort only applies once the turn-lane has been seen
                // stably (kIntentArmSeenFrames consecutive frames) since this
                // intent was latched. A pending intent whose target has never
                // appeared - or only flickered for a frame or two while the
                // robot was repositioned - must wait indefinitely; the
                // "turn_not_in_range" hold keeps the vehicle safely on main-lane.
                if (maneuver_target_seen_since_intent_) {
                    maneuver_dropout_counter_++;
                    if (maneuver_dropout_counter_ > intent_abort_frames_) {
                        current_intent_ = RouteIntent::FOLLOW_MAIN;
                        current_intent_seq_ = 0;
                        current_intent_age_frames_ = 0;
                        last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
                        maneuver_dropout_counter_ = 0;
                        blocked_intent_counter_ = 0;
                        maneuver_target_seen_since_intent_ = false;
                        target_seen_streak_ = 0;
                        hold_reason_ = "intent_aborted_dropout";
                        RCLCPP_WARN(this->get_logger(), "Turn-lane lost beyond intent_abort_frames_. Intent aborted → FOLLOW_MAIN");
                    } else if (maneuver_dropout_counter_ > maneuver_dropout_hold_frames_) {
                        hold_reason_ = "maneuver_dropout_hold_exceeded";
                        RCLCPP_WARN(this->get_logger(), "Turn-lane lost beyond hold window (%d frames). Intent still latched.", maneuver_dropout_counter_);
                    }
                }
            }
        }
        else if (current_intent_ == RouteIntent::LANE_CHANGE_LEFT || current_intent_ == RouteIntent::LANE_CHANGE_RIGHT) {
            if (committed_state_.committed_intent == current_intent_) {
                maneuver_target_seen_since_intent_ = true;
            }
            bool is_left = (current_intent_ == RouteIntent::LANE_CHANGE_LEFT);
            const LaneCandidate* target_other = LegacyLaneModel::select_other_lane(lanes, main_current, is_left);
            
            auto get_x = [](const LaneCandidate* l) {
                if (l->raw_obj.contains("lookahead_x_mm")) return l->raw_obj["lookahead_x_mm"].get<double>();
                if (l->raw_obj.contains("waypoints") && !l->raw_obj["waypoints"].empty()) return l->raw_obj["waypoints"][0][0].get<double>();
                return 0.0;
            };

            bool lane_change_complete = false;
            if (target_other != nullptr) {
                double target_x = get_x(target_other);
                if (std::abs(target_x) < 250.0) {
                    lane_change_complete = true;
                }
            }
            
            if (!lane_change_complete && main_current != nullptr) {
                // If we are close to the center of main_current, and the opposite other lane is detected, complete
                const LaneCandidate* opposite_other = LegacyLaneModel::select_other_lane(lanes, main_current, !is_left);
                if (opposite_other != nullptr) {
                    double opp_x = get_x(opposite_other);
                    double main_x = get_x(main_current);
                    if (is_left && opp_x > 600.0 && main_x > -250.0 && main_x < 250.0) {
                        lane_change_complete = true;
                    } else if (!is_left && opp_x < -600.0 && main_x > -250.0 && main_x < 250.0) {
                        lane_change_complete = true;
                    }
                }
            }

            if (lane_change_complete) {
                current_intent_ = RouteIntent::FOLLOW_MAIN;
                current_intent_seq_ = 0;
                current_intent_age_frames_ = 0;
                last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
                maneuver_dropout_counter_ = 0;
                blocked_intent_counter_ = 0;
                maneuver_target_seen_since_intent_ = false;
                target_seen_streak_ = 0;
                hold_reason_.clear();
                RCLCPP_INFO(this->get_logger(), "Lane change complete. Intent → FOLLOW_MAIN");
            } else if (target_other == nullptr) {
                target_seen_streak_ = 0;
                // Same pending-vs-armed rule as the turn branch above: only count
                // dropout toward the abort once the target other-lane has been
                // seen stably for this intent; before that the intent stays
                // latched under the "lane_change_target_not_detected" hold.
                if (maneuver_target_seen_since_intent_) {
                    maneuver_dropout_counter_++;
                    if (maneuver_dropout_counter_ > intent_abort_frames_) {
                        current_intent_ = RouteIntent::FOLLOW_MAIN;
                        current_intent_seq_ = 0;
                        current_intent_age_frames_ = 0;
                        last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
                        maneuver_dropout_counter_ = 0;
                        blocked_intent_counter_ = 0;
                        maneuver_target_seen_since_intent_ = false;
                        target_seen_streak_ = 0;
                        hold_reason_ = "intent_aborted_dropout";
                        RCLCPP_WARN(this->get_logger(), "Lane-change target lost beyond intent_abort_frames_. Intent aborted → FOLLOW_MAIN");
                    } else if (maneuver_dropout_counter_ > maneuver_dropout_hold_frames_) {
                        hold_reason_ = "maneuver_dropout_hold_exceeded";
                        RCLCPP_WARN(this->get_logger(), "Lane-change target lost beyond hold window (%d frames). Intent still latched.", maneuver_dropout_counter_);
                    }
                }
            } else {
                maneuver_dropout_counter_ = 0;
                target_seen_streak_++;
                if (target_seen_streak_ >= kIntentArmSeenFrames) {
                    maneuver_target_seen_since_intent_ = true;
                }
            }
        }

        (void)stop_line_detected;
        (void)markings;
        (void)is_t;
    }

    // Direct IPM lookahead evaluates the lane polynomial at lookahead_d_mm, which
    // is only a measurement when the lane's observed waypoint span covers that
    // distance; outside the span it is a cubic extrapolation. Mid-intersection the
    // surviving main lane starts far ahead of the vehicle, and extrapolating its
    // polynomial back to the 120-450mm lookahead produced garbage epsilon_x
    // (13-15m observed on real video) while the bridged trajectory stayed correct.
    static bool direct_lookahead_within_span(const LaneCandidate& lane) {
        if (!lane.raw_obj.contains("waypoints") || !lane.raw_obj["waypoints"].is_array()) return false;
        const auto& wps = lane.raw_obj["waypoints"];
        if (wps.size() < 2 || !wps.front().is_array() || wps.front().size() < 2 ||
            !wps.back().is_array() || wps.back().size() < 2) return false;
        double d_la = lane.raw_obj["lookahead_d_mm"].get<double>();
        double y_first = wps.front()[1].get<double>();
        double y_last  = wps.back()[1].get<double>();
        return d_la >= y_first && d_la <= y_last;
    }

    // ── Publish the 3 control error parameters from Active Trajectory ────────
    void publish_control_error_from_trajectory(const ActiveTrajectory& traj, double lookahead_d_mm,
                                                const std::string& control_source = "trajectory_manager") {
        double epsilon_x = 0.0;
        double epsilon_y = 0.0;
        double theta = 0.0;
        double curv = 0.0;

        if (traj.valid && traj.has_precomputed_control) {
            epsilon_x = traj.precomputed_epsilon_x_mm;
            epsilon_y = traj.precomputed_epsilon_y_mm;
            theta = traj.precomputed_theta_rad;
            curv = traj.precomputed_curvature_inv_mm;
            lookahead_d_mm = traj.precomputed_lookahead_d_mm;
        } else if (traj.valid && !traj.points.empty()) {
            LegacyLaneModel::TrajectoryErrorParams params = LegacyLaneModel::evaluate_trajectory_at_lookahead(traj, lookahead_d_mm);
            epsilon_x = params.point.x;
            epsilon_y = params.point.y;
            theta = params.theta;
            curv = params.curvature;
        }

        json out;
        out["lane_state"]    = legacy_lane_state_name(state_); // legacy key for compatibility
        out["target_label"]  = traj.source_labels.empty() ? -1 : traj.source_labels.back();
        out["epsilon_x_mm"]  = std::round(epsilon_x * 10.0) / 10.0;
        out["epsilon_y_mm"]  = std::round(epsilon_y * 10.0) / 10.0;
        out["theta_rad"]     = std::round(theta      * 1000.0) / 1000.0;
        out["curvature_inv_mm"] = curv;
        out["lookahead_d_mm"]   = lookahead_d_mm;
        out["trajectory_valid"] = traj.valid;
        out["timestamp_ms"]  = latest_telemetry_timestamp_ms_;
        out["control_source"] = control_source;
        last_control_source_ = control_source;

        std_msgs::msg::String msg;
        msg.data = out.dump();
        control_error_pub_->publish(msg);
    }

    // ── Publish lane detection state ─────────────────────────────────────────
    void publish_lane_state(bool has_main, bool has_other, bool has_turn, bool has_stop, 
                            bool blocked_by_marking, const ActiveTrajectory& traj,
                            const std::string& selected_lane_id) {
        json state_json;
        state_json["decision_state"]     = decision_state_name(state_);
        state_json["lane_state"]         = legacy_lane_state_name(state_); // legacy key for compatibility
        state_json["route_intent"]       = route_intent_name(current_intent_);
        state_json["pending_intent"]     = route_intent_name(current_intent_);
        state_json["intent_seq"]         = current_intent_seq_;
        state_json["intent_age_frames"]  = current_intent_age_frames_;
        state_json["main_lane_detected"]  = has_main;
        state_json["other_lane_detected"] = has_other;
        state_json["turn_lane_detected"]  = has_turn;
        state_json["stop_line_detected"]  = has_stop;
        state_json["blocked_by_marking"]  = blocked_by_marking;
        state_json["trajectory_valid"]    = committed_state_.trajectory.valid;
        state_json["timestamp_ms"]       = latest_telemetry_timestamp_ms_;
        state_json["control_source"]      = last_control_source_;
        
        if (!traj.debug_trajectories.empty()) {
            state_json["debug_trajectories"] = traj.debug_trajectories;
        }

        if (!selected_lane_id.empty()) {
            state_json["selected_lane_id"] = selected_lane_id;
        }

        // Phase 1 extensions
        state_json["trajectory_kind"] = trajectory_kind_name(committed_state_.trajectory.trajectory_kind);
        state_json["committed_trajectory_id"] = committed_state_.trajectory.target_lane_id;
        state_json["normalization_mode"] = committed_state_.trajectory.normalization_mode;
        state_json["trajectory_confidence"] = committed_state_.trajectory.confidence;
        state_json["dropout_hold_counter"] = committed_state_.dropout_hold_counter;
        state_json["replan_reason"] = committed_state_.replan_reason;
        // Plan A Step 1: candidate planner output for current_intent_, computed every frame
        // regardless of whether the state machine has armed the maneuver yet. Debug-only.
        state_json["candidate_trajectory_kind"] = last_candidate_trajectory_kind_;
        // Plan A Step 2 (A3): frames since the active maneuver's target lane was last detected.
        state_json["maneuver_dropout_counter"] = maneuver_dropout_counter_;
        state_json["hold_reason"] = hold_reason_;
        // Frozen turn execution progress. turn_latch_length_mm is the whole point
        // to watch on the real vehicle: if the latched path turns out too short to
        // carry the vehicle through the junction, this is where that shows up.
        // Ego-motion compensation. ego_yaw_delta_deg is how far the published
        // path was rotated to catch up with the vehicle: it grows while a path
        // is held and snaps back to ~0 the frame the geometry is refreshed, so
        // a large value is a direct readout of how long the system has been
        // coasting on memory. Pinned at exactly 0 while the vehicle is visibly
        // turning means odom yaw is not arriving - check ego_yaw_deg, which
        // would be frozen too. latency_yaw_deg is the extra extrapolation, off
        // by default.
        state_json["ego_yaw_deg"] = current_yaw_rad_ * 180.0 / M_PI;
        state_json["ego_yaw_delta_deg"] = ego_yaw_delta_rad_ * 180.0 / M_PI;
        state_json["ego_yaw_rate_dps"] = yaw_rate_rps_ * 180.0 / M_PI;
        state_json["latency_yaw_deg"] = latency_yaw_rad_ * 180.0 / M_PI;
        state_json["observation_age_ms"] = latest_output_age_ms_;

        state_json["turn_latch_active"] = turn_latch_active_;
        state_json["turn_latch_enter_dropout"] = turn_latch_enter_dropout_;
        state_json["turn_latch_heading_error_deg"] = turn_latch_heading_error_rad_ * 180.0 / M_PI;
        state_json["turn_latch_progress_mm"] = turn_latch_progress_mm_;
        state_json["turn_latch_length_mm"] = turn_latch_length_mm_;
        state_json["turn_latch_elapsed_s"] =
            turn_latch_active_ ? (this->get_clock()->now() - turn_latch_start_time_).seconds() : 0.0;
        state_json["turn_latch_release_reason"] = turn_latch_release_reason_;
        // How far the vehicle has rotated since the latch closed, derived from
        // progress_mm. This is the number to calibrate odom_speed_scale against:
        // read it as the latch releases and compare with
        // turn_latch_target_heading_deg - short means the scale is too high.
        state_json["turn_latch_heading_turned_deg"] = turn_latch_heading_turned_rad_ * 180.0 / M_PI;
        // observed vs extended span is the number to watch on the vehicle: if the
        // observed span sits below turn_latch_min_observed_span_deg the arc was
        // never continued, and the turn will come out short exactly as before.
        state_json["turn_latch_observed_span_deg"] = turn_latch_observed_span_rad_ * 180.0 / M_PI;
        state_json["turn_latch_extended_span_deg"] = turn_latch_extended_span_rad_ * 180.0 / M_PI;
        state_json["turn_latch_extension_mm"] = turn_latch_extension_mm_;
        state_json["turn_latch_deadline_s"] = turn_latch_deadline_s_;
        // Debug-only: which of select_other_lane()'s hard gates rejected each
        // other-lane candidate, populated only while holding on
        // "lane_change_target_not_detected" (see LegacyLaneModel::diagnose_other_lane_gates).
        state_json["lane_change_gate_debug"] = lane_change_gate_debug_;
        // Plan B: no dashed/solid marking detected at all between main and target
        // other-lane in the lane-change corridor - allowed by default policy, but
        // flagged here since perception may simply have missed the marking.
        state_json["marking_confidence_low"] = committed_state_.trajectory.marking_confidence_low;
        // Plan F: yellow legality gate debug (additive - no existing field changed).
        {
            json yellow_gate;
            yellow_gate["enabled"] = legality_gate_enabled_;
            yellow_gate["visible"] = last_legality_report_.yellow_visible;
            yellow_gate["age_frames"] = last_legality_report_.yellow_age_frames;
            json verdicts = json::object();
            for (const auto& kv : last_legality_report_.lane_verdicts) {
                verdicts[kv.first] = lane_legality_name(kv.second);
            }
            yellow_gate["lane_legality"] = verdicts;
            yellow_gate["allow_soft_illegal"] = legality_allow_soft_;
            yellow_gate["illegal_current_streak"] = illegal_current_streak_;
            yellow_gate["legality_return_active"] = legality_return_active_;
            yellow_gate["route_intent_source"] =
                legality_return_active_ ? "legality_gate" : "external";
            state_json["yellow_gate"] = yellow_gate;
        }
        if (committed_state_.trajectory.valid && committed_state_.trajectory.points.size() > 0) {
            json pts = json::array();
            // Subsample if too large to save bandwidth
            size_t step = 1;
            if (committed_state_.trajectory.points.size() > 50) step = committed_state_.trajectory.points.size() / 50;
            for (size_t i = 0; i < committed_state_.trajectory.points.size(); i += step) {
                pts.push_back({std::round(committed_state_.trajectory.points[i].x), std::round(committed_state_.trajectory.points[i].y)});
            }
            // always include last point
            if ((committed_state_.trajectory.points.size() - 1) % step != 0) {
                pts.push_back({std::round(committed_state_.trajectory.points.back().x), std::round(committed_state_.trajectory.points.back().y)});
            }
            state_json["active_trajectory_points"] = pts;
        }

        std_msgs::msg::String msg;
        msg.data = state_json.dump();
        lane_state_pub_->publish(msg);
    }

    // ── ROS2 interfaces ──────────────────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    control_error_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    lane_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    route_intent_ack_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr route_intent_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

    // ── State ────────────────────────────────────────────────────────────────
    std::string last_control_source_ = "trajectory_manager";
    std::string last_candidate_trajectory_kind_ = "unknown";  // Plan A Step 1 debug field
    std::string hold_reason_;
    // Debug-only: which of select_other_lane()'s 4 hard gates rejected each
    // other-lane candidate this frame, populated only while a lane_change
    // intent is held on "lane_change_target_not_detected". See
    // LegacyLaneModel::diagnose_other_lane_gates().
    json lane_change_gate_debug_ = json::array();
    DecisionState state_           = DecisionState::FOLLOW_MAIN;
    RouteIntent current_intent_    = RouteIntent::FOLLOW_MAIN;
    RouteIntent last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
    uint64_t current_intent_seq_ = 0;
    uint64_t next_route_intent_seq_ = 1;
    uint64_t current_intent_age_frames_ = 0;
    int blocked_intent_counter_ = 0;
    std::string last_main_track_id_ = "";

    // Legacy LaneCandidate-model helpers (Plan D D4.6); owns t_junction_counter_
    // internally since that state is only ever touched inside detect_t_junction.
    LegacyLaneModel legacy_model_;

    // ── Memory and Planning State (Phase 1) ──────────────────────────────────
    CommittedTrajectoryState committed_state_;
    int consecutive_invalid_frames_ = 0;
    uint64_t frame_count_ = 0;
    uint64_t latest_telemetry_timestamp_ms_ = 0;

    // ── Odometry and Speed Tracking (Phase 1/9) ──────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double current_speed_mms_ = 0.0;
    rclcpp::Time last_telemetry_time_;

    // ── Ego-motion (yaw) tracking ────────────────────────────────────────────
    // current_yaw_rad_ is the latest ROS yaw. committed_yaw_rad_ is the yaw the
    // geometry now in committed_state_ was built at - it advances only when that
    // geometry is refreshed from an observation, and deliberately stands still
    // through every HOLD. The gap between the two is how stale the held path's
    // heading is, and it is what the published path is rotated by.
    double current_yaw_rad_   = 0.0;
    double committed_yaw_rad_ = 0.0;
    bool   has_yaw_           = false;  // no odom yet -> compensate nothing
    double yaw_rate_rps_      = 0.0;    // damped, only for latency compensation
    rclcpp::Time last_odom_time_;
    // Diagnostics, published on /avs/lane_state.
    double ego_yaw_delta_rad_   = 0.0;
    double latency_yaw_rad_     = 0.0;
    double latest_output_age_ms_ = 0.0;

    // ── Thresholds ───────────────────────────────────────────────────────────
    double turn_proximity_mm_ = 500.0;  // distance to arm turn transition
    double turn_done_mm_      = 200.0;  // past-turn threshold
    double theta_done_rad_    = 0.1;    // heading threshold for turn completion

    // ── Plan A Step 2 (A3): intent latch + hold window ────────────────────────
    // Counts consecutive frames the active maneuver's target lane has gone
    // undetected. Reset to 0 whenever the target is seen again.
    int maneuver_dropout_counter_ = 0;
    int maneuver_dropout_hold_frames_ = 10;  // <= this: keep state_ in the maneuver
    int intent_abort_frames_ = 30;           // > this: clear current_intent_ entirely
    // True once the latched maneuver intent's target lane (turn-lane or target
    // other-lane) has been detected for kIntentArmSeenFrames consecutive frames
    // since the intent was accepted. The dropout-abort path above only runs when
    // this is set: a pending intent whose target has never appeared - or only
    // flickered for a frame or two - waits indefinitely instead of expiring
    // after intent_abort_frames (~2s at the real robot's 14 FPS).
    static constexpr int kIntentArmSeenFrames = 5;
    bool maneuver_target_seen_since_intent_ = false;
    int target_seen_streak_ = 0;

    // ── Frozen turn execution ────────────────────────────────────────────────
    // Once a turn is committed (so the turn-lane has already come inside
    // turn_proximity_mm), losing the turn-lane means the vehicle has reached the
    // junction and perception cannot see the maneuver any more - every candidate
    // from here is a follow_main path across the intersection. Rather than
    // replanning onto it, the last committed turn path is latched and replayed
    // until consumed. See trajectory_latch.hpp for the geometry and the
    // perfect-tracking assumption it rests on.
    bool turn_latch_active_ = false;
    std::vector<Point2D> turn_latch_path_;      // vehicle frame as of the latch frame
    double turn_latch_length_mm_ = 0.0;         // arc length of turn_latch_path_
    double turn_latch_progress_mm_ = 0.0;       // distance travelled along it
    TrajectoryKind turn_latch_kind_ = TrajectoryKind::UNKNOWN;
    double turn_latch_confidence_ = 0.0;
    rclcpp::Time turn_latch_start_time_;
    std::string turn_latch_release_reason_;  // why the last frozen turn ended
    // Diagnostics for the last latch, kept past release so the reason a turn
    // came out short is still readable in /avs/lane_state afterwards.
    double turn_latch_observed_span_rad_ = 0.0;  // turn the camera actually delivered
    double turn_latch_extended_span_rad_ = 0.0;  // turn after continuing the arc
    double turn_latch_extension_mm_ = 0.0;       // path added to reach it
    double turn_latch_deadline_s_ = 0.0;         // wall-clock backstop for this latch

    // Wall-clock backstop. The latch deliberately ignores the frame-count
    // budgets (maneuver_dropout_hold_frames_/intent_abort_frames_), so if
    // /odom_raw stalls or reports zero speed the progress integral never
    // advances and nothing else would ever end the maneuver. This is the floor;
    // latch_deadline_s() stretches it for longer paths.
    double maneuver_max_duration_s_ = 10.0;
    static constexpr double kLatchDeadlineMinSpeedMms = 150.0;
    static constexpr double kLatchDeadlineMaxS = 30.0;

    // Straight stub handed over when a latch releases with nothing left to
    // follow (release_turn_latch). Long enough to outlast the few frames
    // perception needs to label the new road, short enough that following it to
    // the end never carries the vehicle past a junction.
    static constexpr double kPostLatchStubLengthMm = 1500.0;
    static constexpr double kPostLatchStubConfidence = 0.3;

    // Frozen turn execution: carrying the latched path out to the full turn.
    double turn_latch_target_heading_deg_ = 90.0;
    double turn_latch_runout_mm_ = 700.0;
    double turn_latch_skip_lead_s_ = 0.1;
    double turn_latch_min_radius_mm_ = 800.0;
    double turn_latch_max_radius_mm_ = 4000.0;
    double turn_latch_min_observed_span_deg_ = 15.0;
    double turn_latch_release_min_span_frac_ = 0.9;
    double turn_latch_release_max_lane_heading_deg_ = 25.0;
    int turn_latch_enter_dropout_frames_ = 4;
    int turn_latch_enter_dropout_ = 0;
    std::vector<Point2D> turn_latch_fresh_obs_;
    double turn_latch_heading_correction_max_deg_ = 0.0;
    double turn_latch_runout_correction_max_deg_ = 25.0;
    double turn_latch_heading_error_rad_ = 0.0;
    double turn_latch_heading_turned_rad_ = 0.0;  // rotation so far, for /avs/lane_state
    double turn_latch_start_yaw_rad_ = 0.0;       // ROS yaw when the latch closed
    double odom_speed_scale_ = 2500.0;
    // Lifetime of the post-latch straight stub. -1 means no stub is in flight;
    // >= 0 counts frames since the latch released without a real lane observation.
    int post_latch_stub_frames_ = -1;
    int post_latch_stub_max_frames_ = 15;
    bool ego_yaw_compensation_enabled_ = true;
    bool latency_compensation_enabled_ = false;
    double latency_compensation_max_s_ = 0.4;

    // Plan C: normalizer post-blend continuity guard thresholds.
    double continuity_heading_jump_rad_ = 0.35;
    double continuity_lateral_jump_mm_ = 300.0;

    // Plan C: manager composite-deviation replan/hold policy thresholds.
    double replan_lateral_rms_mm_ = 800.0;   // > this (and confident): COMMIT_NEW
    double hold_lateral_rms_mm_ = 50.0;      // < this: HOLD_CURRENT
    double min_overlap_ratio_ = 0.5;         // < this path-length ratio: treat as replan-worthy
    double replan_min_confidence_ = 0.5;     // below this, don't trust a big deviation enough to replan
    int low_conf_hold_frames_ = 10;          // cap on holding through a low-confidence big deviation
    double hold_min_remaining_s_mm_ = 500.0; // below this remaining distance, don't hold - update instead
    double min_path_length_mm_ = 200.0;      // below this, a path is degenerate for metric purposes

    // Plan F: solid/dashed-yellow legality gate + auto-return.
    LaneLegalityGate legality_gate_;
    LaneLegalityReport last_legality_report_;
    bool legality_gate_enabled_ = true;
    bool legality_return_enabled_ = true;
    bool legality_dashed_yellow_enabled_ = true;
    double legality_margin_mm_ = 100.0;
    int legality_yellow_hold_frames_ = 10;
    int legality_return_debounce_frames_ = 5;
    double legality_beta_deg_ = 20.0;
    bool legality_allow_soft_ = false;       // debug: soft-illegal exemption active this frame
    int illegal_current_streak_ = 0;         // consecutive frames the followed lane was ILLEGAL
    bool legality_return_active_ = false;    // internal lane_change override in flight
    LegalityAutoReturn legality_auto_return_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneErrorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
