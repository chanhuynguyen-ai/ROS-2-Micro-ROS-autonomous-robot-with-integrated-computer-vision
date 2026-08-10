#pragma once

#include <cmath>
#include <vector>

#include "avs_perception/decision_types.hpp"

// Ego-motion bookkeeping for paths that outlive the frame they were built in.
//
// Every path here lives in the vehicle frame of the frame that produced it
// (x = lateral, positive right; y = forward; vehicle at the origin heading +y).
// That is exact while the path is used in the same frame it was made, and wrong
// the moment it is carried forward - which happens constantly:
// TrajectoryNormalizer blends the previous committed path with the fresh
// candidate, TrajectoryManager holds the previous path through dropouts and
// low-confidence frames, and plan_follow_main replays a remembered path when no
// lane is visible.
//
// TrajectoryPlanner::project_point_to_path already recovers how far the vehicle
// advanced ALONG such a path, but that is a scalar - it cannot express
// rotation. So while the vehicle yaws (a curve, a turn, any hard steer) the
// remembered geometry keeps the heading it was captured with, and the published
// path swings off the lane by exactly the yaw accumulated since. This header is
// the missing half of that correction.
class EgoMotion {
public:
    static double wrap_pi(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // Planar yaw from a ROS quaternion; only z/w matter for a ground vehicle.
    //
    // Taken from the orientation rather than from twist on purpose: a
    // quaternion is dimensionless, so it cannot inherit the units error that
    // odom_speed_scale exists to paper over. Whatever that scale turns out to
    // be, this stays correct.
    static double yaw_from_quaternion(double z, double w) {
        return std::atan2(2.0 * w * z, 1.0 - 2.0 * z * z);
    }

    // How much the vehicle frame itself rotated between two ROS yaw readings.
    //
    // The sign flip is not a slip. ROS yaw is CCW-positive about +z, so a LEFT
    // turn raises it. This code's vehicle frame puts +x to the right, so its
    // path angles - atan2(dx, dy), see TrajectoryLatch::terminal_heading_rad -
    // are RIGHT-turn positive. The two conventions run opposite ways, and
    // getting this backwards would rotate stale geometry twice as far wrong
    // instead of correcting it.
    static double frame_delta_from_ros_yaw(double ros_yaw_new, double ros_yaw_old) {
        return -wrap_pi(ros_yaw_new - ros_yaw_old);
    }

    // Re-express a path captured at heading psi_old into the frame where the
    // heading is psi_new, given delta = frame_delta_from_ros_yaw(new, old).
    //
    // Same convention as TrajectoryLatch::re_express: rotating by -delta maps a
    // point one unit ahead along the old heading, (sin d, cos d), onto (0, 1).
    //
    // Rotation only, no translation. The callers that carry a path across
    // frames already re-anchor its arc-length origin - project_point_to_path in
    // the normalizer, plan_transition in the dropout replay - and that is the
    // forward component. What none of them correct is heading, and heading is
    // the term that dominates: 15 degrees of yaw moves a point 1000mm ahead by
    // ~260mm sideways, while a quarter second of travel at this vehicle's speed
    // moves it ~100mm almost entirely along the path.
    static std::vector<Point2D> rotate_into_frame(const std::vector<Point2D>& pts,
                                                  double delta_yaw_rad) {
        std::vector<Point2D> out;
        out.reserve(pts.size());
        double c = std::cos(delta_yaw_rad);
        double s = std::sin(delta_yaw_rad);
        for (const auto& p : pts) {
            out.push_back({p.x * c - p.y * s, p.x * s + p.y * c});
        }
        return out;
    }
};
