// Sweeps the turn connector's shape knobs (TrajectoryPlanner::
// turn_lateral_bulge_mult / turn_bezier_handle_scale_mult) over the REAL
// planner header - no reimplementation - and reports the resulting path shape
// so a value can be chosen from measurements instead of by eye.
//
// Build/run: see README.md in this directory.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "avs_perception/label_mapping.hpp"
#include "avs_perception/decision_types.hpp"
#include "avs_perception/path_observation.hpp"
#include "avs_perception/trajectory_planner.hpp"

namespace {

// Live-captured right-turn frame (the same geometry the gtest regression
// guards use): ego main lane straight ahead, turn lane entering from the
// right at ~53 degrees.
const char* kDefaultTelemetry = R"({
  "timestamp_ms": 1000,
  "objects": [
    {"id": "main_lane_1114", "label": 6, "class_name": "main-lane", "confidence": 1.0,
     "waypoints": [[0.0, 100.0], [-17.0, 200.0]]},
    {"id": "turn_lane_1209", "label": 20, "class_name": "turn-lane", "confidence": 1.0,
     "waypoints": [[400.0, 843.0], [500.0, 918.0]]}
  ]
})";

struct Shape {
    bool valid = false;
    double belly_mm = 0.0;      // deepest point on the outside of the turn
    double inside_mm = 0.0;     // deepest point on the inside corner (want ~0)
    double outward_mm = 0.0;    // how far the path swings opposite the turn
    double min_radius_mm = 0.0; // tightest bend = peak steering demand
    double length_mm = 0.0;
    bool folds = false;         // path stops making forward progress
};

// Menger curvature of three consecutive points; radius = 1/curvature.
double radius_of(const Point2D& a, const Point2D& b, const Point2D& c) {
    double ab = std::hypot(b.x - a.x, b.y - a.y);
    double bc = std::hypot(c.x - b.x, c.y - b.y);
    double ca = std::hypot(a.x - c.x, a.y - c.y);
    double area2 = std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    if (area2 < 1e-9) return std::numeric_limits<double>::infinity();
    return (ab * bc * ca) / (2.0 * area2);
}

Shape measure(const std::vector<Point2D>& pts, bool turning_right) {
    Shape s;
    if (pts.size() < 3) return s;
    s.valid = true;

    const Point2D& a = pts.front();
    const Point2D& b = pts.back();
    double chord_len = std::hypot(b.x - a.x, b.y - a.y);
    if (chord_len < 1e-3) return s;

    // Signed offset from the chord, normalised so positive = outside of the
    // turn (the side the belly belongs on) for either turn direction.
    double outside_sign = turning_right ? 1.0 : -1.0;
    s.min_radius_mm = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < pts.size(); ++i) {
        const Point2D& p = pts[i];
        double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        double off = outside_sign * cross / chord_len;
        s.belly_mm = std::max(s.belly_mm, off);
        s.inside_mm = std::min(s.inside_mm, off);
        // Lateral excursion past the vehicle, on the side away from the turn.
        s.outward_mm = std::max(s.outward_mm, turning_right ? -p.x : p.x);
        if (i > 0) {
            s.length_mm += std::hypot(p.x - pts[i - 1].x, p.y - pts[i - 1].y);
            if (p.y < pts[i - 1].y - 1e-6) s.folds = true;
        }
        if (i + 2 < pts.size()) {
            s.min_radius_mm = std::min(s.min_radius_mm, radius_of(pts[i], pts[i + 1], pts[i + 2]));
        }
    }
    return s;
}

std::vector<Point2D> plan_with(const json& telemetry, RouteIntent intent,
                               double handle_mult, double bulge_mult) {
    TrajectoryPlanner::turn_bezier_handle_scale_mult = handle_mult;
    TrajectoryPlanner::turn_lateral_bulge_mult = bulge_mult;

    PathObservationFrame obs = PathObservationBuilder::build(telemetry);
    CommittedTrajectoryState prev_state;
    std::string last_main_id;
    PlannedTrajectory planned = TrajectoryPlanner::plan_candidate_for_intent(
        obs, intent, prev_state, /*is_t=*/false, /*t_junction_pending=*/false, last_main_id);
    return planned.valid ? planned.points : std::vector<Point2D>{};
}

// Rough BEV sketch, same orientation as the dashboard: +x right, +y up,
// vehicle at the bottom marked '^'.
void plot(const std::vector<Point2D>& pts) {
    const int rows = 24, cols = 61;
    double min_x = 0.0, max_x = 0.0, max_y = 0.0;
    for (const auto& p : pts) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
    }
    double pad = std::max(100.0, 0.1 * (max_x - min_x));
    min_x -= pad;
    max_x += pad;
    if (max_y < 1.0 || max_x - min_x < 1.0) return;

    std::vector<std::string> grid(rows, std::string(cols, ' '));
    auto put = [&](double x, double y, char c) {
        int col = static_cast<int>((x - min_x) / (max_x - min_x) * (cols - 1));
        int row = rows - 1 - static_cast<int>(y / max_y * (rows - 1));
        if (col >= 0 && col < cols && row >= 0 && row < rows) grid[row][col] = c;
    };
    for (const auto& p : pts) put(p.x, p.y, 'o');
    put(pts.front().x, pts.front().y, '^');
    put(pts.back().x, pts.back().y, '*');
    // Chord, for reference: the belly is the gap between '.' and 'o'.
    for (int i = 0; i <= 40; ++i) {
        double t = i / 40.0;
        double x = pts.front().x + t * (pts.back().x - pts.front().x);
        double y = pts.front().y + t * (pts.back().y - pts.front().y);
        int col = static_cast<int>((x - min_x) / (max_x - min_x) * (cols - 1));
        int row = rows - 1 - static_cast<int>(y / max_y * (rows - 1));
        if (col >= 0 && col < cols && row >= 0 && row < rows && grid[row][col] == ' ') {
            grid[row][col] = '.';
        }
    }
    std::cout << "\n  (^ = vehicle, o = path, . = straight chord, * = end)\n";
    std::cout << "  x range " << std::fixed << std::setprecision(0) << min_x << " .. " << max_x
              << " mm, y up to " << max_y << " mm\n\n";
    for (const auto& row : grid) std::cout << "  |" << row << "|\n";
    std::cout << "\n";
}

double arg_double(int argc, char** argv, const std::string& flag, double fallback) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (flag == argv[i]) return std::atof(argv[i + 1]);
    }
    return fallback;
}

std::string arg_string(int argc, char** argv, const std::string& flag, const std::string& fallback) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (flag == argv[i]) return argv[i + 1];
    }
    return fallback;
}

}  // namespace

int main(int argc, char** argv) {
    std::string telemetry_path = arg_string(argc, argv, "--telemetry", "");
    std::string intent_name = arg_string(argc, argv, "--intent", "right");
    double from = arg_double(argc, argv, "--from", 0.0);
    double to = arg_double(argc, argv, "--to", 0.8);
    double step = arg_double(argc, argv, "--step", 0.05);
    double handle = arg_double(argc, argv, "--handle", TrajectoryPlanner::turn_bezier_handle_scale_mult);
    double plot_at = arg_double(argc, argv, "--plot", -1.0);
    double current = TrajectoryPlanner::turn_lateral_bulge_mult;

    json telemetry;
    if (telemetry_path.empty()) {
        telemetry = json::parse(kDefaultTelemetry);
    } else {
        std::ifstream in(telemetry_path);
        if (!in) {
            std::cerr << "cannot open " << telemetry_path << "\n";
            return 1;
        }
        telemetry = json::parse(in);
    }

    bool turning_right = (intent_name != "left");
    RouteIntent intent = turning_right ? RouteIntent::TURN_RIGHT : RouteIntent::TURN_LEFT;

    if (plot_at >= 0.0) {
        std::vector<Point2D> pts = plan_with(telemetry, intent, handle, plot_at);
        std::cout << "bulge=" << std::fixed << std::setprecision(2) << plot_at
                  << "  handle=" << handle << "  points=" << pts.size() << "\n";
        if (pts.empty()) {
            std::cerr << "planner returned no path for this geometry\n";
            return 1;
        }
        plot(pts);
        return 0;
    }

    std::cout << "geometry: " << (telemetry_path.empty() ? "built-in right-turn capture" : telemetry_path)
              << "   intent: " << (turning_right ? "TURN_RIGHT" : "TURN_LEFT")
              << "   handle_scale_mult: " << std::fixed << std::setprecision(2) << handle << "\n\n";
    std::cout << "  bulge   belly_mm  inside_mm  outward_mm  min_radius_mm  length_mm  folds\n";
    std::cout << "  -----   --------  ---------  ----------  -------------  ---------  -----\n";

    for (double bulge = from; bulge <= to + 1e-9; bulge += step) {
        std::vector<Point2D> pts = plan_with(telemetry, intent, handle, bulge);
        Shape s = measure(pts, turning_right);
        std::cout << "  " << std::fixed << std::setprecision(2) << std::setw(5) << bulge;
        if (!s.valid) {
            std::cout << "   (planner returned no usable path)\n";
            continue;
        }
        std::cout << std::setprecision(0)
                  << std::setw(11) << s.belly_mm
                  << std::setw(11) << s.inside_mm
                  << std::setw(12) << s.outward_mm
                  << std::setw(15) << (std::isinf(s.min_radius_mm) ? 0.0 : s.min_radius_mm)
                  << std::setw(11) << s.length_mm
                  << std::setw(7) << (s.folds ? "FOLDS" : "-")
                  << (std::abs(bulge - current) < 1e-9 ? "   <- current default" : "")
                  << "\n";
    }

    std::cout << "\nbelly_mm    depth of the curve's belly on the OUTSIDE of the turn\n"
                 "inside_mm   deepest excursion onto the inside corner (want ~0)\n"
                 "outward_mm  how far the path swings past the vehicle away from the turn\n"
                 "min_radius  tightest bend on the path = peak steering demand (bigger = gentler)\n"
                 "FOLDS       path stops making forward progress - unusable, never ship this value\n";
    return 0;
}
