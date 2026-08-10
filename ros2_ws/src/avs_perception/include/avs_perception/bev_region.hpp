#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace avs_perception {

// Plan E1: valid BEV region for the pixel->world backprojection in
// ipm_transform_node. Pixels at or above the homography horizon back-project
// to +/- hundreds of meters (or behind the vehicle), so the projection step
// clips every polygon against this region before it can poison waypoint
// extraction and lane selection downstream.
//
// Clipping interpolates polygon edges at the region boundary
// (Sutherland-Hodgman) instead of dropping vertices: fixture polygons are
// sparse corner lists, and dropping the two above-horizon corners of a quad
// would degenerate the whole (mostly valid) lane to a 2-point sliver.
//
// Pure logic, no ROS dependency: tested directly by gtest
// (test/decision_trajectory_test.cpp, BevRegion.*) per the Plan D3 policy.
struct BevPoint {
    double x;
    double y;
};

struct BevRegion {
    double horizon_margin_px = 10.0;  // keep pixels at least this far below the horizon
    double y_min_mm = 0.0;            // no world points behind the vehicle origin
    double y_max_mm = 8000.0;         // practical forward lookahead range
    double x_abs_max_mm = 4000.0;     // lateral half-width of the useful BEV

    // Horizon row v_h(u): where the homography denominator
    // w = h20*u + h21*v + h22 crosses zero for image column u. Ground pixels
    // sit below it (larger v) for the usual upright forward-facing camera.
    // Returns a very small value when there is no finite horizon row (h21 ~ 0),
    // so every v passes the pixel gate and only the world gate applies.
    static double horizon_v(const double H[3][3], double u) {
        if (std::abs(H[2][1]) < 1e-12) return -1e18;
        return -(H[2][0] * u + H[2][2]) / H[2][1];
    }

    bool accepts_pixel(const double H[3][3], double u, double v) const {
        return v >= horizon_v(H, u) + horizon_margin_px;
    }

    bool accepts_world(double X, double Y) const {
        return Y >= y_min_mm && Y <= y_max_mm && std::abs(X) <= x_abs_max_mm;
    }

    // Clip a pixel-space polygon/polyline against the BEV region and project
    // it to world coordinates (mm). Steps:
    //   1. clip against the horizon line in pixel space (the homography is
    //      only a valid ground-plane mapping below it),
    //   2. project the surviving polygon through H (all points now finite and
    //      in front of the vehicle),
    //   3. clip against the world-space box [y_min, y_max] x [-x_max, x_max].
    // The homography maps straight edges to straight edges, so edge
    // interpolation before projection is geometrically exact.
    // Inputs with >= 3 points are treated as closed polygons; 2-point inputs
    // as open polylines (marking segments).
    std::vector<BevPoint> clip_and_project(const double H[3][3],
                                           const std::vector<BevPoint>& pixel_pts) const {
        std::vector<BevPoint> pts = pixel_pts;
        const bool closed = pts.size() >= 3;

        // 1. Pixel space: inside means v >= v_h(u) + margin, i.e.
        //    (h20/h21)*u + v + h22/h21 - margin >= 0 (only when a finite
        //    horizon row exists).
        if (std::abs(H[2][1]) >= 1e-12) {
            const double a = H[2][0] / H[2][1];
            const double c = H[2][2] / H[2][1] - horizon_margin_px;
            pts = clip_halfplane(pts, a, 1.0, c, closed);
        }

        // 2. Project through H. The horizon clip guarantees |w| is bounded
        //    away from zero, but keep the legacy denominator guard as a last
        //    line of defense.
        std::vector<BevPoint> world;
        world.reserve(pts.size());
        for (const auto& p : pts) {
            double w = H[2][0] * p.x + H[2][1] * p.y + H[2][2];
            if (std::abs(w) <= 1e-6) continue;
            world.push_back({(H[0][0] * p.x + H[0][1] * p.y + H[0][2]) / w,
                             (H[1][0] * p.x + H[1][1] * p.y + H[1][2]) / w});
        }

        // 3. World space box.
        world = clip_halfplane(world, 0.0, 1.0, -y_min_mm, closed);   // Y >= y_min
        world = clip_halfplane(world, 0.0, -1.0, y_max_mm, closed);   // Y <= y_max
        world = clip_halfplane(world, -1.0, 0.0, x_abs_max_mm, closed);  // X <= x_max
        world = clip_halfplane(world, 1.0, 0.0, x_abs_max_mm, closed);   // X >= -x_max
        return world;
    }

private:
    // Sutherland-Hodgman clip against the half-plane a*x + b*y + c >= 0.
    // closed=true wraps the last->first edge (polygon); closed=false clips a
    // polyline segment chain.
    static std::vector<BevPoint> clip_halfplane(const std::vector<BevPoint>& pts,
                                                double a, double b, double c,
                                                bool closed) {
        std::vector<BevPoint> out;
        if (pts.empty()) return out;
        const size_t n = pts.size();
        const size_t edges = closed ? n : n - 1;
        auto f = [&](const BevPoint& p) { return a * p.x + b * p.y + c; };
        for (size_t i = 0; i < edges; ++i) {
            const BevPoint& cur = pts[i];
            const BevPoint& nxt = pts[(i + 1) % n];
            const double fc = f(cur);
            const double fn = f(nxt);
            if (fc >= 0.0) out.push_back(cur);
            if ((fc >= 0.0) != (fn >= 0.0)) {
                const double t = fc / (fc - fn);
                out.push_back({cur.x + t * (nxt.x - cur.x), cur.y + t * (nxt.y - cur.y)});
            }
        }
        if (!closed && n >= 1 && f(pts.back()) >= 0.0) out.push_back(pts.back());
        return out;
    }
};

// Plan E1: safety-net cap on published waypoints per object. The clipped
// y/x range already bounds the count well below this in normal operation;
// hitting the cap indicates upstream garbage and is logged by the node.
inline constexpr std::size_t kMaxWaypointsPerObject = 128;

}  // namespace avs_perception
