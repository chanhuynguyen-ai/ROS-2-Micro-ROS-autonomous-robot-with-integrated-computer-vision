#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "avs_perception/decision_types.hpp"

// Frozen turn execution.
//
// A committed turn loses its turn-lane detection exactly when the vehicle
// enters the junction: from inside the intersection the painted turn-lane is
// behind/beside the camera, so every fresh candidate is a follow_main path
// leading across to the far side. Replanning onto it drives the vehicle
// straight through the turn - so instead the last good turn path is latched
// and replayed open-loop until it is consumed.
//
// The latched path stays exactly as captured, in the vehicle frame of the latch
// frame. Each subsequent frame the vehicle is taken to have travelled
// progress_mm along it *and to be tracking it perfectly*, so its pose follows
// from the path geometry alone - heading is the path tangent at progress_mm -
// and no yaw sensor is involved. The flip side is that real lateral tracking
// error is invisible here by construction: re_express always reports the
// vehicle as sitting on the path, so the controller can only follow curvature,
// never correct accumulated drift. That is acceptable for a 1-3 s turn; a
// longer open-loop stretch would need real yaw.
//
// Point2D is (x = lateral, y = forward), matching decision_types.hpp.
class TrajectoryLatch {
public:
    static double path_length(const std::vector<Point2D>& pts) {
        double len = 0.0;
        for (size_t i = 1; i < pts.size(); ++i) {
            len += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
        }
        return len;
    }

    static double wrap_pi(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // Heading of the final segment. The path is expressed in the vehicle frame,
    // where the vehicle heading is 0 by construction, so under the perfect-
    // tracking assumption above this is exactly how far the vehicle will have
    // rotated by the time it reaches the end of the path.
    static double terminal_heading_rad(const std::vector<Point2D>& pts) {
        if (pts.size() < 2) return 0.0;
        size_t i = pts.size() - 1;
        return std::atan2(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    }

    // Tangent heading at arc-length progress_mm along the path. The path is
    // captured in the vehicle frame of the latch frame, where the vehicle
    // heading is 0, so under the perfect-tracking assumption re_express makes
    // this is exactly how far the vehicle has rotated since the latch closed.
    static double heading_at(const std::vector<Point2D>& pts, double progress_mm) {
        if (pts.size() < 2) return 0.0;
        double cum = 0.0;
        for (size_t i = 1; i < pts.size(); ++i) {
            double seg_len = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
            if (seg_len <= 1e-6) continue;
            if (cum + seg_len >= progress_mm) {
                return std::atan2(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
            }
            cum += seg_len;
        }
        return terminal_heading_rad(pts);
    }

    // Release predicate: the turn is far enough along AND there is a road ahead
    // to hand it over to. Both gates are required because each one alone fails
    // at opposite ends of the turn.
    //
    //  - Lane alignment alone fails at the *start*. The latch closes the moment
    //    the turn-lane leaves view, which is exactly when the old main lane
    //    running straight through the junction is still in frame and perfectly
    //    aligned with the vehicle - zero degrees off. Releasing on that
    //    abandons the turn before it begins.
    //  - Turn progress alone fails at the *end*. Reaching the target heading
    //    says nothing about whether perception has actually acquired the new
    //    road, and releasing into an empty frame is what leaves the vehicle
    //    with no path at all.
    //
    // The two cover for each other over time: early on heading_turned is small
    // so the span gate holds the latch shut, and by the time it opens the old
    // lane has swung ~90 degrees to the side and fails the alignment gate. Only
    // a genuinely new, aligned road satisfies both.
    //
    // This can only ever release *earlier* than the distance/deadline
    // conditions in the caller, never later - so an odometry scale error, which
    // this shares with those conditions through progress_mm, cannot introduce a
    // failure mode that was not already there.
    static bool turn_complete(double heading_turned_rad, double target_rad,
                              double min_span_frac, bool lane_present,
                              double lane_heading_rad, double max_lane_heading_rad) {
        if (!lane_present) return false;
        if (std::abs(target_rad) < 1e-6) return false;
        if (std::abs(heading_turned_rad) < min_span_frac * std::abs(target_rad)) return false;
        return std::abs(lane_heading_rad) <= max_lane_heading_rad;
    }

    // Least-squares circle through pts[from..to] (Kasa): minimises the algebraic
    // residual x^2 + y^2 - 2ax - 2by - c over *every* sample in the window.
    //
    // This replaced a circumcircle through three picked points. Three points fix
    // a circle exactly, so they cannot disagree with it - the fit absorbs all
    // their noise into the answer instead of averaging it away, and the endpoint,
    // where the tangent is then read, is the worst-placed of the three. Measured
    // on the vehicle 2026-08-05 that produced tangents 38 degrees off the path's
    // own direction, and on a gently-curved observation a radius of 3264mm which
    // extrapolated into 4.7 metres of fabricated arc.
    //
    // Returns false when the samples are collinear (no arc to continue) or the
    // system is singular. Populates centre/radius otherwise.
    static bool fit_circle_ls(const std::vector<Point2D>& pts, size_t from, size_t to,
                              Point2D& centre, double& radius) {
        if (to <= from || to >= pts.size()) return false;
        size_t count = to - from + 1;
        if (count < 3) return false;

        double Sx = 0, Sy = 0, Sxx = 0, Syy = 0, Sxy = 0, Sz = 0, Sxz = 0, Syz = 0;
        for (size_t i = from; i <= to; ++i) {
            double x = pts[i].x, y = pts[i].y, z = x * x + y * y;
            Sx += x; Sy += y; Sxx += x * x; Syy += y * y; Sxy += x * y;
            Sz += z; Sxz += x * z; Syz += y * z;
        }
        // [2Sxx 2Sxy Sx][a]   [Sxz]
        // [2Sxy 2Syy Sy][b] = [Syz]
        // [2Sx  2Sy  N ][c]   [Sz ]
        double m[3][4] = {{2 * Sxx, 2 * Sxy, Sx, Sxz},
                          {2 * Sxy, 2 * Syy, Sy, Syz},
                          {2 * Sx,  2 * Sy,  static_cast<double>(count), Sz}};
        for (int col = 0; col < 3; ++col) {
            int piv = col;
            for (int r = col + 1; r < 3; ++r) {
                if (std::abs(m[r][col]) > std::abs(m[piv][col])) piv = r;
            }
            if (std::abs(m[piv][col]) < 1e-9) return false;
            if (piv != col) {
                for (int k = 0; k < 4; ++k) std::swap(m[col][k], m[piv][k]);
            }
            for (int r = 0; r < 3; ++r) {
                if (r == col) continue;
                double f = m[r][col] / m[col][col];
                for (int k = col; k < 4; ++k) m[r][k] -= f * m[col][k];
            }
        }
        double a = m[0][3] / m[0][0];
        double b = m[1][3] / m[1][1];
        double cc = m[2][3] / m[2][2];
        double r2 = cc + a * a + b * b;
        if (!(r2 > 0.0)) return false;
        centre = {a, b};
        radius = std::sqrt(r2);
        return true;
    }

    // Cut back a turn observation to its furthest-turned point.
    //
    // The far end of an IPM-derived path is its least reliable part, and on this
    // vehicle it reliably bends back. Logged left turns (2026-08-05), segment
    // headings in degrees:
    //
    //     ... -42 -52 -63 -68 -76 -72 -66      peak -76, unwinds 10 over 200mm
    //     ... -64 -77 -83 -88 -91 -84 -75      peak -91, unwinds 16
    //     ... -43 -55 -69 -78 -75 -67 -55      peak -78, unwinds 23
    //     ... -33 -47 -62 -71 -72              no unwind
    //
    // terminal_heading_rad reads the last chord, so the observed span and the
    // tangent the extension continues from were both being taken from the part
    // that had already turned back - understating the turn by 10-23 degrees and
    // dragging the fitted tangent off the path. Of the seven turns logged, the
    // only one that got extended was the fourth: the only one whose tail did not
    // unwind.
    //
    // Trimming to the furthest-turned segment keeps the stretch where the
    // observation still agreed with itself. Left alone when the unwind is within
    // sampling noise, or when trimming would leave too little to fit.
    static std::vector<Point2D> trim_flared_tip(std::vector<Point2D> pts,
                                                double target_rad) {
        if (pts.size() < 5 || std::abs(target_rad) < 1e-6) return pts;
        const double s = target_rad < 0.0 ? -1.0 : 1.0;

        auto seg_heading = [&](size_t i) {
            return s * std::atan2(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
        };

        size_t peak = 1;
        double peak_h = seg_heading(1);
        for (size_t i = 2; i < pts.size(); ++i) {
            double h = seg_heading(i);
            if (h > peak_h) { peak_h = h; peak = i; }
        }

        constexpr double kReversalTolRad = 5.0 * M_PI / 180.0;
        constexpr size_t kMinKeptSegments = 3;
        if (peak_h - seg_heading(pts.size() - 1) <= kReversalTolRad) return pts;
        if (peak < kMinKeptSegments) return pts;

        pts.resize(peak + 1);
        return pts;
    }

    // Grow a latched turn until it actually completes the turn.
    //
    // The painted turn-lane runs from the stop line all the way to the merge
    // point, but the camera only ever delivers its near end: at a 1500mm-radius
    // junction a typical observation carries 40-60 degrees of the 90. Replaying
    // such a path leaves the vehicle pointing half-way into the junction, where
    // the new road is still too oblique to be labelled main-lane - so perception
    // never recovers and the turn is never finished.
    //
    // The marking is a constant-radius arc, so the honest completion is to keep
    // the same radius until the heading reaches target_rad, then run out straight.
    // The straight run-out matters as much as the arc: pure pursuit clamps to the
    // final point once the remaining path is shorter than the lookahead, which
    // flattens the command over the last stretch. Ending straight puts that
    // flattened stretch where it costs nothing, and gives perception a settled,
    // aligned view of the new road before the latch releases.
    //
    // target_rad is signed (+ right, - left). The path is returned untouched
    // rather than extrapolated when it does not describe a turn in that
    // direction - a straight path must never be bent into a fabricated turn.
    static std::vector<Point2D> extend_to_turn_angle(std::vector<Point2D> pts,
                                                     double target_rad,
                                                     double runout_mm,
                                                     double min_radius_mm,
                                                     double max_radius_mm,
                                                     double min_observed_span_rad) {
        if (pts.size() < 3 || std::abs(target_rad) < 1e-6) return pts;

        // Everything below reads the path's direction from its end, so the
        // unreliable flared tip has to go first - see trim_flared_tip.
        pts = trim_flared_tip(std::move(pts), target_rad);
        if (pts.size() < 3) return pts;

        double psi = terminal_heading_rad(pts);
        if (psi * target_rad < 0.0) return pts;                    // curves against the intent
        if (std::abs(psi) < min_observed_span_rad) return pts;     // not actually a turn

        // Fit the circle the marking lies on, through three points spread over
        // its tail - the part of the arc the extension continues. A circle fit is
        // worth the arithmetic over chord bookkeeping: chord headings lag the true
        // tangent by half a sample and the arc length between two chord midpoints
        // spans one segment fewer than it looks, biases that compound into a
        // double-digit radius error and put the exit metres off the new lane.
        // Walk back over the tail, but stop at the point where the path was still
        // heading the *other* way.
        //
        // A turn path is not one arc. Measured on the vehicle 2026-08-05, the
        // approach to a left junction swings out to the RIGHT into the middle of
        // the intersection first, and only then arcs left - an S, not a circle.
        // Meanwhile these paths run 650-1000mm end to end, so a flat 800mm window
        // swallowed nearly all of them: the fit windows actually selected were
        // pts[0..10], pts[0..12], pts[2..14], pts[3..16]. Fitting one circle
        // across the whole S describes neither half, and put the endpoint tangent
        // 27-44 degrees off the path's own direction - which is what the sanity
        // check below then (correctly) rejected, on five of seven left turns.
        //
        // Only the final same-signed stretch is the junction arc the extension is
        // meant to continue, so only that is fitted. The tolerance keeps segments
        // that are merely near-straight: they carry no turn either way and
        // dropping them would shorten the window for nothing.
        constexpr double kTailWindowMm = 800.0;
        constexpr double kOppositeToleranceRad = 10.0 * M_PI / 180.0;
        size_t n = pts.size() - 1;
        size_t j = n;
        double tail_len = 0.0;
        while (j > 0 && tail_len < kTailWindowMm) {
            double seg = std::atan2(pts[j].x - pts[j - 1].x, pts[j].y - pts[j - 1].y);
            // seg * target > 0 means this segment already points into the turn.
            if (seg * target_rad < -kOppositeToleranceRad * std::abs(target_rad)) break;
            tail_len += std::hypot(pts[j].x - pts[j - 1].x, pts[j].y - pts[j - 1].y);
            --j;
        }
        if (n - j < 3) return pts;  // too few samples to fit a circle

        const Point2D& c = pts[n];
        Point2D centre;
        double radius = 0.0;
        if (!fit_circle_ls(pts, j, n, centre, radius)) return pts;
        if (radius < 1e-6) return pts;

        // Does this tail actually look like an arc? Ask the data, before the
        // clamp below moves the circle off it. Extending is only defensible when
        // the shape being continued is the shape that was observed; a tail that
        // is not circular gets handed back untouched, because a turn that stops
        // where perception stopped is honest and a turn bent onto a fabricated
        // arc is not.
        //
        // This replaces a guard that compared the fitted tangent against psi and
        // rejected disagreements past 15 degrees. That comparison could not be
        // satisfied: psi is a chord angle over a finite window, the tangent is
        // taken at the endpoint, and on an arc a chord's angle equals the tangent
        // at the middle of the span it subtends, not at its end. The gap is about
        // half the window's arc - on the vehicle's real radii of 310-613mm over a
        // ~200mm window, 15-25 degrees, which is a property of sampling a curve,
        // not evidence of a bad fit. Measured 2026-08-05 (run12): the guard threw
        // out 7 of 8 left turns at errors of 16.6-27.4 degrees while the one it
        // let through sat at 14.6, so the threshold ran straight through the
        // middle of the healthy distribution, and it bit left turns hardest
        // precisely because they curve tightest. Residual has no such bias: those
        // same 13 sound fits measure 2.8-12.7mm, and the one genuinely
        // non-circular tail in the set measures 64.5mm.
        constexpr double kMaxFitResidualMm = 30.0;
        double resid = 0.0;
        for (size_t i = j; i <= n; ++i) {
            double d = std::hypot(pts[i].x - centre.x, pts[i].y - centre.y) - radius;
            resid += d * d;
        }
        if (std::sqrt(resid / static_cast<double>(n - j + 1)) > kMaxFitResidualMm) return pts;
        // Clamping curvature must not move the path or kink its tangent, so pull
        // the centre along the same normal instead of leaving it where it was.
        double nx = (centre.x - c.x) / radius;
        double ny = (centre.y - c.y) / radius;
        radius = std::max(min_radius_mm, std::min(max_radius_mm, radius));
        centre = {c.x + nx * radius, c.y + ny * radius};

        // Angle of the end point about the centre, and the tangent there. The
        // tangent is a quarter turn off the radius; which way tells us the sense
        // of the arc, so take the branch that agrees with the direction of travel.
        double phi = std::atan2(c.x - centre.x, c.y - centre.y);
        double sgn = std::abs(wrap_pi(phi + M_PI_2 - psi)) <=
                             std::abs(wrap_pi(phi - M_PI_2 - psi))
                         ? 1.0
                         : -1.0;
        if (sgn * target_rad < 0.0) return pts;  // arc curves against the intent

        double heading = phi + sgn * M_PI_2;  // exact tangent, no chord lag

        // Three points fix a circle exactly, so noise on any one of them moves the
        // fitted tangent with nothing to average it out - and the endpoint, the
        // worst-conditioned of the three, is the one the tangent is taken at.
        // Measured on the vehicle 2026-08-05: a left turn whose path ran at -62.5
        // degrees fitted a tangent of -100.3. Past the target that way the arc loop
        // below exits immediately, but the run-out was still welded on at that
        // bogus heading, leaving a 38-degree kink 700mm from the end - directly
        // under the 600mm pure-pursuit lookahead for the whole second half of the
        // turn, which is what threw the vehicle off line on the way out.
        //
        // Chord lag alone is small (a 100mm step on a 800mm radius is 3.6 degrees
        // of it), so a fit that disagrees with the path's own final direction by
        // much more than that is not describing this path. Hand back the
        // observation untouched rather than extend it from a tangent we do not
        // trust: a turn that stops where perception stopped is honest, a turn bent
        // onto a fabricated heading is not.
        constexpr double kStepMm = 100.0;
        double goal = std::abs(target_rad);
        while (std::abs(heading) < goal - 1e-9) {
            double d = std::min(kStepMm / radius, goal - std::abs(heading));
            phi += sgn * d;
            heading += sgn * d;
            pts.push_back({centre.x + radius * std::sin(phi), centre.y + radius * std::cos(phi)});
        }

        // The fitted tangent can already sit a little past the target, in which
        // case the loop above added nothing. The run-out still has to leave along
        // the turn target rather than continue past it - it is the stretch that
        // hands the vehicle to the new road, so it must point down that road.
        if (std::abs(heading) > goal) heading = std::copysign(goal, heading);

        Point2D cur = pts.back();
        double hx = std::sin(heading);
        double hy = std::cos(heading);
        for (double s = 0.0; s + 1e-9 < runout_mm; s += kStepMm) {
            cur = {cur.x + kStepMm * hx, cur.y + kStepMm * hy};
            pts.push_back(cur);
        }
        return pts;
    }

    // The still-unconsumed part of the path, rotated and translated into the
    // vehicle frame implied by perfect tracking at arc-length progress_mm.
    //
    // The vehicle origin itself is deliberately not emitted:
    // LegacyLaneModel::evaluate_trajectory_at_lookahead prepends (0,0) to every
    // trajectory it measures, so paths here follow the same convention as the
    // planners and start at the first point *ahead* of the vehicle.
    //
    // Returns an empty vector once the path is consumed. A single point still
    // describes a followable path here - the prepended origin makes it a real
    // segment - so callers must not raise the bar to the >= 2 that
    // PlannedTrajectory::valid uses for observed paths, or they will release the
    // latch one point early and cut the tail off the turn.
    // heading_error_rad closes the loop on rotation: it is how much further the
    // vehicle has actually turned than this path assumes it has by progress_mm
    // (measured yaw minus the path tangent there), and it tilts the frame the
    // remaining path is emitted into. Zero reproduces the pure open-loop
    // behaviour. Lateral error stays invisible - only the heading is corrected -
    // but heading is the term that compounds, because a path emitted at the
    // wrong angle steers the vehicle further wrong, which tilts the next frame
    // further still. Measured 2026-08-05 (run13): with a frozen path carrying
    // exactly 90 degrees in every case, the vehicle came out of turns having
    // rotated 67, 83 and 214 degrees.
    static std::vector<Point2D> re_express(const std::vector<Point2D>& pts, double progress_mm,
                                           double heading_error_rad = 0.0) {
        std::vector<Point2D> out;
        if (pts.size() < 2) return out;

        // Locate the segment holding progress_mm.
        double cum = 0.0;
        size_t seg = 0;
        double seg_len = 0.0;
        bool found = false;
        for (size_t i = 1; i < pts.size(); ++i) {
            seg_len = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
            if (cum + seg_len >= progress_mm && seg_len > 1e-6) {
                seg = i;
                found = true;
                break;
            }
            cum += seg_len;
        }
        if (!found) return out;  // consumed past the end

        double ratio = std::max(0.0, std::min(1.0, (progress_mm - cum) / seg_len));
        double seg_dx = pts[seg].x - pts[seg - 1].x;
        double seg_dy = pts[seg].y - pts[seg - 1].y;
        Point2D origin{pts[seg - 1].x + ratio * seg_dx, pts[seg - 1].y + ratio * seg_dy};

        // Heading = path tangent at progress_mm. Rotating by -psi maps a point
        // one unit ahead along that tangent, (sin psi, cos psi), onto (0, 1).
        double psi = std::atan2(seg_dx, seg_dy) + heading_error_rad;
        double c = std::cos(psi);
        double s = std::sin(psi);

        out.reserve(pts.size() - seg);
        for (size_t i = seg; i < pts.size(); ++i) {
            double dx = pts[i].x - origin.x;
            double dy = pts[i].y - origin.y;
            // Drop points the vehicle is already standing on. progress_mm landing
            // exactly on a vertex (or a duplicated vertex) would otherwise emit a
            // point at the origin, which collides with the (0,0) the lookahead
            // evaluator prepends and leaves it a zero-length segment to take a
            // curvature over.
            if (std::hypot(dx, dy) < 1e-6) continue;
            out.push_back({dx * c - dy * s, dx * s + dy * c});
        }
        return out;
    }
};
