#pragma once

// Plan F: solid-yellow legality gate. A lane may only be planned on when it
// lies on the right of a longitudinal solid-yellow marking, or below a
// transverse one. Both cases collapse into one signed side test once the
// marking direction is oriented by convention (see orient_direction):
// legal <=> cross_z(d̂, p - p0) < 0 with d̂ the unit segment direction.
// Design and decisions: docs/plans/plan_F_solid_yellow_legality_gate.md.

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "avs_perception/decision_types.hpp"
#include "avs_perception/label_mapping.hpp"

using namespace avs_perception;

enum class LaneLegality { LEGAL, ILLEGAL, UNKNOWN };

static const char* lane_legality_name(LaneLegality v) {
    switch (v) {
        case LaneLegality::LEGAL: return "legal";
        case LaneLegality::ILLEGAL: return "illegal";
        case LaneLegality::UNKNOWN: return "unknown";
    }
    return "unknown";
}

// One normalized solid-yellow marking: points collapsed to a centerline along
// the PCA principal axis (sort-by-y from PathObservationBuilder zigzags on
// transverse markings, and polygon fallbacks carry the mask thickness), axis
// oriented by the beta-band convention.
struct YellowReference {
    std::string marking_id;
    std::vector<Point2D> polyline;
    Point2D dir{{0.0}, {1.0}};   // unit principal direction, oriented
    Point2D centroid{{0.0}, {0.0}};
    double proj_min = 0.0;       // projection span of the polyline onto dir
    double proj_max = 0.0;
    bool horizontal_band = false;
    // dashed-yellow: same divider geometry, but an ILLEGAL verdict it produces
    // is "soft" - an active lane_change intent may cross it (solid never).
    bool soft = false;
};

struct LaneLegalityReport {
    std::map<std::string, LaneLegality> lane_verdicts;
    // True when the lane's settled ILLEGAL verdict came from a dashed-yellow
    // (soft) reference; such lanes survive filtering while a lane_change
    // intent is active.
    std::map<std::string, bool> soft_illegal;
    std::map<std::string, double> signed_dist_mm;  // debug, only when a ref applied
    bool yellow_visible = false;   // fresh yellow (solid or dashed) in this frame
    int yellow_age_frames = 0;     // 0 when fresh, grows while running on memory
};

class LaneLegalityGate {
public:
    struct Params {
        bool enabled = true;
        double margin_mm = 100.0;             // dead-zone around the marking
        int yellow_hold_frames = 10;          // reuse a lost reference this long
        double beta_deg = 20.0;               // near-transverse band half-angle
        double beta_hysteresis_deg = 5.0;     // band flip hysteresis
        double min_yellow_length_mm = 300.0;
        double min_yellow_confidence = 0.30;  // reject misdetected solid-yellow
        double min_linearity_ratio = 4.0;     // PCA eigenvalue ratio guard
        double applicability_ext_mm = 500.0;  // span extension for applicability
        double centerline_bin_mm = 100.0;     // collapse thickness/zigzag to centerline
        int verdict_flip_frames = 2;          // per-lane hysteresis
        bool dashed_yellow_enabled = true;    // dashed-yellow as soft reference
    };

    LaneLegalityGate() : params_() {}
    explicit LaneLegalityGate(const Params& params) : params_(params) {}

    // ── Pure geometry helpers (unit-testable without gate state) ────────────

    static bool finite(const Point2D& p) {
        return std::isfinite(p.x) && std::isfinite(p.y);
    }

    // Extract marking points, falling back to polygons_real_world when the
    // builder produced fewer than 2 waypoints (matches the existing marking
    // gate which also considers polygon geometry). Non-finite points from
    // upstream never enter the geometry.
    static std::vector<Point2D> marking_points(const MarkingObservation& m) {
        std::vector<Point2D> pts;
        for (const auto& p : m.points) {
            if (finite(p)) pts.push_back(p);
        }
        if (pts.size() >= 2) return pts;
        pts.clear();
        if (m.raw_obj.contains("polygons_real_world") && m.raw_obj["polygons_real_world"].is_array()) {
            for (const auto& poly : m.raw_obj["polygons_real_world"]) {
                if (!poly.is_array()) continue;
                for (const auto& pt : poly) {
                    if (pt.is_array() && pt.size() >= 2 && pt[0].is_number() && pt[1].is_number()) {
                        Point2D p;
                        p.x = pt[0].get<double>();
                        p.y = pt[1].get<double>();
                        if (finite(p)) pts.push_back(p);
                    }
                }
            }
        }
        return pts;
    }

    // PCA principal axis. Returns false when the point set is too small or
    // too isotropic (L-shape/blob) to trust a direction.
    static bool principal_axis(const std::vector<Point2D>& pts, double min_linearity_ratio,
                               Point2D* centroid_out, Point2D* dir_out) {
        if (pts.size() < 2) return false;
        double mx = 0.0, my = 0.0;
        for (const auto& p : pts) { mx += p.x; my += p.y; }
        mx /= pts.size();
        my /= pts.size();
        double sxx = 0.0, sxy = 0.0, syy = 0.0;
        for (const auto& p : pts) {
            double dx = p.x - mx, dy = p.y - my;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }
        double tr = sxx + syy;
        double det = sxx * syy - sxy * sxy;
        double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));
        double l1 = tr / 2.0 + disc;
        double l2 = tr / 2.0 - disc;
        if (l1 <= 1e-9) return false;
        if (l2 > 1e-9 && l1 / l2 < min_linearity_ratio) return false;
        Point2D dir;
        if (std::abs(sxy) > 1e-9) {
            dir.x = l1 - syy;
            dir.y = sxy;
        } else {
            // Axis-aligned covariance: principal axis is the larger variance axis.
            dir.x = (sxx >= syy) ? 1.0 : 0.0;
            dir.y = (sxx >= syy) ? 0.0 : 1.0;
        }
        double n = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (n <= 1e-12) return false;
        dir.x /= n;
        dir.y /= n;
        if (centroid_out) { centroid_out->x = mx; centroid_out->y = my; }
        if (dir_out) *dir_out = dir;
        return true;
    }

    // Beta-band sign convention. Near-transverse markings point +X (legal side
    // below), everything else points +Y (legal side to the right). A plain
    // "flip when d.y < 0" would flip verdicts between a transverse marking
    // dipping slightly down vs slightly up - exactly the case the rule cares
    // about - hence the band. prev_horizontal adds hysteresis at the band edge.
    static bool orient_direction(Point2D* dir, double beta_deg, double beta_hysteresis_deg,
                                 const bool* prev_horizontal) {
        double angle_from_x_deg = std::atan2(std::abs(dir->y), std::abs(dir->x)) * 180.0 / M_PI;
        bool horizontal = angle_from_x_deg < beta_deg;
        if (prev_horizontal && std::abs(angle_from_x_deg - beta_deg) <= beta_hysteresis_deg) {
            horizontal = *prev_horizontal;
        }
        bool flip = horizontal ? (dir->x < 0.0) : (dir->y < 0.0);
        if (flip) {
            dir->x = -dir->x;
            dir->y = -dir->y;
        }
        return horizontal;
    }

    // Collapse points (already sorted by projection t) into a centerline by
    // averaging bins along the principal axis. Removes both the sort-by-y
    // zigzag of transverse waypoints and the cross-thickness segments a
    // polygon fallback would otherwise produce.
    static std::vector<Point2D> collapse_to_centerline(const std::vector<Point2D>& sorted_pts,
                                                       const Point2D& centroid, const Point2D& dir,
                                                       double bin_mm) {
        std::vector<Point2D> out;
        size_t i = 0;
        while (i < sorted_pts.size()) {
            double t0 = (sorted_pts[i].x - centroid.x) * dir.x + (sorted_pts[i].y - centroid.y) * dir.y;
            double sx = 0.0, sy = 0.0;
            size_t n = 0;
            while (i < sorted_pts.size()) {
                double t = (sorted_pts[i].x - centroid.x) * dir.x + (sorted_pts[i].y - centroid.y) * dir.y;
                if (t - t0 > bin_mm) break;
                sx += sorted_pts[i].x;
                sy += sorted_pts[i].y;
                n++;
                i++;
            }
            Point2D p;
            p.x = sx / n;
            p.y = sy / n;
            out.push_back(p);
        }
        return out;
    }

    static bool build_reference(const MarkingObservation& m, const Params& params,
                                const bool* prev_horizontal, YellowReference* out) {
        if (m.confidence < params.min_yellow_confidence) return false;
        std::vector<Point2D> pts = marking_points(m);
        if (pts.size() < 2) return false;

        Point2D centroid, dir;
        if (!principal_axis(pts, params.min_linearity_ratio, &centroid, &dir)) return false;
        bool horizontal = orient_direction(&dir, params.beta_deg, params.beta_hysteresis_deg, prev_horizontal);

        std::sort(pts.begin(), pts.end(), [&](const Point2D& a, const Point2D& b) {
            return (a.x - centroid.x) * dir.x + (a.y - centroid.y) * dir.y <
                   (b.x - centroid.x) * dir.x + (b.y - centroid.y) * dir.y;
        });
        pts = collapse_to_centerline(pts, centroid, dir, params.centerline_bin_mm);
        if (pts.size() < 2) return false;

        double proj_min = 1e18, proj_max = -1e18, length = 0.0;
        for (size_t i = 0; i < pts.size(); ++i) {
            double t = (pts[i].x - centroid.x) * dir.x + (pts[i].y - centroid.y) * dir.y;
            proj_min = std::min(proj_min, t);
            proj_max = std::max(proj_max, t);
            if (i > 0) length += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
        }
        if (length < params.min_yellow_length_mm) return false;

        out->marking_id = m.marking_id;
        out->polyline = std::move(pts);
        out->dir = dir;
        out->centroid = centroid;
        out->proj_min = proj_min;
        out->proj_max = proj_max;
        out->horizontal_band = horizontal;
        return true;
    }

    // Signed perpendicular distance (mm) from p to the nearest polyline
    // segment, sign from cross_z with the unit segment direction (oriented
    // along the principal axis). Negative = legal side (right/below).
    static double signed_distance_to_reference(const Point2D& p, const YellowReference& ref) {
        double best_abs = 1e18;
        double best_signed = 0.0;
        for (size_t i = 0; i + 1 < ref.polyline.size(); ++i) {
            Point2D a = ref.polyline[i];
            Point2D b = ref.polyline[i + 1];
            double sx = b.x - a.x, sy = b.y - a.y;
            double len = std::hypot(sx, sy);
            if (len <= 1e-9) continue;
            double ux = sx / len, uy = sy / len;
            // Keep every segment oriented with the principal axis so curvature
            // in the polyline cannot flip the sign locally.
            if (ux * ref.dir.x + uy * ref.dir.y < 0.0) { ux = -ux; uy = -uy; }
            double t = std::clamp((p.x - a.x) * ux + (p.y - a.y) * uy, 0.0, len);
            double cx = a.x + t * ux, cy = a.y + t * uy;
            double dist = std::hypot(p.x - cx, p.y - cy);
            if (dist < best_abs) {
                best_abs = dist;
                double cross = ux * (p.y - a.y) - uy * (p.x - a.x);
                best_signed = (cross < 0.0) ? -dist : dist;
            }
        }
        return best_signed;
    }

    // Raw (memory-free) verdict of one lane against a set of references.
    // Only lane points whose projection falls inside the reference span
    // (+ext) participate. With several applicable references the nearest one
    // rules. applicable_out distinguishes "no reference rules this lane"
    // (permissive UNKNOWN that must decay stale memory) from the dead-zone
    // UNKNOWN (|signed| <= margin, keeps the previous verdict).
    static LaneLegality classify_lane(const LaneObservation& lane,
                                      const std::vector<YellowReference>& refs,
                                      const Params& params,
                                      double* signed_out = nullptr,
                                      bool* applicable_out = nullptr,
                                      bool* soft_out = nullptr) {
        if (applicable_out) *applicable_out = false;
        if (soft_out) *soft_out = false;
        if (!params.enabled || refs.empty() || lane.points.empty()) return LaneLegality::UNKNOWN;

        double best_mean_abs = 1e18;
        double best_mean_signed = 0.0;
        bool best_soft = false;
        bool any_applicable = false;

        for (const auto& ref : refs) {
            double sum_signed = 0.0, sum_abs = 0.0;
            int n = 0;
            for (const auto& p : lane.points) {
                if (!finite(p)) continue;
                double t = (p.x - ref.centroid.x) * ref.dir.x + (p.y - ref.centroid.y) * ref.dir.y;
                if (t < ref.proj_min - params.applicability_ext_mm ||
                    t > ref.proj_max + params.applicability_ext_mm) {
                    continue;
                }
                double sd = signed_distance_to_reference(p, ref);
                sum_signed += sd;
                sum_abs += std::abs(sd);
                n++;
            }
            if (n == 0) continue;
            any_applicable = true;
            double mean_abs = sum_abs / n;
            if (mean_abs < best_mean_abs) {
                best_mean_abs = mean_abs;
                best_mean_signed = sum_signed / n;
                best_soft = ref.soft;
            }
        }

        if (!any_applicable) return LaneLegality::UNKNOWN;
        if (applicable_out) *applicable_out = true;
        if (signed_out) *signed_out = best_mean_signed;
        if (soft_out) *soft_out = best_soft;
        if (best_mean_signed < -params.margin_mm) return LaneLegality::LEGAL;
        if (best_mean_signed > params.margin_mm) return LaneLegality::ILLEGAL;
        return LaneLegality::UNKNOWN;
    }

    // ── Stateful per-frame evaluation (yellow memory + per-lane hysteresis) ──

    LaneLegalityReport evaluate(const PathObservationFrame& frame) {
        LaneLegalityReport report;
        if (!params_.enabled) return report;

        // Per-reference hold: a fresh detection refreshes its own entry; a
        // reference missing this frame ages individually and survives up to
        // yellow_hold_frames, so one occluded marking does not dump the other
        // (partial-dropout case at intersections).
        std::map<std::string, HeldRef> next_held;
        bool any_fresh = false;
        for (const auto& m : frame.markings) {
            bool is_solid_yellow = (m.label == LABEL_SOLID_YELLOW);
            bool is_dashed_yellow = (m.label == LABEL_DASHED_YELLOW) && params_.dashed_yellow_enabled;
            if (!is_solid_yellow && !is_dashed_yellow) continue;
            auto held_it = held_.find(m.marking_id);
            const bool* prev_band = (held_it != held_.end()) ? &held_it->second.ref.horizontal_band : nullptr;
            YellowReference ref;
            if (build_reference(m, params_, prev_band, &ref)) {
                ref.soft = is_dashed_yellow;
                HeldRef h;
                h.ref = std::move(ref);
                h.age = 0;
                next_held[m.marking_id] = std::move(h);
                any_fresh = true;
            }
        }
        for (auto& [id, h] : held_) {
            if (next_held.count(id)) continue;
            if (h.age + 1 > params_.yellow_hold_frames) continue;
            HeldRef aged = h;
            aged.age++;
            next_held[id] = std::move(aged);
        }
        held_ = std::move(next_held);

        yellow_age_frames_ = any_fresh ? 0 : yellow_age_frames_ + 1;
        report.yellow_visible = any_fresh;
        report.yellow_age_frames = yellow_age_frames_;

        // No reference at all (never seen, or everything aged out): permissive
        // UNKNOWN and drop per-lane memory so stale verdicts cannot filter
        // lanes forever.
        if (held_.empty()) {
            verdict_memory_.clear();
            for (const auto& lane : frame.lanes) {
                report.lane_verdicts[lane.lane_id] = LaneLegality::UNKNOWN;
            }
            return report;
        }

        std::vector<YellowReference> refs;
        refs.reserve(held_.size());
        for (const auto& [id, h] : held_) refs.push_back(h.ref);

        std::map<std::string, VerdictMemory> next_memory;
        for (const auto& lane : frame.lanes) {
            double signed_dist = 0.0;
            bool applicable = false;
            bool raw_soft = false;
            LaneLegality raw = classify_lane(lane, refs, params_, &signed_dist, &applicable, &raw_soft);

            bool first_seen = verdict_memory_.find(lane.lane_id) == verdict_memory_.end();
            VerdictMemory mem;
            if (!first_seen) mem = verdict_memory_[lane.lane_id];

            if (applicable && raw == LaneLegality::UNKNOWN) {
                // Dead-zone: the lane hugs the marking - keep the settled belief.
                mem.pending = mem.settled;
                mem.pending_soft = mem.soft;
                mem.streak = 0;
            } else {
                // LEGAL/ILLEGAL, or not-applicable (raw UNKNOWN) which must be
                // allowed to settle so a lane that left the marking's span
                // decays to permissive instead of staying filtered on memory.
                if (raw == mem.settled) {
                    mem.pending = mem.settled;
                    // Same verdict, but the ruling reference (and thus its
                    // soft/hard nature) may have changed - track the fresh one.
                    mem.soft = raw_soft;
                    mem.pending_soft = raw_soft;
                    mem.streak = 0;
                } else if (raw == mem.pending && !first_seen) {
                    mem.streak++;
                    mem.pending_soft = raw_soft;
                    if (mem.streak >= params_.verdict_flip_frames) {
                        mem.settled = raw;
                        mem.soft = raw_soft;
                        mem.streak = 0;
                    }
                } else {
                    mem.pending = raw;
                    mem.pending_soft = raw_soft;
                    mem.streak = 1;
                    if (first_seen) {
                        // A first-ever verdict settles immediately: there is no
                        // prior belief to defend, and waiting would let an
                        // illegal lane through the frame it appears.
                        mem.settled = raw;
                        mem.soft = raw_soft;
                        mem.streak = 0;
                    }
                }
            }

            next_memory[lane.lane_id] = mem;
            report.lane_verdicts[lane.lane_id] = mem.settled;
            report.soft_illegal[lane.lane_id] =
                (mem.settled == LaneLegality::ILLEGAL && mem.soft);
            if (applicable) {
                report.signed_dist_mm[lane.lane_id] = signed_dist;
            }
        }
        verdict_memory_ = std::move(next_memory);
        return report;
    }

    // ── Filtering ────────────────────────────────────────────────────────────

    // allow_soft_illegal: pass true while a lane_change intent (or committed
    // lane-change maneuver) is active - lanes ruled ILLEGAL by a dashed-yellow
    // reference stay visible so the planner can pick them as the change
    // target. Hard (solid-yellow) ILLEGAL lanes are always removed.
    // Turn-lane markings are exempt from the drop regardless of verdict: a
    // turn-lane's whole purpose is guiding a crossing over the divider that
    // separates it from the current lane, so classifying it ILLEGAL is
    // expected, not a reason to hide it from the turn selector (see
    // is_turn_commit_ready in legacy_lane_model.hpp, which needs it to ever
    // see a candidate). The verdict itself is still computed and reported
    // for debug - only the removal is skipped.
    static PathObservationFrame filter(const PathObservationFrame& frame,
                                       const LaneLegalityReport& report,
                                       const std::string& exempt_lane_id,
                                       bool allow_soft_illegal = false) {
        PathObservationFrame out;
        out.timestamp_ms = frame.timestamp_ms;
        out.markings = frame.markings;
        for (const auto& lane : frame.lanes) {
            if (lane.label == LABEL_TURN_LANE) { out.lanes.push_back(lane); continue; }
            if (is_filtered(lane.lane_id, report, exempt_lane_id, allow_soft_illegal)) continue;
            out.lanes.push_back(lane);
        }
        return out;
    }

    // Same verdicts applied to the legacy LaneCandidate list so the state
    // machine, turn-lane selection, and direct-IPM fallback never see a lane
    // the planner was denied (single world-view per frame). Turn-lane is
    // exempt here too - see filter() above.
    static std::vector<LaneCandidate> filter_legacy(const std::vector<LaneCandidate>& lanes,
                                                    const LaneLegalityReport& report,
                                                    const std::string& exempt_lane_id,
                                                    bool allow_soft_illegal = false) {
        std::vector<LaneCandidate> out;
        for (size_t i = 0; i < lanes.size(); ++i) {
            if (lanes[i].label == LABEL_TURN_LANE) { out.push_back(lanes[i]); continue; }
            std::string id = legacy_lane_id(lanes[i], i);
            if (is_filtered(id, report, exempt_lane_id, allow_soft_illegal)) continue;
            out.push_back(lanes[i]);
        }
        return out;
    }

    // Mirrors PathObservationBuilder's id derivation (id -> track_id ->
    // obj_<label>_<index>) so legacy candidates and observations agree.
    static std::string legacy_lane_id(const LaneCandidate& cand, size_t index) {
        const json& obj = cand.raw_obj;
        for (const char* key : {"id", "track_id"}) {
            if (obj.contains(key)) {
                const auto& v = obj[key];
                if (v.is_string()) return v.get<std::string>();
                if (!v.is_null()) return v.dump();
            }
        }
        return "obj_" + std::to_string(cand.label) + "_" + std::to_string(index);
    }

    // Auto-return target: legal main/other lane laterally nearest the vehicle
    // centerline. Turn-lanes are never a legality-return target.
    static const LaneObservation* nearest_legal_lane(const PathObservationFrame& frame,
                                                     const LaneLegalityReport& report,
                                                     const std::string& exclude_lane_id) {
        const LaneObservation* best = nullptr;
        double best_abs_x = 1e18;
        for (const auto& lane : frame.lanes) {
            if (lane.lane_id == exclude_lane_id) continue;
            if (lane.label != LABEL_MAIN_LANE && lane.label != LABEL_OTHER_LANE) continue;
            auto it = report.lane_verdicts.find(lane.lane_id);
            if (it == report.lane_verdicts.end() || it->second != LaneLegality::LEGAL) continue;
            if (lane.points.empty()) continue;
            double mean_x = 0.0;
            for (const auto& p : lane.points) mean_x += p.x;
            mean_x /= lane.points.size();
            if (std::abs(mean_x) < best_abs_x) {
                best_abs_x = std::abs(mean_x);
                best = &lane;
            }
        }
        return best;
    }

    const Params& params() const { return params_; }
    void set_params(const Params& p) { params_ = p; }

private:
    struct VerdictMemory {
        LaneLegality settled = LaneLegality::UNKNOWN;
        LaneLegality pending = LaneLegality::UNKNOWN;
        bool soft = false;          // softness of the settled verdict's ruling ref
        bool pending_soft = false;
        int streak = 0;
    };

    struct HeldRef {
        YellowReference ref;
        int age = 0;
    };

    static bool is_filtered(const std::string& lane_id, const LaneLegalityReport& report,
                            const std::string& exempt_lane_id, bool allow_soft_illegal) {
        if (!exempt_lane_id.empty() && lane_id == exempt_lane_id) return false;
        auto it = report.lane_verdicts.find(lane_id);
        if (it == report.lane_verdicts.end() || it->second != LaneLegality::ILLEGAL) return false;
        if (allow_soft_illegal) {
            auto soft_it = report.soft_illegal.find(lane_id);
            if (soft_it != report.soft_illegal.end() && soft_it->second) return false;
        }
        return true;
    }

    Params params_;
    std::map<std::string, HeldRef> held_;
    std::map<std::string, VerdictMemory> verdict_memory_;
    int yellow_age_frames_ = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Plan F auto-return (F3). Debounced decision to leave a stably-ILLEGAL lane
// for the nearest LEGAL one. The caller (control_node) turns a trigger into an
// internal LANE_CHANGE_* intent that flows through the unchanged maneuver
// machinery (latch, hold windows, marking gate, manager commit/abort); this
// class only owns the per-frame debounce so it stays header-testable.
// ─────────────────────────────────────────────────────────────────────────────
class LegalityAutoReturn {
public:
    struct Decision {
        bool trigger = false;        // set the internal LANE_CHANGE intent now
        bool go_left = false;        // direction toward the target lane
        std::string target_lane_id;  // debug only - planner re-selects on its own
    };

    // eligible: FOLLOW_MAIN intent, no committed maneuver, no override active,
    // gate + return both enabled. Any ineligible frame resets the debounce -
    // a flickering ILLEGAL verdict must not accumulate across maneuvers.
    // frame must be the post-filter observation frame (exempt lane included).
    Decision step(bool eligible,
                  const PathObservationFrame& frame,
                  const LaneLegalityReport& report,
                  const std::string& followed_lane_id,
                  int debounce_frames) {
        Decision d;
        if (!eligible || followed_lane_id.empty()) {
            streak_ = 0;
            return d;
        }
        auto it = report.lane_verdicts.find(followed_lane_id);
        bool followed_illegal =
            it != report.lane_verdicts.end() && it->second == LaneLegality::ILLEGAL;
        if (!followed_illegal) {
            streak_ = 0;
            return d;
        }
        streak_++;
        if (streak_ < debounce_frames) return d;
        const LaneObservation* target =
            LaneLegalityGate::nearest_legal_lane(frame, report, followed_lane_id);
        // No legal lane in sight: never drop the path we have - stay on the
        // exempt lane with only the debug streak exposed, retry next frame.
        if (target == nullptr || target->points.empty()) return d;
        double mean_x = 0.0;
        for (const auto& p : target->points) mean_x += p.x;
        mean_x /= static_cast<double>(target->points.size());
        d.trigger = true;
        d.go_left = mean_x < 0.0;
        d.target_lane_id = target->lane_id;
        streak_ = 0;
        return d;
    }

    int streak() const { return streak_; }
    void reset() { streak_ = 0; }

private:
    int streak_ = 0;  // consecutive eligible frames with the followed lane ILLEGAL
};
