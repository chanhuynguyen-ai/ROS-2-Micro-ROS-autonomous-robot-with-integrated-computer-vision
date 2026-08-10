#!/usr/bin/env python3
"""Phân tích jsonl do `turn_observe.py` ghi: tách từng cú rẽ, in số đo.

Chạy trên laptop:
  python3 tools/turn_observation/turn_analyze.py run19.jsonl
  python3 tools/turn_observation/turn_analyze.py run19.jsonl --episodes   # kèm trace từng frame

Bốn mục A–D ứng với bốn thay đổi đang cần kiểm chứng; mỗi mục in ra PHÂN BỐ,
không in một con số tóm tắt. Bài học đã ghi trong skill: ngưỡng đặt sai vì
người ta nhìn trung bình thay vì nhìn hai cụm.
"""
import json
import math
import sys
from collections import Counter

# Đúng chuỗi `route_intent_name` / `trajectory_kind_name` phát ra
# (`decision_types.hpp`): CHỮ THƯỜNG. So sánh có chuẩn hoá hoa/thường vì nhầm
# chỗ này không báo lỗi — nó chỉ lặng lẽ tách ra 0 cú rẽ.
TURN_KINDS = ("turn_left", "turn_right")


def is_turn(name):
    return isinstance(name, str) and name.strip().lower() in TURN_KINDS


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except Exception:
                pass
    return rows


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    i = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[i]


def fmt_dist(vals, unit=""):
    if not vals:
        return "(không có mẫu)"
    return "n=%d  min=%.0f%s  p25=%.0f  median=%.0f  p75=%.0f  max=%.0f%s" % (
        len(vals), min(vals), unit, pct(vals, 25), pct(vals, 50),
        pct(vals, 75), max(vals), unit)


def build_frames(rows):
    """Ghép lane_state với yaw đo gần nhất theo thời gian."""
    frames = []
    yaw = None
    for r in rows:
        if r["topic"] == "odom":
            yaw = r["data"].get("yaw_unwrapped_deg")
        elif r["topic"] == "lane_state":
            d = r["data"]
            if isinstance(d, dict) and "decision_state" in d:
                d = dict(d)
                d["_t"] = r["t"]
                if d.get("_yaw_unwrapped_deg") is None:
                    d["_yaw_unwrapped_deg"] = yaw
                frames.append(d)
    return frames


def path_front_y(f):
    pts = f.get("active_trajectory_points") or []
    return pts[0][1] if pts else None


def split_episodes(frames, tail_frames=40):
    """Một episode = chuỗi frame có turn intent, nối thêm đuôi để bắt post-latch."""
    eps = []
    cur = None
    tail = 0
    for f in frames:
        in_turn = is_turn(f.get("route_intent")) or f.get("turn_latch_active")
        if in_turn:
            if cur is None:
                cur = []
            cur.append(f)
            tail = tail_frames
        elif cur is not None:
            cur.append(f)
            tail -= 1
            if tail <= 0:
                eps.append(cur)
                cur = None
    if cur:
        eps.append(cur)
    return eps


def analyze_episode(ep, idx):
    intents = [f.get("route_intent") for f in ep if is_turn(f.get("route_intent"))]
    intent = Counter(intents).most_common(1)[0][0] if intents else "?"
    kinds = Counter(f.get("trajectory_kind") for f in ep)

    latch_idx = [i for i, f in enumerate(ep) if f.get("turn_latch_active")]
    yaws = [f.get("_yaw_unwrapped_deg") for f in ep if f.get("_yaw_unwrapped_deg") is not None]
    total_rot = (yaws[-1] - yaws[0]) if len(yaws) >= 2 else float("nan")

    out = {
        "idx": idx,
        "intent": intent,
        "frames": len(ep),
        "dur_s": ep[-1]["_t"] - ep[0]["_t"],
        "kinds": kinds,
        "total_rot": total_rot,
        "latched": bool(latch_idx),
        "turn_lane_seen_frac": (sum(1 for f in ep if f.get("turn_lane_detected")) /
                                max(1, len(ep))),
    }

    # Front-y của path trong pha tiếp cận (trước khi latch đóng) — mục A.
    approach_end = latch_idx[0] if latch_idx else len(ep)
    out["approach_front_y"] = [path_front_y(f) for f in ep[:approach_end]
                               if path_front_y(f) is not None]

    # Span quan sát trước latch — mục còn mở 3b-quinquies.
    out["pre_latch_span"] = [abs(f.get("turn_latch_observed_span_deg") or 0.0)
                             for f in ep[:approach_end]]

    if latch_idx:
        a, b = latch_idx[0], latch_idx[-1]
        lf, rf = ep[a], ep[b]
        out["latch_obs_span"] = lf.get("turn_latch_observed_span_deg")
        out["latch_ext_span"] = lf.get("turn_latch_extended_span_deg")
        out["latch_len"] = lf.get("turn_latch_length_mm")
        out["latch_ext_mm"] = lf.get("turn_latch_extension_mm")
        out["latch_frames"] = b - a + 1
        out["turned_at_release"] = rf.get("turn_latch_heading_turned_deg")
        out["progress_max"] = max(f.get("turn_latch_progress_mm") or 0.0 for f in ep[a:b + 1])
        out["consumed_frac"] = (out["progress_max"] / out["latch_len"] * 100.0
                                if out["latch_len"] else float("nan"))
        # Lý do nhả nằm ở frame ngay SAU cạnh xuống.
        out["release_reason"] = (ep[b + 1].get("turn_latch_release_reason")
                                 if b + 1 < len(ep) else rf.get("turn_latch_release_reason"))
        # Góc xoay THẬT (đo từ /odom_raw) trong lúc latch — đối chứng độc lập
        # với con số latch tự báo. Đây là mục D.
        ya = ep[a].get("_yaw_unwrapped_deg")
        yb = ep[b].get("_yaw_unwrapped_deg")
        out["rot_during_latch"] = (yb - ya) if (ya is not None and yb is not None) else float("nan")
        y0 = ep[0].get("_yaw_unwrapped_deg")
        out["rot_before_latch"] = (ya - y0) if (ya is not None and y0 is not None) else float("nan")

        # Nhảy progress = skip-to-runout (mục C).
        skips = []
        prev = None
        for f in ep[a:b + 1]:
            p = f.get("turn_latch_progress_mm") or 0.0
            if prev is not None and p - prev > 300.0:
                skips.append((prev, p, f.get("turn_latch_heading_turned_deg")))
            prev = p
        out["skips"] = skips

    out["stub_frames"] = sum(1 for f in ep if f.get("normalization_mode") == "post_latch_stub")
    out["recovery_frames"] = sum(1 for f in ep if f.get("decision_state") == "RECOVERY")
    out["invalid_frames"] = sum(1 for f in ep if not f.get("trajectory_valid"))
    return out


def print_episode(e, target=90.0):
    print("\n─── Cú rẽ #%d: %s ─── %d frame / %.1fs" %
          (e["idx"], e["intent"], e["frames"], e["dur_s"]))
    print("  xoay THẬT cả episode (odom): %+.1f deg   (giao lộ ~%.0f deg)" %
          (e["total_rot"], target))
    print("  thấy turn-lane: %.0f%% frame | kind: %s" %
          (e["turn_lane_seen_frac"] * 100.0,
           ", ".join("%s=%d" % kv for kv in e["kinds"].most_common(3))))
    fy = e["approach_front_y"]
    if fy:
        print("  [A] path_front_y lúc tiếp cận: %s" % fmt_dist(fy, "mm"))
        bad = [v for v in fy if v >= 300.0]
        print("      >=300mm (ngưỡng neo mới): %d/%d frame = %.0f%%" %
              (len(bad), len(fy), 100.0 * len(bad) / len(fy)))
    if e["pre_latch_span"]:
        over = [v for v in e["pre_latch_span"] if v > 100.0]
        print("  [mở] |span| quan sát trước latch >100 deg: %d/%d frame" %
              (len(over), len(e["pre_latch_span"])))

    if not e["latched"]:
        print("  latch: KHÔNG đóng trong cú rẽ này")
    else:
        print("  latch: obs_span=%.1f ext_span=%.1f len=%.0fmm ext=%.0fmm, %d frame" % (
            e["latch_obs_span"] or 0, e["latch_ext_span"] or 0,
            e["latch_len"] or 0, e["latch_ext_mm"] or 0, e["latch_frames"]))
        print("  [B] nhả: reason=%s  turned=%.1f deg  tiêu thụ %.0f%% path (%.0fmm)" % (
            e["release_reason"], e["turned_at_release"] or 0.0,
            e["consumed_frac"], e["progress_max"]))
        print("  [C] skip-to-runout: %d lần%s" % (
            len(e["skips"]),
            "".join("  (%.0f->%.0fmm @ %.1f deg)" % s for s in e["skips"])))
        print("  [D] xoay đo được: trước latch %+.1f, trong latch %+.1f  "
              "| latch tự báo %.1f" % (
                  e["rot_before_latch"], e["rot_during_latch"],
                  e["turned_at_release"] or 0.0))
    print("  sau latch: stub %d frame, RECOVERY %d frame, path invalid %d frame" %
          (e["stub_frames"], e["recovery_frames"], e["invalid_frames"]))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    paths = [a for a in sys.argv[1:] if not a.startswith("--")]
    show_trace = "--episodes" in sys.argv

    frames = []
    for path in paths:
        f = build_frames(load(path))
        if not f:
            print("Không có frame lane_state nào trong %s" % path)
            continue
        print("%s: %d frame" % (path, len(f)))
        frames.extend(f)
    if not frames:
        sys.exit(1)
    path = " + ".join(paths)

    dur = frames[-1]["_t"] - frames[0]["_t"]
    print("=" * 72)
    print("%s: %d frame lane_state trong %.1fs (%.1f FPS)" %
          (path, len(frames), dur, len(frames) / max(dur, 1e-6)))
    have_yaw = sum(1 for f in frames if f.get("_yaw_unwrapped_deg") is not None)
    print("frame có yaw đo: %d/%d%s" % (
        have_yaw, len(frames),
        "   ⚠ /odom_raw KHÔNG tới — mọi số đo đối chứng ở [D] vô nghĩa"
        if have_yaw < len(frames) * 0.5 else ""))

    eps = split_episodes(frames)
    print("tách được %d cú rẽ" % len(eps))

    results = [analyze_episode(ep, i + 1) for i, ep in enumerate(eps)]
    for e in results:
        print_episode(e)

    # ── Tổng hợp bốn mục cần kiểm chứng ──────────────────────────────────────
    print("\n" + "=" * 72)
    print("TỔNG HỢP")
    latched = [e for e in results if e["latched"]]

    all_fy = [v for e in results for v in e["approach_front_y"]]
    print("\n[A] Neo turn (kTurnAnchorMaxLaneStartYMm=300)")
    print("    path_front_y mọi frame tiếp cận: %s" % fmt_dist(all_fy, "mm"))
    if all_fy:
        cluster = [v for v in all_fy if 450.0 <= v <= 700.0]
        print("    cụm 450-700mm (dấu hiệu neo vào lane bên kia giao lộ): "
              "%d/%d = %.0f%%   [trước fix: 13-14%%]" %
              (len(cluster), len(all_fy), 100.0 * len(cluster) / len(all_fy)))

    print("\n[B] Gate nhả latch (release_min_span_frac=0.9)")
    if latched:
        print("    turned_deg lúc nhả: %s" %
              fmt_dist([abs(e["turned_at_release"] or 0.0) for e in latched], "deg"))
        print("    %% path đã tiêu thụ:  %s" %
              fmt_dist([e["consumed_frac"] for e in latched], "%"))
        print("    lý do nhả: %s" %
              dict(Counter(e["release_reason"] for e in latched)))
        print("    [trước fix: nhả ở 59.5-62.9 deg sau khi chỉ đi 15-43% path]")
    else:
        print("    không có latch nào")

    print("\n[C] Skip-to-runout khi yaw đo đã đủ góc")
    n_skip = sum(len(e["skips"]) for e in latched)
    print("    %d lần trên %d cú latch" % (n_skip, len(latched)))
    if latched:
        print("    turned_deg cuối cùng: %s" %
              fmt_dist([abs(e["turned_at_release"] or 0.0) for e in latched], "deg"))
        print("    [trước fix: một cú rẽ trái đi 2175/2191mm và ra ở 118 deg]")

    print("\n[D] Datum yaw lấy lúc commit thay vì lúc latch")
    for side in TURN_KINDS:
        grp = [e for e in latched if str(e["intent"]).lower() == side]
        if not grp:
            continue
        print("    %s: latch tự báo %s" %
              (side, fmt_dist([abs(e["turned_at_release"] or 0.0) for e in grp], "deg")))
        print("        xoay THẬT cả episode  %s" %
              fmt_dist([abs(e["total_rot"]) for e in grp if not math.isnan(e["total_rot"])], "deg"))
        print("        swing trước latch     %s" %
              fmt_dist([e["rot_before_latch"] for e in grp
                        if not math.isnan(e["rot_before_latch"])], "deg"))
    print("    [trước fix: cùng 80 deg đúng ra thành 59-70 bên phải, 113-125 bên trái]")

    # [E] Hai đại lượng quyết định: góc THẬT lúc nhả (kết quả cuối cùng người
    # ngồi cạnh xe nhìn thấy) và sai số datum (nguyên nhân). Tách riêng vì mọi
    # mục A-D ở trên đều đọc con số latch tự báo, tức đã nhiễm sai số này.
    print("\n[E] Góc THẬT lúc nhả latch, và sai số datum")
    real = [abs(e["rot_during_latch"]) for e in latched
            if not math.isnan(e["rot_during_latch"])]
    if real:
        print("    góc thật lúc nhả:  %s   (mục tiêu 90)" % fmt_dist(real, "deg"))
        near = [v for v in real if 80.0 <= v <= 100.0]
        print("    rơi trong 90+-10 deg: %d/%d = %.0f%%" %
              (len(near), len(real), 100.0 * len(near) / len(real)))
        errs = []
        for e in latched:
            if math.isnan(e["rot_during_latch"]):
                continue
            # Cùng quy ước dấu: |báo| - |thật|. Âm = latch báo THIẾU.
            errs.append(abs(e["turned_at_release"] or 0.0) - abs(e["rot_during_latch"]))
        print("    sai số datum (báo - thật): %s" % fmt_dist(errs, "deg"))
        print("    biên độ dao động: %.0f deg" % (max(errs) - min(errs)))

    print("\n[sau latch] stub/RECOVERY")
    if latched:
        print("    stub frame:     %s" % fmt_dist([e["stub_frames"] for e in latched]))
        print("    RECOVERY frame: %s" % fmt_dist([e["recovery_frames"] for e in latched]))

    if show_trace:
        for ep, e in zip(eps, results):
            print("\n--- trace cú rẽ #%d (%s) ---" % (e["idx"], e["intent"]))
            print("  t     intent      state        kind         latch prog/len   turned yaw    front_y")
            for f in ep:
                pts = f.get("active_trajectory_points") or []
                print("  %5.1f %-11s %-12s %-12s %s %5.0f/%-5.0f %6.1f %6.1f %6s" % (
                    f["_t"], f.get("route_intent", "?")[:11],
                    (f.get("decision_state") or "?")[:12],
                    (f.get("trajectory_kind") or "?")[:12],
                    "Y" if f.get("turn_latch_active") else ".",
                    f.get("turn_latch_progress_mm") or 0.0,
                    f.get("turn_latch_length_mm") or 0.0,
                    f.get("turn_latch_heading_turned_deg") or 0.0,
                    f.get("_yaw_unwrapped_deg") if f.get("_yaw_unwrapped_deg") is not None else float("nan"),
                    ("%.0f" % pts[0][1]) if pts else "-"))


if __name__ == "__main__":
    main()
