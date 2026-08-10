#!/usr/bin/env python3
"""Cut a recorded /avs/telemetry capture (capture_real_telemetry.py JSONL) into a
simulator scenario fixture.

Mask fidelity: every contour of every object is copied VERBATIM from the real
inference output (no simplification, no scaling). points_px holds the first
contour (canvas editor), polygons_px holds all contours (what the pipeline
replays; see ObjectSchema.polygons_px).

Timing: captures are made with video_publisher_node fps_override=4.0 on a 20fps
video (5x slow-motion) so the CPU inference sees ~every video frame. Video time
for a captured frame is therefore wall_elapsed / 5. --target-fps subsamples to
a realistic processing cadence (default 14, the Pi's rate).

Usage:
    python3 telemetry_to_scenario.py --input capture.jsonl \
        --start-s 14.5 --end-s 19.5 --name real_intersection_gap_crossing \
        --intent follow_main --output ../fixtures/real_intersection_gap_crossing.json
"""
import argparse
import json


SLOWDOWN = 5.0  # 20fps video played back at 4fps => wall time is 5x video time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="JSONL from capture_real_telemetry.py")
    parser.add_argument("--start-s", type=float, required=True, help="segment start, video seconds")
    parser.add_argument("--end-s", type=float, required=True, help="segment end, video seconds")
    parser.add_argument("--name", required=True)
    parser.add_argument("--description", default="")
    parser.add_argument("--intent", default="follow_main")
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--target-fps", type=float, default=14.0,
                        help="subsample to this processing cadence in video time (0 = keep all)")
    parser.add_argument("--slowdown", type=float, default=SLOWDOWN,
                        help="wall-time/video-time ratio of the capture run")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    records = [json.loads(l) for l in open(args.input)]
    if not records:
        raise SystemExit("empty capture")
    wall0 = records[0]["recv_wall_ms"]

    frames = []
    next_pick_s = args.start_s
    step_s = (1.0 / args.target_fps) if args.target_fps > 0 else 0.0
    for rec in records:
        video_s = (rec["recv_wall_ms"] - wall0) / 1000.0 / args.slowdown
        if video_s < args.start_s or video_s > args.end_s:
            continue
        if step_s > 0.0 and video_s + 1e-9 < next_pick_s:
            continue
        next_pick_s = video_s + step_s

        objects = []
        for obj in rec["telemetry"].get("objects", []):
            polygons = obj.get("polygons") or []
            polygons = [p for p in polygons if len(p) >= 3]
            if not polygons:
                continue  # nothing the IPM could consume (e.g. empty sign masks)
            objects.append({
                "id": str(obj.get("track_id") or obj.get("id")),
                "class_name": obj.get("class_name", ""),
                "label": int(obj.get("label", -1)),
                "shape": "polygon",
                "points_px": polygons[0],
                "polygons_px": polygons,
                "confidence": float(obj.get("prob", 1.0)),
            })
        frames.append({"frame_id": len(frames) + 1, "objects": objects, "video_s": round(video_s, 3)})

    if not frames:
        raise SystemExit("no frames in the requested window")

    scenario = {
        "name": args.name,
        "description": args.description or (
            f"Real-inference capture from test/test_video/video_test1.mp4 "
            f"[{args.start_s:.1f}s..{args.end_s:.1f}s], masks verbatim (no rescale/reshape), "
            f"subsampled to ~{args.target_fps:g} FPS processing cadence."
        ),
        "source": {
            "video": "test/test_video/video_test1.mp4",
            "video_window_s": [args.start_s, args.end_s],
            "model": "models/best_ncnn_model",
            "target_fps": args.target_fps,
        },
        "canvas": {"width": 640, "height": 480},
        "calibration": {"source": "config/calibration.json"},
        "route_intent": {"intent": args.intent, "seq": args.seq},
        "frames": frames,
    }

    with open(args.output, "w") as f:
        json.dump(scenario, f)
    n_obj = sum(len(f["objects"]) for f in frames)
    print(f"{args.name}: {len(frames)} frames, {n_obj} objects -> {args.output}")


if __name__ == "__main__":
    main()
