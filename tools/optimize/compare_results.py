#!/usr/bin/env python3
"""Build the plan §7 decision table from docs/optimize/results/*.json.

Usage:
    python3 tools/optimize/compare_results.py [results_dir]

Reads every metadata_<run_id>.json + matching benchmark_summary_<run_id>.json
pair in results_dir (default: docs/optimize/results) and prints a markdown
table comparing mode, latency, FPS, and lane-quality signals.
"""
import glob
import json
import os
import sys


def load_run(meta_path, results_dir):
    with open(meta_path) as f:
        meta = json.load(f)
    run_id = meta.get("run_id", "")
    summary_path = os.path.join(results_dir, f"benchmark_summary_{run_id}.json")
    if not os.path.exists(summary_path):
        return None
    with open(summary_path) as f:
        summary = json.load(f)
    if summary.get("status") != "success":
        return None
    return meta, summary


def fmt(v, digits=1):
    return f"{v:.{digits}f}" if isinstance(v, (int, float)) else "N/A"


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else "docs/optimize/results"
    rows = []
    for meta_path in sorted(glob.glob(os.path.join(results_dir, "metadata_*.json"))):
        loaded = load_run(meta_path, results_dir)
        if loaded is None:
            continue
        meta, summary = loaded
        metrics = summary.get("metrics", {})
        derived = summary.get("derived_metrics", {})

        def m(key, field="mean"):
            return metrics.get(key, {}).get(field)

        rows.append({
            "mode": meta.get("mode", "unknown"),
            "input_source": meta.get("input_source", "unknown"),
            "run_id": meta.get("run_id", "unknown"),
            "avg_inference_ms": m("inference_latency_ms", "mean"),
            "p95_total_ms": m("node_total_latency_ms", "p95"),
            "p95_output_age_ms": m("output_age_ms", "p95"),
            "avg_fps": m("processing_fps", "mean"),
            "main_lane_missing_rate": derived.get("main_lane_missing_rate"),
            "turn_lane_present_rate": derived.get("turn_lane_present_rate"),
            "temp_before_c": meta.get("temperature_before", "N/A"),
            "temp_after_c": meta.get("temperature_after", "N/A"),
            "throttled_after": meta.get("throttled_after", "N/A"),
        })

    if not rows:
        print(f"No successful benchmark runs found under {results_dir}", file=sys.stderr)
        sys.exit(1)

    headers = [
        "Mode", "Fixture", "Avg inference (ms)", "P95 total (ms)",
        "P95 output age (ms)", "Avg FPS", "Main-lane missing rate",
        "Turn-lane present rate", "Temp before->after", "Throttled after", "Run ID",
    ]
    print("| " + " | ".join(headers) + " |")
    print("|" + "---|" * len(headers))
    for r in rows:
        print("| " + " | ".join([
            r["mode"],
            r["input_source"],
            fmt(r["avg_inference_ms"]),
            fmt(r["p95_total_ms"]),
            fmt(r["p95_output_age_ms"]),
            fmt(r["avg_fps"], 2),
            fmt(r["main_lane_missing_rate"], 3),
            fmt(r["turn_lane_present_rate"], 3),
            f"{r['temp_before_c']}->{r['temp_after_c']}",
            str(r["throttled_after"]),
            r["run_id"],
        ]) + " |")


if __name__ == "__main__":
    main()
