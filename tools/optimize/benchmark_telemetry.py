#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import argparse
import time
import csv
import os
import math
import sys
from datetime import datetime

class BenchmarkHarness(Node):
    def __init__(self, output_dir, max_frames, duration_sec, warmup_frames, run_id,
                 warmup_timeout_sec=90.0):
        super().__init__('benchmark_harness')
        self.subscription = self.create_subscription(
            String,
            '/avs/telemetry',
            self.telemetry_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.output_dir = output_dir
        self.max_frames = max_frames
        self.duration_sec = duration_sec
        self.warmup_frames = warmup_frames
        self.warmup_timeout_sec = warmup_timeout_sec
        self.run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")

        self.frame_count = 0
        self.recorded_frames = 0
        self.start_time = None

        self.records = []
        self.metrics_keys = [
            'input_fps', 'processing_fps', 'publish_fps', 
            'bridge_latency_ms', 'inference_latency_ms', 'post_processing_latency_ms', 
            'contour_time_ms', 'json_finalize_latency_ms', 'publish_latency_ms',
            'node_processing_latency_ms', 'node_total_latency_ms',
            'profile_preprocess_ms', 'profile_extractor_ms', 'profile_proposal_ms',
            'profile_nms_ms', 'profile_mask_decode_ms',
            'input_age_ms', 'output_age_ms'
        ]

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        self.csv_path = os.path.join(self.output_dir, f'benchmark_{self.run_id}.csv')
        self.json_path = os.path.join(self.output_dir, f'benchmark_summary_{self.run_id}.json')
        
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # We will dynamically write headers when the first record comes in since we want to include detections too.
        self.header_written = False

        self.init_time = time.time()
        self.last_msg_time = self.init_time
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info(f"Benchmark Harness Started.")
        self.get_logger().info(f"Warmup frames: {self.warmup_frames}")
        self.get_logger().info(f"Target: {self.max_frames} frames or {self.duration_sec} seconds")
        self.get_logger().info(f"Output Directory: {self.output_dir}")
        self.get_logger().info(f"Run ID: {self.run_id}")

    def fail_benchmark(self, reason):
        self.get_logger().error(reason)
        self.get_logger().error("Failing non-zero.")
        if not self.csv_file.closed:
            self.csv_file.close()
            
        summary = {
            "run_id": self.run_id,
            "status": "failed",
            "failure_reason": reason,
            "total_frames_recorded": self.recorded_frames
        }
        with open(self.json_path, 'w') as f:
            json.dump(summary, f, indent=4)
        
        os._exit(1)

    def watchdog_callback(self):
        now = time.time()
        if self.frame_count > 0 and now - self.last_msg_time > 10.0:
            self.fail_benchmark("Watchdog timeout: No telemetry received for 10 seconds.")
        if self.start_time is None and (now - self.init_time) > self.warmup_timeout_sec:
            self.fail_benchmark(f"Watchdog timeout: Warmup not completed within {self.warmup_timeout_sec:.0f} seconds.")

    def telemetry_callback(self, msg):
        self.last_msg_time = time.time()
        self.frame_count += 1

        if self.frame_count <= self.warmup_frames:
            if self.frame_count % 10 == 0:
                self.get_logger().info(f"Warmup... {self.frame_count}/{self.warmup_frames}")
            return

        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("Warmup complete. Starting recording.")

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.fail_benchmark(f"Failed to decode telemetry JSON: {e}")

        # Flatten record
        record = {}
        missing_metrics = []
        for key in self.metrics_keys:
            if key not in data:
                missing_metrics.append(key)
            record[key] = data.get(key, 0.0)
            
        if missing_metrics:
            self.fail_benchmark(f"Missing required metrics in telemetry: {missing_metrics}")
            
        if "detections" not in data or not isinstance(data["detections"], dict):
            self.fail_benchmark("Missing or invalid 'detections' object in telemetry")
            
        detections = data["detections"]
        for k, v in detections.items():
            record[f"det_{k}"] = v

        # Quality signals
        if "objects" not in data or not isinstance(data["objects"], list):
            self.fail_benchmark("Missing or invalid 'objects' array in telemetry")
            
        objects = data["objects"]
        total_poly_points = 0
        total_poly_area = 0.0
        classes_of_interest = {"main-lane": 0, "other-lane": 0, "turn-lane": 0, "stop-line": 0}
        poly_area_by_class = {"main-lane": 0.0, "other-lane": 0.0, "turn-lane": 0.0, "stop-line": 0.0}
        
        for obj in objects:
            if not isinstance(obj, dict):
                self.fail_benchmark("Malformed object in telemetry: not a dictionary")
                
            cls_name = obj.get("class_name", "")
            if cls_name in classes_of_interest:
                classes_of_interest[cls_name] += 1
                
            polygons = obj.get("polygons", [])
            if not isinstance(polygons, list):
                self.fail_benchmark("Malformed polygons in telemetry: not a list")
                
            obj_area = 0.0
            for poly in polygons:
                if not isinstance(poly, list):
                    self.fail_benchmark("Malformed polygon: not a list of points")
                    
                n_pts = len(poly)
                total_poly_points += n_pts
                if n_pts >= 3:
                    area = 0.0
                    try:
                        for i in range(n_pts):
                            j = (i + 1) % n_pts
                            if not isinstance(poly[i], list) or len(poly[i]) < 2 or \
                               not isinstance(poly[j], list) or len(poly[j]) < 2:
                                self.fail_benchmark("Malformed polygon point: missing coordinates")
                            area += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1]
                    except (TypeError, IndexError) as e:
                        self.fail_benchmark(f"Malformed polygon data: {e}")
                    obj_area += abs(area) / 2.0
            
            total_poly_area += obj_area
            if cls_name in poly_area_by_class:
                poly_area_by_class[cls_name] += obj_area
                
        record["qual_total_objects"] = len(objects)
        record["qual_total_poly_points"] = total_poly_points
        record["qual_total_poly_area"] = total_poly_area
        for cls_name in classes_of_interest:
            safe_name = cls_name.replace('-', '_')
            record[f"qual_{safe_name}"] = classes_of_interest[cls_name]
            record[f"qual_{safe_name}_poly_area"] = poly_area_by_class[cls_name]

        self.records.append(record)
        self.recorded_frames += 1

        if not self.header_written:
            self.csv_keys = list(record.keys())
            self.csv_writer.writerow(self.csv_keys)
            self.header_written = True

        self.csv_writer.writerow([record[k] for k in self.csv_keys])

        elapsed_time = time.time() - self.start_time
        
        if self.recorded_frames % 30 == 0:
            self.get_logger().info(f"Recorded {self.recorded_frames} frames. Elapsed: {elapsed_time:.1f}s")

        if (self.max_frames > 0 and self.recorded_frames >= self.max_frames) or \
           (self.duration_sec > 0 and elapsed_time >= self.duration_sec):
            self.get_logger().info("Benchmark condition met. Finalizing...")
            self.finalize_benchmark()
            os._exit(0)

    def calculate_percentiles(self, values):
        if not values:
            return {"mean": 0.0, "p50": 0.0, "p90": 0.0, "p95": 0.0, "max": 0.0}
        
        values.sort()
        n = len(values)
        
        def get_p(p):
            idx = int(math.ceil(p / 100.0 * n)) - 1
            idx = max(0, min(idx, n - 1))
            return values[idx]
            
        mean_val = sum(values) / n
        return {
            "mean": round(mean_val, 2),
            "p50": round(get_p(50), 2),
            "p90": round(get_p(90), 2),
            "p95": round(get_p(95), 2),
            "max": round(values[-1], 2)
        }

    def finalize_benchmark(self):
        self.csv_file.close()
        
        summary = {
            "run_id": self.run_id,
            "status": "success",
            "total_frames_recorded": self.recorded_frames,
            "derived_metrics": {},
            "metrics": {}
        }
        
        if self.recorded_frames > 0:
            for key in self.csv_keys:
                values = [r[key] for r in self.records]
                summary["metrics"][key] = self.calculate_percentiles(values)
            
            # Derived metrics
            main_lane_frames = sum(1 for r in self.records if r.get("qual_main_lane", 0) > 0)
            turn_lane_frames = sum(1 for r in self.records if r.get("qual_turn_lane", 0) > 0)
            stop_line_frames = sum(1 for r in self.records if r.get("qual_stop_line", 0) > 0)
            
            summary["derived_metrics"]["main_lane_missing_rate"] = round(1.0 - (main_lane_frames / self.recorded_frames), 4)
            summary["derived_metrics"]["turn_lane_present_rate"] = round(turn_lane_frames / self.recorded_frames, 4)
            summary["derived_metrics"]["stop_line_present_rate"] = round(stop_line_frames / self.recorded_frames, 4)
                
        with open(self.json_path, 'w') as f:
            json.dump(summary, f, indent=4)
            
        self.get_logger().info(f"Results saved to {self.csv_path} and {self.json_path}")
        
        if self.recorded_frames > 0:
            self.get_logger().info("\n--- BENCHMARK SUMMARY ---")
            for m in ['processing_fps', 'node_total_latency_ms', 'inference_latency_ms', 'output_age_ms']:
                if m in summary["metrics"]:
                    s = summary["metrics"][m]
                    self.get_logger().info(f"{m:25s}: mean={s['mean']:6.2f} p95={s['p95']:6.2f} max={s['max']:6.2f}")


def main(args=None):
    parser = argparse.ArgumentParser(description='AVS Telemetry Benchmark Harness')
    parser.add_argument('--output-dir', type=str, default='docs/optimize/results', help='Directory to save results')
    parser.add_argument('--frames', type=int, default=300, help='Number of frames to record (0 for infinite)')
    parser.add_argument('--duration', type=float, default=0.0, help='Duration in seconds (0 for infinite)')
    parser.add_argument('--warmup', type=int, default=30, help='Number of frames to ignore before recording')
    parser.add_argument('--warmup-timeout', type=float, default=90.0, help='Seconds to wait for warmup before failing (raise for Vulkan: first inference compiles SPIR-V pipelines)')
    parser.add_argument('--run-id', type=str, default='', help='Unique identifier for this run')
    
    parsed_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)

    benchmark_harness = BenchmarkHarness(
        output_dir=parsed_args.output_dir,
        max_frames=parsed_args.frames,
        duration_sec=parsed_args.duration,
        warmup_frames=parsed_args.warmup,
        run_id=parsed_args.run_id,
        warmup_timeout_sec=parsed_args.warmup_timeout
    )

    exit_code = 0
    try:
        rclpy.spin(benchmark_harness)
    except SystemExit as e:
        exit_code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
    except KeyboardInterrupt:
        benchmark_harness.get_logger().info("Interrupted by user. Finalizing...")
        benchmark_harness.finalize_benchmark()
    finally:
        benchmark_harness.destroy_node()
        rclpy.shutdown()
        if exit_code != 0:
            sys.exit(exit_code)

if __name__ == '__main__':
    main()
