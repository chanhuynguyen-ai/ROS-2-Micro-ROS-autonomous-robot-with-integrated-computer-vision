#!/usr/bin/env python3
"""Record real /avs/telemetry frames (from the actual NCNN inference node) to JSONL.

Used to build simulator scenarios from real video runs: play a video through
video_publisher_node + ncnn_inference_node, record every telemetry frame here,
then cut segments into scenario fixtures with telemetry_to_scenario.py.

Usage:
    python3 capture_real_telemetry.py --output capture.jsonl [--duration 90]
"""
import argparse
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class TelemetryRecorder(Node):
    def __init__(self, output_path: str):
        super().__init__("telemetry_recorder")
        self.out = open(output_path, "w")
        self.count = 0
        self.sub = self.create_subscription(String, "/avs/telemetry", self.on_msg, 10)

    def on_msg(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        record = {"recv_wall_ms": int(time.time() * 1000), "telemetry": data}
        self.out.write(json.dumps(record) + "\n")
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f"recorded {self.count} frames")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=90.0,
                        help="seconds to record before exiting (0 = until Ctrl-C)")
    args = parser.parse_args()

    rclpy.init()
    node = TelemetryRecorder(args.output)
    deadline = time.time() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if deadline is not None and time.time() >= deadline:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.out.close()
        print(f"recorded {node.count} frames -> {args.output}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
