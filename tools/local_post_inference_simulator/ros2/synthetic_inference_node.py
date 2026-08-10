#!/usr/bin/env python3
import time
import json
import logging
from typing import Dict, Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

# Set up simple logging formatting
logging.basicConfig(level=logging.INFO, format='%(levelname)s [%(name)s]: %(message)s')
logger = logging.getLogger("synthetic_inference_node")

class SyntheticInferenceNode(Node):
    def __init__(self):
        super().__init__('synthetic_inference_node')

        # Publisher to the actual /avs/telemetry
        self.telemetry_pub = self.create_publisher(
            String,
            '/avs/telemetry',
            10
        )

        # Subscriber to receive payloads from the simulator backend
        self.payload_sub = self.create_subscription(
            String,
            '/avs/sim/synthetic_payload',
            self.payload_callback,
            10
        )

        logger.info("SyntheticInferenceNode initialized. Listening to /avs/sim/synthetic_payload -> /avs/telemetry")

    def payload_callback(self, msg: String):
        try:
            telemetry_data = json.loads(msg.data)
            timestamp_ms = telemetry_data.get("timestamp_ms", int(time.time() * 1000))

            # Reconstruct the full message formatting typical of the inference output
            payload = {
                "input_fps": telemetry_data.get("input_fps", 30.0),
                "processing_fps": telemetry_data.get("processing_fps", 30.0),
                "publish_fps": telemetry_data.get("publish_fps", 30.0),
                "fps": telemetry_data.get("fps", 30.0),
                "bridge_latency_ms": telemetry_data.get("bridge_latency_ms", 1.0),
                "inference_latency_ms": telemetry_data.get("inference_latency_ms", 10.0),
                "post_processing_latency_ms": telemetry_data.get("post_processing_latency_ms", 1.0),
                "contour_time_ms": telemetry_data.get("contour_time_ms", 0.5),
                "json_finalize_latency_ms": telemetry_data.get("json_finalize_latency_ms", 0.2),
                "publish_latency_ms": telemetry_data.get("publish_latency_ms", 0.2),
                "node_total_latency_ms": telemetry_data.get("node_total_latency_ms", 12.0),
                "output_age_ms": telemetry_data.get("output_age_ms", 12.0),
                "last_input_age_ms": telemetry_data.get("last_input_age_ms", 0.0),
                "node_processing_latency_ms": telemetry_data.get("node_processing_latency_ms", 12.0),
                "full_latency_ms": telemetry_data.get("full_latency_ms", 12.0),
                "input_age_ms": telemetry_data.get("input_age_ms", 0.0),
                "streaming": telemetry_data.get("streaming", True),
                "timestamp_ms": timestamp_ms,
                "detections": telemetry_data.get("detections", {}),
                "objects": telemetry_data.get("objects", [])
            }

            out_msg = String()
            out_msg.data = json.dumps(payload)
            self.telemetry_pub.publish(out_msg)

            logger.info(f"Published synthetic telemetry to /avs/telemetry (objects: {len(payload['objects'])}, timestamp_ms: {timestamp_ms})")
            
        except Exception as e:
            logger.error(f"Error handling payload message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SyntheticInferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        logger.info("Shutdown requested, stopping.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
