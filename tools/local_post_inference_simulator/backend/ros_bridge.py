import time
import json
import logging
import threading
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

logger = logging.getLogger("avs_sim_bridge")

class ROSSimBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_sim_bridge_node')
        
        # Publishers
        self.synthetic_payload_pub = self.create_publisher(
            String,
            '/avs/sim/synthetic_payload',
            10
        )
        self.route_intent_pub = self.create_publisher(
            String,
            '/avs/route_intent',
            10
        )
        self.cmd_pub = self.create_publisher(
            String,
            '/avs/cmd',
            10
        )

        # Subscribers
        self.telemetry_realworld_sub = self.create_subscription(
            String,
            '/avs/telemetry_realworld',
            self.telemetry_realworld_callback,
            10
        )
        self.ipm_debug_sub = self.create_subscription(
            String,
            '/avs/ipm_debug',
            self.ipm_debug_callback,
            10
        )
        self.lane_state_sub = self.create_subscription(
            String,
            '/avs/lane_state',
            self.lane_state_callback,
            10
        )
        self.control_error_sub = self.create_subscription(
            String,
            '/avs/control_error',
            self.control_error_callback,
            10
        )
        self.route_intent_ack_sub = self.create_subscription(
            String,
            '/avs/route_intent_ack',
            self.route_intent_ack_callback,
            10
        )

        # Thread-safe storage for subscribed data
        self._lock = threading.Lock()
        self.latest_telemetry_realworld: Optional[Dict[str, Any]] = None
        self.latest_ipm_debug: Optional[Dict[str, Any]] = None
        self.latest_lane_state: Optional[Dict[str, Any]] = None
        self.latest_control_error: Optional[Dict[str, Any]] = None
        self.latest_route_intent_ack: Optional[Dict[str, Any]] = None
        self.telemetry_realworld_recv_time: float = 0.0
        self.ipm_debug_recv_time: float = 0.0
        self.lane_state_recv_time: float = 0.0
        self.control_error_recv_time: float = 0.0
        self.route_intent_ack_recv_time: float = 0.0

        logger.info("ROSSimBridgeNode initialized.")

    def telemetry_realworld_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self.latest_telemetry_realworld = data
                self.telemetry_realworld_recv_time = time.time()
            logger.debug("Received telemetry_realworld update.")
        except Exception as e:
            logger.error(f"Error parsing telemetry_realworld message: {e}")

    def ipm_debug_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self.latest_ipm_debug = data
                self.ipm_debug_recv_time = time.time()
            logger.debug("Received ipm_debug update.")
        except Exception as e:
            logger.error(f"Error parsing ipm_debug message: {e}")

    def lane_state_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self.latest_lane_state = data
                self.lane_state_recv_time = time.time()
            logger.debug("Received lane_state update.")
        except Exception as e:
            logger.error(f"Error parsing lane_state message: {e}")

    def control_error_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self.latest_control_error = data
                self.control_error_recv_time = time.time()
            logger.debug("Received control_error update.")
        except Exception as e:
            logger.error(f"Error parsing control_error message: {e}")

    def route_intent_ack_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self.latest_route_intent_ack = data
                self.route_intent_ack_recv_time = time.time()
            logger.debug("Received route_intent_ack update.")
        except Exception as e:
            logger.error(f"Error parsing route_intent_ack message: {e}")

    def publish_telemetry(self, telemetry_data: Dict[str, Any], timestamp_ms: int = 0):
        msg = String()
        # Include metadata that the standalone node will pick up and enrich
        payload = {
            "timestamp_ms": timestamp_ms,
            "detections": telemetry_data.get("detections", {}),
            "objects": telemetry_data.get("objects", [])
        }
        msg.data = json.dumps(payload)
        self.synthetic_payload_pub.publish(msg)
        logger.info(f"Published payload to /avs/sim/synthetic_payload (objects: {len(payload['objects'])}, timestamp_ms: {timestamp_ms})")

    def wait_for_telemetry_subscribers(
        self,
        timeout_sec: float = 1.0,
        poll_interval_sec: float = 0.02,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.synthetic_payload_pub.get_subscription_count() > 0:
                return True
            time.sleep(poll_interval_sec)
        return self.synthetic_payload_pub.get_subscription_count() > 0

    def publish_route_intent(self, intent: str, seq: int = 1):
        msg = String()
        payload = {
            "intent": intent,
            "source": "simulator",
            "seq": seq
        }
        msg.data = json.dumps(payload)
        self.route_intent_pub.publish(msg)
        logger.info(f"Published route_intent to /avs/route_intent: {intent} (seq: {seq})")

    def publish_cmd(self, cmd_dict: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(cmd_dict)
        self.cmd_pub.publish(msg)
        logger.info(f"Published cmd to /avs/cmd: {cmd_dict}")

    def wait_for_cmd_subscribers(
        self,
        timeout_sec: float = 1.0,
        poll_interval_sec: float = 0.02,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.cmd_pub.get_subscription_count() > 0:
                return True
            time.sleep(poll_interval_sec)
        return self.cmd_pub.get_subscription_count() > 0

    def wait_for_route_intent_ack(
        self,
        intent: str,
        seq: int,
        timeout_sec: float = 1.0,
        poll_interval_sec: float = 0.005,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                ack = self.latest_route_intent_ack

            if ack is not None:
                ack_seq = ack.get("seq")
                ack_intent = ack.get("intent")
                ack_requested_intent = ack.get("requested_intent")
                ack_accepted = bool(ack.get("accepted", False))
                if (
                    ack_accepted
                    and ack_seq == seq
                    and (ack_intent == intent or ack_requested_intent == intent)
                ):
                    return True

            time.sleep(poll_interval_sec)

        return False

    def get_latest_outputs(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "telemetry_realworld": self.latest_telemetry_realworld,
                "telemetry_realworld_time": self.telemetry_realworld_recv_time,
                "ipm_debug": self.latest_ipm_debug,
                "ipm_debug_time": self.ipm_debug_recv_time,
                "lane_state": self.latest_lane_state,
                "lane_state_time": self.lane_state_recv_time,
                "control_error": self.latest_control_error,
                "control_error_time": self.control_error_recv_time,
                "route_intent_ack": self.latest_route_intent_ack,
                "route_intent_ack_time": self.route_intent_ack_recv_time
            }

    def clear_latest_outputs(self):
        with self._lock:
            self.latest_telemetry_realworld = None
            self.latest_ipm_debug = None
            self.latest_lane_state = None
            self.latest_control_error = None
            self.latest_route_intent_ack = None
            self.telemetry_realworld_recv_time = 0.0
            self.ipm_debug_recv_time = 0.0
            self.lane_state_recv_time = 0.0
            self.control_error_recv_time = 0.0
            self.route_intent_ack_recv_time = 0.0


# ROS2 context runner
_bridge_node: Optional[ROSSimBridgeNode] = None
_spin_thread: Optional[threading.Thread] = None
_bridge_lock = threading.Lock()

def get_bridge_node() -> ROSSimBridgeNode:
    global _bridge_node, _spin_thread
    with _bridge_lock:
        if _bridge_node is None:
            if not rclpy.ok():
                # Initialise rclpy if not already done
                rclpy.init()
            _bridge_node = ROSSimBridgeNode()
            _spin_thread = threading.Thread(target=lambda: rclpy.spin(_bridge_node), daemon=True)
            _spin_thread.start()
            logger.info("ROS2 spin thread started for simulator bridge.")
        return _bridge_node

def shutdown_bridge(shutdown_context: bool = True, join_timeout_sec: float = 1.0):
    global _bridge_node, _spin_thread
    with _bridge_lock:
        node = _bridge_node
        spin_thread = _spin_thread
        _bridge_node = None
        _spin_thread = None

    if shutdown_context and rclpy.ok():
        rclpy.shutdown()

    if spin_thread is not None and spin_thread.is_alive():
        spin_thread.join(timeout=join_timeout_sec)

    if node is not None:
        node.destroy_node()
        logger.info("ROS2 simulator bridge node shutdown.")
