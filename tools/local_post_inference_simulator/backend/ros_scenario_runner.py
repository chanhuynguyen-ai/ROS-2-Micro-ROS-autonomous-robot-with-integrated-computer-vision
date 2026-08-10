import os
import time
import json
import logging
import threading
import shutil
import subprocess
import tempfile
from typing import Dict, Any, Optional, List

from .scenario_schema import ScenarioSchema
from .mask_to_objects import convert_to_telemetry_payload
from .ros_bridge import get_bridge_node
from .ipm_adapter import (
    discard_stale_telemetry_realworld,
    load_calibration_status,
    matching_telemetry_realworld,
    summarize_ipm_telemetry,
)

logger = logging.getLogger("avs_sim_runner")

# Resolve repository root relative to this file's location (tools/local_post_inference_simulator/backend/ros_scenario_runner.py)
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
WORKSPACE_PREFIX = "/workspace/"

class ScenarioRunner:
    def __init__(self):
        self.scenario: Optional[ScenarioSchema] = None
        self.mode: str = "direct"  # "direct" or "rasterized"
        self.latest_report: Optional[Dict[str, Any]] = None
        self.current_frame_idx: int = 0
        self.playback_rate_hz: float = 5.0
        self.is_playing: bool = False
        # Index of the frame a step() call is currently (or was most recently)
        # actually processing, set before ROS I/O starts and held until the next
        # step() begins. Unlike current_frame_idx (which advances to the *next*
        # frame to run as soon as a step finishes), this is what /api/scenarios/status
        # should report as "the frame ROS is running right now" -- see get_status().
        self._running_frame_idx: Optional[int] = None

        self.history: List[Dict[str, Any]] = []
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._current_token: int = 0
        self._latest_ipm_output: Optional[Dict[str, Any]] = None
        self._debug_param_enabled: bool = False
        self._last_sub_count: int = 0
        self._owns_debug_param: bool = False

    def _ensure_debug_param_enabled(self) -> None:
        bridge = get_bridge_node()
        sub_count = bridge.synthetic_payload_pub.get_subscription_count() if bridge else 0
        if sub_count > 0 and self._last_sub_count == 0:
            logger.info("IPM node came online. Re-applying debug parameters.")
            self._debug_param_enabled = False
            self._owns_debug_param = False
        self._last_sub_count = sub_count
        
        if self._debug_param_enabled or sub_count == 0:
            return
            
        try:
            check = subprocess.run(
                ["ros2", "param", "get", "/ipm_transform_node", "publish_debug_centerline"],
                capture_output=True,
                text=True,
                timeout=1.0
            )
            if "True" in check.stdout:
                logger.info("publish_debug_centerline is already true. Simulator will not take ownership.")
                self._debug_param_enabled = True
                self._owns_debug_param = False
                return

            result = subprocess.run(
                ["ros2", "param", "set", "/ipm_transform_node", "publish_debug_centerline", "true"],
                capture_output=True,
                text=True,
                timeout=1.0
            )
            if result.returncode == 0:
                self._debug_param_enabled = True
                self._owns_debug_param = True
                logger.info("Successfully enabled publish_debug_centerline on IPM node.")
            else:
                logger.error(f"Failed to set debug parameter on IPM node. stderr: {result.stderr.strip()}")
        except Exception as e:
            logger.error(f"Exception setting debug parameter on IPM node: {e}")

    def _disable_debug_param(self) -> None:
        if not self._debug_param_enabled:
            return
            
        if not self._owns_debug_param:
            logger.info("Skipping debug param disable because simulator does not own it.")
            self._debug_param_enabled = False
            return
            
        try:
            result = subprocess.run(
                ["ros2", "param", "set", "/ipm_transform_node", "publish_debug_centerline", "false"],
                capture_output=True,
                text=True,
                timeout=1.0
            )
            if result.returncode == 0:
                self._debug_param_enabled = False
                self._owns_debug_param = False
                logger.info("Disabled publish_debug_centerline on IPM node.")
            else:
                logger.error(
                    f"Failed to disable publish_debug_centerline on IPM node (rc={result.returncode}, stderr={result.stderr.strip()}). "
                    "Leaving ownership state intact so a later call retries."
                )
        except Exception as e:
            logger.warning(f"Exception disabling debug parameter on IPM node: {e}. Will retry later.")

    @staticmethod
    def _workspace_path_to_repo_path(path: str) -> Optional[str]:
        normalized_path = os.path.normpath(path)
        if normalized_path == "/workspace":
            return REPO_ROOT
        if normalized_path.startswith(WORKSPACE_PREFIX):
            return os.path.abspath(os.path.join(REPO_ROOT, normalized_path[len(WORKSPACE_PREFIX):]))
        return None

    @staticmethod
    def _resolve_calibration_path(calib_source: str) -> str:
        if os.path.isabs(calib_source):
            if os.path.exists(calib_source):
                return calib_source

            repo_path = ScenarioRunner._workspace_path_to_repo_path(calib_source)
            if repo_path and os.path.exists(repo_path):
                return repo_path

            return calib_source
        return os.path.abspath(os.path.join(REPO_ROOT, calib_source))

    @staticmethod
    def _write_file_atomically(source_path: str, dest_path: str) -> None:
        dest_dir = os.path.dirname(dest_path)
        os.makedirs(dest_dir, exist_ok=True)

        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb",
                dir=dest_dir,
                prefix=".calibration.",
                suffix=".tmp",
                delete=False,
            ) as tmp_file:
                temp_path = tmp_file.name
                with open(source_path, "rb") as src_file:
                    while True:
                        chunk = src_file.read(1024 * 1024)
                        if not chunk:
                            break
                        tmp_file.write(chunk)
                tmp_file.flush()
                os.fsync(tmp_file.fileno())

            os.replace(temp_path, dest_path)
        finally:
            if temp_path and os.path.exists(temp_path):
                try:
                    os.unlink(temp_path)
                except OSError:
                    pass

    def _next_frame_token(self) -> int:
        self._current_token += 1
        return self._current_token

    @staticmethod
    def _expected_route_intent(intent: str) -> str:
        if intent == "straight":
            return "follow_main"
        return intent

    def _wait_for_route_intent_state(self, bridge, expected_intent: str, timestamp_ms: int) -> bool:
        max_wait_sec = 1.0
        poll_interval_sec = 0.005
        start_wait = time.monotonic()

        while time.monotonic() - start_wait < max_wait_sec:
            lane_state = bridge.get_latest_outputs().get("lane_state")
            if lane_state is not None:
                lane_state_ts = lane_state.get("timestamp_ms", -1)
                lane_state_intent = lane_state.get("route_intent")
                if lane_state_ts == timestamp_ms and lane_state_intent == expected_intent:
                    return True

            time.sleep(poll_interval_sec)

        return False

    @staticmethod
    def _matching_telemetry_realworld(outputs: Dict[str, Any], timestamp_ms: int) -> Optional[Dict[str, Any]]:
        return matching_telemetry_realworld(outputs, timestamp_ms)

    @staticmethod
    def _discard_telemetry_realworld(outputs: Dict[str, Any]) -> Dict[str, Any]:
        return discard_stale_telemetry_realworld(outputs)

    @staticmethod
    def _wait_for_telemetry_realworld(
        bridge,
        timestamp_ms: int,
        timeout_sec: float = 0.5,
        poll_interval_sec: float = 0.005,
    ) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_sec
        outputs = bridge.get_latest_outputs()
        if ScenarioRunner._matching_telemetry_realworld(outputs, timestamp_ms) is not None:
            return outputs

        while time.monotonic() < deadline:
            outputs = bridge.get_latest_outputs()
            if ScenarioRunner._matching_telemetry_realworld(outputs, timestamp_ms) is not None:
                return outputs
            time.sleep(poll_interval_sec)

        return ScenarioRunner._discard_telemetry_realworld(outputs)

    def _sync_route_intent(self, bridge, intent: str, seq: int) -> Dict[str, Any]:
        expected_intent = self._expected_route_intent(intent)

        if bridge.wait_for_route_intent_ack(expected_intent, seq=seq, timeout_sec=1.0):
            return {"synced": True, "method": "route_intent_ack"}

        logger.warning(
            "Timed out waiting for /avs/route_intent_ack for intent=%s seq=%s. "
            "Falling back to lane_state barrier.",
            intent,
            seq,
        )

        barrier_timestamp_ms = self._next_frame_token()
        bridge.clear_latest_outputs()
        bridge.publish_telemetry(
            {"detections": {}, "objects": []},
            timestamp_ms=barrier_timestamp_ms,
        )

        if self._wait_for_route_intent_state(bridge, expected_intent, barrier_timestamp_ms):
            return {
                "synced": True,
                "method": "lane_state_barrier",
                "barrier_timestamp_ms": barrier_timestamp_ms,
            }

        return {
            "synced": False,
            "method": "timeout",
            "barrier_timestamp_ms": barrier_timestamp_ms,
            "error": f"Timed out waiting for route intent '{expected_intent}' to be visible in /avs/lane_state",
        }

    def load_scenario(self, scenario: ScenarioSchema, mode: str = "direct"):
        if mode not in ("direct", "rasterized"):
            raise ValueError(f"Invalid mode '{mode}'. Must be 'direct' or 'rasterized'")
            
        # Stop and join any active playback thread
        old_thread = None
        with self._lock:
            if self._thread and self._thread.is_alive() and self._thread != threading.current_thread():
                self.is_playing = False
                self._stop_event.set()
                old_thread = self._thread
        if old_thread:
            logger.info("Waiting for previous playback thread to exit before loading new scenario...")
            old_thread.join()
            
        self._disable_debug_param()
            
        # Apply the scenario calibration
        calib_source = scenario.calibration.source if scenario.calibration else None
        if calib_source:
            resolved_source = self._resolve_calibration_path(calib_source)
                
            if os.path.exists(resolved_source):
                active_calib_path = "/workspace/config/calibration.json"
                if not os.path.exists(os.path.dirname(active_calib_path)):
                    active_calib_path = os.path.abspath(os.path.join(REPO_ROOT, "config/calibration.json"))
                
                if os.path.abspath(resolved_source) != os.path.abspath(active_calib_path):
                    logger.info(f"Applying scenario calibration from {resolved_source} to {active_calib_path}")
                    try:
                        self._write_file_atomically(resolved_source, active_calib_path)
                    except Exception as e:
                        logger.error(f"Failed to copy scenario calibration file atomically: {e}")
            else:
                logger.warning(f"Scenario calibration source file {calib_source} not found at {resolved_source}")

        with self._lock:
            self.scenario = scenario
            self.mode = mode
            self.current_frame_idx = 0
            self.is_playing = False
            self.history = []
            self._stop_event.clear()
            self._current_token = 0
            self._latest_ipm_output = None
            
        logger.info(f"Loaded scenario: {scenario.name} in mode: {mode}")

    def get_calibration_status(self) -> Dict[str, Any]:
        active_calib_path = "/workspace/config/calibration.json"
        if not os.path.exists(os.path.dirname(active_calib_path)):
            active_calib_path = os.path.abspath(os.path.join(REPO_ROOT, "config/calibration.json"))
        return load_calibration_status(active_calib_path, REPO_ROOT)

    def get_latest_ipm_output(self) -> Dict[str, Any]:
        with self._lock:
            latest_ipm_output = self._latest_ipm_output

        if latest_ipm_output is not None:
            return latest_ipm_output

        return {
            "telemetry_realworld": None,
            "telemetry_realworld_time": 0.0,
            "ipm_summary": summarize_ipm_telemetry(None),
            "calibration": self.get_calibration_status(),
            "lane_state": None,
        }

    def step_ipm(self) -> Optional[Dict[str, Any]]:
        """
        Executes one scenario frame through synthetic telemetry -> production IPM.
        This Phase 3 path only waits for /avs/telemetry_realworld, so it does not
        require control_node or route_intent acknowledgements.
        """
        with self._lock:
            if not self.scenario or not self.scenario.frames:
                logger.warning("No valid scenario loaded to run IPM step.")
                return None

        self._ensure_debug_param_enabled()

        with self._lock:
            frames = self.scenario.frames

            if self.current_frame_idx >= len(frames):
                logger.info("Reached end of scenario frames. Wrapping to 0.")
                self.current_frame_idx = 0

            frame = frames[self.current_frame_idx]
            width = self.scenario.canvas.width
            height = self.scenario.canvas.height
            calibration = self.get_calibration_status()

            bridge = get_bridge_node()
            bridge.clear_latest_outputs()
            telemetry_subscriber_ready = bridge.wait_for_telemetry_subscribers(timeout_sec=1.0)

            telemetry_payload = convert_to_telemetry_payload(
                frame.objects,
                width=width,
                height=height,
                mode=self.mode,
            )

            if not telemetry_subscriber_ready:
                error_msg = "No subscribers ready for /avs/telemetry. Ensure ipm_transform_node is running."
                logger.warning(error_msg)
                step_result = {
                    "run_mode": "ipm",
                    "frame_id": frame.frame_id,
                    "frame_idx": self.current_frame_idx,
                    "input_objects": telemetry_payload["objects"],
                    "calibration": calibration,
                    "ipm_status": {
                        "timestamp_ms": 0,
                        "timestamp_matched": False,
                        "timeout_sec": 1.0,
                        "source_topic": "/avs/telemetry",
                        "output_topic": "/avs/telemetry_realworld",
                        "telemetry_subscriber_ready": False,
                    },
                    "outputs": {
                        "telemetry_realworld": None,
                    },
                    "ipm_summary": summarize_ipm_telemetry(None),
                    "error": error_msg,
                }
                self._latest_ipm_output = {
                    "telemetry_realworld": None,
                    "telemetry_realworld_time": 0.0,
                    "ipm_summary": step_result["ipm_summary"],
                    "calibration": calibration,
                    "ipm_status": step_result["ipm_status"],
                    "error": error_msg,
                }

                if not self.is_playing:
                    self._disable_debug_param()

                return step_result

            frame_timestamp_ms = self._next_frame_token()
            bridge.publish_telemetry(telemetry_payload, timestamp_ms=frame_timestamp_ms)
            outputs = self._wait_for_telemetry_realworld(
                bridge,
                timestamp_ms=frame_timestamp_ms,
                timeout_sec=0.5,
            )

            telemetry_realworld = self._matching_telemetry_realworld(outputs, frame_timestamp_ms)
            timestamp_matched = telemetry_realworld is not None
            ipm_summary = summarize_ipm_telemetry(telemetry_realworld)

            step_result = {
                "run_mode": "ipm",
                "frame_id": frame.frame_id,
                "frame_idx": self.current_frame_idx,
                "input_objects": telemetry_payload["objects"],
                "calibration": calibration,
                "ipm_status": {
                    "timestamp_ms": frame_timestamp_ms,
                    "timestamp_matched": timestamp_matched,
                    "timeout_sec": 0.5,
                    "source_topic": "/avs/telemetry",
                    "output_topic": "/avs/telemetry_realworld",
                    "telemetry_subscriber_ready": telemetry_subscriber_ready,
                },
                "outputs": {
                    "telemetry_realworld": telemetry_realworld,
                },
                "ipm_summary": ipm_summary,
            }

            if not timestamp_matched:
                step_result["error"] = (
                    "Timed out waiting for matching /avs/telemetry_realworld. "
                    "Ensure ipm_transform_node is running and calibration is valid."
                )

            self._latest_ipm_output = {
                "telemetry_realworld": telemetry_realworld,
                "telemetry_realworld_time": outputs.get("telemetry_realworld_time") if timestamp_matched else 0.0,
                "ipm_summary": ipm_summary,
                "calibration": calibration,
                "ipm_status": step_result["ipm_status"],
                "error": step_result.get("error"),
                # step_ipm() never runs control_node, so there is no /avs/lane_state to
                # report. Explicit None (not a missing key) so /api/ipm/bev's debug
                # trajectory overlay can tell "no data because this was Step IPM" apart
                # from a lookup bug.
                "lane_state": None,
            }

            self.history.append(step_result)
            self.current_frame_idx += 1
            if self.current_frame_idx >= len(frames):
                logger.info("Finished running all frames in scenario.")
                self.is_playing = False

            if not self.is_playing:
                self._disable_debug_param()

            return step_result

    def step(self) -> Optional[Dict[str, Any]]:
        """
        Executes a single step (one frame) of the scenario.
        Publishes to ROS2 and waits for processing output.
        """
        with self._lock:
            if not self.scenario or not self.scenario.frames:
                logger.warning("No valid scenario loaded to step.")
                return None

        self._ensure_debug_param_enabled()
        
        with self._lock:
            frames = self.scenario.frames
            
            if self.current_frame_idx >= len(frames):
                logger.info("Reached end of scenario frames. Wrapping to 0.")
                self.current_frame_idx = 0
                
            frame = frames[self.current_frame_idx]
            # Set before any ROS I/O so a concurrent get_status() call can see which
            # frame is actually in flight -- current_frame_idx itself only advances
            # to the *next* frame once this step fully completes.
            self._running_frame_idx = self.current_frame_idx
            width = self.scenario.canvas.width
            height = self.scenario.canvas.height
            if getattr(frame, "route_intent", None):
                intent = frame.route_intent.intent
                seq = frame.route_intent.seq + self.current_frame_idx
            else:
                intent = self.scenario.route_intent.intent
                seq = self.scenario.route_intent.seq + self.current_frame_idx

            # Retrieve the ROS bridge node
            bridge = get_bridge_node()
            
            # Clear old outputs to avoid consuming stale values
            bridge.clear_latest_outputs()

            # Convert objects to telemetry payload format
            telemetry_payload = convert_to_telemetry_payload(
                frame.objects, 
                width=width, 
                height=height, 
                mode=self.mode
            )

            # 1. Publish route intent
            bridge.publish_route_intent(intent, seq=seq)

            route_intent_sync = self._sync_route_intent(bridge, intent, seq)
            if not route_intent_sync["synced"]:
                logger.error(route_intent_sync["error"])
                step_result = {
                    "run_mode": "full",
                    "frame_id": frame.frame_id,
                    "frame_idx": self.current_frame_idx,
                    "input_objects": telemetry_payload["objects"],
                    "route_intent": {"intent": intent, "seq": seq},
                    "route_intent_sync": route_intent_sync,
                    "outputs": {
                        "telemetry_realworld": None,
                        "lane_state": None,
                        "control_error": None
                    },
                    "ipm_summary": summarize_ipm_telemetry(None),
                    "error": route_intent_sync["error"],
                }
                self._latest_ipm_output = {
                    "telemetry_realworld": None,
                    "telemetry_realworld_time": 0.0,
                    "ipm_summary": step_result["ipm_summary"],
                    "calibration": self.get_calibration_status(),
                    "ipm_status": {
                        "timestamp_ms": 0,
                        "timestamp_matched": False,
                        "source_topic": "/avs/telemetry",
                        "output_topic": "/avs/telemetry_realworld",
                    },
                    "error": route_intent_sync["error"],
                }
                self.history.append(step_result)

                if not self.is_playing:
                    self._disable_debug_param()

                return step_result

            bridge.clear_latest_outputs()
            
            if not bridge.wait_for_telemetry_subscribers(timeout_sec=1.0):
                if not self.is_playing:
                    self._disable_debug_param()
                return {
                    "run_mode": "full",
                    "frame_id": frame.frame_id,
                    "frame_idx": self.current_frame_idx,
                    "error": "Timeout waiting for /avs/telemetry subscribers."
                }

            # Generate a unique token after any fallback barrier so timestamps stay monotonic.
            frame_timestamp_ms = self._next_frame_token()
            
            publish_time = time.time()
            # 2. Publish synthetic telemetry with the unique frame timestamp
            bridge.publish_telemetry(telemetry_payload, timestamp_ms=frame_timestamp_ms)

            # 3. Wait for the C++ pipeline (IPM node -> control node) to complete processing.
            # Poll at small intervals up to a maximum duration (e.g. 0.3s) until all outputs are received.
            max_wait_sec = 0.3
            poll_interval_sec = 0.005
            start_wait = time.monotonic()
            
            while time.monotonic() - start_wait < max_wait_sec:
                outputs = bridge.get_latest_outputs()
                
                trw = outputs.get("telemetry_realworld")
                ls = outputs.get("lane_state")
                ce = outputs.get("control_error")
                
                if trw is not None and ls is not None and ce is not None:
                    # Verify trw timestamp matches this frame's unique timestamp
                    trw_ts = trw.get("timestamp_ms", 0)
                    
                    if trw_ts == frame_timestamp_ms:
                        ls_time = outputs.get("lane_state_time", 0.0)
                        ce_time = outputs.get("control_error_time", 0.0)
                        
                        if ls_time >= publish_time and ce_time >= publish_time:
                            break
                        
                time.sleep(poll_interval_sec)

            # 4. Read final outputs from subscribers
            outputs = bridge.get_latest_outputs()
            telemetry_realworld = self._matching_telemetry_realworld(outputs, frame_timestamp_ms)
            ipm_summary = summarize_ipm_telemetry(telemetry_realworld)

            # Compile step result
            step_result = {
                "run_mode": "full",
                "frame_id": frame.frame_id,
                "frame_idx": self.current_frame_idx,
                "input_objects": telemetry_payload["objects"],
                "route_intent": {"intent": intent, "seq": seq},
                "route_intent_sync": route_intent_sync,
                "outputs": {
                    "telemetry_realworld": telemetry_realworld,
                    "lane_state": outputs.get("lane_state"),
                    "control_error": outputs.get("control_error")
                },
                "ipm_summary": ipm_summary
            }

            self._latest_ipm_output = {
                "telemetry_realworld": telemetry_realworld,
                "telemetry_realworld_time": outputs.get("telemetry_realworld_time") if telemetry_realworld is not None else 0.0,
                "ipm_summary": ipm_summary,
                "calibration": self.get_calibration_status(),
                "ipm_status": {
                    "timestamp_ms": frame_timestamp_ms,
                    "timestamp_matched": telemetry_realworld is not None,
                    "source_topic": "/avs/telemetry",
                    "output_topic": "/avs/telemetry_realworld",
                },
                "lane_state": outputs.get("lane_state"),
            }

            # Append to historical list of run outputs
            self.history.append(step_result)
            self.current_frame_idx += 1
            if self.current_frame_idx >= len(frames):
                logger.info("Finished running all frames in scenario.")
                self.is_playing = False

            if not self.is_playing:
                self._disable_debug_param()

            return step_result

    def play(self):
        old_thread = None
        with self._lock:
            if self._thread and self._thread.is_alive() and self._thread != threading.current_thread():
                self.is_playing = False
                self._stop_event.set()
                old_thread = self._thread
        if old_thread:
            logger.info("Waiting for previous playback thread to exit...")
            old_thread.join()

        with self._lock:
            if not self.scenario:
                logger.warning("No scenario loaded to play.")
                return
            if self.is_playing:
                logger.info("Already playing.")
                return
            
            self.is_playing = True
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()
        logger.info("Started playing scenario thread.")

    def pause(self):
        old_thread = None
        with self._lock:
            if not self.is_playing:
                return
            self.is_playing = False
            self._stop_event.set()
            if self._thread and self._thread.is_alive() and self._thread != threading.current_thread():
                old_thread = self._thread
        if old_thread:
            logger.info("Waiting for playback thread to pause...")
            old_thread.join()
        self._disable_debug_param()
        logger.info("Paused scenario playback.")

    def stop(self):
        old_thread = None
        with self._lock:
            self.is_playing = False
            self.current_frame_idx = 0
            self._running_frame_idx = None
            self._stop_event.set()
            if self._thread and self._thread.is_alive() and self._thread != threading.current_thread():
                old_thread = self._thread
        if old_thread:
            logger.info("Waiting for playback thread to stop...")
            old_thread.join()
        self._disable_debug_param()
        logger.info("Stopped scenario playback and reset frame index.")

    def wait_until_stopped(self, timeout: Optional[float] = None) -> bool:
        """Block until the playback thread exits. Returns True once stopped (or if never started).

        Unlike polling `is_playing`, this guarantees `latest_report` is populated when it
        returns True: `_run_loop` sets `latest_report` before returning, and `is_playing`
        alone can go False earlier (e.g. while `step()` is still blocked on a slow
        `ros2 param set` call), so a caller polling `is_playing` can race ahead of the
        thread that produces the report.
        """
        with self._lock:
            thread = self._thread
        if thread is None:
            return True
        thread.join(timeout=timeout)
        return not thread.is_alive()

    def get_status(self) -> Dict[str, Any]:
        # Snapshot is_playing + _running_frame_idx together, before touching _lock.
        # step() holds _lock for its entire ROS round-trip (can be hundreds of ms),
        # so waiting for it here would make this call only ever observe post-step
        # state -- i.e. current_frame_idx already advanced past the frame that was
        # actually running. Plain attribute reads are atomic under the GIL, so this
        # is safe without extra locking.
        #
        # Both the gating check and the "is_playing" field in the response MUST
        # come from this single snapshot, not from a second self.is_playing read
        # taken later inside the lock -- otherwise stop()/pause() flipping the flag
        # in between could produce a self-contradictory response, e.g.
        # is_playing: false with a stale running_frame_idx still set. See review.
        is_playing_snapshot = self.is_playing
        running_frame_idx = self._running_frame_idx if is_playing_snapshot else None
        with self._lock:
            return {
                "scenario_name": self.scenario.name if self.scenario else None,
                "current_frame_idx": self.current_frame_idx,
                "total_frames": len(self.scenario.frames) if self.scenario else 0,
                "is_playing": is_playing_snapshot,
                "mode": self.mode,
                "history_length": len(self.history),
                "running_frame_idx": running_frame_idx,
            }

    def _run_loop(self):
        bridge = get_bridge_node()
        # Publish reset command
        if bridge.wait_for_cmd_subscribers(timeout_sec=1.0):
            bridge.publish_cmd({"cmd": "reset"})
            time.sleep(0.05)
        else:
            logger.warning("No subscribers to /avs/cmd, reset command not sent.")
        
        delay = 1.0 / self.playback_rate_hz
        
        with self._lock:
            if not self.scenario or not self.scenario.frames:
                self.is_playing = False
                return
            frames_len = len(self.scenario.frames)
            self.current_frame_idx = 0
            
        metrics = {
            "selected_lane_switch_count": 0,
            "trajectory_kind_switch_count": 0,
            "replan_count": 0,
            "invalid_frame_count": 0,
            "jitter_epsilon_x_mm": 0.0,
            "jitter_theta_rad": 0.0,
            "control_source_count": {},
            "blocked_recovery_events": [],
        }

        last_lane_id = None
        last_traj_kind = None
        last_replan_reason = None
        last_epsilon_x = None
        last_theta = None
        last_blocked = None
        recovery_reasons = {"recovery", "persistent_invalid_clear", "persistent_low_confidence_deviation"}

        frame_results = []

        while self.is_playing and not self._stop_event.is_set():
            start_time = time.time()
            step_result = self.step()
            
            if step_result:
                outputs = step_result.get("outputs", {})
                lane_state = outputs.get("lane_state") or {}
                control_error = outputs.get("control_error") or {}
                
                if lane_state:
                    if not lane_state.get("trajectory_valid", False):
                        metrics["invalid_frame_count"] += 1
                    
                    replan_reason = lane_state.get("replan_reason", "none")
                    if replan_reason != last_replan_reason:
                        if last_replan_reason is not None:
                            metrics["replan_count"] += 1
                            if replan_reason in recovery_reasons:
                                metrics["blocked_recovery_events"].append({
                                    "frame_id": step_result.get("frame_id"),
                                    "event": "recovery",
                                    "replan_reason": replan_reason,
                                })
                        last_replan_reason = replan_reason

                    traj_kind = lane_state.get("trajectory_kind")
                    if traj_kind != last_traj_kind:
                        if last_traj_kind is not None:
                            metrics["trajectory_kind_switch_count"] += 1
                        last_traj_kind = traj_kind
                        
                    lane_id = lane_state.get("selected_lane_id")
                    if lane_id != last_lane_id:
                        if last_lane_id is not None:
                            metrics["selected_lane_switch_count"] += 1
                        last_lane_id = lane_id

                    control_source = lane_state.get("control_source")
                    if control_source is not None:
                        counts = metrics["control_source_count"]
                        counts[control_source] = counts.get(control_source, 0) + 1

                    blocked = lane_state.get("blocked_by_marking")
                    if blocked != last_blocked:
                        if last_blocked is not None:
                            metrics["blocked_recovery_events"].append({
                                "frame_id": step_result.get("frame_id"),
                                "event": "blocked_by_marking",
                                "from": last_blocked,
                                "to": blocked,
                            })
                        last_blocked = blocked

                if control_error:
                    epsilon_x = control_error.get("epsilon_x_mm")
                    if epsilon_x is not None:
                        if last_epsilon_x is not None:
                            jitter = abs(epsilon_x - last_epsilon_x)
                            metrics["jitter_epsilon_x_mm"] = max(metrics["jitter_epsilon_x_mm"], float(jitter))
                        last_epsilon_x = epsilon_x
                        
                    theta_rad = control_error.get("theta_rad")
                    if theta_rad is not None:
                        if last_theta is not None:
                            jitter = abs(theta_rad - last_theta)
                            metrics["jitter_theta_rad"] = max(metrics["jitter_theta_rad"], float(jitter))
                        last_theta = theta_rad
                
                frame_results.append(step_result)

            with self._lock:
                if self.current_frame_idx == 0:  # step() wrapped it to 0
                    break
                if not self.is_playing:  # step() reached the last frame
                    break

            elapsed = time.time() - start_time
            sleep_time = max(0.001, delay - elapsed)
            if self._stop_event.wait(sleep_time):
                break
                
        with self._lock:
            self.latest_report = {
                "metrics": metrics,
                "frames": frame_results
            }
            self.is_playing = False
            self.current_frame_idx = 0
            self._disable_debug_param()

    def get_report(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self.latest_report
