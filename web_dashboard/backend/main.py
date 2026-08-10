import asyncio
import json
import logging
import os
import threading
from typing import Set
from datetime import datetime

import cv2
import numpy as np

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.responses import StreamingResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("avs_web_bridge")

app = FastAPI(title="AVS Perception Dashboard Bridge")

# Global variables
latest_jpeg_frame = None
latest_telemetry = {}
connected_clients: Set[WebSocket] = set()
loop = None
bridge_node = None

def load_config():
    paths = ["/workspace/config/config.json", "config/config.json"]
    for path in paths:
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    return json.load(f)
            except Exception as e:
                logger.error(f"Error reading config at {path}: {e}")
    return {}

def save_config(config_data):
    paths = ["/workspace/config/config.json", "config/config.json"]
    for path in paths:
        if os.path.exists(path) or path == "/workspace/config/config.json":
            try:
                existing = {}
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        existing = json.load(f)
                existing.update(config_data)
                with open(path, 'w') as f:
                    json.dump(existing, f, indent=2)
                logger.info(f"Saved config to {path}")
                break
            except Exception as e:
                logger.error(f"Error writing config to {path}: {e}")

from rclpy.qos import qos_profile_sensor_data

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')
        
        # Subscribe to compressed raw camera image using sensor data QoS (best-effort)
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )
        
        # Subscribe to telemetry JSON data (prefer realworld if available)
        self.telemetry_sub = self.create_subscription(
            String,
            '/avs/telemetry_realworld',
            self.telemetry_callback,
            10
        )

        # Subscribe to control and lane state (for active trajectory visualization)
        self.lane_state_sub = self.create_subscription(
            String,
            '/avs/lane_state',
            self.lane_state_callback,
            10
        )

        # Subscribe to control errors (for target lookahead point visualization)
        self.control_error_sub = self.create_subscription(
            String,
            '/avs/control_error',
            self.control_error_callback,
            10
        )
        
        # Clients for setting parameters
        self.param_client = self.create_client(
            SetParameters,
            '/ncnn_inference_node/set_parameters'
        )
        self.pub_param_client = self.create_client(
            SetParameters,
            '/video_publisher_node/set_parameters'
        )

        # Publisher for ARM / DISARM commands to the third-party controller
        self.avs_cmd_pub_ = self.create_publisher(String, '/avs/cmd', 10)

        # Publisher for Route Intent commands
        self.route_intent_pub_ = self.create_publisher(String, '/avs/route_intent', 10)
        self.route_intent_seq = 0

        logger.info("ROS2 WebBridgeNode initialized.")
        logger.info("Subscribed to /camera/image_raw/compressed, /avs/telemetry_realworld, /avs/lane_state, and /avs/control_error")

    def image_callback(self, msg):
        global latest_jpeg_frame
        latest_jpeg_frame = bytes(msg.data)

    def telemetry_callback(self, msg):
        global latest_telemetry
        try:
            data = json.loads(msg.data)
            # Update the global dict to preserve control/trajectory data
            latest_telemetry.update(data)
            
            # Broadcast to all connected WebSocket clients
            if connected_clients and loop:
                for client in list(connected_clients):
                    asyncio.run_coroutine_threadsafe(
                        client.send_json(latest_telemetry),
                        loop
                    )
        except Exception as e:
            logger.error(f"Error parsing/broadcasting telemetry: {e}")

    def lane_state_callback(self, msg):
        global latest_telemetry
        try:
            data = json.loads(msg.data)
            # Merge into latest_telemetry so both perception and control data are available
            latest_telemetry.update(data)
            
            # Broadcast to all connected WebSocket clients
            if connected_clients and loop:
                for client in list(connected_clients):
                    asyncio.run_coroutine_threadsafe(
                        client.send_json(latest_telemetry),
                        loop
                    )
        except Exception as e:
            logger.error(f"Error parsing/broadcasting lane_state: {e}")

    def control_error_callback(self, msg):
        global latest_telemetry
        try:
            data = json.loads(msg.data)
            # Merge into latest_telemetry to get the target lookahead point coordinates
            latest_telemetry.update(data)
            
            # Broadcast to all connected WebSocket clients
            if connected_clients and loop:
                for client in list(connected_clients):
                    asyncio.run_coroutine_threadsafe(
                        client.send_json(latest_telemetry),
                        loop
                    )
        except Exception as e:
            logger.error(f"Error parsing/broadcasting control_error: {e}")

# ROS2 background runner thread
def run_ros2():
    rclpy.init()
    global bridge_node
    bridge_node = WebBridgeNode()
    try:
        rclpy.spin(bridge_node)
    except Exception as e:
        logger.error(f"ROS2 Spin Exception: {e}")
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

async def apply_config_on_startup():
    # Wait for nodes to spin up and discover each other
    await asyncio.sleep(3.0)
    config = load_config()
    if not config:
        logger.warning("No config.json found to apply on startup.")
        return
        
    mode = config.get("mode", "camera")
    prob_threshold = config.get("prob_threshold", 0.25)
    nms_threshold = config.get("nms_threshold", 0.45)
    
    if mode == "camera":
        video_path = config.get("camera_device", "/dev/video_source")
    else:
        video_path = config.get("video_path", "/workspace/test/test_video/video_test1.mp4")
        if not os.path.exists(video_path):
            alt_path = f"test/test_video/{os.path.basename(video_path)}"
            if os.path.exists(alt_path):
                video_path = os.path.abspath(alt_path)

    logger.info(f"Applying startup config: Mode={mode}, Source={video_path}, ProbThreshold={prob_threshold}, NMSThreshold={nms_threshold}")
    
    if bridge_node is not None:
        if bridge_node.param_client.wait_for_service(timeout_sec=3.0):
            req = SetParameters.Request()
            req.parameters = [
                Parameter(name="prob_threshold", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=prob_threshold)),
                Parameter(name="nms_threshold", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=nms_threshold))
            ]
            bridge_node.param_client.call_async(req)
            logger.info("Startup inference thresholds set successfully.")
        else:
            logger.warning("Inference parameter service offline during startup config application.")

        if bridge_node.pub_param_client.wait_for_service(timeout_sec=3.0):
            req = SetParameters.Request()
            req.parameters = [
                Parameter(name="video_path", value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=video_path))
            ]
            bridge_node.pub_param_client.call_async(req)
            logger.info(f"Startup video/camera publisher source set to: {video_path}")
        else:
            logger.warning("Video publisher parameter service offline during startup config application.")

@app.on_event("startup")
async def startup_event():
    global loop
    loop = asyncio.get_running_loop()
    # Start ROS2 spinning in a daemon thread
    threading.Thread(target=run_ros2, daemon=True).start()
    logger.info("FastAPI backend started, ROS2 thread spawned.")
    # Apply configurations asynchronously
    asyncio.create_task(apply_config_on_startup())

# Root redirect to UI
@app.get("/")
async def root():
    return RedirectResponse(url="/index.html")

# Color palette for segmentation overlay (matching C++ color scheme - BGR format)
CLASS_COLORS = [
    (255, 0, 0),      # dashed-white: Blue
    (0, 165, 255),    # dashed-yellow: Orange
    (255, 127, 0),    # double-solid-white: Light Blue
    (120, 200, 0),    # light_green: Sea Green
    (80, 80, 255),    # light_red: Salmon Red
    (128, 255, 255),  # light_yellow: Pale Yellow
    (0, 255, 0),      # main-lane: Green
    (0, 0, 255),      # other-lane: Red
    (128, 128, 128),  # parking-zone: Gray
    (60, 20, 220),    # sign-no-left: Crimson
    (0, 0, 180),      # sign-no-parking: Bright Crimson
    (50, 50, 150),    # sign-no-right: Dark Red
    (230, 100, 50),   # sign-parking: Royal Blue
    (0, 0, 255),      # sign-stop: Stop Red
    (235, 206, 135),  # sign-turn-left: Sky Blue
    (180, 130, 70),   # sign-turn-right: Steel Blue
    (255, 255, 0),    # solid-white: Cyan
    (0, 255, 255),    # solid-yellow: Yellow
    (0, 255, 127),    # start: Spring Green
    (0, 0, 128),      # stop-line: Navy
    (127, 0, 255),    # turn-lane: Purple
    (255, 0, 255)     # vehicle: Magenta
]

CLASS_NAMES = [
    "dashed-white", "dashed-yellow", "double-solid-white", "light_green",
    "light_red", "light_yellow", "main-lane", "other-lane",
    "parking-zone", "sign-no-left", "sign-no-parking", "sign-no-right",
    "sign-parking", "sign-stop", "sign-turn-left", "sign-turn-right",
    "solid-white", "solid-yellow", "start", "stop-line",
    "turn-lane", "vehicle"
]

def process_frame(jpeg_frame, telemetry):
    if jpeg_frame is None:
        return None
    # Decode JPEG frame to OpenCV image
    nparr = np.frombuffer(jpeg_frame, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    if img is not None:
        if telemetry and "objects" in telemetry:
            overlay = img.copy()
            for obj in telemetry["objects"]:
                label = obj.get("label", 0)
                prob = obj.get("prob", 0.0)
                box = obj.get("box", [0, 0, 0, 0])
                polygons = obj.get("polygons", [])
                
                color = CLASS_COLORS[label % len(CLASS_COLORS)]
                
                # Draw transparency mask polygons
                for poly in polygons:
                    pts = np.array(poly, dtype=np.int32)
                    if len(pts) > 0:
                        cv2.fillPoly(overlay, [pts], color)
                
                # Draw bounding box
                x, y, w, h = map(int, box)
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                
                # Draw text label
                text = f"{CLASS_NAMES[label]} {prob*100:.1f}%"
                label_size, base_line = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                label_w, label_h = label_size
                
                ty = y - label_h - 2
                if ty < 0: ty = 0
                tx = x
                if tx + label_w > img.shape[1]: tx = img.shape[1] - label_w
                
                cv2.rectangle(img, (tx, ty), (tx + label_w, ty + label_h + base_line), color, -1)
                cv2.putText(img, text, (tx, ty + label_h), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            # Blend the transparency mask overlay
            cv2.addWeighted(overlay, 0.4, img, 0.6, 0, img)
        
        # Re-encode to JPEG
        _, jpeg_bytes = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpeg_bytes.tobytes()
    return jpeg_frame

# Live MJPEG Stream Endpoint with Real-Time Overlay Rendering
async def frame_generator():
    global latest_jpeg_frame, latest_telemetry
    logger.info("MJPEG stream connection opened.")
    
    last_processed_frame = None
    
    try:
        while True:
            if latest_jpeg_frame is not None and latest_jpeg_frame != last_processed_frame:
                last_processed_frame = latest_jpeg_frame
                
                # Run the heavy OpenCV processing in a separate thread pool to keep the event loop responsive
                frame_data = await asyncio.to_thread(process_frame, latest_jpeg_frame, latest_telemetry)
                
                if frame_data is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
            # Check for new frames at ~30 FPS
            await asyncio.sleep(0.033)
    except asyncio.CancelledError:
        logger.info("MJPEG stream connection cancelled.")
    except Exception as e:
        logger.error(f"Error in MJPEG stream: {e}")

@app.get("/api/stream")
async def get_stream():
    return StreamingResponse(
        frame_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

# WebSocket Endpoint for Telemetry
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.add(websocket)
    logger.info(f"WebSocket client connected. Total clients: {len(connected_clients)}")
    
    # Send current latest telemetry immediately
    if latest_telemetry:
        await websocket.send_json(latest_telemetry)
        
    try:
        while True:
            # We don't expect messages from the client on this socket, 
            # but we read to detect disconnection
            await websocket.receive_text()
    except WebSocketDisconnect:
        connected_clients.remove(websocket)
        logger.info(f"WebSocket client disconnected. Total clients: {len(connected_clients)}")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        if websocket in connected_clients:
            connected_clients.remove(websocket)

# API to get current config
@app.get("/api/config")
async def get_config():
    return load_config()

# API to switch running modes (camera vs video)
@app.post("/api/arm")
async def arm_vehicle(cmd: str = Query(..., description="'arm' or 'disarm'")):
    if cmd not in ["arm", "disarm"]:
        return {"status": "error", "message": "Invalid cmd. Use 'arm' or 'disarm'"}
    if bridge_node is None:
        return {"status": "error", "message": "ROS2 bridge node not ready"}
    try:
        from std_msgs.msg import String as StringMsg
        msg = StringMsg()
        msg.data = f'{{"cmd": "{cmd}"}}'
        bridge_node.avs_cmd_pub_.publish(msg)
        logger.info(f"Published /avs/cmd: {msg.data}")
        return {"status": "ok", "cmd": cmd}
    except Exception as e:
        logger.error(f"Error publishing arm cmd: {e}")
        return {"status": "error", "message": str(e)}

@app.post("/api/route_intent")
async def set_route_intent(intent: str = Query(..., description="Route intent: 'follow_main', 'turn_left', 'turn_right', 'lane_change_left', 'lane_change_right'")):
    if intent not in ["follow_main", "turn_left", "turn_right", "lane_change_left", "lane_change_right"]:
        return {"status": "error", "message": "Invalid intent"}
    if bridge_node is None:
        return {"status": "error", "message": "ROS2 bridge node not ready"}
    try:
        bridge_node.route_intent_seq += 1
        payload = {
            "intent": intent,
            "source": "manual",
            "seq": bridge_node.route_intent_seq
        }
        from std_msgs.msg import String as StringMsg
        msg = StringMsg()
        msg.data = json.dumps(payload)
        bridge_node.route_intent_pub_.publish(msg)
        logger.info(f"Published /avs/route_intent: {msg.data}")
        return {"status": "ok", "intent": intent, "seq": bridge_node.route_intent_seq}
    except Exception as e:
        logger.error(f"Error publishing route intent: {e}")
        return {"status": "error", "message": str(e)}

@app.post("/api/mode")
async def change_mode(mode: str = Query(..., description="Run mode: 'camera' or 'video'")):
    if mode not in ["camera", "video"]:
        return {"status": "error", "message": "Invalid mode. Use 'camera' or 'video'"}
        
    config = load_config()
    camera_device = config.get("camera_device", "/dev/video_source")
    video_path = config.get("video_path", "/workspace/test/test_video/video_test1.mp4")
    
    target_source = camera_device if mode == "camera" else video_path
    
    # Check fallback for local dev
    if mode == "video" and not os.path.exists(target_source):
        alt_path = f"test/test_video/{os.path.basename(target_source)}"
        if os.path.exists(alt_path):
            target_source = os.path.abspath(alt_path)

    if bridge_node is None:
        return {"status": "error", "message": "ROS2 node not initialized"}
        
    if not bridge_node.pub_param_client.wait_for_service(timeout_sec=1.0):
        return {"status": "error", "message": "Video publisher parameter service not online"}

    req = SetParameters.Request()
    params_list = [
        Parameter(name="video_path", value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=target_source))
    ]
    
    # When switching to camera mode, also send V4L2 capture settings
    if mode == "camera":
        cam_w = config.get("camera_width", 640)
        cam_h = config.get("camera_height", 480)
        cam_fps = config.get("camera_fps", 30)
        params_list.extend([
            Parameter(name="camera_width", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=cam_w)),
            Parameter(name="camera_height", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=cam_h)),
            Parameter(name="camera_fps", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=cam_fps)),
        ])

    req.parameters = params_list
    bridge_node.pub_param_client.call_async(req)
    save_config({"mode": mode})
    logger.info(f"Requested mode change to: {mode} (source: {target_source})")
    return {"status": "success", "mode": mode, "source": target_source}

# API to update settings (Thresholds) dynamically
@app.post("/api/settings")
async def update_settings(
    prob_threshold: float = Query(..., description="Probability threshold (0.0 to 1.0)"),
    nms_threshold: float = Query(..., description="NMS threshold (0.0 to 1.0)")
):
    if bridge_node is None:
        return {"status": "error", "message": "ROS2 node not initialized"}
        
    if not bridge_node.param_client.wait_for_service(timeout_sec=1.0):
        return {"status": "error", "message": "Inference parameter service not online"}

    req = SetParameters.Request()
    req.parameters = [
        Parameter(name="prob_threshold", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=prob_threshold)),
        Parameter(name="nms_threshold", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=nms_threshold))
    ]
    
    # Non-blocking call
    bridge_node.param_client.call_async(req)
    save_config({
        "prob_threshold": prob_threshold,
        "nms_threshold": nms_threshold
    })
    logger.info(f"Requested parameter change: prob_threshold={prob_threshold}, nms_threshold={nms_threshold}")
    return {"status": "success", "prob_threshold": prob_threshold, "nms_threshold": nms_threshold}

# API to change video sources (switches test videos)
@app.post("/api/source")
async def change_source(video_name: str = Query(..., description="Name of video file, e.g. video_test2.mp4")):
    if bridge_node is None:
        return {"status": "error", "message": "ROS2 node not initialized"}
        
    video_path = f"/workspace/test/test_video/{video_name}"
    # Verify file existence in workspace (using host mounts mapped in container)
    if not os.path.exists(video_path):
        # Also check relative to local paths if running outside docker
        alt_path = f"test/test_video/{video_name}"
        if os.path.exists(alt_path):
            video_path = os.path.abspath(alt_path)
        else:
            return {"status": "error", "message": f"Video file not found: {video_name}"}

    if not bridge_node.pub_param_client.wait_for_service(timeout_sec=1.0):
        return {"status": "error", "message": "Video publisher parameter service not online"}

    req = SetParameters.Request()
    req.parameters = [
        Parameter(name="video_path", value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=video_path))
    ]
    
    bridge_node.pub_param_client.call_async(req)
    save_config({
        "mode": "video",
        "video_path": video_path
    })
    logger.info(f"Requested video source change to: {video_path}")
    return {"status": "success", "video_path": video_path}

# API to get current calibration
@app.get("/api/calibration")
async def get_calibration():
    calib_path = "/workspace/config/calibration.json"
    if not os.path.exists(calib_path):
        calib_path = "config/calibration.json"
    if os.path.exists(calib_path):
        try:
            with open(calib_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"Error reading calibration file: {e}")
            return {"status": "error", "message": str(e)}
    return {"status": "error", "message": "No calibration file found."}

# API to save new calibration (calculates Homography matrix H)
@app.post("/api/calibration")
async def save_calibration(data: dict):
    pixel_points = data.get("pixel_points")
    world_points = data.get("world_points")
    if not pixel_points or not world_points or len(pixel_points) != 4 or len(world_points) != 4:
        return {"status": "error", "message": "Invalid points. Must provide exactly 4 pairs of pixel and world points."}
    try:
        src = np.float32(pixel_points)
        dst = np.float32(world_points)
        H = cv2.getPerspectiveTransform(src, dst)
        
        calib_data = {
            "homography_matrix": H.tolist(),
            "pixel_points": pixel_points,
            "world_points": world_points,
            "image_size": data.get("image_size", [640, 480]),
            "calibrated_at": datetime.now().isoformat()
        }
        
        calib_path = "/workspace/config/calibration.json"
        if not os.path.exists("/workspace/config"):
            os.makedirs("config", exist_ok=True)
            calib_path = "config/calibration.json"
            
        with open(calib_path, 'w') as f:
            json.dump(calib_data, f, indent=2)
            
        logger.info(f"Saved calibration matrix H to {calib_path}")
        
        return {"status": "success", "homography_matrix": H.tolist()}
    except Exception as e:
        logger.error(f"Error computing homography matrix: {e}")
        return {"status": "error", "message": str(e)}

# API to get latest calibration snapshot image
@app.get("/api/calibration/frame")
async def get_calibration_frame():
    global latest_jpeg_frame
    if latest_jpeg_frame is not None:
        from fastapi import Response
        return Response(content=latest_jpeg_frame, media_type="image/jpeg")
    else:
        # Fallback black frame if stream not running
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "No camera stream available", (120, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        _, jpeg_bytes = cv2.imencode('.jpg', img)
        from fastapi import Response
        return Response(content=jpeg_bytes.tobytes(), media_type="image/jpeg")

# Mount static files (Frontend UI)
frontend_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "frontend")
if os.path.exists(frontend_dir):
    app.mount("/", StaticFiles(directory=frontend_dir), name="frontend")
    logger.info(f"Mounted static frontend from {frontend_dir}")
else:
    logger.warning(f"Frontend directory not found at: {frontend_dir}")

if __name__ == "__main__":
    import uvicorn
    # Start web server
    uvicorn.run(app, host="0.0.0.0", port=8000)
