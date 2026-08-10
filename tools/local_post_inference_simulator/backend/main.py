import os
import json
import logging
import subprocess
import time
from contextlib import asynccontextmanager
from typing import Dict, Any, List, Optional

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, Query, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .scenario_schema import ScenarioSchema
from .ros_scenario_runner import ScenarioRunner
from .ipm_adapter import CLASS_COLORS, render_bev_image
from .mask_to_objects import CLASS_MAPPING

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("avs_sim_main")

synthetic_node_process = None

def start_synthetic_node():
    global synthetic_node_process
    script_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ros2", "synthetic_inference_node.py")
    if not os.path.exists(script_path):
        logger.error(f"Could not find synthetic_inference_node.py at {script_path}")
        return
    synthetic_node_process = subprocess.Popen(["python3", script_path])
    # Readiness check: rclpy init can fail fast (missing ROS env, domain conflict, etc).
    # A short poll catches that instead of silently leaving no synthetic node running.
    time.sleep(0.5)
    if synthetic_node_process.poll() is not None:
        logger.error(
            f"synthetic_inference_node.py exited immediately (code {synthetic_node_process.returncode}). "
            "Synthetic telemetry will NOT be published."
        )
    else:
        logger.info(f"Started synthetic_inference_node.py (PID: {synthetic_node_process.pid})")

def stop_synthetic_node():
    global synthetic_node_process
    if synthetic_node_process:
        synthetic_node_process.terminate()
        synthetic_node_process.wait(timeout=2.0)
        logger.info("Stopped synthetic_inference_node.py")
        synthetic_node_process = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    start_synthetic_node()
    yield
    stop_synthetic_node()

app = FastAPI(title="AVS Local Post-Inference Simulator", lifespan=lifespan)

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global ScenarioRunner instance
runner = ScenarioRunner()

@app.post("/api/scenarios/load")
async def load_scenario(scenario: ScenarioSchema, mode: str = Query("direct", description="'direct' or 'rasterized'")):
    try:
        runner.load_scenario(scenario, mode=mode)
        return {"status": "ok", "message": f"Loaded scenario '{scenario.name}'", "runner": runner.get_status()}
    except Exception as e:
        logger.exception("Error loading scenario")
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/api/scenarios/play")
async def play_scenario():
    if not runner.scenario:
        raise HTTPException(status_code=400, detail="No scenario loaded.")
    runner.play()
    return {"status": "ok", "runner": runner.get_status()}

@app.get("/api/scenarios/status")
async def get_scenario_status():
    """Get the current scenario runner status."""
    return {"status": "ok", "runner": runner.get_status()}

@app.get("/api/scenarios/report")
async def get_scenario_report():
    """Get the latest run report metrics and frames."""
    report = runner.get_report()
    if not report:
        raise HTTPException(status_code=404, detail="No report available.")
    return {"status": "ok", "report": report}

@app.post("/api/scenarios/pause")
async def pause_scenario():
    runner.pause()
    return {"status": "ok", "runner": runner.get_status()}

@app.post("/api/scenarios/stop")
async def stop_scenario():
    runner.stop()
    return {"status": "ok", "runner": runner.get_status()}

@app.post("/api/scenarios/step")
async def step_scenario():
    """Run full pipeline: telemetry -> IPM -> Control Error."""
    if not runner.scenario:
        raise HTTPException(status_code=400, detail="No scenario loaded.")
    if not runner.scenario.frames:
        raise HTTPException(status_code=400, detail="Scenario has no frames.")
    result = runner.step()
    return {"status": "ok", "step_result": result, "runner": runner.get_status()}

@app.post("/api/scenarios/step_ipm")
async def step_ipm_scenario():
    """Run partial pipeline: telemetry -> IPM (no control)."""
    if not runner.scenario:
        raise HTTPException(status_code=400, detail="No scenario loaded.")
    if not runner.scenario.frames:
        raise HTTPException(status_code=400, detail="Scenario has no frames.")
    result = runner.step_ipm()
    return {"status": "ok", "step_result": result, "runner": runner.get_status()}

@app.get("/api/label_mapping")
async def get_label_mapping():
    """
    Single source of truth for label<->class_name<->color used by the frontend
    canvas editor, mirroring models/best_ncnn_model/metadata.yaml via CLASS_MAPPING.
    """
    labels = []
    for label, class_name in CLASS_MAPPING.items():
        color_bgr = CLASS_COLORS[label % len(CLASS_COLORS)]
        color_hex = "#{:02x}{:02x}{:02x}".format(color_bgr[2], color_bgr[1], color_bgr[0])
        labels.append({"label": label, "class_name": class_name, "color": color_hex})
    return {"status": "ok", "labels": labels}



@app.get("/api/scenarios/history")
async def get_history():
    return {
        "status": "ok",
        "history": runner.history
    }

@app.get("/api/ipm/latest")
async def get_latest_ipm():
    return {"status": "ok", **runner.get_latest_ipm_output()}

@app.get("/api/ipm/calibration")
async def get_ipm_calibration():
    return {"status": "ok", "calibration": runner.get_calibration_status()}

@app.get("/api/ipm/bev")
async def get_ipm_bev(
    width: int = Query(720, ge=240, le=2000),
    height: int = Query(520, ge=240, le=2000),
    show_slices: bool = Query(False),
    show_raw_midpoints: bool = Query(False),
    show_filtered_midpoints: bool = Query(False),
    show_candidate_trajectory: bool = Query(False),
    show_normalized_trajectory: bool = Query(False),
    show_committed_trajectory: bool = Query(False),
):
    latest = runner.get_latest_ipm_output()
    img = render_bev_image(
        latest.get("telemetry_realworld"),
        width=width,
        height=height,
        show_slices=show_slices,
        show_raw_midpoints=show_raw_midpoints,
        show_filtered_midpoints=show_filtered_midpoints,
        lane_state=latest.get("lane_state"),
        show_candidate_trajectory=show_candidate_trajectory,
        show_normalized_trajectory=show_normalized_trajectory,
        show_committed_trajectory=show_committed_trajectory,
    )
    _, jpeg_bytes = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return Response(content=jpeg_bytes.tobytes(), media_type="image/jpeg")

@app.get("/api/lane_state/latest")
async def get_latest_lane_state():
    """Raw latest /avs/lane_state payload (incl. debug_trajectories), for the
    frontend debug panel and for scripts that want to inspect trajectory
    stages without re-deriving them from the BEV image."""
    latest = runner.get_latest_ipm_output()
    return {"status": "ok", "lane_state": latest.get("lane_state")}

@app.get("/api/scenarios/preview")
async def get_preview():
    """
    Renders a live image showing the current frame objects drawn on a 2D canvas
    for visual verification of bounding boxes, labels, and polygons.
    """
    # Default blank frame
    w, h = 640, 480
    if runner.scenario:
        w = runner.scenario.canvas.width
        h = runner.scenario.canvas.height
    
    img = np.zeros((h, w, 3), dtype=np.uint8)
    
    if runner.scenario and runner.scenario.frames:
        frames = runner.scenario.frames
        # Use current frame (or wrap around if playing completed)
        idx = runner.current_frame_idx
        if idx >= len(frames):
            idx = 0
            
        frame = frames[idx]
        overlay = img.copy()
        
        for obj in frame.objects:
            label = obj.label
            class_name = obj.class_name
            points = obj.points_px
            shape = obj.shape
            
            color = CLASS_COLORS[label % len(CLASS_COLORS)]
            pts_array = np.array(points, dtype=np.int32)
            
            if len(pts_array) > 0:
                # Bounding box
                x, y, box_w, box_h = cv2.boundingRect(pts_array)
                cv2.rectangle(img, (x, y), (x + box_w, y + box_h), color, 1)
                
                # Fill polygon or draw polyline
                if shape == "polyline":
                    cv2.polylines(overlay, [pts_array], isClosed=False, color=color, thickness=6)
                else:
                    cv2.fillPoly(overlay, [pts_array], color)
                
                # Draw text
                text = f"{obj.id} ({class_name})"
                cv2.putText(img, text, (x, max(12, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Alpha blend the transparency overlay
        cv2.addWeighted(overlay, 0.4, img, 0.6, 0, img)
        
        # Add frame info text
        info_text = f"Scenario: {runner.scenario.name} | Frame {idx+1}/{len(frames)}"
        cv2.putText(img, info_text, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    else:
        cv2.putText(img, "No Scenario Loaded", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    _, jpeg_bytes = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return Response(content=jpeg_bytes.tobytes(), media_type="image/jpeg")

# Mount static frontend files if directory exists
frontend_dir = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "frontend"
)
if os.path.exists(frontend_dir):
    app.mount("/", StaticFiles(directory=frontend_dir, html=True), name="frontend")
    logger.info(f"Mounted static frontend files from: {frontend_dir}")
else:
    logger.warning(f"Frontend directory not found at: {frontend_dir}. API endpoints only.")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
