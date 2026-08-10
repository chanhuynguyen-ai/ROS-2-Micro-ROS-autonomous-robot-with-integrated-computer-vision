import os
import json
import cv2
import numpy as np
from typing import List, Dict, Any

# Load class mapping
DEFAULT_CLASS_NAMES = [
    "dashed-white", "dashed-yellow", "double-solid-white", "light_green",
    "light_red", "light_yellow", "main-lane", "other-lane",
    "parking-zone", "sign-no-left", "sign-no-parking", "sign-no-right",
    "sign-parking", "sign-stop", "sign-turn-left", "sign-turn-right",
    "solid-white", "solid-yellow", "start", "stop-line",
    "turn-lane", "vehicle"
]

def load_class_mapping() -> Dict[int, str]:
    mapping_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
        "config",
        "label_mapping.json"
    )
    if os.path.exists(mapping_path):
        try:
            with open(mapping_path, 'r') as f:
                mapping = json.load(f)
            return {int(k): v for k, v in mapping.items()}
        except Exception as e:
            print(f"Error loading label_mapping.json: {e}")
    return {idx: name for idx, name in enumerate(DEFAULT_CLASS_NAMES)}

CLASS_MAPPING = load_class_mapping()
CLASS_NAMES = list(CLASS_MAPPING.values())

def convert_to_telemetry_payload(
    frame_objects: List[Any], 
    width: int = 640, 
    height: int = 480, 
    mode: str = "direct"
) -> Dict[str, Any]:
    if mode not in ("direct", "rasterized"):
        raise ValueError(f"Invalid mode '{mode}'. Must be 'direct' or 'rasterized'")

    telemetry_objects = []
    detections = {name: 0 for name in CLASS_NAMES}

    for obj in frame_objects:
        class_name = obj.class_name
        label = obj.label
        obj_id = obj.id
        points_px = obj.points_px
        confidence = obj.confidence
        shape = obj.shape

        if not points_px:
            continue

        # Increment detection count for this class name
        if class_name in detections:
            detections[class_name] += 1

        if mode == "direct":
            # Direct Mode: Use the coordinates directly as the polygon(s).
            # polygons_px (captured multi-contour objects) takes precedence so
            # replay matches the real inference payload contour-for-contour.
            all_polygons = obj.polygons_px if getattr(obj, "polygons_px", None) else [points_px]
            all_pts = np.array([p for poly in all_polygons for p in poly], dtype=np.int32)
            if len(all_pts) > 0:
                x, y, w, h = cv2.boundingRect(all_pts)
                box = [float(x), float(y), float(w), float(h)]
            else:
                box = [0.0, 0.0, 0.0, 0.0]

            telemetry_objects.append({
                "label": int(label),
                "prob": float(confidence),
                "id": str(obj_id),
                "track_id": str(obj_id),
                "class_name": str(class_name),
                "box": box,
                "polygons": all_polygons
            })

        elif mode == "rasterized":
            # Rasterized Mode: draw polygon(s) on mask, find contours, and compute bounding box
            mask = np.zeros((height, width), dtype=np.uint8)
            source_polygons = obj.polygons_px if getattr(obj, "polygons_px", None) else [points_px]
            pts_arrays = [np.array(p, dtype=np.int32) for p in source_polygons if p]

            if pts_arrays:
                if shape == "polyline":
                    # Draw a thick line for polylines so it can be extracted
                    cv2.polylines(mask, pts_arrays, isClosed=False, color=255, thickness=8)
                else:
                    cv2.fillPoly(mask, pts_arrays, color=255)

                x, y, w, h = cv2.boundingRect(np.concatenate(pts_arrays))
                box = [float(x), float(y), float(w), float(h)]
            else:
                box = [0.0, 0.0, 0.0, 0.0]

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            polygons = []
            
            if contours:
                # Merge contours bounding box
                all_pts = []
                for c in contours:
                    # Simplify contour to reduce payload size like CHAIN_APPROX_SIMPLE
                    poly_points = [[float(pt[0][0]), float(pt[0][1])] for pt in c]
                    if len(poly_points) >= 3:
                        polygons.append(poly_points)
                    all_pts.extend(c)
            
            # If no valid contours were extracted (e.g. drawn off-screen), fallback to direct mode coordinates
            if not polygons:
                polygons = [points_px]

            telemetry_objects.append({
                "label": int(label),
                "prob": float(confidence),
                "id": str(obj_id),
                "track_id": str(obj_id),
                "class_name": str(class_name),
                "box": box,
                "polygons": polygons
            })

    return {
        "detections": detections,
        "objects": telemetry_objects
    }
