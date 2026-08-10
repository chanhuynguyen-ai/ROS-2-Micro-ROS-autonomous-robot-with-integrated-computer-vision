"""Quick test to dump raw NCNN output tensor values for the first anchor."""
import numpy as np

# Use pyncnn if available, otherwise use ultralytics
try:
    import ncnn
    HAS_PYNCNN = True
except ImportError:
    HAS_PYNCNN = False

import cv2

def test_with_ultralytics():
    from ultralytics import YOLO
    import torch

    video = "/home/goln/SimpleSysIDV/test/test_video/video_test1.mp4"
    cap = cv2.VideoCapture(video)
    ret, frame = cap.read()
    cap.release()

    # Test FP16 NCNN model
    print("=== NCNN FP16 via Ultralytics ===")
    model = YOLO("/home/goln/SimpleSysIDV/models/best_ncnn_model", task="segment")
    results = model(frame, imgsz=320, verbose=False)
    r = results[0]
    if r.boxes is not None and len(r.boxes) > 0:
        for box in r.boxes:
            cls = int(box.cls)
            conf = float(box.conf)
            name = r.names[cls]
            print(f"  {name} (id={cls}): conf={conf:.4f}")
    
    # Test INT8 NCNN model
    print("\n=== NCNN INT8 via Ultralytics ===")
    model_int8 = YOLO("/home/goln/SimpleSysIDV/models/yolo26-best_ncnn_model_int8", task="segment")
    results_int8 = model_int8(frame, imgsz=320, verbose=False)
    r8 = results_int8[0]
    if r8.boxes is not None and len(r8.boxes) > 0:
        for box in r8.boxes:
            cls = int(box.cls)
            conf = float(box.conf)
            name = r8.names[cls]
            print(f"  {name} (id={cls}): conf={conf:.4f}")
    else:
        print("  No detections!")

    # Test PyTorch at 320
    print("\n=== PyTorch FP32 at imgsz=320 ===")
    model_pt = YOLO("/home/goln/SimpleSysIDV/models/best.pt")
    results_pt = model_pt(frame, imgsz=320, verbose=False)
    rp = results_pt[0]
    if rp.boxes is not None and len(rp.boxes) > 0:
        for box in rp.boxes:
            cls = int(box.cls)
            conf = float(box.conf)
            name = rp.names[cls]
            print(f"  {name} (id={cls}): conf={conf:.4f}")

test_with_ultralytics()
