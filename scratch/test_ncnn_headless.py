import cv2
import os
from ultralytics import YOLO

def test_headless():
    model_path = "/home/goln/SimpleSysIDV/models/best_ncnn_model_int8"
    video_path = "/home/goln/SimpleSysIDV/test/test_video/video_test1.mp4"
    
    print(f"Loading NCNN INT8 model from: {model_path}")
    model = YOLO(model_path, task="segment")
    
    print(f"Opening video: {video_path}")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error: Could not open video file.")
        return
        
    # Read 5 frames and run inference
    for i in range(1, 6):
        ret, frame = cap.read()
        if not ret:
            print("Finished or could not read frame.")
            break
            
        print(f"\n--- Frame {i} ---")
        results = model(frame, verbose=False)
        
        # Check detected classes
        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                class_name = model.names[cls_id]
                print(f"  Detected: {class_name} (ID: {cls_id}) with confidence {conf:.2f}")
        else:
            print("  No objects detected in this frame.")
            
    cap.release()
    print("\nHeadless verification complete!")

if __name__ == "__main__":
    test_headless()
