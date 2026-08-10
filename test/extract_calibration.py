import cv2
import os

video_path = "/home/goln/SimpleSysIDV/test/test_video/video_test1.mp4"
output_dir = "/home/goln/SimpleSysIDV/test/calibration_images"
output_list = "/home/goln/SimpleSysIDV/test/calibration_images.txt"

os.makedirs(output_dir, exist_ok=True)

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print(f"Error opening video {video_path}")
    exit(1)

total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
# We want to extract around 100 frames distributed evenly
step = max(1, total_frames // 100)

frame_idx = 0
saved_count = 0
image_paths = []

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    if frame_idx % step == 0 and saved_count < 100:
        # Resize to 320x320 to match model input exactly
        resized = cv2.resize(frame, (320, 320))
        img_name = f"frame_{saved_count:03d}.jpg"
        img_path = os.path.join(output_dir, img_name)
        cv2.imwrite(img_path, resized)
        image_paths.append(img_path)
        saved_count += 1
        
    frame_idx += 1

cap.release()

with open(output_list, "w") as f:
    for path in image_paths:
        f.write(path + "\n")

print(f"Extracted {saved_count} frames to {output_dir}")
print(f"Saved image list to {output_list}")
