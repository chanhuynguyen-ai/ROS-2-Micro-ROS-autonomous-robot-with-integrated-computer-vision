#!/usr/bin/env python3
import sys
import os
import torch
import ultralytics.nn.tasks
from ultralytics import YOLO

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 export_ncnn.py <model_path.pt>")
        sys.exit(1)
        
    model_path = sys.argv[1]
    if not os.path.exists(model_path):
        print(f"Error: Model path '{model_path}' does not exist.")
        sys.exit(1)
        
    # Register custom module mapping if model was trained with custom class name
    class WeightedSegmentationModel(ultralytics.nn.tasks.SegmentationModel):
        pass

    sys.modules['__main__'].WeightedSegmentationModel = WeightedSegmentationModel

    # Patch torch.load for PyTorch 2.6+ compatibility with custom checkpoints
    orig_load = torch.load
    def safe_load(*args, **kwargs):
        if 'weights_only' in kwargs:
            kwargs['weights_only'] = False
        return orig_load(*args, **kwargs)
    torch.load = safe_load

    print(f"Loading model from {model_path}...")
    model = YOLO(model_path)
    
    print("Exporting model to NCNN format...")
    # This generates a folder model_name_ncnn_model/ containing model.ncnn.param and model.ncnn.bin
    output_path = model.export(format="ncnn", imgsz=320)
    print(f"Export complete. Model saved to: {output_path}")

if __name__ == "__main__":
    main()

