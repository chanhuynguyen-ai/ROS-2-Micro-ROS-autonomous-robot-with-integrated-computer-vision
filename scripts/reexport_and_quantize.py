#!/usr/bin/env python3
"""
Automated NCNN INT8 Mixed-Precision Quantization Pipeline.

Strategy: Keep the backbone in INT8 for speed, but keep ALL detection heads,
classification heads, mask coefficient heads, box regression heads,
and prototype mask heads in FP16/FP32 for accuracy.
"""
import os
import subprocess
import sys
import cv2

# ─── Absolute Paths ────────────────────────────────────────────────────────────
WORKSPACE = "/home/goln/SimpleSysIDV"
PYTHON_ENV = os.path.join(WORKSPACE, ".venv/bin/python")
NCNN_TOOLS_DIR = os.path.join(WORKSPACE, "ncnn-src/build/tools")
QUANTIZE_DIR = os.path.join(NCNN_TOOLS_DIR, "quantize")

MODEL_PT = os.path.join(WORKSPACE, "models/best.pt")
MODEL_NCNN_DIR = os.path.join(WORKSPACE, "models/best_ncnn_model")
MODEL_INT8_DIR = os.path.join(WORKSPACE, "models/best_ncnn_model_int8")

CALIBRATION_DIR = os.path.join(WORKSPACE, "test/calibration_images")
CALIBRATION_TXT = os.path.join(WORKSPACE, "test/calibration_images.txt")
VIDEO_PATH = os.path.join(WORKSPACE, "test/test_video/video_test1.mp4")

# ─── Layers to EXCLUDE from INT8 (keep in FP16/FP32) ──────────────────────────
# These are ALL layers in the detection/segmentation heads, including
# intermediate DepthWise convolutions and 1x1 projections.
# Identified by tracing the NCNN graph backward from out0 and out1.
EXCLUDE_LAYERS = [
    # === Box Regression Heads (3 scales: P3, P4, P5) ===
    # Scale P3 (conv_81 -> softmax -> conv_83)
    "conv_81", "conv_82", "conv_83",
    # Scale P4 (conv_84 -> softmax -> conv_86)
    "conv_84", "conv_85", "conv_86",
    # Scale P5 (conv_87 -> softmax -> conv_89)
    "conv_87", "conv_88", "conv_89",

    # === Classification Heads (3 scales: P3, P4, P5) ===
    # Each scale: ConvDW -> Conv1x1 -> ConvDW -> Conv1x1 -> Conv_cls (19 outputs)
    # Scale P3
    "convdw_245", "conv_90", "convdw_246", "conv_91", "conv_92",
    # Scale P4
    "convdw_247", "conv_93", "convdw_248", "conv_94", "conv_95",
    # Scale P5
    "convdw_249", "conv_96", "convdw_250", "conv_97", "conv_98",

    # === Mask Coefficient Heads (3 scales) ===
    # Scale P3
    "conv_99", "conv_100", "conv_101",
    # Scale P4
    "conv_102", "conv_103", "conv_104",
    # Scale P5
    "conv_105", "conv_106", "conv_107",

    # === Prototype Mask Head (Proto) ===
    "conv_108", "conv_109", "conv_110", "conv_111",
    "deconv_114", "conv_112", "conv_113",
]


def run_cmd(cmd, desc=""):
    """Run a command and exit on failure."""
    label = desc or " ".join(cmd) if isinstance(cmd, list) else cmd
    print(f"\n▶ {label}")
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode != 0:
        print(f"✗ FAILED!\nSTDOUT:\n{res.stdout}\nSTDERR:\n{res.stderr}")
        sys.exit(res.returncode)
    print(f"  ✓ Done")
    return res.stdout


def step1_export():
    """Re-export PyTorch model to NCNN via Ultralytics (PNNX path)."""
    print("\n" + "=" * 60)
    print("STEP 1: Exporting PyTorch → NCNN (imgsz=320)")
    print("=" * 60)
    run_cmd([
        PYTHON_ENV, "-c",
        f"from ultralytics import YOLO; model = YOLO('{MODEL_PT}'); model.export(format='ncnn', imgsz=320)"
    ], "Ultralytics NCNN export")


def step2_optimize():
    """Fuse layers with ncnnoptimize."""
    print("\n" + "=" * 60)
    print("STEP 2: Running ncnnoptimize")
    print("=" * 60)
    run_cmd([
        os.path.join(NCNN_TOOLS_DIR, "ncnnoptimize"),
        os.path.join(MODEL_NCNN_DIR, "model.ncnn.param"),
        os.path.join(MODEL_NCNN_DIR, "model.ncnn.bin"),
        os.path.join(MODEL_NCNN_DIR, "model-opt.param"),
        os.path.join(MODEL_NCNN_DIR, "model-opt.bin"),
        "0"
    ], "ncnnoptimize")


def step3_extract_calibration():
    """Use the user-provided calibration images from test/calibration_images."""
    print("\n" + "=" * 60)
    print("STEP 3: Preparing calibration images from test/calibration_images")
    print("=" * 60)

    if not os.path.exists(CALIBRATION_DIR):
        print(f"✗ Calibration directory not found: {CALIBRATION_DIR}")
        sys.exit(1)

    # Collect all image paths (jpg, jpeg, png, bmp) in the calibration directory
    valid_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".JPG", ".JPEG", ".PNG", ".BMP")
    image_paths = []
    for fname in os.listdir(CALIBRATION_DIR):
        if fname.endswith(valid_extensions):
            full_path = os.path.abspath(os.path.join(CALIBRATION_DIR, fname))
            image_paths.append(full_path)

    # Sort to ensure deterministic calibration behavior
    image_paths.sort()

    if not image_paths:
        print(f"✗ No calibration images found in: {CALIBRATION_DIR}")
        sys.exit(1)

    # Write the absolute paths to the calibration text file
    with open(CALIBRATION_TXT, "w") as f:
        for p in image_paths:
            f.write(p + "\n")

    print(f"  ✓ Found {len(image_paths)} calibration images in {CALIBRATION_DIR}")
    print(f"  ✓ Wrote paths to {CALIBRATION_TXT}")



def step4_generate_table():
    """Generate calibration table via ncnn2table with KL divergence."""
    print("\n" + "=" * 60)
    print("STEP 4: Generating calibration table (ncnn2table)")
    print("=" * 60)

    table_path = os.path.join(MODEL_NCNN_DIR, "model.table")
    run_cmd([
        os.path.join(QUANTIZE_DIR, "ncnn2table"),
        os.path.join(MODEL_NCNN_DIR, "model-opt.param"),
        os.path.join(MODEL_NCNN_DIR, "model-opt.bin"),
        CALIBRATION_TXT,
        table_path,
        "mean=[0,0,0]",
        "norm=[0.00392157,0.00392157,0.00392157]",
        "shape=[320,320,3]",
        "pixel=RGB",
        "thread=8",
        "method=kl"
    ], "ncnn2table (KL divergence, 500 images)")


def step5_create_mixed_table():
    """Comment out all head layer entries for mixed-precision quantization."""
    print("\n" + "=" * 60)
    print("STEP 5: Creating mixed-precision calibration table")
    print("=" * 60)

    table_path = os.path.join(MODEL_NCNN_DIR, "model.table")
    mixed_path = os.path.join(MODEL_NCNN_DIR, "model-mixed.table")

    if not os.path.exists(table_path):
        print(f"✗ Table not found: {table_path}")
        sys.exit(1)

    # Build a set for exact matching to avoid prefix collisions
    # e.g. "conv_10" should NOT match "conv_100"
    exclude_weight_keys = set()
    exclude_activation_keys = set()
    for layer in EXCLUDE_LAYERS:
        exclude_weight_keys.add(f"{layer}_param_0")
        exclude_activation_keys.add(layer)

    commented = 0
    total = 0
    with open(table_path, "r") as fin, open(mixed_path, "w") as fout:
        for line in fin:
            stripped = line.strip()
            if not stripped:
                fout.write(line)
                continue

            total += 1
            key = stripped.split()[0]

            if key in exclude_weight_keys or key in exclude_activation_keys:
                fout.write(f"# {line}")
                commented += 1
            else:
                fout.write(line)

    print(f"  ✓ Created: {mixed_path}")
    print(f"  ✓ Commented out {commented}/{total} entries ({len(EXCLUDE_LAYERS)} layers excluded)")
    print(f"  ✓ Remaining INT8 layers: {total - commented}")


def step6_quantize():
    """Run ncnn2int8 with the mixed-precision table."""
    print("\n" + "=" * 60)
    print("STEP 6: Quantizing to INT8 (mixed-precision)")
    print("=" * 60)

    os.makedirs(MODEL_INT8_DIR, exist_ok=True)
    mixed_path = os.path.join(MODEL_NCNN_DIR, "model-mixed.table")

    run_cmd([
        os.path.join(QUANTIZE_DIR, "ncnn2int8"),
        os.path.join(MODEL_NCNN_DIR, "model-opt.param"),
        os.path.join(MODEL_NCNN_DIR, "model-opt.bin"),
        os.path.join(MODEL_INT8_DIR, "model.ncnn.param"),
        os.path.join(MODEL_INT8_DIR, "model.ncnn.bin"),
        mixed_path
    ], "ncnn2int8 (mixed precision)")


def step7_copy_metadata():
    """Copy metadata.yaml from FP16 model to INT8 output directory.
    
    Ultralytics requires this file to know the correct imgsz, class names,
    and task type. Without it, inference defaults to imgsz=640 which causes
    garbage output on a model exported at imgsz=320.
    """
    import shutil
    print("\n" + "=" * 60)
    print("STEP 7: Copying metadata.yaml to INT8 model directory")
    print("=" * 60)

    src = os.path.join(MODEL_NCNN_DIR, "metadata.yaml")
    dst = os.path.join(MODEL_INT8_DIR, "metadata.yaml")

    if os.path.exists(src):
        shutil.copy2(src, dst)
        print(f"  ✓ Copied {src} → {dst}")
    else:
        print(f"  ⚠ Warning: {src} not found. Ultralytics may use wrong imgsz!")


def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║   NCNN INT8 Mixed-Precision Quantization Pipeline v2    ║")
    print("╚══════════════════════════════════════════════════════════╝")

    step1_export()
    step2_optimize()
    step3_extract_calibration()
    step4_generate_table()
    step5_create_mixed_table()
    step6_quantize()
    step7_copy_metadata()

    print("\n" + "=" * 60)
    print("✓ PIPELINE COMPLETE")
    print(f"✓ INT8 model saved to: {MODEL_INT8_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
