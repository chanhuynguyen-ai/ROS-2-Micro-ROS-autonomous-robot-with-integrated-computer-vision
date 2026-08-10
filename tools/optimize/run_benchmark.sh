#!/bin/bash

# AVS Benchmark Harness Wrapper
# Gathers system metadata and runs the benchmark script

OUTPUT_DIR="docs/optimize/results"
FRAMES=300
DURATION=0
WARMUP=30
WARMUP_TIMEOUT=90
export RUN_ID=$(date +"%Y%m%d_%H%M%S")
export MODEL="unknown"
export MODE="unknown"
export INPUT_SOURCE="unknown"
export RESOLUTION="unknown"
export NOTES=""
export NUM_THREADS=""
export NCNN_FLAGS=""
export DECODE_NON_CONTROL_MASKS="false"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --output-dir) OUTPUT_DIR="$2"; shift ;;
        --frames) FRAMES="$2"; shift ;;
        --duration) DURATION="$2"; shift ;;
        --warmup) WARMUP="$2"; shift ;;
        --warmup-timeout) WARMUP_TIMEOUT="$2"; shift ;;
        --run-id) RUN_ID="$2"; shift ;;
        --model) MODEL="$2"; shift ;;
        --mode) MODE="$2"; shift ;;
        --input-source) INPUT_SOURCE="$2"; shift ;;
        --resolution) RESOLUTION="$2"; shift ;;
        --notes) NOTES="$2"; shift ;;
        --num-threads) NUM_THREADS="$2"; shift ;;
        --ncnn-flags) NCNN_FLAGS="$2"; shift ;;
        --decode-non-control-masks) DECODE_NON_CONTROL_MASKS="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

mkdir -p "$OUTPUT_DIR"
export META_FILE="$OUTPUT_DIR/metadata_${RUN_ID}.json"

echo "=== AVS Benchmark Wrapper ==="
echo "Run ID: $RUN_ID"
echo "Gathering Initial System Info..."

python3 << 'EOF'
import json, os, subprocess

def get_vcgencmd(cmd):
    try:
        return subprocess.check_output(["vcgencmd", cmd], text=True).strip().split('=')[-1]
    except:
        return "N/A"

def get_git():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    except:
        return "unknown"

def get_gov():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor") as f:
            return f.read().strip()
    except:
        return "unknown"

def get_mem():
    try:
        out = subprocess.check_output(["free", "-m"], text=True).splitlines()[1].split()
        return f"{out[2]}/{out[1]} MB"
    except:
        return "unknown"

def get_cpu():
    try:
        with open("/proc/stat") as f:
            return [l.strip() for l in f.readlines() if l.startswith("cpu")]
    except:
        return []

def get_vulkan():
    try:
        subprocess.check_output(["vulkaninfo", "--summary"], stderr=subprocess.STDOUT)
        return "yes"
    except:
        return "no"

def get_pmic_adc():
    # Pi 5 PMIC rails (EXT5V_V = real 5V input; sags on supply brownout)
    try:
        out = subprocess.check_output(["vcgencmd", "pmic_read_adc"], text=True)
        return [l.strip() for l in out.splitlines() if l.strip()]
    except:
        return []

def get_freq():
    freqs = {}
    for name in ("scaling_cur_freq", "scaling_max_freq"):
        try:
            with open(f"/sys/devices/system/cpu/cpu0/cpufreq/{name}") as f:
                freqs[name] = f.read().strip()
        except:
            freqs[name] = "unknown"
    return freqs

metadata = {
    "run_id": os.environ.get("RUN_ID", ""),
    "git_commit": get_git(),
    "governor": get_gov(),
    "cpu_freq_before": get_freq(),
    "vulkan_available": get_vulkan(),
    "mem_usage_before": get_mem(),
    "cpu_stat_before": get_cpu(),
    "temperature_before": get_vcgencmd("measure_temp"),
    "throttled_before": get_vcgencmd("get_throttled"),
    "pmic_adc_before": get_pmic_adc(),
    "model": os.environ.get("MODEL", ""),
    "mode": os.environ.get("MODE", ""),
    "input_source": os.environ.get("INPUT_SOURCE", ""),
    "resolution": os.environ.get("RESOLUTION", ""),
    "notes": os.environ.get("NOTES", ""),
    "num_threads": os.environ.get("NUM_THREADS", ""),
    "ncnn_flags": os.environ.get("NCNN_FLAGS", ""),
    "decode_non_control_masks": os.environ.get("DECODE_NON_CONTROL_MASKS", "false")
}

with open(os.environ.get("META_FILE"), "w") as f:
    json.dump(metadata, f, indent=4)
EOF

echo "Running benchmark script..."
python3 tools/optimize/benchmark_telemetry.py \
    --output-dir "$OUTPUT_DIR" \
    --frames "$FRAMES" \
    --duration "$DURATION" \
    --warmup "$WARMUP" \
    --warmup-timeout "$WARMUP_TIMEOUT" \
    --run-id "$RUN_ID"

EXIT_CODE=$?

echo "Gathering Final System Info..."
python3 << 'EOF'
import json, os, subprocess

def get_vcgencmd(cmd):
    try:
        return subprocess.check_output(["vcgencmd", cmd], text=True).strip().split('=')[-1]
    except:
        return "N/A"

def get_mem():
    try:
        out = subprocess.check_output(["free", "-m"], text=True).splitlines()[1].split()
        return f"{out[2]}/{out[1]} MB"
    except:
        return "unknown"

def get_cpu():
    try:
        with open("/proc/stat") as f:
            return [l.strip() for l in f.readlines() if l.startswith("cpu")]
    except:
        return []

def get_pmic_adc():
    try:
        out = subprocess.check_output(["vcgencmd", "pmic_read_adc"], text=True)
        return [l.strip() for l in out.splitlines() if l.strip()]
    except:
        return []

def get_freq():
    freqs = {}
    for name in ("scaling_cur_freq", "scaling_max_freq"):
        try:
            with open(f"/sys/devices/system/cpu/cpu0/cpufreq/{name}") as f:
                freqs[name] = f.read().strip()
        except:
            freqs[name] = "unknown"
    return freqs

def calculate_cpu_utilization(stat_before, stat_after):
    utils = {}
    for b_line, a_line in zip(stat_before, stat_after):
        b_parts = b_line.split()
        a_parts = a_line.split()
        core_name = b_parts[0]
        
        b_vals = [int(x) for x in b_parts[1:]]
        a_vals = [int(x) for x in a_parts[1:]]
        
        b_total = sum(b_vals)
        a_total = sum(a_vals)
        
        b_idle = b_vals[3] + b_vals[4] # idle + iowait
        a_idle = a_vals[3] + a_vals[4]
        
        delta_total = a_total - b_total
        delta_idle = a_idle - b_idle
        
        if delta_total > 0:
            util = 100.0 * (1.0 - delta_idle / delta_total)
            utils[core_name] = round(util, 2)
    return utils

meta_file = os.environ.get("META_FILE")
if os.path.exists(meta_file):
    with open(meta_file, "r") as f:
        metadata = json.load(f)
        
    metadata["temperature_after"] = get_vcgencmd("measure_temp")
    metadata["throttled_after"] = get_vcgencmd("get_throttled")
    metadata["pmic_adc_after"] = get_pmic_adc()
    metadata["cpu_freq_after"] = get_freq()
    metadata["mem_usage_after"] = get_mem()
    
    cpu_after = get_cpu()
    metadata["cpu_stat_after"] = cpu_after
    
    cpu_before = metadata.get("cpu_stat_before", [])
    if cpu_before and cpu_after:
        metadata["cpu_utilization_per_core_percent"] = calculate_cpu_utilization(cpu_before, cpu_after)
    
    with open(meta_file, "w") as f:
        json.dump(metadata, f, indent=4)
EOF

if [ $EXIT_CODE -ne 0 ]; then
    echo "Benchmark failed with exit code $EXIT_CODE."
    exit $EXIT_CODE
else
    echo "Benchmark finished successfully. Check $OUTPUT_DIR for results."
fi
