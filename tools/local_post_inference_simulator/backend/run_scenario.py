#!/usr/bin/env python3
import sys
import os
import json
import time
import argparse
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("run_scenario")

# Add workspace root to sys.path
script_dir = os.path.dirname(os.path.abspath(__file__))
workspace_root = os.path.abspath(os.path.join(script_dir, "../../../"))
if workspace_root not in sys.path:
    sys.path.insert(0, workspace_root)

from tools.local_post_inference_simulator.backend.scenario_schema import ScenarioSchema, evaluate_assertions
from tools.local_post_inference_simulator.backend.ros_scenario_runner import ScenarioRunner
from tools.local_post_inference_simulator.backend.ros_bridge import get_bridge_node, shutdown_bridge
import subprocess

synthetic_node_process = None

def start_synthetic_node():
    global synthetic_node_process
    script_path = os.path.join(workspace_root, "tools", "local_post_inference_simulator", "ros2", "synthetic_inference_node.py")
    if os.path.exists(script_path):
        synthetic_node_process = subprocess.Popen(["python3", script_path])
        logger.info(f"Started synthetic_inference_node.py (PID: {synthetic_node_process.pid})")
    else:
        logger.error(f"Could not find synthetic_inference_node.py at {script_path}")

def stop_synthetic_node():
    global synthetic_node_process
    if synthetic_node_process:
        synthetic_node_process.terminate()
        synthetic_node_process.wait(timeout=2.0)
        logger.info("Stopped synthetic_inference_node.py")

def main():
    parser = argparse.ArgumentParser(description="AVS Scenario CLI Runner")
    parser.add_argument("scenario_path", help="Path to scenario JSON file")
    parser.add_argument("--output", help="Path to write the report JSON file")
    parser.add_argument("--mode", default="direct", choices=["direct", "rasterized"],
                        help="Telemetry generation mode ('direct' or 'rasterized')")
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="Max timeout in seconds to wait for scenario completion")
    args = parser.parse_args()

    if not os.path.exists(args.scenario_path):
        logger.error(f"Scenario file not found: {args.scenario_path}")
        sys.exit(1)

    try:
        with open(args.scenario_path, "r") as f:
            scenario_data = json.load(f)
        scenario = ScenarioSchema(**scenario_data)
    except Exception as e:
        logger.error(f"Failed to load or parse scenario JSON: {e}")
        sys.exit(1)

    logger.info(f"Loaded scenario '{scenario.name}' with {len(scenario.frames)} frames")

    # Start the synthetic inference node
    start_synthetic_node()

    # Initialize bridge and runner
    bridge = get_bridge_node()
    runner = ScenarioRunner()
    
    # Wait for subscribers
    logger.info("Waiting for ROS2 subscribers (telemetry and command)...")
    telemetry_ok = bridge.wait_for_telemetry_subscribers(timeout_sec=3.0)
    cmd_ok = bridge.wait_for_cmd_subscribers(timeout_sec=3.0)
    
    if not telemetry_ok or not cmd_ok:
        logger.error("Required ROS2 nodes (ipm_transform_node, control_node) are not subscribing.")
        logger.error(f"  /avs/sim/synthetic_payload subscribers: {bridge.synthetic_payload_pub.get_subscription_count()}")
        logger.error(f"  /avs/cmd subscribers: {bridge.cmd_pub.get_subscription_count()}")
        shutdown_bridge()
        stop_synthetic_node()
        sys.exit(1)

    logger.info("ROS2 nodes connected. Starting playback...")
    
    # Load and run scenario
    runner.load_scenario(scenario, mode=args.mode)
    runner.play()

    # Wait for completion. Join the playback thread directly rather than polling
    # is_playing, which can flip False before the thread finishes writing the report.
    if not runner.wait_until_stopped(timeout=args.timeout):
        logger.error("Timeout reached waiting for scenario playback.")
        runner.stop()
        runner.wait_until_stopped(timeout=2.0)

    report = runner.get_report()
    shutdown_bridge()
    stop_synthetic_node()

    if not report:
        logger.error("Simulation finished but no report was generated.")
        sys.exit(1)

    # Evaluate Assertions
    all_pass, assertions_results = evaluate_assertions(scenario, report)

    # Output report
    output_data = {
        "scenario_name": scenario.name,
        "metrics": report.get("metrics", {}),
        "assertions": assertions_results,
        "all_assertions_passed": all_pass,
        "frames": report.get("frames", [])
    }

    if args.output:
        try:
            with open(args.output, "w") as f:
                json.dump(output_data, f, indent=2)
            logger.info(f"Report written to {args.output}")
        except Exception as e:
            logger.error(f"Failed to write report to {args.output}: {e}")

    # Print results
    print("\n" + "="*40)
    print(f"Scenario: {scenario.name}")
    print("="*40)
    if scenario.assertions:
        print("Assertions Status:")
        for name, res in assertions_results.items():
            status = "PASS" if res["passed"] else "FAIL"
            expected_val = res.get("expected") if "expected" in res else res.get("expected_max")
            print(f"  - {name}: {status} (Expected: {expected_val}, Actual: {res.get('actual')})")
    else:
        print("No assertions defined.")
    print("="*40)

    if all_pass:
        print("RESULT: PASS")
        sys.exit(0)
    else:
        print("RESULT: FAIL")
        sys.exit(1)

if __name__ == "__main__":
    main()
