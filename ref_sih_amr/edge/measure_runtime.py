"""Measure the robot-local decision layer on the actual edge machine.

Run this script on a Raspberry Pi or Jetson. It records the machine identity,
CPU/RAM telemetry, and local decision latency. Desktop simulation numbers must
not be presented as edge-hardware evidence.
"""

import argparse
import json
import os
import platform
import statistics
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    import psutil
except ImportError:
    psutil = None

from robot.edge_policy import PolicyFeatures, SafeEdgePolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=1000)
    parser.add_argument(
        "--output",
        default=None,
        help="Optional JSON output path for the captured hardware evidence.",
    )
    args = parser.parse_args()

    policy = SafeEdgePolicy()
    samples = []
    cpu_samples = []
    ram_samples = []

    features = PolicyFeatures(
        dist_to_nearest_peer=2.0,
        relative_velocity=0.2,
        time_to_conflict=5.0,
        intersection_occupancy=0.0,
        queue_length=1.0,
        local_obstacle_flag=0.0,
        task_urgency=2.0,
        battery=80.0,
        peer_comm_freshness=0.0,
    )

    for i in range(args.iterations):
        t0 = time.perf_counter_ns()
        policy.decide(features, {"CONTINUE", "YIELD", "WAIT", "REROUTE", "DEGRADED_MODE"})
        samples.append((time.perf_counter_ns() - t0) / 1e6)

        if psutil is not None and i % 10 == 0:
            cpu_samples.append(psutil.cpu_percent(interval=None))
            ram_samples.append(psutil.virtual_memory().percent)

    ordered = sorted(samples)
    result = {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "cpu_count": os.cpu_count(),
        "iterations": args.iterations,
        "policy_latency_ms_mean": statistics.mean(samples),
        "policy_latency_ms_p95": ordered[int(len(ordered) * 0.95) - 1],
        "cpu_percent_mean": statistics.mean(cpu_samples) if cpu_samples else None,
        "ram_percent_mean": statistics.mean(ram_samples) if ram_samples else None,
        "psutil_available": psutil is not None,
    }
    rendered = json.dumps(result, indent=2)
    print(rendered)
    if args.output:
        output_dir = os.path.dirname(args.output)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(rendered + "\n")


if __name__ == "__main__":
    main()
