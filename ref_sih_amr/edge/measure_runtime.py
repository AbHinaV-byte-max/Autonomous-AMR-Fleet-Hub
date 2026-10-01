"""Run the robot-local policy on an edge computer and record evidence.

This script is intentionally hardware-dependent. It reports the actual machine
model, Python version, CPU/RAM, and policy latency instead of claiming that a
desktop simulation is an edge-hardware measurement.

Run on Raspberry Pi or Jetson:
    python ref_sih_amr/edge/measure_runtime.py --iterations 1000
"""

import argparse
import json
import platform
import statistics
import time

try:
    import psutil
except ImportError:
    psutil = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=1000)
    args = parser.parse_args()

    from edge_policy import EdgePolicy

    policy = EdgePolicy()
    samples = []
    cpu_samples = []
    ram_samples = []

    for i in range(args.iterations):
        observation = {
            "battery": 80.0,
            "distance_to_goal": 5.0,
            "waiting": float(i % 3),
            "peer_distance": 2.0,
            "blocked": False,
        }
        t0 = time.perf_counter_ns()
        policy.decide(observation)
        samples.append((time.perf_counter_ns() - t0) / 1e6)

        if psutil is not None and i % 10 == 0:
            cpu_samples.append(psutil.cpu_percent(interval=None))
            ram_samples.append(psutil.virtual_memory().percent)

    result = {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "cpu_count": __import__("os").cpu_count(),
        "iterations": args.iterations,
        "policy_latency_ms_mean": statistics.mean(samples),
        "policy_latency_ms_p95": sorted(samples)[int(len(samples) * 0.95) - 1],
        "cpu_percent_mean": statistics.mean(cpu_samples) if cpu_samples else None,
        "ram_percent_mean": statistics.mean(ram_samples) if ram_samples else None,
        "psutil_available": psutil is not None,
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
