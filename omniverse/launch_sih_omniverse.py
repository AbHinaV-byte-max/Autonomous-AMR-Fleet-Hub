"""
SIH-AMR + NVIDIA Omniverse Kit Unified Framework Launcher
Integrates the complete Decentralized Multi-AMR Coordination architecture:
- Hungarian Dynamic Task Allocation & Recovery across diverse warehouse aisles
- Space-Time Reservation Table & Conflict-Based Search
- Differential Drive Physical Kinematic Controllers in Omniverse (simulation5.usd)
- 9-Feature Edge-AI Local Policy with Authoritative Deterministic Safety Arbiter
- Multi-Frame Keyframe Baking into the Omniverse Timeline (0-1500 frames @ 60 FPS)
- Scenario Switcher: S1 (Multi-Task Hungarian), S2 (Crossing), S3 (Narrow Aisle),
  S4 (Dynamic Obstacle), S5 (Fault Recovery), S6 (Comms Delay)
"""

import os
import sys
import argparse

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")

if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

from omni_scenarios import run_scenario

def main():
    parser = argparse.ArgumentParser(description="SIH-AMR Omniverse Runtime & Scenario Launcher")
    parser.add_argument(
        "--scenario", "-s",
        type=str,
        default="s1",
        help="Scenario to execute and bake: s1 (multi_task_hungarian), s2 (crossing), s3 (narrow), s4 (obstacle), s5 (failure), s6 (comms), all"
    )
    parser.add_argument(
        "--duration", "-d",
        type=float,
        default=25.0,
        help="Duration of the simulation in seconds (default: 25.0s = 1500 frames @ 60 FPS)"
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=60.0,
        help="Frames per second for timeline keyframing (default: 60.0)"
    )

    args = parser.parse_args()

    print("=" * 80)
    print("STARTING REAL SIH-AMR DECENTRALIZED RUNTIME ON OMNIVERSE (simulation5.usd)")
    print(f"Target Scenario: {args.scenario.upper()} | Duration: {args.duration}s | FPS: {args.fps}")
    print("=" * 80)

    run_scenario(scenario_name=args.scenario, duration_sec=args.duration, fps=args.fps)

if __name__ == "__main__":
    main()
