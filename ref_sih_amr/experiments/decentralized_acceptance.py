"""Acceptance benchmark for decentralized peer coordination.

Compares the existing stop-and-wait baseline (B2) with P2P local planning.
The benchmark uses the same fixed task workload for both strategies and
measures makespan, waiting time, replans, and collisions. It never hard-codes
a success claim: the 20% target is printed as PASS/FAIL from measured data.
"""

import json
import os
import sys
from dataclasses import asdict, dataclass

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from experiments.runner import SCENARIOS
from models import Task, TaskStatus
from sim.simulator import Simulator


@dataclass
class Result:
    strategy: str
    completed: int
    makespan: int
    waiting_time: float
    replans: float
    collisions: float
    timeout: bool


def fixed_workload(sim: Simulator, count: int = 6):
    sim.tasks.clear()
    sim.task_generator.queue.clear()
    sim.task_generator.enabled = False

    pickups = list(sim.pickup_cells)
    dropoffs = list(sim.dropoff_cells)
    if not pickups or not dropoffs:
        raise RuntimeError("Benchmark scenario has no pickup/dropoff cells")

    for i in range(count):
        task = Task(
            task_id=f"BENCH_{i + 1:02d}",
            pickup_cell=pickups[i % len(pickups)],
            dropoff_cell=dropoffs[i % len(dropoffs)],
            priority=1 + (i % 3),
            status=TaskStatus.QUEUED,
            created_at=0.0,
            source="MANUAL",
        )
        sim.tasks.append(task)

    sim._allocate()


def run(strategy: str, scenario: str, task_count: int, max_ticks: int) -> Result:
    sim = Simulator(ascii_map=SCENARIOS[scenario], headless=True, strategy=strategy)
    fixed_workload(sim, task_count)

    while sim.tick_count < max_ticks and sim.completed_tasks < task_count:
        sim.tick()

    return Result(
        strategy=strategy,
        completed=sim.completed_tasks,
        makespan=sim.tick_count,
        waiting_time=float(sim.metric_values.get("WAITING_TIME", 0.0)),
        replans=float(sim.metric_values.get("REPLAN_COUNT", 0.0)),
        collisions=float(sim.metric_values.get("COLLISION_COUNT", 0.0)),
        timeout=sim.completed_tasks < task_count,
    )


def main():
    scenario = os.getenv("BENCH_SCENARIO", "S2_Crossing")
    task_count = int(os.getenv("BENCH_TASKS", "6"))
    max_ticks = int(os.getenv("BENCH_MAX_TICKS", "1500"))

    baseline = run("B2", scenario, task_count, max_ticks)
    coordinated = run("P2P", scenario, task_count, max_ticks)

    payload = {
        "scenario": scenario,
        "task_count": task_count,
        "baseline": asdict(baseline),
        "p2p": asdict(coordinated),
    }

    if baseline.timeout or coordinated.timeout:
        payload["time_reduction_pct"] = None
        payload["target_20pct"] = False
        payload["status"] = "INCONCLUSIVE_TIMEOUT"
    else:
        reduction = (baseline.makespan - coordinated.makespan) / baseline.makespan * 100.0
        payload["time_reduction_pct"] = reduction
        payload["target_20pct"] = reduction >= 20.0
        payload["zero_collision"] = coordinated.collisions == 0
        payload["status"] = "PASS" if payload["target_20pct"] and payload["zero_collision"] else "FAIL"

    print(json.dumps(payload, indent=2))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
