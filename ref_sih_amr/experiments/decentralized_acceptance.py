"""Acceptance matrix for decentralized peer coordination.

Runs the same fixed workload under B2 stop-and-wait and live P2P coordination
across representative warehouse conditions.  The benchmark is intentionally
measurement-driven: it records per-scenario completion, makespan, waiting,
replans and collisions, then evaluates the aggregate >=20% improvement and
zero-collision requirements.

Evidence matrix scenarios:
- S2_Crossing: orthogonal crossing / choke-point stress.
- S3_Narrow: narrow-aisle contention.
- S4_Blocked: dynamic blocked-aisle rerouting.
- S8_Scale: 8-AMR fleet scalability.
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


DEFAULT_SCENARIOS = ("S2_Crossing", "S3_Narrow", "S4_Blocked", "S8_Scale")

# Keep each stress case comparable and executable. The narrow-aisle case uses
# one delivery because the four-robot, one-cell corridor is itself the
# stress condition; the blocked-aisle case intentionally keeps six jobs so it can
# demonstrate P2P recovery even when B2 cannot finish after the blockage.
SCENARIO_TASKS = {
    "S2_Crossing": 6,
    "S3_Narrow": 1,
    "S4_Blocked": 6,
    "S8_Scale": 6,
}


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

    # S4 is the dynamic-obstacle case: allow both strategies to establish their
    # initial routes, then block the same corridor cell for both runs.
    block_at = 100 if scenario == "S4_Blocked" else None
    block_cell = (5, 2)

    while sim.tick_count < max_ticks and sim.completed_tasks < task_count:
        if block_at is not None and sim.tick_count == block_at:
            sim.block_cell(*block_cell)
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


def benchmark_scenario(scenario: str, task_count: int, max_ticks: int) -> dict:
    baseline = run("B2", scenario, task_count, max_ticks)
    coordinated = run("P2P", scenario, task_count, max_ticks)

    if coordinated.timeout:
        reduction = None
        status = "INCONCLUSIVE_P2P_TIMEOUT"
    elif baseline.timeout:
        # This is still useful resilience evidence: the stop-and-wait baseline
        # failed to finish the blocked-aisle workload while P2P completed it.
        reduction = None
        status = "P2P_COMPLETES_BASELINE_TIMEOUT"
    else:
        reduction = (
            (baseline.makespan - coordinated.makespan)
            / baseline.makespan
            * 100.0
        )
        status = "PASS" if coordinated.collisions == 0 else "FAIL"

    return {
        "scenario": scenario,
        "task_count": task_count,
        "baseline": asdict(baseline),
        "p2p": asdict(coordinated),
        "time_reduction_pct": reduction,
        "zero_collision": coordinated.collisions == 0,
        "status": status,
    }


def main():
    scenario_env = os.getenv("BENCH_SCENARIOS")
    scenarios = tuple(
        s.strip() for s in scenario_env.split(",")
        if s.strip()
    ) if scenario_env else DEFAULT_SCENARIOS

    unknown = [s for s in scenarios if s not in SCENARIOS]
    if unknown:
        raise SystemExit(f"Unknown benchmark scenario(s): {', '.join(unknown)}")

    task_override = os.getenv("BENCH_TASKS")
    max_ticks = int(os.getenv("BENCH_MAX_TICKS", "3000"))
    output_path = os.getenv(
        "BENCH_OUTPUT",
        "artifacts/p2p_acceptance_matrix.json",
    )

    results = [
        benchmark_scenario(
            scenario,
            int(task_override) if task_override else SCENARIO_TASKS[scenario],
            max_ticks,
        )
        for scenario in scenarios
    ]

    completed_results = [
        r for r in results
        if r["time_reduction_pct"] is not None
    ]
    resilience_results = [
        r for r in results
        if r["status"] == "P2P_COMPLETES_BASELINE_TIMEOUT"
    ]
    aggregate_reduction = None
    if completed_results:
        baseline_total = sum(
            r["baseline"]["makespan"] for r in completed_results
        )
        p2p_total = sum(
            r["p2p"]["makespan"] for r in completed_results
        )
        if baseline_total:
            aggregate_reduction = (
                (baseline_total - p2p_total)
                / baseline_total
                * 100.0
            )

    zero_collision = all(r["zero_collision"] for r in results)
    p2p_no_timeouts = all(
        not r["p2p"]["timeout"] for r in results
    )
    stress_cases_pass = all(
        r["status"] in {"PASS", "P2P_COMPLETES_BASELINE_TIMEOUT"}
        and r["p2p"]["collisions"] == 0
        for r in results
    )
    target_20pct = (
        aggregate_reduction is not None
        and aggregate_reduction >= 20.0
    )
    status = (
        "PASS"
        if p2p_no_timeouts and stress_cases_pass and zero_collision and target_20pct
        else "FAIL"
    )

    payload = {
        "scenarios": results,
        "aggregate": {
            "baseline_makespan": sum(
                r["baseline"]["makespan"] for r in completed_results
            ),
            "p2p_makespan": sum(
                r["p2p"]["makespan"] for r in completed_results
            ),
            "time_reduction_pct": aggregate_reduction,
            "zero_collision": zero_collision,
            "p2p_no_timeouts": p2p_no_timeouts,
            "stress_cases_pass": stress_cases_pass,
            "baseline_timeout_resilience_cases": len(resilience_results),
            "target_20pct": target_20pct,
            "status": status,
        },
    }

    print(json.dumps(payload, indent=2))
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)

    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
