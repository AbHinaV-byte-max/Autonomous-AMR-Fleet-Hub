"""
Phase 3 Master Benchmark
=========================
Covers:
  3C - Before/after safety fix comparison (exact failing workload)
  3D - Performance criterion: 3-AMR and 6-AMR benchmarks
  3E - Deterministic 3-run repeatability

Design constraints:
  - Deterministic: fixed random seed, fixed spawn positions, fixed tasks
  - No CBS
  - No fabricated measurements
  - Reports negative results without suppression
  - A single collision must remain visible in the output
"""

import os
import sys
import math
import time
import json
import random
from typing import List, Dict, Tuple, Optional, Any
from itertools import combinations
from copy import deepcopy

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

from ref_sih_amr.models import Task, TaskStatus, RobotStatus
from _collision_tracer import CollisionTracer

# ---------------------------------------------------------------------------
# Benchmark seed and config
# ---------------------------------------------------------------------------
RANDOM_SEED = 42

# ---------- 3-AMR benchmark (Benchmark A) -----------------------------------
BENCH_A_FPS      = 30.0
BENCH_A_DT       = 1.0 / BENCH_A_FPS
BENCH_A_DURATION = 30.0
BENCH_A_FRAMES   = int(BENCH_A_DURATION * BENCH_A_FPS)

# Tasks: AMR_01 east crossing (14m), AMR_02 north crossing (12m), AMR_03 independent (9.5m)
# All completable in ~25s at 1 m/s.  AMR_01 and AMR_02 share junction (4.5, -4.5).
BENCH_A_TASKS = [
    # (task_id, robot_idx, pickup_world, dropoff_world, priority)
    ("BENCH_A_CROSS_E",  0, (-3.0, -4.5),  (11.0, -4.5), 10),
    ("BENCH_A_CROSS_N",  1, ( 4.5, -7.0),  ( 4.5,  5.0),  1),
    ("BENCH_A_INDEP",    2, (18.5, -4.5),  (18.5,  5.0),  2),
]
BENCH_A_SPAWNS = {
    "AMR_01": (-3.0, -4.5,  0.035, 0.0),
    "AMR_02": ( 4.5, -10.0, 0.035, 90.0),
    "AMR_03": (18.5, -9.0,  0.035, 90.0),
}

# ---------- 6-AMR benchmark (Benchmark B) -----------------------------------
BENCH_B_FPS      = 30.0
BENCH_B_DT       = 1.0 / BENCH_B_FPS
BENCH_B_DURATION = 60.0          # longer window for 6-robot fleet
BENCH_B_FRAMES   = int(BENCH_B_DURATION * BENCH_B_FPS)

# 6-robot tasks: 3 conflicting pairs crossing at different junctions
BENCH_B_TASKS = [
    # Pair 1: junction (4.5, -4.5)
    ("B_E1",  0, (-3.0,  -4.5),  (11.0, -4.5), 10),  # AMR_01 east
    ("B_N1",  1, ( 4.5,  -7.0),  ( 4.5,  5.0),  1),  # AMR_02 north
    # Pair 2: junction (18.5, -4.5)
    ("B_E2",  2, ( 8.0,  -4.5),  (25.5, -4.5),  8),  # AMR_03 east
    ("B_N2",  3, (18.5,  -7.0),  (18.5,  5.0),  2),  # AMR_04 north
    # Pair 3: junction (-10.0, -4.5)
    ("B_E3",  4, (-23.0, -4.5),  ( 4.5, -4.5),  6),  # AMR_05 east
    ("B_N3",  5, (-10.0, -7.0),  (-10.0, 5.0),  3),  # AMR_06 north
]
BENCH_B_SPAWNS = {
    "AMR_01": (-3.0,  -4.5,  0.035, 0.0),
    "AMR_02": ( 4.5,  -10.0, 0.035, 90.0),
    "AMR_03": ( 8.0,  -4.5,  0.035, 0.0),
    "AMR_04": (18.5,  -10.0, 0.035, 90.0),
    "AMR_05": (-23.0, -4.5,  0.035, 0.0),
    "AMR_06": (-10.0, -10.0, 0.035, 90.0),
}

COLLISION_THRESHOLD = 0.6


# ===========================================================================
# Stop-and-Wait baseline runner (shared by both benchmarks)
# ===========================================================================
def run_stopwait(tasks, spawns, robot_names, duration, fps, dt, nav_map=None, verbose=False):
    """
    Naive stop-and-wait: each robot plans pickup→dropoff via A*.
    At each tick: stop if any peer is within HEADWAY_M of our next waypoint.
    Returns metrics dict.
    """
    from omni_amr_controller import OmniAMRController
    from omni_warehouse_nav import OmniWarehouseNavMap
    from ref_sih_amr.robot.planner import AStarPlanner
    from ref_sih_amr.robot.coordination import ReservationTable

    HEADWAY_M = 1.8
    frames = int(duration * fps)

    if nav_map is None:
        nav_map = OmniWarehouseNavMap(stage=None, usd_path=None, cell_size=0.5)
    planner = AStarPlanner()

    def make_path(ctrl, goal):
        start_nav = nav_map.world_to_nav(ctrl.actual_x, ctrl.actual_y)
        goal_nav  = nav_map.world_to_nav(goal[0], goal[1])
        pnav = planner.plan(start=start_nav, goal=goal_nav,
                            costmap=nav_map,
                            reservation_table=ReservationTable(),
                            start_time=0.0, robot_id=ctrl.robot_id)
        if pnav:
            wp = [nav_map.nav_to_world(c[0], c[1]) for c in pnav]
            if wp and math.hypot(wp[0][0]-ctrl.actual_x, wp[0][1]-ctrl.actual_y) < 0.25:
                wp.pop(0)
            wp = wp or [goal]
            wp[-1] = goal
        else:
            wp = [goal]
        ctrl.set_path(wp)
        return len(wp)

    controllers = {}
    phases = {}
    pickups = {}
    dropoffs = {}

    for tid, ridx, pickup, dropoff, prio in tasks:
        rid = robot_names[ridx]
        sp = spawns[rid]
        ctrl = OmniAMRController(robot_id=rid, stage=None)
        ctrl.actual_x, ctrl.actual_y, ctrl.actual_z = sp[0], sp[1], sp[2]
        ctrl.actual_heading = sp[3]
        ctrl.is_stopped = True
        controllers[rid] = ctrl
        phases[rid]   = 0
        pickups[rid]  = pickup
        dropoffs[rid] = dropoff
        make_path(ctrl, pickup)

    tracer = CollisionTracer(threshold_m=COLLISION_THRESHOLD, nav_map=nav_map)
    total_waits = 0
    total_wait_time = 0.0
    task_completion_times = {}
    current_time = 0.0

    for f in range(frames):
        positions = {rid: (controllers[rid].actual_x, controllers[rid].actual_y)
                     for rid in robot_names if rid in controllers}
        tracer.check_frame(f, current_time, positions)

        for rid in robot_names:
            if rid not in controllers:
                continue
            ctrl = controllers[rid]
            if phases[rid] == 2:
                continue

            blocked = False
            wp_idx = ctrl.current_waypoint_idx
            if wp_idx < len(ctrl.waypoints):
                nwp = ctrl.waypoints[wp_idx]
                for orid, (ox, oy) in positions.items():
                    if orid == rid or phases.get(orid, 2) == 2:
                        continue
                    if math.hypot(ox - nwp[0], oy - nwp[1]) < HEADWAY_M:
                        blocked = True
                        break

            ctrl.is_stopped = blocked
            if blocked:
                total_waits += 1
                total_wait_time += dt

            ctrl.step(dt, current_time, frame=None)

            goal = pickups[rid] if phases[rid] == 0 else dropoffs[rid]
            d = math.hypot(ctrl.actual_x - goal[0], ctrl.actual_y - goal[1])
            arrived = (ctrl.current_waypoint_idx >= len(ctrl.waypoints)) or d <= 0.45

            if arrived:
                if phases[rid] == 0:
                    phases[rid] = 1
                    make_path(ctrl, dropoffs[rid])
                    if verbose:
                        print(f"  [S&W] {rid}: PICKUP done at t={current_time:.2f}s")
                elif phases[rid] == 1:
                    phases[rid] = 2
                    ctrl.clear_path()
                    task_completion_times[rid] = round(current_time, 3)
                    if verbose:
                        print(f"  [S&W] {rid}: TASK DONE at t={current_time:.2f}s")

        current_time += dt

    n_done = sum(1 for p in phases.values() if p == 2)
    comp_t = max(task_completion_times.values()) if task_completion_times else duration

    return {
        "name": "stop-and-wait",
        "n_completed": n_done, "n_tasks": len(tasks),
        "completion_time": comp_t,
        "all_done": n_done == len(tasks),
        "total_waits": total_waits,
        "total_wait_time": round(total_wait_time, 3),
        "task_completion_times": task_completion_times,
        "collision": tracer.summary_dict(),
        "_tracer": tracer,
    }


# ===========================================================================
# Coordinated system runner (shared by both benchmarks)
# ===========================================================================
def run_coordinated(tasks, spawns, robot_names, duration, fps, dt,
                    nav_map=None, verbose=False, prefix="COORD"):
    """
    Coordinated system: Hungarian + A* + priority arbitration + safe deadlock replan.
    Returns metrics dict.
    """
    from run_omni_sih_simulation import OmniSIHSimulationEngine

    frames = int(duration * fps)
    engine = OmniSIHSimulationEngine(cell_size=0.5)
    engine.dt = dt

    # Override spawns for active robots
    active_rids = [robot_names[ridx] for _, ridx, _, _, _ in tasks]
    for _, ridx, pickup, dropoff, prio in tasks:
        rid = robot_names[ridx]
        agent = next(a for a in engine.agents if a.robot_id == rid)
        sp = spawns[rid]
        agent.controller.actual_x = sp[0]
        agent.controller.actual_y = sp[1]
        agent.controller.actual_heading = sp[3]
        agent.state.position = (sp[0], sp[1])
        agent.state.task_priority = prio
        engine.add_task(f"{prefix}_{rid}", pickup_cell=pickup, dropoff_cell=dropoff, priority=prio)

    engine.run_hungarian_allocation()

    class Cap:
        def __init__(self):
            self._o = sys.__stdout__
            self.conflicts = 0; self.deadlocks = 0; self.replans = 0
        def write(self, s):
            self._o.write(s)
            if "[CONFLICT]" in s and "YIELD" in s: self.conflicts += 1
            if "[DEADLOCK]" in s and "SAFE_REPLAN" in s: self.deadlocks += 1
            if "[PATH]" in s and "path_length" in s: self.replans += 1
        def flush(self): self._o.flush()

    cap = Cap()
    sys.stdout = cap

    tracer = CollisionTracer(threshold_m=COLLISION_THRESHOLD, nav_map=engine.nav_map)
    task_completion_times = {}

    for f in range(frames):
        engine.step(frame=float(f))
        t = engine.current_time
        positions = {a.robot_id: (a.controller.actual_x, a.controller.actual_y)
                     for a in engine.agents if a.robot_id in active_rids}
        tracer.check_frame(f, t - dt, positions, agents=engine.agents)

        for task in engine.tasks:
            if task.status == TaskStatus.COMPLETED and task.task_id not in task_completion_times:
                task_completion_times[task.task_id] = round(t, 3)

    sys.stdout = sys.__stdout__

    n_done = sum(1 for t in engine.tasks if t.status == TaskStatus.COMPLETED)
    comp_t = max(task_completion_times.values()) if task_completion_times else duration

    if tracer.n_collision_events > 0 and verbose:
        print(f"\n  [{prefix}] COLLISION REPORT:")
        tracer.print_report(prefix=prefix)

    return {
        "name": "coordinated",
        "n_completed": n_done, "n_tasks": len(engine.tasks),
        "completion_time": comp_t,
        "all_done": n_done == len(engine.tasks),
        "n_conflicts": cap.conflicts,
        "n_deadlocks": cap.deadlocks,
        "n_replans": cap.replans,
        "task_completion_times": task_completion_times,
        "collision": tracer.summary_dict(),
        "_tracer": tracer,
        "_engine": engine,
    }


# ===========================================================================
# Single benchmark comparison (used for 3D + 3E)
# ===========================================================================
def run_one_benchmark(label, tasks, spawns, robot_names,
                      duration, fps, dt, verbose=False):
    """Run both systems on identical workload, return (baseline, coordinated) dicts."""
    random.seed(RANDOM_SEED)

    nav_map = None   # baseline will create its own; coordinated uses engine's

    print(f"\n  --- BASELINE [{label}] ---")
    baseline = run_stopwait(tasks, spawns, robot_names,
                            duration, fps, dt, verbose=verbose)

    print(f"\n  --- COORDINATED [{label}] ---")
    coord = run_coordinated(tasks, spawns, robot_names,
                            duration, fps, dt, verbose=verbose, prefix=label)

    return baseline, coord


# ===========================================================================
# Comparison printer
# ===========================================================================
def print_comparison(label, baseline, coord, duration):
    b_t = baseline["completion_time"]
    c_t = coord["completion_time"]
    b_n = baseline["n_completed"]
    c_n = coord["n_completed"]
    n   = baseline["n_tasks"]

    if b_n > 0 and c_n > 0:
        time_reduction = (b_t - c_t) / b_t * 100.0
    else:
        time_reduction = None

    b_str = f"{b_n}/{n}"
    c_str = f"{c_n}/{n}"

    print(f"\n  [{label}] COMPARISON")
    print(f"  {'Metric':<34} {'Baseline (S&W)':>14} {'Coordinated':>14}")
    print(f"  {'='*62}")
    print(f"  {'Tasks completed':<34} {b_str:>14} {c_str:>14}")
    print(f"  {'Completion time (s)':<34} {b_t:>14.2f} {c_t:>14.2f}")
    print(f"  {'Total wait time (s)':<34} {baseline['total_wait_time']:>14.2f} {'N/A':>14}")
    print(f"  {'Wait ticks':<34} {baseline['total_waits']:>14,} {'N/A':>14}")
    print(f"  {'Conflict yields':<34} {'N/A':>14} {coord['n_conflicts']:>14}")
    print(f"  {'Deadlock safe-replans':<34} {'N/A':>14} {coord['n_deadlocks']:>14}")
    print(f"  {'A* replans':<34} {'N/A':>14} {coord['n_replans']:>14}")
    b_min = baseline["collision"]["min_distance"]
    c_min = coord["collision"]["min_distance"]
    print(f"  {'Min inter-robot dist (m)':<34} {b_min:>14.4f} {c_min:>14.4f}")
    b_col = baseline["collision"]["collision_events"]
    c_col = coord["collision"]["collision_events"]
    print(f"  {'Collision events':<34} {b_col:>14} {c_col:>14}")
    b_frm = baseline["collision"]["collision_frames"]
    c_frm = coord["collision"]["collision_frames"]
    print(f"  {'Collision frames':<34} {b_frm:>14} {c_frm:>14}")

    print(f"\n  Time reduction: ", end="")
    if time_reduction is not None:
        print(f"{time_reduction:.1f}%", end="")
        if time_reduction >= 20.0:
            print("  [PASS >= 20%]")
        elif time_reduction > 0:
            print(f"  [PARTIAL: positive but < 20%]")
        else:
            print(f"  [FAIL: negative]")
    else:
        print("Cannot compute directly")
        if b_n == 0 and c_n == 0:
            print(f"  Neither finished within {duration}s")
            print(f"  Baseline blocked {baseline['total_wait_time']:.1f}s; "
                  f"Coordinated had {coord['n_conflicts']} yields")
        elif c_n > b_n:
            print(f"  Coordinated completed more tasks: qualitative PASS")

    # Collision safety verdict
    collision_pass = c_col == 0 and c_frm == 0
    print(f"\n  Safety: collision_events={c_col}  collision_frames={c_frm}", end="")
    if collision_pass:
        print("  [PASS: zero collisions]")
    else:
        print("  [FAIL: collisions detected]")
        coord["_tracer"].print_report(prefix=label)

    return time_reduction


# ===========================================================================
# Phase 3E: 3-run repeatability
# ===========================================================================
def run_repeatability(label, tasks, spawns, robot_names,
                      duration, fps, dt, n_runs=3):
    """
    Runs benchmark n_runs times with the same seed.
    Since the simulation is fully deterministic (no random components),
    all runs should be identical. Reports any variation.
    """
    print(f"\n  [REPEATABILITY: {label}]  n_runs={n_runs}  seed={RANDOM_SEED}")
    b_times = []
    c_times = []
    b_colls = []
    c_colls = []
    b_mins  = []
    c_mins  = []

    for run in range(1, n_runs + 1):
        random.seed(RANDOM_SEED)
        baseline, coord = run_one_benchmark(
            f"{label}_run{run}", tasks, spawns, robot_names,
            duration, fps, dt, verbose=False
        )
        b_times.append(baseline["completion_time"])
        c_times.append(coord["completion_time"])
        b_colls.append(baseline["collision"]["collision_events"])
        c_colls.append(coord["collision"]["collision_events"])
        b_mins.append(baseline["collision"]["min_distance"])
        c_mins.append(coord["collision"]["min_distance"])
        print(f"    Run {run}: baseline={baseline['completion_time']:.2f}s "
              f"coord={coord['completion_time']:.2f}s  "
              f"b_col={b_colls[-1]}  c_col={c_colls[-1]}  "
              f"c_min={c_mins[-1]:.4f}m")

    def stats(lst):
        return {"mean": sum(lst)/len(lst), "min": min(lst), "max": max(lst)}

    b_st = stats(b_times); c_st = stats(c_times)

    print(f"\n    Baseline completion : mean={b_st['mean']:.2f}s  "
          f"min={b_st['min']:.2f}s  max={b_st['max']:.2f}s")
    print(f"    Coordinated time   : mean={c_st['mean']:.2f}s  "
          f"min={c_st['min']:.2f}s  max={c_st['max']:.2f}s")
    print(f"    Baseline collisions: {b_colls}  (any > 0 means baseline safety issue)")
    print(f"    Coordinated collisions: {c_colls}  (any > 0 means safety bug)")
    print(f"    Coordinated min dist: {[f'{x:.4f}' for x in c_mins]}")

    if b_st["mean"] > 0 and c_st["mean"] > 0:
        mean_reduction = (b_st["mean"] - c_st["mean"]) / b_st["mean"] * 100.0
        print(f"    Mean time reduction: {mean_reduction:.1f}%", end="")
        print("  [PASS]" if mean_reduction >= 20.0 else "  [FAIL]")
    else:
        mean_reduction = None
        print(f"    Mean time reduction: cannot compute")

    max_c_col = max(c_colls)
    if max_c_col > 0:
        print(f"    SAFETY: {max_c_col} collision(s) detected across runs  [FAIL]")
    else:
        print(f"    SAFETY: zero collisions across all {n_runs} runs  [PASS]")

    return {
        "b_completion_stats": b_st,
        "c_completion_stats": c_st,
        "b_collision_events": b_colls,
        "c_collision_events": c_colls,
        "c_min_distances": c_mins,
        "mean_time_reduction_pct": mean_reduction,
    }


# ===========================================================================
# Phase 3G: repo.bat build check
# ===========================================================================
def check_kit_build():
    """Check if the Kit build artefacts exist."""
    kit_py = os.path.join(REPO_ROOT, "_build", "windows-x86_64", "release", "kit", "python.exe")
    repo_bat = os.path.join(REPO_ROOT, "repo.bat")
    pxr_glob_path = os.path.join(REPO_ROOT, "_build", "windows-x86_64", "release", "extscache")

    print(f"\n  [KIT BUILD CHECK]")
    print(f"  repo.bat exists       : {os.path.exists(repo_bat)}")
    print(f"  _build/ exists        : {os.path.exists(os.path.join(REPO_ROOT, '_build'))}")
    print(f"  kit/python.exe exists : {os.path.exists(kit_py)}")
    print(f"  extscache exists      : {os.path.exists(pxr_glob_path)}")

    if not os.path.exists(kit_py):
        print(f"  ACTION REQUIRED: run .\\repo.bat build  (downloads Kit SDK 110.3.0, ~20 min)")
        print(f"  Kit Python path expected: {kit_py}")
    else:
        print(f"  Kit Python FOUND: {kit_py}")

    return os.path.exists(kit_py)


# ===========================================================================
# Main
# ===========================================================================
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", default="all",
                        choices=["3c", "3d", "3e", "3g", "all"])
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    run_all = (args.phase == "all")

    print("=" * 70)
    print("PHASE 3 MASTER BENCHMARK")
    print(f"  Seed: {RANDOM_SEED}")
    print(f"  Collision threshold: {COLLISION_THRESHOLD}m")
    print(f"  3-AMR tasks: {len(BENCH_A_TASKS)}, duration: {BENCH_A_DURATION}s @ {BENCH_A_FPS}FPS")
    print(f"  6-AMR tasks: {len(BENCH_B_TASKS)}, duration: {BENCH_B_DURATION}s @ {BENCH_B_FPS}FPS")
    print("=" * 70)

    robot_names = ["AMR_01", "AMR_02", "AMR_03", "AMR_04", "AMR_05", "AMR_06"]
    all_results = {}
    t0 = time.time()

    # -----------------------------------------------------------------------
    # Phase 3C / 3D-A: 3-AMR benchmark
    # -----------------------------------------------------------------------
    if run_all or args.phase in ("3c", "3d"):
        print("\n" + "=" * 70)
        print("BENCHMARK A (3-AMR, crossing conflict)")
        print("=" * 70)

        baseline_A, coord_A = run_one_benchmark(
            "BENCH_A", BENCH_A_TASKS, BENCH_A_SPAWNS, robot_names[:3],
            BENCH_A_DURATION, BENCH_A_FPS, BENCH_A_DT, verbose=args.verbose
        )
        tr_A = print_comparison("BENCH_A", baseline_A, coord_A, BENCH_A_DURATION)
        all_results["bench_A"] = {
            "baseline": {k: v for k, v in baseline_A.items() if not k.startswith("_")},
            "coordinated": {k: v for k, v in coord_A.items() if not k.startswith("_")},
            "time_reduction_pct": tr_A,
        }

    # -----------------------------------------------------------------------
    # Phase 3D-B: 6-AMR benchmark
    # -----------------------------------------------------------------------
    if run_all or args.phase == "3d":
        print("\n" + "=" * 70)
        print("BENCHMARK B (6-AMR, three crossing pairs)")
        print("=" * 70)

        baseline_B, coord_B = run_one_benchmark(
            "BENCH_B", BENCH_B_TASKS, BENCH_B_SPAWNS, robot_names,
            BENCH_B_DURATION, BENCH_B_FPS, BENCH_B_DT, verbose=args.verbose
        )
        tr_B = print_comparison("BENCH_B", baseline_B, coord_B, BENCH_B_DURATION)
        all_results["bench_B"] = {
            "baseline": {k: v for k, v in baseline_B.items() if not k.startswith("_")},
            "coordinated": {k: v for k, v in coord_B.items() if not k.startswith("_")},
            "time_reduction_pct": tr_B,
        }

    # -----------------------------------------------------------------------
    # Phase 3E: 3-run repeatability on 3-AMR
    # -----------------------------------------------------------------------
    if run_all or args.phase == "3e":
        print("\n" + "=" * 70)
        print("PHASE 3E: REPEATABILITY (3-AMR, 3 runs)")
        print("=" * 70)

        rep_A = run_repeatability(
            "BENCH_A", BENCH_A_TASKS, BENCH_A_SPAWNS, robot_names[:3],
            BENCH_A_DURATION, BENCH_A_FPS, BENCH_A_DT, n_runs=3
        )
        all_results["repeatability_A"] = rep_A

        print("\n" + "=" * 70)
        print("PHASE 3E: REPEATABILITY (6-AMR, 3 runs)")
        print("=" * 70)
        rep_B = run_repeatability(
            "BENCH_B", BENCH_B_TASKS, BENCH_B_SPAWNS, robot_names,
            BENCH_B_DURATION, BENCH_B_FPS, BENCH_B_DT, n_runs=3
        )
        all_results["repeatability_B"] = rep_B

    # -----------------------------------------------------------------------
    # Phase 3G: Kit build check
    # -----------------------------------------------------------------------
    if run_all or args.phase == "3g":
        print("\n" + "=" * 70)
        print("PHASE 3G: KIT BUILD STATUS")
        print("=" * 70)
        kit_ok = check_kit_build()
        all_results["kit_build"] = {"kit_python_present": kit_ok}

    # -----------------------------------------------------------------------
    # Overall summary
    # -----------------------------------------------------------------------
    elapsed = time.time() - t0
    print("\n" + "=" * 70)
    print("OVERALL SUMMARY")
    print("=" * 70)

    criteria = {}

    for key, bench_label in [("bench_A", "3-AMR"), ("bench_B", "6-AMR")]:
        r = all_results.get(key)
        if r:
            c_col = r["coordinated"]["collision"]["collision_events"]
            tr    = r.get("time_reduction_pct")
            safety_pass = (c_col == 0)
            perf_pass   = (tr is not None and tr >= 20.0)
            print(f"\n  {bench_label}:")
            print(f"    Collision events (coordinated) : {c_col}  "
                  f"{'[PASS]' if safety_pass else '[FAIL]'}")
            print(f"    Time reduction                 : "
                  f"{f'{tr:.1f}%' if tr is not None else 'N/A'}  "
                  f"{'[PASS]' if perf_pass else '[FAIL]'}")
            criteria[key] = {"safety": safety_pass, "performance": perf_pass}

    rep_A = all_results.get("repeatability_A")
    rep_B = all_results.get("repeatability_B")
    if rep_A:
        max_col = max(rep_A["c_collision_events"])
        print(f"\n  3-AMR repeatability: max_collisions={max_col}  "
              f"{'[PASS]' if max_col == 0 else '[FAIL]'}")
    if rep_B:
        max_col = max(rep_B["c_collision_events"])
        print(f"  6-AMR repeatability: max_collisions={max_col}  "
              f"{'[PASS]' if max_col == 0 else '[FAIL]'}")

    print(f"\n  Total benchmark time: {elapsed:.1f}s")

    # Save results
    result_path = os.path.join(REPO_ROOT, "_phase3_benchmark_results.json")
    def _serialize(obj):
        if isinstance(obj, (set, frozenset)):
            return list(obj)
        raise TypeError(repr(obj))
    with open(result_path, "w") as f:
        json.dump(all_results, f, indent=2, default=_serialize)
    print(f"\n  Results saved: {result_path}")
