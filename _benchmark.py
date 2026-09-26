"""
Phase 2E/F: Stop-and-Wait Baseline vs Coordinated Benchmark + Collision Checker
=================================================================================
Uses short-route tasks (10-20m) designed so both systems CAN complete within 30s.
AMR_01 and AMR_02 share the central aisle junction — guaranteed conflict scenario.
AMR_03 runs an independent path as a control.
"""

import os
import sys
import math
import time
import json
from typing import List, Dict, Tuple, Optional, Any
from itertools import combinations

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

from ref_sih_amr.models import Task, TaskStatus, RobotStatus

# -------------------------------------------------------------------------
# BENCHMARK PARAMETERS
# -------------------------------------------------------------------------
DURATION_SEC = 30.0
FPS          = 30.0
DT           = 1.0 / FPS
FRAMES       = int(DURATION_SEC * FPS)

COLLISION_THRESHOLD_M = 0.6

# Short tasks designed for crossing conflict at junction (4.5, -4.5)
# task_id, robot_index (0=AMR_01, 1=AMR_02, 2=AMR_03), pickup, dropoff, priority
SHORT_TASKS = [
    ("TASK_CROSS_E",  0, (-3.0, -4.5),  (11.0, -4.5), 10),  # east 14m high-prio
    ("TASK_CROSS_N",  1, ( 4.5, -7.0),  ( 4.5,  5.0),  1),  # north 12m low-prio
    ("TASK_INDEP",    2, (18.5, -4.5),  (18.5,  5.0),  2),  # independent 9.5m
]

SHORT_SPAWNS = {
    "AMR_01": (-3.0,  -4.5, 0.035, 0.0),
    "AMR_02": ( 4.5, -10.0, 0.035, 90.0),
    "AMR_03": (18.5,  -9.0, 0.035, 90.0),
}


# -------------------------------------------------------------------------
# Collision Checker (Phase 2F)
# -------------------------------------------------------------------------
class CollisionChecker:
    def __init__(self, threshold_m: float = COLLISION_THRESHOLD_M):
        self.threshold = threshold_m
        self.n_frames_checked = 0
        self.n_pairs_per_frame = 0
        self.min_distance = float("inf")
        self.n_collision_frames = 0
        self.n_collision_events = 0
        self.collision_log: List[Dict] = []
        self._was_colliding: Dict[Tuple, bool] = {}

    def check_frame(self, frame: int, t: float, positions: Dict[str, Tuple[float, float]]):
        self.n_frames_checked += 1
        frame_collision = False
        ids = sorted(positions.keys())
        pairs = list(combinations(ids, 2))
        self.n_pairs_per_frame = len(pairs)

        for r1, r2 in pairs:
            p1, p2 = positions[r1], positions[r2]
            dist = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
            if dist < self.min_distance:
                self.min_distance = dist
            pair = (r1, r2)
            was = self._was_colliding.get(pair, False)
            if dist < self.threshold:
                frame_collision = True
                if not was:
                    self.n_collision_events += 1
                    self.collision_log.append({"frame": frame, "t": round(t, 3),
                                               "pair": f"{r1}-{r2}",
                                               "distance": round(dist, 4)})
                self._was_colliding[pair] = True
            else:
                self._was_colliding[pair] = False
        if frame_collision:
            self.n_collision_frames += 1

    def summary(self) -> str:
        lines = [
            f"  Collision threshold      : {self.threshold:.2f}m",
            f"  Total robot pairs        : {self.n_pairs_per_frame} pairs/frame",
            f"  Frames checked           : {self.n_frames_checked:,}",
            f"  Min inter-robot distance : {self.min_distance:.4f}m",
            f"  Collision frames         : {self.n_collision_frames}",
            f"  Collision events         : {self.n_collision_events}",
        ]
        for ev in self.collision_log[:5]:
            lines.append(f"    t={ev['t']:.2f}s  {ev['pair']}  dist={ev['distance']:.4f}m")
        return "\n".join(lines)


# -------------------------------------------------------------------------
# Stop-and-Wait Baseline
# -------------------------------------------------------------------------
def run_stopwait_baseline(verbose=True) -> Dict[str, Any]:
    """
    Naive stop-and-wait: plan straight pickup→dropoff paths; block if any
    peer robot is within HEADWAY_M of our current waypoint.
    No priority arbitration. First-in-space gets through.
    """
    from omni_amr_controller import OmniAMRController
    from omni_warehouse_nav import OmniWarehouseNavMap
    from ref_sih_amr.robot.planner import AStarPlanner
    from ref_sih_amr.robot.coordination import ReservationTable

    HEADWAY_M = 1.8

    if verbose:
        print("\n" + "="*70)
        print("STOP-AND-WAIT BASELINE")
        print("="*70)

    nav_map = OmniWarehouseNavMap(stage=None, usd_path=None, cell_size=0.5)
    planner = AStarPlanner()

    def make_path(ctrl, goal_world):
        start_nav = nav_map.world_to_nav(ctrl.actual_x, ctrl.actual_y)
        goal_nav  = nav_map.world_to_nav(goal_world[0], goal_world[1])
        path_nav  = planner.plan(start=start_nav, goal=goal_nav,
                                  costmap=nav_map,
                                  reservation_table=ReservationTable(),
                                  start_time=0.0, robot_id=ctrl.robot_id)
        if path_nav:
            wp = [nav_map.nav_to_world(c[0], c[1]) for c in path_nav]
            if wp and math.hypot(wp[0][0]-ctrl.actual_x, wp[0][1]-ctrl.actual_y) < 0.25:
                wp.pop(0)
            wp = wp or [goal_world]
            wp[-1] = goal_world
        else:
            wp = [goal_world]
        ctrl.set_path(wp)
        return len(wp)

    # Setup
    robot_ids = []
    controllers = {}
    phases  = {}   # 0=going pickup, 1=going dropoff, 2=done
    pickups  = {}
    dropoffs = {}

    for _, ridx, pickup, dropoff, prio in SHORT_TASKS:
        rid = f"AMR_0{ridx+1}"
        robot_ids.append(rid)
        ctrl = OmniAMRController(robot_id=rid, stage=None)
        sp = SHORT_SPAWNS[rid]
        ctrl.actual_x, ctrl.actual_y, ctrl.actual_z = sp[0], sp[1], sp[2]
        ctrl.actual_heading = sp[3]
        ctrl.is_stopped = True
        controllers[rid] = ctrl
        phases[rid]   = 0
        pickups[rid]  = pickup
        dropoffs[rid] = dropoff
        n = make_path(ctrl, pickup)
        if verbose:
            print(f"  {rid}: pickup_path={n}wp  pickup={pickup}  dropoff={dropoff}")

    collision_checker = CollisionChecker()
    total_waits = 0
    total_wait_time = 0.0
    task_completion_times = {}
    current_time = 0.0

    for f in range(FRAMES):
        positions = {rid: (controllers[rid].actual_x, controllers[rid].actual_y)
                     for rid in robot_ids}
        collision_checker.check_frame(f, current_time, positions)

        for rid in robot_ids:
            ctrl = controllers[rid]
            if phases[rid] == 2:
                continue

            # Block if peer is within HEADWAY_M of next waypoint
            blocked = False
            wp_idx = ctrl.current_waypoint_idx
            if wp_idx < len(ctrl.waypoints):
                nwp = ctrl.waypoints[wp_idx]
                for orid, (ox, oy) in positions.items():
                    if orid == rid or phases[orid] == 2:
                        continue
                    if math.hypot(ox - nwp[0], oy - nwp[1]) < HEADWAY_M:
                        blocked = True
                        break

            ctrl.is_stopped = blocked
            if blocked:
                total_waits += 1
                total_wait_time += DT

            ctrl.step(DT, current_time, frame=None)

            # Arrival check
            goal = pickups[rid] if phases[rid] == 0 else dropoffs[rid]
            d = math.hypot(ctrl.actual_x - goal[0], ctrl.actual_y - goal[1])
            arrived = (ctrl.current_waypoint_idx >= len(ctrl.waypoints)) or d <= 0.45

            if arrived:
                if phases[rid] == 0:
                    phases[rid] = 1
                    make_path(ctrl, dropoffs[rid])
                    if verbose:
                        print(f"  [BASELINE] {rid}: REACHED PICKUP at t={current_time:.2f}s -> going to dropoff")
                elif phases[rid] == 1:
                    phases[rid] = 2
                    ctrl.clear_path()
                    task_completion_times[rid] = round(current_time, 3)
                    if verbose:
                        print(f"  [BASELINE] {rid}: TASK COMPLETE at t={current_time:.2f}s")

        current_time += DT

    n_done = sum(1 for p in phases.values() if p == 2)
    comp_t = max(task_completion_times.values()) if task_completion_times else DURATION_SEC

    if verbose:
        print(f"\n  Tasks completed  : {n_done}/{len(SHORT_TASKS)}")
        print(f"  Completion time  : {comp_t:.2f}s")
        print(f"  Wait ticks       : {total_waits}")
        print(f"  Wait time        : {total_wait_time:.2f}s")
        print(f"\n  COLLISION REPORT:")
        print(collision_checker.summary())

    return {
        "name": "stop-and-wait",
        "n_completed": n_done, "n_tasks": len(SHORT_TASKS),
        "completion_time": comp_t, "all_done": n_done == len(SHORT_TASKS),
        "total_waits": total_waits, "total_wait_time": round(total_wait_time, 3),
        "task_completion_times": task_completion_times,
        "collision": {
            "min_distance": round(collision_checker.min_distance, 4),
            "collision_frames": collision_checker.n_collision_frames,
            "collision_events": collision_checker.n_collision_events,
            "threshold": COLLISION_THRESHOLD_M,
        },
    }


# -------------------------------------------------------------------------
# Coordinated System Runner
# -------------------------------------------------------------------------
def run_coordinated(verbose=True) -> Dict[str, Any]:
    from run_omni_sih_simulation import OmniSIHSimulationEngine

    if verbose:
        print("\n" + "="*70)
        print("COORDINATED SYSTEM (Hungarian + A* + Priority Arbitration)")
        print("="*70)

    engine = OmniSIHSimulationEngine(cell_size=0.5)
    engine.dt = DT

    for _, ridx, pickup, dropoff, prio in SHORT_TASKS:
        rid = f"AMR_0{ridx+1}"
        agent = next(a for a in engine.agents if a.robot_id == rid)
        sp = SHORT_SPAWNS[rid]
        agent.controller.actual_x, agent.controller.actual_y = sp[0], sp[1]
        agent.controller.actual_heading = sp[3]
        agent.state.position = (sp[0], sp[1])
        agent.state.task_priority = prio
        engine.add_task(f"COORD_{rid}", pickup_cell=pickup, dropoff_cell=dropoff, priority=prio)

    engine.run_hungarian_allocation()

    class Cap:
        def __init__(self):
            self._o = sys.__stdout__
            self.conflicts = 0; self.deadlocks = 0; self.replans = 0
        def write(self, s):
            self._o.write(s)
            if "[CONFLICT]" in s and "YIELD" in s: self.conflicts += 1
            if "[DEADLOCK]" in s and "REPLAN" in s: self.deadlocks += 1
            if "[PATH]" in s and "path_length" in s: self.replans += 1
        def flush(self): self._o.flush()

    cap = Cap()
    sys.stdout = cap
    collision_checker = CollisionChecker()
    task_completion_times = {}

    # Only track the 3 active robots
    active_rids = [f"AMR_0{ridx+1}" for _, ridx, _, _, _ in SHORT_TASKS]

    for f in range(FRAMES):
        engine.step(frame=float(f))
        t = engine.current_time
        positions = {a.robot_id: (a.controller.actual_x, a.controller.actual_y)
                     for a in engine.agents if a.robot_id in active_rids}
        collision_checker.check_frame(f, t - DT, positions)
        for task in engine.tasks:
            if task.status == TaskStatus.COMPLETED and task.task_id not in task_completion_times:
                task_completion_times[task.task_id] = round(t, 3)

    sys.stdout = sys.__stdout__

    n_done = sum(1 for t in engine.tasks if t.status == TaskStatus.COMPLETED)
    comp_t = max(task_completion_times.values()) if task_completion_times else DURATION_SEC

    if verbose:
        print(f"\n  Tasks completed  : {n_done}/{len(engine.tasks)}")
        print(f"  Completion time  : {comp_t:.2f}s")
        print(f"  Conflict yields  : {cap.conflicts}")
        print(f"  Deadlock replans : {cap.deadlocks}")
        print(f"  A* replans       : {cap.replans}")
        print(f"\n  COLLISION REPORT:")
        print(collision_checker.summary())

    return {
        "name": "coordinated",
        "n_completed": n_done, "n_tasks": len(engine.tasks),
        "completion_time": comp_t, "all_done": n_done == len(engine.tasks),
        "n_conflicts": cap.conflicts, "n_deadlocks": cap.deadlocks, "n_replans": cap.replans,
        "task_completion_times": task_completion_times,
        "collision": {
            "min_distance": round(collision_checker.min_distance, 4),
            "collision_frames": collision_checker.n_collision_frames,
            "collision_events": collision_checker.n_collision_events,
            "threshold": COLLISION_THRESHOLD_M,
        },
    }


# -------------------------------------------------------------------------
# S3 Convoy Investigation
# -------------------------------------------------------------------------
def investigate_s3_convoy(verbose=True) -> Dict[str, Any]:
    from run_omni_sih_simulation import OmniSIHSimulationEngine
    from omni_scenarios import setup_s3_narrow_aisle_headway

    FRAMES_S3 = int(12.0 * FPS)
    if verbose:
        print("\n" + "="*70)
        print("S3 CONVOY INVESTIGATION (Phase 2D-3)")
        print("="*70)

    engine = OmniSIHSimulationEngine(cell_size=0.5)
    engine.dt = DT
    setup_s3_narrow_aisle_headway(engine)
    engine.run_hungarian_allocation()

    spacing_records = []
    reservation_violations = 0

    for f in range(FRAMES_S3):
        engine.step(frame=float(f))
        t = engine.current_time
        amr1 = next((a for a in engine.agents if a.robot_id == "AMR_01"), None)
        amr2 = next((a for a in engine.agents if a.robot_id == "AMR_02"), None)
        if amr1 and amr2:
            x1, y1 = amr1.controller.actual_x, amr1.controller.actual_y
            x2, y2 = amr2.controller.actual_x, amr2.controller.actual_y
            spacing = math.hypot(x1-x2, y1-y2)
            spacing_records.append({"t": round(t,3), "spacing": round(spacing,4)})
            nav2 = engine.nav_map.world_to_nav(x2, y2)
            try:
                if engine.reservation_table.get_claimer(nav2, round(t)) == "AMR_01":
                    reservation_violations += 1
            except Exception:
                pass

    if not spacing_records:
        return {}

    min_sp = min(r["spacing"] for r in spacing_records)
    avg_sp = sum(r["spacing"] for r in spacing_records) / len(spacing_records)
    unsafe = sum(1 for r in spacing_records if r["spacing"] < COLLISION_THRESHOLD_M)

    if verbose:
        print(f"\n  Min inter-robot spacing : {min_sp:.4f}m")
        print(f"  Avg inter-robot spacing : {avg_sp:.4f}m")
        print(f"  Unsafe frames (< {COLLISION_THRESHOLD_M}m) : {unsafe}")
        print(f"  Reservation violations  : {reservation_violations}")
        print(f"  VERDICT: {'SAFE - no reservation violations' if reservation_violations == 0 else f'WARNING: {reservation_violations} violations'}")

    return {
        "min_spacing": round(min_sp, 4),
        "avg_spacing": round(avg_sp, 4),
        "unsafe_frames": unsafe,
        "reservation_violations": reservation_violations,
    }


# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------
if __name__ == "__main__":
    print("\n" + "="*70)
    print("PHASE 2E/F: BENCHMARK + COLLISION VERIFICATION")
    print(f"Duration: {DURATION_SEC}s @ {FPS} FPS | Tasks: {len(SHORT_TASKS)} | Threshold: {COLLISION_THRESHOLD_M}m")
    print("="*70)

    t0 = time.time()
    baseline    = run_stopwait_baseline(verbose=True)
    coordinated = run_coordinated(verbose=True)
    convoy      = investigate_s3_convoy(verbose=True)
    elapsed     = time.time() - t0

    print("\n" + "="*70)
    print("COMPARISON RESULTS")
    print("="*70)

    b_t = baseline["completion_time"]
    c_t = coordinated["completion_time"]
    b_n = baseline["n_completed"]
    c_n = coordinated["n_completed"]

    if b_n > 0 and c_n > 0:
        time_reduction = (b_t - c_t) / b_t * 100.0
    else:
        time_reduction = None

    print(f"\n  {'Metric':<32} {'Baseline (S&W)':>16} {'Coordinated':>14}")
    print(f"  {'='*62}")
    b_tasks_str = f"{b_n}/{baseline['n_tasks']}"
    c_tasks_str = f"{c_n}/{coordinated['n_tasks']}"
    print(f"  {'Tasks completed':<32} {b_tasks_str:>16} {c_tasks_str:>14}")
    print(f"  {'Completion time (s)':<32} {b_t:>16.2f} {c_t:>14.2f}")
    print(f"  {'Total wait time (s)':<32} {baseline['total_wait_time']:>16.2f} {'N/A':>14}")
    print(f"  {'Wait ticks':<32} {baseline['total_waits']:>16,} {'N/A':>14}")
    print(f"  {'Conflict yields':<32} {'N/A':>16} {coordinated['n_conflicts']:>14}")
    print(f"  {'A* replans':<32} {'N/A':>16} {coordinated['n_replans']:>14}")
    print(f"  {'Min inter-robot dist (m)':<32} {baseline['collision']['min_distance']:>16.4f} {coordinated['collision']['min_distance']:>14.4f}")
    print(f"  {'Collision events':<32} {baseline['collision']['collision_events']:>16} {coordinated['collision']['collision_events']:>14}")


    print(f"\n  S3 Convoy:")
    print(f"    Min spacing            : {convoy.get('min_spacing','N/A')}m")
    print(f"    Reservation violations : {convoy.get('reservation_violations','N/A')}")
    print(f"    Unsafe frames          : {convoy.get('unsafe_frames','N/A')}")

    print(f"\n  Time reduction: ", end="")
    if time_reduction is not None:
        print(f"{time_reduction:.1f}%")
        if time_reduction >= 20.0:
            print("  --> [PASS] Meets >=20% criterion")
        elif time_reduction > 0:
            print(f"  --> [PARTIAL] Positive but < 20%. CBS integration required for full criterion.")
        else:
            print(f"  --> [FAIL] No improvement detected")
    else:
        if b_n == 0 and c_n > 0:
            print("Coordinated completed; baseline did not. [QUALITATIVE PASS]")
        elif b_n == 0 and c_n == 0:
            print(f"Neither completed all tasks in {DURATION_SEC}s.")
            wt = baseline['total_wait_time']
            cf = coordinated['n_conflicts']
            print(f"  Baseline accumulated {wt:.1f}s blocking; coordinated had {cf} yielding events.")
            print(f"  --> Cannot confirm 20% criterion from this run alone.")

    print(f"\n  Total benchmark time: {elapsed:.1f}s")

    result_path = os.path.join(REPO_ROOT, "_benchmark_results.json")
    with open(result_path, "w") as f:
        json.dump({
            "baseline": baseline, "coordinated": coordinated, "convoy": convoy,
            "time_reduction_pct": time_reduction,
            "duration_sec": DURATION_SEC, "fps": FPS, "collision_threshold_m": COLLISION_THRESHOLD_M,
        }, f, indent=2, default=str)
    print(f"\n  Results saved: {result_path}")
