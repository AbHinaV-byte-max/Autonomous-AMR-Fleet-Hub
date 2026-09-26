"""
Phase 5A: Diagnostics and Performance Bottleneck Instrumentation
Measures:
  - task allocation time
  - initial planning time
  - waiting time
  - conflict-resolution time
  - deadlock waiting time
  - deadlock-replanning time
  - extra path distance / detour distance
  - robot acceleration/deceleration overhead
  - task reassignment time
For each task:
  - task_id
  - baseline_completion_time
  - coordinated_completion_time
  - baseline_path_length
  - coordinated_path_length
  - wait_time
  - replan_count
  - deadlock_count
  - detour_distance
"""

import os
import sys
import math
import time
import json
from typing import Dict, List, Tuple, Any

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

from ref_sih_amr.models import Task, TaskStatus, RobotStatus
from _collision_tracer import CollisionTracer
from _phase3_benchmark import (
    BENCH_A_TASKS, BENCH_A_SPAWNS, BENCH_A_DURATION, BENCH_A_FPS, BENCH_A_DT,
    BENCH_B_TASKS, BENCH_B_SPAWNS, BENCH_B_DURATION, BENCH_B_FPS, BENCH_B_DT,
    COLLISION_THRESHOLD
)

ROBOT_NAMES_ALL = [f"AMR_0{i}" for i in range(1, 7)]


def run_instrumented_baseline(tasks, spawns, robot_names, duration, fps, dt):
    from omni_amr_controller import OmniAMRController
    from omni_warehouse_nav import OmniWarehouseNavMap
    from ref_sih_amr.robot.planner import AStarPlanner
    from ref_sih_amr.robot.coordination import ReservationTable

    HEADWAY_M = 1.8
    frames = int(duration * fps)
    nav_map = OmniWarehouseNavMap(stage=None, usd_path=None, cell_size=0.5)
    planner = AStarPlanner()

    t_plan_start = time.perf_counter()
    controllers = {}
    phases = {}
    pickups = {}
    dropoffs = {}
    initial_path_lengths = {}
    actual_distance = {robot_names[ridx]: 0.0 for _, ridx, _, _, _ in tasks}
    prev_pos = {}
    wait_times = {robot_names[ridx]: 0.0 for _, ridx, _, _, _ in tasks}
    task_map = {}

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
        return pnav or []

    for tid, ridx, pickup, dropoff, prio in tasks:
        rid = robot_names[ridx]
        task_map[rid] = tid
        sp = spawns[rid]
        ctrl = OmniAMRController(robot_id=rid, stage=None)
        ctrl.actual_x, ctrl.actual_y, ctrl.actual_z = sp[0], sp[1], sp[2]
        ctrl.actual_heading = sp[3]
        ctrl.is_stopped = True
        controllers[rid] = ctrl
        phases[rid] = 0
        pickups[rid] = pickup
        dropoffs[rid] = dropoff
        prev_pos[rid] = (sp[0], sp[1])
        p1 = make_path(ctrl, pickup)
        initial_path_lengths[rid] = len(p1) * 0.5  # approx metres

    initial_planning_cpu_sec = time.perf_counter() - t_plan_start

    tracer = CollisionTracer(threshold_m=COLLISION_THRESHOLD, nav_map=nav_map)
    task_completion_times = {}
    current_time = 0.0

    for f in range(frames):
        positions = {rid: (controllers[rid].actual_x, controllers[rid].actual_y)
                     for rid in controllers}
        tracer.check_frame(f, current_time, positions)

        for rid in controllers:
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
                wait_times[rid] += dt

            ctrl.step(dt, current_time, frame=None)

            # Accumulate driven distance
            cur_p = (ctrl.actual_x, ctrl.actual_y)
            d_step = math.hypot(cur_p[0] - prev_pos[rid][0], cur_p[1] - prev_pos[rid][1])
            actual_distance[rid] += d_step
            prev_pos[rid] = cur_p

            goal = pickups[rid] if phases[rid] == 0 else dropoffs[rid]
            d = math.hypot(ctrl.actual_x - goal[0], ctrl.actual_y - goal[1])
            arrived = (ctrl.current_waypoint_idx >= len(ctrl.waypoints)) or d <= 0.45

            if arrived:
                if phases[rid] == 0:
                    phases[rid] = 1
                    p2 = make_path(ctrl, dropoffs[rid])
                    initial_path_lengths[rid] += len(p2) * 0.5
                elif phases[rid] == 1:
                    phases[rid] = 2
                    ctrl.clear_path()
                    task_completion_times[task_map[rid]] = round(current_time, 3)

        current_time += dt

    return {
        "initial_planning_cpu_sec": initial_planning_cpu_sec,
        "task_completion_times": task_completion_times,
        "overall_completion_time": max(task_completion_times.values()) if task_completion_times else duration,
        "actual_distance": actual_distance,
        "initial_path_lengths": initial_path_lengths,
        "wait_times": wait_times,
        "collision": tracer.summary_dict(),
        "task_map": task_map,
    }


def run_instrumented_coordinated(tasks, spawns, robot_names, duration, fps, dt, prefix="COORD"):
    from run_omni_sih_simulation import OmniSIHSimulationEngine
    from ref_sih_amr.robot.coordination import detect_deadlock

    frames = int(duration * fps)
    engine = OmniSIHSimulationEngine(cell_size=0.5)
    engine.dt = dt

    active_rids = [robot_names[ridx] for _, ridx, _, _, _ in tasks]
    task_map = {}  # rid -> tid

    # Setup agents
    for tid, ridx, pickup, dropoff, prio in tasks:
        rid = robot_names[ridx]
        task_map[rid] = tid
        agent = next(a for a in engine.agents if a.robot_id == rid)
        sp = spawns[rid]
        agent.controller.actual_x = sp[0]
        agent.controller.actual_y = sp[1]
        agent.controller.actual_heading = sp[3]
        agent.state.position = (sp[0], sp[1])
        agent.state.task_priority = prio
        engine.add_task(f"{prefix}_{rid}", pickup_cell=pickup, dropoff_cell=dropoff, priority=prio)

    t_alloc_start = time.perf_counter()
    engine.run_hungarian_allocation()
    task_allocation_cpu_sec = time.perf_counter() - t_alloc_start

    # Track metrics
    actual_distance = {rid: 0.0 for rid in active_rids}
    agents_by_id = {a.robot_id: a for a in engine.agents}
    prev_pos = {rid: (agents_by_id[rid].controller.actual_x, agents_by_id[rid].controller.actual_y)
                for rid in active_rids}
    initial_planned_distance = {}
    wait_time_sim = {rid: 0.0 for rid in active_rids}
    deadlock_wait_sim = {rid: 0.0 for rid in active_rids}
    conflict_resolution_cpu_sec = 0.0
    deadlock_replan_cpu_sec = 0.0
    replan_count = {rid: 0 for rid in active_rids}
    deadlock_count = {rid: 0 for rid in active_rids}
    task_reassign_cpu_sec = 0.0
    accel_decel_frames = {rid: 0 for rid in active_rids}

    # Initial planned path distance
    for rid in active_rids:
        ag = agents_by_id[rid]
        initial_planned_distance[rid] = len(ag.current_path_nav) * 0.5

    tracer = CollisionTracer(threshold_m=COLLISION_THRESHOLD, nav_map=engine.nav_map)
    task_completion_times = {}

    for f in range(frames):
        current_time = engine.current_time

        # 1. Comms Broadcast
        for agent in engine.agents:
            agent.broadcast_intent(current_time)
        engine.comms.clear()

        # 3. Process Peer Messages
        for agent in engine.agents:
            agent.process_peer_messages(current_time, dropped_peer=None)

        # 4. Edge-AI Policy
        for agent in engine.agents:
            if agent.state.status != RobotStatus.OFFLINE:
                agent.evaluate_edge_ai_and_safety(current_time)

        # 5. Conflict Resolution & Priority Coordination (PROFILE CPU)
        t_cr_start = time.perf_counter()
        for agent in engine.agents:
            agent.resolve_conflicts_and_coordination(current_time, engine.agents)
        conflict_resolution_cpu_sec += (time.perf_counter() - t_cr_start)

        # 6. Deadlock Detection & Safe Replan (PROFILE CPU)
        wait_graph = {a.robot_id: a.waiting_on for a in engine.agents if a.waiting_on is not None}
        cycle = detect_deadlock(wait_graph)
        if cycle:
            for c_rid in cycle:
                if c_rid in deadlock_wait_sim:
                    deadlock_wait_sim[c_rid] += dt

            victim_id = cycle[0]
            last_t = engine._deadlock_last_replan.get(victim_id, -999.0)
            if current_time - last_t >= engine.DEADLOCK_COOLDOWN_SEC:
                t_dl_start = time.perf_counter()
                victim = next((a for a in engine.agents if a.robot_id == victim_id), None)
                if victim and victim.current_task:
                    deadlock_count[victim_id] += 1
                    replan_count[victim_id] += 1

                    if victim.current_task.status == TaskStatus.IN_PROGRESS:
                        replan_target = victim.current_task.dropoff_cell
                    else:
                        replan_target = victim.current_task.pickup_cell

                    FOOTPRINT_RADIUS = 2
                    exclusion_cells = set()
                    for other in engine.agents:
                        if other.robot_id == victim_id or other.state.status == RobotStatus.OFFLINE:
                            continue
                        ox = other.controller.actual_x
                        oy = other.controller.actual_y
                        cx, cy = engine.nav_map.world_to_nav(ox, oy)
                        for dr in range(-FOOTPRINT_RADIUS, FOOTPRINT_RADIUS + 1):
                            for dc in range(-FOOTPRINT_RADIUS, FOOTPRINT_RADIUS + 1):
                                if abs(dr) + abs(dc) <= FOOTPRINT_RADIUS:
                                    exclusion_cells.add((cx + dc, cy + dr))

                    _EXCL_TAG = f"_deadlock_excl_{victim_id}"
                    for (ecx, ecy) in exclusion_cells:
                        engine.nav_map.set_cell_blocked(ecx, ecy, tag=_EXCL_TAG)

                    try:
                        success = victim.plan_path_to_world_target(replan_target, current_time)
                    finally:
                        engine.nav_map.clear_cells_by_tag(_EXCL_TAG)

                    if success:
                        engine._deadlock_last_replan[victim_id] = current_time
                        victim.waiting_on = None
                        victim.wait_time = 0.0
                    else:
                        engine._deadlock_last_replan[victim_id] = current_time
                deadlock_replan_cpu_sec += (time.perf_counter() - t_dl_start)

        # 7. Kinematic Step
        for agent in engine.agents:
            agent.step(engine.dt, current_time, frame=float(f))

        # Check wait time and distance
        for rid in active_rids:
            ag = agents_by_id[rid]
            cur_p = (ag.controller.actual_x, ag.controller.actual_y)
            d_step = math.hypot(cur_p[0] - prev_pos[rid][0], cur_p[1] - prev_pos[rid][1])
            actual_distance[rid] += d_step
            prev_pos[rid] = cur_p

            if ag.controller.is_stopped or ag.state.status == RobotStatus.WAITING:
                wait_time_sim[rid] += dt
            if ag.controller.actual_v > 0.05 and ag.controller.actual_v < 0.95 * ag.controller.max_v:
                accel_decel_frames[rid] += 1

        # Check completion
        for task in engine.tasks:
            t_orig_id = task.task_id.replace(f"{prefix}_", "")
            if task.status == TaskStatus.COMPLETED and t_orig_id not in task_completion_times:
                task_completion_times[t_orig_id] = round(current_time, 3)

        # 8. Check for newly idle robots and run Hungarian allocation (PROFILE CPU)
        t_reassign_start = time.perf_counter()
        engine.run_hungarian_allocation()
        task_reassign_cpu_sec += (time.perf_counter() - t_reassign_start)

        # Collision check
        positions = {a.robot_id: (a.controller.actual_x, a.controller.actual_y)
                     for a in engine.agents if a.robot_id in active_rids}
        tracer.check_frame(f, current_time, positions, agents=engine.agents)

        engine.current_time += engine.dt

    return {
        "task_allocation_cpu_sec": task_allocation_cpu_sec,
        "conflict_resolution_cpu_sec": conflict_resolution_cpu_sec,
        "deadlock_replan_cpu_sec": deadlock_replan_cpu_sec,
        "task_reassign_cpu_sec": task_reassign_cpu_sec,
        "task_completion_times": task_completion_times,
        "overall_completion_time": max(task_completion_times.values()) if task_completion_times else duration,
        "actual_distance": actual_distance,
        "initial_planned_distance": initial_planned_distance,
        "wait_time_sim": wait_time_sim,
        "deadlock_wait_sim": deadlock_wait_sim,
        "replan_count": replan_count,
        "deadlock_count": deadlock_count,
        "accel_decel_overhead_sec": {rid: accel_decel_frames[rid] * dt for rid in active_rids},
        "collision": tracer.summary_dict(),
        "task_map": task_map,
    }


def main():
    print("=" * 80)
    print("PHASE 5A: DIAGNOSTICS & PERFORMANCE BOTTLENECK ANALYSIS")
    print("=" * 80)

    # ---------------- 3-AMR DIAGNOSTICS ----------------
    print("\n>>> Running 3-AMR Baseline...")
    base_a = run_instrumented_baseline(
        BENCH_A_TASKS, BENCH_A_SPAWNS, ROBOT_NAMES_ALL,
        BENCH_A_DURATION, BENCH_A_FPS, BENCH_A_DT
    )

    print(">>> Running 3-AMR Coordinated...")
    coord_a = run_instrumented_coordinated(
        BENCH_A_TASKS, BENCH_A_SPAWNS, ROBOT_NAMES_ALL,
        BENCH_A_DURATION, BENCH_A_FPS, BENCH_A_DT, prefix="DIAG_A"
    )

    # ---------------- 6-AMR DIAGNOSTICS ----------------
    print("\n>>> Running 6-AMR Baseline...")
    base_b = run_instrumented_baseline(
        BENCH_B_TASKS, BENCH_B_SPAWNS, ROBOT_NAMES_ALL,
        BENCH_B_DURATION, BENCH_B_FPS, BENCH_B_DT
    )

    print(">>> Running 6-AMR Coordinated...")
    coord_b = run_instrumented_coordinated(
        BENCH_B_TASKS, BENCH_B_SPAWNS, ROBOT_NAMES_ALL,
        BENCH_B_DURATION, BENCH_B_FPS, BENCH_B_DT, prefix="DIAG_B"
    )

    # Package output
    results = {
        "3_amr": {
            "baseline": base_a,
            "coordinated": coord_a,
        },
        "6_amr": {
            "baseline": base_b,
            "coordinated": coord_b,
        }
    }

    out_file = os.path.join(REPO_ROOT, "_phase5a_diagnostics_results.json")
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"\n[OK] Results saved to {out_file}")


if __name__ == "__main__":
    main()
