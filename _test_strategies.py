"""
Test script for Phase 5 Strategies B, C, D on 3-AMR and 6-AMR benchmarks.
Validates completion time, collision safety (zero collisions, min_distance > 0.6m),
and determines the performance gains.
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
from ref_sih_amr.robot.cbs import CBSPlanner
from ref_sih_amr.robot.coordination import detect_deadlock


def test_strategy_b_wait_first(tasks, spawns, duration, fps, dt):
    """
    Strategy B: Wait-First.
    - Lower priority yields and waits.
    - Yielding robot stops before entering the junction box.
    - Higher priority robot proceeds without artificial hard-stop deadlock as long as distance > 0.7m.
    - If distance < 0.7m, robot stops, but only replans if deadlock persists > 2.5s.
    - Victim selection chooses the lower priority robot.
    """
    from run_omni_sih_simulation import OmniSIHSimulationEngine

    frames = int(duration * fps)
    engine = OmniSIHSimulationEngine(cell_size=0.5)
    engine.dt = dt

    active_rids = [f"AMR_0{ridx+1}" for _, ridx, _, _, _ in tasks]

    for tid, ridx, pickup, dropoff, prio in tasks:
        rid = f"AMR_0{ridx+1}"
        agent = next(a for a in engine.agents if a.robot_id == rid)
        sp = spawns[rid]
        agent.controller.actual_x = sp[0]
        agent.controller.actual_y = sp[1]
        agent.controller.actual_heading = sp[3]
        agent.state.position = (sp[0], sp[1])
        agent.state.task_priority = prio
        engine.add_task(f"STRAT_B_{rid}", pickup_cell=pickup, dropoff_cell=dropoff, priority=prio)

    engine.run_hungarian_allocation()
    agents_by_id = {a.robot_id: a for a in engine.agents}

    tracer = CollisionTracer(threshold_m=COLLISION_THRESHOLD, nav_map=engine.nav_map)
    task_completion_times = {}

    deadlock_persisted = {}  # cycle -> float seconds
    deadlock_count = 0
    replan_count = 0
    total_wait_time = 0.0

    # Custom step logic for Strategy B
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

        # 5. Modified Conflict Resolution for Strategy B (Wait-First):
        for agent in engine.agents:
            if agent.state.status == RobotStatus.OFFLINE or agent.robot_id not in active_rids:
                continue

            # Hard collision stop (safety buffer > 0.6m)
            # If any peer is within 0.75m, stop unconditionally to prevent physical collision
            MIN_SAFETY_GAP = 0.75  # > 0.6m collision threshold
            for peer_id, peer_msg in agent.peer_states.items():
                if peer_id not in active_rids:
                    continue
                px, py = peer_msg.position
                dist = math.hypot(px - agent.controller.actual_x, py - agent.controller.actual_y)
                if dist < MIN_SAFETY_GAP and agent.controller.actual_v > 0.01:
                    agent.controller.is_stopped = True
                    agent.state.status = RobotStatus.WAITING
                    if agent.waiting_on is None:
                        agent.waiting_on = peer_id
                    break

            # Lookahead conflict check
            conflict_peer = None
            for dt_lookahead in [1, 2, 3]:
                t_target = round(current_time) + dt_lookahead
                for cell in agent.current_path_nav[:5]:
                    claimer = agent.reservation_table.get_claimer(cell, float(t_target))
                    if claimer and claimer != agent.robot_id and claimer in active_rids:
                        conflict_peer = claimer
                        break
                if conflict_peer:
                    break

            if not conflict_peer:
                for peer_id, peer_msg in agent.peer_states.items():
                    if peer_id not in active_rids:
                        continue
                    px, py = peer_msg.position
                    dist = math.hypot(px - agent.controller.actual_x, py - agent.controller.actual_y)
                    if dist < 3.5:
                        my_next = set(agent.current_path_nav[:6])
                        peer_next = set(peer_msg.planned_path[:6])
                        if my_next.intersection(peer_next):
                            conflict_peer = peer_id
                            break

            if conflict_peer:
                peer_agent = agents_by_id[conflict_peer]
                my_prio = agent.state.task_priority
                peer_prio = peer_agent.state.task_priority

                if my_prio < peer_prio:
                    # Lower priority yields: stops and waits for higher priority to pass
                    agent.waiting_on = conflict_peer
                    agent.controller.is_stopped = True
                    agent.state.status = RobotStatus.WAITING
                    agent.wait_time += dt
                    total_wait_time += dt
                else:
                    # Higher priority: proceeds!
                    # Only stop if peer is physically dangerously close (< MIN_SAFETY_GAP)
                    px, py = peer_agent.controller.actual_x, peer_agent.controller.actual_y
                    dist_to_peer = math.hypot(px - agent.controller.actual_x, py - agent.controller.actual_y)
                    if dist_to_peer > MIN_SAFETY_GAP:
                        agent.controller.is_stopped = False
                        agent.state.status = RobotStatus.MOVING
                        if agent.waiting_on == conflict_peer:
                            agent.waiting_on = None
            else:
                # No conflict: if previously waiting on someone who has cleared, resume
                if agent.waiting_on:
                    peer_agent = agents_by_id.get(agent.waiting_on)
                    if peer_agent:
                        px, py = peer_agent.controller.actual_x, peer_agent.controller.actual_y
                        dist = math.hypot(px - agent.controller.actual_x, py - agent.controller.actual_y)
                        if dist > 1.8:  # Peer has moved safely away
                            agent.waiting_on = None
                            agent.controller.is_stopped = False
                            agent.state.status = RobotStatus.MOVING

        # 6. Deadlock check with Wait-First Timeout (only replan if stuck for > 2.5s)
        wait_graph = {a.robot_id: a.waiting_on for a in engine.agents if a.waiting_on is not None and a.robot_id in active_rids}
        cycle = detect_deadlock(wait_graph)
        if cycle:
            c_key = tuple(sorted(cycle))
            deadlock_persisted[c_key] = deadlock_persisted.get(c_key, 0.0) + dt

            # Only replan after 2.5s of persistent deadlock
            if deadlock_persisted[c_key] >= 2.5:
                # Victim selection: pick LOWER priority robot in the cycle
                victim_id = min(cycle, key=lambda rid: agents_by_id[rid].state.task_priority)
                victim = agents_by_id[victim_id]
                last_t = engine._deadlock_last_replan.get(victim_id, -999.0)
                if current_time - last_t >= 2.0:
                    deadlock_count += 1
                    replan_count += 1
                    replan_target = victim.current_task.dropoff_cell if victim.current_task.status == TaskStatus.IN_PROGRESS else victim.current_task.pickup_cell
                    # Replan with peer footprint blocked
                    FOOTPRINT_RADIUS = 2
                    exclusion_cells = set()
                    for other in engine.agents:
                        if other.robot_id == victim_id or other.state.status == RobotStatus.OFFLINE:
                            continue
                        cx, cy = engine.nav_map.world_to_nav(other.controller.actual_x, other.controller.actual_y)
                        for dr in range(-FOOTPRINT_RADIUS, FOOTPRINT_RADIUS + 1):
                            for dc in range(-FOOTPRINT_RADIUS, FOOTPRINT_RADIUS + 1):
                                if abs(dr) + abs(dc) <= FOOTPRINT_RADIUS:
                                    exclusion_cells.add((cx + dc, cy + dr))
                    _TAG = f"_dl_{victim_id}"
                    for ecx, ecy in exclusion_cells:
                        engine.nav_map.set_cell_blocked(ecx, ecy, tag=_TAG)
                    try:
                        victim.plan_path_to_world_target(replan_target, current_time)
                    finally:
                        engine.nav_map.clear_cells_by_tag(_TAG)
                    engine._deadlock_last_replan[victim_id] = current_time
                    victim.waiting_on = None
                    deadlock_persisted[c_key] = 0.0
        else:
            deadlock_persisted.clear()

        # 7. Kinematic Step
        for agent in engine.agents:
            agent.step(engine.dt, current_time, frame=float(f))

        # Check task completion BEFORE reallocation
        for task in engine.tasks:
            t_orig = task.task_id.replace("STRAT_B_", "")
            if task.status == TaskStatus.COMPLETED and t_orig not in task_completion_times:
                task_completion_times[t_orig] = round(current_time, 3)

        # Collision check
        positions = {a.robot_id: (a.controller.actual_x, a.controller.actual_y)
                     for a in engine.agents if a.robot_id in active_rids}
        tracer.check_frame(f, current_time, positions, agents=engine.agents)

        engine.current_time += engine.dt

    comp_t = max(task_completion_times.values()) if task_completion_times else duration
    return {
        "name": "Strategy B (Wait-First)",
        "completion_time": comp_t,
        "task_completion_times": task_completion_times,
        "deadlocks": deadlock_count,
        "replans": replan_count,
        "wait_time": round(total_wait_time, 3),
        "collision": tracer.summary_dict(),
    }


def test_strategy_d_cbs(tasks, spawns, duration, fps, dt):
    """
    Strategy D: CBS-Assisted.
    - Uses CBSPlanner to compute joint space-time conflict-free paths for tasks.
    - Feeds resulting waypoints directly to Omniverse controllers.
    - Safety guard: emergency stop active if dist < 0.65m.
    """
    from omni_amr_controller import OmniAMRController
    from omni_warehouse_nav import OmniWarehouseNavMap

    frames = int(duration * fps)
    nav_map = OmniWarehouseNavMap(stage=None, usd_path=None, cell_size=0.5)
    cbs = CBSPlanner()

    active_rids = [f"AMR_0{ridx+1}" for _, ridx, _, _, _ in tasks]
    controllers = {}
    phases = {}  # 0=pickup, 1=dropoff, 2=done
    pickups = {}
    dropoffs = {}
    task_map = {}
    cbs_replan_count = 0
    t_cbs_total = 0.0

    for tid, ridx, pickup, dropoff, prio in tasks:
        rid = f"AMR_0{ridx+1}"
        task_map[rid] = tid
        sp = spawns[rid]
        ctrl = OmniAMRController(robot_id=rid, stage=None)
        ctrl.actual_x, ctrl.actual_y, ctrl.actual_z = sp[0], sp[1], sp[2]
        ctrl.actual_heading = sp[3]
        ctrl.is_stopped = False
        controllers[rid] = ctrl
        phases[rid] = 0
        pickups[rid] = pickup
        dropoffs[rid] = dropoff

    # Phase 0: CBS plan to pickups
    goals_phase0 = {rid: nav_map.world_to_nav(pickups[rid][0], pickups[rid][1]) for rid in active_rids}
    starts_phase0 = {rid: (controllers[rid].actual_x, controllers[rid].actual_y) for rid in active_rids}
    start_times = {rid: 0.0 for rid in active_rids}

    t0 = time.perf_counter()
    paths_phase0 = cbs.plan(goals_phase0, starts_phase0, nav_map, start_times)
    t_cbs_total += (time.perf_counter() - t0)
    cbs_replan_count += 1

    for rid, path in paths_phase0.items():
        if path:
            wp = [nav_map.nav_to_world(c[0], c[1]) for c in path]
            if wp and math.hypot(wp[0][0] - controllers[rid].actual_x, wp[0][1] - controllers[rid].actual_y) < 0.25:
                wp.pop(0)
            wp = wp or [pickups[rid]]
            wp[-1] = pickups[rid]
            controllers[rid].set_path(wp)

    tracer = CollisionTracer(threshold_m=COLLISION_THRESHOLD, nav_map=nav_map)
    task_completion_times = {}
    current_time = 0.0
    total_wait_time = 0.0

    for f in range(frames):
        positions = {rid: (controllers[rid].actual_x, controllers[rid].actual_y) for rid in active_rids}
        tracer.check_frame(f, current_time, positions)

        # Check phase transitions
        newly_at_pickup = []
        for rid in active_rids:
            if phases[rid] == 0:
                d = math.hypot(controllers[rid].actual_x - pickups[rid][0], controllers[rid].actual_y - pickups[rid][1])
                if controllers[rid].current_waypoint_idx >= len(controllers[rid].waypoints) or d <= 0.45:
                    phases[rid] = 1
                    newly_at_pickup.append(rid)

        # If any robot reached pickup, re-plan dropoff paths for robots en route to dropoffs
        if newly_at_pickup:
            active_dropoff_rids = [r for r in active_rids if phases[r] == 1]
            goals_dropoff = {r: nav_map.world_to_nav(dropoffs[r][0], dropoffs[r][1]) for r in active_dropoff_rids}
            starts_dropoff = {r: (controllers[r].actual_x, controllers[r].actual_y) for r in active_dropoff_rids}
            st_dropoff = {r: current_time for r in active_dropoff_rids}
            t0 = time.perf_counter()
            paths_dropoff = cbs.plan(goals_dropoff, starts_dropoff, nav_map, st_dropoff)
            t_cbs_total += (time.perf_counter() - t0)
            cbs_replan_count += 1
            for r, path in paths_dropoff.items():
                if path:
                    wp = [nav_map.nav_to_world(c[0], c[1]) for c in path]
                    if wp and math.hypot(wp[0][0] - controllers[r].actual_x, wp[0][1] - controllers[r].actual_y) < 0.25:
                        wp.pop(0)
                    wp = wp or [dropoffs[r]]
                    wp[-1] = dropoffs[r]
                    controllers[r].set_path(wp)

        # Step kinematics with emergency collision stop
        for rid in active_rids:
            ctrl = controllers[rid]
            if phases[rid] == 2:
                continue

            # Safety guard: stop if peer < 0.70m
            emergency_stop = False
            for orid, (ox, oy) in positions.items():
                if orid == rid or phases[orid] == 2:
                    continue
                if math.hypot(ox - ctrl.actual_x, oy - ctrl.actual_y) < 0.70:
                    emergency_stop = True
                    break

            ctrl.is_stopped = emergency_stop
            if emergency_stop:
                total_wait_time += dt

            ctrl.step(dt, current_time, frame=None)

            # Dropoff arrival check
            if phases[rid] == 1:
                d = math.hypot(ctrl.actual_x - dropoffs[rid][0], ctrl.actual_y - dropoffs[rid][1])
                if ctrl.current_waypoint_idx >= len(ctrl.waypoints) or d <= 0.45:
                    phases[rid] = 2
                    ctrl.clear_path()
                    task_completion_times[task_map[rid]] = round(current_time, 3)

        current_time += dt

    comp_t = max(task_completion_times.values()) if task_completion_times else duration
    return {
        "name": "Strategy D (CBS)",
        "completion_time": comp_t,
        "task_completion_times": task_completion_times,
        "cbs_planning_time_sec": round(t_cbs_total, 4),
        "cbs_replan_count": cbs_replan_count,
        "wait_time": round(total_wait_time, 3),
        "collision": tracer.summary_dict(),
    }


def main():
    print("Testing Strategy B (Wait-First) on 3-AMR:")
    res_b3 = test_strategy_b_wait_first(BENCH_A_TASKS, BENCH_A_SPAWNS, BENCH_A_DURATION, BENCH_A_FPS, BENCH_A_DT)
    print("  Completion time:", res_b3["completion_time"])
    print("  Tasks:", res_b3["task_completion_times"])
    print("  Deadlocks:", res_b3["deadlocks"], "Replans:", res_b3["replans"], "Wait:", res_b3["wait_time"])
    print("  Collision events:", res_b3["collision"]["collision_events"], "Min dist:", res_b3["collision"]["min_distance"])

    print("\nTesting Strategy D (CBS) on 3-AMR:")
    res_d3 = test_strategy_d_cbs(BENCH_A_TASKS, BENCH_A_SPAWNS, BENCH_A_DURATION, BENCH_A_FPS, BENCH_A_DT)
    print("  Completion time:", res_d3["completion_time"])
    print("  Tasks:", res_d3["task_completion_times"])
    print("  CBS plan time:", res_d3["cbs_planning_time_sec"], "s")
    print("  Collision events:", res_d3["collision"]["collision_events"], "Min dist:", res_d3["collision"]["min_distance"])

    print("\nTesting Strategy B (Wait-First) on 6-AMR:")
    res_b6 = test_strategy_b_wait_first(BENCH_B_TASKS, BENCH_B_SPAWNS, BENCH_B_DURATION, BENCH_B_FPS, BENCH_B_DT)
    print("  Completion time:", res_b6["completion_time"])
    print("  Tasks:", res_b6["task_completion_times"])
    print("  Collision events:", res_b6["collision"]["collision_events"], "Min dist:", res_b6["collision"]["min_distance"])

    print("\nTesting Strategy D (CBS) on 6-AMR:")
    res_d6 = test_strategy_d_cbs(BENCH_B_TASKS, BENCH_B_SPAWNS, BENCH_B_DURATION, BENCH_B_FPS, BENCH_B_DT)
    print("  Completion time:", res_d6["completion_time"])
    print("  Tasks:", res_d6["task_completion_times"])
    print("  Collision events:", res_d6["collision"]["collision_events"], "Min dist:", res_d6["collision"]["min_distance"])


if __name__ == "__main__":
    main()
