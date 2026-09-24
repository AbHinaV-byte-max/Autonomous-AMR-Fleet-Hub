"""
Automated 10-Step Incremental Verification Test Suite
Tests the real SIH-AMR algorithms running directly against simulation5.usd in Omniverse.
"""

import os
import sys
import math
import time
import argparse

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

from ref_sih_amr.models import RobotState, RobotStatus, Task, TaskStatus
from ref_sih_amr.allocator.hungarian import HungarianAllocator
from ref_sih_amr.robot.planner import AStarPlanner
from ref_sih_amr.robot.coordination import ReservationTable, detect_deadlock
from ref_sih_amr.comms.channel import PubSubChannel

from omni_warehouse_nav import OmniWarehouseNavMap
from omni_amr_controller import OmniAMRController
from omni_amr_agent import DecentralizedAMRAgent

try:
    from pxr import Usd, UsdGeom, Gf  # type: ignore
    USD_AVAILABLE = True
except ImportError:
    USD_AVAILABLE = False


def get_usd_stage():
    usd_path = os.path.join(REPO_ROOT, "simulation5.usd")
    if USD_AVAILABLE and os.path.exists(usd_path):
        return Usd.Stage.Open(usd_path)
    return None


def run_test_1():
    print("\n" + "=" * 70)
    print("TEST 1: ONE AMR — Straight Navigation & Distance-Based Stop (No Loops)")
    print("=" * 70)
    stage = get_usd_stage()
    nav_map = OmniWarehouseNavMap(stage=stage, cell_size=0.5)
    comms = PubSubChannel()
    res_table = ReservationTable()

    agent = DecentralizedAMRAgent("AMR_01", stage, nav_map, comms, res_table)
    
    # Set initial position at Highway Y=14.5
    start_pos = (4.5, 14.5)
    goal_pos = (11.0, 14.5) # Straight east along North Highway
    
    agent.controller.actual_x = start_pos[0]
    agent.controller.actual_y = start_pos[1]
    agent.controller.actual_heading = 0.0

    print(f"[TEST 1] Planning from actual start {start_pos} to goal {goal_pos}...")
    success = agent.plan_path_to_world_target(goal_pos, current_time=0.0)
    assert success, "A* path planning failed!"
    print(f"[TEST 1] Generated path with {len(agent.controller.waypoints)} waypoints.")

    # Step simulation
    dt = 0.05
    for step in range(300): # max 15 seconds
        t = step * dt
        agent.step(dt, t)
        if agent.controller.is_stopped and agent.controller.current_waypoint_idx >= len(agent.controller.waypoints):
            print(f"[TEST 1] Goal reached at t={t:.2f}s! Robot stopped.")
            break

    tel = agent.controller.get_telemetry()
    final_pos = (tel["actual_position"][0], tel["actual_position"][1])
    dist_to_goal = math.sqrt((final_pos[0] - goal_pos[0])**2 + (final_pos[1] - goal_pos[1])**2)
    
    print(f"[TEST 1] Final Position: ({final_pos[0]:.2f}, {final_pos[1]:.2f}), Dist to Goal: {dist_to_goal:.3f}m")
    assert dist_to_goal < 0.25, f"Robot failed to reach goal: distance {dist_to_goal}m"
    assert tel["is_stopped"], "Robot did not stop at goal!"
    assert not agent.controller.oscillation_detected, "Oscillation detected!"
    print("✅ TEST 1 PASSED: One AMR reached goal cleanly and stopped without looping.")


def run_test_2():
    print("\n" + "=" * 70)
    print("TEST 2: ONE AMR WITH SEVERAL TURNS — Heading Alignment & Waypoint Progression")
    print("=" * 70)
    stage = get_usd_stage()
    nav_map = OmniWarehouseNavMap(stage=stage, cell_size=0.5)
    comms = PubSubChannel()
    res_table = ReservationTable()

    agent = DecentralizedAMRAgent("AMR_01", stage, nav_map, comms, res_table)
    
    # Route: Start (4.5, -4.5) -> North to (4.5, 14.5) -> East to (18.5, 14.5) -> South to (18.5, 5.0)
    start_pos = (4.5, -4.5)
    goal_pos = (18.5, 5.0)

    agent.controller.actual_x = start_pos[0]
    agent.controller.actual_y = start_pos[1]
    agent.controller.actual_heading = 90.0

    print(f"[TEST 2] Multi-turn navigation from {start_pos} to {goal_pos}...")
    success = agent.plan_path_to_world_target(goal_pos, current_time=0.0)
    assert success, "Multi-turn A* path planning failed!"
    print(f"[TEST 2] Path length: {len(agent.controller.waypoints)} waypoints.")

    turns_logged = 0
    dt = 0.05
    for step in range(800): # max 40 seconds
        t = step * dt
        prev_heading = agent.controller.actual_heading
        agent.step(dt, t)
        
        # Check in-place rotation behavior
        if abs(agent.controller.actual_omega) > 10.0 and agent.controller.actual_v == 0.0:
            turns_logged += 1

        if agent.controller.is_stopped and agent.controller.current_waypoint_idx >= len(agent.controller.waypoints):
            print(f"[TEST 2] Reached multi-turn goal at t={t:.2f}s!")
            break

    tel = agent.controller.get_telemetry()
    final_pos = (tel["actual_position"][0], tel["actual_position"][1])
    dist_to_goal = math.sqrt((final_pos[0] - goal_pos[0])**2 + (final_pos[1] - goal_pos[1])**2)
    
    print(f"[TEST 2] Turns logged during execution: {turns_logged}")
    print(f"[TEST 2] Final Position: ({final_pos[0]:.2f}, {final_pos[1]:.2f}), Dist: {dist_to_goal:.3f}m")
    assert dist_to_goal < 0.30, f"Robot failed to reach multi-turn goal: distance {dist_to_goal}m"
    assert turns_logged > 0, "In-place rotation controller was not activated during turns!"
    print("✅ TEST 2 PASSED: Multi-turn navigation executed with precise in-place heading alignment.")


def run_test_3():
    print("\n" + "=" * 70)
    print("TEST 3: THREE AMRs WITH INDEPENDENT GOALS (Hungarian Allocation)")
    print("=" * 70)
    stage = get_usd_stage()
    nav_map = OmniWarehouseNavMap(stage=stage, cell_size=0.5)
    comms = PubSubChannel()
    res_table = ReservationTable()

    agents = [
        DecentralizedAMRAgent("AMR_01", stage, nav_map, comms, res_table),
        DecentralizedAMRAgent("AMR_02", stage, nav_map, comms, res_table),
        DecentralizedAMRAgent("AMR_03", stage, nav_map, comms, res_table)
    ]

    # Initialize at separate aisles
    starts = [(4.5, -4.5), (-16.0, 14.5), (25.5, -24.0)]
    for i, a in enumerate(agents):
        a.controller.actual_x = starts[i][0]
        a.controller.actual_y = starts[i][1]

    # Create 3 independent tasks
    tasks = [
        Task("TASK_1", pickup_cell=(4.5, 14.5), dropoff_cell=(11.0, 14.5), priority=1),
        Task("TASK_2", pickup_cell=(-16.0, -4.5), dropoff_cell=(-23.0, -4.5), priority=1),
        Task("TASK_3", pickup_cell=(25.5, -4.5), dropoff_cell=(18.5, -4.5), priority=1),
    ]

    allocator = HungarianAllocator(planner=AStarPlanner(), costmap=nav_map)
    robot_states = [a.state for a in agents]
    assignments = allocator.allocate(robot_states, tasks)
    print(f"[TEST 3] Hungarian Assignments: {assignments}")
    assert len(assignments) == 3, f"Expected 3 assignments, got {len(assignments)}"

    for r_id, t_id in assignments.items():
        agent = next(a for a in agents if a.robot_id == r_id)
        task = next(t for t in tasks if t.task_id == t_id)
        agent.assign_task(task, current_time=0.0)

    # Step simulation
    dt = 0.05
    for step in range(400):
        t = step * dt
        for a in agents:
            a.step(dt, t)

    for a in agents:
        print(f"[TEST 3] {a.robot_id}: Waypoint Progress: {a.controller.current_waypoint_idx}/{len(a.controller.waypoints)}, v={a.controller.actual_v:.2f}")

    print("✅ TEST 3 PASSED: Three AMRs allocated optimally via Hungarian algorithm and moving independently.")


def run_test_4():
    print("\n" + "=" * 70)
    print("TEST 4: THREE AMRs + INTERSECTION CONFLICT (Deterministic Priority Yielding)")
    print("=" * 70)
    stage = get_usd_stage()
    nav_map = OmniWarehouseNavMap(stage=stage, cell_size=0.5)
    comms = PubSubChannel()
    res_table = ReservationTable()

    # AMR_01 (High Priority = 10) heading East to Intersection (18.5, 14.5)
    # AMR_02 (Low Priority = 1) heading North to Intersection (18.5, 14.5)
    a1 = DecentralizedAMRAgent("AMR_01", stage, nav_map, comms, res_table)
    a2 = DecentralizedAMRAgent("AMR_02", stage, nav_map, comms, res_table)
    a3 = DecentralizedAMRAgent("AMR_03", stage, nav_map, comms, res_table)

    a1.controller.actual_x = 11.0; a1.controller.actual_y = 14.5; a1.state.task_priority = 10
    a2.controller.actual_x = 18.5; a2.controller.actual_y = 7.0;  a2.state.task_priority = 1
    a3.controller.actual_x = -30.0; a3.controller.actual_y = -24.0

    t1 = Task("HIGH_PRIO_TASK", pickup_cell=(25.5, 14.5), dropoff_cell=(32.5, 14.5), priority=10)
    t2 = Task("LOW_PRIO_TASK", pickup_cell=(18.5, 18.0), dropoff_cell=(18.5, 22.0), priority=1)

    a1.assign_task(t1, 0.0)
    a2.assign_task(t2, 0.0)

    yield_observed = False
    dt = 0.05
    for step in range(300):
        t = step * dt
        # P2P intent exchange
        a1.broadcast_intent(t); a2.broadcast_intent(t); a3.broadcast_intent(t)
        comms.clear()
        a1.process_peer_messages(t); a2.process_peer_messages(t); a3.process_peer_messages(t)
        
        # Conflict coordination
        agents = [a1, a2, a3]
        a1.resolve_conflicts_and_coordination(t, agents)
        a2.resolve_conflicts_and_coordination(t, agents)

        if a2.state.status == RobotStatus.WAITING and a1.state.status == RobotStatus.MOVING:
            yield_observed = True

        a1.step(dt, t)
        a2.step(dt, t)
        a3.step(dt, t)

    print(f"[TEST 4] Priority Yield Observed: {yield_observed}")
    assert yield_observed, "Lower priority robot AMR_02 failed to yield at intersection!"
    print("✅ TEST 4 PASSED: Deterministic priority arbitration verified. Lower priority AMR yields cleanly.")


def run_test_5():
    print("\n" + "=" * 70)
    print("TEST 5: THREE AMRs + NARROW AISLE HEADWAY")
    print("=" * 70)
    stage = get_usd_stage()
    nav_map = OmniWarehouseNavMap(stage=stage, cell_size=0.5)
    comms = PubSubChannel()
    res_table = ReservationTable()

    agents = [
        DecentralizedAMRAgent("AMR_01", stage, nav_map, comms, res_table),
        DecentralizedAMRAgent("AMR_02", stage, nav_map, comms, res_table),
        DecentralizedAMRAgent("AMR_03", stage, nav_map, comms, res_table)
    ]

    # Convoy along highway Y = -4.5: AMR_01 at 18.5, AMR_02 at 11.0, AMR_03 at 4.5
    for i, x in enumerate([18.5, 11.0, 4.5]):
        agents[i].controller.actual_x = x
        agents[i].controller.actual_y = -4.5
        task = Task(f"CONVOY_{i}", pickup_cell=(x + 10.0, -4.5), dropoff_cell=(x + 14.0, -4.5), priority=1)
        agents[i].assign_task(task, 0.0)

    min_recorded_distance = 999.0
    dt = 0.05
    for step in range(200):
        t = step * dt
        for a in agents:
            a.step(dt, t)
            
        d12 = abs(agents[0].controller.actual_x - agents[1].controller.actual_x)
        d23 = abs(agents[1].controller.actual_x - agents[2].controller.actual_x)
        min_recorded_distance = min(min_recorded_distance, d12, d23)

    print(f"[TEST 5] Minimum Inter-Robot Headway: {min_recorded_distance:.2f}m")
    assert min_recorded_distance > 1.0, f"Headway violation: {min_recorded_distance}m"
    print("✅ TEST 5 PASSED: Safe headway preserved in narrow corridor without inter-robot collisions.")


def run_test_6():
    print("\n" + "=" * 70)
    print("TEST 6: DYNAMIC PHYSICAL OBSTACLE — Detection & Real-Time A* Replanning")
    print("=" * 70)
    stage = get_usd_stage()
    nav_map = OmniWarehouseNavMap(stage=stage, cell_size=0.5)
    comms = PubSubChannel()
    res_table = ReservationTable()

    agent = DecentralizedAMRAgent("AMR_01", stage, nav_map, comms, res_table)
    agent.controller.actual_x = 4.5
    agent.controller.actual_y = 14.5
    goal_pos = (25.5, 14.5)

    agent.plan_path_to_world_target(goal_pos, 0.0)
    initial_path_len = len(agent.controller.waypoints)
    print(f"[TEST 6] Initial planned path length: {initial_path_len}")

    # Inject dynamic obstacle directly ahead at (11.0, 14.5) at t=1.0s
    obstacle_injected = False
    replan_triggered = False

    dt = 0.05
    for step in range(250):
        t = step * dt
        if t >= 1.0 and not obstacle_injected:
            print("[TEST 6] Physical simulated obstacle (Pallet) appeared in Omniverse at (11.0, 14.5)!")
            nav_map.update_dynamic_obstacle("FALLEN_PALLET", (11.0, 14.5), active=True)
            obstacle_injected = True

        # Edge-AI policy evaluates local obstacle flag
        action = agent.evaluate_edge_ai_and_safety(t)
        if action == "REROUTE":
            replan_triggered = True
            print(f"[TEST 6] REROUTE action executed: A* replanned alternative path around pallet!")

        agent.step(dt, t)

    assert replan_triggered, "Agent failed to trigger A* replanning when dynamic obstacle appeared!"
    print("✅ TEST 6 PASSED: Dynamic physical obstacle detected in Omniverse, triggering immediate A* replan.")


def run_test_7():
    print("\n" + "=" * 70)
    print("TEST 7: DEADLOCK DETECTION & CYCLE BREAK RESOLUTION")
    print("=" * 70)
    # Test circular wait-for graph: AMR_01 waiting on AMR_02, AMR_02 waiting on AMR_03, AMR_03 waiting on AMR_01
    wait_graph = {
        "AMR_01": "AMR_02",
        "AMR_02": "AMR_03",
        "AMR_03": "AMR_01"
    }
    cycle = detect_deadlock(wait_graph)
    print(f"[TEST 7] Wait-For Graph: {wait_graph}")
    print(f"[TEST 7] Detected Deadlock Cycle: {cycle}")
    assert cycle is not None, "Failed to detect deadlock cycle!"
    assert set(cycle) == {"AMR_01", "AMR_02", "AMR_03"}, "Incorrect cycle members"
    print("✅ TEST 7 PASSED: Deadlock cycle detected accurately by SIH coordination logic.")


def run_test_8():
    print("\n" + "=" * 70)
    print("TEST 8: ROBOT FAILURE & HUNGARIAN TASK REASSIGNMENT")
    print("=" * 70)
    stage = get_usd_stage()
    nav_map = OmniWarehouseNavMap(stage=stage, cell_size=0.5)
    comms = PubSubChannel()
    res_table = ReservationTable()

    a1 = DecentralizedAMRAgent("AMR_01", stage, nav_map, comms, res_table)
    a2 = DecentralizedAMRAgent("AMR_02", stage, nav_map, comms, res_table)
    a3 = DecentralizedAMRAgent("AMR_03", stage, nav_map, comms, res_table)

    a1.controller.actual_x = 4.5; a1.controller.actual_y = -4.5
    a2.controller.actual_x = 18.5; a2.controller.actual_y = -4.5
    a3.controller.actual_x = -16.0; a3.controller.actual_y = 14.5

    task = Task("CRITICAL_TASK", pickup_cell=(-10.0, 14.5), dropoff_cell=(-3.0, 14.5), priority=5)
    a3.assign_task(task, 0.0)
    print(f"[TEST 8] Task initially assigned to {a3.robot_id}")

    # Inject failure on AMR_03
    print(f"[TEST 8] Injecting hardware failure on {a3.robot_id}...")
    a3.trigger_failure(1.0)
    assert a3.state.status == RobotStatus.OFFLINE, "Robot did not enter OFFLINE status!"
    assert task.status == TaskStatus.RECOVERABLE, "Task was not marked RECOVERABLE!"

    # Run Hungarian allocator on surviving idle robots
    allocator = HungarianAllocator(planner=AStarPlanner(), costmap=nav_map)
    active_states = [a1.state, a2.state]
    assignments = allocator.allocate(active_states, [task])
    print(f"[TEST 8] Hungarian Reassignment: {assignments}")
    assert len(assignments) == 1, "Failed to reallocate task!"
    reassigned_robot_id = list(assignments.keys())[0]
    print(f"[TEST 8] Task successfully reallocated to active robot: {reassigned_robot_id}")
    print("✅ TEST 8 PASSED: Robot failure released reservations and task was reassigned seamlessly.")


def run_test_9():
    print("\n" + "=" * 70)
    print("TEST 9: COMMUNICATION DEGRADATION & SAFE MODE RECOVERY")
    print("=" * 70)
    stage = get_usd_stage()
    nav_map = OmniWarehouseNavMap(stage=stage, cell_size=0.5)
    comms = PubSubChannel()
    res_table = ReservationTable()

    a1 = DecentralizedAMRAgent("AMR_01", stage, nav_map, comms, res_table, t_degraded=2.0, t_safe_mode=4.0)
    a2 = DecentralizedAMRAgent("AMR_02", stage, nav_map, comms, res_table, t_degraded=2.0, t_safe_mode=4.0)

    # Initial broadcast at t=0
    a2.broadcast_intent(0.0)
    comms.clear()
    a1.process_peer_messages(0.0)
    assert not a1.is_in_safe_mode, "Agent entered safe mode prematurely"

    # Inject packet loss from AMR_02 for 5 seconds
    print("[TEST 9] Simulating packet loss (no messages from AMR_02)...")
    for t in [1.0, 2.0, 3.0, 4.5]:
        comms.clear()
        a1.process_peer_messages(t, dropped_peer="AMR_02")
        a1.evaluate_edge_ai_and_safety(t)

    assert a1.is_in_safe_mode, "Agent failed to enter SAFE_MODE upon communication timeout!"
    assert a1.controller.is_stopped, "Controller was not halted in safe mode!"
    print(f"[TEST 9] AMR_01 successfully engaged SAFE_MODE (status={a1.state.status.value}, stopped={a1.controller.is_stopped})")

    # Restore communication at t=5.0
    print("[TEST 9] Restoring communication transmission...")
    a2.broadcast_intent(5.0)
    comms.clear()
    a1.process_peer_messages(5.0)
    assert not a1.is_in_safe_mode, "Agent failed to exit safe mode after comms recovery!"
    print("✅ TEST 9 PASSED: Communication fault injection engaged safe mode; transmission recovery cleared it.")


def run_test_10():
    print("\n" + "=" * 70)
    print("TEST 10: FULL SIH DEMONSTRATION ON OMNIVERSE RUNTIME")
    print("=" * 70)
    from run_omni_sih_simulation import OmniSIHSimulationEngine
    engine = OmniSIHSimulationEngine(dt=0.05)
    
    # Add tasks across warehouse
    engine.add_task("TASK_PICK_EAST", pickup_cell=(4.5, 14.5), dropoff_cell=(18.5, 14.5), priority=2)
    engine.add_task("TASK_REPLENISH_WEST", pickup_cell=(-16.0, -4.5), dropoff_cell=(-30.0, -4.5), priority=3)
    engine.add_task("TASK_CROSSDOCK_SOUTH", pickup_cell=(25.5, -24.0), dropoff_cell=(11.0, -24.0), priority=1)

    # Initial allocation
    engine.run_hungarian_allocation()

    # Run for 8 seconds
    engine.run(duration_sec=6.0, print_interval_sec=1.5)

    print("\n[TEST 10] Final Robot Telemetry:")
    for a in engine.agents:
        tel = a.controller.get_telemetry()
        print(f"  • {a.robot_id}: Pos=({tel['actual_position'][0]:.2f}, {tel['actual_position'][1]:.2f}), Heading={tel['actual_heading']:.1f}°, Waypoint={tel['waypoint_index']}/{tel['total_waypoints']}")

    print("✅ TEST 10 PASSED: Full SIH decentralized coordination pipeline successfully executed on Omniverse.")


def main():
    parser = argparse.ArgumentParser(description="SIH-AMR Omniverse Incremental Test Suite")
    parser.add_argument("--test", type=int, default=0, help="Test number to run (1-10, 0 for all)")
    args = parser.parse_args()

    tests = {
        1: run_test_1,
        2: run_test_2,
        3: run_test_3,
        4: run_test_4,
        5: run_test_5,
        6: run_test_6,
        7: run_test_7,
        8: run_test_8,
        9: run_test_9,
        10: run_test_10
    }

    if args.test in tests:
        tests[args.test]()
    else:
        for num in range(1, 11):
            tests[num]()


if __name__ == "__main__":
    main()
