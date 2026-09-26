"""
SIH-AMR Scenario Suite for NVIDIA Omniverse (simulation5.usd)
Implements all key operational scenarios from the SIH specification:
- S1: Multi-Task Hungarian Allocation (Continuous Pick & Delivery Cycle)
- S2: Crossing Priority & Intersection Yielding
- S3: Narrow Aisle Headway & Convoy
- S4: Dynamic Obstacle Blockage & Space-Time A* Detour
- S5: Robot Failure & Automated Hungarian Task Recovery
- S6: Communication Packet Loss, Safe Mode Halt & Auto-Resume
- S0: Unified Comprehensive Multi-Phase Demonstration

Phase 1 changes:
  F2 ? Added proper __main__ block so scenarios are directly runnable:
           python omni_scenarios.py s2
  F3 ? run_scenario() now delegates to engine.step() exclusively.
       All scenario-specific events are injected via event_callback
       so there is ONE authoritative tick loop (engine.step).
       Deadlock detection, which lives in engine.step(), now runs on
       every frame regardless of which scenario is active.
"""

import os
import sys
import time
import math
from typing import List, Dict, Any, Optional

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

from run_omni_sih_simulation import OmniSIHSimulationEngine
from omni_amr_controller import clear_all_stage_time_samples
from ref_sih_amr.models import Task, TaskStatus, RobotStatus


# ---------------------------------------------------------------------------
# Scenario setup functions (unchanged API)
# ---------------------------------------------------------------------------

def setup_s1_multi_task_hungarian(engine: OmniSIHSimulationEngine):
    """
    Scenario 1: Full 6-AMR Multi-Pattern Hungarian Fleet Workflows.
    Allocates tasks with distinct patterns across the warehouse, picking up goods
    and delivering them directly to official warehouse dropping points (dropPoint1 and dropPoint2).
    Upon dropoff completion, tasks transition to COMPLETED, triggering the Hungarian algorithm
    to dynamically assign subsequent waves of tasks.
    """
    print("\n[SCENARIO 1: FULL 6-AMR MULTI-PATTERN HUNGARIAN ALLOCATION]")
    print("Populating multi-wave task queue with routes to dropPoint1 and dropPoint2...")
    
    DROP_1 = (32.86, 23.92)  # East Sector Outbound Drop Point (dropPoint1)
    DROP_2 = (-34.14, 22.13) # West Sector Sortation/Receiving Drop Point (dropPoint2)
    
    # Wave 1 Tasks (allocated across all 6 AMRs at t=0 via Hungarian Algorithm)
    # Pattern 1 (AMR_01): Outbound Picking East -> Highway -> dropPoint1
    engine.add_task("TASK_01_EAST_PICK", pickup_cell=(4.5, 5.0), dropoff_cell=DROP_1, priority=2)
    # Pattern 2 (AMR_02): Inbound Replenishment West Dock -> High-Bay Storage
    engine.add_task("TASK_02_WEST_REPLENISH", pickup_cell=(-34.14, 22.13), dropoff_cell=(-23.0, -18.0), priority=3)
    # Pattern 3 (AMR_03): Cross-Dock Transfer East Express -> dropPoint1
    engine.add_task("TASK_03_CROSSDOCK_EAST", pickup_cell=(18.5, 5.0), dropoff_cell=DROP_1, priority=1)
    # Pattern 4 (AMR_04): Narrow Aisle S-Curve Picking -> dropPoint2
    engine.add_task("TASK_04_SORTATION_WEST", pickup_cell=(-10.0, -4.5), dropoff_cell=DROP_2, priority=2)
    # Pattern 5 (AMR_05): Deep Depot Heavy Transport -> dropPoint1
    engine.add_task("TASK_05_HEAVY_DEPOT", pickup_cell=(25.5, -4.5), dropoff_cell=DROP_1, priority=2)
    # Pattern 6 (AMR_06): Buffer & Staging Loop -> dropPoint2
    engine.add_task("TASK_06_BUFFER_LOOP", pickup_cell=(-30.0, -4.5), dropoff_cell=DROP_2, priority=3)
    
    # Wave 2 Tasks (dynamically allocated by Hungarian as robots complete dropoffs and become IDLE)
    engine.add_task("TASK_07_OUTBOUND_EXPRESS", pickup_cell=(11.0, 5.0), dropoff_cell=DROP_1, priority=2)
    engine.add_task("TASK_08_INBOUND_PUTAWAY", pickup_cell=(-16.0, 5.0), dropoff_cell=DROP_2, priority=2)
    engine.add_task("TASK_09_VNA_TRANSFER", pickup_cell=(25.5, 5.0), dropoff_cell=DROP_1, priority=1)
    engine.add_task("TASK_10_STAGING_RETURN", pickup_cell=(-3.0, 5.0), dropoff_cell=DROP_2, priority=2)
    engine.add_task("TASK_11_NORTH_TRANSFER", pickup_cell=(18.5, 14.5), dropoff_cell=DROP_1, priority=2)
    engine.add_task("TASK_12_SOUTH_RECYCLE", pickup_cell=(-23.0, 5.0), dropoff_cell=DROP_2, priority=2)


def setup_s2_crossing_priority(engine: OmniSIHSimulationEngine):
    """
    Scenario 2: Crossing Priority & Intersection Yielding at (4.5, -4.5).
    AMR_01 (P=10, Urgent) moving East across the intersection.
    AMR_02 (P=1, Low Urgency) moving North across the intersection.
    AMR_02 yields cleanly, waits for AMR_01 to clear, and then proceeds.
    """
    print("\n[SCENARIO 2: CROSSING PRIORITY & INTERSECTION YIELDING]")
    engine.agents[0].controller.actual_x = -3.0
    engine.agents[0].controller.actual_y = -4.5
    engine.agents[0].controller.actual_heading = 0.0
    engine.agents[0].state.position = (-3.0, -4.5)
    engine.agents[0].state.task_priority = 10

    engine.agents[1].controller.actual_x = 4.5
    engine.agents[1].controller.actual_y = -10.0
    engine.agents[1].controller.actual_heading = 90.0
    engine.agents[1].state.position = (4.5, -10.0)
    engine.agents[1].state.task_priority = 1

    engine.add_task("URGENT_EAST_CROSSING", pickup_cell=(0.0, -4.5), dropoff_cell=(11.0, -4.5), priority=10)
    engine.add_task("LOW_PRIO_NORTH_AISLE", pickup_cell=(4.5, -7.0), dropoff_cell=(4.5, 5.0), priority=1)


def setup_s3_narrow_aisle_headway(engine: OmniSIHSimulationEngine):
    """
    Scenario 3: Narrow Aisle Headway & Convoy Navigation.
    AMR_01 and AMR_02 travel in convoy along Aisle X=4.5 maintaining safe headway.
    """
    print("\n[SCENARIO 3: NARROW AISLE HEADWAY & CONVOY]")
    engine.agents[0].controller.actual_x = 4.5
    engine.agents[0].controller.actual_y = 2.0
    engine.agents[0].state.position = (4.5, 2.0)

    engine.agents[1].controller.actual_x = 4.5
    engine.agents[1].controller.actual_y = -4.5
    engine.agents[1].state.position = (4.5, -4.5)

    engine.add_task("LEAD_AMR_TASK", pickup_cell=(4.5, 5.0), dropoff_cell=(4.5, 14.5), priority=2)
    engine.add_task("TRAIL_AMR_TASK", pickup_cell=(4.5, 0.0), dropoff_cell=(4.5, 10.0), priority=2)


def setup_s4_dynamic_obstacle(engine: OmniSIHSimulationEngine):
    """
    Scenario 4: Dynamic Obstacle Injection & Space-Time A* Detour.
    AMR_01 travels North along Aisle X=4.5. An obstacle blocks (4.5, 5.0).
    AMR_01 senses blockage, re-plans detour via Aisle X=11.0, and reaches goal safely.
    """
    print("\n[SCENARIO 4: DYNAMIC OBSTACLE DETOUR]")
    engine.add_task("DELIVERY_ACROSS_AISLE", pickup_cell=(4.5, 0.0), dropoff_cell=(4.5, 14.5), priority=2)


def setup_s5_robot_failure_recovery(engine: OmniSIHSimulationEngine):
    """
    Scenario 5: Hardware Breakdown & Automated Hungarian Task Recovery.
    AMR_03 breaks down mid-transit. Task is marked RECOVERABLE and reassigned to AMR_01.
    """
    print("\n[SCENARIO 5: HARDWARE BREAKDOWN & HUNGARIAN TASK RECOVERY]")
    engine.add_task("CRITICAL_DEPOT_DELIVERY", pickup_cell=(18.5, 3.0), dropoff_cell=(18.5, 14.5), priority=5)
    engine.add_task("ROUTINE_SORT_TASK", pickup_cell=(-16.0, 3.0), dropoff_cell=(-10.0, -4.5), priority=2)


def setup_s6_comms_degradation(engine: OmniSIHSimulationEngine):
    """
    Scenario 6: Communication Degradation, Safe Mode Halt & Auto-Resume.
    AMR_02 experiences packet loss, engages SAFE_MODE, and resumes when packets return.
    """
    print("\n[SCENARIO 6: COMMUNICATION DEGRADATION & SAFE MODE RECOVERY]")
    engine.add_task("LONG_HAUL_REPLENISH", pickup_cell=(-16.0, 3.0), dropoff_cell=(-16.0, 14.5), priority=2)


# ---------------------------------------------------------------------------
# F3 FIX: Scenario event callbacks (one per scenario that needs events)
# Each callback is called by engine.step() at the start of every tick.
# Return value: None (no comms fault) or the robot_id to drop packets for.
# ---------------------------------------------------------------------------

def _make_s4_callback():
    """Factory returns a stateful S4 event callback."""
    obstacle_injected = [False]  # mutable cell so closure can write

    def _cb(engine, t: float):
        if not obstacle_injected[0] and t >= 3.0:
            obstacle_injected[0] = True
            obs_world = (4.5, 5.0)
            obs_nav = engine.nav_map.world_to_nav(obs_world[0], obs_world[1])
            print(f"\n[OBSTACLE] t={t:.2f}s  INJECTED: FALLEN_PALLET at world={obs_world}  nav_cell={obs_nav}")

            # Inject into the nav map (this immediately makes the cell '#' for A*)
            engine.nav_map.update_dynamic_obstacle("FALLEN_PALLET", obs_world, active=True)

            # Check which active robots have this cell on their CURRENT planned path
            affected_current = []
            will_affect = []   # robots not yet at dropoff but whose dropoff path will pass through it
            for a in engine.agents:
                if obs_nav in a.current_path_nav:
                    affected_current.append(a.robot_id)
                elif a.current_task is not None:
                    # Heuristic: if dropoff cell is north of obstacle, the path likely passes it
                    from ref_sih_amr.models import TaskStatus
                    if a.current_task.status != TaskStatus.COMPLETED:
                        goal = (a.current_task.dropoff_cell
                                if a.current_task.status.value in ("in_progress", "assigned")
                                else None)
                        if goal and goal[1] > obs_world[1]:
                            will_affect.append((a.robot_id, goal))

            if affected_current:
                print(f"[OBSTACLE] CURRENT PATH BLOCKED = YES  robots={affected_current}")
                for robot_id in affected_current:
                    a = next(ag for ag in engine.agents if ag.robot_id == robot_id)
                    target = (a.current_task.dropoff_cell
                              if a.current_task and a.current_task.status.value == "in_progress"
                              else (a.current_task.pickup_cell if a.current_task else None))
                    if target:
                        print(f"[OBSTACLE] REPLAN TRIGGERED for {robot_id}  new_target={target}")
                        a.plan_path_to_world_target(target, t)
                    else:
                        print(f"[OBSTACLE] {robot_id} has no active target; no replan issued")
            else:
                print(f"[OBSTACLE] CURRENT PATH AFFECTED = NO  (obstacle injected before dropoff path planned)")
                if will_affect:
                    print(f"[OBSTACLE] NOTE: Future dropoff paths for {[r for r,_ in will_affect]} will route around it")
                    print(f"[OBSTACLE]       (A* will avoid nav_cell={obs_nav} when they reach pickup and replan)")
                else:
                    print(f"[OBSTACLE] No robots currently have paths through this cell")

        return None  # no comms fault

    return _cb


def _make_s5_callback():
    """Factory returns a stateful S5 event callback."""
    failure_triggered = [False]

    def _cb(engine: OmniSIHSimulationEngine, t: float):
        if not failure_triggered[0] and t >= 4.0:
            failure_triggered[0] = True
            amr3 = next((a for a in engine.agents if a.robot_id == "AMR_03"), None)
            if amr3 and amr3.state.status != RobotStatus.OFFLINE:
                failed_task = amr3.current_task.task_id if amr3.current_task else "None"
                print(f"\n[FAILURE] t={t:.2f}s  robot=AMR_03  task={failed_task}")
                print(f"[FAILURE] action=TASK_REASSIGNMENT  (Hungarian reallocation will run next tick)")
                amr3.trigger_failure(t)
        return None

    return _cb


def _make_s6_callback():
    """Factory returns a stateful S6 event callback (returns dropped_peer or None)."""
    comms_fault_start = 2.5
    comms_fault_end   = 6.5
    logged_start = [False]
    logged_end   = [False]

    def _cb(engine: OmniSIHSimulationEngine, t: float):
        if comms_fault_start <= t <= comms_fault_end:
            if not logged_start[0]:
                logged_start[0] = True
                print(f"\n[COMMS] t={t:.2f}s  Packet loss injected for AMR_02  (until t={comms_fault_end}s)")
            return "AMR_02"   # drop AMR_02 broadcasts
        else:
            if logged_start[0] and not logged_end[0] and t > comms_fault_end:
                logged_end[0] = True
                print(f"\n[COMMS] t={t:.2f}s  Network restored for AMR_02  Heartbeats nominal.")
            return None        # no comms fault

    return _cb


# ---------------------------------------------------------------------------
# F3 FIX: Unified run_scenario() ? delegates entirely to engine.step()
# ---------------------------------------------------------------------------

def run_scenario(scenario_name: str = "multi_task_hungarian",
                 duration_sec: float = 25.0, fps: float = 60.0):
    """
    Runs the selected scenario, executing decentralized coordination and baking to simulation5.usd.

    F3: This function now calls engine.step() on every frame ? the ONLY tick path.
    Scenario-specific events are passed as an event_callback so engine.step() injects
    them at the right time without any duplicate loop logic here.
    """
    engine = OmniSIHSimulationEngine(cell_size=0.5)

    # Print grid diagnostic immediately after construction so we have proof
    engine.nav_map.print_grid_summary()

    name_lower = scenario_name.lower().strip()

    # Choose setup and optional event callback
    event_callback = None

    if name_lower in ["s1", "s1_hungarian", "multi_task_hungarian", "all", "unified", "default"]:
        setup_s1_multi_task_hungarian(engine)
    elif name_lower in ["s2", "s2_crossing", "crossing", "crossing_priority"]:
        setup_s2_crossing_priority(engine)
    elif name_lower in ["s3", "s3_narrow", "narrow", "narrow_aisle_headway"]:
        setup_s3_narrow_aisle_headway(engine)
    elif name_lower in ["s4", "s4_blocked", "obstacle", "dynamic_obstacle"]:
        setup_s4_dynamic_obstacle(engine)
        event_callback = _make_s4_callback()
    elif name_lower in ["s5", "s5_failure", "failure", "fault_recovery"]:
        setup_s5_robot_failure_recovery(engine)
        event_callback = _make_s5_callback()
    elif name_lower in ["s6", "s6_comms", "comms", "comms_recovery"]:
        setup_s6_comms_degradation(engine)
        event_callback = _make_s6_callback()
    else:
        print(f"Unknown scenario '{scenario_name}', defaulting to Multi-Task Hungarian.")
        setup_s1_multi_task_hungarian(engine)

    total_frames = int(duration_sec * fps)
    dt_frame = 1.0 / fps
    engine.dt = dt_frame
    print_frames = max(1, int(2.0 * fps))

    print(f"\n[SCENARIO RUNNER] Executing '{scenario_name}' ({duration_sec}s, {total_frames} frames @ {fps} FPS)...")
    print(f"[SCENARIO RUNNER] Tick loop: engine.step() (unified ? deadlock detection active every frame)")

    # Initial Hungarian allocation at t=0
    engine.run_hungarian_allocation()

    # F3: Single authoritative tick loop ? engine.step() handles everything including
    # deadlock detection. Scenario events flow through event_callback.
    for f in range(total_frames):
        engine.step(frame=float(f), event_callback=event_callback)

        if f % print_frames == 0:
            t = engine.current_time
            print(f"\n[Frame {f:4d} | t={t:.2f}s]")
            for a in engine.agents:
                tel = a.controller.get_telemetry()
                pos = tel["actual_position"]
                task_str = a.current_task.task_id if a.current_task else "IDLE"
                stopped_str = " STOPPED" if tel["is_stopped"] else ""
                waiting_str = f" waiting_on={a.waiting_on}" if a.waiting_on else ""
                print(f"  ? {a.robot_id}: Pos=({pos[0]:.2f},{pos[1]:.2f})"
                      f"  Hdg={tel['actual_heading']:.0f}?  v={tel['linear_velocity']:.2f}m/s"
                      f"  Status={a.state.status.value}  Task={task_str}"
                      f"{stopped_str}{waiting_str}")

    # Finalize stage
    if engine.stage:
        engine.stage.SetStartTimeCode(0.0)
        engine.stage.SetEndTimeCode(float(total_frames))
        engine.stage.SetTimeCodesPerSecond(fps)
        engine.stage.Save()
        print(f"\n[SCENARIO RUNNER] ? Saved {total_frames} keyframes to {engine.usd_path}")

        # Synchronize to Downloads if present
        downloads_usd = r"C:\Users\goruv\Downloads\simulation5.usd"
        if os.path.exists(downloads_usd):
            try:
                from omni_usd_env import Usd, UsdGeom
                dst_stage = Usd.Stage.Open(downloads_usd)
                if dst_stage:
                    clear_all_stage_time_samples(dst_stage)
                    for r_id in engine.robot_ids:
                        for prefix in ["/World/Warehouse/Robots", "/World/P_DYNEX_Depot/Robots"]:
                            path = f"{prefix}/{r_id}"
                            s_prim = engine.stage.GetPrimAtPath(path)
                            d_prim = dst_stage.GetPrimAtPath(path)
                            if s_prim.IsValid() and d_prim.IsValid():
                                s_xf = UsdGeom.Xformable(s_prim)
                                d_xf = UsdGeom.Xformable(d_prim)
                                for s_op in s_xf.GetOrderedXformOps():
                                    d_op = next((o for o in d_xf.GetOrderedXformOps() if o.GetOpName() == s_op.GetOpName()), None)
                                    if d_op:
                                        d_attr = d_op.GetAttr()
                                        d_attr.Clear()
                                        def_val = s_op.Get()
                                        if def_val is not None:
                                            d_op.Set(def_val)
                                        for t in s_op.GetTimeSamples():
                                            d_op.Set(s_op.Get(time=t), time=t)
                    dst_stage.SetStartTimeCode(0.0)
                    dst_stage.SetEndTimeCode(float(total_frames))
                    dst_stage.SetTimeCodesPerSecond(fps)
                    dst_stage.Save()
                    print(f"[SCENARIO RUNNER] ? Synchronized to {downloads_usd} via USD API")
            except Exception as e:
                print(f"[SCENARIO RUNNER] Warning syncing to Downloads: {e}")

    print("\n" + "=" * 80)
    print(f"? SCENARIO '{scenario_name.upper()}' BAKED SUCCESSFULLY TO OMNIVERSE!")
    print("=" * 80)


# ---------------------------------------------------------------------------
# F2 FIX: __main__ entry point ? makes scenarios directly runnable
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    VALID_SCENARIOS = {
        "s1": "Multi-Task Hungarian Allocation (6 robots, 2 waves)",
        "s2": "Crossing Priority & Intersection Yielding",
        "s3": "Narrow Aisle Headway & Convoy",
        "s4": "Dynamic Obstacle Blockage & Space-Time A* Detour",
        "s5": "Robot Failure & Automated Hungarian Task Recovery",
        "s6": "Communication Packet Loss, Safe Mode & Auto-Resume",
    }

    parser = argparse.ArgumentParser(
        description="SIH-AMR Omniverse Scenario Runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join(f"  {k}: {v}" for k, v in VALID_SCENARIOS.items())
    )
    parser.add_argument(
        "scenario",
        nargs="?",
        default="s1",
        help="Scenario key: s1 s2 s3 s4 s5 s6  (default: s1)"
    )
    parser.add_argument(
        "--duration", "-d",
        type=float,
        default=25.0,
        help="Simulation duration in seconds  (default: 25.0)"
    )
    parser.add_argument(
        "--fps", "-f",
        type=float,
        default=60.0,
        help="Frames per second for USD keyframe baking  (default: 60.0)"
    )
    args = parser.parse_args()

    print(f"\n{'='*80}")
    print(f"SIH-AMR Phase 1 ? Omniverse Scenario: {args.scenario.upper()}")
    if args.scenario.lower() in VALID_SCENARIOS:
        print(f"  {VALID_SCENARIOS[args.scenario.lower()]}")
    print(f"  Duration: {args.duration}s @ {args.fps} FPS")
    print(f"{'='*80}\n")

    run_scenario(args.scenario, duration_sec=args.duration, fps=args.fps)
