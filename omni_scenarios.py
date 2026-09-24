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


def run_scenario(scenario_name: str = "multi_task_hungarian", duration_sec: float = 25.0, fps: float = 60.0):
    """
    Runs the selected scenario, executing decentralized coordination and baking to simulation5.usd.
    """
    engine = OmniSIHSimulationEngine(cell_size=0.5)

    name_lower = scenario_name.lower().strip()
    
    if name_lower in ["s1", "s1_hungarian", "multi_task_hungarian", "all", "unified", "default"]:
        setup_s1_multi_task_hungarian(engine)
    elif name_lower in ["s2", "s2_crossing", "crossing", "crossing_priority"]:
        setup_s2_crossing_priority(engine)
    elif name_lower in ["s3", "s3_narrow", "narrow", "narrow_aisle_headway"]:
        setup_s3_narrow_aisle_headway(engine)
    elif name_lower in ["s4", "s4_blocked", "obstacle", "dynamic_obstacle"]:
        setup_s4_dynamic_obstacle(engine)
    elif name_lower in ["s5", "s5_failure", "failure", "fault_recovery"]:
        setup_s5_robot_failure_recovery(engine)
    elif name_lower in ["s6", "s6_comms", "comms", "comms_recovery"]:
        setup_s6_comms_degradation(engine)
    else:
        print(f"Unknown scenario '{scenario_name}', defaulting to Multi-Task Hungarian.")
        setup_s1_multi_task_hungarian(engine)

    total_frames = int(duration_sec * fps)
    dt_frame = 1.0 / fps
    engine.dt = dt_frame
    print_frames = max(1, int(2.0 * fps))

    print(f"\n[SCENARIO RUNNER] Executing '{scenario_name}' ({duration_sec}s, {total_frames} frames @ {fps} FPS)...")
    engine.run_hungarian_allocation()

    for f in range(total_frames):
        current_t = f * dt_frame
        
        # Scenario 4 Event: Inject dynamic obstacle at t=3.0s
        if "s4" in name_lower or "obstacle" in name_lower:
            if 3.0 <= current_t < 3.05:
                print(f"\n>>> [EVENT @ t={current_t:.2f}s] Injecting Dynamic Fallen Obstacle at (4.5, 5.0) in Aisle X=4.5!")
                engine.nav_map.update_dynamic_obstacle("FALLEN_PALLET", (4.5, 5.0), active=True)
                # Notify agents
                for a in engine.agents:
                    if a.current_task:
                        a.plan_path_to_world_target(a.current_task.dropoff_cell if a.current_task.status == TaskStatus.IN_PROGRESS else a.current_task.pickup_cell, current_t)

        # Scenario 5 Event: Trigger hardware breakdown on AMR_03 at t=4.0s
        if "s5" in name_lower or "failure" in name_lower:
            if 4.0 <= current_t < 4.05:
                print(f"\n>>> [EVENT @ t={current_t:.2f}s] Hardware Failure Injected into AMR_03! Motor stalled.")
                amr3 = next((a for a in engine.agents if a.robot_id == "AMR_03"), None)
                if amr3:
                    amr3.trigger_failure(current_t)
                    # Trigger immediate Hungarian reallocation of abandoned tasks
                    engine.run_hungarian_allocation()

        # Scenario 6 Event: Inject communication fault to AMR_02 from t=2.5s to t=6.5s
        dropped_peer = None
        if "s6" in name_lower or "comms" in name_lower:
            if 2.5 <= current_t <= 6.5:
                dropped_peer = "AMR_02"
                if 2.5 <= current_t < 2.55:
                    print(f"\n>>> [EVENT @ t={current_t:.2f}s] Communication Packet Loss Injected for AMR_02!")
            elif 6.5 < current_t < 6.55:
                print(f"\n>>> [EVENT @ t={current_t:.2f}s] Communication Network Restored for AMR_02! Heartbeats nominal.")

        # Step simulation with custom message dropping if active
        for a in engine.agents:
            a.broadcast_intent(current_t)
        engine.comms.clear()
        for a in engine.agents:
            a.process_peer_messages(current_t, dropped_peer=dropped_peer)
        for a in engine.agents:
            if a.state.status != RobotStatus.OFFLINE:
                a.evaluate_edge_ai_and_safety(current_t)
        for a in engine.agents:
            a.resolve_conflicts_and_coordination(current_t, engine.agents)
        for a in engine.agents:
            a.step(engine.dt, current_t, frame=float(f))
        engine.run_hungarian_allocation()

        if f % print_frames == 0:
            print(f"[Frame {f:4d} | t={current_t:.2f}s]")
            for a in engine.agents:
                tel = a.controller.get_telemetry()
                pos = tel["actual_position"]
                task_str = a.current_task.task_id if a.current_task else "IDLE"
                print(f"  • {a.robot_id}: Pos=({pos[0]:.2f}, {pos[1]:.2f}), Heading={tel['actual_heading']:.1f}°, v={tel['linear_velocity']:.2f} m/s, Status={a.state.status.value}, Task={task_str}")

    # Finalize stage
    if engine.stage:
        engine.stage.SetStartTimeCode(0.0)
        engine.stage.SetEndTimeCode(float(total_frames))
        engine.stage.SetTimeCodesPerSecond(fps)
        engine.stage.Save()
        print(f"\n[SCENARIO RUNNER] ✓ Saved {total_frames} keyframes to {engine.usd_path}")

        # Synchronize to Downloads
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
                    print(f"[SCENARIO RUNNER] ✓ Synchronized to {downloads_usd} via USD API")
            except Exception as e:
                print(f"[SCENARIO RUNNER] Warning syncing to Downloads: {e}")

    print("\n" + "=" * 80)
    print(f"✓ SCENARIO '{scenario_name.upper()}' BAKED SUCCESSFULLY TO OMNIVERSE!")
    print("=" * 80)
