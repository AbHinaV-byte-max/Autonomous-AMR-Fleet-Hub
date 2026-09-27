"""
Unified Omniverse SIH-AMR Real Runtime Engine
Directly runs decentralized multi-AMR coordination on simulation5.usd in NVIDIA Omniverse.
Bakes continuous multi-pattern Hungarian-allocated workflows into USD TimeSamples for Omniverse timeline playback.
"""

import os
import sys
import time
import math
import shutil
from typing import List, Dict, Any, Optional, Tuple

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

from omni_usd_env import Usd, UsdGeom, Gf, Sdf, USD_AVAILABLE
from ref_sih_amr.models import RobotState, RobotStatus, Task, TaskStatus
from ref_sih_amr.allocator.hungarian import HungarianAllocator
from ref_sih_amr.robot.planner import AStarPlanner
from ref_sih_amr.robot.coordination import ReservationTable, detect_deadlock
from ref_sih_amr.comms.channel import PubSubChannel

from omni_warehouse_nav import OmniWarehouseNavMap
from omni_amr_controller import OmniAMRController, clear_all_stage_time_samples
from omni_amr_agent import DecentralizedAMRAgent


class OmniHungarianCostPlanner:
    """Computes exact Manhattan corridor transit distance for Hungarian fleet allocation."""
    def plan(self, start, goal, costmap=None, *args, **kwargs):
        dx = abs(goal[0] - start[0])
        dy = abs(goal[1] - start[1])
        steps = max(1, int(round((dx + dy) * 2)))
        return [(0, 0)] * steps


class OmniSIHSimulationEngine:
    """
    Executes the true closed-loop SIH-AMR decentralized coordination system
    directly inside NVIDIA Omniverse using simulation5.usd as the physical runtime.
    """
    def __init__(self, usd_path: Optional[str] = None, dt: float = 0.05, cell_size: float = 0.5):
        self.usd_path = usd_path or os.path.join(REPO_ROOT, "simulation5.usd")
        self.dt = float(dt)
        self.current_time = 0.0
        
        self.stage = None
        if USD_AVAILABLE and os.path.exists(self.usd_path):
            # LoadNone prevents USD from trying to load external Omniverse CDN payloads
            # that are not accessible offline, eliminating hundreds of warning messages.
            self.stage = Usd.Stage.Open(self.usd_path, Usd.Stage.LoadNone)
            print(f"[OMNI ENGINE] Opened OpenUSD stage: {self.usd_path}")
            # Clear old legacy time samples from previous iterations
            clear_all_stage_time_samples(self.stage)
            self.stage.SetStartTimeCode(0.0)
            self.stage.SetEndTimeCode(1200.0)
            self.stage.SetTimeCodesPerSecond(60.0)
        else:
            print(f"[OMNI ENGINE] Warning: Running without direct USD binding.")

        # 1. World & Navigation Map
        self.nav_map = OmniWarehouseNavMap(stage=self.stage, usd_path=self.usd_path, cell_size=cell_size)
        
        # 2. Inter-Robot Coordination Infrastructure
        self.comms = PubSubChannel()
        self.reservation_table = ReservationTable()
        self.allocator = HungarianAllocator(planner=OmniHungarianCostPlanner(), costmap=None)

        # 3. Decentralized AMR Agents (6-AMR Full Fleet)
        self.robot_ids = ["AMR_01", "AMR_02", "AMR_03", "AMR_04", "AMR_05", "AMR_06"]
        self.agents: List[DecentralizedAMRAgent] = [
            DecentralizedAMRAgent(
                robot_id=r_id,
                stage=self.stage,
                nav_map=self.nav_map,
                comms_channel=self.comms,
                shared_reservation_table=self.reservation_table
            )
            for r_id in self.robot_ids
        ]

        # Explicitly clear time samples for each controller's prim bindings and reset to spawn
        for a in self.agents:
            a.controller.clear_time_samples(reset_spawn=True)

        self.tasks: List[Task] = []
        self.completed_tasks = 0
        # Phase 3A: Per-victim deadlock replan cooldown.
        # Maps victim_id -> last_replan_time.  Suppresses repeated deadlock
        # triggers for the same cycle until DEADLOCK_COOLDOWN_SEC has elapsed.
        self._deadlock_last_replan: Dict[str, float] = {}
        self.DEADLOCK_COOLDOWN_SEC = 1.5

    def add_task(self, task_id: str, pickup_cell: Tuple[float, float], dropoff_cell: Tuple[float, float], priority: int = 1):
        """Register a warehouse logistics task."""
        task = Task(
            task_id=task_id,
            pickup_cell=pickup_cell,
            dropoff_cell=dropoff_cell,
            priority=priority,
            status=TaskStatus.QUEUED
        )
        self.tasks.append(task)

    def add_default_warehouse_tasks(self):
        """Populates rich, diverse warehouse tasks covering multiple aisles and corridors."""
        DROP_1 = (32.86, 23.92)  # East Drop Point 1
        DROP_2 = (-34.14, 22.13) # West Drop Point 2
        tasks_def = [
            ("TASK_01_EAST_PICK", (4.5, 5.0), DROP_1, 2),
            ("TASK_02_WEST_REPLENISH", (-34.14, 22.13), (-23.0, -18.0), 3),
            ("TASK_03_CROSSDOCK_EAST", (18.5, 5.0), DROP_1, 1),
            ("TASK_04_SORTATION_WEST", (-10.0, -4.5), DROP_2, 2),
            ("TASK_05_HEAVY_DEPOT", (25.5, -4.5), DROP_1, 2),
            ("TASK_06_BUFFER_LOOP", (-30.0, -4.5), DROP_2, 3),
            ("TASK_07_OUTBOUND_EXPRESS", (11.0, 5.0), DROP_1, 2),
            ("TASK_08_INBOUND_PUTAWAY", (-16.0, 5.0), DROP_2, 2),
            ("TASK_09_VNA_TRANSFER", (25.5, 5.0), DROP_1, 1),
            ("TASK_10_STAGING_RETURN", (-3.0, 5.0), DROP_2, 2),
            ("TASK_11_NORTH_TRANSFER", (18.5, 14.5), DROP_1, 2),
            ("TASK_12_SOUTH_RECYCLE", (-23.0, 5.0), DROP_2, 2),
        ]
        for tid, p, d, prio in tasks_def:
            self.add_task(tid, p, d, prio)

    def run_hungarian_allocation(self):
        """Runs the Hungarian Algorithm from ref_sih_amr using actual robot positions."""
        robot_states = [a.state for a in self.agents]
        queued_tasks = [t for t in self.tasks if t.status in (TaskStatus.QUEUED, TaskStatus.RECOVERABLE)]
        
        if not queued_tasks:
            return

        assignments = self.allocator.allocate(robot_states, queued_tasks)
        for r_id, t_id in assignments.items():
            agent = next((a for a in self.agents if a.robot_id == r_id), None)
            task = next((t for t in self.tasks if t.task_id == t_id), None)
            if agent and task:
                print(f"[HUNGARIAN @ t={self.current_time:.2f}s] Allocated {task.task_id} (P={task.priority}) to {agent.robot_id} -> Pickup: {task.pickup_cell}, Dropoff: {task.dropoff_cell}")
                agent.assign_task(task, self.current_time)

    def step(self, frame: Optional[float] = None,
             event_callback=None, dropped_peer: Optional[str] = None):
        """
        Executes one simulation tick across all decentralized agents and bakes to USD.

        Parameters
        ----------
        frame        : USD timeline frame number for keyframe baking (None = no baking).
        event_callback : Optional callable invoked BEFORE the agent loop so scenarios
                         can inject events (obstacles, failures, comms faults) at precise
                         times without duplicating the core tick logic.
                         Signature: event_callback(engine, current_time) -> Optional[str]
                         Return value (if any) is used as dropped_peer override.
        dropped_peer : robot_id whose broadcast messages are suppressed this tick
                       (comms fault injection).  event_callback return value wins.
        """
        # 0. Scenario event injection BEFORE physics tick
        if event_callback is not None:
            cb_result = event_callback(self, self.current_time)
            if cb_result is not None:
                dropped_peer = cb_result

        # 1. P2P Broadcast
        for agent in self.agents:
            agent.broadcast_intent(self.current_time)
            
        # 2. Swap comms channel buffer
        self.comms.clear()

        # 3. Process Peer Messages (with optional comms fault)
        for agent in self.agents:
            agent.process_peer_messages(self.current_time, dropped_peer=dropped_peer)

        # 4. Edge-AI Policy & Safety Arbitration
        for agent in self.agents:
            if agent.state.status != RobotStatus.OFFLINE:
                agent.evaluate_edge_ai_and_safety(self.current_time)

        # 5. Conflict Resolution & Priority Coordination
        for agent in self.agents:
            agent.resolve_conflicts_and_coordination(self.current_time, self.agents)

        # 6. Deadlock Detection across Wait-For Graph
        wait_graph = {a.robot_id: a.waiting_on for a in self.agents if a.waiting_on is not None}
        cycle = detect_deadlock(wait_graph)
        if cycle:
            victim_id = cycle[0]

            # Cooldown: suppress re-triggering for the same victim until enough
            # time has elapsed since the last safe replan.
            last_t = self._deadlock_last_replan.get(victim_id, -999.0)
            if self.current_time - last_t < self.DEADLOCK_COOLDOWN_SEC:
                pass  # Still in cooldown window — skip replan
            else:
                print(f"[DEADLOCK] cycle={cycle}")
                victim = next((a for a in self.agents if a.robot_id == victim_id), None)
                if victim and victim.current_task:
                    # --- Phase 3A Safety Fix ---
                    # Determine correct replan target (pickup or dropoff depending on task phase)
                    from ref_sih_amr.models import TaskStatus
                    if victim.current_task.status == TaskStatus.IN_PROGRESS:
                        replan_target = victim.current_task.dropoff_cell
                    else:
                        replan_target = victim.current_task.pickup_cell

                    # Gather physical footprints of ALL other live robots as exclusion zone.
                    # radius=2 forces A* to use an adjacent row (1m clearance around peer body)
                    FOOTPRINT_RADIUS = 2  # cells; 2 * 0.5m/cell = 1.0m clearance
                    exclusion_cells = set()
                    for other in self.agents:
                        if other.robot_id == victim_id or other.state.status == RobotStatus.OFFLINE:
                            continue
                        ox = other.controller.actual_x
                        oy = other.controller.actual_y
                        cx, cy = self.nav_map.world_to_nav(ox, oy)
                        for dr in range(-FOOTPRINT_RADIUS, FOOTPRINT_RADIUS + 1):
                            for dc in range(-FOOTPRINT_RADIUS, FOOTPRINT_RADIUS + 1):
                                if abs(dr) + abs(dc) <= FOOTPRINT_RADIUS:
                                    exclusion_cells.add((cx + dc, cy + dr))

                    print(f"[DEADLOCK] victim={victim_id}  target={replan_target}  "
                          f"exclusion_zone={len(exclusion_cells)} cells  action=SAFE_REPLAN")

                    _EXCL_TAG = f"_deadlock_excl_{victim_id}"
                    for (ecx, ecy) in exclusion_cells:
                        self.nav_map.set_cell_blocked(ecx, ecy, tag=_EXCL_TAG)

                    try:
                        success = victim.plan_path_to_world_target(replan_target, self.current_time)
                    finally:
                        self.nav_map.clear_cells_by_tag(_EXCL_TAG)

                    if success:
                        # Replan succeeded: record cooldown time, clear victim's waiting_on
                        # so the deadlock graph resets properly.
                        self._deadlock_last_replan[victim_id] = self.current_time
                        victim.waiting_on = None
                        victim.wait_time = 0.0
                    else:
                        print(f"[DEADLOCK] {victim_id}: safe replan failed (cornered), "
                              f"backing off for {self.DEADLOCK_COOLDOWN_SEC}s")
                        self._deadlock_last_replan[victim_id] = self.current_time

        # 7. Physical Kinematic Step in Omniverse (writing time-sample keyframe)
        for agent in self.agents:
            agent.step(self.dt, self.current_time, frame=frame)

        # 8. Check for newly idle robots and run Hungarian allocation
        self.run_hungarian_allocation()

        self.current_time += self.dt

    def run(self, duration_sec: float = 20.0, print_interval_sec: float = 2.0, fps: float = 60.0):
        """
        Runs the simulation for a given duration, baking every frame into the Omniverse timeline.
        """
        total_frames = int(duration_sec * fps)
        dt_frame = 1.0 / fps
        self.dt = dt_frame
        print_frames = max(1, int(print_interval_sec * fps))

        print(f"\n[OMNI ENGINE] Starting closed-loop simulation ({duration_sec}s, {total_frames} frames @ {fps} FPS)...")
        
        # Initial Hungarian allocation at t=0
        self.run_hungarian_allocation()

        for f in range(total_frames):
            self.step(frame=float(f))
            if f % print_frames == 0:
                print(f"[Frame {f:4d} | t={self.current_time:.2f}s]")
                for a in self.agents:
                    tel = a.controller.get_telemetry()
                    pos = tel["actual_position"]
                    task_str = a.current_task.task_id if a.current_task else "IDLE"
                    print(f"  • {a.robot_id}: Pos=({pos[0]:.2f}, {pos[1]:.2f}), Heading={tel['actual_heading']:.1f}°, v={tel['linear_velocity']:.2f} m/s, Status={a.state.status.value}, Task={task_str}")

        if self.stage:
            self.stage.SetStartTimeCode(0.0)
            self.stage.SetEndTimeCode(float(total_frames))
            self.stage.SetTimeCodesPerSecond(fps)
            self.stage.Save()
            print(f"[OMNI ENGINE] ✓ Saved updated simulation state ({total_frames} keyframes) to {self.usd_path}")
            
            # Also keep Downloads/simulation5.usd in sync if present via USD API
            downloads_usd = os.path.join(os.path.expanduser("~"), "Downloads", "simulation5.usd")
            if os.path.exists(downloads_usd):
                try:
                    dst_stage = Usd.Stage.Open(downloads_usd)
                    if dst_stage:
                        for r_id in self.robot_ids:
                            for prefix in ["/World/Warehouse/Robots", "/World/P_DYNEX_Depot/Robots"]:
                                path = f"{prefix}/{r_id}"
                                s_prim = self.stage.GetPrimAtPath(path)
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
                        print(f"[OMNI ENGINE] ✓ Synchronized to {downloads_usd} via USD API")
                except Exception as e:
                    print(f"[OMNI ENGINE] Warning syncing to Downloads: {e}")
