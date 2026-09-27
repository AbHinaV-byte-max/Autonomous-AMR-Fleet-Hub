"""
Decentralized AMR Agent Module
Connects the real SIH-AMR algorithms (Hungarian allocation, Space-Time A*,
Reservation Table, Priority Calculator, Deadlock Detection, Edge-AI Policy, and Safety Arbiter)
directly to the physical Omniverse robot controller (OmniAMRController).
"""

import os
import sys
import math
import time
from typing import List, Tuple, Dict, Set, Optional, Any

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

# Import SIH-AMR Reference Architecture Components
from ref_sih_amr.models import RobotState, RobotStatus, Intent, IntentMessage, Task, TaskStatus
from ref_sih_amr.robot.planner import AStarPlanner
from ref_sih_amr.robot.coordination import (
    ReservationTable, PriorityCalculator, detect_deadlock,
    check_vertex_conflict, check_edge_swap
)
from ref_sih_amr.robot.edge_policy import DeterministicPolicy, PolicyFeatures
from ref_sih_amr.comms.channel import PubSubChannel
from omni_amr_controller import OmniAMRController
from omni_warehouse_nav import OmniWarehouseNavMap


class SafetyArbiter:
    """
    Authoritative safety arbiter enforcing non-negotiable physical constraints
    over Edge-AI recommendations (Section 9.2).
    """
    def __init__(self, robot_id: str):
        self.robot_id = robot_id

    def arbitrate(self, ai_action: str, safety_context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Safety Rules:
        1. If in safe mode or comms lost -> HARD STOP
        2. If active conflict detected -> MUST WAIT / YIELD (cannot CONTINUE)
        3. If obstacle directly blocking next waypoint -> MUST REROUTE / STOP
        4. If in deadlock -> MUST REPLAN / YIELD
        """
        if safety_context.get("is_in_safe_mode", False):
            return {"final_action": "WAIT", "override": ai_action != "WAIT", "reason": "Communication safe mode active"}

        if safety_context.get("is_blocked", False):
            return {"final_action": "REROUTE", "override": ai_action != "REROUTE", "reason": "Corridor obstacle detected"}

        if safety_context.get("has_active_conflict", False) and ai_action == "CONTINUE":
            return {"final_action": "WAIT", "override": True, "reason": "Safety override: Unresolved space-time conflict"}

        # Approved
        return {"final_action": ai_action, "override": False, "reason": "AI recommendation aligns with safety constraints"}


class DecentralizedAMRAgent:
    """
    Independent Autonomous Mobile Robot agent running locally in Omniverse.
    Maintains its own state, goals, path, intent, peer models, and edge policy.
    """
    def __init__(self, robot_id: str, stage, nav_map: OmniWarehouseNavMap,
                 comms_channel: PubSubChannel, shared_reservation_table: ReservationTable,
                 t_degraded: float = 2.0, t_safe_mode: float = 5.0):
        self.robot_id = robot_id
        self.stage = stage
        self.nav_map = nav_map
        self.comms = comms_channel
        self.reservation_table = shared_reservation_table
        self.t_degraded = float(t_degraded)
        self.t_safe_mode = float(t_safe_mode)

        # Kinematic physical controller in Omniverse
        self.controller = OmniAMRController(robot_id=robot_id, stage=stage)

        # Core Algorithmic Components
        self.planner = AStarPlanner()
        self.priority_calc = PriorityCalculator()
        self.edge_policy = DeterministicPolicy()
        self.safety_arbiter = SafetyArbiter(robot_id=robot_id)

        # Agent Observable State
        self.state = RobotState(
            robot_id=robot_id,
            timestamp=0.0,
            position=(self.controller.actual_x, self.controller.actual_y),
            heading=self.controller.actual_heading,
            velocity=0.0,
            battery=100.0,
            current_task_id=None,
            task_priority=0,
            status=RobotStatus.IDLE
        )

        self.current_task: Optional[Task] = None
        self.current_path_nav: List[Tuple[int, int]] = []
        self.current_path_world: List[Tuple[float, float]] = []

        self.wait_time: float = 0.0
        self.waiting_on: Optional[str] = None
        self.peer_states: Dict[str, IntentMessage] = {}
        self.last_seen: Dict[str, float] = {}
        self.is_in_safe_mode: bool = False
        self.seq: int = 0
        self.confidence: float = 0.95
        # Per-peer comm state tracking for transition-only logging (Phase 2D fix)
        # States: 'normal', 'degraded', 'safe_mode'
        self._peer_comm_state: Dict[str, str] = {}
        # Phase 4F: Track per-peer PROCEED state to suppress repeated logs.
        # Maps peer_id -> True if we are currently in the "higher-priority PROCEED" state for that peer.
        self._proceeding_on: Dict[str, bool] = {}
        # Phase 4F: Track safety-stop state per peer to suppress HARD_STOP spam.
        self._safety_stopped_on: Dict[str, bool] = {}

    def assign_task(self, task: Task, current_time: float):
        """Assigns a task to this AMR and plans an A* space-time path to the pickup point."""
        self.current_task = task
        self.state.current_task_id = task.task_id
        self.state.task_priority = task.priority
        self.state.status = RobotStatus.MOVING
        task.status = TaskStatus.ASSIGNED
        task.assigned_robot_id = self.robot_id

        # Plan to pickup
        self.plan_path_to_world_target(task.pickup_cell, current_time)

    def plan_path_to_world_target(self, target_world: Tuple[float, float], current_time: float) -> bool:
        """
        Executes real Space-Time A* on the Omniverse navigation map from the robot's ACTUAL position.
        """
        if self.state.status == RobotStatus.OFFLINE or self.is_in_safe_mode:
            return False

        start_nav = self.nav_map.world_to_nav(self.controller.actual_x, self.controller.actual_y)
        goal_nav = self.nav_map.world_to_nav(target_world[0], target_world[1])

        # Release existing reservations
        self.reservation_table.expire(self.robot_id, after_time=current_time)

        # Plan space-time collision-free path
        nav_path = self.planner.plan(
            start=start_nav,
            goal=goal_nav,
            costmap=self.nav_map,
            reservation_table=self.reservation_table,
            start_time=current_time,
            robot_id=self.robot_id
        )

        if not nav_path:
            print(f"[PATH] {self.robot_id}: start={start_nav} goal={goal_nav} -> NO PATH FOUND (blocked or unreachable)")
            self.state.status = RobotStatus.WAITING
            self.controller.clear_path()
            return False

        # Diagnostic: log the plan
        print(f"[PATH] {self.robot_id}: start={start_nav} goal={goal_nav}  path_length={len(nav_path)} cells  target_world={target_world}")

        # Convert navigation path to world coordinates for kinematic controller
        world_path = [self.nav_map.nav_to_world(cell[0], cell[1]) for cell in nav_path]
        
        # If first waypoint is very close to current position, pop it
        if world_path:
            d0 = math.sqrt((world_path[0][0] - self.controller.actual_x)**2 + (world_path[0][1] - self.controller.actual_y)**2)
            if d0 < 0.25:
                world_path.pop(0)

        if world_path:
            world_path[-1] = (float(target_world[0]), float(target_world[1]))
        else:
            world_path = [(float(target_world[0]), float(target_world[1]))]

        self.current_path_nav = nav_path
        self.current_path_world = world_path

        # Commit reservations to the shared space-time table
        self.reservation_table.commit(self.robot_id, nav_path, current_time + 1.0)
        self.state.planned_path = nav_path
        self.state.status = RobotStatus.MOVING

        # Pass physical waypoints to kinematic controller
        self.controller.set_path(world_path)
        return True

    def broadcast_intent(self, current_time: float):
        """Broadcasts local AMR intent & state to peers over PubSubChannel."""
        if self.state.status == RobotStatus.OFFLINE:
            return

        next_cell = self.current_path_nav[0] if self.current_path_nav else None
        intent_type = Intent.MOVE
        if self.state.status == RobotStatus.WAITING:
            intent_type = Intent.WAIT
        elif self.state.status == RobotStatus.REROUTING:
            intent_type = Intent.REROUTE

        msg = IntentMessage(
            robot_id=self.robot_id,
            seq=self.seq,
            timestamp=current_time,
            position=(self.controller.actual_x, self.controller.actual_y),
            velocity=self.controller.actual_v,
            intent=intent_type,
            next_intersection=next_cell,
            task_id=self.state.current_task_id,
            priority=self.state.task_priority,
            planned_path=self.current_path_nav[:10],
            reservation_horizon=10.0,
            battery=self.state.battery,
            waiting_on=self.waiting_on,
            heartbeat=current_time
        )
        self.seq += 1
        self.comms.send(msg)

    def process_peer_messages(self, current_time: float, dropped_peer: Optional[str] = None):
        """Processes incoming messages from peers; monitors communication quality."""
        messages = self.comms.receive()
        for msg in messages:
            if msg.robot_id == self.robot_id:
                continue
            if dropped_peer and msg.robot_id == dropped_peer:
                continue  # Injected packet loss

            self.peer_states[msg.robot_id] = msg
            self.last_seen[msg.robot_id] = current_time

        # Check for stale peers & communication degradation.
        # Phase 2D fix: log ONLY on state TRANSITIONS (normal->degraded, degraded->safe_mode,
        # degraded/safe_mode->restored).  Never log on repeated frames in the same state.
        comm_healthy = True
        for peer_id, last_t in list(self.last_seen.items()):
            age = current_time - last_t
            prev_state = self._peer_comm_state.get(peer_id, "normal")

            if age >= self.t_safe_mode:
                if prev_state != "safe_mode":
                    print(f"[COMMS] {self.robot_id}: peer={peer_id} age={age:.1f}s"
                          f" --> SAFE_MODE (t_safe_mode={self.t_safe_mode}s)  action=HARD_STOP")
                    self._peer_comm_state[peer_id] = "safe_mode"
                if not self.is_in_safe_mode:
                    self.is_in_safe_mode = True
                self.state.status = RobotStatus.DEGRADED
                self.controller.is_stopped = True
                comm_healthy = False

            elif age >= self.t_degraded:
                if prev_state == "normal":
                    print(f"[COMMS] {self.robot_id}: peer={peer_id} age={age:.1f}s"
                          f" --> DEGRADED (t_degraded={self.t_degraded}s)")
                    self._peer_comm_state[peer_id] = "degraded"
                self.state.status = RobotStatus.DEGRADED
                comm_healthy = False

            else:
                # Peer heartbeat is fresh -- restore if previously degraded
                if prev_state in ("degraded", "safe_mode"):
                    print(f"[COMMS] {self.robot_id}: peer={peer_id} age={age:.1f}s"
                          f" --> RESTORED (was {prev_state})")
                    self._peer_comm_state[peer_id] = "normal"

        if comm_healthy and self.is_in_safe_mode:
            print(f"[COMMS] {self.robot_id}: All peers restored  --> resuming MOVING")
            self.is_in_safe_mode = False
            self.state.status = RobotStatus.MOVING
            self.controller.is_stopped = False

    def evaluate_edge_ai_and_safety(self, current_time: float) -> str:
        """
        Extracts 9-feature policy vector, computes Edge-AI recommendation,
        and runs through the authoritative Safety Arbiter.
        """
        # Find distance to nearest peer
        min_peer_dist = 999.0
        rel_vel = 0.0
        for peer_id, peer_msg in self.peer_states.items():
            px, py = peer_msg.position
            dist = math.sqrt((px - self.controller.actual_x)**2 + (py - self.controller.actual_y)**2)
            if dist < min_peer_dist:
                min_peer_dist = dist
                rel_vel = abs(self.controller.actual_v - peer_msg.velocity)

        # Check obstacle in next waypoint
        local_obs_flag = 0.0
        target = self.controller.waypoints[self.controller.current_waypoint_idx] if self.controller.waypoints and self.controller.current_waypoint_idx < len(self.controller.waypoints) else None
        if target and not self.nav_map.is_walkable(target[0], target[1]):
            local_obs_flag = 1.0

        # Time since freshest peer message
        peer_comm_freshness = 0.0
        if self.last_seen:
            freshest_t = max(self.last_seen.values())
            peer_comm_freshness = current_time - freshest_t

        features = PolicyFeatures(
            dist_to_nearest_peer=min_peer_dist,
            relative_velocity=rel_vel,
            time_to_conflict=min_peer_dist / max(0.1, self.controller.actual_v + 0.1),
            intersection_occupancy=1.0 if min_peer_dist < 2.0 else 0.0,
            queue_length=float(len(self.peer_states)),
            local_obstacle_flag=local_obs_flag,
            task_urgency=float(self.state.task_priority),
            battery=self.state.battery,
            peer_comm_freshness=peer_comm_freshness
        )

        ai_action = self.edge_policy.decide(features)

        # Safety Arbiter check
        safety_context = {
            "is_in_safe_mode": self.is_in_safe_mode,
            "is_blocked": local_obs_flag > 0.5,
            "has_active_conflict": self.waiting_on is not None,
        }

        arbitration = self.safety_arbiter.arbitrate(ai_action, safety_context)
        final_action = arbitration["final_action"]

        if final_action in ["WAIT", "YIELD"]:
            self.controller.is_stopped = True
            self.state.status = RobotStatus.WAITING
        elif final_action == "REROUTE":
            if self.current_task:
                target_pos = self.current_task.dropoff_cell if self.state.status == RobotStatus.MOVING else self.current_task.pickup_cell
                self.plan_path_to_world_target(target_pos, current_time)
        elif final_action == "CONTINUE":
            if not self.is_in_safe_mode:
                self.controller.is_stopped = False
                self.state.status = RobotStatus.MOVING

        return final_action

    def resolve_conflicts_and_coordination(self, current_time: float, all_agents: List['DecentralizedAMRAgent']):
        """
        Detects space-time conflicts, prioritizes via PriorityCalculator,
        and detects deadlocks via detect_deadlock.

        Phase 3A Safety Fix:
        SAFETY_STOP_DIST (1.0m) enforces a hard physical stop independent of priority.
        Priority arbitration governs only who resumes first after the gap opens.
        This prevents the higher-priority robot from driving through the lower-priority
        robot's occupied cell.
        """
        if self.state.status == RobotStatus.OFFLINE:
            return

        # ---------------------------------------------------------------
        # HARD SAFETY STOP (Phase 3A)
        # If this robot is moving and ANY peer is within SAFETY_STOP_DIST,
        # stop unconditionally before contact.
        # SAFETY_STOP_DIST > COLLISION_THRESHOLD (0.6m) so we stop before impact.
        # The deadlock resolver handles escape: it replans the victim with the
        # peer's footprint (radius-2 ring) as a temporary nav obstacle, forcing
        # A* to find a route through an adjacent walkable row.
        # ---------------------------------------------------------------
        SAFETY_STOP_DIST = 1.0  # metres: must be > collision threshold (0.6m)
        my_v = self.controller.actual_v

        if my_v > 0.01:  # Only when actually moving forward
            for peer_id, peer_msg in self.peer_states.items():
                px, py = peer_msg.position
                dist = math.hypot(px - self.controller.actual_x, py - self.controller.actual_y)
                if dist < SAFETY_STOP_DIST:
                    # Phase 4F: log HARD_STOP only on transition into stopped state for this peer
                    if not self._safety_stopped_on.get(peer_id, False):
                        print(f"[SAFETY] {self.robot_id}: peer={peer_id} dist={dist:.3f}m < "
                              f"{SAFETY_STOP_DIST}m  -> HARD_STOP")
                        self._safety_stopped_on[peer_id] = True
                    self.controller.is_stopped = True
                    self.state.status = RobotStatus.WAITING
                    # Record waiting_on so deadlock detector can see the circular wait
                    if self.waiting_on is None:
                        self.waiting_on = peer_id
                    return
                else:
                    # Peer moved far enough away — clear the safety-stop flag for this peer
                    if self._safety_stopped_on.get(peer_id, False):
                        self._safety_stopped_on[peer_id] = False

        conflict_peer = None

        # 1. Check space-time reservation table over lookahead window
        for dt_lookahead in [1, 2, 3]:
            t_target = round(current_time) + dt_lookahead
            for cell in self.current_path_nav[:4]:
                claimer = self.reservation_table.get_claimer(cell, float(t_target))
                if claimer and claimer != self.robot_id:
                    conflict_peer = claimer
                    break
            if conflict_peer:
                break

        # 2. Check trajectory proximity and intersection conflict
        if not conflict_peer:
            for peer_id, peer_msg in self.peer_states.items():
                px, py = peer_msg.position
                dist = math.hypot(px - self.controller.actual_x, py - self.controller.actual_y)
                if dist < 4.0:
                    my_next_cells = set(self.current_path_nav[:8])
                    peer_next_cells = set(peer_msg.planned_path[:8])
                    if my_next_cells.intersection(peer_next_cells) or dist < 2.0:
                        conflict_peer = peer_id
                        break

        if conflict_peer:
            # Deterministic Priority calculation from ref_sih_amr
            peer_agent = next((a for a in all_agents if a.robot_id == conflict_peer), None)
            if peer_agent:
                my_priority = self.priority_calc.calculate(
                    urgency=float(self.state.task_priority),
                    wait_time=self.wait_time,
                    battery=self.state.battery,
                    distance=float(len(self.current_path_nav)),
                    robot_id=self.robot_id
                )
                peer_priority = self.priority_calc.calculate(
                    urgency=float(peer_agent.state.task_priority),
                    wait_time=peer_agent.wait_time,
                    battery=peer_agent.state.battery,
                    distance=float(len(peer_agent.current_path_nav)),
                    robot_id=peer_agent.robot_id
                )

                if my_priority < peer_priority:
                    # Lower priority yields cleanly
                    if self.waiting_on != conflict_peer:
                        # Log only on first yield (not every repeated frame)
                        print(f"[CONFLICT] t={current_time:.2f}s  cell=~{self.current_path_nav[:1]}")
                        print(f"[CONFLICT]   {self.robot_id} priority={my_priority[0]:.3f}  vs  {conflict_peer} priority={peer_priority[0]:.3f}")
                        print(f"[CONFLICT]   decision={self.robot_id} YIELD -> waiting_on={conflict_peer}")
                    self.waiting_on = conflict_peer
                    self.wait_time += 0.05
                    self.controller.is_stopped = True
                    self.state.status = RobotStatus.WAITING
                    return
                else:
                    # Higher priority: PROCEED
                    # Phase 4F: log PROCEED only once per transition (first frame in this state for this peer)
                    already_proceeding = self._proceeding_on.get(conflict_peer, False)
                    was_waiting = self.waiting_on is not None
                    if not already_proceeding:
                        print(f"[CONFLICT] t={current_time:.2f}s  {self.robot_id} priority={my_priority[0]:.3f} > {conflict_peer} priority={peer_priority[0]:.3f}  -> PROCEED")
                        self._proceeding_on[conflict_peer] = True
                    if was_waiting:
                        self.waiting_on = None
                        self.wait_time = 0.0
                    if not self.is_in_safe_mode:
                        self.controller.is_stopped = False
                        self.state.status = RobotStatus.MOVING
                    return

        # If previous conflict cleared, resume motion
        # Phase 4F: also clear the PROCEED state tracker for all resolved peers
        if conflict_peer is None:
            # No conflict detected this frame; clear all PROCEED flags
            self._proceeding_on.clear()
        if self.waiting_on:
            prev = self.waiting_on
            self.waiting_on = None
            if not self.is_in_safe_mode and len(self.controller.waypoints) > 0 and self.controller.current_waypoint_idx < len(self.controller.waypoints):
                print(f"[CONFLICT] t={current_time:.2f}s  {self.robot_id}: conflict with {prev} cleared  -> RESUMING")
                self.controller.is_stopped = False
                self.state.status = RobotStatus.MOVING

    def trigger_failure(self, current_time: float):
        """Simulates robot hardware/system failure."""
        self.state.status = RobotStatus.OFFLINE
        self.controller.is_failed = True
        self.controller.is_stopped = True
        self.reservation_table.expire(self.robot_id)
        if self.current_task:
            self.current_task.status = TaskStatus.RECOVERABLE
            self.current_task.assigned_robot_id = None
            self.current_task = None

    def step(self, dt: float, current_time: float, frame: Optional[float] = None):
        """Advances physical kinematics in Omniverse and updates observable state."""
        telemetry = self.controller.step(dt, current_time, frame=frame)

        # Update observable state from actual telemetry
        self.state.position = (telemetry["actual_position"][0], telemetry["actual_position"][1])
        self.state.heading = telemetry["actual_heading"]
        self.state.velocity = telemetry["linear_velocity"]
        self.state.timestamp = current_time

        # Check goal reached
        if self.current_task:
            target_goal = self.current_task.dropoff_cell if self.current_task.status == TaskStatus.IN_PROGRESS else self.current_task.pickup_cell
            dist_to_goal = math.hypot(self.controller.actual_x - target_goal[0], self.controller.actual_y - target_goal[1])
            reached = (len(self.controller.waypoints) > 0 and self.controller.current_waypoint_idx >= len(self.controller.waypoints)) or (dist_to_goal <= 0.45)
            
            if reached:
                if self.current_task.status == TaskStatus.ASSIGNED:
                    print(f"[{self.robot_id} @ t={current_time:.2f}s] REACHED PICKUP {self.current_task.pickup_cell} for {self.current_task.task_id} -> En route to Dropoff {self.current_task.dropoff_cell}")
                    self.current_task.status = TaskStatus.IN_PROGRESS
                    self.plan_path_to_world_target(self.current_task.dropoff_cell, current_time)
                elif self.current_task.status == TaskStatus.IN_PROGRESS:
                    print(f"[{self.robot_id} @ t={current_time:.2f}s] [OK] COMPLETED {self.current_task.task_id} at Dropoff {self.current_task.dropoff_cell}! Ready for Hungarian reallocation.")
                    self.current_task.status = TaskStatus.COMPLETED
                    self.state.status = RobotStatus.IDLE
                    self.current_task = None
                    self.controller.clear_path()
                    self.reservation_table.expire(self.robot_id)
