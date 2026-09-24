from typing import Optional, Any, List
from models import RobotState, RobotStatus, Intent, IntentMessage, Task, TaskStatus
from interfaces import Planner, CommsChannel
from robot.coordination import ReservationTable, PriorityCalculator, check_vertex_conflict, check_edge_swap

# How many consecutive ticks a robot can wait on the same peer before
# proactively replanning (without needing a full cycle to be detected).
WAIT_REPLAN_THRESHOLD = 5

class LocalTaskManager:
    def __init__(self, robot_state: RobotState, planner: Planner, comms: CommsChannel, costmap: Any, strategy: str = "P1", event_logger: Any = None):
        self.state = robot_state
        self.planner = planner
        self.comms = comms
        self.costmap = costmap
        self.strategy = strategy
        self.event_logger = event_logger
        self.current_task: Optional[Task] = None
        self.seq = 0
        self.target_cell: Optional[tuple[int, int]] = None
        
        # Decentralized coordination
        self.reservation_table = ReservationTable()
        self.priority_calc = PriorityCalculator()
        self.wait_time = 0.0
        self.waiting_on: Optional[str] = None
        self.peer_states: dict[str, IntentMessage] = {}
        self.last_seen: dict[str, float] = {}
        # Tracks how many consecutive ticks this robot has been stuck
        # on the *same* peer — used for proactive replan before a full
        # cycle is detected by the global deadlock detector.
        self.wait_ticks_on_peer: int = 0
        self._prev_waiting_on: Optional[str] = None

        # CBS integration
        # When True, _replan() is suppressed — CBS owns all path decisions.
        self.cbs_mode: bool = False
        # Set to True when the robot reaches its pickup and switches to the
        # dropoff goal. The simulator checks this flag and triggers CBS replan.
        self.checkpoint_reached: bool = False

    def assign_task(self, task: Task):
        self.current_task = task
        self.state.current_task_id = task.task_id
        self.state.status = RobotStatus.MOVING
        task.status = TaskStatus.ASSIGNED
        task.assigned_robot_id = self.state.robot_id
        self.target_cell = task.pickup_cell
        self.wait_time = 0.0
        self.waiting_on = None
        self.wait_ticks_on_peer = 0
        self._prev_waiting_on = None
        self.checkpoint_reached = False
        if not self.cbs_mode:
            self._replan()

    def _replan(self):
        """Internal A* replan. Skipped when cbs_mode=True (CBS owns paths)."""
        if self.cbs_mode:
            return  # CBS will inject the path externally
        if self.target_cell:
            path = self.planner.plan(
                start=self.state.position, 
                goal=self.target_cell, 
                costmap=self.costmap,
                reservation_table=self.reservation_table,
                start_time=self.state.timestamp,
                robot_id=self.state.robot_id
            )
            current_int = (int(self.state.position[0]), int(self.state.position[1]))
            if path and path[0] == current_int:
                path.pop(0)
            self.state.planned_path = path
            
            # Commit the new path to our own reservation table
            if path:
                self.reservation_table.commit(self.state.robot_id, path, self.state.timestamp + 1)
        else:
            self.state.planned_path = []
            curr_pos = (int(self.state.position[0]), int(self.state.position[1]))
            self.reservation_table.commit(self.state.robot_id, [curr_pos] * 200, self.state.timestamp)

    def inject_path(self, path: List, goal: tuple):
        """
        Called by the CBS coordinator to load a pre-computed, conflict-free path.

        Strips the start cell from the path if it matches the robot's current
        position (the robot is already there — no need to 'move' to it).
        Also commits the new path to the shared reservation table so that
        the decentralised coordination layer stays in sync.
        """
        self.target_cell = goal
        current_int = (int(self.state.position[0]), int(self.state.position[1]))
        if path and (int(path[0][0]), int(path[0][1])) == current_int:
            path = path[1:]
        self.state.planned_path = list(path)
        self.wait_time = 0.0
        self.waiting_on = None
        self.wait_ticks_on_peer = 0
        self._prev_waiting_on = None
        if self.state.planned_path:
            self.state.status = RobotStatus.MOVING
            # Keep the reservation table in sync for the comms layer
            self.reservation_table.commit(
                self.state.robot_id, self.state.planned_path, self.state.timestamp + 1
            )

    def get_current_goal(self) -> Optional[tuple]:
        """Returns the robot's active goal cell (pickup or dropoff), or None."""
        return self.target_cell

    def _replan_fallback(self):
        """Emergency internal A* replan that ignores cbs_mode (e.g., total path failure)."""
        prev = self.cbs_mode
        self.cbs_mode = False
        self._replan()
        self.cbs_mode = prev

    def _update_peer_reservations(self, current_time: float):
        """Update local reservation table based on received intents."""
        messages = self.comms.receive()
        for msg in messages:
            if msg.robot_id == self.state.robot_id:
                continue
                
            self.last_seen[msg.robot_id] = msg.timestamp
            self.peer_states[msg.robot_id] = msg
            
            # Update reservation table
            if msg.planned_path:
                # Assuming intent messages broadcast the full planned path starting from timestamp + 1
                self.reservation_table.commit(msg.robot_id, msg.planned_path, msg.timestamp + 1)
            else:
                self.reservation_table.expire(msg.robot_id)

        # Check for degraded comms (Phase 4)
        degraded = False
        for peer_id, last_t in list(self.last_seen.items()):
            gap = current_time - last_t
            if gap > 10.0:  # Beyond heartbeat timeout — peer is offline, not delayed
                self.last_seen.pop(peer_id, None)
                self.peer_states.pop(peer_id, None)
                continue
            if gap > 3.0:  # 3 ticks threshold
                degraded = True
                break
                
        if degraded and self.state.status == RobotStatus.MOVING:
            self.state.status = RobotStatus.DEGRADED

        if not degraded and self.state.status == RobotStatus.DEGRADED:
            self.state.status = RobotStatus.MOVING

    def get_priority(self) -> tuple[float, str]:
        urgency = float(self.state.task_priority)
        dist = float(len(self.state.planned_path))
        return self.priority_calc.calculate(urgency, self.wait_time, self.state.battery, dist, self.state.robot_id)

    def _check_conflicts(self, current_time: float) -> bool:
        """Returns True if it's safe to move to the next cell."""
        if not self.state.planned_path:
            return False
            
        next_cell = self.state.planned_path[0]
        current_cell = (int(self.state.position[0]), int(self.state.position[1]))
        
        # Check static obstacles (Phase 4 Blocked Aisles)
        if self.costmap.get_cell(next_cell[0], next_cell[1]) == '#':
            if self.cbs_mode:
                # Blocked aisle: clear path, signal simulator to rerun CBS
                self.state.planned_path = []
                self.checkpoint_reached = True  # reuse flag to signal replan needed
            else:
                self._replan()
            return False

        if self.cbs_mode:
            # CBS produced a conflict-free path — skip priority negotiation,
            # but perform physical-block and proximity checks:
            physical_block = None
            current_occupant = self.reservation_table.get_claimer(next_cell, current_time)
            if current_occupant and current_occupant != self.state.robot_id:
                physical_block = current_occupant
            else:
                for peer_id, peer_msg in self.peer_states.items():
                    if peer_id == self.state.robot_id:
                        continue
                    if (int(peer_msg.position[0]), int(peer_msg.position[1])) == next_cell:
                        physical_block = peer_id
                        break

            if physical_block:
                if self.event_logger and self.state.status != RobotStatus.WAITING:
                    self.event_logger.log_conflict(
                        self.state.robot_id, physical_block,
                        "OCCUPIED_CELL", "YIELD_WAIT", int(current_time)
                    )
                self.state.status = RobotStatus.WAITING
                self.wait_time += 1.0
                self.waiting_on = physical_block
                return False

            if self.state.status == RobotStatus.WAITING:
                self.state.status = RobotStatus.MOVING
            return True

        if self.strategy == "B1":
            # Independent A* (No coordination)
            if self.state.status == RobotStatus.WAITING:
                self.state.status = RobotStatus.MOVING
            return True
            
        if self.strategy == "B2":
            # Stop-and-wait
            for peer_id, msg in self.peer_states.items():
                if current_time - self.last_seen.get(peer_id, 0) < 3.0:
                    px, py = msg.position
                    if int(px) == next_cell[0] and int(py) == next_cell[1]:
                        if self.event_logger and self.state.status != RobotStatus.WAITING:
                            self.event_logger.log_conflict(self.state.robot_id, peer_id, "PROXIMITY_STOP", "WAIT", int(current_time))
                        self.state.status = RobotStatus.WAITING
                        self.wait_time += 1.0
                        return False
            if self.state.status == RobotStatus.WAITING:
                self.state.status = RobotStatus.MOVING
            return True

        # Phase 3 Conflict Detection (P1 Strategy)
        # Check if any peer is physically stationary in the target cell
        for peer_id, peer_msg in self.peer_states.items():
            if peer_id == self.state.robot_id:
                continue
            if (int(peer_msg.position[0]), int(peer_msg.position[1])) == next_cell:
                if peer_msg.intent == Intent.WAIT or not peer_msg.planned_path or peer_msg.velocity == 0:
                    if self.event_logger and self.state.status != RobotStatus.WAITING:
                            self.event_logger.log_conflict(self.state.robot_id, peer_id, "OCCUPIED_CELL", "WAIT", int(current_time))
                    self.state.status = RobotStatus.WAITING
                    self.wait_time += 1.0
                    self.waiting_on = peer_id
                    # Track consecutive stall ticks on the same peer
                    if peer_id == self._prev_waiting_on:
                        self.wait_ticks_on_peer += 1
                    else:
                        self.wait_ticks_on_peer = 1
                        self._prev_waiting_on = peer_id
                    # Proactive replan: peer is also stuck (mutual starvation)
                    if self.wait_ticks_on_peer >= WAIT_REPLAN_THRESHOLD:
                        peer_msg = self.peer_states.get(peer_id)
                        peer_stuck = (peer_msg is not None and
                                      (peer_msg.intent == Intent.WAIT or peer_msg.velocity == 0))
                        if peer_stuck:
                            if self.event_logger:
                                self.event_logger.log_conflict(
                                    self.state.robot_id, peer_id,
                                    "MUTUAL_STARVATION", "PROACTIVE_REPLAN", int(current_time))
                            self.wait_ticks_on_peer = 0
                            self._prev_waiting_on = None
                            self._replan()
                    return False

        # Check if next_cell is already physically occupied in this tick
        # (a peer moved there earlier in the same simulation step and staked it).
        current_occupant = self.reservation_table.get_claimer(next_cell, current_time)
        if current_occupant and current_occupant != self.state.robot_id:
            if self.event_logger and self.state.status != RobotStatus.WAITING:
                self.event_logger.log_conflict(
                    self.state.robot_id, current_occupant,
                    "PHYSICAL_BLOCK", "WAIT", int(current_time))
            self.state.status = RobotStatus.WAITING
            self.wait_time += 1.0
            self.waiting_on = current_occupant
            return False

        conflicting_robot = check_vertex_conflict(self.reservation_table, self.state.robot_id, next_cell, current_time + 1)
        conflict_type = "VERTEX_CONFLICT"
        if not conflicting_robot:
            conflicting_robot = check_edge_swap(self.reservation_table, self.state.robot_id, current_cell, next_cell, current_time)
            conflict_type = "EDGE_SWAP"
            
        if conflicting_robot:
            self.waiting_on = conflicting_robot
            
            # Conflict Resolution: Compare priorities
            # CRITICAL: use strict > so that on equal priority the lower-ranked
            # robot (smaller tuple) always yields — preventing both robots from
            # simultaneously deciding to "proceed" which causes a collision.
            my_pri = self.get_priority()
            if conflicting_robot in self.peer_states:
                peer_msg = self.peer_states[conflicting_robot]
                # Use peer's broadcast wait indicator: if peer is broadcasting
                # WAIT intent, treat their priority as boosted by a small wait bonus
                # so robots that have been yielding longer get higher priority.
                peer_wait = 1.0 if peer_msg.intent == Intent.WAIT else 0.0
                peer_pri = self.priority_calc.calculate(
                    float(peer_msg.priority), peer_wait, peer_msg.battery,
                    float(len(peer_msg.planned_path)), conflicting_robot
                )
                
                if my_pri <= peer_pri:
                    # I yield — I'm lower priority, or tied (peer's robot_id is >= mine)
                    if self.event_logger and self.state.status != RobotStatus.WAITING:
                        self.event_logger.log_conflict(self.state.robot_id, conflicting_robot, conflict_type, "YIELD", int(current_time))
                    self.state.status = RobotStatus.WAITING
                    self.wait_time += 1.0
                    return False
                else:
                    # I have strictly higher priority — proceed, the other robot should yield.
                    # But only if the peer can actually move out of the cell we want.
                    px, py = peer_msg.position
                    if int(px) == next_cell[0] and int(py) == next_cell[1]:
                        if peer_msg.intent == Intent.WAIT or not peer_msg.planned_path:
                            # Peer is physically parked there and can't move — we must wait.
                            if self.event_logger and self.state.status != RobotStatus.WAITING:
                                self.event_logger.log_conflict(self.state.robot_id, conflicting_robot, conflict_type, "WAIT_OCCUPIED", int(current_time))
                            self.state.status = RobotStatus.WAITING
                            self.wait_time += 1.0
                            return False
                    # Peer should yield — proceed
                    if self.event_logger:
                        self.event_logger.log_conflict(self.state.robot_id, conflicting_robot, conflict_type, "PRIORITY_PASS", int(current_time))
                    pass
            else:
                # Peer unknown — play safe and wait
                if self.event_logger and self.state.status != RobotStatus.WAITING:
                    self.event_logger.log_conflict(self.state.robot_id, conflicting_robot, conflict_type, "CAUTION_WAIT", int(current_time))
                self.state.status = RobotStatus.WAITING
                self.wait_time += 1.0
                return False
                
        self.waiting_on = None
        self.wait_time = 0.0
        self.wait_ticks_on_peer = 0
        self._prev_waiting_on = None
        if self.state.status == RobotStatus.WAITING:
            self.state.status = RobotStatus.MOVING
        return True

    def force_reroute(self):
        """Called when a deadlock is broken. If CBS manages this robot, the
        simulator will re-run CBS; otherwise fall back to internal A*."""
        if not self.cbs_mode:
            self._replan()
        self.wait_time = 0.0
        self.waiting_on = None
        self.wait_ticks_on_peer = 0
        self._prev_waiting_on = None

    def tick(self, current_time: float):
        self.state.timestamp = current_time
        self._update_peer_reservations(current_time)
        
        if self.state.status == RobotStatus.OFFLINE:
            return

        if self.state.status != RobotStatus.OFFLINE:
            if self._check_conflicts(current_time):
                # Move
                next_cell = self.state.planned_path.pop(0)
                curr_int = (int(self.state.position[0]), int(self.state.position[1]))
                target_int = (int(next_cell[0]), int(next_cell[1]))
                if target_int == curr_int:
                    self.state.status = RobotStatus.WAITING
                    self.wait_time += 1.0
                else:
                    dx = target_int[0] - curr_int[0]
                    dy = target_int[1] - curr_int[1]
                    import math
                    self.state.heading = math.degrees(math.atan2(dy, dx))
                    self.state.position = (float(next_cell[0]), float(next_cell[1]))
                    if self.state.status != RobotStatus.DEGRADED:
                        self.state.status = RobotStatus.MOVING
                    # Immediately stake current position in the shared reservation table
                    # so that later robots ticking in the same step see us here and
                    # don't also move into this cell (fixes the simultaneous-entry collision).
                    self.reservation_table.commit(
                        self.state.robot_id,
                        [target_int],
                        current_time
                    )
            elif not self.state.planned_path and self.target_cell:
                # Reached target
                self._handle_arrival()
                
        # Broadcast intent
        self.seq += 1
        
        broadcast_path = self.state.planned_path.copy()
        if self.state.status in (RobotStatus.WAITING, RobotStatus.IDLE):
            curr_pos = (int(self.state.position[0]), int(self.state.position[1]))
            broadcast_path = [curr_pos] * 200 + broadcast_path

        msg = IntentMessage(
            robot_id=self.state.robot_id,
            seq=self.seq,
            timestamp=current_time,
            position=self.state.position,
            velocity=1.0 if self.state.status == RobotStatus.MOVING else 0.0,
            intent=Intent.WAIT if self.state.status in (RobotStatus.WAITING, RobotStatus.IDLE) else Intent.MOVE,
            next_intersection=None,
            task_id=self.state.current_task_id,
            priority=self.state.task_priority,
            planned_path=broadcast_path,
            waiting_on=self.waiting_on,
            heartbeat=current_time
        )
        self.comms.send(msg)

    def _handle_arrival(self):
        if not self.current_task:
            return
            
        current_int = (int(self.state.position[0]), int(self.state.position[1]))
        
        if self.current_task.status == TaskStatus.ASSIGNED:
            px, py = self.current_task.pickup_cell
            dist = abs(current_int[0] - px) + abs(current_int[1] - py)
            
            arrived = False
            if dist == 0:
                arrived = True
            elif dist == 1 and self.costmap.get_cell(px, py) == '#':
                arrived = True
                
            if arrived:
                self.current_task.status = TaskStatus.IN_PROGRESS
                self.target_cell = self.current_task.dropoff_cell
                if self.cbs_mode:
                    # Signal the simulator to run CBS for the new dropoff goal.
                    self.checkpoint_reached = True
                    self.state.planned_path = []   # Clear stale pickup path
                else:
                    self._replan()
                
        elif current_int == self.current_task.dropoff_cell and self.current_task.status == TaskStatus.IN_PROGRESS:
            self.current_task.status = TaskStatus.COMPLETED
            self.state.status = RobotStatus.IDLE
            self.state.current_task_id = None
            self.current_task = None
            self.target_cell = None
            self.reservation_table.commit(self.state.robot_id, [current_int] * 200, self.state.timestamp)
