import random
import time
import uuid
from typing import List, Dict, Optional, Set
try:
    import pygame
except ImportError:
    pygame = None

from config import GridMap, load_map
from models import RobotState, Task, RobotStatus, TaskStatus
from allocator.task_generator import TaskGenerator
from allocator.hungarian import HungarianAllocator
from robot.planner import AStarPlanner
from robot.cbs import CBSPlanner, ObstacleCostmap
from robot.task_manager import LocalTaskManager
from robot.coordination import detect_deadlock, PriorityCalculator
from comms.channel import PubSubChannel
import metrics

HEARTBEAT_TIMEOUT   = 10  # ticks without a heartbeat before robot is OFFLINE
DEADLOCK_THRESHOLD  = 4   # ticks a robot may WAIT before deadlock check triggers
REALLOC_INTERVAL    = 5   # allocator runs every N ticks
CBS_REPLAN_INTERVAL = 12  # rolling-horizon CBS replan every N ticks (keeps paths fresh)


class EventLog:
    """Append-only log consumed by TelemetryBus (Phase 5)."""
    def __init__(self):
        self.conflict_events: List[dict] = []
        self.deadlock_events: List[dict] = []

    def log_conflict(self, robot_id: str, peer_id: str, conflict_type: str, outcome: str, tick: int):
        self.conflict_events.append({
            "tick": tick, "robot_id": robot_id, "peer_id": peer_id,
            "type": conflict_type, "outcome": outcome
        })

    def log_deadlock_break(self, cycle: List[str], broken_robot: str, tick: int):
        self.deadlock_events.append({
            "tick": tick, "cycle": cycle, "broken_robot": broken_robot
        })


class Simulator:
    def __init__(self, ascii_map: str, headless: bool = True,
                 telemetry_bus=None, strategy: str = "P1"):
        self.grid_map = load_map(ascii_map)
        self.headless = headless
        self.telemetry_bus = telemetry_bus   # Phase 5 — write-only publish, never reads back
        self.strategy = strategy

        self.pickup_cells = []
        for x, y in self.grid_map.find_all('#'):
            # Only add racks that are adjacent to an aisle
            if any(self.grid_map.get_cell(x+dx, y+dy) == '.' for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]):
                self.pickup_cells.append((x, y))
        self.dropoff_cells = self.grid_map.find_all('D')
        self.spawn_cells   = self.grid_map.find_all('R')
        self.free_cells    = self.grid_map.find_all('.')

        self.task_generator = TaskGenerator(
            self.pickup_cells, self.dropoff_cells, spawn_interval=5)

        self.planner   = AStarPlanner()
        self.comms     = PubSubChannel()
        self.allocator = HungarianAllocator(planner=self.planner, costmap=self.grid_map)
        self.event_log = EventLog()
        self.priority_calc = PriorityCalculator()
        # CBS coordinator — used when strategy == "P1"
        self.cbs_planner = CBSPlanner()

        self.robot_managers: List[LocalTaskManager] = []
        self.tasks: List[Task] = []
        self.completed_tasks = 0
        self.tick_count = 0
        self.blocked_cells: Set[tuple] = set()

        # Metric counters
        self.metric_values: Dict[str, float] = {
            metrics.COLLISION_COUNT: 0,
            metrics.DEADLOCK_COUNT: 0,
            metrics.REPLAN_COUNT: 0,
            metrics.THROUGHPUT: 0,
            metrics.WAITING_TIME: 0,
            metrics.MAKESPAN: 0,
        }

        # Heartbeat tracker: robot_id -> last heartbeat tick
        self.last_heartbeat: Dict[str, float] = {}

        # Shared global reservation table to enable Prioritized Planning
        from robot.coordination import ReservationTable
        self.global_reservation_table = ReservationTable()

        # Spawn robots at R markers
        num_robots = len(self.spawn_cells) if self.spawn_cells else 3
        for i in range(num_robots):
            spawn = self.spawn_cells[i] if self.spawn_cells else (0, 0)
            state = RobotState(
                robot_id=f"robot-{i}",
                timestamp=0.0,
                position=(float(spawn[0]), float(spawn[1])),
                heading=0.0,
                velocity=0.0,
                battery=100.0,
                current_task_id=None,
                task_priority=i + 1,  # unique base priority per robot — prevents priority ties
                status=RobotStatus.IDLE
            )
            manager = LocalTaskManager(state, self.planner, self.comms, self.grid_map, strategy=self.strategy, event_logger=self.event_log)

            # OVERRIDE the local table with the global one so robots instantly see each other's paths
            # during sequential allocation, solving the simultaneous-planning collision bug.
            manager.reservation_table = self.global_reservation_table

            # Enable CBS mode for P1 strategy — CBS is the sole path authority
            if self.strategy == "P1":
                manager.cbs_mode = True

            self.robot_managers.append(manager)
            self.last_heartbeat[state.robot_id] = 0.0

        # Rendering setup
        self.cell_size = 28
        self.screen_w = self.grid_map.width  * self.cell_size
        self.screen_h = self.grid_map.height * self.cell_size
        if not self.headless:
            if pygame is None:
                raise RuntimeError("Install pygame for rendering. Use headless=True for CI.")
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_w, self.screen_h))
            pygame.display.set_caption("Multi-AMR Simulator")
            self.clock = pygame.time.Clock()

    # -------------------------------------------------------------------------
    # Public debug/test hooks
    # -------------------------------------------------------------------------

    def block_cell(self, x: int, y: int):
        """Mark a free cell as temporarily blocked (Phase 4 Scenario S4)."""
        self.blocked_cells.add((x, y))
        self.grid_map.grid[y][x] = '#'
        # Notify all robots to replan if path now hits this cell
        for m in self.robot_managers:
            if any(c == (x, y) for c in m.state.planned_path):
                m.force_reroute()
                self.metric_values[metrics.REPLAN_COUNT] += 1

    def unblock_cell(self, x: int, y: int):
        self.blocked_cells.discard((x, y))
        self.grid_map.grid[y][x] = '.'

    def kill_robot(self, robot_id: str):
        """Immediately offline a robot (Phase 4 Scenario S5)."""
        manager = next((m for m in self.robot_managers if m.state.robot_id == robot_id), None)
        if manager:
            manager.state.status = RobotStatus.OFFLINE
            manager.reservation_table.expire(robot_id)
            self._orphan_task(manager)
            for m in self.robot_managers:
                m.last_seen.pop(robot_id, None)
                m.peer_states.pop(robot_id, None)

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------

    def _orphan_task(self, manager: LocalTaskManager):
        if manager.current_task:
            t = manager.current_task
            t.status = TaskStatus.RECOVERABLE
            t.assigned_robot_id = None
            manager.current_task = None
            manager.state.current_task_id = None

    def _check_heartbeats(self):
        for m in self.robot_managers:
            if m.state.status == RobotStatus.OFFLINE:
                continue
            gap = self.tick_count - self.last_heartbeat.get(m.state.robot_id, 0)
            if gap > HEARTBEAT_TIMEOUT:
                m.state.status = RobotStatus.OFFLINE
                m.reservation_table.expire(m.state.robot_id)
                self._orphan_task(m)
                for peer in self.robot_managers:
                    peer.last_seen.pop(m.state.robot_id, None)
                    peer.peer_states.pop(m.state.robot_id, None)

    def _run_deadlock_detection(self):
        wait_graph: Dict[str, str] = {}
        wait_times: Dict[str, float] = {}
        for m in self.robot_managers:
            if m.state.status == RobotStatus.WAITING and m.waiting_on:
                wait_graph[m.state.robot_id] = m.waiting_on
                wait_times[m.state.robot_id] = m.wait_time

        # --- Cycle detection: break the deadlock cycle ---
        long_waiters = {r for r, wt in wait_times.items() if wt >= DEADLOCK_THRESHOLD}
        if long_waiters:
            cycle = detect_deadlock(wait_graph)
            if cycle:
                self.metric_values[metrics.DEADLOCK_COUNT] += 1

                # The lowest-priority robot should reroute: it has the least
                # claim on right-of-way and choosing it minimises disruption
                # to the higher-priority robots in the cycle.
                def pri(rid):
                    m = next((x for x in self.robot_managers if x.state.robot_id == rid), None)
                    if m:
                        return m.get_priority()
                    return (0.0, rid)

                breaker = min(cycle, key=pri)   # lowest priority yields
                m = next(x for x in self.robot_managers if x.state.robot_id == breaker)
                m.force_reroute()
                self.metric_values[metrics.REPLAN_COUNT] += 1
                self.event_log.log_deadlock_break(cycle, breaker, self.tick_count)
                self.event_log.log_conflict(
                    breaker, cycle[0] if len(cycle) > 1 else breaker,
                    "DEADLOCK_BREAK", "REROUTE", self.tick_count)
                # CBS replan for all active robots after breaking the deadlock
                if self.strategy == "P1":
                    self._run_cbs_planning()

        # --- Secondary sweep: force-replan stalled robots not in any cycle ---
        stalled_any = False
        for m in self.robot_managers:
            if (m.state.status == RobotStatus.WAITING and
                    m.wait_time >= DEADLOCK_THRESHOLD * 2 and
                    m.state.robot_id not in wait_graph):
                m.force_reroute()
                stalled_any = True
                self.event_log.log_conflict(
                    m.state.robot_id, m.waiting_on or "NONE",
                    "STALL_REPLAN", "FORCE_REROUTE", self.tick_count)
        if stalled_any and self.strategy == "P1":
            self._run_cbs_planning()


    def _check_collisions(self):
        """Count vertex collisions for metrics (robots should not share cells after Phase 3)."""
        positions: Dict[tuple, str] = {}
        for m in self.robot_managers:
            if m.state.status == RobotStatus.OFFLINE:
                continue
            pos = (int(m.state.position[0]), int(m.state.position[1]))
            if pos in positions:
                self.metric_values[metrics.COLLISION_COUNT] += 1
            else:
                positions[pos] = m.state.robot_id

    def _allocate(self):
        eligible = [m.state for m in self.robot_managers
                    if m.state.status in (RobotStatus.IDLE,)]
        queueable = [t for t in self.tasks
                     if t.status in (TaskStatus.QUEUED, TaskStatus.RECOVERABLE)]
        if not eligible or not queueable:
            return
        assignments = self.allocator.allocate(eligible, queueable)
        newly_assigned = []
        for robot_id, task_id in assignments.items():
            manager = next(m for m in self.robot_managers if m.state.robot_id == robot_id)
            task = next(t for t in self.tasks if t.task_id == task_id)
            manager.assign_task(task)
            newly_assigned.append(robot_id)
        # After new assignments, replan all active robots with CBS so new
        # robots don't conflict with robots already on their way.
        if newly_assigned and self.strategy == "P1":
            self._run_cbs_planning()

    def _run_cbs_planning(self):
        """
        Run CBS for all active robots and inject collision-free paths.

        Collects every robot that has a goal (pickup or dropoff), calls
        CBSPlanner.plan(), and feeds each resulting path back via inject_path().
        Only runs when strategy == 'P1' and CBS mode is active.

        Robots without a goal (IDLE, OFFLINE) are excluded.
        """
        goals     = {}
        positions = {}
        starts    = {}

        for m in self.robot_managers:
            if m.state.status in (RobotStatus.OFFLINE, RobotStatus.IDLE):
                continue
            goal = m.get_current_goal()
            if goal is None:
                continue
            rid = m.state.robot_id
            goals[rid]     = goal
            positions[rid] = m.state.position
            starts[rid]    = self.state_timestamp(m)

        if not goals:
            return

        # Idle / offline robots are static obstacles — wrap the costmap so CBS
        # treats their cells as walls (O(1) per get_cell, zero constraint overhead).
        idle_cells = [
            m.state.position
            for m in self.robot_managers
            if m.state.status in (RobotStatus.IDLE, RobotStatus.OFFLINE)
        ]
        planning_map = (
            ObstacleCostmap(self.grid_map, idle_cells) if idle_cells else self.grid_map
        )

        paths = self.cbs_planner.plan(
            goals, positions, planning_map, starts,
            event_logger=self.event_log, tick=self.tick_count
        )

        injected = 0
        for m in self.robot_managers:
            rid = m.state.robot_id
            if rid in paths and paths[rid]:
                goal = goals[rid]
                m.inject_path(paths[rid], goal)
                injected += 1
        if injected:
            self.metric_values[metrics.REPLAN_COUNT] += 1  # count CBS runs, not individual paths

    def state_timestamp(self, manager: "LocalTaskManager") -> float:
        """Helper: returns the current simulation time for path planning start."""
        return float(self.tick_count)

    # -------------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------------

    @property
    def robots(self) -> List[RobotState]:
        return [m.state for m in self.robot_managers]

    def tick(self):
        self.tick_count += 1
        t = float(self.tick_count)

        # 1. Generate tasks
        new_tasks = self.task_generator.tick(t)
        self.tasks.extend(new_tasks)

        # 2. Allocate
        if new_tasks or self.tick_count % REALLOC_INTERVAL == 0:
            self._allocate()

        # 3. Tick each robot manager
        if self.strategy == "B0":
            active_robots = [m for m in self.robot_managers if m.state.status == RobotStatus.MOVING and m.state.planned_path]
            if not active_robots:
                candidates = [m for m in self.robot_managers if m.target_cell and m.state.status != RobotStatus.OFFLINE]
                if candidates:
                    candidates[0].state.status = RobotStatus.MOVING
                    active_robots = [candidates[0]]

            for m in self.robot_managers:
                if active_robots and m != active_robots[0] and m.target_cell and m.state.status != RobotStatus.OFFLINE:
                    m.state.status = RobotStatus.WAITING
                    m.wait_time += 1.0

        for m in self.robot_managers:
            m.tick(t)

        # 3b. CBS checkpoint check + rolling-horizon replan
        if self.strategy == "P1":
            checkpoint_triggered = any(m.checkpoint_reached for m in self.robot_managers)
            rolling_replan       = (self.tick_count % CBS_REPLAN_INTERVAL == 0)
            if checkpoint_triggered or rolling_replan:
                for m in self.robot_managers:
                    m.checkpoint_reached = False
                self._run_cbs_planning()

        # 4. Read heartbeats from comms channel
        for msg in self.comms.receive():
            if msg.heartbeat > 0:
                self.last_heartbeat[msg.robot_id] = t

        # 5. Update metrics
        waiting = sum(1 for m in self.robot_managers if m.state.status == RobotStatus.WAITING)
        self.metric_values[metrics.WAITING_TIME] += waiting
        self.metric_values[metrics.MAKESPAN] = t

        # 6. Count completed tasks
        for task in self.tasks:
            if task.status == TaskStatus.COMPLETED and not hasattr(task, '_counted'):
                task._counted = True
                self.completed_tasks += 1
        self.metric_values[metrics.THROUGHPUT] = self.completed_tasks / t if t > 0 else 0.0

        # 7. Heartbeat check & deadlock detection
        self._check_heartbeats()
        self._run_deadlock_detection()
        self._check_collisions()

        # 8. Clear comms
        self.comms.clear()

        # 9. Publish telemetry (Phase 5 — non-blocking, best-effort)
        if self.telemetry_bus is not None:
            snapshot = self._build_snapshot()
            self.telemetry_bus.publish(snapshot)

    def _build_snapshot(self) -> dict:
        metrics_dict = dict(self.metric_values)
        metrics_dict.update({
            "makespan": self.metric_values.get(metrics.MAKESPAN, 0.0),
            "throughput": self.metric_values.get(metrics.THROUGHPUT, 0.0),
            "collision_count": self.metric_values.get(metrics.COLLISION_COUNT, 0),
            "deadlock_count": self.metric_values.get(metrics.DEADLOCK_COUNT, 0),
            "replan_count": self.metric_values.get(metrics.REPLAN_COUNT, 0),
            "waiting_time": self.metric_values.get(metrics.WAITING_TIME, 0.0),
        })
        return {
            "tick": self.tick_count,
            "robots": [
                {
                    "robot_id": m.state.robot_id,
                    "position": m.state.position,
                    "status": m.state.status.value,
                    "battery": m.state.battery,
                    "current_task_id": m.state.current_task_id,
                    "task_priority": m.state.task_priority,
                    "planned_path": m.state.planned_path,
                    "communication_quality": m.state.communication_quality,
                    "localization_confidence": m.state.localization_confidence,
                    "heading": getattr(m.state, "heading", 0.0),
                    "has_cargo": m.current_task is not None and m.current_task.status == TaskStatus.IN_PROGRESS,
                    "target_cell": m.target_cell,
                }
                for m in self.robot_managers
            ],
            "tasks": [
                {
                    "task_id": t.task_id,
                    "pickup_cell": t.pickup_cell,
                    "dropoff_cell": t.dropoff_cell,
                    "status": t.status.value,
                    "priority": t.priority,
                    "assigned_robot_id": t.assigned_robot_id,
                }
                for t in self.tasks
            ],
            "conflicts": self.event_log.conflict_events[-20:],
            "deadlocks": self.event_log.deadlock_events[-10:],
            "metrics": metrics_dict,
            "grid": self.grid_map.grid if self.grid_map else [],
            "scenario": getattr(self, "scenario_name", "S1_Normal"),
        }

    # -------------------------------------------------------------------------
    # Rendering
    # -------------------------------------------------------------------------

    ROBOT_COLORS = [(220, 80, 80), (80, 180, 220), (100, 210, 120)]

    def render(self):
        if self.headless or pygame is None:
            return
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); raise SystemExit

        self.screen.fill((30, 30, 35))
        cs = self.cell_size

        for y in range(self.grid_map.height):
            for x in range(self.grid_map.width):
                char = self.grid_map.get_cell(x, y)
                rect = pygame.Rect(x * cs, y * cs, cs - 1, cs - 1)
                if char == '#':
                    pygame.draw.rect(self.screen, (70, 70, 80), rect, border_radius=2)
                elif char == 'P':
                    pygame.draw.rect(self.screen, (40, 160, 80), rect, border_radius=2)
                elif char == 'D':
                    pygame.draw.rect(self.screen, (60, 100, 210), rect, border_radius=2)
                else:
                    pygame.draw.rect(self.screen, (45, 45, 52), rect, border_radius=2)

        for idx, m in enumerate(self.robot_managers):
            color = self.ROBOT_COLORS[idx % len(self.ROBOT_COLORS)]
            state = m.state

            if state.planned_path:
                pts = [(int((px + .5) * cs), int((py + .5) * cs))
                       for px, py in state.planned_path]
                rx, ry = state.position
                pts = [(int((rx + .5) * cs), int((ry + .5) * cs))] + pts
                if len(pts) >= 2:
                    pygame.draw.lines(self.screen,
                                      (color[0]//2, color[1]//2, color[2]//2),
                                      False, pts, 2)

            rx, ry = state.position
            cx = int((rx + .5) * cs)
            cy = int((ry + .5) * cs)
            r  = cs // 2 - 2
            pygame.draw.circle(self.screen, color, (cx, cy), r)
            if state.status == RobotStatus.WAITING:
                pygame.draw.circle(self.screen, (255, 220, 0), (cx, cy), r, 2)
            elif state.status == RobotStatus.OFFLINE:
                pygame.draw.circle(self.screen, (60, 60, 60), (cx, cy), r)

        pygame.display.flip()
        self.clock.tick(10)

    def run(self, max_ticks: int = 500):
        for _ in range(max_ticks):
            self.tick()
            if not self.headless:
                self.render()
