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
from data.industrial_robot_profiles import maintenance_profile
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
        self.staging_cells = self.grid_map.find_all('S')
        self.charger_cells = self.grid_map.find_all('C')
        self.spawn_cells   = self.grid_map.find_all('R')
        self.free_cells    = self.grid_map.find_all('.')

        # Scenario maps define explicit S/C bays. Small unit-test/custom maps
        # may omit infrastructure, so derive deterministic fixed bays once at
        # initialization rather than falling back to arbitrary dynamic parking.
        if not self.staging_cells or not self.charger_cells:
            fallback_staging, fallback_chargers = self._derive_fixed_service_cells()
            if not self.staging_cells:
                self.staging_cells = fallback_staging
            if not self.charger_cells:
                self.charger_cells = fallback_chargers

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

        # Each AMR owns its reservation table. Peer intent messages are the
        # only mechanism used to learn other robots' planned occupancy.
        # There is deliberately no fleet-wide reservation table.
        # Spawn robots at R markers with realistic initial battery levels for demonstration
        num_robots = len(self.spawn_cells) if self.spawn_cells else 3
        initial_batteries = [96.0, 42.0, 88.0, 68.0, 82.0, 32.0, 91.0, 54.0, 94.0]
        for i in range(num_robots):
            spawn = self.spawn_cells[i] if self.spawn_cells else (0, 0)
            state = RobotState(
                robot_id=f"robot-{i}",
                timestamp=0.0,
                position=(float(spawn[0]), float(spawn[1])),
                heading=0.0,
                velocity=0.0,
                battery=initial_batteries[i % len(initial_batteries)],
                current_task_id=None,
                task_priority=i + 1,  # unique base priority per robot — prevents priority ties
                status=RobotStatus.IDLE
            )
            manager = LocalTaskManager(state, self.planner, self.comms, self.grid_map, strategy=self.strategy, event_logger=self.event_log)
            manager.maintenance_profile = maintenance_profile(f"item_{i}")

            # Enable CBS mode for P1 strategy — CBS is the sole path authority
            if self.strategy == "P1":
                manager.cbs_mode = True

            self.robot_managers.append(manager)
            self.last_heartbeat[state.robot_id] = 0.0

        # Seed the local communication watchdog with an initial expectation
        # for every peer. This lets a robot detect a peer that never delivers
        # its first heartbeat.
        robot_ids = [m.state.robot_id for m in self.robot_managers]
        for manager in self.robot_managers:
            manager.last_seen = {
                peer_id: 0.0
                for peer_id in robot_ids
                if peer_id != manager.state.robot_id
            }

        # Seed initial tasks so ALL robots have active paths and targets at tick 0
        if self.pickup_cells and self.dropoff_cells:
            for i in range(num_robots):
                pickup = self.pickup_cells[i % len(self.pickup_cells)]
                dropoff = self.dropoff_cells[i % len(self.dropoff_cells)]
                t = Task(
                    task_id=f"INIT_TASK_{i+1}",
                    pickup_cell=pickup,
                    dropoff_cell=dropoff,
                    priority=i + 1,
                    status=TaskStatus.QUEUED,
                    created_at=0.0
                )
                self.tasks.append(t)
                self.task_generator.queue.append(t)
            self._allocate()
            # _allocate() assigns tasks; P1 robots intentionally do not build
            # their own A* path. Seed the initial fleet through CBS immediately
            # so the first telemetry frame already contains executable routes.
            if self.strategy == "P1":
                self._run_cbs_planning()

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

    def _derive_fixed_service_cells(self):
        """Choose deterministic fixed service bays for maps without S/C markers."""
        specials = set(self.dropoff_cells + self.pickup_cells + self.spawn_cells)
        occupied = set(specials)
        candidates = []

        for cell in self.grid_map.find_all('.'):
            cell = (int(cell[0]), int(cell[1]))
            degree = sum(
                1
                for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0))
                if self.grid_map.get_cell(cell[0] + dx, cell[1] + dy) != '#'
            )
            if degree < 2:
                continue

            min_special_distance = min(
                (
                    abs(cell[0] - sx) + abs(cell[1] - sy)
                    for sx, sy in specials
                ),
                default=0,
            )
            candidates.append((-min_special_distance, -degree, cell[1], cell[0], cell))

        candidates.sort()

        staging = []
        for _, _, _, _, cell in candidates:
            if cell in occupied:
                continue
            if any(abs(cell[0] - other[0]) + abs(cell[1] - other[1]) < 3 for other in staging):
                continue
            staging.append(cell)
            occupied.add(cell)
            # Keep enough fixed staging capacity for a small fleet.
            # Service bays are explicit infrastructure, not dynamic parking.
            if len(staging) >= min(4, max(3, len(self.free_cells) // 10)):
                break

        chargers = []
        for _, _, _, _, cell in candidates:
            if cell in occupied:
                continue
            if any(abs(cell[0] - other[0]) + abs(cell[1] - other[1]) < 3 for other in chargers):
                continue
            chargers.append(cell)
            occupied.add(cell)
            if len(chargers) >= min(2, max(1, len(self.free_cells) // 20)):
                break

        return staging, chargers

    def clear_queued_auto_tasks(self) -> int:
        """Remove pending automatic orders when switching to manual-only mode.

        Assigned/in-transit tasks are deliberately preserved so active AMRs
        continue their current work. Manual tasks are never removed here.
        """
        queued_auto_ids = {
            t.task_id for t in self.tasks
            if t.status == TaskStatus.QUEUED and getattr(t, "source", "AUTO") == "AUTO"
        }
        if not queued_auto_ids:
            return 0

        self.tasks = [t for t in self.tasks if t.task_id not in queued_auto_ids]
        self.task_generator.queue = [
            t for t in self.task_generator.queue if t.task_id not in queued_auto_ids
        ]
        return len(queued_auto_ids)

    def block_cell(self, x: int, y: int):
        """Mark a free cell as temporarily blocked (Phase 4 Scenario S4)."""
        self.blocked_cells.add((x, y))
        self.grid_map.grid[y][x] = '#'
        # In CBS mode force_reroute() deliberately defers to the
        # simulator-level CBS coordinator. A dynamic obstacle must therefore
        # explicitly invalidate stale paths/reservations.
        for m in self.robot_managers:
            if any(c == (x, y) for c in m.state.planned_path):
                m.state.planned_path = []
                m.reservation_table.expire(m.state.robot_id)
                m.checkpoint_reached = True
                self.metric_values[metrics.REPLAN_COUNT] += 1

    def unblock_cell(self, x: int, y: int):
        self.blocked_cells.discard((x, y))
        self.grid_map.grid[y][x] = '.'

    def kill_robot(self, robot_id: str):
        """Immediately offline a robot and automatically reallocate its task to an available peer."""
        manager = next((m for m in self.robot_managers if m.state.robot_id == robot_id), None)
        if manager and manager.state.status != RobotStatus.OFFLINE:
            prev_task = manager.current_task
            task_id = prev_task.task_id if prev_task else None
            manager.state.status = RobotStatus.OFFLINE
            manager.state.planned_path = []
            manager.target_cell = None
            manager.reservation_table.expire(robot_id)
            self._orphan_task(manager)
            for m in self.robot_managers:
                m.last_seen.pop(robot_id, None)
                m.peer_states.pop(robot_id, None)

            # Trigger immediate task reallocation to an available peer
            self._allocate()

            if prev_task:
                new_owner = prev_task.assigned_robot_id or "RECOVERABLE_QUEUE"
                self.event_log.log_conflict(
                    robot_id, new_owner, "TASK_REALLOCATION",
                    f"Robot {robot_id} SHUTDOWN -> Task {task_id} reallocated to {new_owner}",
                    self.tick_count
                )

    def revive_robot(self, robot_id: str):
        """Revive an offline or charging robot back to active IDLE with 100% battery."""
        manager = next((m for m in self.robot_managers if m.state.robot_id == robot_id), None)
        if manager:
            manager.state.status = RobotStatus.IDLE
            manager.state.battery = 100.0
            manager.state.planned_path = []
            manager.target_cell = None
            manager.post_task_mode = None
            manager.wait_time = 0.0
            self.last_heartbeat[robot_id] = float(self.tick_count)
            for peer in self.robot_managers:
                peer.last_seen[robot_id] = float(self.tick_count)
            self.event_log.log_conflict(
                robot_id, "SYSTEM", "ROBOT_ONLINE",
                f"Robot {robot_id} restored to full service (100% Battery)",
                self.tick_count
            )
            self._allocate()

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
            manager.state.planned_path = []
            manager.target_cell = None
            manager.post_task_mode = None

    def _check_heartbeats(self):
        for m in self.robot_managers:
            if m.state.status in (RobotStatus.OFFLINE, RobotStatus.CHARGING):
                continue
            gap = self.tick_count - self.last_heartbeat.get(m.state.robot_id, 0)
            if gap > HEARTBEAT_TIMEOUT:
                m.state.status = RobotStatus.OFFLINE
                m.reservation_table.expire(m.state.robot_id)
                self._orphan_task(m)
                for peer in self.robot_managers:
                    peer.last_seen.pop(m.state.robot_id, None)
                    peer.peer_states.pop(m.state.robot_id, None)
                self._allocate()


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

    def _find_free_service_cell(self, manager: LocalTaskManager, cells: List[tuple]) -> Optional[tuple]:
        """Return the nearest currently free fixed service bay.

        Staging and charging bays are explicit single-occupancy resources. The
        physical occupancy check handles robots already sitting on a bay, while
        the reservation-table check prevents two post-task routes from claiming
        the same bay at the same execution time.
        """
        if not cells:
            return None

        occupied = {
            (int(m.state.position[0]), int(m.state.position[1]))
            for m in self.robot_managers
            if m is not manager and m.state.status != RobotStatus.OFFLINE
        }
        candidates = []
        for cell in cells:
            cell = (int(cell[0]), int(cell[1]))
            if cell in occupied:
                continue
            if manager.reservation_table.get_claimer(cell, float(self.tick_count + 1)):
                continue
            distance = (
                abs(cell[0] - int(manager.state.position[0]))
                + abs(cell[1] - int(manager.state.position[1]))
            )
            candidates.append((distance, cell))

        candidates.sort()
        return candidates[0][1] if candidates else None

    def _schedule_post_task_destination(self, manager: LocalTaskManager, mode: str) -> bool:
        """Send a taskless AMR to a fixed charger or staging bay.

        This is fleet housekeeping, not order generation, so it continues even
        when Auto Mode is OFF. A delivery cell is released immediately when a
        task completes and is never used as a parking destination.
        """
        if manager.current_task is not None or manager.state.status == RobotStatus.OFFLINE:
            return False

        if mode not in ("CHARGER", "STAGING"):
            raise ValueError(f"Unsupported post-task mode: {mode}")

        cells = self.charger_cells if mode == "CHARGER" else self.staging_cells
        goal = self._find_free_service_cell(manager, cells)
        manager.post_task_mode = mode

        if goal is None:
            manager.target_cell = None
            manager.state.planned_path = []
            manager.state.status = RobotStatus.WAITING
            manager.waiting_on = f"{mode}_RESOURCE"
            manager.checkpoint_reached = True
            self.event_log.log_conflict(
                manager.state.robot_id,
                "SYSTEM",
                f"{mode}_RESOURCE",
                "WAIT_RESOURCE",
                self.tick_count,
            )
            return False

        manager.target_cell = goal
        manager.state.planned_path = []
        manager.state.status = RobotStatus.MOVING
        manager.wait_time = 0.0
        manager.waiting_on = None
        manager.checkpoint_reached = True
        if not manager.cbs_mode:
            manager._replan()
        self.event_log.log_conflict(
            manager.state.robot_id,
            "SYSTEM",
            f"POST_TASK_{mode}",
            f"Reserved fixed {mode.lower()} bay {goal}",
            self.tick_count,
        )
        return True

    def _retry_post_task_destinations(self):
        """Retry taskless robots waiting for a fixed service resource."""
        for manager in self.robot_managers:
            if manager.current_task is not None or manager.state.status == RobotStatus.OFFLINE:
                continue
            if manager.post_task_mode not in ("CHARGER", "STAGING"):
                continue
            if manager.target_cell is not None:
                continue

            mode = manager.post_task_mode
            if manager.state.battery <= 20.0:
                mode = "CHARGER"
            self._schedule_post_task_destination(manager, mode)

    def _allocate(self):
        eligible = [
            m.state for m in self.robot_managers
            if (
                m.state.battery > 20.0
                and (
                    m.state.status in (RobotStatus.IDLE, RobotStatus.STAGING)
                    or (
                        m.state.status == RobotStatus.MOVING
                        and m.current_task is None
                        and m.post_task_mode == "STAGING"
                    )
                )
            )
        ]
        queueable = [t for t in self.tasks
                     if t.status in (TaskStatus.QUEUED, TaskStatus.RECOVERABLE)]
        if not eligible or not queueable:
            return

        # A dropoff is a shared physical delivery slot. The allocator only
        # prevents multiple NEW assignments from claiming the same slot in
        # this allocation pass. It deliberately does not inspect a parked
        # robot's physical cell: in the P2P strategy the receiving robot may
        # already be carrying the next task and its local conflict checks will
        # wait/yield until the slot is physically clear. P1/CBS has an
        # additional goal-slot check in _run_cbs_planning().
        occupied_dropoffs = {
            tuple(m.current_task.dropoff_cell)
            for m in self.robot_managers
            if m.current_task is not None
            and m.state.status not in (RobotStatus.OFFLINE, RobotStatus.CHARGING)
        }
        available_queueable = [
            task for task in queueable
            if tuple(task.dropoff_cell) not in occupied_dropoffs
        ]
        if not available_queueable:
            return

        # Reserve each currently-free dropoff cell for at most one new assignment
        # in this allocation pass. The Hungarian allocator sees the candidate
        # tasks simultaneously, so filtering only already-occupied cells is not
        # enough to prevent two queued tasks from claiming the same free slot.
        # Keep the highest-priority candidate for each exact dropoff cell.
        available_queueable.sort(
            key=lambda t: (0 if t.status == TaskStatus.RECOVERABLE else 1, t.priority, t.created_at, t.task_id)
        )
        unique_dropoff_tasks = []
        reserved_dropoffs = set(occupied_dropoffs)
        for task in available_queueable:
            dropoff = tuple(task.dropoff_cell)
            if dropoff in reserved_dropoffs:
                self.event_log.log_conflict(
                    "SYSTEM",
                    task.task_id,
                    "GOAL_SLOT",
                    "YIELD_WAIT",
                    self.tick_count,
                )
                continue
            reserved_dropoffs.add(dropoff)
            unique_dropoff_tasks.append(task)

        if not unique_dropoff_tasks:
            return

        assignments = self.allocator.allocate(eligible, unique_dropoff_tasks)
        newly_assigned = []
        for robot_id, task_id in assignments.items():
            manager = next(m for m in self.robot_managers if m.state.robot_id == robot_id)
            task = next(t for t in self.tasks if t.task_id == task_id)
            was_recoverable = (task.status == TaskStatus.RECOVERABLE)
            manager.assign_task(task)
            newly_assigned.append(robot_id)
            if was_recoverable:
                self.event_log.log_conflict(
                    "SYSTEM", robot_id, "TASK_TAKEOVER",
                    f"Task {task_id} successfully reallocated & assigned to {robot_id}",
                    self.tick_count
                )
        # After new assignments, replan all active robots with CBS so new
        # robots don't conflict with robots already on their way.
        if newly_assigned and self.strategy == "P1":
            self._run_cbs_planning()

    def _run_cbs_planning(self):
        """
        Run CBS for all active robots and inject collision-free paths.

        Shared pickup/dropoff cells are physical single-occupancy slots. If
        multiple active tasks currently target the same slot, CBS must not be
        asked to make all of them occupy that cell at once. One robot is
        selected for the slot (prefer a robot already carrying a load, then
        the oldest task); the other active robots remain WAITING and keep
        their tasks. They become eligible for CBS again after the slot holder
        completes and leaves the cell.
        """
        goals     = {}
        positions = {}
        starts    = {}

        # A completed AMR remains physically on its dropoff cell until it
        # receives another task.  If another active task was already assigned
        # to that same cell before the first task completed, the goal is
        # temporarily unavailable.  Do not feed that occupied goal into CBS:
        # ObstacleCostmap intentionally exposes parked robots as '#', and the
        # low-level planner may otherwise treat the dynamic blockage like a
        # rack/adjacent-approach goal.  That is the source of the observed
        # AMR "looping" around an order that never reaches COMPLETED.
        target_groups = {}
        for m in self.robot_managers:
            if m.state.status in (RobotStatus.OFFLINE, RobotStatus.IDLE, RobotStatus.STAGING, RobotStatus.CHARGING):
                continue
            goal = m.get_current_goal()
            if goal is None:
                continue

            goal_cell = (int(goal[0]), int(goal[1]))
            current_cell = (
                int(m.state.position[0]),
                int(m.state.position[1]),
            )
            goal_occupant = next(
                (
                    peer for peer in self.robot_managers
                    if peer is not m
                    and peer.state.status != RobotStatus.OFFLINE
                    and (int(peer.state.position[0]), int(peer.state.position[1])) == goal_cell
                ),
                None,
            )
            service_cells = set(self.dropoff_cells) | set(self.staging_cells) | set(self.charger_cells)
            if goal_cell in service_cells and goal_occupant is not None and current_cell != goal_cell:
                if m.state.status != RobotStatus.WAITING or m.waiting_on != f"GOAL_SLOT_OCCUPIED:{goal_cell[0]},{goal_cell[1]}":
                    self.event_log.log_conflict(
                        m.state.robot_id,
                        goal_occupant.state.robot_id,
                        "GOAL_SLOT_OCCUPIED",
                        "WAIT",
                        self.tick_count,
                    )
                m.state.planned_path = []
                m.state.status = RobotStatus.WAITING
                m.waiting_on = f"GOAL_SLOT_OCCUPIED:{goal_cell[0]},{goal_cell[1]}"
                continue

            target_groups.setdefault(goal_cell, []).append(m)

        for goal_cell, managers in target_groups.items():
            if len(managers) > 1:
                # Carrying robots get the slot first because their cargo is
                # already committed to this task. Then prefer the oldest task.
                def slot_priority(manager):
                    task = manager.current_task
                    carrying = (manager.current_task is not None and manager.current_task.status == TaskStatus.IN_PROGRESS)
                    created = getattr(task, "created_at", 0.0) if task else 0.0
                    task_id = getattr(task, "task_id", "") if task else ""
                    return (0 if carrying else 1, created, task_id)

                managers.sort(key=slot_priority)
                slot_holder = managers[0]

                for blocked in managers[1:]:
                    blocked.state.planned_path = []
                    blocked.state.status = RobotStatus.WAITING
                    blocked.waiting_on = f"GOAL_SLOT:{goal_cell[0]},{goal_cell[1]}"
                    blocked.wait_time += 1.0
                    self.event_log.log_conflict(
                        blocked.state.robot_id,
                        slot_holder.state.robot_id,
                        "GOAL_SLOT",
                        "YIELD_WAIT",
                        self.tick_count,
                    )

                managers = [slot_holder]

            for m in managers:
                rid = m.state.robot_id
                goals[rid]     = m.get_current_goal()
                positions[rid] = m.state.position
                starts[rid]    = self.state_timestamp(m)

        if not goals:
            return

        # In the P2P strategy, every robot plans locally from peer intents.
        # CBS remains available only as an explicit centralized benchmark/legacy
        # strategy; the live fleet path uses peer-to-peer coordination.
        if self.strategy == "P2P":
            return

        # Idle / offline / charging robots are static obstacles — wrap the costmap so CBS
        # treats their cells as walls (O(1) per get_cell, zero constraint overhead).
        idle_cells = [
            m.state.position
            for m in self.robot_managers
            if m.state.status in (RobotStatus.IDLE, RobotStatus.STAGING, RobotStatus.OFFLINE, RobotStatus.CHARGING)
        ]
        planning_map = (
            ObstacleCostmap(self.grid_map, idle_cells) if idle_cells else self.grid_map
        )

        paths = self.cbs_planner.plan(
            goals, positions, planning_map, starts,
            event_logger=self.event_log, tick=self.tick_count
        )

        # The CBS planner can exhaust its conflict-tree budget on a dense
        # rolling-horizon problem. A partial/empty CBS result must not strand
        # every active AMR. Robots with no CBS route get one low-level A* path
        # to their current goal as a liveness fallback; execution-time
        # occupancy checks still remain authoritative.
        fallback_count = 0
        for m in self.robot_managers:
            rid = m.state.robot_id
            goal = goals.get(rid)
            if goal is None or paths.get(rid):
                continue
            fallback = self.planner.plan(
                start=m.state.position,
                goal=goal,
                costmap=self.grid_map,
                reservation_table=m.reservation_table,
                start_time=float(self.tick_count),
                robot_id=rid,
            )
            if fallback:
                paths[rid] = fallback
                fallback_count += 1
                self.event_log.log_conflict(
                    rid, "CBS", "NO_JOINT_PATH", "ASTAR_FALLBACK", self.tick_count
                )

        injected = 0
        for m in self.robot_managers:
            rid = m.state.robot_id
            goal = goals.get(rid)
            path = paths.get(rid)

            if path:
                m.inject_path(path, goal)
                injected += 1
                continue

            # A* fallback above should normally cover a CBS budget miss. Keep the
            # last-resort WAIT only for a genuine no-path result.
            # CBS may legitimately fail to find a route for one active robot
            # while other robots still receive valid paths. Never leave that
            # robot advertising MOVING/IN TRANSIT with an empty route.
            if goal is not None:
                current_cell = (
                    int(m.state.position[0]),
                    int(m.state.position[1]),
                )
                if current_cell == (int(goal[0]), int(goal[1])):
                    m._handle_arrival()
                    if m.current_task is None or m.target_cell is None:
                        continue

                m.state.planned_path = []
                m.state.status = RobotStatus.WAITING
                m.waiting_on = "CBS"
                m.wait_time += 1.0
                self.event_log.log_conflict(
                    rid, "CBS", "NO_PATH", "WAIT_REPLAN", self.tick_count
                )

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

        # Track active tasks before any arrival resolution so a completion
        # during this tick can trigger immediate dispatch.
        active_task_robots_before_tick = {
            m.state.robot_id for m in self.robot_managers if m.current_task is not None
        }

        # 1. Generate tasks
        new_tasks = self.task_generator.tick(t)
        self.tasks.extend(new_tasks)

        # 2. Allocate
        if new_tasks or self.tick_count % REALLOC_INTERVAL == 0:
            self._allocate()

        # 3. Resolve robots that are already sitting on their target
        # before allowing any robot to move during this tick.
        #
        # This prevents sequential execution from allowing robot A to enter
        # a cell that robot B already occupies but has not yet processed its
        # arrival/completion state.
        for m in self.robot_managers:
            if m.state.status in (RobotStatus.OFFLINE, RobotStatus.CHARGING):
                continue

            if not m.state.planned_path and m.target_cell:
                current_cell = (
                    int(m.state.position[0]),
                    int(m.state.position[1]),
                )

                if current_cell == m.target_cell:
                    m._handle_arrival()

        # 3. Tick each robot manager
        positions_before_tick = {
            m.state.robot_id: (
                int(m.state.position[0]),
                int(m.state.position[1]),
            )
            for m in self.robot_managers
        }

        if self.strategy == "B0":
            active_robots = [m for m in self.robot_managers if m.state.status == RobotStatus.MOVING and m.state.planned_path]
            if not active_robots:
                candidates = [m for m in self.robot_managers if m.target_cell and m.state.status not in (RobotStatus.OFFLINE, RobotStatus.CHARGING)]
                if candidates:
                    candidates[0].state.status = RobotStatus.MOVING
                    active_robots = [candidates[0]]

            for m in self.robot_managers:
                if active_robots and m != active_robots[0] and m.target_cell and m.state.status not in (RobotStatus.OFFLINE, RobotStatus.CHARGING):
                    m.state.status = RobotStatus.WAITING
                    m.wait_time += 1.0

        for m in self.robot_managers:
            m.tick(t)

        # Final physical safety interlock for the lockstep simulator.
        # Planning/reservations are the primary coordination mechanism, but
        # execution is sequential and can race with delayed peer telemetry.
        # If a robot nevertheless enters a cell occupied by another robot in
        # the same tick, roll the later mover back before the state is published.
        # This models a safety stop rather than allowing an actual simulated
        # collision to persist into telemetry.
        occupied_after_tick = {}
        for manager in self.robot_managers:
            if manager.state.status == RobotStatus.OFFLINE:
                continue
            cell = (
                int(manager.state.position[0]),
                int(manager.state.position[1]),
            )
            if cell not in occupied_after_tick:
                occupied_after_tick[cell] = manager
                continue

            prior_cell = positions_before_tick[manager.state.robot_id]
            manager.state.position = (float(prior_cell[0]), float(prior_cell[1]))
            manager.state.planned_path = []
            manager.state.status = RobotStatus.WAITING
            manager.waiting_on = occupied_after_tick[cell].state.robot_id
            manager.checkpoint_reached = True
            manager.wait_time += 1.0
            self.event_log.log_conflict(
                manager.state.robot_id,
                occupied_after_tick[cell].state.robot_id,
                "PHYSICAL_SAFETY_STOP",
                "ROLLBACK",
                self.tick_count,
            )

        # Re-plan any taskless robots whose fixed service resource became
        # available, then route newly completed robots away from the delivery cell.
        self._retry_post_task_destinations()

        # A robot that has just reached a fixed staging bay is immediately
        # eligible for another queued order. This is still the normal allocator;
        # the trigger simply avoids waiting for the periodic 5-tick sweep.
        if any(
            m.state.status == RobotStatus.STAGING
            and m.current_task is None
            and m.post_task_mode is None
            for m in self.robot_managers
        ):
            self._allocate()

        completed_robot_ids = [
            m.state.robot_id
            for m in self.robot_managers
            if (
                m.state.robot_id in active_task_robots_before_tick
                and m.current_task is None
                and m.state.status == RobotStatus.IDLE
            )
        ]

        # Delivery cells are released by _handle_arrival(). The completed AMR
        # immediately transitions to a fixed staging bay, or to a fixed charger
        # when its battery is low. This removes the root cause of endpoint
        # blocking instead of waiting for another order to pull it away.
        for manager in self.robot_managers:
            if manager.state.robot_id not in completed_robot_ids:
                continue
            mode = "CHARGER" if manager.state.battery <= 20.0 else "STAGING"
            self._schedule_post_task_destination(manager, mode)

        # 3a. Battery discharge, low-battery failover & recharge cycle
        for m in self.robot_managers:
            if m.state.status == RobotStatus.OFFLINE:
                continue

            if m.state.status == RobotStatus.CHARGING:
                # Recharging: realistic +1.5% per tick
                m.state.battery = min(100.0, m.state.battery + 1.5)
                if m.state.battery >= 90.0:
                    m.state.status = RobotStatus.IDLE
                    m.post_task_mode = None
                    self.event_log.log_conflict(
                        m.state.robot_id, "SYSTEM", "CHARGE_COMPLETE",
                        f"Recharge complete ({m.state.battery:.0f}%) -> Returned to active fleet",
                        self.tick_count
                    )
                    self._allocate()
            else:
                # Realistic operational discharge:
                # Higher power draw when carrying cargo
                if m.state.status == RobotStatus.MOVING:
                    burn = 0.35 if (m.current_task and m.current_task.status == TaskStatus.IN_PROGRESS) else 0.22
                else:
                    burn = 0.06

                m.state.battery = max(0.0, m.state.battery - burn)

                # Low battery safety threshold (<= 20.0%):
                # Shed an active task if needed, then route to a fixed charger.
                # A robot already travelling to a charger is allowed to finish that
                # route instead of repeatedly resetting itself to the same goal.
                if m.state.battery <= 20.0 and m.post_task_mode != "CHARGER":
                    orphaned = m.current_task
                    task_id = orphaned.task_id if orphaned else None
                    self._orphan_task(m)
                    m.reservation_table.expire(m.state.robot_id)
                    self._schedule_post_task_destination(m, "CHARGER")

                    self.event_log.log_conflict(
                        m.state.robot_id, "SYSTEM", "LOW_BATTERY_SHED",
                        f"Battery low ({m.state.battery:.1f}%) -> Task {task_id} reallocated; routing to charger",
                        self.tick_count
                    )
                    # Immediately reallocate the shed task to an available peer.
                    self._allocate()

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
        """Build the canonical dashboard telemetry snapshot.

        The simulator remains the source of truth.  Dashboard-only aliases are
        emitted here so the WebSocket payload has one stable contract without
        inventing values that the simulator does not model.
        """
        metrics_dict = dict(self.metric_values)
        metrics_dict.update({
            "makespan": self.metric_values.get(metrics.MAKESPAN, 0.0),
            "throughput": self.metric_values.get(metrics.THROUGHPUT, 0.0),
            "collision_count": self.metric_values.get(metrics.COLLISION_COUNT, 0),
            "deadlock_count": self.metric_values.get(metrics.DEADLOCK_COUNT, 0),
            "replan_count": self.metric_values.get(metrics.REPLAN_COUNT, 0),
            "waiting_time": self.metric_values.get(metrics.WAITING_TIME, 0.0),
            "sum_completion_time": self.metric_values.get(metrics.SUM_COMPLETION_TIME, 0.0),
            "avg_task_completion_time": self.metric_values.get(metrics.AVG_TASK_COMPLETION_TIME, 0.0),
            "comm_latency": self.metric_values.get(metrics.COMM_LATENCY, 0.0),
            "message_loss": self.metric_values.get(metrics.MESSAGE_LOSS, 0.0),
            "cpu_memory": self.metric_values.get(metrics.CPU_MEMORY, 0.0),
            "edge_inference_latency": self.metric_values.get(metrics.EDGE_INFERENCE_LATENCY, 0.0),
            "energy_proxy": self.metric_values.get(metrics.ENERGY_PROXY, 0.0),
        })

        grid = self.grid_map.grid if self.grid_map else []
        obstacles = [
            [x, y]
            for y, row in enumerate(grid)
            for x, cell in enumerate(row)
            if cell == '#'
        ]
        pickup_stations = [
            {"location": [x, y]}
            for x, y in (self.grid_map.find_all('P') if self.grid_map else [])
        ]
        dropoff_stations = [
            {"location": [x, y]}
            for x, y in (self.grid_map.find_all('D') if self.grid_map else [])
        ]
        staging_stations = [
            {"location": [x, y]}
            for x, y in (self.grid_map.find_all('S') if self.grid_map else [])
        ]
        charging_stations = [
            {"location": [x, y]}
            for x, y in (self.grid_map.find_all('C') if self.grid_map else [])
        ]
        spawn_points = [
            [x, y]
            for x, y in (self.grid_map.find_all('R') if self.grid_map else [])
        ]

        robots = {}
        for m in self.robot_managers:
            state = m.state
            robot = {
                "id": state.robot_id,
                "position": list(state.position),
                "heading": getattr(state, "heading", 0.0),
                "velocity": getattr(state, "velocity", 0.0),
                "state": state.status.value,
                "battery_pct": state.battery,
                "current_task_id": state.current_task_id,
                "task_priority": state.task_priority,
                "planned_path": [list(p) for p in state.planned_path],
                "communication_quality": state.communication_quality,
                "localization_confidence": state.localization_confidence,
                "has_cargo": (
                    m.current_task is not None
                    and m.current_task.status == TaskStatus.IN_PROGRESS
                ),
                "target_cell": list(m.target_cell) if m.target_cell is not None else None,
                "maintenance": getattr(m, "maintenance_profile", {}),
            }
            robots[state.robot_id] = robot

        tasks = [
            {
                "id": t.task_id,
                "pickup": list(t.pickup_cell),
                "dropoff": list(t.dropoff_cell),
                "priority": t.priority,
                "status": t.status.value,
                "assigned_robot_id": t.assigned_robot_id,
                "created_at": t.created_at,
            }
            for t in self.tasks
        ]

        return {
            "schema_version": 1,
            "timestamp": time.time(),
            "tick": self.tick_count,
            "scenario": getattr(self, "scenario_name", "S1_Normal"),
            "warehouse": {
                "width": self.grid_map.width if self.grid_map else 0,
                "height": self.grid_map.height if self.grid_map else 0,
                "grid": grid,
                "obstacles": obstacles,
                "pickup_stations": pickup_stations,
                "dropoff_stations": dropoff_stations,
                "staging_stations": staging_stations,
                "charging_stations": charging_stations,
                "spawn_points": spawn_points,
                "human_zones": [],
            },
            "robots": robots,
            "tasks": tasks,
            "conflicts": self.event_log.conflict_events[-20:],
            "deadlocks": self.event_log.deadlock_events[-10:],
            "metrics": metrics_dict,
            "congestion_heatmap": {"cells": []},
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
                elif char == 'S':
                    pygame.draw.rect(self.screen, (40, 170, 150), rect, border_radius=2)
                elif char == 'C':
                    pygame.draw.rect(self.screen, (230, 170, 60), rect, border_radius=2)
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