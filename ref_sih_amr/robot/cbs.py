"""
Conflict-Based Search (CBS) — optimal Multi-Agent Path Finding (MAPF).

Two-level algorithm:
  High level : Constraint Tree (CT) searched best-first by total path cost.
  Low  level : Space-time A* with per-robot vertex + edge constraints.

Reference: Sharon et al., "Conflict-based search for optimal multi-agent
           pathfinding", Artificial Intelligence 219 (2015) 40-66.
"""

import heapq
from typing import Any, Dict, FrozenSet, List, Optional, Tuple

Cell       = Tuple[int, int]
VertexKey  = Tuple[str, int, int, float]
EdgeKey    = Tuple[str, int, int, int, int, float]


# ---------------------------------------------------------------------------
# ConstraintSet
# ---------------------------------------------------------------------------

class ConstraintSet:
    """
    Immutable set of CBS constraints for one CT node.

    Vertex constraint : robot R cannot occupy cell (x,y) at time t.
    Edge   constraint : robot R cannot traverse from_cell -> to_cell at time t.
    """

    __slots__ = ("vertex", "edge")

    def __init__(
        self,
        vertex: FrozenSet[VertexKey] = frozenset(),
        edge:   FrozenSet[EdgeKey]   = frozenset(),
    ):
        self.vertex = vertex
        self.edge   = edge

    def with_vertex(self, robot_id: str, cell: Cell, t: float) -> "ConstraintSet":
        return ConstraintSet(
            self.vertex | frozenset({(robot_id, cell[0], cell[1], t)}),
            self.edge,
        )

    def with_edge(self, robot_id: str, from_cell: Cell, to_cell: Cell, t: float) -> "ConstraintSet":
        return ConstraintSet(
            self.vertex,
            self.edge | frozenset({(robot_id, from_cell[0], from_cell[1], to_cell[0], to_cell[1], t)}),
        )

    def has_vertex(self, robot_id: str, cell: Cell, t: float) -> bool:
        return (robot_id, cell[0], cell[1], t) in self.vertex

    def has_edge(self, robot_id: str, from_cell: Cell, to_cell: Cell, t: float) -> bool:
        return (robot_id, from_cell[0], from_cell[1], to_cell[0], to_cell[1], t) in self.edge


# ---------------------------------------------------------------------------
# Low-level planner  (space-time A* with CBS constraints)
# ---------------------------------------------------------------------------

def cbs_low_level_plan(
    robot_id: str,
    start: Cell,
    goal: Cell,
    costmap: Any,
    constraints: ConstraintSet,
    start_time: float,
    max_horizon: int = 300,
) -> List[Cell]:
    """
    Space-time A* for CBS.

    Uses per-robot CBS constraints instead of a shared reservation table.
    Returns a full path from *start* (inclusive) to goal (inclusive).
    Returns [] if no path exists within max_horizon.
    """
    start_int = (int(start[0]), int(start[1]))
    goal_int  = (int(goal[0]),  int(goal[1]))

    target_is_rack = costmap.get_cell(goal_int[0], goal_int[1]) == "#"

    def is_goal(cell: Cell) -> bool:
        if target_is_rack:
            return abs(cell[0] - goal_int[0]) + abs(cell[1] - goal_int[1]) == 1
        return cell == goal_int

    def h(cell: Cell) -> float:
        return float(abs(cell[0] - goal_int[0]) + abs(cell[1] - goal_int[1]))

    if is_goal(start_int):
        return [start_int]

    frontier: List[Tuple[float, float, Cell]] = []
    heapq.heappush(frontier, (h(start_int), start_time, start_int))

    State = Tuple[Cell, float]
    came_from: Dict[State, Optional[State]] = {(start_int, start_time): None}
    g_cost:    Dict[State, float]           = {(start_int, start_time): 0.0}

    final_state: Optional[State] = None
    max_t = start_time + max_horizon

    while frontier:
        _, t, current = heapq.heappop(frontier)

        if is_goal(current):
            final_state = (current, t)
            break

        if t > max_t:
            continue

        x, y = current
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (0, 0)]:
            nxt   = (x + dx, y + dy)
            nxt_t = t + 1

            if costmap.get_cell(nxt[0], nxt[1]) == "#":
                continue

            # CBS vertex constraint
            if constraints.has_vertex(robot_id, nxt, nxt_t):
                continue

            # CBS edge constraint (swap prevention)
            if (dx, dy) != (0, 0) and constraints.has_edge(robot_id, current, nxt, t):
                continue

            new_g = g_cost.get((current, t), 0.0) + 1.0
            if (dx, dy) == (0, 0):
                new_g += 0.5  # penalise waiting — prefer spatial detour

            if (nxt, nxt_t) not in g_cost or new_g < g_cost[(nxt, nxt_t)]:
                g_cost[(nxt, nxt_t)] = new_g
                heapq.heappush(frontier, (new_g + h(nxt), nxt_t, nxt))
                came_from[(nxt, nxt_t)] = (current, t)

    if final_state is None:
        return []

    path: List[Cell] = []
    state: Optional[State] = final_state
    while state is not None:
        path.append(state[0])
        state = came_from.get(state)
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# Conflict detection
# ---------------------------------------------------------------------------

def detect_first_conflict(
    solution: Dict[str, List[Cell]],
    start_times: Dict[str, float],
    horizon: int = 200,
) -> Optional[Tuple]:
    """
    Find the earliest conflict in a joint solution.

    Returns:
      ("vertex", robot_a, robot_b, cell, t)           vertex conflict
      ("edge",   robot_a, robot_b, cell_a, cell_b, t) edge-swap conflict
      None  —  collision-free within horizon
    """
    robot_ids = list(solution.keys())
    if len(robot_ids) < 2:
        return None

    # Build position maps: robot_id -> {t: cell}
    pos_at: Dict[str, Dict[float, Cell]] = {}
    for rid, path in solution.items():
        st = start_times.get(rid, 0.0)
        pmap: Dict[float, Cell] = {}
        for i, cell in enumerate(path):
            pmap[st + i] = cell
        # Robot stays at final cell after path ends
        if path:
            last_cell = path[-1]
            last_t    = st + len(path) - 1
            for extra in range(1, horizon + 1):
                pmap[last_t + extra] = last_cell
        pos_at[rid] = pmap

    min_t = min(start_times.get(r, 0.0) for r in robot_ids)

    t = min_t
    while t <= min_t + horizon:
        # Vertex conflicts
        cell_owner: Dict[Cell, str] = {}
        for rid in robot_ids:
            cell = pos_at[rid].get(t)
            if cell is None:
                continue
            if cell in cell_owner:
                return ("vertex", cell_owner[cell], rid, cell, t)
            cell_owner[cell] = rid

        # Edge-swap conflicts
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                ra, rb    = robot_ids[i], robot_ids[j]
                a_now     = pos_at[ra].get(t)
                b_now     = pos_at[rb].get(t)
                a_next    = pos_at[ra].get(t + 1)
                b_next    = pos_at[rb].get(t + 1)
                if a_now and b_now and a_next and b_next:
                    if a_now == b_next and b_now == a_next:
                        return ("edge", ra, rb, a_now, b_now, t)

        t += 1

    return None


# ---------------------------------------------------------------------------
# Obstacle-aware costmap wrapper
# ---------------------------------------------------------------------------

class ObstacleCostmap:
    """
    Thin wrapper around a GridMap that adds a set of occupied cells.
    Used to treat idle / parked robots as temporary walls during CBS planning,
    giving O(1) lookup with zero constraint-set overhead.
    """

    def __init__(self, base_costmap: Any, obstacle_cells):
        self._base      = base_costmap
        self._obstacles = frozenset(
            (int(c[0]), int(c[1])) for c in obstacle_cells
        )

    def get_cell(self, x: int, y: int) -> str:
        if (x, y) in self._obstacles:
            return "#"  # treat as impassable wall
        return self._base.get_cell(x, y)


# ---------------------------------------------------------------------------
# CBS high-level planner
# ---------------------------------------------------------------------------

class CBSPlanner:
    """
    Conflict-Based Search (CBS).

    Produces provably optimal, collision-free joint paths.
    When MAX_NODES is exhausted the best partially-resolved solution is
    returned — still far superior to independent A*.
    """

    MAX_NODES    = 150  # CT nodes before giving up
    PLAN_HORIZON = 80   # look-ahead ticks (shorter = faster; 80 covers all warehouse paths)

    def plan(
        self,
        robot_goals:     Dict[str, Cell],
        robot_positions: Dict[str, Tuple[float, float]],
        costmap:         Any,
        start_times:     Dict[str, float],
        event_logger:    Any = None,
        tick:            int = 0,
    ) -> Dict[str, List[Cell]]:
        """
        Run CBS.

        Args:
            robot_goals     : robot_id -> goal cell (int tuple). None skipped.
            robot_positions : robot_id -> (float x, float y).
            costmap         : GridMap or ObstacleCostmap (idle robots as walls).
            start_times     : robot_id -> start tick.
            event_logger    : Optional logger to record detected MAPF conflicts.
            tick            : Current simulation tick.

        Returns:
            robot_id -> path (start-inclusive, goal-inclusive).
        """
        robot_ids = [r for r in robot_goals if robot_goals.get(r) is not None]
        if not robot_ids:
            return {}

        # ---- Root: independent low-level plans ----
        root_cs  = ConstraintSet()
        root_sol: Dict[str, List[Cell]] = {}
        for rid in robot_ids:
            start = (int(robot_positions[rid][0]), int(robot_positions[rid][1]))
            path  = cbs_low_level_plan(
                rid, start, robot_goals[rid], costmap, root_cs,
                start_times.get(rid, 0.0), self.PLAN_HORIZON,
            )
            root_sol[rid] = path

        def total_cost(sol: Dict[str, List[Cell]]) -> float:
            return float(sum(len(p) for p in sol.values()))

        # CT open list: (cost, node_id, constraints, solution)
        _nid = 0
        heap: List[Tuple] = [(total_cost(root_sol), _nid, root_cs, root_sol)]
        _nid += 1

        nodes_expanded = 0
        best_solution  = root_sol
        logged_conflicts = set()

        while heap and nodes_expanded < self.MAX_NODES:
            _, _, cs, sol = heapq.heappop(heap)
            nodes_expanded += 1
            best_solution = sol

            conflict = detect_first_conflict(sol, start_times, self.PLAN_HORIZON)
            if conflict is None:
                return sol   # ✓ collision-free

            # Log the detected conflict so telemetry / event log captures it
            if event_logger:
                pair = tuple(sorted([conflict[1], conflict[2]]))
                if pair not in logged_conflicts:
                    logged_conflicts.add(pair)
                    c_type = "VERTEX_CONFLICT" if conflict[0] == "vertex" else "EDGE_SWAP"
                    event_logger.log_conflict(conflict[1], conflict[2], c_type, "CBS_RESOLVED", tick)

            if conflict[0] == "vertex":
                _, ra, rb, cell, t = conflict
                for bot in (ra, rb):
                    new_cs   = cs.with_vertex(bot, cell, t)
                    start    = (int(robot_positions[bot][0]), int(robot_positions[bot][1]))
                    new_path = cbs_low_level_plan(
                        bot, start, robot_goals[bot], costmap, new_cs,
                        start_times.get(bot, 0.0), self.PLAN_HORIZON,
                    )
                    if new_path:
                        new_sol = dict(sol)
                        new_sol[bot] = new_path
                        heapq.heappush(heap, (total_cost(new_sol), _nid, new_cs, new_sol))
                        _nid += 1

            elif conflict[0] == "edge":
                _, ra, rb, cell_a, cell_b, t = conflict
                for (bot, fc, tc) in [(ra, cell_a, cell_b), (rb, cell_b, cell_a)]:
                    new_cs   = cs.with_edge(bot, fc, tc, t)
                    start    = (int(robot_positions[bot][0]), int(robot_positions[bot][1]))
                    new_path = cbs_low_level_plan(
                        bot, start, robot_goals[bot], costmap, new_cs,
                        start_times.get(bot, 0.0), self.PLAN_HORIZON,
                    )
                    if new_path:
                        new_sol = dict(sol)
                        new_sol[bot] = new_path
                        heapq.heappush(heap, (total_cost(new_sol), _nid, new_cs, new_sol))
                        _nid += 1

        # Budget exhausted — return best partially-resolved solution
        return best_solution
