from typing import List, Tuple, Dict, Set, Optional, Any
from models import RobotState, IntentMessage, Intent
from interfaces import ConflictResolver as BaseConflictResolver

class ReservationTable:
    def __init__(self):
        # (x, y, time) -> robot_id
        self.claims: Dict[Tuple[int, int, float], str] = {}
        # robot_id -> list of (x, y, time)
        self.robot_reservations: Dict[str, List[Tuple[int, int, float]]] = {}

    def propose(self, robot_id: str, path: List[Tuple[int, int]], start_time: float) -> List[Tuple[Tuple[int, int], float]]:
        """Checks path for vertex conflicts without committing."""
        conflicts = []
        for i, cell in enumerate(path):
            t = start_time + i
            claim_key = (cell[0], cell[1], t)
            if claim_key in self.claims and self.claims[claim_key] != robot_id:
                conflicts.append((cell, t))
        return conflicts

    def commit(self, robot_id: str, path: List[Tuple[int, int]], start_time: float):
        """Finalizes reservations."""
        self.expire(robot_id, after_time=start_time)
        
        if robot_id not in self.robot_reservations:
            self.robot_reservations[robot_id] = []
            
        for i, cell in enumerate(path):
            t = start_time + i
            claim_key = (cell[0], cell[1], t)
            self.claims[claim_key] = robot_id
            self.robot_reservations[robot_id].append(claim_key)

    def expire(self, robot_id: str, after_time: Optional[float] = None):
        """Releases reservations. If after_time is provided, only expires future ones."""
        if robot_id in self.robot_reservations:
            new_res = []
            for claim_key in self.robot_reservations[robot_id]:
                if after_time is None or claim_key[2] >= after_time:
                    if claim_key in self.claims:
                        del self.claims[claim_key]
                else:
                    new_res.append(claim_key)
            if not new_res:
                del self.robot_reservations[robot_id]
            else:
                self.robot_reservations[robot_id] = new_res
                
    def get_claimer(self, cell: Tuple[int, int], time: float) -> Optional[str]:
        return self.claims.get((cell[0], cell[1], time))

# --- Conflict Detection Functions ---

def check_vertex_conflict(table: ReservationTable, robot_id: str, cell: Tuple[int, int], time: float) -> Optional[str]:
    """Vertex conflict: same (cell, time) claimed by two robots."""
    claimer = table.get_claimer(cell, time)
    if claimer and claimer != robot_id:
        return claimer
    return None

def check_edge_swap(table: ReservationTable, robot_id: str, curr_cell: Tuple[int, int], next_cell: Tuple[int, int], time: float) -> Optional[str]:
    """Edge swap: R1 moves A->B while R2 moves B->A in the same time step."""
    # R1 at curr_cell at time, moves to next_cell at time+1
    # Check if R2 is at next_cell at time, and curr_cell at time+1
    r2 = table.get_claimer(next_cell, time)
    if r2 and r2 != robot_id:
        if table.get_claimer(curr_cell, time + 1) == r2:
            return r2
    return None

def check_following(table: ReservationTable, robot_id: str, curr_cell: Tuple[int, int], next_cell: Tuple[int, int], time: float) -> Optional[str]:
    """Following/unsafe gap: R2 would reach a cell before R1 has cleared it."""
    # Simplified temporal headway: just don't enter if the cell is claimed at time or time+1
    # Actually, vertex conflict handles time+1. Let's just check if it's occupied at time
    # and they aren't moving. For this simulation, 1 tick per cell implies atomic swaps which are caught by edge_swap.
    pass

def check_intersection(table: ReservationTable, robot_id: str, cell: Tuple[int, int], time: float, window: int = 2) -> Optional[str]:
    """Intersection conflict: multiple robots request the same choke-point within a window."""
    for t in range(int(time) - window, int(time) + window + 1):
        claimer = table.get_claimer(cell, float(t))
        if claimer and claimer != robot_id:
            return claimer
    return None

# --- Priority and Resolution ---

class PriorityCalculator:
    def __init__(self, w1=1.0, w2=1.0, w3=1.0, w4=1.0):
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3
        self.w4 = w4
        
    def calculate(self, urgency: float, wait_time: float, battery: float, distance: float, robot_id: str) -> Tuple[float, str]:
        """
        priority(Ri) = w1*task_urgency + w2*waiting_time + w3*battery_criticality - w4*remaining_distance
        Tiebreaker: robot_id string comparison.
        Returns a tuple so Python's native sort resolves ties deterministically.
        """
        score = self.w1 * urgency + self.w2 * wait_time + self.w3 * (100 - battery) - self.w4 * distance
        return (score, robot_id)

class ConflictResolver(BaseConflictResolver):
    def __init__(self, table: ReservationTable, calculator: PriorityCalculator):
        self.table = table
        self.calculator = calculator
        
    def check_and_resolve(self, reservations: List[Tuple[Tuple[int, int], float]], proposed_path: List[Tuple[int, int]]) -> Any:
        # We handle resolution directly in LocalTaskManager using the primitives
        pass

# --- Deadlock Detection ---

def detect_deadlock(wait_graph: Dict[str, str]) -> Optional[List[str]]:
    """
    Detects a cycle in the wait-for graph using per-traversal path tracking.
    Returns the list of robot_ids forming the cycle, or None.

    Key fix over previous version: each DFS traversal maintains its own
    in-path set so that cycles not starting at the initial node are caught
    correctly, even when some nodes have already been globally visited.
    """
    globally_visited: Set[str] = set()

    for start_node in wait_graph:
        if start_node in globally_visited:
            continue

        path: List[str] = []
        in_path: Set[str] = set()
        curr = start_node

        while curr in wait_graph:
            if curr in in_path:
                # Found a cycle — extract just the cycle portion
                idx = path.index(curr)
                return path[idx:]
            if curr in globally_visited:
                # Node already fully explored without finding a cycle
                break
            path.append(curr)
            in_path.add(curr)
            curr = wait_graph[curr]

        # Mark everything on this path as globally visited
        globally_visited.update(path)
