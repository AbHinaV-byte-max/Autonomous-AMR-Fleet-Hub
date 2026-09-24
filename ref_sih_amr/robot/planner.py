import heapq
from typing import List, Tuple, Any
from interfaces import Planner

class AStarPlanner(Planner):
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int], costmap: Any, reservation_table: Any = None, start_time: float = 0.0, robot_id: str = "") -> List[Tuple[int, int]]:
        """
        Space-Time A* path planning.
        """
        start_int = (int(start[0]), int(start[1]))
        goal_int = (int(goal[0]), int(goal[1]))
        
        if start_int == goal_int:
            return [start_int]
            
        def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
            
        frontier = []
        import heapq
        heapq.heappush(frontier, (0, start_time, start_int))
        
        came_from = {(start_int, start_time): None}
        cost_so_far = {(start_int, start_time): 0.0}
        
        final_state = None
        max_time = start_time + 400  # Cutoff search
        
        target_is_rack = False
        if costmap.get_cell(goal_int[0], goal_int[1]) == '#':
            target_is_rack = True
            
        def is_goal(cell: Tuple[int, int]) -> bool:
            if target_is_rack:
                return abs(cell[0] - goal_int[0]) + abs(cell[1] - goal_int[1]) == 1
            return cell == goal_int
        
        while frontier:
            _, t, current = heapq.heappop(frontier)
            
            if is_goal(current):
                final_state = (current, t)
                break
                
            if t > max_time:
                continue
                
            x, y = current
            # 4-directional + wait in place
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (0, 0)]:
                nxt = (x + dx, y + dy)
                nxt_t = t + 1
                
                char = costmap.get_cell(nxt[0], nxt[1])
                if char == '#':
                    continue
                    
                if reservation_table:
                    claimer = reservation_table.get_claimer(nxt, nxt_t)
                    if claimer and claimer != robot_id:
                        continue
                        
                    # Edge swap conflict
                    if dx != 0 or dy != 0:
                        r2 = reservation_table.get_claimer(nxt, t)
                        if r2 and r2 != robot_id:
                            if reservation_table.get_claimer(current, nxt_t) == r2:
                                continue
                else:
                    # If no reservation table, drop the time dimension to vastly speed up A* (used by allocator)
                    nxt_t = 0
                                
                new_cost = cost_so_far.get((current, t), 0) + 1.0
                if dx == 0 and dy == 0:
                    new_cost += 0.5  # Penalise waiting in-place: prefer detour over temporal stall
                
                if (nxt, nxt_t) not in cost_so_far or new_cost < cost_so_far[(nxt, nxt_t)]:
                    cost_so_far[(nxt, nxt_t)] = new_cost
                    priority = new_cost + heuristic(nxt, goal_int)
                    heapq.heappush(frontier, (priority, nxt_t, nxt))
                    came_from[(nxt, nxt_t)] = (current, t)
                    
        if final_state is None:
            return []
            
        path = []
        curr_state = final_state
        while curr_state is not None:
            path.append(curr_state[0])
            curr_state = came_from[curr_state]
            
        path.reverse()
        return path
