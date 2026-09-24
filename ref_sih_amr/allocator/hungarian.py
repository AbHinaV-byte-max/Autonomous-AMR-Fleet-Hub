import numpy as np
from scipy.optimize import linear_sum_assignment
from typing import List, Dict, Any
try:
    from ref_sih_amr.models import RobotState, Task, RobotStatus, TaskStatus
    from ref_sih_amr.interfaces import TaskAllocator, Planner
except (ImportError, ModuleNotFoundError):
    from models import RobotState, Task, RobotStatus, TaskStatus
    from interfaces import TaskAllocator, Planner

class HungarianAllocator(TaskAllocator):
    def __init__(self, planner: Planner, costmap: Any = None):
        self.planner = planner
        self.costmap = costmap

    def allocate(self, robots: List[RobotState], tasks: List[Task]) -> Dict[str, str]:
        """
        Allocates tasks to robots using the Hungarian Algorithm to minimize total travel time.
        """
        def is_idle(r):
            st = getattr(r, "status", None)
            if st is None or st == "IDLE":
                return True
            if hasattr(st, "value") and st.value == "IDLE":
                return True
            return st == RobotStatus.IDLE

        def is_queued(t):
            st = getattr(t, "status", None)
            if st is None or st in ("QUEUED", "RECOVERABLE"):
                return True
            if hasattr(st, "value") and st.value in ("QUEUED", "RECOVERABLE"):
                return True
            return st in (TaskStatus.QUEUED, TaskStatus.RECOVERABLE)

        idle_robots = [r for r in robots if is_idle(r)]
        queued_tasks = [t for t in tasks if is_queued(t)]

        if not idle_robots or not queued_tasks:
            return {}

        n_robots = len(idle_robots)
        n_tasks = len(queued_tasks)
        
        # Build cost matrix: C(Ri, Tj)
        cost_matrix = np.zeros((n_robots, n_tasks))
        
        for i, robot in enumerate(idle_robots):
            for j, task in enumerate(queued_tasks):
                # Estimated travel time from A* path length
                path = self.planner.plan(robot.position, task.pickup_cell, self.costmap)
                if not path:
                    cost_matrix[i, j] = 999999.0 # Unreachable penalty
                else:
                    cost_matrix[i, j] = float(len(path) - 1)
                    
        # Note: Section 8.2 explicitly warns distance-in-metres can dominate 
        # a 0-1 priority score if not normalized. 
        # Normalize costs before combining.
        max_cost = np.max(cost_matrix)
        if max_cost > 0:
            cost_matrix = cost_matrix / max_cost
            
        # In the future (Phase 3/4), add weighted terms:
        # congestion_cost = 0.0
        # battery_penalty = 0.0
        # task_priority_penalty = 0.0
        # reassignment_penalty = 0.0
        
        # Scipy handles non-square matrices automatically
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        
        assignment = {}
        for r_idx, t_idx in zip(row_ind, col_ind):
            path_len = cost_matrix[r_idx, t_idx] * max_cost if max_cost > 0 else cost_matrix[r_idx, t_idx]
            if path_len < 999999.0:
                robot_id = idle_robots[r_idx].robot_id
                task_id = queued_tasks[t_idx].task_id
                assignment[robot_id] = task_id
                
        return assignment
