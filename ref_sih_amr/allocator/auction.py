"""Robot-local auction allocator for decentralized task bidding.

Each eligible robot computes its own bid from its local A* cost and task
priority. The simulator only resolves the submitted bids; it does not run
a global Hungarian optimization for the live P2P strategy.
"""

from typing import Dict, List, Any


class AuctionAllocator:
    def __init__(self, planner: Any, costmap: Any = None):
        self.planner = planner
        self.costmap = costmap

    def bid(self, robot, task) -> float:
        path = self.planner.plan(robot.position, task.pickup_cell, self.costmap)
        if not path:
            return float("inf")
        distance = max(0, len(path) - 1)
        urgency_penalty = max(0, 10 - int(task.priority)) * 0.5
        battery_penalty = max(0.0, 30.0 - float(robot.battery)) * 0.2
        return float(distance) + urgency_penalty + battery_penalty

    def allocate(self, robots: List[Any], tasks: List[Any]) -> Dict[str, str]:
        assignments: Dict[str, str] = {}
        remaining = list(tasks)
        available = list(robots)
        while remaining and available:
            candidates = []
            for robot in available:
                for task in remaining:
                    candidates.append((self.bid(robot, task), robot.robot_id, task.task_id))
            candidates.sort(key=lambda item: (item[0], item[1], item[2]))
            bid, robot_id, task_id = candidates[0]
            if bid == float("inf"):
                break
            assignments[robot_id] = task_id
            remaining = [task for task in remaining if task.task_id != task_id]
            available = [robot for robot in available if robot.robot_id != robot_id]
        return assignments