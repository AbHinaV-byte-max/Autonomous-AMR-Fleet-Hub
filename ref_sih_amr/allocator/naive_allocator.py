from typing import List, Dict
from models import RobotState, Task, RobotStatus, TaskStatus
from interfaces import TaskAllocator

class NaiveAllocator(TaskAllocator):
    def allocate(self, robots: List[RobotState], tasks: List[Task]) -> Dict[str, str]:
        """
        Assigns the next queued task to the first idle robot.
        """
        assignment = {}
        idle_robots = [r for r in robots if r.status in (RobotStatus.IDLE, None)]
        queued_tasks = [t for t in tasks if t.status == TaskStatus.QUEUED]
        
        for i in range(min(len(idle_robots), len(queued_tasks))):
            robot = idle_robots[i]
            task = queued_tasks[i]
            assignment[robot.robot_id] = task.task_id
            
        return assignment
