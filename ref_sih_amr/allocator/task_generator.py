import random
import uuid
from typing import List, Tuple
from models import Task, TaskStatus

class TaskGenerator:
    def __init__(self, pickup_cells: List[Tuple[int, int]], dropoff_cells: List[Tuple[int, int]], spawn_interval: int = 10):
        self.pickup_cells = pickup_cells
        self.dropoff_cells = dropoff_cells
        self.spawn_interval = spawn_interval
        self.ticks_since_last_spawn = 0
        self.queue: List[Task] = []

    def tick(self, current_time: float) -> List[Task]:
        """Called every simulation tick. Returns newly generated tasks."""
        self.ticks_since_last_spawn += 1
        new_tasks = []
        if self.ticks_since_last_spawn >= self.spawn_interval:
            self.ticks_since_last_spawn = 0
            if self.pickup_cells and self.dropoff_cells:
                pickup = random.choice(self.pickup_cells)
                dropoff = random.choice(self.dropoff_cells)
                task = Task(
                    task_id=str(uuid.uuid4()),
                    pickup_cell=pickup,
                    dropoff_cell=dropoff,
                    priority=1,
                    status=TaskStatus.QUEUED,
                    created_at=current_time
                )
                self.queue.append(task)
                new_tasks.append(task)
        return new_tasks
