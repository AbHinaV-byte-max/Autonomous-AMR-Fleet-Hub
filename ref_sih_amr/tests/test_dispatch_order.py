import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from experiments.runner import SCENARIOS


# Contract used by the manual dispatch UI for S1_Normal.
S1_PICKUP_STATIONS = {
    "PK-1 (Aisle 1)": (2, 5),
    "PK-2 (Aisle 2)": (14, 5),
    "PK-3 (Aisle 3)": (27, 5),
}

S1_TARGET_DOCKS = {
    "DD-1 (Left Bay)": (3, 17),
    "DD-2 (Right Bay)": (27, 17),
}


def test_s1_dispatch_points_are_real_map_cells():
    grid = SCENARIOS["S1_Normal"].splitlines()

    for label, (x, y) in S1_PICKUP_STATIONS.items():
        assert grid[y][x] == ".", f"{label} must be a traversable pickup cell"

    for label, (x, y) in S1_TARGET_DOCKS.items():
        assert grid[y][x] == "D", f"{label} must point to a real delivery dock"


def test_s1_dispatch_points_are_distinct():
    assert len(set(S1_PICKUP_STATIONS.values())) == len(S1_PICKUP_STATIONS)
    assert len(set(S1_TARGET_DOCKS.values())) == len(S1_TARGET_DOCKS)


def test_manual_dispatch_can_preempt_unstarted_auto_task():
    from models import Task, TaskStatus
    from sim.simulator import Simulator

    sim = Simulator(SCENARIOS["S1_Normal"], headless=True, strategy="P1")
    sim.task_generator.enabled = False

    manual = Task(
        task_id="MANUAL_PRIORITY",
        pickup_cell=S1_PICKUP_STATIONS["PK-1 (Aisle 1)"],
        dropoff_cell=S1_TARGET_DOCKS["DD-1 (Left Bay)"],
        priority=1,
        status=TaskStatus.QUEUED,
        created_at=sim.tick_count,
        source="MANUAL",
    )
    sim.tasks.append(manual)

    assigned = sim.dispatch_manual_task(manual)

    assert assigned is not None
    assert manual.status == TaskStatus.ASSIGNED
    manager = next(m for m in sim.robot_managers if m.state.robot_id == assigned)
    assert manager.current_task is manual

    # The displaced automatic task remains recoverable/queued rather than lost.
    displaced = [
        t for t in sim.tasks
        if t.task_id != manual.task_id and t.source == "AUTO"
        and t.assigned_robot_id is None
        and t.status == TaskStatus.RECOVERABLE
    ]
    assert displaced


def test_manual_dispatch_never_preempts_loaded_auto_robot():
    from models import TaskStatus
    from sim.simulator import Simulator

    sim = Simulator(SCENARIOS["S1_Normal"], headless=True, strategy="P1")
    sim.task_generator.enabled = False

    loaded = sim.robot_managers[0]
    loaded_task = loaded.current_task
    assert loaded_task is not None
    loaded_task.status = TaskStatus.IN_PROGRESS
    loaded.target_cell = loaded_task.dropoff_cell

    manual = __import__("models").Task(
        task_id="MANUAL_NO_PAYLOAD_PREEMPT",
        pickup_cell=S1_PICKUP_STATIONS["PK-2 (Aisle 2)"],
        dropoff_cell=S1_TARGET_DOCKS["DD-2 (Right Bay)"],
        priority=1,
        status=TaskStatus.QUEUED,
        created_at=sim.tick_count,
        source="MANUAL",
    )
    sim.tasks.append(manual)

    assigned = sim.dispatch_manual_task(manual)

    assert assigned is not None
    assert assigned != loaded.state.robot_id
    assert loaded.current_task is loaded_task
    assert loaded_task.status == TaskStatus.IN_PROGRESS
