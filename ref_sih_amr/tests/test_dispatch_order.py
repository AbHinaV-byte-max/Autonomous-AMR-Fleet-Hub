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


def test_p1_initial_allocation_injects_executable_paths():
    from experiments.runner import SCENARIOS
    from sim.simulator import Simulator

    sim = Simulator(SCENARIOS["S1_Normal"], headless=True, strategy="P1")
    active = [m for m in sim.robot_managers if m.current_task is not None]
    assert active, "P1 should assign initial work"
    assert any(m.state.planned_path for m in active), "P1 initial assignments need executable CBS/A* paths"


def test_auto_tick_keeps_moving_after_task_generation():
    from experiments.runner import SCENARIOS
    from sim.simulator import Simulator

    sim = Simulator(SCENARIOS["S1_Normal"], headless=True, strategy="P1")
    sim.task_generator.enabled = True
    before = {m.state.robot_id: tuple(m.state.position) for m in sim.robot_managers}
    for _ in range(3):
        sim.tick()
    after = {m.state.robot_id: tuple(m.state.position) for m in sim.robot_managers}
    assert any(before[rid] != after[rid] for rid in before), f"No AMR moved: before={before}, after={after}"


def test_completed_amr_leaves_delivery_bay_for_staging():
    from experiments.runner import SCENARIOS
    from sim.simulator import Simulator
    from models import RobotStatus

    sim = Simulator(SCENARIOS["S1_Normal"], headless=True, strategy="P1")
    robot = sim.robot_managers[0]

    # Isolate a completed robot parked on a delivery bay.
    for other in sim.robot_managers[1:]:
        other.state.status = RobotStatus.OFFLINE
        other.state.planned_path = []
        other.current_task = None
        other.target_cell = None
        sim.global_reservation_table.expire(other.state.robot_id)

    robot.current_task = None
    robot.state.current_task_id = None
    robot.state.status = RobotStatus.IDLE
    robot.state.planned_path = []
    robot.target_cell = None
    robot.state.position = tuple(float(v) for v in sim.dropoff_cells[0])
    sim.global_reservation_table.expire(robot.state.robot_id)

    sim._stage_idle_robots()

    assert robot.state.status == RobotStatus.MOVING
    assert getattr(robot, "staging_target", None) in sim.staging_cells
    assert tuple(robot.target_cell) != tuple(sim.dropoff_cells[0])

    # Let the robot complete the return-to-staging leg.
    for _ in range(100):
        sim.tick()
        if robot.state.status == RobotStatus.IDLE and getattr(robot, "staging_target", None) is None:
            break

    assert robot.state.status == RobotStatus.IDLE
    assert tuple(robot.state.position) in sim.staging_cells
    assert tuple(robot.state.position) not in sim.dropoff_cells


def test_manual_dispatch_can_use_robot_after_post_delivery_staging():
    from experiments.runner import SCENARIOS
    from sim.simulator import Simulator
    from models import RobotStatus, Task, TaskStatus

    sim = Simulator(SCENARIOS["S1_Normal"], headless=True, strategy="P1")
    sim.task_generator.enabled = False

    # Turn one completed robot into a staged, available fleet member.
    robot = sim.robot_managers[0]
    for other in sim.robot_managers[1:]:
        other.state.status = RobotStatus.OFFLINE
        other.state.planned_path = []
        other.current_task = None
        other.target_cell = None
        sim.global_reservation_table.expire(other.state.robot_id)

    robot.current_task = None
    robot.state.current_task_id = None
    robot.state.status = RobotStatus.IDLE
    robot.state.planned_path = []
    robot.target_cell = None
    robot.state.position = tuple(float(v) for v in sim.dropoff_cells[0])
    sim.global_reservation_table.expire(robot.state.robot_id)
    sim._stage_idle_robots()

    for _ in range(100):
        sim.tick()
        if robot.state.status == RobotStatus.IDLE and getattr(robot, "staging_target", None) is None:
            break

    pickup = sim.pickup_cells[0]
    dropoff = sim.dropoff_cells[-1]
    task = Task(
        task_id="MANUAL_STAGING_TEST",
        pickup_cell=pickup,
        dropoff_cell=dropoff,
        priority=1,
        status=TaskStatus.QUEUED,
        source="MANUAL",
    )
    sim.tasks.append(task)
    sim._allocate()

    assert task.assigned_robot_id == robot.state.robot_id
    assert task.status == TaskStatus.ASSIGNED
    assert robot.current_task is task
