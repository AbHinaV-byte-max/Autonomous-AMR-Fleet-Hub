from sim.simulator import Simulator
from models import RobotStatus


def test_post_task_staging_bays_are_unique_and_reassigned_if_occupied():
    """Completed AMRs must not fight over one fixed staging bay."""
    ascii_map = """\
###########
#R.......R#
#..S...S..#
#....D....#
###########
"""
    sim = Simulator(ascii_map=ascii_map, headless=True, strategy="P2P")
    sim.tasks.clear()
    sim.task_generator.queue.clear()

    # The simulator seeds initial work during construction. Clearing the
    # workload must also clear those obsolete routes/reservations; otherwise
    # this isolated service-bay test can mistake a stale peer route for a
    # live reservation on a staging bay.
    for manager in sim.robot_managers:
        manager.state.planned_path = []
        manager.target_cell = None
        manager.current_task = None
        manager.reservation_table = type(manager.reservation_table)()

    first, second = sim.robot_managers[:2]
    assert len(sim.staging_cells) >= 2

    assert sim._schedule_post_task_destination(first, "STAGING")
    first_goal = first.target_cell

    assert sim._schedule_post_task_destination(second, "STAGING")
    second_goal = second.target_cell

    assert first_goal != second_goal

    # Simulate another AMR occupying second's previously selected bay.
    first.target_cell = None
    first.post_task_mode = None
    first.state.planned_path = []
    first.state.position = (float(second_goal[0]), float(second_goal[1]))
    first.state.status = RobotStatus.STAGING

    second.target_cell = second_goal
    second.post_task_mode = "STAGING"
    second.state.planned_path = []
    second.state.status = RobotStatus.MOVING

    sim._retry_post_task_destinations()

    assert second.target_cell is not None
    assert second.target_cell != second_goal
    assert second.target_cell == first_goal
    assert second.state.status == RobotStatus.MOVING


def test_completed_robot_retries_service_bay_after_temporary_exhaustion():
    """A completed robot must not remain stranded on the delivery cell."""
    ascii_map = """\
###########
#R.......R#
#..S......#
#....D....#
###########
"""
    sim = Simulator(ascii_map=ascii_map, headless=True, strategy="P2P")
    sim.tasks.clear()
    sim.task_generator.queue.clear()

    first, second = sim.robot_managers[:2]
    for manager in sim.robot_managers:
        manager.state.planned_path = []
        manager.target_cell = None
        manager.current_task = None
        manager.reservation_table = type(manager.reservation_table)()

    dropoff = sim.dropoff_cells[0]
    staging = sim.staging_cells[0]

    # Simulate a completed AMR stranded on the delivery cell while its only
    # staging resource is temporarily occupied.
    first.state.position = (float(dropoff[0]), float(dropoff[1]))
    first.state.status = RobotStatus.WAITING
    first.post_task_mode = None

    second.state.position = (float(staging[0]), float(staging[1]))
    second.state.status = RobotStatus.STAGING
    second.post_task_mode = None

    sim._retry_post_task_destinations()

    assert first.post_task_mode in ("STAGING", "HOLD")
    assert first.target_cell != dropoff
    assert first.target_cell is not None
    assert first.state.status == RobotStatus.MOVING

    # Once the fixed staging bay becomes available, the completed AMR must
    # leave the overflow hold and take the real bay.
    second.state.position = (0.0, 1.0)
    second.state.status = RobotStatus.IDLE
    sim._retry_post_task_destinations()

    assert first.target_cell == staging
    assert first.state.status == RobotStatus.MOVING
    assert first.post_task_mode == "STAGING"


def test_inbound_payload_evicts_taskless_robot_from_dropoff():
    """A loaded AMR must not sit on pickup because a parked peer occupies the dock."""
    ascii_map = """\
###########
#R.......R#
#..S......#
#....D....#
###########
"""
    sim = Simulator(ascii_map=ascii_map, headless=True, strategy="P2P")
    sim.tasks.clear()
    sim.task_generator.queue.clear()
    sim.task_generator.enabled = False

    from models import Task, TaskStatus

    first, second = sim.robot_managers[:2]
    for manager in sim.robot_managers:
        manager.state.planned_path = []
        manager.target_cell = None
        manager.current_task = None
        manager.post_task_mode = None
        manager.reservation_table = type(manager.reservation_table)()

    dropoff = sim.dropoff_cells[0]
    pickup = (1, 1)

    first.state.position = (float(dropoff[0]), float(dropoff[1]))
    first.state.status = RobotStatus.WAITING
    first.post_task_mode = "STAGING"

    task = Task(
        task_id="MANUAL_PAYLOAD",
        pickup_cell=pickup,
        dropoff_cell=dropoff,
        priority=1,
        status=TaskStatus.IN_PROGRESS,
        created_at=0.0,
        source="MANUAL",
        assigned_robot_id=second.state.robot_id,
    )
    sim.tasks.append(task)
    second.current_task = task
    second.state.current_task_id = task.task_id
    second.state.position = (float(pickup[0]), float(pickup[1]))
    second.state.status = RobotStatus.MOVING
    second.target_cell = dropoff
    second.state.planned_path = []
    if not second.cbs_mode:
        second._replan()

    sim._vacate_dropoffs_for_inbound_cargo()

    first_pos = (int(first.state.position[0]), int(first.state.position[1]))
    assert first_pos == dropoff
    assert first.target_cell is not None
    assert first.target_cell != dropoff
    assert first.state.status == RobotStatus.MOVING


def test_repeated_manual_dispatch_delivers_after_auto_off():
    """Fourth manual order must still complete the dropoff after Auto Mode is off."""
    from experiments.runner import SCENARIOS
    from models import Task, TaskStatus

    sim = Simulator(SCENARIOS["S1_Normal"], headless=True, strategy="P2P")
    sim.task_generator.enabled = False
    sim.clear_queued_auto_tasks()
    for manager in sim.robot_managers:
        manager.current_task = None
        manager.state.current_task_id = None
        manager.state.planned_path = []
        manager.target_cell = None
        manager.post_task_mode = None
        manager.state.status = RobotStatus.IDLE

    pickup = (2, 5)
    dropoff = (3, 17)
    completed = 0
    for i in range(4):
        task = Task(
            task_id=f"MANUAL_{i}",
            pickup_cell=pickup,
            dropoff_cell=dropoff,
            priority=1,
            status=TaskStatus.QUEUED,
            created_at=float(sim.tick_count),
            source="MANUAL",
        )
        sim.tasks.append(task)
        sim._allocate()
        for _ in range(250):
            sim.tick()
            if task.status == TaskStatus.COMPLETED:
                completed += 1
                break
        assert task.status == TaskStatus.COMPLETED, (
            f"manual order {i} stuck at {task.status} after auto off"
        )
    assert completed == 4
