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

    assert first.post_task_mode == "STAGING"
    assert first.target_cell is None
    assert first.state.status == RobotStatus.WAITING

    # Once the fixed staging bay becomes available, the completed AMR must
    # leave the delivery cell on the next retry.
    second.state.position = (0.0, 1.0)
    sim._retry_post_task_destinations()

    assert first.target_cell == staging
    assert first.state.status == RobotStatus.MOVING
