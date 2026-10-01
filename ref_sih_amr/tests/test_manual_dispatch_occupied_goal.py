from sim.simulator import Simulator
from models import Task, TaskStatus, RobotStatus


OCCUPIED_GOAL_MAP = """\
#########
#R...R..#
#.......#
#...D...#
#########
"""


def test_p2p_manual_dispatch_holds_when_pickup_is_occupied_by_idle_peer():
    """A manual order must wait, not oscillate, when its pickup is parked on."""
    sim = Simulator(
        ascii_map=OCCUPIED_GOAL_MAP,
        headless=True,
        strategy="P2P",
        comms_mode="local",
    )
    sim.tasks.clear()
    sim.task_generator.queue.clear()
    sim.task_generator.enabled = False

    mover, blocker = sim.robot_managers[:2]
    mover.state.position = (1.0, 1.0)
    blocker.state.position = (5.0, 1.0)
    blocker.state.status = RobotStatus.IDLE
    blocker.current_task = None
    blocker.target_cell = None
    blocker.state.planned_path = []

    task = Task(
        task_id="MANUAL_OCCUPIED_PICKUP",
        pickup_cell=(5, 1),
        dropoff_cell=(4, 3),
        priority=5,
        status=TaskStatus.QUEUED,
        created_at=0.0,
        source="MANUAL",
    )
    sim.tasks.append(task)
    mover.assign_task(task)

    positions = []
    for _ in range(20):
        sim.tick()
        positions.append(
            (int(mover.state.position[0]), int(mover.state.position[1]))
        )

    assert task.status == TaskStatus.ASSIGNED
    assert mover.state.status == RobotStatus.WAITING
    assert mover.waiting_on == blocker.state.robot_id
    assert mover.target_cell == (5, 1)
    assert not mover.state.planned_path
    assert all(pos != blocker.state.position for pos in positions)
    # Once the peer's occupancy is learned, the mover must settle instead of
    # bouncing between detour cells while the goal remains permanently blocked.
    assert len(set(positions[-10:])) == 1
    assert sim.metric_values["COLLISION_COUNT"] == 0
