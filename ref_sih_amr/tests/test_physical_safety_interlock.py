from sim.simulator import Simulator
from models import RobotStatus


def test_physical_safety_stop_rolls_back_mover_not_stationary_occupant():
    """A stale peer intent must never displace the robot already occupying a cell."""
    ascii_map = """\\
#######
#R...R#
#.....#
#######
"""
    sim = Simulator(ascii_map=ascii_map, headless=True, strategy="P2P")
    sim.tasks.clear()
    sim.task_generator.queue.clear()

    mover, occupant = sim.robot_managers[:2]

    mover.state.position = (2.0, 1.0)
    mover.state.status = RobotStatus.MOVING
    mover.target_cell = (3, 1)
    mover.state.planned_path = [(3, 1)]
    mover.peer_states.clear()
    mover.reservation_table.expire(mover.state.robot_id)

    occupant.state.position = (3.0, 1.0)
    occupant.state.status = RobotStatus.WAITING
    occupant.current_task = None
    occupant.target_cell = None
    occupant.state.planned_path = []
    occupant.peer_states.clear()
    occupant.reservation_table.expire(occupant.state.robot_id)

    sim.tick()

    assert (int(mover.state.position[0]), int(mover.state.position[1])) == (2, 1)
    assert (int(occupant.state.position[0]), int(occupant.state.position[1])) == (3, 1)
    assert sim.metric_values["COLLISION_COUNT"] == 0
    assert any(
        event["type"] == "PHYSICAL_SAFETY_STOP"
        and event["robot_id"] == mover.state.robot_id
        and event["peer_id"] == occupant.state.robot_id
        for event in sim.event_log.conflict_events
    )
