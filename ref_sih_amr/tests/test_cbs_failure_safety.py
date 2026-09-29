import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from models import RobotStatus
from sim.simulator import Simulator


class EmptyCBS:
    def plan(self, *args, **kwargs):
        return {}


def test_cbs_failure_clears_stale_routes_and_enters_rerouting():
    sim = Simulator(
        ascii_map="""########
#R....D#
#......#
########
""",
        headless=True,
        strategy="P1",
    )

    manager = sim.robot_managers[0]
    manager.target_cell = (6, 1)
    manager.state.status = RobotStatus.MOVING
    manager.state.planned_path = [(2, 1), (3, 1), (4, 1)]

    sim.cbs_planner = EmptyCBS()
    sim._run_cbs_planning()

    assert manager.state.planned_path == []
    assert manager.state.status == RobotStatus.REROUTING
    assert any(
        event["type"] == "CBS_NO_SOLUTION"
        for event in sim.event_log.conflict_events
    )


class PartialCBS:
    def plan(self, *args, **kwargs):
        goals = args[0]
        first = next(iter(goals))
        return {first: [(1, 1), (2, 1)]}


def test_cbs_partial_result_never_leaves_missing_robot_on_stale_route():
    sim = Simulator(
        ascii_map="""########
#R....R#
#......#
#..DD..#
########
""",
        headless=True,
        strategy="P1",
    )

    for index, manager in enumerate(sim.robot_managers):
        manager.target_cell = (3 + index, 3)
        manager.state.status = RobotStatus.MOVING
        manager.state.planned_path = [(2 + index, 1), (3 + index, 1)]

    stale_ids = [m.state.robot_id for m in sim.robot_managers]
    sim.cbs_planner = PartialCBS()
    sim._run_cbs_planning()

    for manager in sim.robot_managers:
        assert manager.state.robot_id in stale_ids
        if manager.state.robot_id == stale_ids[0]:
            assert manager.state.planned_path
        else:
            assert manager.state.planned_path == []
            assert manager.state.status == RobotStatus.REROUTING
