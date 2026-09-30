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
