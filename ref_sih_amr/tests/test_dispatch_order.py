import sys
import os

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




# S2-S6 dashboard dispatch contracts. S1 remains covered by the original tests above.
SCENARIO_DISPATCH_POINTS = {
    "S2_Crossing": {
        "pickup": {"PK-1 (West Aisle)": (2, 5), "PK-2 (Center Aisle)": (10, 5), "PK-3 (East Aisle)": (18, 5)},
        "dropoff": {"DD-1 (Top Bay)": (5, 1), "DD-2 (East Bay)": (22, 6)},
    },
    "S3_Narrow": {
        "pickup": {"PK-1 (West Passage)": (6, 5), "PK-2 (East Passage)": (16, 5), "PK-3 (Lower Passage)": (4, 7)},
        "dropoff": {"DD-1 (Left Bay)": (3, 9), "DD-2 (Right Bay)": (19, 9)},
    },
    "S4_Blocked": {
        "pickup": {"PK-1 (West Aisle)": (4, 6), "PK-2 (Center Aisle)": (11, 6), "PK-3 (East Aisle)": (18, 6)},
        "dropoff": {"DD-1 (Left Bay)": (3, 9), "DD-2 (Right Bay)": (19, 9)},
    },
    "S5_Failure": {
        "pickup": {"PK-1 (West Aisle)": (5, 4), "PK-2 (Center Aisle)": (12, 4), "PK-3 (East Aisle)": (18, 4)},
        "dropoff": {"DD-1 (Left Bay)": (3, 10), "DD-2 (Right Bay)": (19, 10)},
    },
    "S6_CommDelay": {
        "pickup": {"PK-1 (West Aisle)": (5, 4), "PK-2 (Center Aisle)": (15, 4), "PK-3 (East Aisle)": (20, 4)},
        "dropoff": {"DD-1 (Left Bay)": (3, 10), "DD-2 (Right Bay)": (20, 10)},
    },
}


def test_s2_s6_dispatch_points_are_real_map_cells_and_distinct():
    for scenario, points in SCENARIO_DISPATCH_POINTS.items():
        grid = SCENARIOS[scenario].splitlines()
        pickups = list(points["pickup"].items())
        dropoffs = list(points["dropoff"].items())

        assert len({cell for _, cell in pickups}) == len(pickups)
        assert len({cell for _, cell in dropoffs}) == len(dropoffs)

        for label, (x, y) in pickups:
            assert grid[y][x] == ".", f"{scenario}: {label} must be a traversable pickup cell"
        for label, (x, y) in dropoffs:
            assert grid[y][x] == "D", f"{scenario}: {label} must point to a real delivery dock"


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
