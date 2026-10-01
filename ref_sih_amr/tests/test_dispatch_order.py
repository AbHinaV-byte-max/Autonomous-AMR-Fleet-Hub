import sys
import os

from experiments.runner import SCENARIOS
from config import load_map
from robot.planner import AStarPlanner


# Contract mirrored by the dashboard manual-dispatch UI for S1-S6.
# Keep these coordinates explicit: they are named operator stations/bays,
# not heuristic cells inferred from the map.
DISPATCH_POINTS = {
    "S1_Normal": {
        "pickup": {
            "PK-1 (Aisle 1)": (2, 5),
            "PK-2 (Aisle 2)": (14, 5),
            "PK-3 (Aisle 3)": (27, 5),
        },
        "dropoff": {
            "DD-1 (Left Bay)": (3, 17),
            "DD-2 (Right Bay)": (27, 17),
        },
    },
    "S2_Crossing": {
        "pickup": {
            "PK-1 (West Aisle)": (2, 5),
            "PK-2 (Center Aisle)": (10, 5),
            "PK-3 (East Aisle)": (18, 5),
        },
        "dropoff": {
            "DD-1 (Top Bay)": (5, 1),
            "DD-2 (East Bay)": (21, 6),
        },
    },
    "S3_Narrow": {
        "pickup": {
            "PK-1 (West Passage)": (6, 5),
            "PK-2 (East Passage)": (16, 5),
            "PK-3 (Lower Passage)": (4, 7),
        },
        "dropoff": {
            "DD-1 (Left Bay)": (2, 9),
            "DD-2 (Right Bay)": (18, 9),
        },
    },
    "S4_Blocked": {
        "pickup": {
            "PK-1 (West Aisle)": (4, 6),
            "PK-2 (Center Aisle)": (11, 6),
            "PK-3 (East Aisle)": (18, 6),
        },
        "dropoff": {
            "DD-1 (Left Bay)": (2, 9),
            "DD-2 (Right Bay)": (17, 9),
        },
    },
    "S5_Failure": {
        "pickup": {
            "PK-1 (West Aisle)": (5, 4),
            "PK-2 (Center Aisle)": (12, 4),
            "PK-3 (East Aisle)": (18, 4),
        },
        "dropoff": {
            "DD-1 (Left Bay)": (2, 10),
            "DD-2 (Right Bay)": (18, 10),
        },
    },
    "S6_CommDelay": {
        "pickup": {
            "PK-1 (West Aisle)": (5, 4),
            "PK-2 (Center Aisle)": (15, 4),
            "PK-3 (East Aisle)": (20, 4),
        },
        "dropoff": {
            "DD-1 (Left Bay)": (2, 10),
            "DD-2 (Right Bay)": (20, 10),
        },
    },
}


def test_dispatch_points_cover_dashboard_scenarios():
    assert set(DISPATCH_POINTS) == {
        "S1_Normal",
        "S2_Crossing",
        "S3_Narrow",
        "S4_Blocked",
        "S5_Failure",
        "S6_CommDelay",
    }


def test_dispatch_points_are_real_map_cells_and_distinct():
    for scenario, points in DISPATCH_POINTS.items():
        grid = SCENARIOS[scenario].splitlines()
        pickups = points["pickup"]
        dropoffs = points["dropoff"]

        assert len(set(pickups.values())) == len(pickups), scenario
        assert len(set(dropoffs.values())) == len(dropoffs), scenario
        assert not (set(pickups.values()) & set(dropoffs.values())), scenario

        for label, (x, y) in pickups.items():
            assert grid[y][x] == ".", f"{scenario}: {label} must be a raw traversable pickup cell"

        for label, (x, y) in dropoffs.items():
            assert grid[y][x] == "D", f"{scenario}: {label} must point to a real delivery dock"


def test_every_dispatch_pair_has_an_astar_route():
    planner = AStarPlanner()

    for scenario, points in DISPATCH_POINTS.items():
        costmap = load_map(SCENARIOS[scenario])
        for pickup_label, pickup in points["pickup"].items():
            for dropoff_label, dropoff in points["dropoff"].items():
                path = planner.plan(pickup, dropoff, costmap)
                assert path, (
                    f"{scenario}: no A* route from {pickup_label} {pickup} "
                    f"to {dropoff_label} {dropoff}"
                )


def test_p1_initial_allocation_injects_executable_paths():
    from sim.simulator import Simulator

    sim = Simulator(SCENARIOS["S1_Normal"], headless=True, strategy="P1")
    active = [m for m in sim.robot_managers if m.current_task is not None]
    assert active, "P1 should assign initial work"
    assert any(m.state.planned_path for m in active), "P1 initial assignments need executable CBS/A* paths"


def test_auto_tick_keeps_moving_after_task_generation():
    from sim.simulator import Simulator

    sim = Simulator(SCENARIOS["S1_Normal"], headless=True, strategy="P1")
    sim.task_generator.enabled = True
    before = {m.state.robot_id: tuple(m.state.position) for m in sim.robot_managers}
    for _ in range(3):
        sim.tick()
    after = {m.state.robot_id: tuple(m.state.position) for m in sim.robot_managers}
    assert any(before[rid] != after[rid] for rid in before), f"No AMR moved: before={before}, after={after}"
