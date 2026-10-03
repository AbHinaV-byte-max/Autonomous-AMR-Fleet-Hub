"""
Tests for Feature 1: LiDAR scan emulation and sensor simulator.
Validates ray casting, obstacle detection, max range, and architecture freeze compliance.
"""
import dataclasses
import math
import pytest

from models import RobotState, RobotStatus
from config import load_map
from robot.sensors import SensorSimulator, LidarScanPayload
from sim.simulator import Simulator
from experiments.runner import SCENARIOS


def test_robot_state_schema_freeze():
    """Verify RobotState dataclass schema was untouched (Phase 0 architecture freeze)."""
    expected_fields = [
        "robot_id",
        "timestamp",
        "position",
        "heading",
        "velocity",
        "battery",
        "current_task_id",
        "task_priority",
        "planned_path",
        "reserved_cells",
        "status",
        "localization_confidence",
        "communication_quality",
    ]
    actual_fields = [f.name for f in dataclasses.fields(RobotState)]
    assert actual_fields == expected_fields, (
        f"RobotState dataclass fields were modified! Expected: {expected_fields}, got: {actual_fields}"
    )


def test_simulate_lidar_basic_and_obstacles():
    """Verify simulate_lidar casts rays and detects obstacles."""
    # 5x5 map with obstacle at (3, 2):
    # (0,0) .....
    # (0,1) .....
    # (0,2) ..R#.
    # (0,3) .....
    # (0,4) .....
    raw_map = (
        ".....\n"
        ".....\n"
        "..R#.\n"
        ".....\n"
        ".....\n"
    )
    costmap = load_map(raw_map)
    sim = SensorSimulator()

    # Robot at (2, 2), heading 0 degrees (+X axis toward obstacle at (3, 2))
    rays = sim.simulate_lidar(position=(2, 2), heading=0.0, costmap=costmap, num_rays=4, max_range=6)
    assert len(rays) == 4
    assert all(isinstance(r, float) for r in rays)

    # Ray 0 is along heading 0 (+X), distance to obstacle at (3, 2) is 1.0 cell
    assert rays[0] == 1.0

    # Ray 1 (90 deg = +Y, down to (2, 3), (2, 4)), hits map boundary at step 3: (2, 5) -> '#'
    assert rays[1] == 3.0

    # Ray 2 (180 deg = -X, left to (1, 2), (0, 2)), hits map boundary at step 3: (-1, 2) -> '#'
    assert rays[2] == 3.0

    # Ray 3 (270 deg = -Y, up to (2, 1), (2, 0)), hits map boundary at step 3: (2, -1) -> '#'
    assert rays[3] == 3.0


def test_simulate_lidar_max_range():
    """Verify rays terminate at max_range when no obstacles are encountered."""
    raw_map = (
        ".........\n"
        ".........\n"
        ".........\n"
        ".........\n"
        "....R....\n"
        ".........\n"
        ".........\n"
        ".........\n"
        ".........\n"
    )
    costmap = load_map(raw_map)
    sim = SensorSimulator()

    # Robot at center (4, 4), max_range = 3 (within open 9x9 grid)
    rays = sim.simulate_lidar(position=(4, 4), heading=0.0, costmap=costmap, num_rays=16, max_range=3)
    assert len(rays) == 16
    assert all(r == 3.0 for r in rays)


def test_simulator_telemetry_snapshot_lidar():
    """Verify Simulator includes lidar_scan in telemetry snapshot without altering RobotState."""
    sim = Simulator(ascii_map=SCENARIOS["S1_Normal"], headless=True)
    snapshot = sim._build_snapshot()

    assert "lidar_scan" in snapshot, "Root snapshot must contain 'lidar_scan' key"
    assert len(snapshot["lidar_scan"]) == 16, "Default LiDAR scan must contain 16 rays"

    # Verify per-robot lidar_scan
    assert "robots" in snapshot
    for rid, r in snapshot["robots"].items():
        assert "lidar_scan" in r
        assert len(r["lidar_scan"]) == 16
        assert all(isinstance(val, float) for val in r["lidar_scan"])

    # Tick the simulator and verify telemetry updates
    sim.tick()
    snapshot2 = sim._build_snapshot()
    assert "lidar_scan" in snapshot2
