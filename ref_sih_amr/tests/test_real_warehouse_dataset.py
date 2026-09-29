import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from data.warehouse_benchmark import REAL_SCENARIOS, WAREHOUSE_SAMPLES, load_movingai_map
from sim.simulator import Simulator


@pytest.mark.parametrize("scenario", ["S1_Normal", "S2_Crossing", "S3_Narrow"])
def test_real_warehouse_scenario_is_movingai_compatible(scenario):
    ascii_map = REAL_SCENARIOS[scenario]
    rows = ascii_map.splitlines()
    assert len(rows) == 63
    assert all(len(row) == 161 for row in rows)

    starts = [p for p, _ in WAREHOUSE_SAMPLES[scenario]]
    goals = [p for _, p in WAREHOUSE_SAMPLES[scenario]]
    for x, y in starts + goals:
        assert rows[y][x] in {".", "R", "D"}


def test_real_warehouse_simulator_uses_benchmark_pairs():
    scenario = "S1_Normal"
    sim = Simulator(
        ascii_map=REAL_SCENARIOS[scenario],
        headless=True,
        strategy="B0",
        benchmark_pairs=WAREHOUSE_SAMPLES[scenario],
    )

    assert len(sim.robot_managers) == 3
    assert len(sim.tasks) == 3
    actual_positions = sorted(tuple(m.state.position) for m in sim.robot_managers)
    expected_positions = sorted(
        (float(x), float(y)) for (x, y), _ in WAREHOUSE_SAMPLES[scenario]
    )
    assert actual_positions == expected_positions
    assert [(t.pickup_cell, t.dropoff_cell) for t in sim.tasks] == WAREHOUSE_SAMPLES[scenario]


def test_movingai_map_loader_rejects_invalid_dimensions(tmp_path):
    bad = tmp_path / "bad.map"
    bad.write_text("type octile\nheight 2\nwidth 3\nmap\n...\n..\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_movingai_map(bad)
