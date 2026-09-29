"""MovingAI warehouse benchmark adapter.

The supplied .scen files are start/goal workloads, not simulator maps.
This module keeps that distinction explicit and converts MovingAI maps
(T=blocked, .=free) into the simulator's #/. representation.

The first three instances from each supplied workload are used for the live
3-AMR dashboard workload. The source files contain hundreds of instances;
the complete source files remain the benchmark provenance outside the runtime
path so we do not silently pretend the live simulator is executing all of them.
"""

from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parent
DATA_DIR = ROOT / "maps"

Point = Tuple[int, int]
Pair = Tuple[Point, Point]

WAREHOUSE_SAMPLES: Dict[str, List[Pair]] = {
    "S1_Normal": [
        ((69, 39), (139, 11)),
        ((57, 7), (147, 37)),
        ((120, 43), (58, 36)),
    ],
    "S2_HighTraffic": [
        ((153, 23), (59, 37)),
        ((124, 8), (51, 22)),
        ((153, 16), (31, 16)),
    ],
    "S3_LongRoutes": [
        ((13, 30), (38, 43)),
        ((13, 3), (13, 7)),
        ((157, 61), (12, 25)),
    ],
}

WAREHOUSE_METADATA = {
    "S1_Normal": {"map": "warehouse-10-20-10-2-1.map", "source": "warehouse-10-20-10-2-1-even-1.scen", "source_instances": 450},
    "S2_HighTraffic": {"map": "warehouse-10-20-10-2-1.map", "source": "warehouse-10-20-10-2-1-even-2.scen", "source_instances": 450},
    "S3_LongRoutes": {"map": "warehouse-10-20-10-2-1.map", "source": "warehouse-10-20-10-2-1-even-3.scen", "source_instances": 430},
}

def load_movingai_map(path: Path) -> str:
    lines = path.read_text(encoding="utf-8").splitlines()
    try:
        map_idx = lines.index("map")
    except ValueError as exc:
        raise ValueError(f"MovingAI map header missing in {path.name}") from exc
    rows = [line.strip() for line in lines[map_idx + 1:] if line.strip()]
    if not rows:
        raise ValueError(f"MovingAI map has no grid rows: {path.name}")
    width = int(next(line.split()[1] for line in lines if line.startswith("width ")))
    height = int(next(line.split()[1] for line in lines if line.startswith("height ")))
    if len(rows) != height or any(len(row) != width for row in rows):
        raise ValueError(f"MovingAI dimensions do not match grid: {path.name}")
    if any(ch not in {"T", "."} for row in rows for ch in row):
        raise ValueError(f"Unsupported MovingAI cell in {path.name}")
    return "\n".join(row.replace("T", "#") for row in rows)

def build_scenario_map(scenario: str) -> str:
    metadata = WAREHOUSE_METADATA[scenario]
    ascii_map = load_movingai_map(DATA_DIR / metadata["map"])
    grid = [list(row) for row in ascii_map.splitlines()]
    pairs = WAREHOUSE_SAMPLES[scenario]
    for start, goal in pairs:
        sx, sy = start
        gx, gy = goal
        if grid[sy][sx] != "." or grid[gy][gx] != ".":
            raise ValueError(f"{scenario}: benchmark coordinate is not traversable")
    for start, _ in pairs:
        grid[start[1]][start[0]] = "R"
    for _, goal in pairs:
        grid[goal[1]][goal[0]] = "D"
    return "\n".join("".join(row) for row in grid)

def build_real_scenarios() -> Dict[str, str]:
    return {name: build_scenario_map(name) for name in WAREHOUSE_SAMPLES}

REAL_SCENARIOS = build_real_scenarios()
