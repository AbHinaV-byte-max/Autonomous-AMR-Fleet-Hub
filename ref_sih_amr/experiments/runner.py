import sys, os
import multiprocessing
import time
import json
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sim.simulator import Simulator
from config import GridMap

SCENARIOS = {
    # S1: Baseline large distribution center (Universal live dashboard map)
    "S1_Normal": """\
##############################
#R.##......R........R......###
#..##......................###
#..####..####....####..####..#
#..####..####....####..####..#
#R..........................R#
#..####..####....####..####..#
#..####..####....####..####..#
#R..........................R#
#..####..####....####..####..#
#..####..####....####..####..#
#R..........................R#
#..####..####....####..####..#
#..####..####....####..####..#
#............................#
#..####..####....####..####..#
#..####..####....####..####..#
#..DD......................DD#
#..DD......................DD#
##############################
""",

    # S2: Cross-docking terminal with orthogonal intersecting avenues (4-way crossing stress)
    "S2_Crossing": """\
########################
#....DD...R............#
#..######......######..#
#..######......######..#
#..######......######..#
#......................#
#R....................D#
#D....................R#
#......................#
#..######......######..#
#..######......######..#
#..######......######..#
#............R..DD.....#
########################
""",

    # S3: VNA (Very Narrow Aisle) high-density warehouse (1-cell wide corridor contention)
    "S3_Narrow": """\
########################
#R....................R#
#.####################.#
#R....................R#
#.##########..########.#
#......................#
#.##########..########.#
#...........##.........#
#.####################.#
#..DD..............DD..#
#......................#
########################
""",

    # S4: High-throughput warehouse with dynamic blockage at (5, 2) forcing bypass routing
    "S4_Blocked": """\
########################
#R....................R#
#####.#############.####
#R....................R#
#..####..######..####..#
#..####..######..####..#
#......................#
#..####..######..####..#
#..####..######..####..#
#..DD..............DD..#
#..DD..............DD..#
########################
""",

    # S5: Multi-bay facility with robot breakdown and automated task recovery
    "S5_Failure": """\
########################
#R...####..R...####...R#
#....####......####....#
#....####......####....#
#......................#
#....####..##..####....#
#....####..##..####....#
#......................#
#....####..##..####....#
#....####..##..####....#
#..DD..............DD..#
#..DD......R.......DD..#
#......................#
########################
""",

    # S6: Multi-zone facility with long transit spans and degraded comms
    "S6_CommDelay": """\
########################
#R...####......####...R#
#....####......####....#
#....####......####....#
#......................#
#..########..########..#
#..########..########..#
#......................#
#....####......####....#
#....####......####....#
#R.DD####......####DD.R#
#..DD..............DD..#
#......................#
########################
""",

    # S7: Boundary & obstacle edge-case layout
    "S7_Malformed": """\
################
#R..#..##...#.R#
#..##########..#
#..####..####..#
#..............#
#..............#
#..####..####..#
#..####DD####..#
#D.....DD...R.D#
################
""",

    # S8: High-density fleet scalability benchmark (8 AMRs)
    "S8_Scale": """\
##########################
#R.##....R.....R....##..R#
#..##...............##...#
#..####..#####..####..#..#
#..####..#####..####..#..#
#R......................R#
#..####..#####..####..#..#
#..####..#####..####..#..#
#R......................R#
#..####..#####..####..#..#
#..####..#####..####..#..#
#..DD..............DD....#
#..DD..............DD....#
##########################
"""
}

STRATEGIES = ["B0", "B1", "B2", "P1"]
TRIALS = 20
MAX_TICKS = 1000

def run_trial(args):
    scenario_name, strategy, trial_idx = args
    ascii_map = SCENARIOS[scenario_name]
    
    sim = Simulator(ascii_map=ascii_map, headless=True, strategy=strategy)
    
    # Specific scenario injections
    if scenario_name == "S4_Blocked":
        sim.run(max_ticks=100)
        sim.block_cell(5, 2)
        sim.run(max_ticks=MAX_TICKS - 100)
    elif scenario_name == "S5_Failure":
        sim.run(max_ticks=80)
        sim.kill_robot("robot-0")
        sim.run(max_ticks=MAX_TICKS - 80)
    elif scenario_name == "S6_CommDelay":
        original_send = sim.comms.send
        def patched_send(msg):
            if msg.robot_id != "robot-0":
                original_send(msg)
        sim.comms.send = patched_send
        sim.run(max_ticks=MAX_TICKS)
    else:
        sim.run(max_ticks=MAX_TICKS)
        
    metrics = sim.metric_values.copy()
    metrics["strategy"] = strategy
    metrics["scenario"] = scenario_name
    metrics["trial"] = trial_idx
    metrics["completed_tasks"] = sim.completed_tasks
    
    return metrics

def main():
    from dashboard.backend.db import init_db, persist_snapshot, close_db
    
    def save_results(results):
        init_db()
        for res in results:
            persist_snapshot({
                "tick": MAX_TICKS,
                "metrics": res,
                "strategy": res["strategy"],
                "scenario": res["scenario"],
                "trial": res["trial"]
            })
        close_db()

    tasks = []
    for scenario in SCENARIOS:
        for strategy in STRATEGIES:
            for trial in range(TRIALS):
                tasks.append((scenario, strategy, trial))
                
    print(f"Starting {len(tasks)} trials ({TRIALS} per config)...")
    t0 = time.time()
    
    with multiprocessing.Pool() as pool:
        results = pool.map(run_trial, tasks)
        
    print(f"Finished in {time.time() - t0:.1f}s. Saving to DB...")
    save_results(results)
    print("Done.")

if __name__ == "__main__":
    main()
