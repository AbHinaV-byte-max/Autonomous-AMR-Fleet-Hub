import sys, os, random
sys.path.insert(0, os.path.abspath('ref_sih_amr'))
from sim.simulator import Simulator
from experiments.runner import SCENARIOS

random.seed(42)
sim = Simulator(ascii_map=SCENARIOS['S4_Blocked'], headless=True, strategy='P1')

print(f"Spawned {len(sim.robot_managers)} robots in S4_Blocked:")
for r in sim.robot_managers:
    print(f"  {r.state.robot_id}: spawn pos={r.state.position}")

print("\nRunning 100 ticks with dynamic blockage at tick 40...")
for t in range(100):
    if t == 40:
        print(f"--- [DYNAMIC EVENT] Blocking cell (5, 2) at tick {t} ---")
        sim.block_cell(5, 2)
    sim.tick()

print(f"\nCompleted tasks: {sim.completed_tasks}")
for r in sim.robot_managers:
    print(f"  {r.state.robot_id}: pos={r.state.position}, task={r.state.current_task_id}, status={r.state.status}")
