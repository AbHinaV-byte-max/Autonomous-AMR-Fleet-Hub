import sys, os, random
sys.path.insert(0, os.path.abspath('ref_sih_amr'))
from sim.simulator import Simulator
from experiments.runner import SCENARIOS

random.seed(42)
sim = Simulator(ascii_map=SCENARIOS['S5_Failure'], headless=True, strategy='P1')

print(f"Spawned {len(sim.robot_managers)} robots in S5_Failure:")
for r in sim.robot_managers:
    print(f"  {r.state.robot_id}: spawn pos={r.state.position}")

print("\nRunning 100 ticks with robot failure at tick 50...")
for t in range(100):
    if t == 50:
        print(f"--- [DYNAMIC EVENT] Killing robot-0 at tick {t} ---")
        sim.kill_robot("robot-0")
    sim.tick()

print(f"\nCompleted tasks: {sim.completed_tasks}")
for r in sim.robot_managers:
    print(f"  {r.state.robot_id}: pos={r.state.position}, task={r.state.current_task_id}, status={r.state.status}")
