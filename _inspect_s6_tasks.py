import sys, os, random
sys.path.insert(0, os.path.abspath('ref_sih_amr'))
from sim.simulator import Simulator
from experiments.runner import SCENARIOS

random.seed(42)
sim = Simulator(ascii_map=SCENARIOS['S6_CommDelay'], headless=True, strategy='P1')

# S6 CommDelay: patch comms so robot-0 drops broadcasts
original_send = sim.comms.send
def patched_send(msg):
    if msg.robot_id != "robot-0":
        original_send(msg)
sim.comms.send = patched_send

print(f"Spawned {len(sim.robot_managers)} robots in S6_CommDelay:")
for r in sim.robot_managers:
    print(f"  {r.state.robot_id}: spawn pos={r.state.position}")

print("\nRunning 100 ticks with comm delay on robot-0...")
for t in range(100):
    sim.tick()

print(f"\nCompleted tasks: {sim.completed_tasks}")
for r in sim.robot_managers:
    print(f"  {r.state.robot_id}: pos={r.state.position}, task={r.state.current_task_id}, status={r.state.status}")
