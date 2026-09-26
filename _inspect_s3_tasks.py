import sys, os, random
sys.path.insert(0, os.path.abspath('ref_sih_amr'))
from sim.simulator import Simulator
from experiments.runner import SCENARIOS

random.seed(42)
sim = Simulator(ascii_map=SCENARIOS['S3_Narrow'], headless=True, strategy='P1')

ticks = 100
tasks_log = {}

for t in range(ticks):
    sim.tick()
    for task in sim.tasks:
        tid = task.task_id
        if tid not in tasks_log:
            tasks_log[tid] = {
                'id': tid,
                'pick': task.pickup_cell,
                'drop': task.dropoff_cell,
                'assigned': task.assigned_robot_id,
                'pick_tick': None,
                'drop_tick': None
            }
        t_info = tasks_log[tid]
        if task.assigned_robot_id and not t_info['assigned']:
            t_info['assigned'] = task.assigned_robot_id
        st = str(task.status)
        if ('IN_PROGRESS' in st or task.status == 3) and t_info['pick_tick'] is None:
            t_info['pick_tick'] = t
        if ('COMPLETED' in st or task.status == 4) and t_info['drop_tick'] is None:
            t_info['drop_tick'] = t

print(f"S3 Simulation completed: {sim.completed_tasks} completed tasks across {len(sim.robot_managers)} robots.")
picked = [v for v in tasks_log.values() if v['pick_tick'] is not None]
print(f"Picked up tasks in S3: {len(picked)}")
for p in picked[:6]:
    print(f"  Task {p['id']}: pick={p['pick']} (t={p['pick_tick']}) -> drop={p['drop']} (t={p['drop_tick']}) by {p['assigned']}")
