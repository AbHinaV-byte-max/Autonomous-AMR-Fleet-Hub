import sys, os, random
sys.path.insert(0, os.path.abspath('ref_sih_amr'))
from sim.simulator import Simulator
from experiments.runner import SCENARIOS

random.seed(42)
sim = Simulator(ascii_map=SCENARIOS['S1_Normal'], headless=True, strategy='P1')

ticks = 100
history = []
for t in range(ticks):
    sim.tick()
    snap = {
        'tick': t,
        'completed': sim.completed_tasks,
        'robots': {
            r.state.robot_id: {
                'pos': r.state.position,
                'status': str(r.state.status),
                'task': r.state.current_task_id
            } for r in sim.robot_managers
        },
        'tasks': [
            {
                'id': tsk.task_id,
                'pick': tsk.pickup_cell,
                'drop': tsk.dropoff_cell,
                'status': str(tsk.status),
                'assigned': tsk.assigned_robot_id
            } for tsk in sim.tasks
        ]
    }
    history.append(snap)

print(f"Recorded {len(history)} ticks. Total completed tasks at tick {ticks-1}: {sim.completed_tasks}")
for rid in sorted(history[-1]['robots'].keys()):
    r = history[-1]['robots'][rid]
    print(f"  {rid}: pos={r['pos']}, task={r['task']}, status={r['status']}")
