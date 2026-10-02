from experiments.decentralized_acceptance import fixed_workload
from experiments.runner import SCENARIOS
from sim.simulator import Simulator

def test_s8_debug_state_trace():
    sim = Simulator(ascii_map=SCENARIOS["S8_Scale"], headless=True, strategy="P2P", seed=26123)
    sim.benchmark_scenario = "S8_Scale"
    fixed_workload(sim, 6, seed=26123)
    for _ in range(300):
        if sim.completed_tasks >= 6:
            break
        sim.tick()
    assert False, repr({
        "tick": sim.tick_count,
        "completed": sim.completed_tasks,
        "tasks": [(t.task_id, t.status.value, t.assigned_robot_id, t.pickup_cell, t.dropoff_cell) for t in sim.tasks],
        "robots": [(m.state.robot_id, m.state.status.value, m.state.position, m.state.current_task_id, m.current_task.status.value if m.current_task else None, m.target_cell, len(m.state.planned_path), m.wait_time, m.waiting_on) for m in sim.robot_managers],
        "events": sim.event_log.conflict_events[-80:],
    })
