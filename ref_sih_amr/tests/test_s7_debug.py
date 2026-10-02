from experiments.decentralized_acceptance import fixed_workload
from experiments.runner import SCENARIOS
from sim.simulator import Simulator

def test_s7_debug_state_trace():
    sim = Simulator(ascii_map=SCENARIOS["S7_Malformed"], headless=True, strategy="P2P", seed=26123)
    sim.benchmark_scenario = "S7_Malformed"
    fixed_workload(sim, 6, seed=26123)
    for _ in range(300):
        if sim.tick_count == 20:
            sim.comms.next_messages.append({"malformed": True})
        if sim.completed_tasks >= 6:
            break
        sim.tick()

    print("S7_DEBUG tick", sim.tick_count, "completed", sim.completed_tasks)
    for task in sim.tasks:
        print("S7_TASK", task.task_id, task.status.value, task.assigned_robot_id, task.pickup_cell, task.dropoff_cell)
    for manager in sim.robot_managers:
        print(
            "S7_ROBOT",
            manager.state.robot_id,
            manager.state.status.value,
            manager.state.position,
            manager.state.current_task_id,
            manager.current_task.status.value if manager.current_task else None,
            manager.current_task.dropoff_cell if manager.current_task else None,
            manager.target_cell,
            "path_len", len(manager.state.planned_path),
            "wait", manager.wait_time,
            "waiting_on", manager.waiting_on,
        )
    assert sim.completed_tasks == 6, repr({
        "tasks": [(t.task_id, t.status.value, t.assigned_robot_id, t.pickup_cell, t.dropoff_cell) for t in sim.tasks],
        "robots": [(m.state.robot_id, m.state.status.value, m.state.position, m.state.current_task_id, m.current_task.status.value if m.current_task else None, m.current_task.dropoff_cell if m.current_task else None, m.target_cell, len(m.state.planned_path), m.wait_time, m.waiting_on) for m in sim.robot_managers],
        "events": sim.event_log.conflict_events[-50:],
    })
