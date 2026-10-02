from experiments.decentralized_acceptance import fixed_workload
from experiments.runner import SCENARIOS
from sim.simulator import Simulator

def test_s6_debug_state_trace():
    sim = Simulator(ascii_map=SCENARIOS["S6_CommDelay"], headless=True, strategy="P2P", seed=26123)
    sim.benchmark_scenario = "S6_CommDelay"
    fixed_workload(sim, 6, seed=26123)
    delayed_messages = []

    def degraded_send(message):
        if message.robot_id == "robot-0" and sim.tick_count % 4 == 0:
            return
        release_tick = sim.tick_count + 2 + (sim.tick_count % 2)
        delayed_messages.append((release_tick, message))

    def degraded_clear():
        sim.comms.current_messages = [msg for release, msg in delayed_messages if release <= sim.tick_count]
        delayed_messages[:] = [(release, msg) for release, msg in delayed_messages if release > sim.tick_count]
        sim.comms.next_messages = []

    sim.comms.send = degraded_send
    sim.comms.clear = degraded_clear

    for _ in range(300):
        if sim.completed_tasks >= 6:
            break
        sim.tick()

    assert sim.completed_tasks == 6, repr({
        "tick": sim.tick_count,
        "tasks": [(t.task_id, t.status.value, t.assigned_robot_id, t.pickup_cell, t.dropoff_cell) for t in sim.tasks],
        "robots": [(m.state.robot_id, m.state.status.value, m.state.position, m.state.current_task_id, m.current_task.status.value if m.current_task else None, m.current_task.dropoff_cell if m.current_task else None, m.target_cell, len(m.state.planned_path), m.wait_time, m.waiting_on, {k: v.timestamp for k,v in m.peer_states.items()}) for m in sim.robot_managers],
        "events": sim.event_log.conflict_events[-60:],
    })
