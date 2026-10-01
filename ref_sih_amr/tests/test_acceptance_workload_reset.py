from experiments.decentralized_acceptance import fixed_workload
from experiments.runner import SCENARIOS
from sim.simulator import Simulator


def test_acceptance_workload_removes_seeded_manager_state():
    sim = Simulator(
        ascii_map=SCENARIOS["S2_Crossing"],
        headless=True,
        strategy="P2P",
    )

    assert any(manager.current_task is not None for manager in sim.robot_managers)

    fixed_workload(sim, count=1)

    assert len(sim.tasks) == 1
    assert sim.tasks[0].task_id == "BENCH_01"
    assert all(
        manager.current_task is None
        or manager.current_task.task_id == "BENCH_01"
        for manager in sim.robot_managers
    )
    assert all(
        manager.state.current_task_id in (None, "BENCH_01")
        for manager in sim.robot_managers
    )
