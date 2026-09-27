"""
Deterministic Replay Test for AMR Fleet Coordination.
Executes an identical multi-AMR scenario twice under the same seed and asserts exact trace parity.
"""

import sys
import os
import json
import random

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import fleet_controller
from fleet_controller import AMR, SharedBlackboard, CommunicationNetwork, TaskManager, EventLogger


def run_single_simulation(seed=42):
    random.seed(seed)
    EventLogger.clear()
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    
    agents = [
        AMR("AMR_01", (40, 40), None, blackboard),
        AMR("AMR_02", (41, 39), None, blackboard),
        AMR("AMR_03", (44, 40), None, blackboard)
    ]
    task_mgr = TaskManager([(42, 40), (41, 41), (44, 43)])
    task_mgr.allocate_tasks(agents, 0, set())
    
    step_traces = []
    
    for step in range(5):
        current_time = step
        network.broadcast_state(agents, current_time)
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and not amr.failed and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)
            
        step_traces.append({
            "step": step,
            "positions": {a.name: a.pos for a in agents},
            "idle_states": {a.name: a.idle for a in agents}
        })

    events = EventLogger.get_recent_events(100)
    return {
        "step_traces": step_traces,
        "events": events
    }


def main():
    print("="*60)
    print("DETERMINISTIC REPLAY VERIFICATION")
    print("="*60)
    
    print("Executing Run 1 (Seed 42)...")
    run_1 = run_single_simulation(seed=42)
    
    print("Executing Run 2 (Seed 42)...")
    run_2 = run_single_simulation(seed=42)
    
    # Assert exact match of step traces
    assert run_1["step_traces"] == run_2["step_traces"], "Step traces must match bit-for-bit"
    print("✓ Step Traces: 100% Exact Match between Run 1 and Run 2")
    
    # Assert exact match of event logs
    events_1 = [e["event_type"] for e in run_1["events"]]
    events_2 = [e["event_type"] for e in run_2["events"]]
    assert events_1 == events_2, f"Event sequences must match: {events_1} vs {events_2}"
    print(f"✓ Event Log Parity: {len(events_1)} events matched perfectly")
    
    print("\n" + "="*60)
    print("DETERMINISTIC REPLAY: PASS (Exact Bit-Level Reproducibility Verified)")
    print("="*60)


if __name__ == "__main__":
    main()
