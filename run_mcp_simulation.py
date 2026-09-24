"""
Interactive MCP-Driven AMR Fleet Simulation Runner
Controls and inspects the multi-AMR warehouse simulation entirely through MCP tools
while updating OpenUSD (simulation5.usd) and streaming telemetry to Gemini AI Advisor.
"""

import os
import sys
import time
import json

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from mcp_fleet.client import FleetMCPClient
from mcp_fleet.gemini_advisor import GeminiFleetAdvisor
import fleet_controller
from fleet_controller import (
    AMR, SharedBlackboard, CommunicationNetwork, TaskManager,
    DynamicObstacle, EventLogger, FleetContext
)
from omniverse_adapter import OmniverseSimulatorAdapter


def run_mcp_simulation(max_steps=10, step_delay=0.5):
    print("=" * 65)
    print("STARTING AMR FLEET SIMULATION & PROGRAM VIA MCP")
    print("=" * 65)

    # 1. Initialize MCP Client
    client = FleetMCPClient(mode="in_process")
    advisor = GeminiFleetAdvisor()
    
    # 2. List available MCP tools
    tools = client.list_tools()
    print(f"\n[MCP] Discovered {len(tools)} available MCP Tools:")
    for t in tools:
        print(f"  • {t['name']}: {t['description']}")

    # 3. Start Simulation via MCP
    print("\n[MCP CALL] Invoking 'start_simulation'...")
    res = client.call_tool("start_simulation")
    print(f"[MCP RESPONSE] {res}")

    # 4. Setup Simulation World & Omniverse Adapter
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    EventLogger.clear()

    agents = [
        AMR("AMR_01", (8, 10), None, blackboard),
        AMR("AMR_02", (10, 10), None, blackboard),
        AMR("AMR_03", (10, 6), None, blackboard)
    ]
    
    tasks = [(14, 10), (10, 14), (6, 6)]
    task_mgr = TaskManager(tasks)
    
    # Initialize Omniverse USD adapter for live sync
    adapter = OmniverseSimulatorAdapter(cell_size=1.0)
    for agent in agents:
        adapter.register_robot(agent.name)
        adapter.update_robot_pose(agent.name, agent.pos[0], agent.pos[1], current_time=0)

    # Allocate initial tasks
    task_mgr.allocate_tasks(agents, 0, set())

    # Dynamic obstacle at t=2
    dyn_obs = DynamicObstacle("PALLET_BLOCK_01", (12, 10), activation_time=2)
    blackboard.add_dynamic_obstacle(dyn_obs)

    # Update FleetContext for MCP tools
    FleetContext.agents = agents
    FleetContext.blackboard = blackboard
    FleetContext.network = network
    FleetContext.task_manager = task_mgr
    FleetContext.current_time = 0
    FleetContext.simulation_status = "RUNNING"

    print("\n" + "=" * 65)
    print("RUNNING INTERACTIVE SIMULATION TIMESTEPS")
    print("=" * 65)

    for t in range(max_steps):
        print(f"\n>>> [TIMESTEP {t}] <<<")

        # Check dynamic obstacle activation
        for obs in blackboard.dynamic_obstacles:
            if t >= obs.activation_time:
                obs.active = True
                adapter.set_dynamic_obstacle_state(obs.pos[0], obs.pos[1], active=True)

        # Broadcast state across network
        network.broadcast_state(agents, t)

        # Execute decentralized step for each AMR
        for agent in agents:
            agent.step(t)
            adapter.update_robot_pose(agent.name, agent.pos[0], agent.pos[1], current_time=t)

        # Check completed tasks and reallocate
        completed = [a.name for a in agents if a.idle and a.goal is None]
        task_mgr.allocate_tasks(agents, t + 1, set())

        # Update FleetContext
        FleetContext.current_time = t + 1

        # 5. Query Fleet State via MCP Tool
        fleet_state = client.call_tool("get_fleet_state")
        print(f"\n[MCP INSPECT: get_fleet_state]")
        for r in fleet_state["robots"]:
            state_str = "IDLE" if r["idle"] else "MOVING"
            if r["safe_mode"]: state_str = "SAFE_MODE"
            if r["failed"]: state_str = "FAILED"
            print(f"  Robot {r['robot_id']}: Pos=({r['position']['x']}, {r['position']['y']}), State={state_str}, Battery={r['battery_level']:.1f}%")

        # 6. Query Active Space-Time Reservations via MCP Tool
        reservations = client.call_tool("get_active_reservations")
        print(f"[MCP INSPECT: get_active_reservations] Count: {len(reservations.get('reservations', []))}")

        # 7. Query Metrics via MCP Tool
        metrics = client.call_tool("get_metrics")
        print(f"[MCP INSPECT: get_metrics] Tasks Completed: {metrics['task_completion_count']}, Dropped Packets: {metrics['messages_dropped']}, AI Agreement: {metrics['ai_agreements']}/{metrics['ai_decisions']}")

        # 8. Sample Gemini AI Explanation for active robot
        if t in [2, 5]:
            print(f"\n[GEMINI AI ADVISORY EXPLANATION @ t={t}]")
            explanation = advisor.answer_query("Why is AMR_01 waiting or moving?")
            print(f"  {explanation}")

        time.sleep(step_delay)

    # 9. Stop Simulation via MCP
    print("\n" + "=" * 65)
    print("[MCP CALL] Invoking 'stop_simulation'...")
    stop_res = client.call_tool("stop_simulation")
    print(f"[MCP RESPONSE] {stop_res}")

    # Final summary metrics via MCP
    final_metrics = client.call_tool("get_metrics")
    print("\nFINAL FLEET METRICS (via MCP):")
    print(json.dumps(final_metrics, indent=2))
    print("\n✓ Simulation and program execution via MCP completed successfully!")


if __name__ == "__main__":
    run_mcp_simulation(max_steps=8, step_delay=0.1)
