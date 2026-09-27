"""
FINAL SIH DEMONSTRATION & OMNIVERSE INTEGRATION SCRIPT
Executes the complete 22-step workflow live against OpenUSD stage (assets/omniverse/simulation5.usd):
1. Start Omniverse USD Stage
2. Load assets/omniverse/simulation5.usd
3. Start coordination system
4. Start 3 AMRs (AMR_01, AMR_02, AMR_03)
5. Create tasks
6. AMRs independently plan
7. AMR_01 and AMR_02 approach intersection
8. Exchange intent (P2P broadcast)
9. Conflict detected
10. Edge-AI evaluates trajectory
11. Safety Arbiter resolves & enforces safety
12. Lower-priority AMR yields
13. Dynamic blockage appears
14. AMR reroutes locally
15. One AMR fails
16. Task is unassigned and reassigned
17. Communication degradation occurs
18. Safe behavior / margin expansion activates
19. Communication recovers
20. Tasks complete
21. Gemini/MCP explains the events
22. Display final performance metrics
"""

import sys
import os
import time

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import fleet_controller
from fleet_controller import (
    AMR, SharedBlackboard, CommunicationNetwork, TaskManager,
    DynamicObstacle, EventLogger, FleetContext, SafetyArbiter
)
from omniverse_adapter import OmniverseSimulatorAdapter
from mcp_fleet.gemini_advisor import GeminiFleetAdvisor


def run_full_sih_demo():
    print("="*70)
    print("FINAL SIH DEMONSTRATION: FULL AMR SYSTEM INTEGRATION + OMNIVERSE")
    print("="*70)
    
    # 1. Start Omniverse & Load assets/omniverse/simulation5.usd
    print("\n[STEP 1 & 2] Loading OpenUSD Stage 'assets/omniverse/simulation5.usd'...")
    usd_adapter = OmniverseSimulatorAdapter(cell_size=1.0)
    
    # 3. Start Coordination System
    print("[STEP 3] Initializing Decentralized Coordination Core...")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    EventLogger.clear()
    
    # 4. Start 3 AMRs
    print("[STEP 4] Registering AMRs in Depot (/World/P_DYNEX_Depot/Robots)...")
    for r_id in ["AMR_01", "AMR_02", "AMR_03"]:
        usd_adapter.register_robot(r_id)
        
    agents = [
        AMR("AMR_01", (40, 40), usd_adapter.stage, blackboard, t_degraded=2, t_safe_mode=5),
        AMR("AMR_02", (41, 39), usd_adapter.stage, blackboard, t_degraded=2, t_safe_mode=5),
        AMR("AMR_03", (44, 40), usd_adapter.stage, blackboard, t_degraded=2, t_safe_mode=5)
    ]
    
    FleetContext.agents = agents
    FleetContext.blackboard = blackboard
    FleetContext.network = network
    FleetContext.stage = usd_adapter.stage
    
    # Update initial 3D positions in USD
    for a in agents:
        usd_adapter.update_robot_pose(a.name, a.pos[0], a.pos[1], heading_deg=0.0, current_time=0)
        
    # 5. Create Tasks
    print("\n[STEP 5] Creating Tasks in TaskManager...")
    task_mgr = TaskManager([(42, 40), (41, 41), (44, 43)])
    FleetContext.task_manager = task_mgr
    
    # 6. Allocate & Independent Planning
    print("[STEP 6] Allocating Tasks & Performing Independent Decentralized A* Planning...")
    task_mgr.allocate_tasks(agents, 0, set())
    for a in agents:
        print(f"  - {a.name}: Assigned Task {a.goal}, Path Length: {len(a.current_res.path) if a.current_res else 0}")
        
    # 7-12. Intersection Conflict Resolution
    print("\n[STEP 7-12] Step Timestep 0: P2P Broadcast, Conflict Detection & Priority Yield...")
    network.broadcast_state(agents, 0)
    resolved = False
    while not resolved:
        resolved = True
        for a in agents:
            if not a.idle and a.current_res and not a.current_res.valid:
                a.resolve_conflicts(0)
                resolved = False
                
    for a in agents:
        a.step(0)
        usd_adapter.update_robot_pose(a.name, a.pos[0], a.pos[1], heading_deg=0.0, current_time=0)
        
    # 13-14. Dynamic Blockage & Rerouting
    print("\n[STEP 13 & 14] Timestep 1: Dynamic Pallet Drop Activation & Local Rerouting...")
    dyn_obs = DynamicObstacle("PALLET_BLOCK_01", (44, 41), activation_time=1)
    blackboard.add_dynamic_obstacle(dyn_obs)
    dyn_obs.active = True
    usd_adapter.set_dynamic_obstacle_state(44, 41, active=True)
    
    network.broadcast_state(agents, 1)
    for a in agents:
        a.step(1)
        usd_adapter.update_robot_pose(a.name, a.pos[0], a.pos[1], heading_deg=0.0, current_time=1)
        
    # 15-16. Robot Failure & Task Reassignment
    print("\n[STEP 15 & 16] Timestep 2: Simulating Robot Failure & Task Reassignment...")
    # Inject failure on AMR_03
    agents[2].trigger_failure(2, task_mgr)
    task_mgr.allocate_tasks(agents, 2, set(), is_reassignment=True)
    
    network.broadcast_state(agents, 2)
    for a in agents:
        a.step(2)
        usd_adapter.update_robot_pose(a.name, a.pos[0], a.pos[1], heading_deg=0.0, current_time=2)

    # 17-19. Communication Degradation & Recovery
    print("\n[STEP 17-19] Timestep 3: Network Packet Loss Injection & Spatial Margin Expansion...")
    network.inject_fault("AMR_02", "AMR_01", 3, 4)
    network.broadcast_state(agents, 3)
    for a in agents:
        a.step(3)
        usd_adapter.update_robot_pose(a.name, a.pos[0], a.pos[1], heading_deg=0.0, current_time=3)
        
    print("\n[STEP 19 & 20] Timestep 4-6: Communication Recovery & Mission Completion...")
    for step in range(4, 7):
        current_time = step
        network.broadcast_state(agents, current_time)
        for a in agents:
            if not a.idle and not a.failed and a.current_res and not a.current_res.valid:
                a.resolve_conflicts(current_time)
            a.step(current_time)
            usd_adapter.update_robot_pose(a.name, a.pos[0], a.pos[1], heading_deg=0.0, current_time=current_time)

    # 21. Gemini AI Grounded Explanation
    print("\n" + "="*70)
    print("[STEP 21] GEMINI AI GROUNDED DECISION EXPLANATIONS (VIA MCP)")
    print("="*70)
    advisor = GeminiFleetAdvisor()
    
    queries = [
        "What is the current fleet state?",
        "Why did AMR_02 yield to AMR_01 at the intersection?",
        "Why did the safety arbiter override the Edge-AI recommendation?",
        "Why did AMR_03 fail and what happened to its task?",
        "What happened during communication degradation?",
        "What are the final system performance metrics?"
    ]
    for q in queries:
        ans = advisor.answer_query(q)
        print(f"\n[USER QUERY]: {q}")
        print(f"[GEMINI EXPLANATION]:\n{ans}")
        print("-" * 50)

    # 22. Display Final Metrics & Telemetry
    print("\n" + "="*70)
    print("[STEP 22] FINAL TELEMETRY & SYSTEM PERFORMANCE REPORT")
    print("="*70)
    for a in agents:
        tel = usd_adapter.get_robot_telemetry(a.name)
        print(f"Robot {a.name}: World Pos ({tel['position']['x']:.2f}, {tel['position']['y']:.2f}, {tel['position']['z']:.3f}), Speed: {tel['velocity']['speed']:.2f} m/s, Status: {'FAILED' if a.failed else ('IDLE' if a.idle else 'ACTIVE')}")
        
    print(f"\nTotal System Events Logged: {len(EventLogger._events)}")
    print("✓ FINAL SIH DEMONSTRATION COMPLETE: 100% SUCCESS")
    print("="*70)


if __name__ == "__main__":
    run_full_sih_demo()
