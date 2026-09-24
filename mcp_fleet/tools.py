"""
MCP Tool Implementations for AMR Fleet Inspection, Advisory, and Lifecycle Controls.
"""

import sys
import os
import json
import time

# Ensure repo root is on sys.path
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import fleet_controller


def get_fleet_state():
    ctx = fleet_controller.FleetContext
    robots = []
    for agent in ctx.agents:
        robots.append({
            "robot_id": agent.name,
            "position": {"x": agent.pos[0], "y": agent.pos[1]},
            "goal": {"x": agent.goal[0], "y": agent.goal[1]} if agent.goal else None,
            "idle": agent.idle,
            "failed": agent.failed,
            "safe_mode": agent.is_in_safe_mode,
            "battery_level": getattr(agent, "battery", 100.0)
        })

    tasks = []
    if ctx.task_manager:
        for t in ctx.task_manager.tasks:
            tasks.append({"task_id": str(t), "goal": t, "status": "PENDING"})

    comm_status = {}
    if ctx.network:
        for a in ctx.agents:
            comm_status[a.name] = {peer: state["status"] for peer, state in a.peer_states.items()}

    active_conflicts = []
    active_reservations = []
    if ctx.blackboard:
        for r_id, res in ctx.blackboard.active_reservations_by_robot.items():
            active_reservations.append({
                "robot_id": r_id,
                "path_length": len(res.path),
                "path": res.path,
                "valid": res.valid,
                "emergency": res.emergency
            })
            if res.valid:
                c_list = ctx.blackboard.get_conflicts(res, ctx.blackboard.robot_positions.get(r_id, (0, 0)))
                for c in c_list:
                    active_conflicts.append({
                        "robot_a": r_id,
                        "robot_b": c.robot_id,
                        "status": "DETECTED"
                    })

    return {
        "robots": robots,
        "tasks": tasks,
        "simulation_time": ctx.current_time,
        "communication_status": comm_status,
        "active_conflicts": active_conflicts,
        "active_reservations": active_reservations
    }


def get_robot_state(robot_id):
    ctx = fleet_controller.FleetContext
    agent = next((a for a in ctx.agents if a.name == robot_id), None)
    if not agent:
        return {"error": f"Robot '{robot_id}' not found in fleet context"}

    state_str = "IDLE" if agent.idle else ("FAILED" if agent.failed else ("SAFE_MODE" if agent.is_in_safe_mode else "MOVING"))
    current_path = agent.current_res.path if agent.current_res else []
    
    return {
        "robot_id": agent.name,
        "position": {"x": agent.pos[0], "y": agent.pos[1]},
        "velocity": {"vx": 1.0 if not agent.idle else 0.0, "vy": 0.0},
        "heading": 0.0,
        "battery_level": getattr(agent, "battery", 100.0),
        "current_task": str(agent.goal) if agent.goal else None,
        "current_goal": {"x": agent.goal[0], "y": agent.goal[1]} if agent.goal else None,
        "state": state_str,
        "current_path": current_path,
        "communication_status": "SAFE_MODE" if agent.is_in_safe_mode else ("DEGRADED" if any(s["status"] == "DEGRADED" for s in agent.peer_states.values()) else "CONNECTED"),
        "peer_states": agent.peer_states
    }


def get_task_state(task_id):
    ctx = fleet_controller.FleetContext
    # Check assigned robot goals
    for agent in ctx.agents:
        if agent.goal and (str(agent.goal) == str(task_id) or f"task_{agent.goal}" == str(task_id)):
            return {
                "task_id": str(task_id),
                "status": "IN_PROGRESS",
                "assigned_robot": agent.name,
                "pickup_location": {"x": agent.pos[0], "y": agent.pos[1]},
                "drop_location": {"x": agent.goal[0], "y": agent.goal[1]},
                "priority": 1 if agent.emergency else 0,
                "deadline": None
            }

    # Check pending tasks in queue
    if ctx.task_manager:
        for t in ctx.task_manager.tasks:
            if str(t) == str(task_id):
                return {
                    "task_id": str(task_id),
                    "status": "PENDING",
                    "assigned_robot": None,
                    "pickup_location": None,
                    "drop_location": {"x": t[0], "y": t[1]},
                    "priority": 0,
                    "deadline": None
                }

    return {"task_id": str(task_id), "status": "UNKNOWN_OR_COMPLETED"}


def get_active_reservations():
    ctx = fleet_controller.FleetContext
    reservations = []
    if ctx.blackboard:
        for r_id, res in ctx.blackboard.active_reservations_by_robot.items():
            if res.path:
                start_t = res.path[0][2]
                end_t = res.path[-1][2]
                reservations.append({
                    "robot_id": r_id,
                    "location": [{"x": s[0], "y": s[1], "t": s[2]} for s in res.path],
                    "start_time": start_t,
                    "end_time": end_t,
                    "status": "VALID" if res.valid else "INVALID_OR_WAIT"
                })
    return {"reservations": reservations}


def get_conflicts():
    ctx = fleet_controller.FleetContext
    conflicts = []
    if ctx.blackboard:
        seen = set()
        for r_id, res in ctx.blackboard.active_reservations_by_robot.items():
            pos = ctx.blackboard.robot_positions.get(r_id, (0, 0))
            c_list = ctx.blackboard.get_conflicts(res, pos)
            for c in c_list:
                pair = tuple(sorted([r_id, c.robot_id]))
                if pair not in seen:
                    seen.add(pair)
                    conflicts.append({
                        "conflict_id": f"CONF_{pair[0]}_{pair[1]}_{ctx.current_time}",
                        "robots": [pair[0], pair[1]],
                        "location": pos,
                        "priority": f"Resolved by deterministic tie-break" if res.compare_priority(c) else "Yield required",
                        "reservation": str(res.path[:2]),
                        "resolution": "YIELD" if not res.valid else "PROCEED",
                        "status": "ACTIVE",
                        "timestamp": ctx.current_time
                    })
    return {"conflicts": conflicts}


def get_recent_events(limit=50):
    events = fleet_controller.EventLogger.get_recent_events(limit=limit)
    return {"count": len(events), "events": events}


def get_metrics():
    ctx = fleet_controller.FleetContext
    net_metrics = ctx.network.metrics if ctx.network else {"messages_sent": 0, "messages_received": 0, "messages_dropped": 0}
    ai_metrics = fleet_controller.SafetyArbiter.metrics
    
    total_decisions = ai_metrics.get("ai_decisions", 0)
    agreements = ai_metrics.get("ai_agreements", 0)
    disagreements = ai_metrics.get("ai_disagreements", 0)
    agreement_rate = (agreements / total_decisions) if total_decisions > 0 else 1.0

    all_latencies = []
    for a in ctx.agents:
        all_latencies.extend(a.edge_ai.latencies)
    
    avg_latency = (sum(all_latencies) / len(all_latencies)) if all_latencies else 0.0

    return {
        "collision_count": 0,
        "near_collision_count": 0,
        "deadlock_count": 0,
        "task_completion_count": sum(1 for a in ctx.agents if a.idle and not a.goal),
        "makespan": ctx.current_time,
        "total_distance": sum(len(a.current_res.path) for a in ctx.agents if a.current_res),
        "waiting_time": sum(1 for a in ctx.agents if (a.current_res and a.is_wait_path())),
        "idle_time": sum(1 for a in ctx.agents if a.idle),
        "conflict_count": len(get_conflicts().get("conflicts", [])),
        "reroute_count": sum(1 for e in fleet_controller.EventLogger._events if e.get("event_type") == "REROUTING_STARTED"),
        "reservation_failures": sum(1 for e in fleet_controller.EventLogger._events if e.get("event_type") == "RESERVATION_REJECTED"),
        "messages_sent": net_metrics.get("messages_sent", 0),
        "messages_received": net_metrics.get("messages_received", 0),
        "messages_dropped": net_metrics.get("messages_dropped", 0),
        "communication_latency": 0.0,
        "ai_decisions": total_decisions,
        "ai_agreements": agreements,
        "ai_disagreements": disagreements,
        "ai_agreement_rate": agreement_rate,
        "ai_latency_ms": avg_latency
    }


def get_simulation_state():
    ctx = fleet_controller.FleetContext
    robot_positions = {a.name: {"x": a.pos[0], "y": a.pos[1]} for a in ctx.agents}
    robot_velocities = {a.name: {"vx": 1.0 if not a.idle else 0.0, "vy": 0.0} for a in ctx.agents}
    robot_headings = {a.name: 0.0 for a in ctx.agents}
    active_obstacles = ctx.blackboard.get_active_dynamic_obstacles() if ctx.blackboard else []

    return {
        "simulation_time": ctx.current_time,
        "simulation_status": ctx.simulation_status,
        "robot_positions": robot_positions,
        "robot_velocities": robot_velocities,
        "robot_headings": robot_headings,
        "active_obstacles": [{"x": obs[0], "y": obs[1]} for obs in active_obstacles]
    }


def request_route_analysis(robot_id):
    """
    Read-only advisory route analysis. Uses existing A* heuristic and blackboard constraints.
    Does NOT modify active reservations or command movement.
    """
    ctx = fleet_controller.FleetContext
    agent = next((a for a in ctx.agents if a.name == robot_id), None)
    if not agent:
        return {"error": f"Robot '{robot_id}' not found"}

    current_path = agent.current_res.path if agent.current_res else []
    path_cost = len(current_path)
    
    known_conflicts = []
    if agent.current_res and ctx.blackboard:
        conflicts = ctx.blackboard.get_conflicts(agent.current_res, agent.pos)
        known_conflicts = [c.robot_id for c in conflicts]

    blocked_regions = list(ctx.blackboard.get_active_dynamic_obstacles()) if ctx.blackboard else []
    
    # Calculate possible alternative path purely for analysis
    alt_path = []
    if agent.goal:
        all_obs = ctx.static_obstacles.union(set(blocked_regions))
        for other_a in ctx.agents:
            if other_a.name != agent.name:
                all_obs.add(other_a.pos)
        alt_raw = fleet_controller.space_time_a_star(agent.pos, ctx.current_time, agent.goal, all_obs)
        if alt_raw:
            alt_path = alt_raw

    return {
        "robot_id": robot_id,
        "current_path": current_path,
        "path_cost": path_cost,
        "known_conflicts": known_conflicts,
        "blocked_regions": [{"x": b[0], "y": b[1]} for b in blocked_regions],
        "reservation_conflicts": len(known_conflicts) > 0,
        "possible_alternative_paths": [alt_path] if alt_path else []
    }


def start_simulation():
    fleet_controller.FleetContext.simulation_status = "RUNNING"
    return {"status": "SUCCESS", "simulation_status": "RUNNING"}


def pause_simulation():
    fleet_controller.FleetContext.simulation_status = "PAUSED"
    return {"status": "SUCCESS", "simulation_status": "PAUSED"}


def resume_simulation():
    fleet_controller.FleetContext.simulation_status = "RUNNING"
    return {"status": "SUCCESS", "simulation_status": "RUNNING"}


def reset_simulation():
    fleet_controller.FleetContext.simulation_status = "RESET"
    fleet_controller.EventLogger.clear()
    return {"status": "SUCCESS", "simulation_status": "RESET"}


def stop_simulation():
    fleet_controller.FleetContext.simulation_status = "STOPPED"
    return {"status": "SUCCESS", "simulation_status": "STOPPED"}
