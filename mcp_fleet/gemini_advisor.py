"""
Gemini AI Advisory & Decision Explanation Engine for AMR Fleet Coordination.
Uses MCP tools to inspect grounded telemetry and explain decentralized coordination events.
"""

import sys
import os
import json

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from mcp_fleet.client import FleetMCPClient


class GeminiFleetAdvisor:
    """
    Advisory, Inspection, and Explanation Engine powered by grounded MCP telemetry.
    Strictly observational — does NOT control motors, velocity, or safety arbitration.
    """
    def __init__(self, mcp_client: FleetMCPClient = None):
        self.mcp = mcp_client or FleetMCPClient()

    def answer_query(self, query: str) -> str:
        q = query.lower()
        if "fleet state" in q or "where are" in q:
            return self.explain_fleet_state()
        elif "why is" in q and "waiting" in q:
            # e.g. "Why is AMR_02 waiting?"
            for r_id in ["amr_01", "amr_02", "amr_03"]:
                if r_id in q:
                    return self.explain_robot_decision(r_id.upper())
            return self.explain_robot_decision("AMR_02")
        elif "override" in q or "edge-ai" in q or "arbiter" in q:
            for r_id in ["amr_01", "amr_02", "amr_03"]:
                if r_id in q:
                    return self.explain_edge_ai_override(r_id.upper())
            return self.explain_edge_ai_override("AMR_01")
        elif "reroute" in q or "blockage" in q or "obstacle" in q:
            for r_id in ["amr_01", "amr_02", "amr_03"]:
                if r_id in q:
                    return self.explain_reroute_decision(r_id.upper())
            return self.explain_reroute_decision("AMR_01")
        elif "deadlock" in q:
            return self.explain_deadlock_resolution()
        elif "communication" in q or "safe mode" in q or "degraded" in q:
            return self.explain_comm_degradation()
        elif "performance" in q or "metrics" in q:
            return self.explain_performance_metrics()
        else:
            return self.explain_fleet_state()

    def explain_fleet_state(self) -> str:
        state = self.mcp.call_tool("get_fleet_state")
        robots = state.get("robots", [])
        time_step = state.get("simulation_time", 0)
        
        lines = [f"=== Grounded Fleet Telemetry (Timestep {time_step}) ==="]
        for r in robots:
            pos = r['position']
            goal = r['goal']
            status = "FAILED" if r['failed'] else ("SAFE_MODE" if r['safe_mode'] else ("IDLE" if r['idle'] else "ACTIVE"))
            lines.append(f"- {r['robot_id']}: Position ({pos['x']}, {pos['y']}), Goal: {goal}, Status: {status}")
            
        reservations = state.get("active_reservations", [])
        lines.append(f"Active Space-Time Reservations: {len(reservations)}")
        conflicts = state.get("active_conflicts", [])
        lines.append(f"Active Conflicts Detected: {len(conflicts)}")
        return "\n".join(lines)

    def explain_robot_decision(self, robot_id: str) -> str:
        r_state = self.mcp.call_tool("get_robot_state", {"robot_id": robot_id})
        if "error" in r_state:
            return f"Explanation Error: {r_state['error']}"

        events = self.mcp.call_tool("get_recent_events", {"limit": 20}).get("events", [])
        relevant_events = [e for e in events if e.get("details", {}).get("robot") == robot_id or e.get("details", {}).get("robot_id") == robot_id]

        status = r_state.get("state")
        if status == "SAFE_MODE":
            return (f"{robot_id} is in SAFE_MODE (STOPPED) because peer communication heartbeats exceeded "
                    f"the maximum allowable threshold T_SAFE_MODE. Deterministic safety rules enforce full halt.")
        
        if status == "IDLE":
            return f"{robot_id} is currently IDLE as it has completed its assigned task and is awaiting new allocations."

        # Check recent conflicts and arbitration
        recent_conflict = next((e for e in reversed(events) if "CONFLICT" in e.get("event_type", "")), None)
        recent_arb = next((e for e in reversed(events) if "AI_SAFETY" in e.get("event_type", "")), None)
        
        if recent_arb and recent_arb.get("event_type") == "AI_SAFETY_DISAGREEMENT":
            reason = recent_arb.get("details", {}).get("override_reason", "Safety Arbiter override")
            return (f"{robot_id} is waiting because the deterministic Safety Arbiter rejected an optimistic AI recommendation. "
                    f"Override reason: {reason}.")

        if recent_conflict:
            return (f"{robot_id} is waiting/yielding due to a space-time reservation conflict. "
                    f"The higher-priority AMR was granted the trajectory reservation, while {robot_id} yielded deterministically.")

        return f"{robot_id} is operating nominally along its reserved space-time trajectory towards goal {r_state.get('current_goal')}."

    def explain_edge_ai_override(self, robot_id: str) -> str:
        events = self.mcp.call_tool("get_recent_events", {"limit": 50}).get("events", [])
        disagreements = [e for e in events if e.get("event_type") == "AI_SAFETY_DISAGREEMENT" and (e.get("details", {}).get("robot") == robot_id or not e.get("details", {}).get("robot"))]
        
        if disagreements:
            last_dis = disagreements[-1]["details"]
            ai_rec = last_dis.get("ai_recommendation", "PROCEED")
            final_act = last_dis.get("final_action", "WAIT")
            reason = last_dis.get("override_reason", "Hard safety invariant")
            return (f"Edge-AI recommended '{ai_rec}'. However, the deterministic Safety Arbiter intervened and overrode the action "
                    f"to '{final_act}'. Rationale: {reason}. (Authoritative Safety Boundary intact)")
        
        return f"No safety overrides logged for {robot_id}. Edge-AI recommendations and Safety Arbiter constraints are in full agreement."

    def explain_reroute_decision(self, robot_id: str) -> str:
        events = self.mcp.call_tool("get_recent_events", {"limit": 50}).get("events", [])
        reroutes = [e for e in events if e.get("event_type") in ["BLOCKAGE_DETECTED", "REROUTING_STARTED"]]
        if reroutes:
            return (f"{robot_id} initiated decentralized local rerouting after detecting an active dynamic obstacle "
                    f"intersecting its planned space-time reservation. A* computed an alternative conflict-free route.")
        return f"No dynamic blockage or rerouting events currently logged for {robot_id}."

    def explain_deadlock_resolution(self) -> str:
        events = self.mcp.call_tool("get_recent_events", {"limit": 50}).get("events", [])
        deadlocks = [e for e in events if "DEADLOCK" in e.get("event_type", "")]
        if deadlocks:
            return ("A deadlock cycle was detected in the decentralized Wait-For Graph (WFG). "
                    "The AMR with the lowest deterministic ID was selected as the recovery agent to perform YIELD_AND_REPLAN, "
                    "breaking the mutual wait cycle safely.")
        return "No deadlock cycles currently active in the fleet."

    def explain_comm_degradation(self) -> str:
        events = self.mcp.call_tool("get_recent_events", {"limit": 50}).get("events", [])
        comm_events = [e for e in events if "COMMUNICATION" in e.get("event_type", "")]
        if comm_events:
            return ("Communication network dropped packets between peers. AMRs expanded spatial safety margins around stale "
                    "last-known peer positions during DEGRADED status, and enforced full STOP upon reaching SAFE_MODE threshold.")
        return "Communication network operating with nominal 100% packet delivery."

    def explain_performance_metrics(self) -> str:
        metrics = self.mcp.call_tool("get_metrics")
        return (f"System Metrics:\n"
                f"- Task Completions: {metrics['task_completion_count']}\n"
                f"- Total AI Decisions: {metrics['ai_decisions']}\n"
                f"- AI Agreement Rate: {metrics['ai_agreement_rate'] * 100:.1f}%\n"
                f"- Avg AI Decision Latency: {metrics['ai_latency_ms']:.4f} ms\n"
                f"- Dropped Packets: {metrics['messages_dropped']}")
