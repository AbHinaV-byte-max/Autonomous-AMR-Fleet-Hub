"""
MCP Tool Schemas for AMR Fleet Inspection, Advisory, and Lifecycle Controls.
"""

FLEET_MCP_TOOLS = [
    {
        "name": "get_fleet_state",
        "description": "Returns the complete live state of all AMRs, active tasks, communication topology, and active space-time reservations.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "get_robot_state",
        "description": "Returns the detailed operational telemetry, position, goal, battery, and peer communication state for a specific robot.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "robot_id": {
                    "type": "string",
                    "description": "Identifier of the AMR (e.g., 'AMR_01')"
                }
            },
            "required": ["robot_id"]
        }
    },
    {
        "name": "get_task_state",
        "description": "Returns status, assigned robot, pickup/drop locations, and deadline for a specific task.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "task_id": {
                    "type": "string",
                    "description": "Task identifier or goal coordinate string (e.g. '(42, 40)')"
                }
            },
            "required": ["task_id"]
        }
    },
    {
        "name": "get_active_reservations",
        "description": "Returns all currently active space-time reservations across the fleet.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "get_conflicts",
        "description": "Returns all detected space-time conflicts, participating robots, and resolution states.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "get_recent_events",
        "description": "Returns a chronological log of recent fleet events (conflicts, yields, reroutes, failures, safety decisions).",
        "inputSchema": {
            "type": "object",
            "properties": {
                "limit": {
                    "type": "integer",
                    "description": "Maximum number of recent events to return (default 50)",
                    "default": 50
                }
            },
            "required": []
        }
    },
    {
        "name": "get_metrics",
        "description": "Returns system-wide performance, reliability, network, and Edge-AI metrics.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "get_simulation_state",
        "description": "Returns current simulation lifecycle state, timestep, robot positions, and active dynamic obstacles.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "request_route_analysis",
        "description": "Performs advisory, read-only space-time route analysis for an AMR without modifying any active reservations or plans.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "robot_id": {
                    "type": "string",
                    "description": "Identifier of the AMR to analyze"
                }
            },
            "required": ["robot_id"]
        }
    },
    {
        "name": "start_simulation",
        "description": "Sets simulation lifecycle status to RUNNING.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "pause_simulation",
        "description": "Sets simulation lifecycle status to PAUSED.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "resume_simulation",
        "description": "Resumes simulation lifecycle status to RUNNING.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "reset_simulation",
        "description": "Resets simulation context and clears event buffers.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "stop_simulation",
        "description": "Sets simulation lifecycle status to STOPPED.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    }
]
