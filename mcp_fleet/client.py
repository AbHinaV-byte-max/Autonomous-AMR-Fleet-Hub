"""
MCP Client interface for AMR Fleet Coordination.
Enables programmatic querying and inspection of the fleet coordinator.
"""

import sys
import os
import json

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import mcp_fleet.tools as fleet_tools
from mcp_fleet.schemas import FLEET_MCP_TOOLS


class FleetMCPClient:
    def __init__(self, mode="in_process"):
        self.mode = mode
        self.connected = True

    def disconnect(self):
        """Simulate MCP client disconnection without affecting AMR fleet operations."""
        self.connected = False

    def reconnect(self):
        self.connected = True

    def list_tools(self):
        if not self.connected:
            raise ConnectionError("MCP Client is disconnected.")
        return FLEET_MCP_TOOLS

    def call_tool(self, name: str, arguments: dict = None):
        if not self.connected:
            raise ConnectionError("MCP Client is disconnected.")
        arguments = arguments or {}
        
        if name == "get_fleet_state":
            return fleet_tools.get_fleet_state()
        elif name == "get_robot_state":
            return fleet_tools.get_robot_state(arguments.get("robot_id", ""))
        elif name == "get_task_state":
            return fleet_tools.get_task_state(arguments.get("task_id", ""))
        elif name == "get_active_reservations":
            return fleet_tools.get_active_reservations()
        elif name == "get_conflicts":
            return fleet_tools.get_conflicts()
        elif name == "get_recent_events":
            return fleet_tools.get_recent_events(arguments.get("limit", 50))
        elif name == "get_metrics":
            return fleet_tools.get_metrics()
        elif name == "get_simulation_state":
            return fleet_tools.get_simulation_state()
        elif name == "request_route_analysis":
            return fleet_tools.request_route_analysis(arguments.get("robot_id", ""))
        elif name == "start_simulation":
            return fleet_tools.start_simulation()
        elif name == "pause_simulation":
            return fleet_tools.pause_simulation()
        elif name == "resume_simulation":
            return fleet_tools.resume_simulation()
        elif name == "reset_simulation":
            return fleet_tools.reset_simulation()
        elif name == "stop_simulation":
            return fleet_tools.stop_simulation()
        else:
            raise ValueError(f"Unknown MCP tool '{name}'")
