"""
AMR Fleet Model Context Protocol (MCP) Server.
Exposes fleet state, robot telemetry, route analysis, and event log over JSON-RPC 2.0 stdio.
"""

import sys
import os
import json
import traceback

sys.stdout.reconfigure(encoding='utf-8')
sys.stdin.reconfigure(encoding='utf-8')

# Ensure repo root is in sys.path
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from mcp_fleet.schemas import FLEET_MCP_TOOLS
import mcp_fleet.tools as fleet_tools


def dispatch_tool(tool_name: str, arguments: dict):
    if tool_name == "get_fleet_state":
        return json.dumps(fleet_tools.get_fleet_state(), indent=2)
    elif tool_name == "get_robot_state":
        return json.dumps(fleet_tools.get_robot_state(arguments.get("robot_id", "")), indent=2)
    elif tool_name == "get_task_state":
        return json.dumps(fleet_tools.get_task_state(arguments.get("task_id", "")), indent=2)
    elif tool_name == "get_active_reservations":
        return json.dumps(fleet_tools.get_active_reservations(), indent=2)
    elif tool_name == "get_conflicts":
        return json.dumps(fleet_tools.get_conflicts(), indent=2)
    elif tool_name == "get_recent_events":
        return json.dumps(fleet_tools.get_recent_events(arguments.get("limit", 50)), indent=2)
    elif tool_name == "get_metrics":
        return json.dumps(fleet_tools.get_metrics(), indent=2)
    elif tool_name == "get_simulation_state":
        return json.dumps(fleet_tools.get_simulation_state(), indent=2)
    elif tool_name == "request_route_analysis":
        return json.dumps(fleet_tools.request_route_analysis(arguments.get("robot_id", "")), indent=2)
    elif tool_name == "start_simulation":
        return json.dumps(fleet_tools.start_simulation(), indent=2)
    elif tool_name == "pause_simulation":
        return json.dumps(fleet_tools.pause_simulation(), indent=2)
    elif tool_name == "resume_simulation":
        return json.dumps(fleet_tools.resume_simulation(), indent=2)
    elif tool_name == "reset_simulation":
        return json.dumps(fleet_tools.reset_simulation(), indent=2)
    elif tool_name == "stop_simulation":
        return json.dumps(fleet_tools.stop_simulation(), indent=2)
    else:
        raise ValueError(f"Unknown MCP tool: '{tool_name}'")


def send_response(resp: dict):
    sys.stdout.write(json.dumps(resp) + "\n")
    sys.stdout.flush()


def main():
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        line = line.strip()
        if not line:
            continue
            
        try:
            req = json.loads(line)
        except json.JSONDecodeError:
            continue
            
        req_id = req.get("id")
        method = req.get("method")
        params = req.get("params", {})
        
        if method == "initialize":
            send_response({
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {
                        "tools": {}
                    },
                    "serverInfo": {
                        "name": "amr-fleet-coordinator",
                        "version": "1.0.0"
                    }
                }
            })
        elif method == "notifications/initialized":
            pass
        elif method == "ping":
            send_response({
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {}
            })
        elif method == "tools/list":
            send_response({
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {
                    "tools": FLEET_MCP_TOOLS
                }
            })
        elif method == "tools/call":
            tool_name = params.get("name")
            tool_args = params.get("arguments", {})
            try:
                result_text = dispatch_tool(tool_name, tool_args)
                send_response({
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {
                        "content": [
                            {"type": "text", "text": result_text}
                        ],
                        "isError": False
                    }
                })
            except Exception as e:
                err_msg = f"Error executing tool '{tool_name}': {str(e)}\n{traceback.format_exc()}"
                send_response({
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {
                        "content": [
                            {"type": "text", "text": err_msg}
                        ],
                        "isError": True
                    }
                })
        else:
            if req_id is not None:
                send_response({
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "error": {
                        "code": -32601,
                        "message": f"Method '{method}' not found"
                    }
                })


if __name__ == "__main__":
    main()
