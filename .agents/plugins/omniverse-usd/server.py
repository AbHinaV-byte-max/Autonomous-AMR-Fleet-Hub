"""
Omniverse & OpenUSD Model Context Protocol (MCP) Server
Provides OpenUSD scene inspection, manipulation, and Omniverse Kit launching tools.
"""

import sys
import os
import json
import traceback
import subprocess

# Ensure stdout is unbuffered or UTF-8
sys.stdout.reconfigure(encoding='utf-8')
sys.stdin.reconfigure(encoding='utf-8')

# Configure Omniverse USD Environment
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
KIT_RELEASE_DIR = os.path.join(REPO_ROOT, "_build", "windows-x86_64", "release")
EXTSCACHE_DIR = os.path.join(KIT_RELEASE_DIR, "extscache")

usd_libs_dir = None
if os.path.exists(EXTSCACHE_DIR):
    for entry in os.listdir(EXTSCACHE_DIR):
        if entry.startswith("omni.usd.libs"):
            usd_libs_dir = os.path.join(EXTSCACHE_DIR, entry)
            break

if usd_libs_dir:
    bin_dir = os.path.join(usd_libs_dir, "bin")
    if hasattr(os, "add_dll_directory") and os.path.exists(bin_dir):
        os.add_dll_directory(bin_dir)
    os.environ["PATH"] = bin_dir + ";" + os.environ.get("PATH", "")
    if usd_libs_dir not in sys.path:
        sys.path.insert(0, usd_libs_dir)

try:
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf, Vt  # type: ignore
    USD_AVAILABLE = True
except Exception as e:
    USD_AVAILABLE = False
    USD_IMPORT_ERROR = str(e)

# Open stages cache: {filepath: stage}
_stages = {}

def get_stage(path: str):
    path = os.path.abspath(path)
    if path in _stages:
        return _stages[path]
    if not os.path.exists(path):
        raise FileNotFoundError(f"USD file not found: {path}")
    stage = Usd.Stage.Open(path)
    if not stage:
        raise ValueError(f"Failed to open USD stage from {path}")
    _stages[path] = stage
    return stage

# Tool Definitions
TOOLS = [
    {
        "name": "usd_open_stage",
        "description": "Opens and inspects an OpenUSD file (.usd, .usda, .usdc). Returns stage metadata, up-axis, linear units, timecodes, and top-level prim hierarchy.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "Absolute path to the USD file (e.g. C:/Users/goruv/Downloads/simulation5.usd)"
                }
            },
            "required": ["file_path"]
        }
    },
    {
        "name": "usd_list_prims",
        "description": "Lists all prims or children under a given path in the USD stage, with their types, active status, and kinds.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "Absolute path to the USD file"
                },
                "root_path": {
                    "type": "string",
                    "description": "Root path to list prims from (defaults to '/' for all prims)",
                    "default": "/"
                },
                "recursive": {
                    "type": "boolean",
                    "description": "Whether to traverse recursively down the hierarchy (default true)",
                    "default": True
                },
                "max_depth": {
                    "type": "integer",
                    "description": "Maximum recursion depth when recursive is true (default 3)",
                    "default": 3
                }
            },
            "required": ["file_path"]
        }
    },
    {
        "name": "usd_get_prim_details",
        "description": "Inspects a specific prim in detail: type, attributes, values, property list, and applied schemas.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "Absolute path to the USD file"
                },
                "prim_path": {
                    "type": "string",
                    "description": "Prim path (e.g. '/World', '/Environment/defaultLight')"
                }
            },
            "required": ["file_path", "prim_path"]
        }
    },
    {
        "name": "usd_dump_usda",
        "description": "Exports or dumps a USD stage or prim hierarchy to human-readable USDA text.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "Absolute path to the USD file"
                },
                "max_lines": {
                    "type": "integer",
                    "description": "Maximum number of USDA lines to return (default 200)",
                    "default": 200
                }
            },
            "required": ["file_path"]
        }
    },
    {
        "name": "launch_omniverse_editor",
        "description": "Launches the Omniverse Kit application with the specified USD file loaded directly in the 3D viewport.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "Absolute path to the USD file to open in Omniverse Editor"
                }
            },
            "required": ["file_path"]
        }
    }
]

def handle_usd_open_stage(args):
    if not USD_AVAILABLE:
        return f"USD library unavailable: {USD_IMPORT_ERROR}"
    file_path = args.get("file_path")
    stage = get_stage(file_path)
    
    root_prims = []
    for prim in stage.GetPseudoRoot().GetChildren():
        root_prims.append({
            "path": str(prim.GetPath()),
            "type": prim.GetTypeName(),
            "isActive": prim.IsActive()
        })
        
    info = {
        "filePath": os.path.abspath(file_path),
        "defaultPrim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "upAxis": UsdGeom.GetStageUpAxis(stage),
        "metersPerUnit": UsdGeom.GetStageMetersPerUnit(stage),
        "startTimeCode": stage.GetStartTimeCode(),
        "endTimeCode": stage.GetEndTimeCode(),
        "timeCodesPerSecond": stage.GetTimeCodesPerSecond(),
        "rootPrims": root_prims
    }
    return json.dumps(info, indent=2)

def handle_usd_list_prims(args):
    if not USD_AVAILABLE:
        return f"USD library unavailable: {USD_IMPORT_ERROR}"
    file_path = args.get("file_path")
    root_path_str = args.get("root_path", "/")
    recursive = args.get("recursive", True)
    max_depth = args.get("max_depth", 3)
    
    stage = get_stage(file_path)
    
    if root_path_str == "/":
        start_prim = stage.GetPseudoRoot()
    else:
        start_prim = stage.GetPrimAtPath(root_path_str)
        if not start_prim:
            return f"Prim path '{root_path_str}' not found in stage."

    results = []
    
    def traverse(prim, current_depth):
        for child in prim.GetChildren():
            results.append({
                "path": str(child.GetPath()),
                "name": child.GetName(),
                "typeName": child.GetTypeName() or "(untyped)",
                "isActive": child.IsActive()
            })
            if recursive and current_depth < max_depth:
                traverse(child, current_depth + 1)
                
    traverse(start_prim, 1)
    return json.dumps({"count": len(results), "prims": results}, indent=2)

def handle_usd_get_prim_details(args):
    if not USD_AVAILABLE:
        return f"USD library unavailable: {USD_IMPORT_ERROR}"
    file_path = args.get("file_path")
    prim_path_str = args.get("prim_path")
    
    stage = get_stage(file_path)
    prim = stage.GetPrimAtPath(prim_path_str)
    if not prim or not prim.IsValid():
        return f"Prim '{prim_path_str}' not found or invalid."
        
    attrs = {}
    for attr in prim.GetAttributes():
        name = attr.GetName()
        val = attr.Get()
        attrs[name] = {
            "typeName": str(attr.GetTypeName()),
            "value": str(val) if val is not None else None
        }
        
    details = {
        "path": str(prim.GetPath()),
        "name": prim.GetName(),
        "typeName": prim.GetTypeName(),
        "isActive": prim.IsActive(),
        "attributes": attrs,
        "appliedSchemas": list(prim.GetAppliedSchemas()),
        "children": [str(c.GetPath()) for c in prim.GetChildren()]
    }
    return json.dumps(details, indent=2)

def handle_usd_dump_usda(args):
    if not USD_AVAILABLE:
        return f"USD library unavailable: {USD_IMPORT_ERROR}"
    file_path = args.get("file_path")
    max_lines = args.get("max_lines", 200)
    
    stage = get_stage(file_path)
    root_layer = stage.GetRootLayer()
    ascii_content = root_layer.ExportToString()
    
    lines = ascii_content.splitlines()
    total_lines = len(lines)
    truncated = False
    if total_lines > max_lines:
        lines = lines[:max_lines]
        truncated = True
        
    res = "\n".join(lines)
    if truncated:
        res += f"\n... [Truncated {total_lines - max_lines} more lines]"
    return res

def handle_launch_omniverse_editor(args):
    file_path = os.path.abspath(args.get("file_path"))
    bat_path = os.path.join(KIT_RELEASE_DIR, "my_company.my_editor.kit.bat")
    
    if not os.path.exists(bat_path):
        # Fallback to kit.bat
        bat_path = os.path.join(KIT_RELEASE_DIR, "kit.bat")
        
    if not os.path.exists(bat_path):
        return f"Omniverse Kit launcher not found at {bat_path}."
        
    cmd = [bat_path, file_path]
    subprocess.Popen(cmd, cwd=KIT_RELEASE_DIR)
    return f"Successfully initiated Omniverse Kit with {file_path} (Process running in background)."

def dispatch_tool(name: str, args: dict):
    if name == "usd_open_stage":
        return handle_usd_open_stage(args)
    elif name == "usd_list_prims":
        return handle_usd_list_prims(args)
    elif name == "usd_get_prim_details":
        return handle_usd_get_prim_details(args)
    elif name == "usd_dump_usda":
        return handle_usd_dump_usda(args)
    elif name == "launch_omniverse_editor":
        return handle_launch_omniverse_editor(args)
    else:
        raise ValueError(f"Unknown tool: {name}")

def send_response(response: dict):
    body = json.dumps(response, separators=(',', ':'))
    sys.stdout.write(body + "\n")
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
                        "name": "omniverse-usd",
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
                    "tools": TOOLS
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
