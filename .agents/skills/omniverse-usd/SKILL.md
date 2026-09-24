---
name: omniverse-usd
description: >-
  Use this skill to inspect, query, modify, and launch OpenUSD scenes (.usd, .usda, .usdc)
  and NVIDIA Omniverse Kit applications using the embedded Omniverse Kit Python runtime and MCP server.
---

# Omniverse & OpenUSD Skill

This skill provides step-by-step instructions and command references to inspect, query, author, and launch OpenUSD files (such as `.usd`, `.usda`, and `.usdc`) using NVIDIA Omniverse Kit.

## Key Capabilities

1. **Direct USD Stage Inspection**:
   - Query root prims, stage up-axis, linear units, timecodes, metadata.
   - List prims and child hierarchies with their types (`Xform`, `Mesh`, `Camera`, `DistantLight`, etc.).
   - Inspect prim attributes, applied API schemas, and property values.
   - Export binary USD crates to human-readable USDA text.

2. **Omniverse Kit Application Launching**:
   - Open any USD file directly in the Omniverse Kit viewport:
     ```cmd
     _build\windows-x86_64\release\my_company.my_editor.kit.bat <path_to_usd>
     ```

## Python USD Environment

The Omniverse Kit environment provides full Python 3.12 bindings with `pxr.Usd`:
- **Python Binary**: `_build\windows-x86_64\release\kit\python\python.exe`
- **USD Libraries Path**: `_build\windows-x86_64\release\extscache\omni.usd.libs-*\bin`

### Running USD Python Scripts

```python
import os, sys

kit_root = r"c:\Users\goruv\kit-app-template\_build\windows-x86_64\release"
extscache = os.path.join(kit_root, "extscache")

# Find omni.usd.libs
for item in os.listdir(extscache):
    if item.startswith("omni.usd.libs"):
        usd_libs = os.path.join(extscache, item)
        break

bin_dir = os.path.join(usd_libs, "bin")
os.add_dll_directory(bin_dir)
os.environ["PATH"] = bin_dir + ";" + os.environ.get("PATH", "")
sys.path.insert(0, usd_libs)

from pxr import Usd, UsdGeom

stage = Usd.Stage.Open(r"C:\Users\goruv\Downloads\simulation5.usd")
print("Default Prim:", stage.GetDefaultPrim())
```

## MCP Server Tools

The MCP server is located at `.agents/plugins/omniverse-usd/server.py` and exposes:
- `usd_open_stage`: Opens and inspects stage metadata and root prims.
- `usd_list_prims`: Lists hierarchy under a prim path.
- `usd_get_prim_details`: Inspects attributes and schemas of a specific prim.
- `usd_dump_usda`: Dumps stage content as human-readable ASCII text.
- `launch_omniverse_editor`: Launches Omniverse Kit Base Editor with the USD scene.
