import os
import sys

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
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

from pxr import Usd, UsdGeom, UsdPhysics, Gf  # type: ignore

paths = [
    r"c:\Users\goruv\kit-app-template\simulation5.usd",
    r"C:\Users\goruv\Downloads\simulation5.usd"
]

for p in paths:
    if not os.path.exists(p):
        continue
    print(f"\n==========================================")
    print(f"INSPECTING: {p}")
    print(f"==========================================")
    stage = Usd.Stage.Open(p)
    if not stage:
        print("Failed to open stage")
        continue

    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        name = prim.GetName()
        if any(k in path_str for k in ["Robot", "Warehouse", "Floor", "Physics", "Ground", "DynamicObstacle"]):
            type_name = prim.GetTypeName()
            attrs = [a.GetName() for a in prim.GetAttributes()]
            physics_attrs = [a for a in attrs if "physics" in a.lower()]
            print(f"Path: {path_str} | Type: {type_name} | PhysicsAttrs: {physics_attrs}")
