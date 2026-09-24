"""
Omniverse Simulator Adapter for AMR Fleet Coordination.
Provides bidirectional integration between the Decentralized Coordination Core and OpenUSD (simulation5.usd).
"""

import os
import sys
import math
import time

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# Configure Omniverse USD Environment
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
    from pxr import Usd, UsdGeom, Gf  # type: ignore
except ImportError:
    Usd = UsdGeom = Gf = None  # type: ignore


class OmniverseSimulatorAdapter:
    """
    Simulator-agnostic adapter connecting coordination core to OpenUSD stage.
    Handles coordinate transformations, USD Prim manipulation, and telemetry streaming.
    """
    def __init__(self, usd_path=None, cell_size=1.0, origin_offset=(0.0, 0.0, 0.035)):
        self.usd_path = usd_path or os.path.join(REPO_ROOT, "simulation5.usd")
        self.cell_size = float(cell_size)
        self.origin_offset = origin_offset
        self.stage = None
        self.robot_ops = {}
        self.last_telemetry = {}
        self.last_time = 0
        self._load_stage()

    def _load_stage(self):
        if os.path.exists(self.usd_path):
            self.stage = Usd.Stage.Open(self.usd_path)
            print(f"[USD ADAPTER] Successfully loaded USD stage from {self.usd_path}")
        else:
            print(f"[USD ADAPTER] Warning: {self.usd_path} not found. Running in headless virtual mode.")

    def grid_to_world(self, gx, gy, gz=None):
        """Transform discrete grid coordinates to continuous USD world coordinates (meters)."""
        wx = gx * self.cell_size + self.origin_offset[0]
        wy = gy * self.cell_size + self.origin_offset[1]
        wz = gz if gz is not None else self.origin_offset[2]
        return (wx, wy, wz)

    def world_to_grid(self, wx, wy):
        """Transform continuous USD world coordinates to nearest discrete grid coordinates."""
        gx = round((wx - self.origin_offset[0]) / self.cell_size)
        gy = round((wy - self.origin_offset[1]) / self.cell_size)
        return (gx, gy)

    def register_robot(self, robot_id):
        """Cache USD Prim translation and rotation ops for fast updates."""
        if not self.stage:
            return
        prim_path = f"/World/P_DYNEX_Depot/Robots/{robot_id}"
        prim = self.stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            translate_op = None
            rotate_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                elif op.GetOpType() in [UsdGeom.XformOp.TypeRotateZ, UsdGeom.XformOp.TypeRotateXYZ]:
                    rotate_op = op
            if not translate_op:
                translate_op = xform.AddTranslateOp()
            if not rotate_op:
                rotate_op = xform.AddRotateZOp()
            self.robot_ops[robot_id] = (translate_op, rotate_op)
            print(f"[USD ADAPTER] Registered USD prim for {robot_id} at {prim_path}")
        else:
            print(f"[USD ADAPTER] Warning: Prim not found at {prim_path}")

    def update_robot_pose(self, robot_id, gx, gy, heading_deg=0.0, current_time=0):
        """Update robot 3D translation and orientation in the USD stage."""
        wx, wy, wz = self.grid_to_world(gx, gy)
        
        # Update USD Prim
        if robot_id in self.robot_ops:
            translate_op, rotate_op = self.robot_ops[robot_id]
            translate_op.Set(Gf.Vec3d(wx, wy, wz))
            if rotate_op:
                rotate_op.Set(heading_deg)
                
        # Calculate velocity telemetry
        vx, vy = 0.0, 0.0
        dt = current_time - self.last_time if current_time > self.last_time else 1.0
        if robot_id in self.last_telemetry:
            prev_x, prev_y = self.last_telemetry[robot_id]["position"]["x"], self.last_telemetry[robot_id]["position"]["y"]
            vx = (wx - prev_x) / dt
            vy = (wy - prev_y) / dt
            
        self.last_telemetry[robot_id] = {
            "robot_id": robot_id,
            "position": {"x": wx, "y": wy, "z": wz},
            "velocity": {"vx": vx, "vy": vy, "speed": math.sqrt(vx*vx + vy*vy)},
            "heading": heading_deg,
            "timestamp": current_time
        }
        self.last_time = current_time

    def get_robot_telemetry(self, robot_id):
        """Return real-time simulator telemetry for an AMR."""
        if robot_id in self.last_telemetry:
            return self.last_telemetry[robot_id]
        return {
            "robot_id": robot_id,
            "position": {"x": 0.0, "y": 0.0, "z": 0.035},
            "velocity": {"vx": 0.0, "vy": 0.0, "speed": 0.0},
            "heading": 0.0,
            "timestamp": 0
        }

    def set_dynamic_obstacle_state(self, gx, gy, active=True):
        """Update USD dynamic obstacle prim translation and visibility."""
        if not self.stage:
            return
        obs_prim = self.stage.GetPrimAtPath("/World/P_DYNEX_Depot/DynamicObstacle")
        if obs_prim.IsValid():
            wx, wy, wz = self.grid_to_world(gx, gy, gz=0.25)
            xform = UsdGeom.Xformable(obs_prim)
            translate_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break
            if not translate_op:
                translate_op = xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(wx, wy, wz))
            imageable = UsdGeom.Imageable(obs_prim)
            if active:
                imageable.MakeVisible()
            else:
                imageable.MakeInvisible()
            print(f"[USD ADAPTER] Dynamic obstacle updated at ({wx:.2f}, {wy:.2f}), visible={active}")
