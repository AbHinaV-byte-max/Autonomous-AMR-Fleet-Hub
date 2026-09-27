"""
Bake and Run Omniverse Simulation Animation.
Bakes the complete multi-AMR space-time simulation directly as USD TimeSamples
into assets/omniverse/simulation5.usd (both in Downloads and repository) so Omniverse Kit viewport
plays the live motion smoothly when playing the timeline!
"""

import os
import sys
import math

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import fleet_controller
from fleet_controller import (
    AMR, SharedBlackboard, CommunicationNetwork, TaskManager,
    DynamicObstacle, EventLogger, FleetContext
)
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


def bake_simulation_to_usd(usd_paths=None, frames_per_timestep=20):
    if not usd_paths:
        usd_paths = [
            os.path.join(REPO_ROOT, "assets/omniverse/simulation5.usd"),
            r"C:\Users\goruv\Downloads\assets/omniverse/simulation5.usd"
        ]
        
    for usd_path in usd_paths:
        if not os.path.exists(usd_path):
            continue
            
        print(f"\n[ANIMATION BAKER] Opening USD Stage: {usd_path}")
        stage = Usd.Stage.Open(usd_path)
        if not stage:
            print(f"Failed to open {usd_path}")
            continue

        # Set stage timeline bounds
        stage.SetTimeCodesPerSecond(60.0)
        stage.SetStartTimeCode(0.0)
        
        # Setup robot ops
        robot_ops = {}
        for r_id in ["AMR_01", "AMR_02", "AMR_03"]:
            prim = stage.GetPrimAtPath(f"/World/P_DYNEX_Depot/Robots/{r_id}")
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
                robot_ops[r_id] = (translate_op, rotate_op)

        # Dynamic obstacle
        obs_prim = stage.GetPrimAtPath("/World/P_DYNEX_Depot/DynamicObstacle")
        obs_translate_op = None
        obs_imageable = None
        if obs_prim.IsValid():
            obs_xform = UsdGeom.Xformable(obs_prim)
            for op in obs_xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    obs_translate_op = op
                    break
            if not obs_translate_op:
                obs_translate_op = obs_xform.AddTranslateOp()
            obs_imageable = UsdGeom.Imageable(obs_prim)

        # Run 3-AMR warehouse scenario
        blackboard = SharedBlackboard()
        network = CommunicationNetwork()
        EventLogger.clear()
        
        # AMR initial depot coordinates
        agents = [
            AMR("AMR_01", (8, 10), None, blackboard),
            AMR("AMR_02", (10, 10), None, blackboard),
            AMR("AMR_03", (10, 6), None, blackboard)
        ]
        
        # Tasks across warehouse aisles
        task_mgr = TaskManager([(14, 10), (10, 14), (6, 6)])
        task_mgr.allocate_tasks(agents, 0, set())
        
        dyn_obs = DynamicObstacle("PALLET_BLOCK_01", (12, 10), activation_time=2)
        blackboard.add_dynamic_obstacle(dyn_obs)
        
        total_steps = 10
        total_frames = total_steps * frames_per_timestep
        stage.SetEndTimeCode(float(total_frames))
        
        for step in range(total_steps):
            current_time = step
            frame = float(step * frames_per_timestep)
            
            if current_time >= 2:
                dyn_obs.active = True
                if obs_translate_op:
                    obs_translate_op.Set(Gf.Vec3d(12.0, 10.0, 0.25), time=frame)
                if obs_imageable:
                    obs_imageable.MakeVisible()
            else:
                if obs_translate_op:
                    obs_translate_op.Set(Gf.Vec3d(12.0, 10.0, -10.0), time=frame)
                if obs_imageable:
                    obs_imageable.MakeInvisible()

            network.broadcast_state(agents, current_time)
            
            resolved_all = False
            while not resolved_all:
                resolved_all = True
                for amr in agents:
                    if not amr.idle and not amr.failed and amr.current_res and not amr.current_res.valid:
                        amr.resolve_conflicts(current_time)
                        resolved_all = False
                        
            for amr in agents:
                amr.step(current_time)
                # Bake time-sampled pose
                if amr.name in robot_ops:
                    trans_op, rot_op = robot_ops[amr.name]
                    trans_op.Set(Gf.Vec3d(float(amr.pos[0]), float(amr.pos[1]), 0.035), time=frame)
                    # Also write default for immediate static view
                    trans_op.Set(Gf.Vec3d(float(amr.pos[0]), float(amr.pos[1]), 0.035))
                    
        stage.Save()
        print(f"[ANIMATION BAKER] Saved baked time-sampled animation (0 - {total_frames} frames) to {usd_path}")

    print("\n✓ Animation Baking Complete: Open or play timeline in Omniverse to see AMRs move in real-time!")


if __name__ == "__main__":
    bake_simulation_to_usd()
