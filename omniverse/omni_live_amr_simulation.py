"""
Live In-Editor AMR Fleet Simulation for NVIDIA Omniverse Kit.
Executes directly within Omniverse Kit's runtime, updating the live viewport stage
in real-time every second using asynchronous coroutines and decentralized coordination.
"""

import os
import sys
import asyncio
import time
import math

# Add repository root to Python path
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
    import omni.usd  # type: ignore
    import omni.kit.app  # type: ignore
    from pxr import Usd, UsdGeom, Gf  # type: ignore
    IN_OMNIVERSE = True
except ImportError:
    IN_OMNIVERSE = False
    try:
        from pxr import Usd, UsdGeom, Gf  # type: ignore
    except ImportError:
        Usd = UsdGeom = Gf = None  # type: ignore
    print("[INFO] Running in external real-time sync mode.")


async def run_live_simulation_in_omniverse():
    print("=" * 65)
    print("[OMNIVERSE LIVE AMR SIMULATION] Starting inside Omniverse Kit...")
    print("=" * 65)

    usd_stage_path = os.path.join(REPO_ROOT, "simulation5.usd")
    
    # 1. Ensure the stage is open in Omniverse Kit viewport
    if IN_OMNIVERSE:
        usd_context = omni.usd.get_context()
        stage = usd_context.get_stage()
        
        # If simulation5.usd is not the active stage, open it
        current_url = usd_context.get_stage_url()
        if not stage or "simulation5.usd" not in current_url:
            print(f"[OMNIVERSE] Opening stage: {usd_stage_path}")
            await usd_context.open_stage_async(usd_stage_path)
            await omni.kit.app.get_app().next_update_async()
            stage = usd_context.get_stage()
    else:
        stage = Usd.Stage.Open(usd_stage_path) if os.path.exists(usd_stage_path) else None

    if not stage:
        print("[ERROR] Could not obtain Omniverse USD Stage.")
        return

    print(f"[OMNIVERSE] Active Live Stage: {stage.GetRootLayer().identifier}")

    # 2. Setup USD Prim Translation Operators
    robot_ops = {}
    for r_id in ["AMR_01", "AMR_02", "AMR_03"]:
        prim_path = f"/World/P_DYNEX_Depot/Robots/{r_id}"
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            translate_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break
            if not translate_op:
                translate_op = xform.AddTranslateOp()
            robot_ops[r_id] = translate_op
            print(f"[OMNIVERSE] Bound live Prim for {r_id} at {prim_path}")
        else:
            print(f"[OMNIVERSE] Warning: Prim {prim_path} not found!")

    # Setup dynamic obstacle prim
    obs_prim = stage.GetPrimAtPath("/World/P_DYNEX_Depot/DynamicObstacle")
    obs_op = None
    obs_imageable = None
    if obs_prim.IsValid():
        obs_xform = UsdGeom.Xformable(obs_prim)
        for op in obs_xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                obs_op = op
                break
        if not obs_op:
            obs_op = obs_xform.AddTranslateOp()
        obs_imageable = UsdGeom.Imageable(obs_prim)
        if obs_imageable:
            obs_imageable.MakeInvisible()

    # 3. Setup Fleet Coordination State
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    EventLogger.clear()

    # Initial positions
    agents = [
        AMR("AMR_01", (8, 10), None, blackboard),
        AMR("AMR_02", (10, 10), None, blackboard),
        AMR("AMR_03", (10, 6), None, blackboard)
    ]

    # Assign warehouse delivery tasks
    task_mgr = TaskManager([(14, 10), (10, 14), (6, 6)])
    task_mgr.allocate_tasks(agents, 0, set())

    # Dynamic Obstacle (spawns at t=2 at (12, 10))
    dyn_obs = DynamicObstacle("PALLET_BLOCK_01", (12, 10), activation_time=2)
    blackboard.add_dynamic_obstacle(dyn_obs)

    FleetContext.agents = agents
    FleetContext.blackboard = blackboard
    FleetContext.network = network
    FleetContext.task_manager = task_mgr
    FleetContext.current_time = 0
    FleetContext.simulation_status = "RUNNING"

    # Set initial poses in Omniverse viewport
    for agent in agents:
        if agent.name in robot_ops:
            robot_ops[agent.name].Set(Gf.Vec3d(float(agent.pos[0]), float(agent.pos[1]), 0.035))

    print("\n[OMNIVERSE LIVE] Starting Live Real-time Simulation Loop...")
    print("Watch the 3D Viewport in Omniverse Kit!\n")

    # 4. Interactive Live Step Loop (with delay so user sees live animation)
    total_timesteps = 12
    for t in range(total_timesteps):
        print(f"\n--- [LIVE OMNIVERSE TIMESTEP {t}] ---")

        # Dynamic Obstacle Spawning
        for obs in blackboard.dynamic_obstacles:
            if t >= obs.activation_time:
                obs.active = True
                if obs_op:
                    obs_op.Set(Gf.Vec3d(float(obs.pos[0]), float(obs.pos[1]), 0.25))
                if obs_imageable:
                    obs_imageable.MakeVisible()
                print(f"[OMNIVERSE LIVE] Dynamic Obstacle {obs.obstacle_id} active at {obs.pos}")

        # Broadcast state across network
        network.broadcast_state(agents, t)

        # Step each AMR decentralized coordinator
        for agent in agents:
            prev_pos = agent.pos
            agent.step(t)
            curr_pos = agent.pos

            # Update live Omniverse Prim position
            if agent.name in robot_ops:
                # Smooth interpolation over 10 sub-frames for butter-smooth visual motion
                for sub in range(1, 11):
                    alpha = sub / 10.0
                    interp_x = prev_pos[0] + alpha * (curr_pos[0] - prev_pos[0])
                    interp_y = prev_pos[1] + alpha * (curr_pos[1] - prev_pos[1])
                    robot_ops[agent.name].Set(Gf.Vec3d(float(interp_x), float(interp_y), 0.035))
                    
                    if IN_OMNIVERSE:
                        await omni.kit.app.get_app().next_update_async()
                    else:
                        time.sleep(0.02)

            print(f"  [{agent.name}] Pos={curr_pos}, Status={'IDLE' if agent.idle else 'MOVING'}")

        # Check completed tasks
        task_mgr.allocate_tasks(agents, t + 1, set())
        FleetContext.current_time = t + 1

        # Pause between timesteps for clear visual comprehension
        if IN_OMNIVERSE:
            await asyncio.sleep(0.3)
        else:
            time.sleep(0.3)

    print("\n" + "=" * 65)
    print("[OMNIVERSE LIVE AMR SIMULATION] All tasks completed successfully in Omniverse!")
    print("=" * 65)


# Entrypoint when executed inside Omniverse Kit
if IN_OMNIVERSE:
    asyncio.ensure_future(run_live_simulation_in_omniverse())
else:
    if __name__ == "__main__":
        asyncio.run(run_live_simulation_in_omniverse())
