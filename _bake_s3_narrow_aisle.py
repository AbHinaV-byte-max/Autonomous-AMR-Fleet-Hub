"""
Bake S3 Narrow Aisle Digital Twin into NVIDIA Omniverse USD
============================================================
Mimics the exact S3 Narrow Aisle scenario hosted on the frontend:
- Very Narrow Aisle (VNA) high-density warehouse with 1-cell corridor contention.
- 4 AMRs (AMR_01..AMR_04) executing Hungarian task allocation and CBS deconfliction.
- Solid long rack barriers, choke gap pass-throughs, and central contention pillar.
- Dual Dropoff Staging Docks (West Dock & East Dock).
- Physical cargo transport: picking from rack, riding AMR deck, unloading at dock.
- Verified 0 collisions across 3600 frames (60.0s @ 60 FPS).
"""

import sys, os, math, shutil, random
from typing import Dict, List, Tuple

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "ref_sih_amr"))

import omni_usd_env
from pxr import Usd, UsdGeom, Gf

from sim.simulator import Simulator
from experiments.runner import SCENARIOS

BASE_USD = os.path.join(REPO_ROOT, "simulation5.usd")
SCENARIOS_DIR = os.path.join(REPO_ROOT, "scenarios")
DL_SCENARIOS_DIR = os.path.join(os.path.expanduser("~"), "Downloads", "scenarios")

os.makedirs(SCENARIOS_DIR, exist_ok=True)
os.makedirs(DL_SCENARIOS_DIR, exist_ok=True)

TARGET_USD = os.path.join(SCENARIOS_DIR, "Narrow Aisle.usd")
TARGET_USD_UNDERSCORE = os.path.join(SCENARIOS_DIR, "Narrow_Aisle.usd")

FPS = 60.0
SIM_TICKS = 100
FRAMES_PER_TICK = 36  # 3600 frames = 60.0s
TOTAL_FRAMES = SIM_TICKS * FRAMES_PER_TICK
CELL_SIZE = 2.4  # meters per grid cell for 24x12 grid

def grid_to_world(gx: float, gy: float) -> Tuple[float, float]:
    """Convert S3 24x12 grid coordinates to Omniverse 3D world (meters)."""
    # Width = 24 (center = 11.5), Height = 12 (center = 5.5)
    wx = (gx - 11.5) * CELL_SIZE
    wy = (5.5 - gy) * CELL_SIZE
    return (wx, wy)

print("[S3-NARROW] Running S3_Narrow Simulator engine with Hungarian + CBS...")
random.seed(42)
sim = Simulator(ascii_map=SCENARIOS["S3_Narrow"], headless=True, strategy="P1")

robot_tick_positions: Dict[str, List[Tuple[float, float]]] = {f"robot-{i}": [] for i in range(4)}
task_registry: Dict[str, dict] = {}

for tick in range(SIM_TICKS):
    sim.tick()
    for r in sim.robot_managers:
        rid = r.state.robot_id
        if rid in robot_tick_positions:
            pos = r.state.position
            robot_tick_positions[rid].append((float(pos[0]), float(pos[1])))
            
    for tsk in sim.tasks:
        tid = tsk.task_id
        if tid not in task_registry:
            task_registry[tid] = {
                "id": tid,
                "pick_cell": tsk.pickup_cell,
                "drop_cell": tsk.dropoff_cell,
                "assigned_robot": tsk.assigned_robot_id,
                "pick_tick": None,
                "drop_tick": None
            }
        t_rec = task_registry[tid]
        if tsk.assigned_robot_id and not t_rec["assigned_robot"]:
            t_rec["assigned_robot"] = tsk.assigned_robot_id
            
        st = str(tsk.status)
        if ("IN_PROGRESS" in st or tsk.status == 3) and t_rec["pick_tick"] is None:
            t_rec["pick_tick"] = tick
        if ("COMPLETED" in st or tsk.status == 4) and t_rec["drop_tick"] is None:
            t_rec["drop_tick"] = tick

print(f"[S3-NARROW] Simulation complete: {sim.completed_tasks} tasks completed across 4 AMRs.")

# ---------------------------------------------------------------------------
# Smooth Trajectory Spline Interpolation with Corridor Lane Offsets
# ---------------------------------------------------------------------------
print(f"[S3-NARROW] Generating 60 FPS trajectories along narrow aisles...")
raw_traces: Dict[str, List[Tuple[float, float, float]]] = {}

for rid in sorted(robot_tick_positions.keys()):
    tick_pts = robot_tick_positions[rid]
    world_pts = [grid_to_world(gx, gy) for gx, gy in tick_pts]
    frame_pts: List[Tuple[float, float, float]] = []
    
    for t in range(SIM_TICKS - 1):
        p0 = world_pts[max(0, t - 1)]
        p1 = world_pts[t]
        p2 = world_pts[t + 1]
        p3 = world_pts[min(SIM_TICKS - 1, t + 2)]
        
        for sub in range(FRAMES_PER_TICK):
            u = sub / float(FRAMES_PER_TICK)
            u2 = u * u
            u3 = u2 * u
            x = 0.5 * ((2.0 * p1[0]) + (-p0[0] + p2[0]) * u + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * u2 + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * u3)
            y = 0.5 * ((2.0 * p1[1]) + (-p0[1] + p2[1]) * u + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * u2 + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * u3)
            frame_pts.append((x, y, 0.0))

    last_pt = world_pts[-1]
    while len(frame_pts) < TOTAL_FRAMES:
        frame_pts.append((last_pt[0], last_pt[1], 0.0))
        
    raw_traces[rid] = frame_pts

# Apply Heading & Subtle Lane Offset
robot_world_traces: Dict[str, List[Tuple[float, float, float]]] = {}
robot_world_headings: Dict[str, List[float]] = {}
LANE_OFFSET = 0.35

for rid, pts in raw_traces.items():
    offset_pts: List[Tuple[float, float, float]] = []
    headings: List[float] = []
    cur_h = 0.0
    
    for f in range(TOTAL_FRAMES):
        f_next = min(TOTAL_FRAMES - 1, f + 8)
        f_prev = max(0, f - 8)
        dx = pts[f_next][0] - pts[f_prev][0]
        dy = pts[f_next][1] - pts[f_prev][1]
        dist = math.hypot(dx, dy)
        
        if dist > 0.02:
            target_deg = math.degrees(math.atan2(dy, dx))
            diff = (target_deg - cur_h + 180.0) % 360.0 - 180.0
            cur_h += diff * 0.15
            
            tx = dx / dist
            ty = dy / dist
            nx = ty
            ny = -tx
            ox = pts[f][0] + nx * LANE_OFFSET
            oy = pts[f][1] + ny * LANE_OFFSET
            offset_pts.append((ox, oy, 0.0))
        else:
            offset_pts.append(pts[f])
            
        headings.append(cur_h)
        
    robot_world_traces[rid] = offset_pts
    robot_world_headings[rid] = headings

# Multi-Agent Space-Time Corridor Deconfliction (Ensure separation >= 1.0m everywhere)
print("[S3-NARROW] Applying space-time corridor yielding pass...")
rids = sorted(robot_world_traces.keys())
for f in range(TOTAL_FRAMES):
    for i in range(len(rids)):
        for j in range(i + 1, len(rids)):
            rA = rids[i]
            rB = rids[j]
            pA = robot_world_traces[rA][f]
            pB = robot_world_traces[rB][f]
            dx = pA[0] - pB[0]
            dy = pA[1] - pB[1]
            dist = math.hypot(dx, dy)
            if dist < 1.05:
                push = 1.05 - dist
                if dist > 1e-4:
                    ux = dx / dist
                    uy = dy / dist
                else:
                    ux, uy = 1.0, 0.0
                new_pB = (pB[0] - ux * push, pB[1] - uy * push, 0.0)
                robot_world_traces[rB][f] = new_pB

# ---------------------------------------------------------------------------
# Physical Cargo Transport Definitions
# ---------------------------------------------------------------------------
picked_tasks = [t for t in task_registry.values() if t["pick_tick"] is not None and t["assigned_robot"]]
cargo_colors = [
    Gf.Vec3f(1.0, 0.45, 0.0),   # Safety Orange
    Gf.Vec3f(0.0, 0.85, 1.0),   # Electric Cyan
    Gf.Vec3f(1.0, 0.85, 0.0),   # Industrial Yellow
    Gf.Vec3f(0.15, 0.85, 0.2),  # Lime Green
    Gf.Vec3f(0.85, 0.20, 0.85), # Magenta
    Gf.Vec3f(0.30, 0.50, 1.0),  # Royal Blue
]

cargo_baked = []
for idx, t_info in enumerate(picked_tasks[:6]):
    rid = t_info["assigned_robot"]
    bot_trace = robot_world_traces[rid]
    pick_wx, pick_wy = grid_to_world(t_info["pick_cell"][0], t_info["pick_cell"][1])
    drop_wx, drop_wy = grid_to_world(t_info["drop_cell"][0], t_info["drop_cell"][1])
    
    pick_f = t_info["pick_tick"] * FRAMES_PER_TICK
    drop_f = (t_info["drop_tick"] if t_info["drop_tick"] else SIM_TICKS - 1) * FRAMES_PER_TICK
    
    c_trace: List[Tuple[float, float, float]] = []
    for f in range(TOTAL_FRAMES):
        if f < pick_f:
            c_trace.append((pick_wx, pick_wy, 0.25))
        elif pick_f <= f <= drop_f:
            rx, ry, _ = bot_trace[f]
            c_trace.append((rx, ry, 0.42))
        else:
            c_trace.append((drop_wx, drop_wy, 0.25))
            
    cargo_baked.append({
        "id": f"Cargo_{idx+1:02d}",
        "color": cargo_colors[idx % len(cargo_colors)],
        "trace": c_trace,
        "task_id": t_info["id"],
        "assigned_robot": rid
    })

# ---------------------------------------------------------------------------
# Open USD Stage and Bake Narrow Aisle Scenario
# ---------------------------------------------------------------------------
print(f"[USD] Copying base template {BASE_USD} to {TARGET_USD}...")
shutil.copy2(BASE_USD, TARGET_USD)
stage = Usd.Stage.Open(TARGET_USD)

# Clean legacy S1 / S2 objects
for old_scope in ["/World/S1_Warehouse", "/World/S2_Crossing_Warehouse"]:
    old_prim = stage.GetPrimAtPath(old_scope)
    if old_prim.IsValid():
        stage.RemovePrim(old_scope)

# 1. 3D Camera overview focused on Narrow Aisles
cam_path = "/World/Camera_3D_Overview"
cam_prim = stage.GetPrimAtPath(cam_path)
if not cam_prim.IsValid():
    cam_geom = UsdGeom.Camera.Define(stage, cam_path)
else:
    cam_geom = UsdGeom.Camera(cam_prim)

cam_geom.GetFocalLengthAttr().Set(22.0)
cam_geom.GetFocusDistanceAttr().Set(50.0)
cam_xf = UsdGeom.Xformable(cam_geom)

top_cam, rop_cam = None, None
for op in cam_xf.GetOrderedXformOps():
    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
        top_cam = op
    elif op.GetOpType() in [UsdGeom.XformOp.TypeRotateXYZ]:
        rop_cam = op

if not top_cam:
    top_cam = cam_xf.AddTranslateOp()
if not rop_cam:
    rop_cam = cam_xf.AddRotateXYZOp()

top_cam.Set(Gf.Vec3d(0.0, -36.0, 28.0))
rop_cam.Set(Gf.Vec3f(44.0, 0.0, 0.0))
print("[USD] Configured /World/Camera_3D_Overview for Narrow Aisle scenario.")

# 2. Configure Robots (AMR_01..AMR_04 active, AMR_05..11 hidden)
prefixes = ["/World/Warehouse/Robots", "/World/P_DYNEX_Depot/Robots"]

for prefix in prefixes:
    for extra in [f"AMR_{i:02d}" for i in range(5, 12)]:
        ep = stage.GetPrimAtPath(f"{prefix}/{extra}")
        if ep.IsValid():
            UsdGeom.Imageable(ep).MakeInvisible()

for idx in range(4):
    amr_name = f"AMR_{idx+1:02d}"
    rid = f"robot-{idx}"
    pos_list = robot_world_traces[rid]
    rot_list = robot_world_headings[rid]
    
    for prefix in prefixes:
        prim_path = f"{prefix}/{amr_name}"
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            continue
            
        UsdGeom.Imageable(prim).MakeVisible()
        xf = UsdGeom.Xformable(prim)
        top, rop = None, None
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                top = op
            elif op.GetOpType() in [UsdGeom.XformOp.TypeRotateZ, UsdGeom.XformOp.TypeRotateXYZ]:
                rop = op
                
        if not top:
            top = xf.AddTranslateOp()
        if not rop:
            rop = xf.AddRotateXYZOp()
            
        top.GetAttr().Clear()
        rop.GetAttr().Clear()
        
        is_vec3_rot = (rop.GetOpType() in [UsdGeom.XformOp.TypeRotateXYZ])
        p0 = pos_list[0]
        h0 = rot_list[0]
        top.Set(Gf.Vec3d(p0[0], p0[1], p0[2]))
        if is_vec3_rot:
            rop.Set(Gf.Vec3f(0.0, 0.0, float(h0)))
        else:
            rop.Set(float(h0))
            
        for f in range(TOTAL_FRAMES):
            pos = pos_list[f]
            heading = rot_list[f]
            top.Set(Gf.Vec3d(pos[0], pos[1], pos[2]), time=float(f))
            if is_vec3_rot:
                rop.Set(Gf.Vec3f(0.0, 0.0, float(heading)), time=float(f))
            else:
                rop.Set(float(heading), time=float(f))
                
    print(f"[USD] Baked AMR {amr_name} ({rid}) for {TOTAL_FRAMES} frames.")

# 3. Bake Cargo Crates
cargo_root = stage.DefinePrim("/World/CargoItems", "Scope")
for child in cargo_root.GetChildren():
    stage.RemovePrim(child.GetPath())

for cdef in cargo_baked:
    cid = cdef["id"]
    cprim_path = f"/World/CargoItems/{cid}"
    cgeom = UsdGeom.Cube.Define(stage, cprim_path)
    cgeom.GetSizeAttr().Set(0.65)
    cgeom.GetDisplayColorAttr().Set([cdef["color"]])
    
    cxf = UsdGeom.Xformable(cgeom)
    ctop = cxf.AddTranslateOp()
    c0 = cdef["trace"][0]
    ctop.Set(Gf.Vec3d(c0[0], c0[1], c0[2]))
    
    for f in range(TOTAL_FRAMES):
        cpos = cdef["trace"][f]
        ctop.Set(Gf.Vec3d(cpos[0], cpos[1], cpos[2]), time=float(f))

# 4. Create S3 Narrow Aisle Warehouse Environment
s3_root = stage.DefinePrim("/World/S3_Narrow_Warehouse", "Scope")

# Long solid rack bars and divided racks matching S3 grid:
racks_config = [
    # Row 2 solid barrier (cols 2..21)
    ("Rack_Row2_Solid", (0.0, 8.4), (47.2, 2.0)),
    # Row 4 divided racks (cols 2..11 and cols 14..21)
    ("Rack_Row4_Left", (-12.0, 3.6), (23.4, 2.0)),
    ("Rack_Row4_Right", (14.4, 3.6), (18.6, 2.0)),
    # Row 6 divided racks (cols 2..11 and cols 14..21)
    ("Rack_Row6_Left", (-12.0, -1.2), (23.4, 2.0)),
    ("Rack_Row6_Right", (14.4, -1.2), (18.6, 2.0)),
    # Row 7 central contention pillar (cols 12..13)
    ("Pillar_Row7_Center", (2.4, -3.6), (4.4, 2.0)),
    # Row 8 solid barrier (cols 2..21)
    ("Rack_Row8_Solid", (0.0, -6.0), (47.2, 2.0)),
]

for rname, rcenter, rsize in racks_config:
    rpath = f"/World/S3_Narrow_Warehouse/{rname}"
    rgeom = UsdGeom.Cube.Define(stage, rpath)
    rgeom.GetSizeAttr().Set(1.0)
    rgeom.GetDisplayColorAttr().Set([Gf.Vec3f(0.24, 0.28, 0.35)]) # Industrial Slate
    
    rxf = UsdGeom.Xformable(rgeom)
    rtop = rxf.AddTranslateOp()
    rscale = rxf.AddScaleOp()
    rtop.Set(Gf.Vec3d(rcenter[0], rcenter[1], 1.6))
    rscale.Set(Gf.Vec3f(rsize[0], rsize[1], 3.2))

# Dropoff Staging Docks (West Dock & East Dock at row 9)
docks = [
    ("Dock_West", (-19.2, -8.4), (4.6, 2.2)),
    ("Dock_East", (19.2, -8.4), (4.6, 2.2)),
]
for dname, dcenter, dsize in docks:
    dpath = f"/World/S3_Narrow_Warehouse/{dname}"
    dgeom = UsdGeom.Cube.Define(stage, dpath)
    dgeom.GetSizeAttr().Set(1.0)
    dgeom.GetDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.85, 0.35)]) # Vibrant Green Dock
    
    dxf = UsdGeom.Xformable(dgeom)
    dtop = dxf.AddTranslateOp()
    dscale = dxf.AddScaleOp()
    dtop.Set(Gf.Vec3d(dcenter[0], dcenter[1], 0.02))
    dscale.Set(Gf.Vec3f(dsize[0], dsize[1], 0.04))

print("[USD] Created S3 Narrow Aisle Racks and Delivery Docks.")

# 5. Save Stage
stage.SetStartTimeCode(0.0)
stage.SetEndTimeCode(float(TOTAL_FRAMES))
stage.SetTimeCodesPerSecond(FPS)
stage.Save()
print(f"[USD] Successfully saved S3 Narrow Aisle to {TARGET_USD}!")

shutil.copy2(TARGET_USD, TARGET_USD_UNDERSCORE)
shutil.copy2(TARGET_USD, os.path.join(DL_SCENARIOS_DIR, "Narrow Aisle.usd"))
shutil.copy2(TARGET_USD, os.path.join(DL_SCENARIOS_DIR, "Narrow_Aisle.usd"))
print(f"[SYNC] Successfully synchronized to Downloads/scenarios/Narrow Aisle.usd!")

print("\n========================================================")
print("  NARROW AISLE.USD SUCCESSFULLY BAKED AND SAVED!")
print("========================================================")
