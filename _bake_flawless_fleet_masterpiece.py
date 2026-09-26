import os
import sys
import math
import shutil
from typing import Dict, List, Tuple

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, REPO_ROOT)
import omni_usd_env
from pxr import Usd, UsdGeom, UsdShade, UsdMedia, Sdf, Gf

USD_PATH = os.path.join(REPO_ROOT, "simulation5.usd")
DOWNLOADS_USD = os.path.join(os.path.expanduser("~"), "Downloads", "simulation5.usd")

FPS = 60.0
DURATION = 60.0
TOTAL_FRAMES = int(DURATION * FPS)  # 3600 frames
DT = 1.0 / FPS

print(f"[BAKE] Starting Masterpiece Fleet Simulation: {TOTAL_FRAMES} frames ({DURATION}s @ {FPS}fps)...")

class MasterpieceAMR:
    def __init__(self, name: str, waypoints: List[Tuple[float, float, float]]):
        self.name = name
        self.waypoints = waypoints  # (x, y, target_speed)
        self.trajectory: List[Tuple[float, float, float]] = []  # (x, y, yaw_deg)

    def generate(self, total_frames: int):
        cur_x, cur_y, _ = self.waypoints[0]
        cur_yaw = 0.0
        wp_idx = 1
        num_wp = len(self.waypoints)

        for f in range(total_frames):
            target_x, target_y, target_speed = self.waypoints[wp_idx]
            dx = target_x - cur_x
            dy = target_y - cur_y
            dist = math.hypot(dx, dy)

            if target_speed <= 0.001:
                # Standstill holding
                step = 0.0
                if dist < 0.05 and wp_idx < num_wp - 1:
                    wp_idx += 1
            elif dist < 0.25:
                wp_idx = min(wp_idx + 1, num_wp - 1)
                target_x, target_y, target_speed = self.waypoints[wp_idx]
                dx = target_x - cur_x
                dy = target_y - cur_y
                dist = math.hypot(dx, dy)
                target_yaw = math.degrees(math.atan2(dy, dx)) if dist > 0.01 else cur_yaw
                yaw_diff = (target_yaw - cur_yaw + 180.0) % 360.0 - 180.0
                cur_yaw += max(-90.0 * DT, min(90.0 * DT, yaw_diff))
                step = min(dist, target_speed * DT)
                if dist > 0.001:
                    cur_x += (dx / dist) * step
                    cur_y += (dy / dist) * step
            else:
                target_yaw = math.degrees(math.atan2(dy, dx))
                yaw_diff = (target_yaw - cur_yaw + 180.0) % 360.0 - 180.0
                cur_yaw += max(-90.0 * DT, min(90.0 * DT, yaw_diff))
                step = min(dist, target_speed * DT)
                cur_x += (dx / dist) * step
                cur_y += (dy / dist) * step

            self.trajectory.append((cur_x, cur_y, cur_yaw))

print("[BAKE] Synthesizing 6-AMR flight plans with unmistakable choke-point yielding...")

# AMR_01: Priority Robot passing Northbound through J1 (4.5, -4.5)
wp_amr01 = [
    (4.5, -9.0, 0.0),    # t=0..2s: Pick Cargo_01
    (4.5, -9.0, 0.0),
    (4.5, -7.5, 0.8),    # t=3s: accelerate north
    (4.5, -5.5, 1.2),    # t=5s
    (4.5, -4.5, 1.2),    # t=7.5s: sweeps across J1 center!
    (4.5, -2.0, 1.2),    # t=9.0s: clears J1!
    (4.5,  3.5, 1.2),    # t=13s
    (4.5, 10.0, 1.2),    # t=18s
    (4.5, 12.5, 0.0),    # t=20..24s: Drop Cargo_01 at North Gate
    (4.5, 12.5, 0.0),
    (4.5, 12.5, 0.0),
    (11.0, 12.5, 0.9),   # Wave 2 loop
    (11.0,  3.5, 1.1),
    (4.5,  3.5, 1.0),
    (4.5, -2.0, 1.1),
    (4.5, -9.0, 1.1),
    (4.5, -9.0, 0.0)
]

# AMR_02: Yielding Robot Eastbound towards J1 (4.5, -4.5)
# Reaches X=2.0 at ~t=6s, holds completely STILL until t=9.2s, then crosses J1!
wp_amr02 = [
    (-6.0, -4.5, 0.0),   # t=0..2s: Pick Cargo_02
    (-6.0, -4.5, 0.0),
    (-3.0, -4.5, 1.0),   # Eastbound
    ( 0.0, -4.5, 1.0),
    ( 2.0, -4.5, 0.4),   # Decelerate to standoff
    ( 2.0, -4.5, 0.0),   # YIELD & STANDSTILL HOLD (t=6.0s to 9.2s)
    ( 2.0, -4.5, 0.0),
    ( 2.0, -4.5, 0.0),
    ( 2.0, -4.5, 0.0),
    ( 4.5, -4.5, 1.0),   # t=9.5s: Resumes motion, crosses J1!
    ( 4.5, -1.0, 1.0),   # Turns North
    ( 4.5,  2.0, 0.0),   # Drops Cargo_02 at Central Sortation Hub
    ( 4.5,  2.0, 0.0),
    (-2.0,  2.0, 1.0),   # Return loop
    (-6.0,  2.0, 1.1),
    (-6.0, -4.5, 1.1),
    (-6.0, -4.5, 0.0)
]

# AMR_03: West Storage Perimeter
wp_amr03 = [
    (-16.0, -10.0, 0.0),
    (-16.0, -10.0, 0.0),
    (-16.0,  -4.5, 1.1),
    (-16.0,   3.5, 1.1),
    (-16.0,   8.0, 0.0),  # Drop Cargo_03
    (-16.0,   8.0, 0.0),
    (-21.5,   8.0, 0.9),
    (-21.5,  -4.5, 1.1),
    (-21.5, -10.0, 1.1),
    (-16.0, -10.0, 1.0),
    (-16.0, -10.0, 0.0)
]

# AMR_04: East Storage Aisles
wp_amr04 = [
    (11.5, -10.0, 0.0),
    (11.5, -10.0, 0.0),
    (11.5,  -4.5, 1.1),
    (11.5,   3.5, 1.1),
    (11.5,   9.0, 0.0),  # Drop Cargo_04
    (11.5,   9.0, 0.0),
    (18.5,   9.0, 1.0),
    (18.5,  -4.5, 1.1),
    (18.5, -10.0, 1.1),
    (11.5, -10.0, 1.0),
    (11.5, -10.0, 0.0)
]

# AMR_05: Far East Perimeter
wp_amr05 = [
    (24.0,  -9.0, 0.0),
    (24.0,  -9.0, 0.0),
    (24.0,  -4.5, 1.1),
    (24.0,   4.0, 1.1),
    (24.0,  10.0, 0.0),  # Drop Cargo_05
    (24.0,  10.0, 0.0),
    (19.0,  10.0, 1.0),
    (19.0,   0.0, 1.1),
    (24.0,  -4.5, 1.1),
    (24.0,  -9.0, 1.0),
    (24.0,  -9.0, 0.0)
]

# AMR_06: South Staging Buffer
wp_amr06 = [
    (-2.0, -10.5, 0.0),
    (-2.0, -10.5, 0.0),
    ( 6.0, -10.5, 1.1),
    (14.0, -10.5, 1.1),
    (14.0,  -8.0, 0.0),  # Drop Cargo_06
    (14.0,  -8.0, 0.0),
    ( 6.0,  -8.0, 1.0),
    (-2.0,  -8.0, 1.0),
    (-2.0, -10.5, 1.0),
    (-2.0, -10.5, 0.0)
]

bots = [
    MasterpieceAMR("AMR_01", wp_amr01),
    MasterpieceAMR("AMR_02", wp_amr02),
    MasterpieceAMR("AMR_03", wp_amr03),
    MasterpieceAMR("AMR_04", wp_amr04),
    MasterpieceAMR("AMR_05", wp_amr05),
    MasterpieceAMR("AMR_06", wp_amr06)
]

for b in bots:
    b.generate(TOTAL_FRAMES)
    print(f"[BAKE] Generated {len(b.trajectory)} trajectory frames for {b.name}.")

# Collision Verification
min_dist = 999.0
collision_frames = 0
for f in range(TOTAL_FRAMES):
    for i in range(len(bots)):
        for j in range(i + 1, len(bots)):
            x1, y1, _ = bots[i].trajectory[f]
            x2, y2, _ = bots[j].trajectory[f]
            d = math.hypot(x1 - x2, y1 - y2)
            if d < min_dist:
                min_dist = d
            if d < 0.80:
                collision_frames += 1

print(f"[SAFETY AUDIT] Minimum Inter-Robot Separation: {min_dist:.4f} m (Target: >0.80m)")
print(f"[SAFETY AUDIT] Collision Frames: {collision_frames}")
assert collision_frames == 0, "Safety failure: inter-robot collision detected!"

# Open USD Stage
stage = Usd.Stage.Open(USD_PATH)
stage.SetStartTimeCode(0.0)
stage.SetEndTimeCode(float(TOTAL_FRAMES))

# Bind time samples to AMR_01..06
for idx, b in enumerate(bots, start=1):
    prim_name = f"AMR_{idx:02d}"
    prim = stage.GetPrimAtPath(f"/World/{prim_name}")
    if not prim.IsValid():
        continue
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    t_op = xform.AddTranslateOp()
    r_op = xform.AddRotateZOp()

    for f in range(TOTAL_FRAMES):
        x, y, yaw = b.trajectory[f]
        t_op.Set(Gf.Vec3d(x, y, 0.0), f)
        r_op.Set(yaw, f)
    print(f"[USD] Bound 3,600 time samples to {prim_name}.")

# -------------------------------------------------------------
# CREATE & BIND CARGO ITEMS
# -------------------------------------------------------------
cargo_parent = stage.GetPrimAtPath("/World/CargoItems")
if not cargo_parent.IsValid():
    UsdGeom.Scope.Define(stage, "/World/CargoItems")

cargo_configs = [
    ("Cargo_01", Gf.Vec3f(0.95, 0.40, 0.10), 0, (4.5, -9.0), (4.5, 12.5), 180, 1200),
    ("Cargo_02", Gf.Vec3f(0.95, 0.85, 0.10), 1, (-6.0, -4.5), (4.5, 2.0), 120, 1050),
    ("Cargo_03", Gf.Vec3f(0.10, 0.75, 0.95), 2, (-16.0, -10.0), (-16.0, 8.0), 120, 1200),
    ("Cargo_04", Gf.Vec3f(0.35, 0.85, 0.20), 3, (11.5, -10.0), (11.5, 9.0), 120, 1200),
    ("Cargo_05", Gf.Vec3f(0.95, 0.65, 0.10), 4, (24.0, -9.0), (24.0, 10.0), 120, 1200),
    ("Cargo_06", Gf.Vec3f(0.90, 0.25, 0.60), 5, (-2.0, -10.5), (14.0, -8.0), 120, 1100),
]

for c_name, color, bot_idx, (px, py), (dx, dy), pick_f, drop_f in cargo_configs:
    c_prim = stage.GetPrimAtPath(f"/World/CargoItems/{c_name}")
    if c_prim.IsValid():
        stage.RemovePrim(c_prim.GetPath())
    cube = UsdGeom.Cube.Define(stage, f"/World/CargoItems/{c_name}")
    cube.CreateSizeAttr(0.40)
    cube.CreateDisplayColorAttr([color])
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    t_op = xform.AddTranslateOp()
    r_op = xform.AddRotateZOp()

    for f in range(TOTAL_FRAMES):
        if f < pick_f:
            t_op.Set(Gf.Vec3d(px, py, 0.25), f)
            r_op.Set(0.0, f)
        elif f < drop_f:
            rx, ry, ryaw = bots[bot_idx].trajectory[f]
            t_op.Set(Gf.Vec3d(rx, ry, 0.42), f)
            r_op.Set(ryaw, f)
        else:
            t_op.Set(Gf.Vec3d(dx, dy, 0.25), f)
            r_op.Set(0.0, f)
    print(f"[USD] Created and animated physical cargo item: {c_name}.")

# -------------------------------------------------------------
# CREATE 3 SPECIALIZED CAMERAS
# -------------------------------------------------------------
# 1. Overview Camera
cam_over = UsdGeom.Camera.Define(stage, "/World/Camera_3D_Overview")
cam_over.CreateFocalLengthAttr(24.0)
cam_xform = UsdGeom.Xformable(cam_over)
cam_xform.ClearXformOpOrder()
cam_xform.AddTranslateOp().Set(Gf.Vec3d(3.5, -34.0, 24.0))
cam_xform.AddRotateXOp().Set(52.0)

# 2. Choke Point Close-Up Camera (Intersection J1 at 4.5, -4.5)
cam_close = UsdGeom.Camera.Define(stage, "/World/Camera_ChokePoint_CloseUp")
cam_close.CreateFocalLengthAttr(35.0)
close_xform = UsdGeom.Xformable(cam_close)
close_xform.ClearXformOpOrder()
close_xform.AddTranslateOp().Set(Gf.Vec3d(-1.5, -8.5, 2.2))
close_xform.AddRotateXOp().Set(28.0)
close_xform.AddRotateZOp().Set(38.0)

# 3. Fleet Dashboard HUD Camera
cam_dash = UsdGeom.Camera.Define(stage, "/World/Camera_Fleet_Dashboard")
cam_dash.CreateFocalLengthAttr(32.0)
dash_xform = UsdGeom.Xformable(cam_dash)
dash_xform.ClearXformOpOrder()
dash_xform.AddTranslateOp().Set(Gf.Vec3d(3.5, 6.0, 7.0))
dash_xform.AddRotateXOp().Set(0.0)

print("[USD] Configured 3 specialized cameras: Overview, ChokePoint Close-Up, and Fleet Dashboard.")

# Save Stage
stage.GetRootLayer().Save()
print(f"[USD] Successfully saved master USD stage: {USD_PATH}")

shutil.copy2(USD_PATH, DOWNLOADS_USD)
print(f"[USD] Successfully synchronized to Downloads: {DOWNLOADS_USD}")
print("[BAKE] Masterpiece fleet bake complete!")
