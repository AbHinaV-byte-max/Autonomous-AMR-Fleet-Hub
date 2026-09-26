import os, glob
from pxr import Usd, UsdGeom, Gf

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
SCENARIOS_DIR = os.path.join(REPO_ROOT, "scenarios")

usd_files = sorted(glob.glob(os.path.join(SCENARIOS_DIR, "*.usd")))
print(f"[CAMERAS] Found {len(usd_files)} scenario USD files to upgrade with free rotation & AMR inspection cameras.")

for usd_path in usd_files:
    fname = os.path.basename(usd_path)
    print(f"\n[UPGRADE] Processing: {fname}...")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"  [ERROR] Could not open {usd_path}")
        continue

    # 1. Add /World/Camera_Free_Orbit (Unlocked Free Orbit & Tumble Camera)
    free_cam_path = "/World/Camera_Free_Orbit"
    free_cam_prim = stage.GetPrimAtPath(free_cam_path)
    if not free_cam_prim.IsValid():
        free_cam = UsdGeom.Camera.Define(stage, free_cam_path)
    else:
        free_cam = UsdGeom.Camera(free_cam_prim)

    free_cam.GetFocalLengthAttr().Set(24.0)
    free_cam.GetFocusDistanceAttr().Set(42.0)
    free_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))
    free_cam.GetHorizontalApertureAttr().Set(20.955)
    free_cam.GetVerticalApertureAttr().Set(15.291)

    free_xf = UsdGeom.Xformable(free_cam)
    free_xf.ClearXformOpOrder()
    t_op = free_xf.AddTranslateOp()
    r_op = free_xf.AddRotateXYZOp()
    t_op.Set(Gf.Vec3d(0.0, -36.0, 26.0))
    r_op.Set(Gf.Vec3f(42.0, 0.0, 0.0))

    # 2. Add /World/Camera_3D_Overview free tumbling unlocks
    overview_cam_prim = stage.GetPrimAtPath("/World/Camera_3D_Overview")
    if overview_cam_prim.IsValid():
        ov_cam = UsdGeom.Camera(overview_cam_prim)
        ov_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 10000.0))

    # 3. Add Dedicated Close-Up Inspection Cameras to Every AMR
    prefixes = ["/World/P_DYNEX_Depot/Robots", "/World/Warehouse/Robots"]
    amr_count = 0

    for prefix in prefixes:
        base_prim = stage.GetPrimAtPath(prefix)
        if not base_prim.IsValid():
            continue

        for child in base_prim.GetChildren():
            cname = child.GetName()
            if not cname.startswith("AMR_"):
                continue

            cam_path = f"{child.GetPath().pathString}/Camera_Inspect"
            cam_prim = stage.GetPrimAtPath(cam_path)
            if not cam_prim.IsValid():
                cam_geom = UsdGeom.Camera.Define(stage, cam_path)
            else:
                cam_geom = UsdGeom.Camera(cam_prim)

            cam_geom.GetFocalLengthAttr().Set(28.0)
            cam_geom.GetFocusDistanceAttr().Set(3.2)
            cam_geom.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
            cam_geom.GetHorizontalApertureAttr().Set(20.955)
            cam_geom.GetVerticalApertureAttr().Set(15.291)

            cam_xf = UsdGeom.Xformable(cam_geom)
            cam_xf.ClearXformOpOrder()
            t_amr = cam_xf.AddTranslateOp()
            r_amr = cam_xf.AddRotateXYZOp()

            # Mount camera slightly behind and above the AMR looking down at chassis and cargo
            t_amr.Set(Gf.Vec3d(0.0, -2.6, 1.7))
            r_amr.Set(Gf.Vec3f(30.0, 0.0, 0.0))
            amr_count += 1

    print(f"  [OK] Added /World/Camera_Free_Orbit + {amr_count} AMR close-up inspection cameras to {fname}.")
    stage.Save()

print("\n[COMPLETE] All 6 scenario USD files successfully updated with free orbit & AMR inspection cameras!")
