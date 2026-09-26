import sys, os, shutil
REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, REPO_ROOT)
import omni_usd_env
from pxr import Usd, UsdGeom, Gf

USD_PATH = os.path.join(REPO_ROOT, "simulation5.usd")
DOWNLOADS_USD = os.path.join(os.path.expanduser("~"), "Downloads", "simulation5.usd")

stage = Usd.Stage.Open(USD_PATH)
s1_scope = stage.DefinePrim("/World/S1_Warehouse", "Scope")
racks_scope = stage.DefinePrim("/World/S1_Warehouse/RackClusters", "Scope")

# 4 Column clusters x 5 Row blocks = 20 Rack Clusters
col_centers = [-20.0, -8.0, 8.0, 20.0]
row_centers = [12.0, 6.0, 0.0, -6.0, -12.0]

for c_idx, cx in enumerate(col_centers):
    for r_idx, ry in enumerate(row_centers):
        r_name = f"Rack_C{c_idx+1}_R{r_idx+1}"
        r_path = f"/World/S1_Warehouse/RackClusters/{r_name}"
        prim = stage.GetPrimAtPath(r_path)
        if not prim.IsValid():
            geom = UsdGeom.Cube.Define(stage, r_path)
        else:
            geom = UsdGeom.Cube(prim)
            
        geom.GetSizeAttr().Set(1.0)
        # Industrial steel gray/blue color
        geom.GetDisplayColorAttr().Set([Gf.Vec3f(0.22, 0.26, 0.32)])
        
        xf = UsdGeom.Xformable(geom)
        top, scale = None, None
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                top = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                scale = op
        if not top:
            top = xf.AddTranslateOp()
        if not scale:
            scale = xf.AddScaleOp()
            
        # Rack dimension: Width=7.6m (leaving 0.4m margin), Depth=3.6m, Height=3.2m
        top.Set(Gf.Vec3d(cx, ry, 1.6))
        scale.Set(Gf.Vec3f(7.4, 3.4, 3.2))

print(f"[USD] Generated 20 S1 Rack Clusters matching S1 grid obstacles.")
stage.Save()

if os.path.exists(DOWNLOADS_USD):
    try:
        shutil.copy2(USD_PATH, DOWNLOADS_USD)
        print(f"[SYNC] Synced to {DOWNLOADS_USD}")
    except Exception as e:
        print(f"[SYNC] Error: {e}")
