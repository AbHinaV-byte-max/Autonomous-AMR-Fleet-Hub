import sys, os
REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, REPO_ROOT)
import omni_usd_env
from pxr import Usd, UsdGeom

USD_PATH = os.path.join(REPO_ROOT, "scenarios", "Blocked Aisle.usd")
stage = Usd.Stage.Open(USD_PATH)

bots = [f"AMR_{i+1:02d}" for i in range(4)]
bot_prims = []
for b in bots:
    p = stage.GetPrimAtPath(f"/World/P_DYNEX_Depot/Robots/{b}")
    if not p.IsValid():
        p = stage.GetPrimAtPath(f"/World/Warehouse/Robots/{b}")
    bot_prims.append((b, p))

print(f"Auditing {len(bot_prims)} AMRs in Blocked Aisle across 3600 frames...")
min_dist = float("inf")
min_pair = ("", "")
min_frame = 0
collision_count = 0

for f in range(0, 3600, 6):
    positions = []
    for name, prim in bot_prims:
        xf = UsdGeom.Xformable(prim)
        mat = xf.ComputeLocalToWorldTransform(float(f))
        trans = mat.ExtractTranslation()
        positions.append((name, trans[0], trans[1]))
        
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            dx = positions[i][1] - positions[j][1]
            dy = positions[i][2] - positions[j][2]
            d = (dx*dx + dy*dy)**0.5
            if d < min_dist:
                min_dist = d
                min_pair = (positions[i][0], positions[j][0])
                min_frame = f
            if d < 0.70:
                collision_count += 1

print(f"\n[S4 AUDIT RESULTS]")
print(f"Total Collisions (<0.70m): {collision_count}")
print(f"Minimum Separation: {min_dist:.4f}m between {min_pair[0]} and {min_pair[1]} at frame {min_frame} (t={min_frame/60.0:.2f}s)")
if collision_count == 0:
    print(">>> 100% COLLISION FREE IN S4 BLOCKED AISLE! <<<")
else:
    print(">>> WARNING: Collisions detected! <<<")
