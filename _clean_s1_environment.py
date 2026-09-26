import sys, os, shutil
REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, REPO_ROOT)
import omni_usd_env
from pxr import Usd, UsdGeom

USD_PATH = os.path.join(REPO_ROOT, "simulation5.usd")
DOWNLOADS_USD = os.path.join(os.path.expanduser("~"), "Downloads", "simulation5.usd")

stage = Usd.Stage.Open(USD_PATH)
old_racks = stage.GetPrimAtPath("/World/P_DYNEX_Depot/RackClusters")
if old_racks.IsValid():
    UsdGeom.Imageable(old_racks).MakeInvisible()
    print("[USD] Hidden old legacy RackClusters to display pure S1 layout.")

stage.Save()

if os.path.exists(DOWNLOADS_USD):
    shutil.copy2(USD_PATH, DOWNLOADS_USD)
    print(f"[SYNC] Synced to {DOWNLOADS_USD}")
