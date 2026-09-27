import os
import sys

# Configure Omniverse USD Environment
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__)))
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
except ImportError as e:
    Usd = UsdGeom = Gf = None  # type: ignore

def move_amr(usd_path, prim_path, new_translation):
    print(f"Opening stage {usd_path}...")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("Failed to open stage")
        sys.exit(1)
        
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"Prim {prim_path} is invalid")
        sys.exit(1)
        
    print(f"Found prim: {prim.GetName()}")
    
    # Get or create the translate operation
    xform = UsdGeom.Xformable(prim)
    xform_ops = xform.GetOrderedXformOps()
    
    translate_op = None
    for op in xform_ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
            
    if not translate_op:
        translate_op = xform.AddTranslateOp()
        
    print(f"Old translation: {translate_op.Get()}")
    translate_op.Set(new_translation)
    print(f"New translation: {translate_op.Get()}")
    
    stage.Save()
    print("Stage saved.")

if __name__ == "__main__":
    usd_file = r"C:\Users\goruv\Downloads\assets/omniverse/simulation5.usd"
    prim = "/World/P_DYNEX_Depot/Robots/AMR_01"
    # Moving it by +5 in X
    new_pos = Gf.Vec3d(-15.0, 16.0, 0.035)
    move_amr(usd_file, prim, new_pos)
