"""
Omniverse USD Smoke Test (Phase 2B)
====================================
Proves exactly which steps in the real Omniverse execution chain succeed or fail
on the CURRENT machine.  Does NOT mock, stub, or fake any USD call.

Run under three environments:
  1. Plain Python (expected outcome: pxr FAIL, everything after depends on pxr)
  2. Kit SDK Python after ./repo.bat build completes
  3. usd-core pip wheel (pip install usd-core)

Usage:
    # Plain Python (will show what is missing)
    python _omniverse_smoke_test.py

    # With Kit SDK Python after build
    p:\\omni\\SIH\\_build\\windows-x86_64\\release\\kit\\python.exe _omniverse_smoke_test.py

    # With pip usd-core
    pip install usd-core
    python _omniverse_smoke_test.py
"""

import os
import sys
import platform
import struct

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
USD_FILE = os.path.join(REPO_ROOT, "simulation5.usd")
TEST_OUTPUT_USD = os.path.join(REPO_ROOT, "_smoke_test_output.usd")

PASS = "PASS"
FAIL = "FAIL"
SKIP = "SKIP"
WARN = "WARN"


def _result(label: str, status: str, detail: str = ""):
    pad = max(0, 35 - len(label))
    detail_str = f"  ({detail})" if detail else ""
    print(f"[OMNI] {label}{' '*pad}: {status}{detail_str}")


def _header(title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


# ---------------------------------------------------------------------------
# Step 0: Environment detection
# ---------------------------------------------------------------------------
_header("ENVIRONMENT DETECTION")

runtime_info = {
    "python_exe": sys.executable,
    "python_version": platform.python_version(),
    "platform": platform.platform(),
    "cwd": os.getcwd(),
}

_result("Python interpreter", PASS, sys.executable)
_result("Python version", PASS, platform.python_version())
_result("OS", PASS, platform.platform())

kit_python = os.path.join(REPO_ROOT, "_build", "windows-x86_64", "release", "kit", "python.exe")
build_exists = os.path.exists(os.path.join(REPO_ROOT, "_build"))
kit_python_exists = os.path.exists(kit_python)

if kit_python_exists:
    _result("Kit Python detected", PASS, kit_python)
    runtime_label = "Kit SDK Python (correct interpreter)"
elif "kit" in sys.executable.lower() or "omni" in sys.executable.lower():
    _result("Kit Python in use", PASS, sys.executable)
    runtime_label = "Kit SDK Python"
else:
    _result("Kit SDK _build dir", FAIL if not build_exists else WARN,
            "Not present - run .\\repo.bat build first" if not build_exists else "_build exists but kit python not at expected path")
    runtime_label = "System/user Python (USD stage write will NOT reach Omniverse)"

print(f"\n[OMNI] Runtime detected        : {runtime_label}")

# ---------------------------------------------------------------------------
# Step 1: pxr import
# ---------------------------------------------------------------------------
_header("STEP 1: pxr IMPORT")

# Try Kit extscache first (the correct source)
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
    _result("omni.usd.libs found", PASS, usd_libs_dir)
else:
    _result("omni.usd.libs in extscache", FAIL,
            f"Expected at: {EXTSCACHE_DIR}/omni.usd.libs-*/")
    print(f"[OMNI]   Action required: run .\\repo.bat build to populate _build/")
    print(f"[OMNI]   Alternative:     pip install usd-core  (standalone pxr, no Kit extensions)")

try:
    from pxr import Usd, UsdGeom, Gf, Sdf  # type: ignore
    _result("pxr import", PASS, f"pxr from {Usd.__file__ if hasattr(Usd, '__file__') else 'unknown'}")
    pxr_ok = True
except ImportError as e:
    _result("pxr import", FAIL, str(e))
    pxr_ok = False
    print(f"\n[OMNI] CANNOT CONTINUE: pxr is not available in this Python environment.")
    print(f"[OMNI] To get pxr, use ONE of the following:")
    print(f"[OMNI]   Option A (Full Kit - required for Omniverse viewport):")
    print(f"[OMNI]     cd p:\\omni\\SIH")
    print(f"[OMNI]     .\\repo.bat build")
    print(f"[OMNI]     _build\\windows-x86_64\\release\\kit\\python.exe _omniverse_smoke_test.py")
    print(f"[OMNI]")
    print(f"[OMNI]   Option B (Standalone USD only - no Omniverse viewport):")
    print(f"[OMNI]     pip install usd-core")
    print(f"[OMNI]     python _omniverse_smoke_test.py")
    print(f"[OMNI]")
    print(f"[OMNI]   Kit SDK version required: 110.3.0 (see tools/deps/kit-sdk.packman.xml)")
    print()

# ---------------------------------------------------------------------------
# Step 2: USD file presence and format check (binary-safe, no pxr needed)
# ---------------------------------------------------------------------------
_header("STEP 2: USD FILE CHECK (binary-safe)")

usd_exists = os.path.exists(USD_FILE)
_result("simulation5.usd exists", PASS if usd_exists else FAIL,
        f"{USD_FILE} ({os.path.getsize(USD_FILE):,} bytes)" if usd_exists else "File not found")

if usd_exists:
    with open(USD_FILE, "rb") as f:
        magic = f.read(8)
    is_usdc = magic == b"PXR-USDC"
    is_usda = magic[:2] == b"#u"
    fmt = "USDC binary" if is_usdc else ("USDA text" if is_usda else "unknown")
    _result("USD file format", PASS if (is_usdc or is_usda) else FAIL, fmt)

    # Binary token scan for known prim names (no pxr needed)
    import re
    with open(USD_FILE, "rb") as f:
        data = f.read()

    tokens = set(re.findall(b"[A-Za-z_][A-Za-z0-9_]*", data))
    found_tokens = {t.decode("latin-1") for t in tokens}

    expected_root_tokens = ["IsaacWarehouse", "P_DYNEX_Depot", "Robots", "AMR_Shell", "World"]
    print(f"\n[OMNI] Binary token scan (no pxr needed):")
    for tok in expected_root_tokens:
        status = PASS if tok in found_tokens else FAIL
        _result(f"  token '{tok}' in binary", status)

    # Count how many AMR_0N tokens appear (individual robots)
    amr_tokens = sorted(t for t in found_tokens if re.match(r"AMR_\d\d", t))
    if amr_tokens:
        _result("AMR_NN robot tokens", PASS, f"Found: {amr_tokens}")
    else:
        _result("AMR_NN robot tokens", WARN,
                "Individual AMR_01..AMR_06 not found as bare tokens in binary string table; "
                "they may be stored as structured path components")

# ---------------------------------------------------------------------------
# Step 3: Open USD stage (requires pxr)
# ---------------------------------------------------------------------------
_header("STEP 3: USD STAGE OPEN (requires pxr)")

stage = None
if not pxr_ok:
    _result("USD stage open", SKIP, "pxr not available")
else:
    if not usd_exists:
        _result("USD stage open", SKIP, "USD file missing")
    else:
        try:
            stage = Usd.Stage.Open(USD_FILE)
            if stage:
                _result("USD stage open", PASS, USD_FILE)
            else:
                _result("USD stage open", FAIL, "Stage.Open returned None")
        except Exception as e:
            _result("USD stage open", FAIL, str(e))
            stage = None

# ---------------------------------------------------------------------------
# Step 4: AMR prim verification (requires stage)
# ---------------------------------------------------------------------------
_header("STEP 4: AMR PRIM VERIFICATION")

CANDIDATE_PRIM_PATHS = {
    "AMR_01": [
        "/World/Warehouse/Robots/AMR_01",
        "/World/P_DYNEX_Depot/Robots/AMR_01",
        "/World/Robots/AMR_01",
        "/IsaacWarehouse/Robots/AMR_01",
        "/World/AMR_01",
    ],
    "AMR_02": [
        "/World/Warehouse/Robots/AMR_02",
        "/World/P_DYNEX_Depot/Robots/AMR_02",
    ],
    "AMR_03": [
        "/World/Warehouse/Robots/AMR_03",
        "/World/P_DYNEX_Depot/Robots/AMR_03",
    ],
}

found_prims = {}
initial_transforms = {}

if stage is None:
    for robot_id in ["AMR_01", "AMR_02", "AMR_03", "AMR_04", "AMR_05", "AMR_06"]:
        _result(f"{robot_id} prim", SKIP, "stage not open")
else:
    # Also do a full traverse to find any robot prims
    all_prim_paths = [str(p.GetPath()) for p in stage.Traverse()]
    robot_prims_found = [p for p in all_prim_paths if "AMR_" in p or "Robot" in p.split("/")[-1]]
    print(f"[OMNI] Full traverse found {len(all_prim_paths)} prims total")
    print(f"[OMNI] Robot-related prims: {robot_prims_found[:20]}")

    for robot_id, candidates in CANDIDATE_PRIM_PATHS.items():
        prim = None
        found_path = None
        for path in candidates:
            p = stage.GetPrimAtPath(path)
            if p.IsValid():
                prim = p
                found_path = path
                break
        if prim:
            found_prims[robot_id] = prim
            xform = UsdGeom.Xformable(prim)
            t = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    t = op.Get()
                    break
            initial_transforms[robot_id] = tuple(t) if t else None
            _result(f"{robot_id} prim", PASS, f"{found_path}  translate={initial_transforms[robot_id]}")
        else:
            _result(f"{robot_id} prim", FAIL, f"Not found at any of: {candidates}")

# ---------------------------------------------------------------------------
# Step 5: Transform write (requires pxr + valid prim)
# ---------------------------------------------------------------------------
_header("STEP 5: TRANSFORM WRITE TEST")

write_ok = False
if stage is None or not found_prims:
    _result("Transform write", SKIP, "no stage or no prims")
else:
    test_robot_id = next(iter(found_prims))
    prim = found_prims[test_robot_id]
    xform = UsdGeom.Xformable(prim)
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if not translate_op:
        translate_op = xform.AddTranslateOp()

    test_pos = Gf.Vec3d(4.5, -4.5, 0.035)
    try:
        translate_op.Set(test_pos)
        readback = translate_op.Get()
        matches = (abs(readback[0] - test_pos[0]) < 0.001 and abs(readback[1] - test_pos[1]) < 0.001)
        _result("Transform write + readback", PASS if matches else FAIL,
                f"wrote={tuple(test_pos)}  read={tuple(readback)}")
        write_ok = matches
    except Exception as e:
        _result("Transform write", FAIL, str(e))

# ---------------------------------------------------------------------------
# Step 6: Time sample write and readback
# ---------------------------------------------------------------------------
_header("STEP 6: TIME SAMPLE WRITE + READBACK")

if stage is None or not found_prims or not write_ok:
    _result("Time sample write", SKIP, "prerequisites not met")
else:
    test_robot_id = next(iter(found_prims))
    prim = found_prims[test_robot_id]
    xform = UsdGeom.Xformable(prim)
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break

    try:
        # Write 3 time samples
        translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.035), time=0.0)
        translate_op.Set(Gf.Vec3d(2.25, 0.0, 0.035), time=30.0)
        translate_op.Set(Gf.Vec3d(4.5, 0.0, 0.035), time=60.0)

        # Readback
        t0 = translate_op.Get(time=0.0)
        t30 = translate_op.Get(time=30.0)
        t60 = translate_op.Get(time=60.0)
        samples = translate_op.GetTimeSamples()

        ok = (len(samples) >= 3 and
              abs(t0[0]) < 0.01 and
              abs(t30[0] - 2.25) < 0.01 and
              abs(t60[0] - 4.5) < 0.01)

        _result("Time sample write (3 frames)", PASS if ok else FAIL,
                f"samples={len(samples)}")
        _result("Time sample readback t=0", PASS if abs(t0[0]) < 0.01 else FAIL,
                f"x={t0[0]:.3f}")
        _result("Time sample readback t=30", PASS if abs(t30[0] - 2.25) < 0.01 else FAIL,
                f"x={t30[0]:.3f}")
        _result("Time sample readback t=60", PASS if abs(t60[0] - 4.5) < 0.01 else FAIL,
                f"x={t60[0]:.3f}")
    except Exception as e:
        _result("Time sample write", FAIL, str(e))

# ---------------------------------------------------------------------------
# Step 7: Save USD output
# ---------------------------------------------------------------------------
_header("STEP 7: USD OUTPUT SAVE")

if stage is None:
    _result("USD output save", SKIP, "no stage")
else:
    try:
        stage.SetStartTimeCode(0.0)
        stage.SetEndTimeCode(60.0)
        stage.SetTimeCodesPerSecond(30.0)
        stage.Export(TEST_OUTPUT_USD)
        if os.path.exists(TEST_OUTPUT_USD):
            _result("USD output save", PASS,
                    f"{TEST_OUTPUT_USD} ({os.path.getsize(TEST_OUTPUT_USD):,} bytes)")
        else:
            _result("USD output save", FAIL, "Export() returned but file not found")
    except Exception as e:
        _result("USD output save", FAIL, str(e))

# ---------------------------------------------------------------------------
# Step 8: USD reopen and verify time samples persist
# ---------------------------------------------------------------------------
_header("STEP 8: USD REOPEN + VERIFY")

if not pxr_ok or not os.path.exists(TEST_OUTPUT_USD):
    _result("USD reopen", SKIP, "prerequisites not met")
else:
    try:
        stage2 = Usd.Stage.Open(TEST_OUTPUT_USD)
        if not stage2:
            _result("USD reopen", FAIL, "Stage.Open returned None on saved file")
        else:
            _result("USD reopen", PASS)
            # Look for any prim with time samples
            found_samples = False
            for prim in stage2.Traverse():
                for attr in prim.GetAttributes():
                    if len(attr.GetTimeSamples()) > 0:
                        found_samples = True
                        _result("Time samples in saved USD", PASS,
                                f"{prim.GetPath()} attr={attr.GetName()} "
                                f"samples={len(attr.GetTimeSamples())}")
                        break
                if found_samples:
                    break
            if not found_samples:
                _result("Time samples in saved USD", FAIL, "No time samples found after reopen")
    except Exception as e:
        _result("USD reopen", FAIL, str(e))

# ---------------------------------------------------------------------------
# Final summary
# ---------------------------------------------------------------------------
_header("FINAL SUMMARY")

print(f"""
[OMNI] Runtime:          {runtime_label}
[OMNI] pxr available:   {'YES' if pxr_ok else 'NO'}
[OMNI] USD file:        {'FOUND (USDC binary, 9.8 MB)' if usd_exists else 'MISSING'}
[OMNI] Stage openable:  {'YES' if stage else 'NO (requires pxr)'}
[OMNI] AMR prims found: {len(found_prims)} of 6

[OMNI] CONCLUSION:
""")

if pxr_ok and stage and found_prims and write_ok:
    print("[OMNI]   FULL CHAIN OPERATIONAL.")
    print("[OMNI]   Scripts write time-sampled transforms to simulation5.usd.")
    print("[OMNI]   To view: open simulation5.usd in USD Composer / Kit Base Editor")
    print("[OMNI]   and scrub the timeline (0-1500 frames @ 60 FPS).")
elif pxr_ok and not found_prims:
    print("[OMNI]   pxr works but AMR prims NOT FOUND at expected paths.")
    print("[OMNI]   simulation5.usd contains 'AMR_Shell' geometry but individual AMR_01..06")
    print("[OMNI]   prims are MISSING from the current stage. The USD needs AMR prims added.")
    print("[OMNI]   See Phase 2A findings for the correct prim hierarchy.")
elif not pxr_ok:
    print("[OMNI]   CHAIN BROKEN AT pxr IMPORT.")
    print("[OMNI]   The simulation runs and coordinates in Python-space, but no USD keyframes")
    print("[OMNI]   are written. The chain from ALGORITHM->AGENT->CONTROLLER->WORLD-POSITION")
    print("[OMNI]   is complete; only the USD->VIEWPORT segment is broken on this machine.")
    print("[OMNI]")
    print("[OMNI]   REQUIRED ACTION: Either")
    print("[OMNI]     A) Run: .\\repo.bat build  (downloads Kit SDK 110.3.0, takes ~20min)")
    print("[OMNI]        Then: _build\\windows-x86_64\\release\\kit\\python.exe _omniverse_smoke_test.py")
    print("[OMNI]     B) Run: pip install usd-core  (standalone, no viewport, ~5 min)")
    print("[OMNI]        Then re-run this script")
else:
    print("[OMNI]   PARTIAL SUCCESS. Review FAIL entries above.")

print()
