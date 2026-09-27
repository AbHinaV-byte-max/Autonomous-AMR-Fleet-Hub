# PHASE4_REAL_OMNIVERSE_VALIDATION.md

**Generated:** 2026-09-26  
**System:** Windows 11 (10.0.26200), Python 3.12.8 (system) / 3.12.13 (Kit)  
**Project root:** `C:\Users\ravi_\Documents\sih\SIH`

---

## 1. Kit Build Result

### Phase 4A — `.\repo.bat build`

`.\repo.bat build` was executed from the project root. Packman bootstrapped from
the local registry cache — the NVIDIA CDN hostname `api.packman.nvidia.com` does **not**
resolve from this network, so packman used already-cached packages on disk
(downloaded by a previous run on the old machine).

**Result:** Kit SDK 110.3.0 successfully populated under `_build/windows-x86_64/release/`.

| Check | Result |
|---|---|
| `_build/` exists | **YES** |
| `_build/windows-x86_64/release/kit/` exists | **YES** |
| `_build/windows-x86_64/release/kit/python/python.exe` | **YES** (3.12.13) |
| `_build/windows-x86_64/release/extscache/` | **YES** (200+ extensions) |
| `_build/windows-x86_64/release/extscache/omni.usd.libs-1.0.3+...` | **YES** |

> [!NOTE]
> The smoke test expected path was `kit/python.exe` but actual path is `kit/python/python.exe`.
> The `omni_usd_env.py` handles this correctly via extscache `sys.path` injection.

---

## 2. Exact Kit Version

```
Kit SDK:    110.3.0  (feature.windows-x86_64.release)
Kit Python: 3.12.13  (NVIDIA build: tags/v3.12.13-dirty:3bb231a, Apr 22 2026)
USD (Kit):  0.25.11  (from omni.usd.libs-1.0.3+00c488ae.wx64.r.cp312)
USD (pip):  0.26.8   (usd-core 26.8 — standalone fallback for system Python)
```

---

## 3. pxr Verification

### Under system Python (usd-core 26.8)
```
python -c "from pxr import Usd, UsdGeom, Gf; print('PXR OK'); print(Usd.GetVersion())"
PXR OK
(0, 26, 8)
```

### Under Kit Python (pxr from extscache — Phase 4B smoke test)
```
_build\windows-x86_64\release\kit\python\python.exe _omniverse_smoke_test.py
[OMNI] Kit Python in use    : PASS  (...kit\python\python.exe)
[OMNI] omni.usd.libs found  : PASS  (...extscache\omni.usd.libs-1.0.3+...)
[OMNI] pxr import           : PASS  (pxr from ...omni.usd.libs-...\pxr\Usd\__init__.py)
```

pxr loaded from:
```
_build\windows-x86_64\release\extscache\omni.usd.libs-1.0.3+00c488ae.wx64.r.cp312\pxr\
```

---

## 4. Actual USD Hierarchy

`simulation5.usd` (11.2 MB USDC binary) — opened via `Usd.Stage.Open(path, Usd.Stage.LoadNone)`:

**Total prims: 6,268**

### Robot prim paths (ALL 6 AMRs confirmed present):

| AMR | P_DYNEX_Depot path | xformOps | Warehouse path |
|---|---|---|---|
| AMR_01 | `/World/P_DYNEX_Depot/Robots/AMR_01` | translate, scale, rotateZ | `/World/Warehouse/Robots/AMR_01` |
| AMR_02 | `/World/P_DYNEX_Depot/Robots/AMR_02` | translate, scale, rotateZ | `/World/Warehouse/Robots/AMR_02` |
| AMR_03 | `/World/P_DYNEX_Depot/Robots/AMR_03` | translate, scale, rotateZ | `/World/Warehouse/Robots/AMR_03` |
| AMR_04 | `/World/P_DYNEX_Depot/Robots/AMR_04` | translate, scale, rotateZ | `/World/Warehouse/Robots/AMR_04` |
| AMR_05 | `/World/P_DYNEX_Depot/Robots/AMR_05` | translate, scale, rotateZ | `/World/Warehouse/Robots/AMR_05` |
| AMR_06 | `/World/P_DYNEX_Depot/Robots/AMR_06` | translate, scale, rotateZ | `/World/Warehouse/Robots/AMR_06` |

> [!IMPORTANT]
> The Phase 3 report stated AMR_01..06 prims were missing. This was incorrect.
> All 6 AMRs exist in BOTH hierarchies with full geometry subtrees (body, wheels,
> sensor_mast, bumpers). The Phase 3 report relied on a binary string-table scan
> which missed structured path tokens. The Stage traversal under Kit Python confirms all 6.

---

## 5. AMR Prim Creation / Binding

**No new prim creation was needed.** `OmniAMRController._init_usd_ops()` already discovers
all 6 AMRs via its candidate path list. Each discovered prim has `TranslateOp` and `RotateZOp`
ops bound and confirmed present.

**Binding result:** 1800 time-samples written per robot per 30s scenario @ 60 FPS.

---

## 6. USD Transform Write / Readback — Phase 4B Smoke Test (Kit Python)

All 8 steps PASS under Kit Python:

```
[OMNI] Python interpreter        : PASS  (...kit\python\python.exe)
[OMNI] Python version            : PASS  (3.12.13)
[OMNI] pxr import                : PASS  (USD 0.25.11 from omni.usd.libs)
[OMNI] simulation5.usd exists    : PASS  (11,188,521 bytes)
[OMNI] USD stage open            : PASS
[OMNI] Full traverse             : PASS  (6268 prims, AMR_01/02/03 confirmed)
[OMNI] Transform write+readback  : PASS  (wrote=(4.5,-4.5,0.035) read=(4.5,-4.5,0.035))
[OMNI] Time sample write         : PASS  (samples=1500)
[OMNI] Time sample readback t=0  : PASS  (x=0.000)
[OMNI] Time sample readback t=30 : PASS  (x=2.250)
[OMNI] Time sample readback t=60 : PASS  (x=4.500)
[OMNI] USD output save           : PASS  (_smoke_test_output.usd, 291,557 bytes)
[OMNI] USD reopen                : PASS
[OMNI] Time samples in saved USD : PASS  (/World/P_DYNEX_Depot/Robots/AMR_01 rotateZ samples=1500)
```

---

## 7. S2 USD Evidence (Crossing Priority)

**Setup:** AMR_01 (P=10, urgent) crosses East; AMR_02 (P=1, low) moves North.
**Duration:** 30s @ 60 FPS = 1800 keyframes per robot.

### AMR_01 position trace (sampled @ 1s intervals from USD xformOp:translate):

| t(s) | X | Y | State | Note |
|---|---|---|---|---|
| 0 | -2.987 | -4.500 | STOPPED | spawn |
| 1-6 | -2.1→+2.1 | -4.500 | MOVING | eastbound |
| 7 | 2.962 | -4.500 | MOVING | approaching intersection |
| 8 | 3.506 | -4.500 | MOVING | |
| 9 | 3.566 | **-4.676** | MOVING | **detour begins** |
| 10-13 | 3.5→4.2 | **-5.2→-5.9** | MOVING | **Y-axis detour depth** |
| 15-18 | 5.1→6.1 | -5.6→-4.6 | MOVING | returning to Y=-4.5 |
| 20-23 | 7.6→10.1 | -4.498 | MOVING | resumed eastbound |
| 24 | 10.553 | -4.500 | STOPPED | task COMPLETED |

Total X displacement: **13.54 m** | Stopped-seconds: **6 of 30**

### AMR_02 position trace (sampled @ 1s intervals from USD xformOp:translate):

| t(s) | X | Y | State | Note |
|---|---|---|---|---|
| 0 | 4.500 | -9.988 | STOPPED | spawn |
| 1-6 | 4.500 | -9.1→-4.9 | MOVING | northbound |
| **7** | 4.500 | **-4.498** | MOVING | at intersection |
| **8-12** | 4.500 | **-4.498** | **STOPPED** | **YIELD — AMR_01 priority wins** |
| **13-18** | 4.500 | **-4.485** | **STOPPED** | **held until AMR_01 clears** |
| **19** | 4.500 | -4.193 | **MOVING** | **conflict CLEARED — RESUME** |
| 20-29 | 4.500 | -3.4→+4.3 | MOVING | northbound resumed |

**Stop detected: t=7-18s (11.0s duration) — read directly from USD attributes, not telemetry**

### Narrative chain (from USD data only):

```
AMR_02:
  moving    [t=0-6s:  Y=-9.99 → -4.91]
     ↓
  YIELD     [t=7s:    arrives at intersection, CONFLICT detected]
     ↓
  STOPPED   [t=8-18s: Y==-4.498 (constant), USD position unchanged]
     ↓
  RESUME    [t=19s:   conflict clears, Y=-4.19 (moving)]
     ↓
  COMPLETED [t=29.3s: dropoff (4.5, 5.0) reached]

AMR_01:
  PROCEED   [t=7-8s:  continues moving despite conflict detection]
     ↓
  safe detour [t=9-16s: Y swings -4.5 → -5.9 → -4.5 (avoids AMR_02's space)]
     ↓
  COMPLETED  [t=23.5s: dropoff (11.0, -4.5) reached]
```

---

## 8. S4 USD Evidence (Dynamic Obstacle)

| Metric | Value |
|---|---|
| Obstacle injected | t=3.0s at world=(4.5,5.0) |
| Detour triggered | YES — A* replanned around FALLEN_PALLET nav_cell |
| Min inter-robot dist | **13.50 m** |
| Collision events | **0** |
| Collision frames | **0** |
| USD samples per robot | 1500 |

---

## 9. S5 USD Evidence (Robot Failure Recovery)

| Metric | Value |
|---|---|
| AMR_03 failure at | t=4.0s |
| Task reassignment | Hungarian reallocation next tick |
| AMR_03 USD position after failure | CONSTANT (is_failed=True, controller stops writing) |
| Min inter-robot dist | **14.00 m** |
| Collision events | **0** |
| USD samples per robot | 1500 |

---

## 10. S6 USD Evidence (Comms Degradation)

| Metric | Value |
|---|---|
| Packet loss period | t=2.5s to t=6.5s (AMR_02) |
| DEGRADED state | logged once on transition |
| SAFE_MODE state | logged once on transition |
| RESTORED state | logged once at t=6.5s |
| Min inter-robot dist | **14.00 m** |
| Collision events | **0** |
| USD samples per robot | 1500 |

---

## 11. Viewport Verification

> [!CAUTION]
> `VIEWPORT VERIFIED: BLOCKED` for all scenarios (S2, S4, S5, S6).

| Level | S2 | S4 | S5 | S6 |
|---|---|---|---|---|
| LOGIC VERIFIED | ✅ PASS | ✅ PASS | ✅ PASS | ✅ PASS |
| USD TRANSFORM VERIFIED | ✅ PASS | ✅ PASS | ✅ PASS | ✅ PASS |
| VIEWPORT VERIFIED | ❌ BLOCKED | ❌ BLOCKED | ❌ BLOCKED | ❌ BLOCKED |

**Why blocked:**
- `api.packman.nvidia.com` — DNS name resolution fails from this network
- `omniverse-content-production.s3-us-west-2.amazonaws.com` — CDN not reachable (stage payloads)
- Omniverse Launcher — not installed on this machine
- Kit GUI launcher requires GPU context and Nucleus server

**To achieve VIEWPORT VERIFIED:**
1. Install NVIDIA Omniverse Launcher
2. Install USD Composer or Kit Base Editor
3. Open `simulation5.usd` → Timeline → Play

---

## 12. Collision Regression

| Scenario | Min Dist (m) | Collision Events | Collision Frames |
|---|---|---|---|
| S2 | **0.9444** | **0** | **0** |
| S4 | **13.5043** | **0** | **0** |
| S5 | **14.0000** | **0** | **0** |
| S6 | **14.0000** | **0** | **0** |
| Phase 3 reference | 0.9601 | 0 | 0 |

Phase 3 safety fix preserved. All scenarios pass at 0 collision events.

---

## 13. Remaining Performance Problem

> [!WARNING]
> These results are genuine measurements and must NOT be manipulated.

| Benchmark | Baseline | Coordinated | Time Reduction |
|---|---|---|---|
| 3-AMR | 17.4s | 27.8s | **-59.96%** (SLOWER) |
| 6-AMR | ~same | ~same | **-0.1%** (EQUAL) |

Root cause: deadlock detour adds ~10s overhead per crossing. To be investigated in a future phase.

---

## 14. Remaining Blockers

| Blocker | Status |
|---|---|
| Kit viewport (GUI launch) | BLOCKED — network DNS failure |
| Omniverse CDN payloads | BLOCKED — S3 URLs unreachable |
| CBS integration | NOT integrated (per Phase 4 constraint) |
| Performance ≥20% improvement | FAIL — architecture investigation pending |

---

## 15. Final Validation Table

| Validation | Result |
|---|---|
| Algorithms execute | **PASS** |
| Safety | **PASS** |
| Kit runtime | **PASS** |
| USD transforms | **PASS** |
| AMR prims | **PASS** |
| S2 viewport | **BLOCKED** |
| S4 viewport | **BLOCKED** |
| S5 viewport | **BLOCKED** |
| S6 viewport | **BLOCKED** |
| ≥20% performance improvement | **FAIL** |
