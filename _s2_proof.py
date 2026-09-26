"""
Phase 2C: S2 Algorithm-to-Position Proof
=========================================
Records per-frame telemetry for S2 and correlates [CONFLICT] events
with actual world-space position and velocity changes.

Produces:
  _s2_telemetry.csv   - frame-by-frame positions + conflict state
  _s2_analysis.txt    - human readable correlation report

The chain being proven:
  PLAN -> CONFLICT -> PRIORITY DECISION -> is_stopped=True
  -> position stops changing -> AMR_01 continues -> conflict clears
  -> AMR_02 resumes -> position changes again
"""

import os
import sys
import csv
import math

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

from run_omni_sih_simulation import OmniSIHSimulationEngine
from omni_scenarios import setup_s2_crossing_priority, _make_s6_callback
from ref_sih_amr.models import RobotStatus

# -------------------------------------------------------------------------
# Run S2 with full per-frame telemetry recording
# -------------------------------------------------------------------------
DURATION = 14.0
FPS      = 60.0
DT       = 1.0 / FPS
FRAMES   = int(DURATION * FPS)

engine = OmniSIHSimulationEngine(cell_size=0.5)
engine.nav_map.print_grid_summary()

print("\n[S2-PROOF] Setting up Scenario 2: Crossing Priority")
setup_s2_crossing_priority(engine)
engine.dt = DT

print("[S2-PROOF] Initial Hungarian allocation...")
engine.run_hungarian_allocation()

# -------------------------------------------------------------------------
# Per-frame telemetry record
# -------------------------------------------------------------------------
rows = []   # list of dicts, one per frame

# Conflict event log (frame, description)
conflict_events = []
path_events = []
prev_stopped = {}    # robot_id -> bool
prev_waiting = {}    # robot_id -> str or None

# Capture stdout for tagged events
import io, re

class CaptureTee:
    """Mirrors stdout and captures lines matching a prefix."""
    def __init__(self):
        self._orig = sys.__stdout__
        self.lines = []
    def write(self, s):
        self._orig.write(s)
        self.lines.extend(s.splitlines())
    def flush(self):
        self._orig.flush()
    def drain(self):
        out = list(self.lines)
        self.lines.clear()
        return out

cap = CaptureTee()
sys.stdout = cap

print(f"[S2-PROOF] Starting simulation: {FRAMES} frames @ {FPS} FPS ({DURATION}s)")

for f in range(FRAMES):
    engine.step(frame=float(f))
    t = engine.current_time

    row = {"frame": f, "t": round(t, 4)}

    for a in engine.agents:
        rid = a.robot_id
        tel = a.controller.get_telemetry()
        pos = tel["actual_position"]
        row[f"{rid}_x"]        = round(pos[0], 4)
        row[f"{rid}_y"]        = round(pos[1], 4)
        row[f"{rid}_v"]        = round(tel["linear_velocity"], 4)
        row[f"{rid}_stopped"]  = int(tel["is_stopped"])
        row[f"{rid}_waiting"]  = a.waiting_on or ""
        row[f"{rid}_status"]   = a.state.status.value

        # Detect stop/start transitions
        cur_stopped = tel["is_stopped"]
        if rid not in prev_stopped:
            prev_stopped[rid] = cur_stopped
        elif prev_stopped[rid] != cur_stopped:
            event_type = "STOPPED" if cur_stopped else "RESUMED"
            conflict_events.append({
                "frame": f, "t": round(t, 4), "robot": rid,
                "event": event_type,
                "pos_x": round(pos[0], 4), "pos_y": round(pos[1], 4),
                "velocity": round(tel["linear_velocity"], 4)
            })
            prev_stopped[rid] = cur_stopped

        # Detect waiting_on transitions
        cur_wait = a.waiting_on
        if rid not in prev_waiting:
            prev_waiting[rid] = cur_wait
        elif prev_waiting[rid] != cur_wait:
            if cur_wait is not None:
                conflict_events.append({
                    "frame": f, "t": round(t, 4), "robot": rid,
                    "event": f"YIELD->waiting_on={cur_wait}",
                    "pos_x": round(pos[0], 4), "pos_y": round(pos[1], 4),
                    "velocity": round(tel["linear_velocity"], 4)
                })
            else:
                conflict_events.append({
                    "frame": f, "t": round(t, 4), "robot": rid,
                    "event": f"CONFLICT_CLEARED (was waiting_on={prev_waiting[rid]})",
                    "pos_x": round(pos[0], 4), "pos_y": round(pos[1], 4),
                    "velocity": round(tel["linear_velocity"], 4)
                })
            prev_waiting[rid] = cur_wait

    rows.append(row)

    # Capture tagged log lines for this frame
    for line in cap.drain():
        if line.startswith("[PATH]") or line.startswith("[CONFLICT]") or "[HUNGARIAN" in line:
            path_events.append({"frame": f, "t": round(t, 4), "line": line})

sys.stdout = sys.__stdout__
# Drain any remaining
for line in cap.lines:
    sys.stdout.write(line + "\n")

# -------------------------------------------------------------------------
# Save CSV
# -------------------------------------------------------------------------
csv_path = os.path.join(REPO_ROOT, "_s2_telemetry.csv")
if rows:
    fieldnames = list(rows[0].keys())
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n[S2-PROOF] Telemetry CSV saved: {csv_path}  ({len(rows)} rows)")

# -------------------------------------------------------------------------
# Produce analysis report
# -------------------------------------------------------------------------
analysis_path = os.path.join(REPO_ROOT, "_s2_analysis.txt")

ROBOT_IDS = ["AMR_01", "AMR_02", "AMR_03", "AMR_04", "AMR_05", "AMR_06"]
# We care most about AMR_01 and AMR_02 for S2

def interp_pos(rows, robot_id, t_target):
    """Find nearest row to t_target."""
    best = min(rows, key=lambda r: abs(r["t"] - t_target))
    return best[f"{robot_id}_x"], best[f"{robot_id}_y"], best[f"{robot_id}_v"]

with open(analysis_path, "w", encoding="utf-8") as out:
    def w(s=""):
        out.write(s + "\n")
        print(s)

    w("=" * 70)
    w("S2 ALGORITHM-TO-POSITION PROOF  (Phase 2C)")
    w("=" * 70)
    w()
    w("Scenario setup:")
    w("  AMR_01: priority=10 (URGENT), starting at (-3.0, -4.5), heading=0 (East)")
    w("  AMR_02: priority=1  (LOW),    starting at (4.5, -10.0),  heading=90 (North)")
    w("  Both paths intersect at aisle junction near (4.5, -4.5)")
    w()

    # --- Conflict events table
    w("=" * 70)
    w("CONFLICT/STOP EVENTS (from controller state transitions):")
    w("=" * 70)
    for ev in conflict_events:
        w(f"  frame={ev['frame']:4d}  t={ev['t']:6.2f}s  {ev['robot']:6s}  "
          f"EVENT={ev['event']:40s}  pos=({ev['pos_x']:.2f},{ev['pos_y']:.2f})  v={ev['velocity']:.3f}m/s")

    w()
    w("=" * 70)
    w("LOGGED EVENTS (from diagnostic print lines):")
    w("=" * 70)
    for ev in path_events:
        w(f"  frame={ev['frame']:4d}  t={ev['t']:6.2f}s  {ev['line']}")

    # --- Position snapshot at key moments
    w()
    w("=" * 70)
    w("POSITION SNAPSHOTS AT KEY MOMENTS:")
    w("=" * 70)

    key_times = [0.0, 2.0, 4.0, 6.0, 6.5, 8.0, 10.0, 11.0, 12.0, 13.0]
    w(f"  {'t(s)':>6}  {'AMR_01 x':>10} {'AMR_01 y':>10} {'v01':>7}  "
      f"{'AMR_02 x':>10} {'AMR_02 y':>10} {'v02':>7}  {'AMR_02 stop':>12}")
    w("  " + "-" * 80)
    for t_snap in key_times:
        if t_snap > DURATION:
            continue
        # Find nearest row
        near = min(rows, key=lambda r: abs(r["t"] - t_snap))
        x1 = near.get("AMR_01_x", "?")
        y1 = near.get("AMR_01_y", "?")
        v1 = near.get("AMR_01_v", "?")
        x2 = near.get("AMR_02_x", "?")
        y2 = near.get("AMR_02_y", "?")
        v2 = near.get("AMR_02_v", "?")
        stopped2 = near.get("AMR_02_stopped", "?")
        waiting2 = near.get("AMR_02_waiting", "")
        w(f"  {near['t']:>6.2f}  {x1:>10.4f} {y1:>10.4f} {v1:>7.4f}  "
          f"{x2:>10.4f} {y2:>10.4f} {v2:>7.4f}  "
          f"{'STOPPED' if stopped2 else 'MOVING':>12}  {waiting2}")

    # --- Movement delta analysis (prove robot stopped = position stopped changing)
    w()
    w("=" * 70)
    w("POSITION DELTA ANALYSIS (proves stop = no position change):")
    w("=" * 70)

    # Find the YIELD event frame for AMR_02
    yield_frame = None
    resume_frame = None
    for ev in conflict_events:
        if ev["robot"] == "AMR_02" and "YIELD" in ev["event"] and yield_frame is None:
            yield_frame = ev["frame"]
        if ev["robot"] == "AMR_02" and "CONFLICT_CLEARED" in ev["event"] and resume_frame is None:
            resume_frame = ev["frame"]

    if yield_frame is not None and resume_frame is not None:
        w(f"  AMR_02 YIELD detected at frame={yield_frame}  (t={rows[yield_frame]['t']:.2f}s)")
        w(f"  AMR_02 RESUME detected at frame={resume_frame}  (t={rows[resume_frame]['t']:.2f}s)")
        w(f"  Wait duration: {rows[resume_frame]['t'] - rows[yield_frame]['t']:.2f}s")
        w()

        # Position at yield
        r_yield = rows[yield_frame]
        r_resume = rows[resume_frame]

        w(f"  Position at YIELD  : AMR_02=({r_yield['AMR_02_x']:.4f}, {r_yield['AMR_02_y']:.4f})")
        w(f"  Position at RESUME : AMR_02=({r_resume['AMR_02_x']:.4f}, {r_resume['AMR_02_y']:.4f})")

        # Compute displacement during wait
        dx = r_resume["AMR_02_x"] - r_yield["AMR_02_x"]
        dy = r_resume["AMR_02_y"] - r_yield["AMR_02_y"]
        displacement = math.sqrt(dx*dx + dy*dy)
        w(f"  Displacement during wait: {displacement:.4f}m  (expected: ~0.0m)")

        # Confirm AMR_01 DID move during this window
        dx1 = r_resume["AMR_01_x"] - r_yield["AMR_01_x"]
        dy1 = r_resume["AMR_01_y"] - r_yield["AMR_01_y"]
        displacement1 = math.sqrt(dx1*dx1 + dy1*dy1)
        w(f"  AMR_01 displacement during AMR_02 wait: {displacement1:.4f}m  (expected: > 0)")

        # Verify: AMR_02 stopped changing, AMR_01 kept moving
        stopped_while_waiting = displacement < 0.01
        amr1_moved = displacement1 > 0.1
        w()
        if stopped_while_waiting:
            w("  PROOF: AMR_02 position was FROZEN during yield (displacement < 0.01m) [PASS]")
        else:
            w(f"  WARNING: AMR_02 moved {displacement:.4f}m during yield window [FAIL]")
        if amr1_moved:
            w("  PROOF: AMR_01 continued moving while AMR_02 was stopped [PASS]")
        else:
            w(f"  WARNING: AMR_01 barely moved ({displacement1:.4f}m) during AMR_02 yield [FAIL]")
    else:
        w("  No YIELD/RESUME event pair detected for AMR_02.")
        w("  Conflict may not have fired in this run duration.")

    # --- USD transform note
    w()
    w("=" * 70)
    w("USD TRANSFORM STATUS:")
    w("=" * 70)
    w("  USD_AVAILABLE = False on this machine (pxr not installed)")
    w("  The position changes above ARE the world-space transforms that would be")
    w("  written to USD via OmniAMRController._apply_usd_transform().")
    w("  Specifically, when AMR_02 is stopped:")
    w("    controller.is_stopped = True")
    w("    controller.actual_v   = 0.0")
    w("    => no position integration => translate_op.Set() called with same Vec3d")
    w("    => USD time sample at frame N == frame N-1 (flat segment on timeline)")
    w("  When AMR_02 resumes:")
    w("    controller.is_stopped = False")
    w("    controller.actual_v   > 0.0")
    w("    => new Vec3d written each frame => ascending translate_op time samples")
    w("    => visible motion on Omniverse timeline")
    w()
    w("  To see this in Omniverse:")
    w("    1. Run: .\\repo.bat build")
    w("    2. Run: _build\\windows-x86_64\\release\\kit\\python.exe omni_scenarios.py s2")
    w("    3. Open simulation5.usd in USD Composer")
    w("    4. Play timeline: AMR_02 stops at the intersection, then resumes")

    w()
    w("=" * 70)
    w("END OF PHASE 2C ANALYSIS")
    w("=" * 70)

print(f"\n[S2-PROOF] Analysis saved: {analysis_path}")
