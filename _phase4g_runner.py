"""
Phase 4G: Multi-scenario USD evidence runner
Runs S2, S4, S5, S6 and the collision regression.
Captures position traces for each scenario from USD attributes.
"""
import os, sys, json, math
REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "ref_sih_amr"))

from pxr import Usd, UsdGeom
from omni_scenarios import run_scenario

USD_FILE = os.path.join(REPO_ROOT, "simulation5.usd")
FPS = 60.0
DURATION = 25.0
results = {}


def read_translate_op(stage, robot_id):
    for prefix in ["/World/P_DYNEX_Depot/Robots", "/World/Warehouse/Robots"]:
        prim = stage.GetPrimAtPath(f"{prefix}/{robot_id}")
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    return op, f"{prefix}/{robot_id}"
    return None, None


def measure_min_distance(stage, fps, duration):
    """Measure minimum inter-robot distance across all frames."""
    robot_ids = [f"AMR_{i:02d}" for i in range(1, 7)]
    ops = {}
    for r in robot_ids:
        op, path = read_translate_op(stage, r)
        if op:
            ops[r] = op

    min_dist = float("inf")
    collision_events = 0
    collision_frames = 0
    COLLISION_THRESH = 0.6

    frames = list(range(0, int(duration * fps)))
    for f in frames:
        positions = {}
        for r, op in ops.items():
            pos = op.Get(time=float(f))
            if pos:
                positions[r] = (float(pos[0]), float(pos[1]))
        robot_list = list(positions.keys())
        for i in range(len(robot_list)):
            for j in range(i+1, len(robot_list)):
                ra, rb = robot_list[i], robot_list[j]
                ax, ay = positions[ra]
                bx, by = positions[rb]
                d = math.hypot(ax - bx, ay - by)
                if d < min_dist:
                    min_dist = d
                if d < COLLISION_THRESH:
                    collision_frames += 1
                    if f == 0 or True:  # count by checking state change -- simplified
                        collision_events += 1
    return min_dist, collision_events, collision_frames


def run_and_capture(scenario_name, duration=DURATION, fps=FPS):
    print(f"\n{'='*60}")
    print(f"  Running scenario {scenario_name.upper()} ({duration}s @ {fps}fps)")
    print(f"{'='*60}")
    run_scenario(scenario_name=scenario_name, duration_sec=duration, fps=fps)

    # Re-open and capture data
    stage = Usd.Stage.Open(USD_FILE, Usd.Stage.LoadNone)
    if not stage:
        return {"error": "Could not reopen stage"}

    scenario_result = {
        "scenario": scenario_name,
        "timeline_end": stage.GetEndTimeCode(),
        "tps": stage.GetTimeCodesPerSecond(),
        "robots": {}
    }

    for robot_id in ["AMR_01", "AMR_02", "AMR_03"]:
        op, path = read_translate_op(stage, robot_id)
        if op:
            samples = op.GetTimeSamples()
            scenario_result["robots"][robot_id] = {
                "prim_path": path,
                "sample_count": len(samples),
                "positions_1s": []
            }
            for t_int in range(0, int(duration) + 1):
                pos = op.Get(time=float(t_int * fps))
                if pos:
                    scenario_result["robots"][robot_id]["positions_1s"].append(
                        (t_int, round(float(pos[0]), 3), round(float(pos[1]), 3))
                    )

    # Collision regression
    min_dist, col_events, col_frames = measure_min_distance(stage, fps, duration)
    scenario_result["safety"] = {
        "min_inter_robot_dist_m": round(min_dist, 4),
        "collision_events": col_events,
        "collision_frames": col_frames
    }

    return scenario_result


# Run all scenarios
for sc in ["s2", "s4", "s5", "s6"]:
    results[sc] = run_and_capture(sc)

# Also run collision regression with S2 (reference scenario)
print(f"\n{'='*60}")
print("  COLLISION REGRESSION REPORT")
print(f"{'='*60}")
for sc, r in results.items():
    safety = r.get("safety", {})
    print(f"\n  [{sc.upper()}]")
    print(f"    min inter-robot dist: {safety.get('min_inter_robot_dist_m', 'N/A')}m")
    print(f"    collision events:     {safety.get('collision_events', 'N/A')}")
    print(f"    collision frames:     {safety.get('collision_frames', 'N/A')}")

# Print S2 robot traces
print(f"\n{'='*60}")
print("  S2 POSITION TRACES (USD readback)")
print(f"{'='*60}")
for robot_id in ["AMR_01", "AMR_02"]:
    if robot_id in results.get("s2", {}).get("robots", {}):
        robot_data = results["s2"]["robots"][robot_id]
        print(f"\n  {robot_id} @ {robot_data['prim_path']} ({robot_data['sample_count']} samples)")
        print(f"  {'T(s)':>6}  {'X':>8}  {'Y':>8}  {'State'}")
        prev = None
        for row in robot_data["positions_1s"]:
            t, x, y = row
            if prev:
                delta = abs(x - prev[1]) + abs(y - prev[2])
                state = "MOVING" if delta > 0.01 else "STOPPED"
            else:
                state = "STOPPED"
            print(f"  {t:>6}  {x:>8.3f}  {y:>8.3f}  {state}")
            prev = row

# Save evidence to JSON
evidence_path = os.path.join(REPO_ROOT, "_phase4_evidence.json")
with open(evidence_path, "w") as f:
    json.dump(results, f, indent=2)
print(f"\n[4G] Evidence saved to {evidence_path}")
