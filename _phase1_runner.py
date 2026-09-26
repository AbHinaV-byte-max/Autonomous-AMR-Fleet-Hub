"""
Phase 1 Validation Runner — runs all 6 scenarios and captures output.
Uses a shorter duration (12s) at lower FPS (20) to keep wall-clock time reasonable
while still producing enough frames for coordination events to fire.
"""
import os, sys, io, time, traceback, json

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SIH_ROOT = os.path.join(REPO_ROOT, "ref_sih_amr")
if SIH_ROOT not in sys.path:
    sys.path.insert(0, SIH_ROOT)

# --- Capture stdout for each scenario -----------------------------------------
class Tee:
    """Writes to both sys.stdout and a StringIO buffer simultaneously."""
    def __init__(self, buf):
        self._buf = buf
        self._orig = sys.__stdout__
    def write(self, s):
        self._orig.write(s)
        self._buf.write(s)
    def flush(self):
        self._orig.flush()
        self._buf.flush()

def run_and_capture(scenario: str, duration: float = 12.0, fps: float = 20.0) -> str:
    """Runs one scenario and returns all console output as a string."""
    from omni_scenarios import run_scenario
    buf = io.StringIO()
    tee = Tee(buf)
    sys.stdout = tee
    try:
        run_scenario(scenario, duration_sec=duration, fps=fps)
    except Exception:
        sys.stdout.write("\n[RUNNER ERROR]\n" + traceback.format_exc())
    finally:
        sys.stdout = sys.__stdout__
    return buf.getvalue()

# -------------------------------------------------------------------------------

def extract_key_lines(log: str, prefixes: tuple) -> list[str]:
    """Return only lines that start with one of the given prefixes."""
    return [l for l in log.splitlines() if any(l.strip().startswith(p) for p in prefixes)]

SCENARIOS = ["s1", "s2", "s3", "s4", "s5", "s6"]
# Quick test — only 3 s for non-event scenarios to keep the run fast
DURATIONS = {"s1": 8.0, "s2": 12.0, "s3": 8.0, "s4": 12.0, "s5": 10.0, "s6": 12.0}
FPS = 20.0

results = {}
t_start = time.time()

for sc in SCENARIOS:
    print(f"\n{'='*70}")
    print(f"VALIDATION: Running Scenario {sc.upper()}  (duration={DURATIONS[sc]}s @ {FPS} FPS)")
    print(f"{'='*70}")
    t0 = time.time()
    log = run_and_capture(sc, duration=DURATIONS[sc], fps=FPS)
    elapsed = time.time() - t0

    # Extract structured events
    grid_lines   = extract_key_lines(log, ("  Walkable", "  Blocked", "  USD obs", "  Aisle", "  Grid status", "  Total", "  Dynamic"))
    path_lines   = [l for l in log.splitlines() if l.startswith("[PATH]")]
    conflict_lines = [l for l in log.splitlines() if l.startswith("[CONFLICT]")]
    deadlock_lines = [l for l in log.splitlines() if l.startswith("[DEADLOCK]")]
    obstacle_lines = [l for l in log.splitlines() if l.startswith("[OBSTACLE]")]
    failure_lines  = [l for l in log.splitlines() if l.startswith("[FAILURE]")]
    comms_lines    = [l for l in log.splitlines() if l.startswith("[COMMS]")]
    hungarian_lines= [l for l in log.splitlines() if "[HUNGARIAN" in l]
    reached_lines  = [l for l in log.splitlines() if "REACHED PICKUP" in l or "COMPLETED" in l]

    results[sc] = {
        "log": log,
        "elapsed_s": round(elapsed, 2),
        "grid_lines": grid_lines,
        "path_events": len(path_lines),
        "conflict_events": len(conflict_lines),
        "deadlock_events": len(deadlock_lines),
        "obstacle_events": len(obstacle_lines),
        "failure_events": len(failure_lines),
        "comms_events": len(comms_lines),
        "hungarian_events": len(hungarian_lines),
        "reached_events": len(reached_lines),
        "path_lines": path_lines,
        "conflict_lines": conflict_lines,
        "deadlock_lines": deadlock_lines,
        "obstacle_lines": obstacle_lines,
        "failure_lines": failure_lines,
        "comms_lines": comms_lines,
        "hungarian_lines": hungarian_lines,
        "reached_lines": reached_lines,
    }

total_elapsed = time.time() - t_start
print(f"\n\n{'='*70}")
print(f"ALL SCENARIOS COMPLETE  ({total_elapsed:.1f}s total)")
print(f"{'='*70}")

# Save raw logs
log_dir = os.path.join(REPO_ROOT, "_phase1_validation_logs")
os.makedirs(log_dir, exist_ok=True)
for sc, r in results.items():
    with open(os.path.join(log_dir, f"scenario_{sc}.log"), "w", encoding="utf-8") as f:
        f.write(r["log"])

# Save summary JSON for the report
summary = {sc: {k: v for k, v in r.items() if k != "log"} for sc, r in results.items()}
with open(os.path.join(log_dir, "summary.json"), "w") as f:
    json.dump(summary, f, indent=2)

print(f"\nRaw logs saved to: {log_dir}")
print("summary.json written.")

# Print quick results table
print(f"\n{'Scenario':<8} {'PATH':>5} {'CONFLICT':>9} {'DEADLOCK':>9} {'OBSTACLE':>9} {'FAILURE':>8} {'COMMS':>6} {'HUNGARIAN':>10} {'REACHED':>8} {'Time':>6}")
print("-" * 90)
for sc in SCENARIOS:
    r = results[sc]
    print(f"{sc.upper():<8} {r['path_events']:>5} {r['conflict_events']:>9} "
          f"{r['deadlock_events']:>9} {r['obstacle_events']:>9} "
          f"{r['failure_events']:>8} {r['comms_events']:>6} "
          f"{r['hungarian_events']:>10} {r['reached_events']:>8} "
          f"{r['elapsed_s']:>5.1f}s")
print()
