"""
FastAPI dashboard backend for the prototype live simulation.

The dashboard backend currently owns the live Simulator lifecycle so the web UI
can control scenarios, pause/resume/step execution, submit tasks, and stream
TelemetryBus snapshots. The Section 5.2 architecture guard applies in the
opposite direction: core modules under /robot, /allocator, /sim, and /comms
must not import the dashboard. This keeps the simulation core independent of
the presentation layer while allowing the prototype dashboard to orchestrate
a local simulation instance.
"""
import asyncio
import json
import sys
import os
import threading
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from sim.simulator import Simulator
from experiments.runner import SCENARIOS
from models import RobotStatus, Task, TaskStatus

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from dashboard.backend.telemetry import TelemetryBus
from dashboard.backend.db import init_db, persist_snapshot

app = FastAPI(title="AMR Dashboard API")

# The dashboard is normally served by this same FastAPI app, so cross-origin
# access is disabled by default. Explicit origins can be supplied for a
# separate frontend deployment without reopening the API to every origin.
_cors_origins = [
    origin.strip()
    for origin in os.getenv("DASHBOARD_CORS_ORIGINS", "").split(",")
    if origin.strip()
]
if _cors_origins:
    app.add_middleware(
        CORSMiddleware,
        allow_origins=_cors_origins,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["Content-Type"],
    )

@app.middleware("http")
async def add_security_headers(request: Request, call_next):
    response = await call_next(request)
    response.headers.setdefault("X-Content-Type-Options", "nosniff")
    response.headers.setdefault("X-Frame-Options", "DENY")
    response.headers.setdefault("Referrer-Policy", "strict-origin-when-cross-origin")
    response.headers.setdefault(
        "Permissions-Policy",
        "camera=(), microphone=(), geolocation=()",
    )
    return response

RUNNING = True
LIVE_SCENARIO = "S1_Normal"
SIM_TICK_RATE = 0.8  # Seconds between simulation ticks.
SIM_PAUSED = False
SIM_STEP_REQUEST = False
AUTO_TASKS_ENABLED = True

# Shared telemetry bus — injected by the simulation process on startup
_bus: TelemetryBus = TelemetryBus()
_latest_snapshot: dict = {}
_snapshot_lock = threading.Lock()


def get_bus() -> TelemetryBus:
    return _bus


def inject_bus(bus: TelemetryBus):
    """Called by the simulation harness to wire the bus in."""
    global _bus
    _bus = bus


CURRENT_SIM = None
_SIM_LOCK = threading.RLock()

def live_simulation_loop(bus):
    global LIVE_SCENARIO, CURRENT_SIM
    while RUNNING:
        current_scen = LIVE_SCENARIO
        sim = Simulator(ascii_map=SCENARIOS[current_scen], headless=True, telemetry_bus=bus, strategy="P2P")
        sim.scenario_name = current_scen
        sim.task_generator.enabled = AUTO_TASKS_ENABLED
        CURRENT_SIM = sim

        # S6 CommDelay: patch comms so robot-0 drops broadcasts
        if current_scen == "S6_CommDelay":
            original_send = sim.comms.send
            def patched_send(msg):
                if msg.robot_id != "robot-0":
                    original_send(msg)
            sim.comms.send = patched_send

        # Keep one simulator instance alive for the whole scenario. Recreating
        # it every 500 ticks silently discarded live tasks, robot positions,
        # batteries and reservations, which made Auto Mode appear to create
        # fresh work after a reset and made long-running fleet behaviour
        # impossible to observe truthfully.
        while RUNNING and current_scen == LIVE_SCENARIO:
            global SIM_PAUSED, SIM_STEP_REQUEST
            while SIM_PAUSED and not SIM_STEP_REQUEST and RUNNING and current_scen == LIVE_SCENARIO:
                time.sleep(0.05)
            if not RUNNING or current_scen != LIVE_SCENARIO:
                break

            # Apply one-shot scenario events against the persistent simulator.
            # These used to be tied to the old 500-tick recreation loop.
            next_tick = sim.tick_count + 1
            if current_scen == "S4_Blocked" and next_tick == 40:
                sim.block_cell(5, 2)
            elif current_scen == "S5_Failure" and next_tick == 50:
                sim.kill_robot("robot-0")

            sim.task_generator.enabled = AUTO_TASKS_ENABLED
            with _SIM_LOCK:
                sim.tick()
            if SIM_STEP_REQUEST:
                SIM_STEP_REQUEST = False
            time.sleep(SIM_TICK_RATE)  # Smooth, observable pace


@app.on_event("startup")
async def _on_startup():
    init_db()
    # Background task: drain telemetry bus → update latest snapshot + persist
    asyncio.create_task(_drain_bus())
    # Start the continuous live simulation loop for the UI map
    threading.Thread(target=live_simulation_loop, args=(_bus,), daemon=True).start()


@app.post("/api/scenario")
async def set_scenario(request: dict):
    """Switch the live simulation scenario and wait for the new sim to become authoritative."""
    global LIVE_SCENARIO
    if not request or "scenario" not in request:
        return {"status": "error", "message": "Scenario is required"}

    scen = request["scenario"]
    if scen not in SCENARIOS:
        raise HTTPException(status_code=400, detail="Invalid scenario")

    LIVE_SCENARIO = scen

    # The simulation loop owns Simulator creation. Wait briefly until it has
    # replaced the old instance so subsequent robot/task commands cannot
    # accidentally target the previous scenario.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        sim = CURRENT_SIM
        if sim is not None and getattr(sim, "scenario_name", None) == scen:
            if sim.telemetry_bus is not None:
                sim.telemetry_bus.publish(sim._build_snapshot())
            return {"status": "ok", "scenario": scen, "ready": True}
        await asyncio.sleep(0.05)

    return {
        "status": "ok",
        "scenario": scen,
        "ready": False,
        "message": "Scenario requested; simulation is still transitioning."
    }


@app.get("/api/scenario")
async def get_scenario():
    return {"scenario": LIVE_SCENARIO}

@app.get("/api/tasks/auto")
async def get_auto_task_mode():
    return {"enabled": AUTO_TASKS_ENABLED}


@app.post("/api/tasks/auto")
async def set_auto_task_mode(request: dict):
    global AUTO_TASKS_ENABLED
    if not request or "enabled" not in request:
        raise HTTPException(status_code=400, detail="enabled is required")
    AUTO_TASKS_ENABLED = bool(request["enabled"])
    cleared_auto_tasks = 0
    sim = CURRENT_SIM
    if sim is not None:
        with _SIM_LOCK:
            sim.task_generator.enabled = AUTO_TASKS_ENABLED
            if not AUTO_TASKS_ENABLED:
                cleared_auto_tasks = sim.clear_queued_auto_tasks()
            if sim.telemetry_bus is not None:
                sim.telemetry_bus.publish(sim._build_snapshot())
    return {"status": "ok", "enabled": AUTO_TASKS_ENABLED, "cleared_auto_tasks": cleared_auto_tasks}


@app.post("/api/speed")
async def set_speed(request: dict):
    """Adjusts live simulation tick rate (seconds per tick)."""
    global SIM_TICK_RATE
    if not request or "rate" not in request:
        raise HTTPException(status_code=400, detail="Missing simulation rate")
    try:
        rate = float(request["rate"])
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail="Simulation rate must be numeric")
    if not 0.2 <= rate <= 3.0:
        raise HTTPException(status_code=400, detail="Simulation rate must be between 0.2 and 3.0")
    SIM_TICK_RATE = rate
    return {"rate": SIM_TICK_RATE}


@app.post("/api/robot/{robot_id}/toggle-power")
async def toggle_robot_power(robot_id: str):
    """Shuts down an active robot (triggering task handover) or revives an offline robot."""
    global CURRENT_SIM
    if not CURRENT_SIM:
        raise HTTPException(status_code=503, detail="Simulation not running")
    manager = next((m for m in CURRENT_SIM.robot_managers if m.state.robot_id == robot_id), None)
    if not manager:
        raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
    
    is_offline = (manager.state.status == RobotStatus.OFFLINE) or (str(manager.state.status).upper().endswith("OFFLINE"))
    if is_offline:
        CURRENT_SIM.revive_robot(robot_id)
        action = "revived"
    else:
        CURRENT_SIM.kill_robot(robot_id)
        action = "shutdown"

    if CURRENT_SIM.telemetry_bus is not None:
        snap = CURRENT_SIM._build_snapshot()
        CURRENT_SIM.telemetry_bus.publish(snap)

    return {"status": "ok", "robot_id": robot_id, "action": action, "new_status": manager.state.status.value, "battery": manager.state.battery}


@app.post("/api/robot/{robot_id}/force-battery")
async def force_robot_battery(robot_id: str, request: dict = None):
    """Sets a robot's battery level (e.g. 18% to trigger auto-shedding or 95% to charge)."""
    global CURRENT_SIM
    if not CURRENT_SIM:
        raise HTTPException(status_code=503, detail="Simulation not running")
    manager = next((m for m in CURRENT_SIM.robot_managers if m.state.robot_id == robot_id), None)
    if not manager:
        raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
    
    target_batt = 18.0
    if request and "battery" in request:
        try:
            target_batt = float(request["battery"])
        except (TypeError, ValueError):
            raise HTTPException(status_code=400, detail="Battery level must be numeric")
    if not 0 <= target_batt <= 100:
        raise HTTPException(status_code=400, detail="Battery level must be between 0 and 100")
    
    manager.state.battery = target_batt

    if target_batt <= 20.0:
        if manager.state.status not in (RobotStatus.CHARGING, RobotStatus.OFFLINE):
            orphaned = manager.current_task
            task_id = orphaned.task_id if orphaned else None
            CURRENT_SIM._orphan_task(manager)
            manager.state.status = RobotStatus.CHARGING
            manager.state.planned_path = []
            manager.target_cell = None
            manager.reservation_table.expire(manager.state.robot_id)
            CURRENT_SIM.event_log.log_conflict(
                robot_id, "SYSTEM", "LOW_BATTERY_SHED",
                f"Battery forced low ({target_batt:.1f}%) -> Task {task_id} reallocated to fleet",
                CURRENT_SIM.tick_count
            )
            CURRENT_SIM._allocate()
    elif target_batt >= 90.0 and manager.state.status == RobotStatus.CHARGING:
        manager.state.status = RobotStatus.IDLE
        CURRENT_SIM.event_log.log_conflict(
            robot_id, "SYSTEM", "CHARGE_COMPLETE",
            f"Quick charged to {target_batt:.0f}% -> Restored to active service",
            CURRENT_SIM.tick_count
        )
        CURRENT_SIM._allocate()

    if CURRENT_SIM.telemetry_bus is not None:
        snap = CURRENT_SIM._build_snapshot()
        CURRENT_SIM.telemetry_bus.publish(snap)

    return {"status": "ok", "robot_id": robot_id, "battery": manager.state.battery, "new_status": manager.state.status.value}

@app.on_event("shutdown")
async def _on_shutdown():
    global RUNNING
    RUNNING = False

async def _drain_bus():
    while RUNNING:
        snap = await _bus.subscribe(timeout=0.1)
        if snap is not None:
            with _snapshot_lock:
                global _latest_snapshot
                _latest_snapshot = snap
            persist_snapshot(snap)


@app.get("/snapshot")
async def http_snapshot():
    """HTTP fallback — returns the latest snapshot once."""
    with _snapshot_lock:
        return _latest_snapshot

@app.get("/api/fleet/status")
async def get_fleet_status():
    with _snapshot_lock:
        return _latest_snapshot

@app.get("/api/metrics/live")
async def get_live_metrics():
    with _snapshot_lock:
        snap = _latest_snapshot
        m = snap.get("metrics", {})
        return {
            "collision_count": m.get("COLLISION_COUNT", m.get("collision_count", 0)),
            "makespan": m.get("MAKESPAN", m.get("makespan", 0)),
            "throughput": m.get("THROUGHPUT", m.get("throughput", 0)),
            "replan_count": m.get("REPLAN_COUNT", m.get("replan_count", 0)),
            "waiting_time": m.get("WAITING_TIME", m.get("waiting_time", 0))
        }

@app.post("/api/simulation/pause")
async def pause_simulation():
    global SIM_PAUSED
    SIM_PAUSED = True
    return {"is_running": False}


@app.post("/api/simulation/start")
async def start_simulation():
    global SIM_PAUSED, SIM_STEP_REQUEST
    SIM_PAUSED = False
    SIM_STEP_REQUEST = False
    return {"is_running": True}


@app.post("/api/simulation/step")
async def step_simulation():
    global SIM_PAUSED, SIM_STEP_REQUEST
    SIM_PAUSED = True
    SIM_STEP_REQUEST = True
    return {"is_running": False, "step_requested": True}


@app.post("/api/tasks/submit")
async def submit_task(request: dict):
    global CURRENT_SIM
    if CURRENT_SIM is None:
        return {"status": "error", "message": "Simulation not running"}

    required = ("pickup_x", "pickup_y", "dropoff_x", "dropoff_y")
    if any(key not in request for key in required):
        return {"status": "error", "message": "pickup/dropoff coordinates are required"}

    pickup = (int(request["pickup_x"]), int(request["pickup_y"]))
    dropoff = (int(request["dropoff_x"]), int(request["dropoff_y"]))
    if CURRENT_SIM.grid_map.get_cell(*pickup) == "#":
        return {"status": "error", "message": "pickup cell is blocked"}
    if CURRENT_SIM.grid_map.get_cell(*dropoff) != "D":
        return {"status": "error", "message": "dropoff must be a configured delivery dock"}

    task_id = f"TASK-{int(time.time() * 1000)}"
    priority = int(request.get("priority", 1))
    task = Task(task_id=task_id, pickup_cell=pickup, dropoff_cell=dropoff,
                priority=priority, status=TaskStatus.QUEUED,
                created_at=float(CURRENT_SIM.tick_count), source="MANUAL")
    # The live simulation thread mutates the same Simulator every tick.
    # Manual dispatch must use the simulator lock as well, otherwise a task can
    # be appended while allocation/CBS is rebuilding paths, producing stale
    # assignment or telemetry state in the UI.
    with _SIM_LOCK:
        CURRENT_SIM.tasks.append(task)
        CURRENT_SIM._allocate()
        assigned_robot_id = task.assigned_robot_id
        task_status = task.status.value

        if CURRENT_SIM.telemetry_bus is not None:
            CURRENT_SIM.telemetry_bus.publish(CURRENT_SIM._build_snapshot())

    return {"status": "ok", "task_id": task_id,
            "pickup": list(pickup), "dropoff": list(dropoff),
            "priority": priority,
            "assigned_robot_id": assigned_robot_id,
            "task_status": task_status}


_benchmark_lock = asyncio.Lock()

def _run_dashboard_benchmark_trials(scenario: str):
    """Run the bounded benchmark off the event-loop thread.

    The benchmark runner is synchronous and intentionally in-process for
    serverless compatibility. Keeping it off the FastAPI event-loop thread
    allows the live WebSocket telemetry coroutine to continue serving the
    dashboard while the benchmark is executing.
    """
    import experiments.runner

    previous_max_ticks = experiments.runner.MAX_TICKS
    experiments.runner.MAX_TICKS = 100
    try:
        results = []
        for strategy in ["B0", "B1", "B2", "P1"]:
            for trial in range(2):
                results.append(experiments.runner.run_trial((scenario, strategy, trial)))
        return results
    finally:
        experiments.runner.MAX_TICKS = previous_max_ticks


@app.post("/api/benchmark")
async def trigger_benchmark(request: dict = None):
    """Run the bounded dashboard benchmark without blocking live telemetry."""

    scenario = "S1_Normal"
    if request and "scenario" in request:
        scenario = request["scenario"]
        if scenario not in SCENARIOS:
            raise HTTPException(status_code=400, detail="Invalid scenario")
        global LIVE_SCENARIO
        LIVE_SCENARIO = scenario

    # Serialize benchmark runs because the runner temporarily changes its
    # module-level MAX_TICKS. asyncio.to_thread keeps the event loop free for
    # WebSocket telemetry while the synchronous runner does its work.
    async with _benchmark_lock:
        try:
            results = await asyncio.to_thread(_run_dashboard_benchmark_trials, scenario)
        except Exception as exc:
            # Keep the API failure actionable instead of hiding the actual cause.
            raise HTTPException(
                status_code=500,
                detail=f"Benchmark execution failed: {type(exc).__name__}: {exc}",
            ) from exc

    summary = {}
    for r in results:
        strat = r["strategy"]
        if strat not in summary:
            summary[strat] = {
                "completed": 0,
                "wait": 0,
                "collisions": 0,
                "makespan": 0,
                "throughput": 0,
                "deadlocks": 0,
                "replans": 0,
                "count": 0,
            }
        summary[strat]["completed"] += r.get("completed_tasks", 0)
        summary[strat]["wait"] += r.get("WAITING_TIME", r.get("waiting_time", 0))
        summary[strat]["collisions"] += r.get("COLLISION_COUNT", r.get("collision_count", 0))
        summary[strat]["makespan"] += r.get("MAKESPAN", r.get("makespan", 0))
        summary[strat]["throughput"] += r.get("THROUGHPUT", r.get("throughput", 0))
        summary[strat]["deadlocks"] += r.get("DEADLOCK_COUNT", r.get("deadlock_count", 0))
        summary[strat]["replans"] += r.get("REPLAN_COUNT", r.get("replan_count", 0))
        summary[strat]["count"] += 1

    for strat in summary:
        count = summary[strat]["count"]
        for key in ("completed", "wait", "collisions", "makespan", "throughput", "deadlocks", "replans"):
            summary[strat][key] /= count

    return {
        "scenario": scenario,
        "trial_count": 2,
        "ticks_per_trial": 100,
        "results": summary,
    }

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    """Streams live snapshots at ~10 Hz."""
    await ws.accept()
    try:
        while True:
            with _snapshot_lock:
                snap = dict(_latest_snapshot)
            if snap:
                await ws.send_text(json.dumps(snap, default=str))
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass

@app.websocket("/ws/fleet-stream")
async def ws_fleet_stream(ws: WebSocket):
    await websocket_endpoint(ws)

# Mount web directory to serve friend's integrated dashboard
from pathlib import Path
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

web_dir = Path(__file__).resolve().parent.parent / "web"
if web_dir.exists():
    app.mount("/static", StaticFiles(directory=str(web_dir)), name="static")

    @app.get("/", include_in_schema=False)
    async def serve_index():
        return FileResponse(str(web_dir / "index.html"))

