# 🤖 Edge-AI Distributed Fleet Coordination for AMRs

**SIH 2026 · Problem Statement 26123 · Bharat Electronics Limited**

A distributed multi-robot coordination prototype for **Autonomous Mobile Robots (AMRs)** operating in smart warehouses.

The project combines task allocation, grid-based path planning, multi-robot conflict resolution, collision-safety checks, rerouting, failure recovery, simulated peer communication, telemetry, benchmarking, and a live fleet dashboard.

> **Primary runtime:** `ref_sih_amr/`  
> **Dashboard:** FastAPI + WebSocket + HTML5 Canvas  
> **Coordination:** Hungarian allocation + A* + CBS + runtime safety checks

---

## ✨ What the prototype does

| Capability | Implementation |
|---|---|
| 🤖 Multi-AMR coordination | Concurrent simulated robot managers |
| 📦 Task allocation | Fleet allocator with Hungarian assignment |
| 🧭 Navigation | Grid-based A* path planning |
| 🔀 Conflict resolution | Space-time planning + CBS |
| 🛡️ Safety | Vertex, edge-swap, occupancy and reservation checks |
| 🚧 Dynamic rerouting | Blocked-cell detection and replanning |
| 🔋 Resilience | Battery/failure handling and task reassignment |
| 📡 Peer communication | Simulated heartbeat/message channel with degradation handling |
| 📊 Telemetry | Queue-based `TelemetryBus` + WebSocket stream |
| 🖥️ Operations dashboard | Live fleet state, task pipeline and warehouse view |
| 🧪 Benchmarking | Reproducible scenarios and coordination strategies |

---

## 🏗️ System architecture

```text
                         SMART WAREHOUSE
                               │
                     ┌─────────▼─────────┐
                     │   Fleet Simulator │
                     │   Robot Managers  │
                     └─────────┬─────────┘
                               │
                ┌──────────────┼──────────────┐
                │              │              │
          Task Allocation   A* Planning   Peer Comms
          Hungarian Method  Grid Search    Heartbeats
                │              │              │
                └──────────────┼──────────────┘
                               │
                     ┌─────────▼─────────┐
                     │   CBS Coordinator │
                     │ conflicts / paths │
                     └─────────┬─────────┘
                               │
                     Deterministic Safety
                               │
                     ┌─────────▼─────────┐
                     │    TelemetryBus   │
                     └─────────┬─────────┘
                               │
                       FastAPI WebSocket
                               │
                     ┌─────────▼─────────┐
                     │   Fleet Dashboard │
                     │  Canvas Warehouse │
                     └───────────────────┘
```

The **deterministic safety and coordination layer remains authoritative** over higher-level decision logic.

---

## 🖥️ Run the live dashboard

### Requirements

- Python environment with the project dependencies installed
- Windows for the provided `.bat` launcher

### Start

```powershell
.\start_dashboard.bat
```

Then open:

```text
http://localhost:8000
```

The dashboard backend starts the simulator and streams live fleet telemetry to the web frontend.

### Dashboard provides

- Fleet robot state and positions
- Battery and velocity information
- Current task assignments
- Logistics order pipeline
- Coordination state
- Warehouse visualization
- Benchmark information

---

## 🧪 Validation & benchmarking

Validation records are maintained under [`docs/validation/`](docs/validation/).

Run the regression suite:

```powershell
python -m pytest ref_sih_amr/tests -q
```

The current regression suite covers architecture boundaries, allocation, collision avoidance, blocked aisles, robot failure recovery, communication degradation, planning behavior, and fleet coordination scenarios.

Benchmark scenarios and metrics live under:

```text
ref_sih_amr/experiments/
```

Tracked metrics include:

- Makespan
- Task completion time
- Waiting time
- Collision count
- Deadlock count
- Replan count
- Throughput
- Communication latency
- Message loss
- CPU / memory usage
- Edge inference latency
- Energy proxy metrics

The benchmark framework includes sequential execution, independent planning, stop-and-wait coordination, and the proposed coordinated fleet strategy.

---

## 📁 Repository structure

`ref_sih_amr/` is the canonical SIH runtime. The Omniverse integration is grouped under `omniverse/` so the repository root stays focused on project entry points and configuration.

```text
SIH/
├── ref_sih_amr/                 # Canonical SIH AMR runtime
│   ├── allocator/               # Task allocation
│   ├── comms/                   # Simulated peer communication
│   ├── dashboard/               # FastAPI backend + web dashboard
│   ├── experiments/             # Benchmark scenarios
│   ├── robot/                   # Robot policies, A*, CBS, task management
│   ├── sim/                     # Fleet simulation and orchestration
│   └── tests/                   # Regression and behavioral tests
│
├── omniverse/                   # OpenUSD / Omniverse integration
│   ├── fleet_controller.py
│   ├── omni_* / omniverse_adapter.py
│   ├── run_omni_sih_simulation.py
│   └── integration and demo scripts
│
├── assets/                      # Warehouse and presentation assets
│   ├── omniverse/assets/omniverse/simulation5.usd
│   ├── warehouse/warehouse_map.txt
│   └── dashboard/fleet_dashboard_hud.png
│
├── docs/validation/             # Validation records
├── mcp_fleet/                   # MCP integration
├── scenarios/                   # Omniverse scenario assets
├── source/                      # Omniverse Kit project infrastructure
├── templates/                   # Omniverse templates
├── tools/                       # Repository tooling
├── start_dashboard.bat          # Canonical dashboard launcher
└── README.md
```

### Canonical runtime vs. Omniverse integration

`ref_sih_amr/` is the **primary implementation** used by the current automated regression suite and live dashboard.

The Omniverse/MCP components are retained as an integration and visualization environment. They are **not required** to run the canonical simulator and dashboard.

This separation keeps the fleet-coordination runtime independently testable while preserving the OpenUSD/Omniverse integration path.

---

## 🧠 Coordination pipeline

1. **Tasks enter the fleet queue.**
2. **The allocator assigns work** to eligible robots.
3. **A*** generates obstacle-aware paths.
4. **CBS** resolves multi-robot path conflicts using space-time constraints.
5. **Runtime safety checks** guard against occupancy, vertex and edge-swap conflicts.
6. **Blocked paths or changing conditions** trigger replanning.
7. **Robot failure or degraded communication** can move work into recovery/reassignment.
8. **TelemetryBus** publishes the current fleet state to the dashboard.

---

## 🔬 Engineering principles

- Core simulation and coordination code does **not** depend on the dashboard.
- The dashboard consumes telemetry rather than owning coordination logic.
- Robot coordination is modeled at the **distributed edge-node** level.
- Deterministic safety checks remain authoritative.
- Benchmark scenarios remain reproducible.
- Omniverse integration remains separated from the canonical runtime.

---

## 🎯 SIH Problem Statement

**26123 — Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs) in Smart Warehouses**

The prototype targets coordinated operation of multiple AMRs in warehouse environments, with emphasis on local decision-making, collision avoidance, task allocation, rerouting, resilience, telemetry, and measurable fleet performance.

---

## 📌 Status

**SIH 2026 prototype · actively developed**

For the fastest path to the working demonstration, use the canonical runtime and dashboard described above.

