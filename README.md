# SIH 2026 — Edge-AI Distributed Fleet Coordination for Autonomous Mobile Robots

**Problem Statement 26123 — Bharat Electronics Limited**

This repository contains the SIH 2026 prototype for decentralized coordination of Autonomous Mobile Robots (AMRs) in a smart warehouse.

The primary implementation is the `ref_sih_amr` runtime. It provides multi-robot task allocation, path planning, collision avoidance, dynamic rerouting, failure recovery, simulated peer communication, telemetry, benchmarking, and a live dashboard.

---

## System Architecture

```text
                    SIH AMR Fleet
                         |
              +----------+----------+
              |                     |
        Task Allocation        Fleet Simulation
        Hungarian Method       & Robot Managers
              |                     |
              +----------+----------+
                         |
                 Local Coordination
                         |
              +----------+----------+
              |                     |
             A*                   CBS
        path planning       multi-robot conflict
                            resolution / reservations
              |                     |
              +----------+----------+
                         |
                 TelemetryBus
                         |
                  Dashboard API
                         |
                  WebSocket Stream
                         |
                 Live Fleet UI

The canonical implementation is under:

ref_sih_amr/
├── allocator/       Task allocation
├── comms/           Simulated peer communication
├── dashboard/       FastAPI backend and web dashboard
├── experiments/     Reproducible benchmark scenarios
├── robot/           Robot policies, A*, CBS, task management
├── sim/             Fleet simulator and orchestration
└── tests/           Regression and behavioral tests
Core Capabilities
Distributed fleet coordination

The simulator models multiple AMRs operating concurrently in a warehouse environment.

Each robot maintains local state and participates in simulated peer communication.

Coordination includes:

task assignment
path planning
reservations
conflict detection
waiting/yielding
rerouting
deadlock handling
robot failure recovery
degraded communication handling
Task allocation

Queued and recoverable tasks are assigned to available robots using the fleet allocator.

Recoverable tasks can be reassigned when a robot becomes unavailable.

Path planning

The runtime uses grid-based path planning with:

A* shortest-path search
obstacle-aware planning
dynamic occupancy checks
space-time constraints
CBS-based multi-robot conflict resolution

CBS is used to coordinate paths where independent plans would conflict.

Safety layer

The coordination stack includes runtime checks for:

vertex conflicts
edge-swap conflicts
occupied cells
reservations
blocked cells
stale peer heartbeats
robot failure
rerouting conditions

The deterministic safety/coordination layer remains authoritative over higher-level decision logic.

Dashboard

The primary dashboard is served by the FastAPI backend.

Start it with:

.\start_dashboard.bat

Then open:

http://localhost:8000

The dashboard receives live fleet telemetry through the backend WebSocket stream.

It provides visibility into:

robot state
position
battery
velocity
current tasks
task pipeline
fleet coordination
benchmark information
warehouse visualization

The dashboard visualization is implemented with the project's web frontend and Canvas-based warehouse rendering.

Benchmarking

The repository contains reproducible benchmark scenarios and strategies under:

ref_sih_amr/experiments/

The project evaluates metrics including:

makespan
task completion time
waiting time
collision count
deadlock count
replan count
throughput
communication latency
message loss
CPU/memory usage
edge inference latency
energy proxy metrics

The intended comparison includes sequential execution, independent planning, stop-and-wait coordination, and the proposed coordinated fleet strategy.

Testing

Run the complete regression suite with:

python -m pytest ref_sih_amr/tests -q

The current regression suite validates:

architecture boundaries
task allocation
collision avoidance
blocked-aisle handling
robot failure recovery
communication degradation
planning behavior
fleet coordination scenarios
Repository Structure
ref_sih_amr/         Canonical SIH AMR runtime
start_dashboard.bat  Dashboard launcher

mcp_fleet/           Optional/legacy MCP integration
fleet_controller.py  Omniverse fleet integration
omniverse_adapter.py Omniverse adapter
omni_*.py             Omniverse integration modules
simulation5.usd      OpenUSD warehouse stage

scenarios/            Omniverse scenario assets
source/               Omniverse Kit project infrastructure
templates/            Omniverse Kit templates
tools/                Repository tooling

readme-assets/        Documentation assets
.github/              GitHub configuration
Canonical vs. Omniverse integration

ref_sih_amr is the primary implementation used by the current automated regression suite and dashboard.

The Omniverse/MCP components are retained as an integration and visualization environment. They are not required to run the canonical simulator and dashboard.

This separation keeps the fleet-coordination runtime independently testable while preserving the repository's OpenUSD/Omniverse integration.

Running the Core Runtime

Run the regression suite:

python -m pytest ref_sih_amr/tests -q

Start the live dashboard:

.\start_dashboard.bat

The dashboard backend starts the simulator and exposes live fleet telemetry to the web frontend.

Development Principles

The project follows these architectural boundaries:

Core simulation and coordination code must not depend on the dashboard.
Dashboard code consumes telemetry rather than owning coordination logic.
Robot coordination is modeled at distributed edge-node level.
Deterministic safety checks remain authoritative.
Benchmark scenarios should remain reproducible.
Omniverse integration remains isolated from the canonical runtime.
SIH Problem Statement

Problem Statement 26123

Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs) in Smart Warehouses

The prototype targets coordinated operation of multiple AMRs in warehouse environments with emphasis on local decision-making, collision avoidance, task allocation, rerouting, resilience, telemetry, and measurable fleet performance.

License

See LICENSE.
