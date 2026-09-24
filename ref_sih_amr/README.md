# Decentralized Multi-AMR Warehouse Coordination

### SIH 2026 — Problem Statement 26123

An edge-oriented simulation and coordination framework for multiple
Autonomous Mobile Robots (AMRs) operating in a shared warehouse environment.

The system focuses on three core challenges:

- Decentralized inter-robot coordination
- Conflict resolution and collision avoidance
- Dynamic task allocation and recovery

The project is designed around local robot decision-making, while the
dashboard is used for monitoring and visualization.

---

## Problem

In a smart warehouse, multiple AMRs may need to use the same aisles,
intersections, and choke points at the same time.

A centralized stop-and-wait approach can cause:

- unnecessary waiting
- congestion at narrow intersections
- deadlocks
- poor task throughput
- difficulty recovering when an aisle or robot becomes unavailable

This project explores a decentralized approach in which each robot maintains
its own state, plans its movement, communicates intent with peers, and adapts
when the environment changes.

---

## Proposed Approach

The system follows the pipeline:

**Task → Assignment → Path Planning → Reservation → Coordination → Execution → Replanning**

Each robot maintains local information such as:

- current position and velocity
- assigned task
- planned path
- reservations
- peer intent
- communication quality
- battery/state information

When a conflict or failure is detected, the robot can wait, yield, reroute,
or enter a degraded operating mode depending on the situation.

---

## Key Components

### 1. Path Planning

The simulator uses grid-based path planning with A* and a space-time
representation for considering reservations and movement over time.

### 2. Multi-Robot Coordination

Robots exchange intent/state information and use reservations to reduce
conflicts at shared cells and intersections.

The coordination layer includes:

- vertex conflict detection
- edge-swap conflict detection
- intersection checks
- priority handling
- deadlock detection
- replanning

### 3. Dynamic Task Allocation

Tasks are assigned to available robots using a Hungarian-algorithm-based
allocator.

The system also supports task recovery when a robot becomes unavailable.

### 4. Failure Handling

The simulator includes scenarios for:

- blocked aisles
- robot failure
- communication degradation/loss
- task reassignment
- replanning

### 5. Edge Decision Layer

A local policy layer is provided for robot-side decisions such as:

- `CONTINUE`
- `YIELD`
- `WAIT`
- `REROUTE`
- `DEGRADED_MODE`

A deterministic safety policy acts as the fallback/guard layer, while an
ONNX-based policy can be evaluated locally.

### 6. Communication & Security

The communication layer provides a pluggable message-channel abstraction
for peer coordination.

The security layer includes validation concepts such as:

- robot authentication
- message integrity
- sequence checking
- freshness checking
- physical plausibility checks

### 7. Dashboard

A web dashboard provides visualization and monitoring of the simulation
and experiment results.

### 8. Installation & Setup

#### Prerequisites

Ensure you have the following installed on your system:

- **Python 3.10+**
- **Node.js 18+** and **npm**
- **Git**

---

#### 1. Clone the Repository

```bash
git clone https://github.com/shreyasjakaraddi07/SIH-AMR.git
cd SIH-AMR
```

---

#### 2. Backend Setup (Simulation & Telemetry API)

##### 1. Create and activate a virtual environment

**Windows (PowerShell):**

```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

**Linux / macOS:**

```bash
python3 -m venv venv
source venv/bin/activate
```

##### 2. Install Python dependencies

```bash
pip install fastapi uvicorn websockets scipy numpy pandas matplotlib onnxruntime pytest
```

*Optional: Install **`pygame`** if you want local non-headless window rendering.*

##### 3. Start the FastAPI Telemetry Server

```bash
uvicorn dashboard.backend.main:app --reload --port 8000
```

> The live simulation loop will start automatically and stream telemetry at `ws://localhost:8000/ws`.

---

#### 3. Frontend Setup (Live Dashboard & Map View)

##### 1. Navigate to the frontend directory

```bash
cd dashboard/frontend
```

##### 2. Install Node.js dependencies

```bash
npm install
```

##### 3. Start the Vite development server

```bash
npm run dev
```

##### 4. Open in your browser

Navigate to **`http://localhost:5173`** to access the live warehouse dashboard.

---

## Repository Structure

```text
SIH-AMR/
│
├── allocator/          # Task allocation and task generation
├── comms/              # Inter-robot communication abstraction
├── dashboard/          # Backend API and web dashboard
├── experiments/        # Benchmarking and experiment runners
├── robot/              # Robot-local planning and coordination
├── sim/                # Multi-AMR simulation engine
├── tests/              # Automated tests
│
├── config.py           # Warehouse/grid configuration
├── interfaces.py       # Pluggable system interfaces
├── metrics.py          # Experiment metrics
├── models.py           # Shared data models
└── README.md
