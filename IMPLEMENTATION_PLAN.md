# 🛠️ AMR Multi-Part Sensor Simulation Implementation Plan

**Project:** Autonomous Mobile Robot (AMR) Fleet Coordination (`SIH26123`)  
**Repository:** `Autonomous-AMR-Fleet-Hub`  
**Architecture Guard:** Phase 0 Architecture Freeze (`RobotState` in `models.py` strictly preserved)

---

## 📌 Executive Summary

This document establishes the architecture, implementation progress, and roadmap for the multi-part sensor simulation expansion in the SIH26123 AMR fleet coordination prototype. 

All sensor simulation additions adhere to the Section 5.2 observer-only boundary:
* **Decoupled Architecture:** Core simulation modules (`robot`, `sim`, `allocator`, `comms`) never import `dashboard`.
* **Frozen Dataclass:** `RobotState` in [`ref_sih_amr/models.py`](models.py) remains frozen without adding sensor fields.
* **Dynamic Telemetry:** Sensor outputs are injected directly into the telemetry snapshot dictionary published by `TelemetryBus` and streamed over WebSockets to the digital twin frontend.

---

## 🗺️ Multi-Part Roadmap & Status

| Phase | Feature | Status | Description |
|---|---|---|---|
| **Phase 1** | **LiDAR Scan Emulation** | **COMPLETED** | 360° 16-ray obstacle scanning with dashboard HUD overlay |
| **Phase 2** | **Proximity Sensors** | Scheduled | Ultrasonic / IR bumper rangefinders for safety perimeter |
| **Phase 3** | **Odometry Drift & Wheel Slip** | Scheduled | Cumulative dead-reckoning drift & encoder noise |
| **Phase 4** | **RFID Tag Reading** | Scheduled | Ground checkpoint verification & drift zeroing |
| **Phase 5** | **IMU 6-DOF Tilt & Accel** | Scheduled | 3-axis accelerometer and gyro for ramp/jerk detection |
| **Phase 6** | **Motor Current & Load Torque** | Scheduled | Drive/lift power draw & predictive maintenance linkage |
| **Phase 7** | **Fine Docking Alignment** | Scheduled | Sub-centimeter optical guide for station handover |

---

## ✅ Feature 1: LiDAR Scan Emulation (COMPLETED)

### 1. Specification & Core Module
* **Location:** [`ref_sih_amr/robot/sensors.py`](ref_sih_amr/robot/sensors.py)
* **Class:** `SensorSimulator`
* **Method:** `simulate_lidar(position, heading, costmap, num_rays=16, max_range=6) -> List[float]`
* **Behavior:**
  - Casts `num_rays` (default 16) evenly-spaced rays across 360° starting from the AMR's current heading.
  - Steps outward cell-by-cell reusing `costmap.get_cell()` from the existing `GridMap` object (no duplicate obstacle maps).
  - Stops when hitting a blocked cell (`#`) or reaching `max_range`.
  - Returns a list of floating-point distance values in grid cell units.

### 2. Simulator Integration
* **Location:** [`ref_sih_amr/sim/simulator.py`](ref_sih_amr/sim/simulator.py)
* **Initialization (Line ~121):** Instantiates `self.sensor_simulator = SensorSimulator()` and calls `simulate_lidar()` for each AMR upon spawn.
* **Per-Tick Update (Line ~1240):** Runs `simulate_lidar()` on each active robot before telemetry publication to reflect updated positions and headings.
* **Snapshot Payload:**
  - Injected directly into the snapshot dict: `snapshot["lidar_scan"]` and per-robot `snapshot["robots"][id]["lidar_scan"]`.
  - Zero modifications made to `RobotState` in [`models.py`](ref_sih_amr/models.py).

### 3. Dashboard WebGL Visualization
* **Location:** [`ref_sih_amr/dashboard/web/index.html`](ref_sih_amr/dashboard/web/index.html)
* **Overlay:**
  - Renders 16 radiating laser lines around each AMR in Three.js WebGL space.
  - Lengths proportional to distance in world units (`dist * CELL_SIZE`).
  - **Dynamic Hazard Coloring:** Neutral/slate gray (`#94a3b8`) for open distance; switches to bright warning red (`#ef4444`) with heightened opacity when obstacles are closer than 1.5 cells.
  - Thin contour ring connecting the ray tips to display the scanned room boundary.
* **Controls:** Small **"Show LiDAR"** toggle button placed in the map viewport header (defaults to **ON**).

### 4. Verification Suite
* **Test Suite:** [`ref_sih_amr/tests/test_sensors.py`](ref_sih_amr/tests/test_sensors.py)
* **Status:** 44/44 tests passing (`pytest ref_sih_amr/tests/ -v`).
* Confirmed:
  - Architecture guard ([`test_architecture.py`](ref_sih_amr/tests/test_architecture.py)) passes.
  - `RobotState` schema freeze verified.
  - Exact ray distances, obstacles, and boundary detection verified.

---

## ⏳ Future Sensor Simulation Features (Upcoming Phases)

### 🔹 Feature 2: Proximity Sensors (Ultrasonic / IR)
* **Target Module:** `ref_sih_amr/robot/sensors.py`
* **Method:** `simulate_proximity(position, heading, costmap, zones=("front", "rear", "left", "right"), detection_range=1.2) -> Dict[str, float]`
* **Scope:**
  - Models discrete bumper ultrasonic/IR rangefinder cones around the chassis boundary.
  - Triggers emergency stop overrides when an obstacle or peer encroaches within $< 0.4\text{ m}$.
  - Dashboard: Semi-transparent hazard arc on the robot mesh showing proximity warning threshold.

### 🔹 Feature 3: Odometry Drift & Wheel Slip
* **Target Module:** `ref_sih_amr/robot/sensors.py`
* **Method:** `simulate_odometry_drift(actual_pos, actual_heading, velocity, slip_coeff=0.02, noise_var=0.005) -> Tuple[Tuple[float, float], float]`
* **Scope:**
  - Introduces cumulative Dead-Reckoning (DR) error as wheels slip over polished concrete surfaces.
  - Maintains separation between true ground truth position and AMR local estimated position.
  - Feeds into existing `localization_confidence` metric in telemetry.

### 🔹 Feature 4: RFID Tag Reading & Waypoint Verification
* **Target Module:** `ref_sih_amr/robot/sensors.py`
* **Method:** `simulate_rfid_reader(position, rfid_map, read_radius=0.6) -> Optional[str]`
* **Scope:**
  - Emulates passive floor-embedded RFID transponder tags at aisle intersections and station entries.
  - When an AMR passes over an RFID tag, it resets accumulated odometry drift to zero.
  - Logs `RFID_CHECKPOINT_VERIFIED` events to `EventLog`.

### 🔹 Feature 5: IMU 6-DOF Acceleration & Tilt Sensing
* **Target Module:** `ref_sih_amr/robot/sensors.py`
* **Method:** `simulate_imu(velocity, target_velocity, angular_velocity, floor_grade_deg=0.0) -> Dict[str, float]`
* **Scope:**
  - Emulates 3-axis accelerometer ($a_x, a_y, a_z$) and 3-axis gyroscope ($g_x, g_y, g_z$).
  - Detects rapid braking jerk, centrifugal roll during 90° turns, and floor ramp inclination.

### 🔹 Feature 6: Motor Current & Dynamic Load Torque Sensing
* **Target Module:** `ref_sih_amr/robot/sensors.py`
* **Method:** `simulate_motor_current(velocity, has_cargo, cargo_mass_kg=50.0) -> float`
* **Scope:**
  - Calculates dynamic drive motor current draw (Amperes) based on acceleration, speed, and loaded cargo state.
  - Directly correlates with predictive maintenance models (`ref_sih_amr/predictive_maint.py`).
  - Detects motor anomalies and mechanical resistance spikes.

### 🔹 Feature 7: Fine Docking Alignment Sensors (Optical / Laser)
* **Target Module:** `ref_sih_amr/robot/sensors.py`
* **Method:** `simulate_dock_alignment(position, dock_cell, target_tol=0.05) -> Dict[str, Union[float, bool]]`
* **Scope:**
  - High-precision optical alignment sensing for docking at pickup (`P`), dropoff (`D`), and charging (`C`) stations.
  - Generates cross-track lateral offset and angular yaw error for terminal docking maneuvers.
