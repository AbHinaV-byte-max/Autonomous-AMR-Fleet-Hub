"""
Frozen data models for the multi-AMR warehouse coordination simulator.

Phase 0 architecture freeze — do not modify signatures without team agreement.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class RobotStatus(enum.Enum):
    """Operating status of an individual robot."""

    IDLE = "IDLE"
    MOVING = "MOVING"
    WAITING = "WAITING"
    REROUTING = "REROUTING"
    DEGRADED = "DEGRADED"
    OFFLINE = "OFFLINE"


class Intent(enum.Enum):
    """Broadcast intent type carried in an IntentMessage."""

    MOVE = "MOVE"
    YIELD = "YIELD"
    WAIT = "WAIT"
    REROUTE = "REROUTE"


class TaskStatus(enum.Enum):
    """Lifecycle status of a warehouse task."""

    QUEUED = "QUEUED"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    UNAVAILABLE = "UNAVAILABLE"
    RECOVERABLE = "RECOVERABLE"


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------

@dataclass
class RobotState:
    """Full observable state vector for a single robot."""

    robot_id: str
    timestamp: float
    position: Tuple[float, float]
    heading: float
    velocity: float
    battery: float
    current_task_id: Optional[str]
    task_priority: int
    planned_path: List[Tuple[int, int]] = field(default_factory=list)
    reserved_cells: List[Tuple[Tuple[int, int], float]] = field(default_factory=list)
    status: RobotStatus = RobotStatus.IDLE
    localization_confidence: float = 1.0
    communication_quality: float = 1.0


@dataclass
class IntentMessage:
    """Lightweight broadcast message used for peer-to-peer coordination."""

    robot_id: str
    seq: int
    timestamp: float
    position: Tuple[float, float]
    velocity: float
    intent: Intent
    next_intersection: Optional[Tuple[int, int]]
    task_id: Optional[str]
    priority: int
    planned_path: List[Tuple[int, int]] = field(default_factory=list)
    reservation_horizon: float = 0.0
    battery: float = 100.0
    waiting_on: Optional[str] = None
    heartbeat: float = 0.0
    auth_tag: str = ""


@dataclass
class Task:
    """A single pick-and-place warehouse task."""

    task_id: str
    pickup_cell: Tuple[int, int]
    dropoff_cell: Tuple[int, int]
    priority: int
    status: TaskStatus = TaskStatus.QUEUED
    created_at: float = 0.0
    assigned_robot_id: Optional[str] = None
