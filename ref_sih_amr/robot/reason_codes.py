"""
Defense-Grade Explainable Reason Codes for Autonomous Mobile Robots (SIH 26123 - BEL).
Zero black-box ML: every routing, yield, wait, and speed adjustment decision is logged
with auditable, standardized reason codes.
"""

from enum import Enum


class ReasonCode(str, Enum):
    # Nominal Navigation
    RC_TASK_ASSIGNED = "RC_TASK_ASSIGNED"
    RC_MOVING_NOMINAL = "RC_MOVING_NOMINAL"
    RC_TASK_PICKUP_ARRIVED = "RC_TASK_PICKUP_ARRIVED"
    RC_TASK_DROPOFF_COMPLETE = "RC_TASK_DROPOFF_COMPLETE"

    # Decentralized Conflict Arbitration
    RC_CONTINUE_RIGHT_OF_WAY = "RC_CONTINUE_RIGHT_OF_WAY"
    RC_YIELD_INTERSECTION_CONFLICT = "RC_YIELD_INTERSECTION_CONFLICT"
    RC_YIELD_HEAD_ON_SWAP = "RC_YIELD_HEAD_ON_SWAP"
    RC_WAIT_CLEARANCE = "RC_WAIT_CLEARANCE"
    RC_REROUTE_DYNAMIC = "RC_REROUTE_DYNAMIC"
    RC_REROUTE_BLOCKED = "RC_REROUTE_BLOCKED"

    # Industry-Alignment: Proactive Congestion & Safety
    RC_PROACTIVE_REROUTE_CONGESTION = "RC_PROACTIVE_REROUTE_CONGESTION"
    RC_SAFETY_HUMAN_ZONE_SPEED_DERATING = "RC_SAFETY_HUMAN_ZONE_SPEED_DERATING"

    # Health & Maintenance
    RC_BATTERY_LOW_RETURN_TO_CHARGE = "RC_BATTERY_LOW_RETURN_TO_CHARGE"
    RC_MAINTENANCE_SERVICE_DUE = "RC_MAINTENANCE_SERVICE_DUE"

    # Hybrid AI Layer (Research-Aligned: RL-guided Prioritized Planning pattern)
    # Applied ONLY as a priority-weight adjustment before the deterministic arbiter runs.
    # The deterministic DSS arbiter's safety checks are never bypassed.
    RC_HYBRID_PRIORITY_ADJUSTED = "RC_HYBRID_PRIORITY_ADJUSTED"
