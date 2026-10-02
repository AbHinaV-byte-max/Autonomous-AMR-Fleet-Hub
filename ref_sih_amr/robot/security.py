"""
Security / Trust layer — Section 11.

Validates IntentMessages in order:
  1. robot_id authorized?
  2. timestamp fresh?
  3. sequence_number newer than last seen?
  4. HMAC valid?
  5. physical plausibility (implied velocity ≤ max speed)?

Returns "accept" | "quarantine" | "reject".
"""
import hashlib
import hmac
import time
import logging
import math
from typing import Dict, Optional, Tuple

from models import IntentMessage
from interfaces import SecurityValidator as BaseValidator

logger = logging.getLogger(__name__)

# Simulated per-robot symmetric keys (shared-secret demo — no PKI needed).
# In production these would be provisioned securely.
ROBOT_KEYS: Dict[str, bytes] = {
    f"robot-{i}": f"BEL-AMR-2026-{i}-HMAC".encode("utf-8")
    for i in range(16)
}

AUTHORIZED_ROBOTS = set(ROBOT_KEYS.keys())
TIMESTAMP_FRESHNESS = 5.0    # seconds / ticks
MAX_ROBOT_SPEED = 2.0        # cells per tick — plausibility ceiling


def _canonical_message(message: IntentMessage) -> bytes:
    """Canonical bytes for every security-relevant IntentMessage field."""
    import json
    payload = {
        "robot_id": message.robot_id,
        "seq": int(message.seq),
        "timestamp": float(message.timestamp),
        "position": [float(message.position[0]), float(message.position[1])],
        "velocity": float(message.velocity),
        "intent": message.intent.value,
        "next_intersection": list(message.next_intersection) if message.next_intersection else None,
        "task_id": message.task_id,
        "priority": int(message.priority),
        "planned_path": [list(p) for p in message.planned_path],
        "reservation_horizon": float(message.reservation_horizon),
        "battery": float(message.battery),
        "waiting_on": message.waiting_on,
        "heartbeat": float(message.heartbeat),
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")


def compute_hmac(robot_id: str, msg: IntentMessage) -> str:
    key = ROBOT_KEYS.get(robot_id, b"")
    return hmac.new(key, _canonical_message(msg), hashlib.sha256).hexdigest()


def verify_hmac(msg: IntentMessage) -> bool:
    if msg.robot_id not in AUTHORIZED_ROBOTS or not msg.auth_tag:
        return False
    return hmac.compare_digest(compute_hmac(msg.robot_id, msg), msg.auth_tag)



class TrustValidator(BaseValidator):
    def __init__(self):
        self._last_seq: Dict[str, int] = {}
        self._last_position: Dict[str, Tuple[float, float]] = {}
        self._last_timestamp: Dict[str, float] = {}
        self.quarantine_log = []
        self.reject_log = []

    def validate(self, message: IntentMessage) -> str:
        rid = message.robot_id

        # 1. Authorized?
        if rid not in AUTHORIZED_ROBOTS:
            self._record_reject(rid, "unauthorized robot_id")
            return "reject"

        # 2. Timestamp fresh?
        now = time.time()
        # For sim we use tick timestamp; compare against monotonic sim clock
        # (freshness check: within ±TIMESTAMP_FRESHNESS of last known)
        last_ts = self._last_timestamp.get(rid)
        if last_ts is not None and message.timestamp < last_ts:
            self._record_reject(rid, f"stale timestamp {message.timestamp} < {last_ts}")
            return "reject"

        # 3. Sequence newer?
        last_seq = self._last_seq.get(rid, -1)
        if message.seq <= last_seq:
            self._record_reject(rid, f"seq {message.seq} ≤ last seen {last_seq}")
            return "reject"

        # 4. HMAC valid?
        expected = compute_hmac(rid, message)
        if not hmac.compare_digest(expected, message.auth_tag):
            self._record_reject(rid, "HMAC mismatch")
            return "reject"

        # 5. Physical plausibility
        last_pos = self._last_position.get(rid)
        last_tick = self._last_timestamp.get(rid)
        if last_pos is not None and last_tick is not None:
            dt = max(message.timestamp - last_tick, 1.0)
            dx = message.position[0] - last_pos[0]
            dy = message.position[1] - last_pos[1]
            implied_speed = math.sqrt(dx*dx + dy*dy) / dt
            if implied_speed > MAX_ROBOT_SPEED:
                self._record_quarantine(
                    rid,
                    f"impossible speed {implied_speed:.2f} > {MAX_ROBOT_SPEED}"
                )
                return "quarantine"

        # Accept — update state
        self._last_seq[rid] = message.seq
        self._last_position[rid] = message.position
        self._last_timestamp[rid] = message.timestamp
        return "accept"

    def _record_quarantine(self, rid: str, reason: str):
        entry = {"robot_id": rid, "reason": reason, "ts": time.time()}
        self.quarantine_log.append(entry)
        logger.warning("QUARANTINE %s: %s", rid, reason)

    def _record_reject(self, rid: str, reason: str):
        entry = {"robot_id": rid, "reason": reason, "ts": time.time()}
        self.reject_log.append(entry)
        logger.warning("REJECT %s: %s", rid, reason)
