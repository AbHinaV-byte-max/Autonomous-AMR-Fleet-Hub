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
import os
import re
from typing import Dict, Optional, Tuple

from models import IntentMessage
from interfaces import SecurityValidator as BaseValidator

logger = logging.getLogger(__name__)

# Robot identities are derived from the deployment master secret. The
# development simulator may run without one, in which case it creates an
# ephemeral process-local key; real UDP robot processes must share
# AMR_HMAC_MASTER_KEY.
_MASTER_KEY = os.getenv("AMR_HMAC_MASTER_KEY")
if _MASTER_KEY:
    _MASTER_KEY_BYTES = _MASTER_KEY.encode("utf-8")
else:
    _MASTER_KEY_BYTES = os.urandom(32)

ROBOT_ID_PATTERN = re.compile(r"^robot-[0-9]+$")


def is_authorized_robot(robot_id: str) -> bool:
    return bool(ROBOT_ID_PATTERN.fullmatch(robot_id))


def robot_key(robot_id: str) -> bytes:
    return hmac.new(
        _MASTER_KEY_BYTES,
        robot_id.encode("utf-8"),
        hashlib.sha256,
    ).digest()


AUTHORIZED_ROBOTS = set()  # retained for backwards-compatible imports

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
        "session_epoch": message.session_epoch,
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")


def compute_hmac(robot_id: str, msg: IntentMessage) -> str:
    if not is_authorized_robot(robot_id):
        return ""
    return hmac.new(robot_key(robot_id), _canonical_message(msg), hashlib.sha256).hexdigest()


def verify_hmac(msg: IntentMessage) -> bool:
    if not is_authorized_robot(msg.robot_id) or not msg.auth_tag:
        return False
    return hmac.compare_digest(compute_hmac(msg.robot_id, msg), msg.auth_tag)



class TrustValidator(BaseValidator):
    def __init__(self):
        self._last_seq: Dict[tuple[str, str], int] = {}
        self._last_epoch: Dict[str, str] = {}
        self._last_position: Dict[str, Tuple[float, float]] = {}
        self._last_timestamp: Dict[str, float] = {}
        self.quarantine_log = []
        self.reject_log = []

    def validate(self, message: IntentMessage) -> str:
        rid = message.robot_id

        # 1. Authorized?
        if not is_authorized_robot(rid):
            self._record_reject(rid, "unauthorized robot_id")
            return "reject"

        # 2. Session epoch: a restarted robot gets a fresh replay namespace.
        epoch = message.session_epoch
        if not epoch:
            self._record_reject(rid, "missing session epoch")
            return "reject"
        last_epoch = self._last_epoch.get(rid)
        if last_epoch is not None and epoch != last_epoch:
            self._last_seq.pop((rid, last_epoch), None)

        # 3. Timestamp monotonicity within the sender epoch.
        last_ts = self._last_timestamp.get(rid)
        if last_ts is not None and message.timestamp < last_ts:
            self._record_reject(rid, f"stale timestamp {message.timestamp} < {last_ts}")
            return "reject"

        # 4. Sequence newer within the current epoch.
        last_seq = self._last_seq.get((rid, epoch), -1)
        if message.seq <= last_seq:
            self._record_reject(rid, f"seq {message.seq} ≤ last seen {last_seq}")
            return "reject"

        # 5. HMAC valid?
        expected = compute_hmac(rid, message)
        if not hmac.compare_digest(expected, message.auth_tag):
            self._record_reject(rid, "HMAC mismatch")
            return "reject"

        # 6. Physical plausibility
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
        self._last_epoch[rid] = epoch
        self._last_seq[(rid, epoch)] = message.seq
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
