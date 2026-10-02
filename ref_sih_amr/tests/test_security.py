"""Phase 6B Security tests — Section 11.3 anomaly demo."""
import sys, os
import time
from models import IntentMessage, Intent
from robot.security import TrustValidator, compute_hmac


def test_anomaly_injection_teleport():
    """
    Section 11.3 demo: Inject a message claiming robot teleported to an
    impossible position.
    Assert:
      1. Signature & freshness pass
      2. Physical plausibility check fails
      3. Message is quarantined (or robot marked degraded)
    """
    validator = TrustValidator()

    msg1 = IntentMessage(
        robot_id="robot-0",
        seq=1,
        timestamp=time.time(),
        position=(0.0, 0.0),
        velocity=1.0,
        intent=Intent.MOVE,
        next_intersection=None,
        task_id=None,
        priority=0,
        auth_tag="",
        session_epoch="test-epoch",
    )
    msg1.auth_tag = compute_hmac("robot-0", msg1)
    assert validator.validate(msg1) == "accept", "Initial valid message rejected"

    msg2 = IntentMessage(
        robot_id="robot-0",
        seq=2,
        timestamp=msg1.timestamp + 0.1,
        position=(100.0, 100.0),
        velocity=1.0,
        intent=Intent.MOVE,
        next_intersection=None,
        task_id=None,
        priority=0,
        auth_tag="",
        session_epoch="test-epoch",
    )
    msg2.auth_tag = compute_hmac("robot-0", msg2)
    result = validator.validate(msg2)
    assert result == "quarantine", f"Expected quarantine due to impossible speed, got {result}"
    print("Security anomaly demo passed: impossible teleport quarantined.")


if __name__ == "__main__":
    test_anomaly_injection_teleport()
    print("\n=== All security tests passed ===")


def test_hmac_covers_planned_path_and_rejects_tampering():
    msg = IntentMessage(
        robot_id="robot-0", seq=1, timestamp=time.time(), position=(1.0, 1.0),
        velocity=1.0, intent=Intent.MOVE, next_intersection=None, task_id="T1",
        priority=1, planned_path=[(1, 1), (2, 1)], auth_tag="", session_epoch="test-epoch"
    )
    msg.auth_tag = compute_hmac("robot-0", msg)
    msg.planned_path = [(1, 1), (9, 9)]
    assert TrustValidator().validate(msg) == "reject"
