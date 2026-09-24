"""
Phase 6B Security tests — Section 11.3 anomaly demo.
"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

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
    
    # Setup initial state
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
        auth_tag=""
    )
    msg1.auth_tag = compute_hmac("robot-0", msg1)
    
    assert validator.validate(msg1) == "accept", "Initial valid message rejected"
    
    # 2. Inject impossible teleport message a split second later
    msg2 = IntentMessage(
        robot_id="robot-0",
        seq=2,
        timestamp=msg1.timestamp + 0.1,  # 0.1 seconds later
        position=(100.0, 100.0),         # 141 cells away — impossible!
        velocity=1.0,
        intent=Intent.MOVE,
        next_intersection=None,
        task_id=None,
        priority=0,
        auth_tag=""
    )
    msg2.auth_tag = compute_hmac("robot-0", msg2)
    
    result = validator.validate(msg2)
    
    assert result == "quarantine", f"Expected quarantine due to impossible speed, got {result}"
    print("Security anomaly demo passed: impossible teleport quarantined.")


if __name__ == "__main__":
    test_anomaly_injection_teleport()
    print("\n=== All security tests passed ===")
