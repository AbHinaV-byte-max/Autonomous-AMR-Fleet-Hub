"""
Phase 6A edge AI tests — deterministic policy, safety override (Section 9.2).
"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from robot.edge_policy import (
    DeterministicPolicy, SafeEdgePolicy, PolicyFeatures,
    ACTION_CONTINUE, ACTION_YIELD, ACTION_WAIT, ACTION_REROUTE,
    ACTION_DEGRADED_MODE, ACTIONS
)


def _feat(**kwargs):
    defaults = dict(
        dist_to_nearest_peer=5.0, relative_velocity=0.0, time_to_conflict=10.0,
        intersection_occupancy=0.0, queue_length=0.0, local_obstacle_flag=0.0,
        task_urgency=3.0, battery=80.0, peer_comm_freshness=0.5
    )
    defaults.update(kwargs)
    return PolicyFeatures(**defaults)


# ── Deterministic policy unit tests ──────────────────────────────────────────

def test_deterministic_degraded_on_stale_comms():
    pol = DeterministicPolicy()
    f = _feat(peer_comm_freshness=5.0)
    assert pol.decide(f) == ACTION_DEGRADED_MODE


def test_deterministic_reroute_on_obstacle():
    pol = DeterministicPolicy()
    f = _feat(local_obstacle_flag=1.0, peer_comm_freshness=0.5)
    assert pol.decide(f) == ACTION_REROUTE


def test_deterministic_wait_on_close_conflict():
    pol = DeterministicPolicy()
    f = _feat(time_to_conflict=1.0, dist_to_nearest_peer=1.5, task_urgency=2.0)
    assert pol.decide(f) in (ACTION_WAIT, ACTION_YIELD)


def test_deterministic_continue_when_clear():
    pol = DeterministicPolicy()
    f = _feat()
    assert pol.decide(f) == ACTION_CONTINUE


# ── Safety override test (Section 9.2) ───────────────────────────────────────

class _MockOnnx:
    """Simulates an ONNX model that always suggests CONTINUE."""
    available = True

    def decide(self, features):
        return ACTION_CONTINUE, 0.5


def test_safety_override_blocks_unsafe_ml_suggestion():
    """
    Feed SafeEdgePolicy an adversarial scenario:
    - ONNX model suggests CONTINUE
    - safe_actions does NOT include CONTINUE (cell is reserved)
    Assert: the robot does NOT take CONTINUE; deterministic fallback is used.
    """
    policy = SafeEdgePolicy()
    policy._onnx = _MockOnnx()   # inject adversarial ML model

    # Feature vector that deterministic policy would decide ACTION_WAIT
    f = _feat(time_to_conflict=1.0, dist_to_nearest_peer=1.0, task_urgency=2.0)

    # Safe actions do not include CONTINUE — it would violate a reservation
    safe = {ACTION_WAIT, ACTION_YIELD, ACTION_REROUTE}
    action = policy.decide(f, safe_actions=safe)

    assert action != ACTION_CONTINUE, (
        "Safety override failed — robot took CONTINUE despite reservation conflict!"
    )
    print(f"Safety override test passed — action taken: {action}")


if __name__ == "__main__":
    test_deterministic_degraded_on_stale_comms()
    test_deterministic_reroute_on_obstacle()
    test_deterministic_wait_on_close_conflict()
    test_deterministic_continue_when_clear()
    test_safety_override_blocks_unsafe_ml_suggestion()
    print("\n=== All edge AI tests passed ===")
