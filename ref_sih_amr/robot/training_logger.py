"""
Training data logger — instruments LocalTaskManager ticks.
Emits (features, decision, outcome) rows to logs/training_data.jsonl.
"""
import json
import os
import time
from pathlib import Path
from robot.edge_policy import PolicyFeatures, DeterministicPolicy, ACTIONS

LOG_DIR = Path(__file__).parent.parent / "logs"
LOG_FILE = LOG_DIR / "training_data.jsonl"

_policy = DeterministicPolicy()
_fh = None


def _get_fh():
    global _fh
    if _fh is None:
        LOG_DIR.mkdir(exist_ok=True)
        _fh = open(LOG_FILE, "a", encoding="utf-8")
    return _fh


def log_decision(features: PolicyFeatures, decision: str, outcome: str):
    """Appends one training row.  outcome is filled in retrospectively by the caller."""
    row = {
        "ts": time.time(),
        "features": features.to_array().tolist(),
        "decision": decision,
        "outcome": outcome,
    }
    _get_fh().write(json.dumps(row) + "\n")
    _get_fh().flush()


def close():
    global _fh
    if _fh:
        _fh.close()
        _fh = None
