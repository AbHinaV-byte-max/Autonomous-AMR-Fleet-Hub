"""
Edge AI local policy — Section 9.

Decision pipeline (in order):
  1. DeterministicPolicy.decide()  — always ships, always authoritative
  2. OnnxPolicy.decide()           — optional ML suggestion
  3. Safety override               — if ML suggests unsafe action,
                                     deterministic result wins (Section 9.2)

Feature vector (9 floats, in this exact order):
  [dist_to_nearest_peer, relative_velocity, time_to_conflict,
   intersection_occupancy, queue_length, local_obstacle_flag,
   task_urgency, battery, peer_comm_freshness]

Output actions:
  CONTINUE / YIELD / WAIT / REROUTE / DEGRADED_MODE
"""
from __future__ import annotations
import time
import logging
from dataclasses import dataclass
from typing import Optional
import numpy as np

logger = logging.getLogger(__name__)

ACTION_CONTINUE      = "CONTINUE"
ACTION_YIELD         = "YIELD"
ACTION_WAIT          = "WAIT"
ACTION_REROUTE       = "REROUTE"
ACTION_DEGRADED_MODE = "DEGRADED_MODE"

ACTIONS = [ACTION_CONTINUE, ACTION_YIELD, ACTION_WAIT, ACTION_REROUTE, ACTION_DEGRADED_MODE]
ACTION_INDEX = {a: i for i, a in enumerate(ACTIONS)}


@dataclass
class PolicyFeatures:
    dist_to_nearest_peer: float
    relative_velocity: float
    time_to_conflict: float
    intersection_occupancy: float
    queue_length: float
    local_obstacle_flag: float   # 0.0 or 1.0
    task_urgency: float
    battery: float
    peer_comm_freshness: float   # ticks since last peer message (lower = fresher)

    def to_array(self) -> np.ndarray:
        return np.array([
            self.dist_to_nearest_peer,
            self.relative_velocity,
            self.time_to_conflict,
            self.intersection_occupancy,
            self.queue_length,
            self.local_obstacle_flag,
            self.task_urgency,
            self.battery,
            self.peer_comm_freshness,
        ], dtype=np.float32)


class DeterministicPolicy:
    """
    Rule-based baseline policy — Section 14 "Deterministic local policy if ML
    not ready".  Ships regardless of whether OnnxPolicy is available.
    """

    def decide(self, features: PolicyFeatures) -> str:
        if features.peer_comm_freshness > 3.0:
            return ACTION_DEGRADED_MODE
        if features.local_obstacle_flag > 0.5:
            return ACTION_REROUTE
        if features.time_to_conflict < 1.5 and features.dist_to_nearest_peer < 2.0:
            if features.task_urgency > 5.0:
                return ACTION_YIELD
            return ACTION_WAIT
        if features.intersection_occupancy > 0.5:
            return ACTION_YIELD
        return ACTION_CONTINUE


class OnnxPolicy:
    """
    ONNX-runtime inference wrapper.  Measures and logs latency for
    EDGE_INFERENCE_LATENCY metric.
    """

    def __init__(self, model_path: str):
        try:
            import onnxruntime as ort
            self._session = ort.InferenceSession(model_path)
            self._input_name = self._session.get_inputs()[0].name
            self._available = True
            logger.info("ONNX edge policy loaded from %s", model_path)
        except Exception as e:
            logger.warning("OnnxPolicy unavailable: %s", e)
            self._available = False

    @property
    def available(self) -> bool:
        return self._available

    def decide(self, features: PolicyFeatures) -> tuple[str, float]:
        """Returns (action, latency_ms)."""
        t0 = time.perf_counter()
        arr = features.to_array().reshape(1, -1)
        out = self._session.run(None, {self._input_name: arr})
        latency_ms = (time.perf_counter() - t0) * 1000.0
        action_idx = int(np.argmax(out[0]))
        return ACTIONS[action_idx], latency_ms


class SafeEdgePolicy:
    """
    Composed policy with mandatory safety override (Section 9.2).
    ML suggestion is applied only when it is both available AND safe.
    Unsafe suggestions are overridden and logged.
    """

    def __init__(self, model_path: Optional[str] = None):
        self._deterministic = DeterministicPolicy()
        self._onnx: Optional[OnnxPolicy] = None
        if model_path:
            self._onnx = OnnxPolicy(model_path)
        self.last_latency_ms: float = 0.0

    def decide(self, features: PolicyFeatures, safe_actions: set[str]) -> str:
        """
        safe_actions: set of actions that would NOT violate current reservations.
        If the ONNX suggestion is not in safe_actions, the deterministic result
        is used instead (hard override — Section 9.2).
        """
        det_action = self._deterministic.decide(features)

        if self._onnx and self._onnx.available:
            ml_action, lat = self._onnx.decide(features)
            self.last_latency_ms = lat

            if ml_action in safe_actions:
                return ml_action
            else:
                # SAFETY OVERRIDE — ML would cause reservation violation
                logger.warning(
                    "Safety override: ML suggested %s (unsafe), using deterministic %s",
                    ml_action, det_action
                )
                return det_action

        return det_action
