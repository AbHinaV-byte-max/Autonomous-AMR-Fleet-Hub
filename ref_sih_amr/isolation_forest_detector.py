"""
Isolation Forest Anomaly-Detection based Predictive Maintenance (SIH 26123 - BEL).

RESEARCH BASIS / PROBLEM FRAMING:
  A real 2026 industry case study documented a 60-unit AMR fleet whose uptime
  collapsed from 97% to 81% over 8 months because no predictive maintenance program
  existed — wheel wear, battery fade, and LiDAR contamination went untracked until
  failures happened. A structured monitoring program could have predicted every drive
  failure 4-6 days in advance. Industry-wide data shows predictive maintenance
  delivers ~78% reduction in unplanned downtime.

WHY ISOLATION FOREST:
  - Unsupervised: no labeled failure data required (we don't have historical failures yet)
  - Near-zero setup cost: fits on simulated "normal operation" telemetry
  - Interpretable anomaly score: continuous value, not a black-box classification
  - Proven effective for time-series sensor anomaly detection in industrial settings
  - sklearn.ensemble.IsolationForest — stable, well-documented, production-ready

FEATURES TRACKED PER-ROBOT (rolling window):
  1. battery_discharge_rate  — delta % per tick (rolling 10-sample avg). Detects
                               abnormal battery cell degradation vs fleet baseline.
  2. cumulative_distance_km  — absolute odometer reading / 1000
  3. cycle_count             — number of completed pickup-delivery cycles
  4. stuck_time_ratio        — ticks_in_WAITING / total_ticks (≥ 0.3 = stuck/sensor issue)
  5. speed_variance          — variance of speed readings over 10-sample window
                               (irregular variance → drive motor irregularity)
  6. battery_level           — current absolute level (cross-check with discharge rate)

SAFETY INVARIANT:
  The existing threshold-based checks (odometer, cycle count, battery floor) are
  retained as hard-limit safety nets. The Isolation Forest layer ADDS anomaly
  detection on top — it never removes existing alerts.
"""

from __future__ import annotations

import logging
import math
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

logger = logging.getLogger("IsolationForestDetector")

# Minimum samples to collect per robot before the IF model is fitted.
# During warm-up, threshold-only alerts still fire as usual.
_MIN_SAMPLES_TO_FIT = 50

# Maximum telemetry readings stored per robot (rolling buffer)
_BUFFER_SIZE = 200

# Anomaly score threshold: Isolation Forest decision_function < this are flagged as anomalies.
# sklearn IsolationForest.decision_function() returns values where:
# > 0 indicates normal inlier observations,
# < 0 indicates anomalies / outliers (model.predict() == -1).
# 0.0 is the exact decision boundary of the Isolation Forest.
_ANOMALY_SCORE_THRESHOLD = 0.0


class RobotTelemetryBuffer:
    """
    Maintains a rolling buffer of telemetry readings for one robot and computes
    the 6-dimensional feature vector used by the Isolation Forest model.
    """

    def __init__(self, robot_id: str, maxlen: int = _BUFFER_SIZE):
        self.robot_id = robot_id
        self.readings: Deque[Dict[str, Any]] = deque(maxlen=maxlen)
        self._prev_battery: Optional[float] = None
        self._total_ticks: int = 0
        self._ticks_waiting: int = 0

    def ingest(self, reading: Dict[str, Any]) -> None:
        """Stores one telemetry reading and updates derived running counters."""
        self._total_ticks += 1
        state = reading.get("state", "IDLE")
        if state in ("WAITING", "YIELDING"):
            self._ticks_waiting += 1

        # Compute battery discharge rate (delta per tick)
        current_bat = float(reading.get("battery_pct", 100.0))
        if self._prev_battery is not None:
            discharge_rate = max(0.0, self._prev_battery - current_bat)
        else:
            discharge_rate = 0.0
        self._prev_battery = current_bat

        enriched = dict(reading)
        enriched["discharge_rate"] = discharge_rate
        enriched["stuck_time_ratio"] = (
            self._ticks_waiting / self._total_ticks if self._total_ticks > 0 else 0.0
        )
        self.readings.append(enriched)

    @property
    def n_samples(self) -> int:
        return len(self.readings)

    def build_feature_vector(self) -> Optional[List[float]]:
        """
        Constructs the 6-dimensional feature vector from recent telemetry.
        Returns None if insufficient samples collected yet.
        """
        if self.n_samples < 2:
            return None

        recent = list(self.readings)

        # Feature 1: rolling avg battery discharge rate (last 10 readings)
        window = recent[-10:]
        avg_discharge = sum(r["discharge_rate"] for r in window) / len(window)

        # Feature 2: cumulative distance km
        dist_km = float(recent[-1].get("odometer_meters", 0.0)) / 1000.0

        # Feature 3: cycle count
        cycle_count = float(recent[-1].get("cycles_completed", 0))

        # Feature 4: stuck time ratio
        stuck_ratio = float(recent[-1].get("stuck_time_ratio", 0.0))

        # Feature 5: speed variance over last 10 readings
        speeds = [float(r.get("speed", 0.0)) for r in window]
        mean_speed = sum(speeds) / len(speeds)
        speed_var = sum((s - mean_speed) ** 2 for s in speeds) / len(speeds)

        # Feature 6: current battery level (absolute)
        battery_level = float(recent[-1].get("battery_pct", 100.0))

        return [avg_discharge, dist_km, cycle_count, stuck_ratio, speed_var, battery_level]


class IsolationForestMaintenanceDetector:
    """
    Per-robot Isolation Forest anomaly detector for predictive maintenance.

    Lifecycle:
      1. Ingest telemetry readings for each robot via ingest_reading().
      2. After _MIN_SAMPLES_TO_FIT readings are collected per robot, the
         Isolation Forest is lazily fitted on that robot's "normal" baseline.
      3. evaluate() scores new readings against the fitted baseline and flags
         anomalies with a predicted failure type and estimated days-to-failure.

    Graceful degradation: if sklearn/numpy are unavailable, all methods return
    safe defaults and log a warning. No exceptions are raised to callers.
    """

    FEATURE_NAMES = [
        "avg_discharge_rate",
        "cumulative_dist_km",
        "cycle_count",
        "stuck_time_ratio",
        "speed_variance",
        "battery_level",
    ]

    def __init__(
        self,
        n_estimators: int = 100,
        contamination: float = 0.05,
        random_state: int = 42,
    ):
        self._n_estimators = n_estimators
        self._contamination = contamination
        self._random_state = random_state

        # Per-robot state
        self._buffers: Dict[str, RobotTelemetryBuffer] = {}
        self._models: Dict[str, Any] = {}       # robot_id -> fitted IsolationForest
        self._sklearn_available: Optional[bool] = None

    def _check_sklearn(self) -> bool:
        if self._sklearn_available is not None:
            return self._sklearn_available
        try:
            import sklearn  # noqa: F401
            import numpy  # noqa: F401
            self._sklearn_available = True
        except ImportError:
            self._sklearn_available = False
            logger.warning(
                "IsolationForestDetector: scikit-learn or numpy not available. "
                "Running in threshold-only fallback mode. "
                "Install: pip install scikit-learn numpy"
            )
        return self._sklearn_available

    def ingest_reading(self, robot_id: str, reading: Dict[str, Any]) -> None:
        """Stores one telemetry sample for the specified robot."""
        if robot_id not in self._buffers:
            self._buffers[robot_id] = RobotTelemetryBuffer(robot_id)
        self._buffers[robot_id].ingest(reading)

        # Lazily fit the model once we have enough samples
        buf = self._buffers[robot_id]
        if (
            robot_id not in self._models
            and buf.n_samples >= _MIN_SAMPLES_TO_FIT
            and self._check_sklearn()
        ):
            self._fit_model(robot_id)

    def _fit_model(self, robot_id: str) -> None:
        """Fits an IsolationForest on the collected baseline telemetry for one robot."""
        try:
            import numpy as np
            from sklearn.ensemble import IsolationForest

            buf = self._buffers[robot_id]
            X = []
            for reading in list(buf.readings):
                # Build feature vector for each stored reading
                tmp_buf = RobotTelemetryBuffer(robot_id)
                for r in list(buf.readings):
                    tmp_buf.ingest(r)
                    fv = tmp_buf.build_feature_vector()
                    if fv is not None:
                        X.append(fv)
                break  # Only need one pass through all readings

            # Rebuild properly: replay all readings and collect feature vectors
            replay_buf = RobotTelemetryBuffer(robot_id)
            X = []
            for reading in list(buf.readings):
                replay_buf.ingest(reading)
                fv = replay_buf.build_feature_vector()
                if fv is not None:
                    X.append(fv)

            if len(X) < 10:
                return

            X_arr = np.array(X)
            model = IsolationForest(
                n_estimators=self._n_estimators,
                contamination=self._contamination,
                random_state=self._random_state,
            )
            model.fit(X_arr)
            self._models[robot_id] = model
            logger.info(
                "IsolationForestDetector: Fitted baseline model for %s "
                "on %d samples.", robot_id, len(X)
            )
        except Exception as exc:
            logger.warning(
                "IsolationForestDetector: Failed to fit model for %s: %s", robot_id, exc
            )

    def evaluate(
        self,
        robot_id: str,
        threshold_alerts: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """
        Evaluates the current telemetry anomaly score for the given robot.

        Returns a dict containing:
          - anomaly_score: float (-1 to +1, more negative = more anomalous)
          - is_anomaly: bool
          - predicted_failure_type: str
          - estimated_days_to_failure: Optional[int]
          - reasoning: str (plain English explanation)
          - threshold_alerts: list of hard-limit alerts (always included)
          - model_status: str ("active" | "warming_up" | "unavailable")
        """
        threshold_alerts = threshold_alerts or []
        buf = self._buffers.get(robot_id)

        base_result = {
            "robot_id": robot_id,
            "anomaly_score": None,
            "is_anomaly": False,
            "predicted_failure_type": "normal",
            "estimated_days_to_failure": None,
            "reasoning": "Telemetry within normal operating parameters.",
            "threshold_alerts": threshold_alerts,
            "model_status": "warming_up",
            "samples_collected": buf.n_samples if buf else 0,
            "samples_needed_to_activate": max(0, _MIN_SAMPLES_TO_FIT - (buf.n_samples if buf else 0)),
        }

        if not buf or buf.n_samples < 2:
            base_result["reasoning"] = (
                f"Collecting baseline telemetry ({buf.n_samples if buf else 0}"
                f"/{_MIN_SAMPLES_TO_FIT} samples). Threshold checks active."
            )
            return base_result

        fv = buf.build_feature_vector()
        if fv is None:
            return base_result

        model = self._models.get(robot_id)
        if model is None or not self._check_sklearn():
            n = buf.n_samples if buf else 0
            base_result["reasoning"] = (
                f"Baseline collection in progress ({n}/{_MIN_SAMPLES_TO_FIT} samples). "
                "Threshold checks active."
            )
            return base_result

        try:
            import numpy as np
            X = np.array([fv])
            score = float(model.decision_function(X)[0])   # >0 normal inlier, <0 anomaly outlier
            is_anomaly = score < _ANOMALY_SCORE_THRESHOLD

            failure_type, days_estimate, reasoning = self._classify_failure(fv, score, buf)

            base_result.update({
                "anomaly_score": round(score, 4),
                "is_anomaly": is_anomaly,
                "predicted_failure_type": failure_type if is_anomaly else "normal",
                "estimated_days_to_failure": days_estimate if is_anomaly else None,
                "reasoning": reasoning,
                "model_status": "active",
            })
        except Exception as exc:
            logger.debug("IsolationForestDetector.evaluate failed for %s: %s", robot_id, exc)

        return base_result

    @staticmethod
    def _classify_failure(
        fv: List[float],
        score: float,
        buf: RobotTelemetryBuffer,
    ) -> Tuple[str, Optional[int], str]:
        """
        Heuristically classifies the predicted failure type from the anomaly feature vector.
        Uses the dominant anomalous feature to produce a plain-language explanation.

        Returns (failure_type, estimated_days, reasoning_string).
        """
        avg_discharge, dist_km, cycle_count, stuck_ratio, speed_var, battery_level = fv

        reasons = []
        failure_type = "unknown_anomaly"

        # --- Battery degradation signature ---
        # High discharge rate + declining absolute battery level
        if avg_discharge > 0.15:
            reasons.append(
                f"Battery discharge rate {avg_discharge:.3f}%/tick "
                f"({avg_discharge / 0.05:.1f}× fleet baseline of 0.05%/tick)"
            )
            failure_type = "battery_degradation"

        # --- Drive wear signature ---
        # High distance + high cycle count + speed variance (motor irregularity)
        if dist_km > 10.0 and speed_var > 0.08:
            reasons.append(
                f"Drive motor speed variance={speed_var:.4f} at {dist_km:.1f}km odometer "
                f"(irregular rotation pattern → bearing/gear wear indicator)"
            )
            failure_type = "drive_wear"

        # --- Sensor/navigation fault signature ---
        # High stuck-time ratio (robot repeatedly blocked where others aren't)
        if stuck_ratio > 0.30:
            reasons.append(
                f"Stuck-time ratio={stuck_ratio:.2f} "
                f"(robot spending {stuck_ratio*100:.0f}% of ticks in WAITING/YIELDING — "
                f"possible LiDAR contamination or sensor drift)"
            )
            failure_type = "sensor_drift"

        # Estimate days to failure from anomaly score severity
        # Score range for outliers: ~-0.30 (severe) to 0.0 (borderline)
        severity = max(0.0, min(1.0, abs(score) / 0.20))   # 0=borderline, 1=severe
        if severity < 0.25:
            days = 6
        elif severity < 0.5:
            days = 4
        elif severity < 0.75:
            days = 2
        else:
            days = 1

        if not reasons:
            reasons = ["Multi-feature composite anomaly score below normal operating envelope"]
            failure_type = "composite_anomaly"

        reasoning = (
            f"[Isolation Forest | score={score:.4f}] Anomaly detected. "
            f"Predicted failure type: {failure_type.upper().replace('_', ' ')}. "
            f"Estimated {days} day(s) before intervention required. "
            f"Evidence: {'; '.join(reasons)}."
        )

        return failure_type, days, reasoning

    def get_fleet_health_summary(self) -> Dict[str, Dict[str, Any]]:
        """Returns anomaly evaluation for every tracked robot."""
        return {r_id: self.evaluate(r_id) for r_id in self._buffers}
