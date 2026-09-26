"""
Predictive Maintenance Facade (SIH 26123 - Bharat Electronics Limited).

Wraps two complementary maintenance detection layers:
  1. ThresholdMaintenanceClassifier — original rule-based hard-limit checks.
     Retained as deterministic safety net (odometer, cycle count, battery floor).
  2. IsolationForestMaintenanceDetector — anomaly-detection layer (new, 2026).
     Unsupervised; learns "normal" telemetry baseline then flags genuine deviations.

RESEARCH BASIS:
  A 2026 industry case study: a 60-unit AMR fleet saw uptime collapse 97% → 81%
  over 8 months with no predictive maintenance program. Wheel wear, battery fade,
  and LiDAR contamination went untracked until failures occurred. A structured
  monitoring program with battery discharge rate, drive cycle count, and stuck-time
  pattern tracking could have predicted every drive failure 4-6 days in advance.
  Industry-wide data: predictive maintenance delivers ~78% reduction in unplanned downtime.
"""

from typing import Dict, Any, List, Optional

from .isolation_forest_detector import IsolationForestMaintenanceDetector


class ThresholdMaintenanceClassifier:
    """
    Original rule-based hard-limit checks — retained as deterministic safety net.
    These thresholds fire regardless of whether the Isolation Forest model is fitted.
    """

    def __init__(
        self,
        distance_km_threshold: float = 15.0,
        operating_cycles_threshold: int = 40,
    ):
        self.distance_km_threshold = distance_km_threshold
        self.operating_cycles_threshold = operating_cycles_threshold

    def evaluate_robot(
        self,
        robot_id: str,
        odometer_meters: float,
        cycles_completed: int,
        battery_pct: float,
    ) -> Dict[str, Any]:
        """Evaluates cumulative wear metrics against hard maintenance thresholds."""
        odometer_km = odometer_meters / 1000.0
        service_due = False
        reasons: List[str] = []

        distance_wear_pct = (odometer_km / self.distance_km_threshold) * 100.0
        if odometer_km >= self.distance_km_threshold:
            service_due = True
            reasons.append(
                f"Odometer threshold exceeded: {odometer_km:.2f} km "
                f">= {self.distance_km_threshold} km"
            )

        cycle_wear_pct = (cycles_completed / self.operating_cycles_threshold) * 100.0
        if cycles_completed >= self.operating_cycles_threshold:
            service_due = True
            reasons.append(
                f"Operating cycles threshold exceeded: {cycles_completed} "
                f">= {self.operating_cycles_threshold}"
            )

        if battery_pct < 15.0:
            reasons.append(f"Low battery level: {battery_pct:.1f}%")

        overall_wear_pct = min(100.0, max(distance_wear_pct, cycle_wear_pct))

        return {
            "robot_id": robot_id,
            "service_due": service_due,
            "wear_pct": round(overall_wear_pct, 1),
            "reasons": reasons,
            "threshold_reasons": reasons,
            "odometer_km": round(odometer_km, 3),
            "cycles_completed": cycles_completed,
            "battery_pct": round(battery_pct, 1),
        }


class PredictiveMaintenanceClassifier:
    """
    Facade combining ThresholdMaintenanceClassifier (hard-limit safety net) and
    IsolationForestMaintenanceDetector (learned anomaly detection).

    Output merges both layers into a single, dashboard-ready health dict.
    The threshold layer always runs; the IF layer activates after 50 baseline samples.
    """

    def __init__(
        self,
        distance_km_threshold: float = 15.0,
        operating_cycles_threshold: int = 40,
    ):
        self._threshold = ThresholdMaintenanceClassifier(
            distance_km_threshold=distance_km_threshold,
            operating_cycles_threshold=operating_cycles_threshold,
        )
        self._if_detector = IsolationForestMaintenanceDetector(
            n_estimators=100,
            contamination=0.05,
            random_state=42,
        )

    def ingest_telemetry(self, robot_id: str, full_telemetry: Dict[str, Any]) -> None:
        """
        Feeds one telemetry snapshot into the Isolation Forest buffer.
        Call this every tick for each active robot.
        """
        self._if_detector.ingest_reading(robot_id, full_telemetry)

    def evaluate_robot(
        self,
        robot_id: str,
        odometer_meters: float,
        cycles_completed: int,
        battery_pct: float,
    ) -> Dict[str, Any]:
        """
        Runs both maintenance layers and merges their outputs.

        Returns a unified health dict with:
          - All threshold fields (wear_pct, service_due, threshold_reasons)
          - All IF fields (anomaly_score, is_anomaly, predicted_failure_type,
                          estimated_days_to_failure, reasoning, model_status)
        """
        # Layer 1: deterministic threshold check (always runs)
        threshold_result = self._threshold.evaluate_robot(
            robot_id=robot_id,
            odometer_meters=odometer_meters,
            cycles_completed=cycles_completed,
            battery_pct=battery_pct,
        )

        # Layer 2: IF anomaly detection (activates after warm-up)
        if_result = self._if_detector.evaluate(
            robot_id=robot_id,
            threshold_alerts=threshold_result["threshold_reasons"],
        )

        # Merge: threshold fields + IF fields — alert if EITHER layer flags
        needs_attention = threshold_result["service_due"] or if_result["is_anomaly"]

        return {
            # Identity
            "robot_id": robot_id,
            # Alert aggregation
            "needs_attention": needs_attention,
            # Threshold layer
            "service_due": threshold_result["service_due"],
            "wear_pct": threshold_result["wear_pct"],
            "reasons": threshold_result["threshold_reasons"],
            "threshold_reasons": threshold_result["threshold_reasons"],
            "odometer_km": threshold_result["odometer_km"],
            "cycles_completed": threshold_result["cycles_completed"],
            "battery_pct": threshold_result["battery_pct"],
            # Isolation Forest layer
            "anomaly_score": if_result["anomaly_score"],
            "is_anomaly": if_result["is_anomaly"],
            "predicted_failure_type": if_result["predicted_failure_type"],
            "estimated_days_to_failure": if_result["estimated_days_to_failure"],
            "reasoning": if_result["reasoning"],
            "model_status": if_result["model_status"],
            "samples_collected": if_result["samples_collected"],
            "samples_needed_to_activate": if_result["samples_needed_to_activate"],
        }
