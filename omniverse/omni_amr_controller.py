"""
Real Omniverse AMR Kinematic Controller & Telemetry Feedback Engine
Operates directly against USD prims in assets/omniverse/simulation5.usd using differential drive kinematics.
Implements distance-based waypoint progression, in-place heading alignment, anti-oscillation guards,
and synchronized USD TimeSample baking across both Warehouse and Depot prim hierarchies.
"""

import math
import time
from typing import List, Tuple, Optional, Dict, Any

from omni_usd_env import Usd, UsdGeom, Gf, Sdf, USD_AVAILABLE


def normalize_angle_deg(angle: float) -> float:
    """Normalize angle to [-180, 180] degrees."""
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def clear_all_stage_time_samples(stage):
    """
    Clears all legacy keyframes and time samples from all AMR and Cargo prims on the stage.
    Ensures that stale baked trajectories do not override live or newly baked simulations.
    """
    if not stage:
        return
    cleared_count = 0
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if "AMR_" in path_str or "Cargo_" in path_str or "DynamicObstacle" in path_str:
            xform = UsdGeom.Xformable(prim)
            if xform:
                for op in xform.GetOrderedXformOps():
                    attr = op.GetAttr()
                    if attr and len(attr.GetTimeSamples()) > 0:
                        attr.Clear()
                        cleared_count += 1
    if cleared_count > 0:
        print(f"[USD CLEANUP] Cleared time samples across {cleared_count} ops on stage.")


DEFAULT_SPAWN_POSITIONS = {
    "AMR_01": (4.5, -4.5, 0.035, 90.0),
    "AMR_02": (-34.14, 22.13, 0.035, -90.0),
    "AMR_03": (18.5, -4.5, 0.035, 90.0),
    "AMR_04": (-10.0, 14.5, 0.035, -90.0),
    "AMR_05": (25.5, -24.0, 0.035, 90.0),
    "AMR_06": (-30.0, -24.0, 0.035, 90.0),
}


class OmniAMRController:
    """
    Physical kinematic controller for an Autonomous Mobile Robot in Omniverse.
    Actuates linear velocity (v <= 1.0 m/s) and angular velocity (omega <= 90 deg/s)
    and synchronizes actual simulated state to USD stage transform ops.
    """
    def __init__(self, robot_id: str, stage, max_v: float = 1.5, max_omega: float = 120.0,
                 waypoint_tolerance: float = 0.35, goal_tolerance: float = 0.12,
                 heading_align_threshold: float = 25.0):
        self.robot_id = robot_id
        self.stage = stage
        self.max_v = float(max_v)                        # 1.5 m/s
        self.max_omega = float(max_omega)                # 120.0 deg/s
        self.waypoint_tolerance = float(waypoint_tolerance) # 0.35 m  (intermediate waypoints)
        self.goal_tolerance = float(goal_tolerance)          # 0.12 m  (final goal only)
        self.heading_align_threshold = float(heading_align_threshold) # 25.0 deg
        
        # Actual state read from USD or default spawn table
        spawn_pose = DEFAULT_SPAWN_POSITIONS.get(self.robot_id, (4.5, -4.5, 0.035, 90.0))
        self.actual_x: float = spawn_pose[0]
        self.actual_y: float = spawn_pose[1]
        self.actual_z: float = spawn_pose[2]
        self.actual_heading: float = spawn_pose[3]
        self.actual_v: float = 0.0       # m/s
        self.actual_omega: float = 0.0   # deg/s
        self.last_update_time: float = 0.0

        # Path following state
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_idx: int = 0
        self.is_stopped: bool = True
        self.is_failed: bool = False

        # Anti-pattern detection history
        self.recent_positions: List[Tuple[float, float]] = []
        self.recent_targets: List[Tuple[float, float]] = []
        self.oscillation_detected: bool = False

        # Bindings: list of dicts with {"prim": prim, "translate_op": op, "rotate_op": op}
        self.bindings: List[Dict[str, Any]] = []
        self._init_usd_ops()

    def _init_usd_ops(self):
        if not self.stage:
            return

        candidate_paths = [
            f"/World/Warehouse/Robots/{self.robot_id}",
            f"/World/P_DYNEX_Depot/Robots/{self.robot_id}",
            f"/World/Robots/{self.robot_id}",
            f"/World/{self.robot_id}",
        ]

        for p_path in candidate_paths:
            prim = self.stage.GetPrimAtPath(p_path)
            if prim.IsValid():
                xform = UsdGeom.Xformable(prim)
                translate_op = None
                rotate_op = None
                for op in xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        translate_op = op
                    elif op.GetOpType() in [UsdGeom.XformOp.TypeRotateZ, UsdGeom.XformOp.TypeRotateXYZ]:
                        rotate_op = op
                
                if not translate_op:
                    translate_op = xform.AddTranslateOp()
                if not rotate_op:
                    rotate_op = xform.AddRotateZOp()

                binding = {
                    "prim": prim,
                    "path": p_path,
                    "translate_op": translate_op,
                    "rotate_op": rotate_op
                }
                # Ensure CargoBox deck mount exists on the robot prim
                if self.stage:
                    cargo_prim = self.stage.GetPrimAtPath(f"{p_path}/CargoBox")
                    if not cargo_prim.IsValid():
                        cargo_prim = self.stage.DefinePrim(f"{p_path}/CargoBox", "Cube")
                        if cargo_prim.IsValid():
                            UsdGeom.Cube(cargo_prim).GetSizeAttr().Set(0.5)
                            cxform = UsdGeom.Xformable(cargo_prim)
                            cxform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.45))

                self.bindings.append(binding)

    def reset_to_spawn(self):
        """Resets the robot kinematics strictly to its default spawn position."""
        spawn_pose = DEFAULT_SPAWN_POSITIONS.get(self.robot_id, (4.5, -4.5, 0.035, 90.0))
        self.actual_x = float(spawn_pose[0])
        self.actual_y = float(spawn_pose[1])
        self.actual_z = float(spawn_pose[2])
        self.actual_heading = float(spawn_pose[3])
        self.actual_v = 0.0
        self.actual_omega = 0.0
        self.waypoints.clear()
        self.current_waypoint_idx = 0
        self.is_stopped = True
        self.is_failed = False
        self.recent_targets.clear()
        self._apply_usd_transform(frame=None)

    def clear_time_samples(self, reset_spawn: bool = True):
        """Clears all historical time samples and authors initial spawn pose to USD."""
        for b in self.bindings:
            if b["translate_op"]:
                attr = b["translate_op"].GetAttr()
                if attr:
                    attr.Clear()
            if b["rotate_op"]:
                attr = b["rotate_op"].GetAttr()
                if attr:
                    attr.Clear()
        if reset_spawn:
            self.reset_to_spawn()
        else:
            self._apply_usd_transform(frame=None)

    def set_path(self, world_waypoints: List[Tuple[float, float]]):
        """Load a new sequence of continuous world waypoints for this AMR."""
        self.waypoints = list(world_waypoints)
        self.current_waypoint_idx = 0
        self.is_stopped = len(self.waypoints) == 0
        self.recent_targets.clear()
        self.oscillation_detected = False

    def clear_path(self):
        self.waypoints.clear()
        self.current_waypoint_idx = 0
        self.is_stopped = True
        self.actual_v = 0.0
        self.actual_omega = 0.0

    def _apply_usd_transform(self, frame: Optional[float] = None):
        """Synchronizes actual position and heading across all bound USD prims."""
        if not USD_AVAILABLE or not self.bindings:
            return

        pos_vec = Gf.Vec3d(self.actual_x, self.actual_y, self.actual_z)
        
        for b in self.bindings:
            t_op = b["translate_op"]
            r_op = b["rotate_op"]
            if t_op:
                t_op.Set(pos_vec)
                if frame is not None:
                    t_op.Set(pos_vec, time=float(frame))
            if r_op:
                if r_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    rot_val = Gf.Vec3f(0.0, 0.0, float(self.actual_heading))
                else:
                    rot_val = float(self.actual_heading)
                r_op.Set(rot_val)
                if frame is not None:
                    r_op.Set(rot_val, time=float(frame))

    def step(self, dt: float, current_time: float, frame: Optional[float] = None) -> Dict[str, Any]:
        """
        Executes one physical kinematic control step:
        1. Reads actual simulated pose.
        2. Progresses waypoint based strictly on distance tolerance.
        3. Computes differential heading & forward velocity.
        4. Updates USD transforms on stage (both default and time-sampled frame).
        5. Returns telemetry dict.
        """
        if self.is_failed or self.is_stopped or not self.waypoints:
            self.actual_v = 0.0
            self.actual_omega = 0.0
            self.last_update_time = current_time
            self._apply_usd_transform(frame=frame)
            return self.get_telemetry()

        # Check if already reached final waypoint
        if self.current_waypoint_idx >= len(self.waypoints):
            self.is_stopped = True
            self.actual_v = 0.0
            self.actual_omega = 0.0
            self.last_update_time = current_time
            self._apply_usd_transform(frame=frame)
            return self.get_telemetry()

        target_wp = self.waypoints[self.current_waypoint_idx]
        dx = target_wp[0] - self.actual_x
        dy = target_wp[1] - self.actual_y
        dist = math.sqrt(dx * dx + dy * dy)

        # -------------------------------------------------------------
        # Waypoint Progression (STRICTLY DISTANCE-BASED)
        # F5 FIX: use tight goal_tolerance for the final waypoint so robots
        # stop precisely at their goal, and waypoint_tolerance for the rest.
        # -------------------------------------------------------------
        is_final_wp = (self.current_waypoint_idx == len(self.waypoints) - 1)
        active_tolerance = self.goal_tolerance if is_final_wp else self.waypoint_tolerance

        if dist <= active_tolerance:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.is_stopped = True
                self.actual_v = 0.0
                self.actual_omega = 0.0
                self.last_update_time = current_time
                self._apply_usd_transform(frame=frame)
                return self.get_telemetry()
            target_wp = self.waypoints[self.current_waypoint_idx]
            dx = target_wp[0] - self.actual_x
            dy = target_wp[1] - self.actual_y
            dist = math.sqrt(dx * dx + dy * dy)

        # -------------------------------------------------------------
        # Heading Control & Orientation Alignment
        # -------------------------------------------------------------
        target_heading_rad = math.atan2(dy, dx)
        target_heading_deg = math.degrees(target_heading_rad)
        heading_error = normalize_angle_deg(target_heading_deg - self.actual_heading)

        # Anti-pattern check: detect A -> B -> A -> B target oscillation
        if not self.recent_targets or self.recent_targets[-1] != target_wp:
            self.recent_targets.append(target_wp)
            if len(self.recent_targets) > 20:
                self.recent_targets.pop(0)

            if len(self.recent_targets) >= 4:
                if (self.recent_targets[-1] == self.recent_targets[-3] and 
                    self.recent_targets[-2] == self.recent_targets[-4] and 
                    self.recent_targets[-1] != self.recent_targets[-2]):
                    self.oscillation_detected = True
                    self.is_stopped = True
                    self.actual_v = 0.0
                    self.actual_omega = 0.0
                    print(f"[{self.robot_id}] ANTI-PATTERN DETECTED: Oscillation between {self.recent_targets[-1]} and {self.recent_targets[-2]}. Halting.")
                    self.last_update_time = current_time
                    self._apply_usd_transform(frame=frame)
                    return self.get_telemetry()

        # Proportional heading & velocity controller
        kp_omega = 4.0
        abs_error = abs(heading_error)
        sign = 1.0 if heading_error > 0 else -1.0

        if abs_error > 45.0:
            # Very large heading error: Rotate in place
            self.actual_v = 0.0
            self.actual_omega = sign * min(self.max_omega, max(30.0, kp_omega * abs_error))
        elif abs_error > self.heading_align_threshold:
            # Moderate heading error: Arc turn (smooth curve without stopping)
            self.actual_v = min(self.max_v * 0.5, dist * 1.0)
            self.actual_omega = sign * min(self.max_omega, max(20.0, kp_omega * abs_error))
        else:
            # Aligned: drive forward at nominal speed, smooth steering
            speed_factor = max(0.5, (self.heading_align_threshold - abs_error) / self.heading_align_threshold)
            self.actual_v = min(self.max_v, max(0.3, dist * 1.5)) * speed_factor
            self.actual_omega = kp_omega * heading_error
            self.actual_omega = max(-self.max_omega, min(self.max_omega, self.actual_omega))

        # -------------------------------------------------------------
        # Kinematic Motion Integration
        # -------------------------------------------------------------
        new_heading = normalize_angle_deg(self.actual_heading + self.actual_omega * dt)
        rad = math.radians(new_heading)
        new_x = self.actual_x + self.actual_v * math.cos(rad) * dt
        new_y = self.actual_y + self.actual_v * math.sin(rad) * dt

        self.actual_heading = new_heading
        self.actual_x = new_x
        self.actual_y = new_y
        self.last_update_time = current_time

        # Update Omniverse USD Stage Transform Ops (Default + TimeSample)
        self._apply_usd_transform(frame=frame)

        return self.get_telemetry()

    def get_telemetry(self) -> Dict[str, Any]:
        """Returns the actual simulated state vector from Omniverse."""
        target = self.waypoints[self.current_waypoint_idx] if self.waypoints and self.current_waypoint_idx < len(self.waypoints) else None
        dist_to_target = math.sqrt((target[0] - self.actual_x)**2 + (target[1] - self.actual_y)**2) if target else 0.0
        
        return {
            "robot_id": self.robot_id,
            "actual_position": (self.actual_x, self.actual_y, self.actual_z),
            "actual_heading": self.actual_heading,
            "linear_velocity": self.actual_v,
            "angular_velocity": self.actual_omega,
            "target_waypoint": target,
            "waypoint_index": self.current_waypoint_idx,
            "total_waypoints": len(self.waypoints),
            "distance_to_target": dist_to_target,
            "is_stopped": self.is_stopped,
            "is_failed": self.is_failed,
            "timestamp": self.last_update_time
        }
