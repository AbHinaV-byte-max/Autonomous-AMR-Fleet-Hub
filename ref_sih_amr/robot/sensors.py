"""
Sensor simulation module for Autonomous Mobile Robots (AMRs).

Provides onboard sensor emulation for digital twin telemetry and hardware testing.
Phase 1 of multi-part sensor simulation plan: LiDAR Scan Emulation.
"""
import math
from typing import Any, List, Sequence, Tuple, Union


class SensorSimulator:
    """Emulates onboard sensors for AMRs in grid warehouse environments."""

    def simulate_lidar(
        self,
        position: Sequence[Union[float, int]],
        heading: float,
        costmap: Any,
        num_rays: int = 16,
        max_range: float = 6,
    ) -> List[float]:
        """
        Cast num_rays evenly-spaced rays from `position`, starting at `heading`
        and going around 360 degrees. For each ray, step outward cell-by-cell
        (reusing costmap.get_cell()) until hitting a blocked cell ('#') or
        reaching max_range.
        
        Args:
            position: (x, y) coordinates of the robot in grid space.
            heading: Direction angle of the robot in degrees (or radians).
            costmap: Existing GridMap/costmap object with get_cell(x, y) -> str.
            num_rays: Number of rays to cast around 360 degrees (default: 16).
            max_range: Maximum scan distance in cells (default: 6).

        Returns:
            List of distance values (floats) for each ray.
        """
        if num_rays <= 0:
            return []
        max_range_float = float(max_range)
        if max_range_float <= 0.0:
            return [0.0] * num_rays
        if costmap is None:
            return [max_range_float] * num_rays

        # In ref_sih_amr, heading is stored in degrees (0 = +X, 90 = +Y).
        # We detect radians if a small non-zero float is passed, but default to degrees.
        if abs(heading) > 2.0 * math.pi or heading == 0.0 or isinstance(heading, int):
            base_angle_rad = math.radians(heading)
        else:
            base_angle_rad = float(heading)

        angle_step_rad = (2.0 * math.pi) / float(num_rays)
        max_steps = int(math.floor(max_range_float))

        px = float(position[0])
        py = float(position[1])

        distances: List[float] = []

        for i in range(num_rays):
            angle = base_angle_rad + (i * angle_step_rad)
            ux = math.cos(angle)
            uy = math.sin(angle)

            hit_dist = max_range_float
            for step in range(1, max_steps + 1):
                cx = int(round(px + (step * ux)))
                cy = int(round(py + (step * uy)))
                cell = costmap.get_cell(cx, cy)
                if cell == '#' or cell is None:
                    hit_dist = float(step)
                    break

            distances.append(hit_dist)

        return distances


class LidarScanPayload(list):
    """
    Hybrid container for snapshot['lidar_scan'].
    - As a list: contains the primary/lead robot's ray distances.
    - As a dict: allows indexing by robot_id (e.g. snap['lidar_scan']['robot-0']).
    """
    def __init__(self, scans_by_id: dict):
        self._by_id = dict(scans_by_id)
        lead = next(iter(self._by_id.values())) if self._by_id else []
        super().__init__(lead)

    def __getitem__(self, key):
        if isinstance(key, str):
            return self._by_id[key]
        return super().__getitem__(key)

    def __contains__(self, key):
        if isinstance(key, str):
            return key in self._by_id
        return super().__contains__(key)

    def get(self, key, default=None):
        return self._by_id.get(key, default)

    def keys(self):
        return self._by_id.keys()

    def values(self):
        return self._by_id.values()

    def items(self):
        return self._by_id.items()
