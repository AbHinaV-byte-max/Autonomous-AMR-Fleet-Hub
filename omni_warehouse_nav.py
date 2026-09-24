"""
Omniverse Warehouse World & Navigation Mapping Layer
Directly extracts 3D geometry from simulation5.usd to construct a continuous-to-discrete
occupancy and navigation costmap along verified warehouse corridors and aisles.
Compatible with ref_sih_amr AStarPlanner and HungarianAllocator.
"""

import os
import sys
import math
from typing import Tuple, List, Set, Dict, Optional, Any

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
from omni_usd_env import Usd, UsdGeom, Gf, Sdf, USD_AVAILABLE


class OmniWarehouseNavMap:
    """
    Builds the internal planning occupancy grid from actual Omniverse USD stage geometry.
    This grid is strictly an internal navigation/search abstraction for A* and Hungarian allocation.
    Physical execution and telemetry remain in continuous world-space.
    """
    def __init__(self, stage=None, usd_path=None, cell_size=0.5, margin=0.3):
        self.cell_size = float(cell_size)
        self.margin = float(margin)
        self.usd_path = usd_path or os.path.join(REPO_ROOT, "simulation5.usd")
        self.stage = stage
        
        if self.stage is None and USD_AVAILABLE and os.path.exists(self.usd_path):
            self.stage = Usd.Stage.Open(self.usd_path)

        # Coordinate bounds of warehouse
        self.x_min = -36.0
        self.x_max = 36.0
        self.y_min = -26.0
        self.y_max = 26.0

        # High-traffic verified highways and aisles
        self.HIGHWAYS_Y = [14.5, -4.5, -24.0]
        self.AISLES_X = [-30.0, -23.0, -16.0, -10.0, -3.0, 4.5, 11.0, 18.5, 25.5, 32.5]

        self.obstacle_boxes: List[Tuple[float, float, float, float]] = []
        self.static_obstacle_cells: Set[Tuple[int, int]] = set()
        self.dynamic_obstacle_cells: Set[Tuple[int, int]] = set()
        self.dynamic_obstacles: Dict[str, Dict[str, Any]] = {}

        self._extract_geometry_and_build_grid()

    def world_to_nav(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert continuous Omniverse coordinates to discrete navigation cells."""
        gx = int(round(wx / self.cell_size))
        gy = int(round(wy / self.cell_size))
        return (gx, gy)

    def nav_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert discrete navigation cells back to continuous Omniverse coordinates."""
        wx = float(gx) * self.cell_size
        wy = float(gy) * self.cell_size
        return (wx, wy)

    def _extract_geometry_and_build_grid(self):
        """Inspects simulation5.usd for racks, piles, and obstacle prims."""
        if self.stage is not None:
            for prim in self.stage.Traverse():
                path = str(prim.GetPath())
                name = prim.GetName()

                if any(skip in path for skip in ["Robots", "dropPoint", "Floor", "Ground", "Physics", "Charging_Pads"]):
                    continue

                if any(k in name for k in ["Rack", "Pile", "Crate", "Container", "OilHazard"]):
                    xf = UsdGeom.Xformable(prim)
                    tr = None
                    for op in xf.GetOrderedXformOps():
                        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                            tr = op.Get()
                            break
                    if tr is not None and self.x_min <= tr[0] <= self.x_max and self.y_min <= tr[1] <= self.y_max:
                        # Bounding half-extents with safety inflation
                        hw = 1.2 + self.margin
                        hh = 0.8 + self.margin
                        box = (tr[0] - hw, tr[0] + hw, tr[1] - hh, tr[1] + hh)
                        self.obstacle_boxes.append(box)

        # Rasterize obstacle bounding boxes into discrete cells
        for (xmin, xmax, ymin, ymax) in self.obstacle_boxes:
            gx_min, gy_min = self.world_to_nav(xmin, ymin)
            gx_max, gy_max = self.world_to_nav(xmax, ymax)
            for gx in range(gx_min, gx_max + 1):
                for gy in range(gy_min, gy_max + 1):
                    self.static_obstacle_cells.add((gx, gy))

        # Boundary walls
        gx_bounds_min = int(round(self.x_min / self.cell_size)) - 1
        gx_bounds_max = int(round(self.x_max / self.cell_size)) + 1
        gy_bounds_min = int(round(self.y_min / self.cell_size)) - 1
        gy_bounds_max = int(round(self.y_max / self.cell_size)) + 1

        for gx in range(gx_bounds_min, gx_bounds_max + 1):
            self.static_obstacle_cells.add((gx, gy_bounds_min))
            self.static_obstacle_cells.add((gx, gy_bounds_max))
        for gy in range(gy_bounds_min, gy_bounds_max + 1):
            self.static_obstacle_cells.add((gx_bounds_min, gy))
            self.static_obstacle_cells.add((gx_bounds_max, gy))

    def update_dynamic_obstacle(self, name: str, world_pos: Tuple[float, float], active: bool = True):
        """Adds or removes physical simulated obstacle (e.g., fallen pallet in Omniverse)."""
        gx, gy = self.world_to_nav(world_pos[0], world_pos[1])
        if active:
            self.dynamic_obstacles[name] = {"pos": world_pos, "grid": (gx, gy), "active": True}
            # Inflate obstacle cell
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    self.dynamic_obstacle_cells.add((gx + dx, gy + dy))
        else:
            if name in self.dynamic_obstacles:
                del self.dynamic_obstacles[name]
            # Recompute dynamic obstacle cells
            self.dynamic_obstacle_cells.clear()
            for obs in self.dynamic_obstacles.values():
                ogx, ogy = obs["grid"]
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        self.dynamic_obstacle_cells.add((ogx + dx, ogy + dy))

    def get_cell(self, gx: int, gy: int) -> str:
        """
        Implements the costmap interface expected by ref_sih_amr.robot.planner.AStarPlanner:
        Returns '#' for occupied/obstacle and '.' for free walkable aisle.
        """
        cell = (gx, gy)
        if cell in self.static_obstacle_cells or cell in self.dynamic_obstacle_cells:
            return '#'
        return '.'

    def is_walkable(self, wx: float, wy: float) -> bool:
        """Checks if continuous world coordinate is inside a walkable corridor."""
        gx, gy = self.world_to_nav(wx, wy)
        return self.get_cell(gx, gy) == '.'

    def find_all(self, char: str) -> List[Tuple[int, int]]:
        """Returns cells of given type (for compatibility with ref_sih_amr)."""
        if char == '#':
            return list(self.static_obstacle_cells.union(self.dynamic_obstacle_cells))
        return []
