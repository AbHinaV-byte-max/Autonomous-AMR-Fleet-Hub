"""
Omniverse Warehouse World & Navigation Mapping Layer
Directly extracts 3D geometry from simulation5.usd to construct a continuous-to-discrete
occupancy and navigation costmap along verified warehouse corridors and aisles.
Compatible with ref_sih_amr AStarPlanner and HungarianAllocator.

F1 FIX (Phase 1): Added aisle-based fallback grid so the map contains real blocked
cells regardless of whether USD pxr is available or prim names match.  The USD
geometry scan is preserved and runs first; the aisle fallback only fills cells
that the USD scan did NOT mark as obstacles.
"""

import os
import sys
import math
from typing import Tuple, List, Set, Dict, Optional, Any

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
from omni_usd_env import Usd, UsdGeom, Gf, Sdf, USD_AVAILABLE


# Half-widths for corridor free zones (metres).
# A cell is walkable if its world-centre falls within HIGHWAY_HALF_WIDTH of
# any highway row OR within AISLE_HALF_WIDTH of any aisle column.
HIGHWAY_HALF_WIDTH = 1.5   # highways are ~3 m wide
AISLE_HALF_WIDTH   = 1.5   # aisles   are ~3 m wide


class OmniWarehouseNavMap:
    """
    Builds the internal planning occupancy grid from actual Omniverse USD stage geometry.
    This grid is strictly an internal navigation/search abstraction for A* and Hungarian allocation.
    Physical execution and telemetry remain in continuous world-space.

    Grid construction priority:
      1. USD geometry scan (preserved, unchanged) — finds rack/pile/crate obstacle prims.
      2. Aisle-based fallback — fills all interior cells that are NOT on a verified
         highway or aisle as blocked.  This guarantees a non-empty obstacle field even
         when USD is unavailable or prim names don't match.
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

        # High-traffic verified highways (horizontal corridors) and aisles (vertical corridors)
        self.HIGHWAYS_Y = [14.5, -4.5, -24.0]
        self.AISLES_X = [-30.0, -23.0, -16.0, -10.0, -3.0, 4.5, 11.0, 18.5, 25.5, 32.5]

        self.obstacle_boxes: List[Tuple[float, float, float, float]] = []
        self.static_obstacle_cells: Set[Tuple[int, int]] = set()
        self.dynamic_obstacle_cells: Set[Tuple[int, int]] = set()
        self.dynamic_obstacles: Dict[str, Dict[str, Any]] = {}
        # Phase 3A: tagged temporary exclusion cells (for safe deadlock replan).
        # Maps tag_str -> set of (cx,cy) cells. Cleared by clear_cells_by_tag().
        self._tagged_exclusion_cells: Dict[str, Set[Tuple[int, int]]] = {}

        # Counters for diagnostics
        self._usd_obstacle_count: int = 0
        self._aisle_blocked_count: int = 0

        self._extract_geometry_and_build_grid()
        self._apply_aisle_fallback()

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # USD geometry scan (preserved from original, unchanged)
    # ------------------------------------------------------------------

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

        self._usd_obstacle_count = len(self.static_obstacle_cells)

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

    # ------------------------------------------------------------------
    # F1 FIX: Aisle-based fallback grid
    # ------------------------------------------------------------------

    def _is_on_corridor(self, wx: float, wy: float) -> bool:
        """
        Returns True if the world-space point (wx, wy) lies within the free
        zone of any known highway (horizontal) or aisle (vertical) corridor.
        Drop-point approach zones at the north-east and north-west corners are
        also kept free so robots can reach delivery goals.
        """
        # Horizontal highways
        for hy in self.HIGHWAYS_Y:
            if abs(wy - hy) <= HIGHWAY_HALF_WIDTH:
                return True

        # Vertical aisles
        for ax in self.AISLES_X:
            if abs(wx - ax) <= AISLE_HALF_WIDTH:
                return True

        # Drop-point approach corridors (north perimeter at y > 20)
        if wy > 20.0:
            return True

        # South perimeter approach (y < -22)
        if wy < -22.0:
            return True

        return False

    def _apply_aisle_fallback(self):
        """
        For every interior grid cell NOT already marked as an obstacle by the
        USD scan, block it if it does NOT lie on a known corridor.  This converts
        the flat open-plan grid into a proper warehouse aisle network.

        Cells already blocked by the USD scan are left as-is (USD wins).
        """
        gx_min = int(round(self.x_min / self.cell_size))
        gx_max = int(round(self.x_max / self.cell_size))
        gy_min = int(round(self.y_min / self.cell_size))
        gy_max = int(round(self.y_max / self.cell_size))

        before = len(self.static_obstacle_cells)

        for gx in range(gx_min, gx_max + 1):
            for gy in range(gy_min, gy_max + 1):
                if (gx, gy) in self.static_obstacle_cells:
                    # Already blocked by USD geometry — do not override
                    continue
                wx, wy = self.nav_to_world(gx, gy)
                if not self._is_on_corridor(wx, wy):
                    self.static_obstacle_cells.add((gx, gy))

        self._aisle_blocked_count = len(self.static_obstacle_cells) - before

    # ------------------------------------------------------------------
    # Dynamic obstacle management (unchanged)
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Phase 3A: Tagged temporary cell blocking (for safe deadlock replan)
    # ------------------------------------------------------------------

    def set_cell_blocked(self, cx: int, cy: int, tag: str):
        """Temporarily marks a nav cell as blocked under the given tag.
        A* treats the cell as '#' until clear_cells_by_tag(tag) is called.
        Multiple tags can coexist on the same cell.
        """
        if tag not in self._tagged_exclusion_cells:
            self._tagged_exclusion_cells[tag] = set()
        self._tagged_exclusion_cells[tag].add((cx, cy))

    def clear_cells_by_tag(self, tag: str):
        """Removes all cell-block entries registered under the given tag.
        Safe to call even if tag was never set.
        """
        self._tagged_exclusion_cells.pop(tag, None)

    # ------------------------------------------------------------------
    # Grid query interface (unchanged contract)
    # ------------------------------------------------------------------

    def get_cell(self, gx: int, gy: int) -> str:
        """
        Implements the costmap interface expected by ref_sih_amr.robot.planner.AStarPlanner:
        Returns '#' for occupied/obstacle and '.' for free walkable aisle.
        Also checks _tagged_exclusion_cells (used during safe deadlock replan).
        """
        cell = (gx, gy)
        if cell in self.static_obstacle_cells or cell in self.dynamic_obstacle_cells:
            return '#'
        # Check tagged temporary exclusions (e.g., robot footprints during deadlock replan)
        for tag_cells in self._tagged_exclusion_cells.values():
            if cell in tag_cells:
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

    # ------------------------------------------------------------------
    # F1 FIX: Diagnostic summary
    # ------------------------------------------------------------------

    def print_grid_summary(self):
        """
        Prints a structured diagnostic summary of the navigation grid.
        Call this after construction to verify the grid is non-empty.
        """
        gx_min = int(round(self.x_min / self.cell_size)) - 1
        gx_max = int(round(self.x_max / self.cell_size)) + 1
        gy_min = int(round(self.y_min / self.cell_size)) - 1
        gy_max = int(round(self.y_max / self.cell_size)) + 1

        total_cells = (gx_max - gx_min + 1) * (gy_max - gy_min + 1)
        blocked_cells = len(self.static_obstacle_cells) + len(self.dynamic_obstacle_cells)
        walkable_cells = total_cells - blocked_cells
        pct_blocked = 100.0 * blocked_cells / max(1, total_cells)

        print("=" * 60)
        print("  NAVIGATION GRID SUMMARY")
        print("=" * 60)
        print(f"  Warehouse bounds : X=[{self.x_min}, {self.x_max}]  Y=[{self.y_min}, {self.y_max}]")
        print(f"  Cell size        : {self.cell_size} m")
        print(f"  Total cells      : {total_cells:,}")
        print(f"  Walkable cells   : {walkable_cells:,}")
        print(f"  Blocked cells    : {blocked_cells:,}  ({pct_blocked:.1f}%)")
        print(f"  USD obstacles    : {self._usd_obstacle_count:,}  (from prim scan)")
        print(f"  Aisle fallback   : {self._aisle_blocked_count:,}  (rack-fill between corridors)")
        print(f"  Dynamic obstacles: {len(self.dynamic_obstacle_cells):,}")
        print(f"  Highways (Y)     : {self.HIGHWAYS_Y}")
        print(f"  Aisles   (X)     : {self.AISLES_X}")
        status = "NON-EMPTY [OK]" if blocked_cells > 100 else "EMPTY -- FALLBACK FAILED [FAIL]"
        print(f"  Grid status      : {status}")
        print("=" * 60)

    def render_ascii(self, scale: int = 4) -> str:
        """
        Renders a compact ASCII view of the grid for console verification.
        scale controls how many grid cells map to one ASCII character.
        """
        gx_min = int(round(self.x_min / self.cell_size))
        gx_max = int(round(self.x_max / self.cell_size))
        gy_min = int(round(self.y_min / self.cell_size))
        gy_max = int(round(self.y_max / self.cell_size))

        lines = []
        gy = gy_max
        while gy >= gy_min:
            row = ""
            gx = gx_min
            while gx <= gx_max:
                cell = self.get_cell(gx, gy)
                if cell == '#' and (gx, gy) in self.dynamic_obstacle_cells:
                    row += 'O'
                else:
                    row += cell
                gx += scale
            lines.append(row)
            gy -= scale
        return "\n".join(lines)
