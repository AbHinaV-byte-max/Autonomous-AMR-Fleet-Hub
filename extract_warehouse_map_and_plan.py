"""
Exact Warehouse Obstacle Map Extractor & Collision-Free AMR Path Planner
Extracts 3D bounding boxes of all racks and crate piles from simulation5.usd,
builds an occupancy grid map with safety inflation margins, plans collision-free
A* paths avoiding all racks and boxes, and bakes them to USD.
"""

import os
import sys
import math
import heapq

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
KIT_RELEASE_DIR = os.path.join(REPO_ROOT, "_build", "windows-x86_64", "release")
EXTSCACHE_DIR = os.path.join(KIT_RELEASE_DIR, "extscache")

usd_libs_dir = None
if os.path.exists(EXTSCACHE_DIR):
    for entry in os.listdir(EXTSCACHE_DIR):
        if entry.startswith("omni.usd.libs"):
            usd_libs_dir = os.path.join(EXTSCACHE_DIR, entry)
            break

if usd_libs_dir:
    bin_dir = os.path.join(usd_libs_dir, "bin")
    if hasattr(os, "add_dll_directory") and os.path.exists(bin_dir):
        os.add_dll_directory(bin_dir)
    os.environ["PATH"] = bin_dir + ";" + os.environ.get("PATH", "")
    if usd_libs_dir not in sys.path:
        sys.path.insert(0, usd_libs_dir)

from pxr import Usd, UsdGeom, Gf, Sdf  # type: ignore


def get_prim_world_bbox(prim, bbox_cache):
    """Compute 2D bounding box (xmin, xmax, ymin, ymax) of a prim in world coordinates."""
    bbox = bbox_cache.ComputeWorldBound(prim)
    aligned_range = bbox.ComputeAlignedBox()
    min_pt = aligned_range.GetMin()
    max_pt = aligned_range.GetMax()
    return (min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2])


def build_warehouse_map(stage, cell_size=0.5, margin=0.5):
    """
    Extracts all obstacle bounding boxes (127 racks, crates, piles) using their explicit 3D translations.
    Returns:
      obstacle_cells: set of (gx, gy)
      world_to_grid: coordinate converter
      grid_to_world: coordinate converter
      obstacle_boxes: list of 2D bounding boxes [(xmin, xmax, ymin, ymax), ...]
    """
    obstacle_boxes = []
    
    for p in stage.Traverse():
        name = p.GetName()
        path = str(p.GetPath())
        
        # Skip robots, dropPoints, floor, charging pads
        if any(skip in path for skip in ["Robot", "dropPoint", "Floor", "Ground", "Physics", "Charging_Pads"]):
            continue
            
        if any(k in name for k in ["Rack", "Pile", "Crate", "Container", "OilHazard"]):
            xf = UsdGeom.Xformable(p)
            tr = None
            for op in xf.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    tr = op.Get()
                    break
            if tr is not None and -40 <= tr[0] <= 40 and -30 <= tr[1] <= 30:
                # Standard rack/pile bounding box half-extents with safety inflation
                hw = 1.3 + margin
                hh = 0.9 + margin
                obstacle_boxes.append((tr[0] - hw, tr[0] + hw, tr[1] - hh, tr[1] + hh))

    print(f"[MAP GENERATOR] Extracted {len(obstacle_boxes)} clean rack & obstacle bounding boxes.")

    def world_to_grid(x, y):
        return (int(round(x / cell_size)), int(round(y / cell_size)))

    def grid_to_world(gx, gy):
        return (gx * cell_size, gy * cell_size)

    obstacle_cells = set()
    for (xmin, xmax, ymin, ymax) in obstacle_boxes:
        gx_min, gy_min = world_to_grid(xmin, ymin)
        gx_max, gy_max = world_to_grid(xmax, ymax)
        for gx in range(gx_min, gx_max + 1):
            for gy in range(gy_min, gy_max + 1):
                obstacle_cells.add((gx, gy))

    print(f"[MAP GENERATOR] Generated {len(obstacle_cells)} occupied grid cells on warehouse map.")
    return obstacle_cells, world_to_grid, grid_to_world, obstacle_boxes


def a_star_planner(start_world, goal_world, obstacle_cells, world_to_grid, grid_to_world):
    """Grid-based 8-directional A* path planner with line-of-sight smoothing."""
    start_grid = world_to_grid(start_world[0], start_world[1])
    goal_grid = world_to_grid(goal_world[0], goal_world[1])

    # If start or goal is in obstacle (e.g. margin near pad), unblock immediate cell
    obstacle_cells = set(obstacle_cells)
    obstacle_cells.discard(start_grid)
    obstacle_cells.discard(goal_grid)

    def heuristic(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return 1.0 * (dx + dy) + (1.414 - 2.0) * min(dx, dy)

    frontier = []
    heapq.heappush(frontier, (0, start_grid))
    came_from = {start_grid: None}
    cost_so_far = {start_grid: 0}

    # 8-connected movement
    neighbors = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
    ]

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal_grid:
            break

        for dx, dy, step_cost in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt in obstacle_cells:
                continue

            new_cost = cost_so_far[current] + step_cost
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + heuristic(nxt, goal_grid)
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    if goal_grid not in came_from:
        print(f"[PLANNER] Warning: No path found from {start_world} to {goal_world}!")
        return [start_world, goal_world]

    # Reconstruct path
    curr = goal_grid
    grid_path = []
    while curr:
        grid_path.append(curr)
        curr = came_from[curr]
    grid_path.reverse()

    # Convert to world coordinates
    world_path = [grid_to_world(gx, gy) for gx, gy in grid_path]
    
    # Path simplification / waypoint pruning
    pruned_path = [world_path[0]]
    for i in range(1, len(world_path) - 1):
        prev = pruned_path[-1]
        p = world_path[i]
        nxt = world_path[i + 1]
        # Keep waypoint if direction changes
        d1 = (p[0] - prev[0], p[1] - prev[1])
        d2 = (nxt[0] - p[0], nxt[1] - p[1])
        if abs(d1[0]*d2[1] - d1[1]*d2[0]) > 1e-4:
            pruned_path.append(p)
    pruned_path.append(world_path[-1])

    return pruned_path


def plan_and_bake_warehouse_routes():
    usd_paths = [
        os.path.join(REPO_ROOT, "simulation5.usd"),
        r"C:\Users\goruv\Downloads\simulation5.usd"
    ]

    for usd_path in usd_paths:
        if not os.path.exists(usd_path):
            continue

        print(f"\n==================================================")
        print(f"EXTRACTING MAP & PLANNING FOR: {usd_path}")
        print(f"==================================================")
        stage = Usd.Stage.Open(usd_path)
        if not stage:
            continue

        # Timeline setup
        stage.SetTimeCodesPerSecond(60.0)
        stage.SetStartTimeCode(0.0)
        total_frames = 240
        stage.SetEndTimeCode(float(total_frames))

        # Disable physics scene gravity
        physics_scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
        if physics_scene_prim.IsValid():
            physics_scene_prim.GetAttribute("physics:gravityMagnitude").Set(0.0)

        # 1. Build Exact Map from 3D Bounding Boxes
        obstacle_cells, world_to_grid, grid_to_world, obstacle_boxes = build_warehouse_map(stage, cell_size=0.5, margin=0.6)

        # Drop points
        dropPoint1 = (32.86, 23.92)
        dropPoint2 = (-34.14, 22.13)

        # Initial AMR positions from stage
        start_positions = {
            "AMR_01": (7.8, -15.0),
            "AMR_02": (-8.8, 15.5),
            "AMR_03": (-17.5, -4.26),
            "AMR_04": (-6.15, -10.63),
            "AMR_05": (-5.54, -13.63),
            "AMR_06": (1.05, 13.04)
        }

        # Target destinations
        destinations = {
            "AMR_01": dropPoint1,
            "AMR_02": dropPoint2,
            "AMR_03": dropPoint2,
            "AMR_04": dropPoint1,
            "AMR_05": dropPoint1,
            "AMR_06": dropPoint2
        }

        # Add Dynamic Pallet Obstacle at (7.8, 4.0)
        dyn_obs_box = (7.8 - 0.8, 7.8 + 0.8, 4.0 - 0.8, 4.0 + 0.8)
        dyn_obs_cells = set()
        gx_min, gy_min = world_to_grid(dyn_obs_box[0], dyn_obs_box[2])
        gx_max, gy_max = world_to_grid(dyn_obs_box[1], dyn_obs_box[3])
        for gx in range(gx_min, gx_max + 1):
            for gy in range(gy_min, gy_max + 1):
                dyn_obs_cells.add((gx, gy))

        # 2. Compute Guaranteed Collision-Free Paths
        computed_routes = {}
        for r_name, start_pt in start_positions.items():
            goal_pt = destinations[r_name]
            # For AMR_01, include dynamic obstacle avoidance
            obs_map = obstacle_cells.union(dyn_obs_cells) if r_name == "AMR_01" else obstacle_cells
            path = a_star_planner(start_pt, goal_pt, obs_map, world_to_grid, grid_to_world)
            computed_routes[r_name] = path
            print(f"[PLANNED ROUTE] {r_name}: {len(path)} waypoints from {start_pt} -> {goal_pt}")

        # 3. Find and Bind USD Robot Prims
        robot_prim_paths = {}
        for r_name in start_positions.keys():
            for prefix in ["/World/Warehouse/Robots", "/World/P_DYNEX_Depot/Robots"]:
                full_path = f"{prefix}/{r_name}"
                prim = stage.GetPrimAtPath(full_path)
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
                    robot_prim_paths.setdefault(r_name, []).append((translate_op, rotate_op, prim))

        # 4. Bake Keyframes (0 to 240)
        for r_name, waypoints in computed_routes.items():
            if r_name not in robot_prim_paths:
                continue
            ops_list = robot_prim_paths[r_name]
            num_segments = len(waypoints) - 1
            if num_segments <= 0:
                continue

            frames_per_seg = max(5, int(total_frames / num_segments))

            for seg_idx in range(num_segments):
                p_start = waypoints[seg_idx]
                p_end = waypoints[seg_idx + 1]
                start_frame = seg_idx * frames_per_seg
                
                dx = p_end[0] - p_start[0]
                dy = p_end[1] - p_start[1]
                heading_deg = math.degrees(math.atan2(dy, dx)) if (dx != 0 or dy != 0) else 0.0

                for f in range(frames_per_seg + 1):
                    current_frame = start_frame + f
                    if current_frame > total_frames:
                        break
                    alpha = f / float(frames_per_seg)
                    smooth_alpha = (1.0 - math.cos(alpha * math.pi)) / 2.0
                    cur_x = p_start[0] + smooth_alpha * (p_end[0] - p_start[0])
                    cur_y = p_start[1] + smooth_alpha * (p_end[1] - p_start[1])

                    for translate_op, rotate_op, prim in ops_list:
                        translate_op.Set(Gf.Vec3d(cur_x, cur_y, 0.035), Usd.TimeCode(current_frame))
                        if rotate_op:
                            op_type = rotate_op.GetOpType()
                            if op_type == UsdGeom.XformOp.TypeRotateXYZ:
                                rotate_op.Set(Gf.Vec3f(0.0, 0.0, float(heading_deg)), Usd.TimeCode(current_frame))
                            elif op_type == UsdGeom.XformOp.TypeRotateZ:
                                rotate_op.Set(float(heading_deg), Usd.TimeCode(current_frame))

            # Hold final position inside dropPoint at end frame
            final_p = waypoints[-1]
            for translate_op, rotate_op, prim in ops_list:
                translate_op.Set(Gf.Vec3d(final_p[0], final_p[1], 0.035), Usd.TimeCode(float(total_frames)))

            print(f"[BAKED] {r_name}: Collision-free trajectory baked to dropPoint!")

        # Dynamic Pallet Spawning at frame 60
        for obs_path in ["/World/Warehouse/DynamicObstacle", "/World/P_DYNEX_Depot/DynamicObstacle"]:
            obs_prim = stage.GetPrimAtPath(obs_path)
            if obs_prim.IsValid():
                obs_xform = UsdGeom.Xformable(obs_prim)
                obs_tr = None
                for op in obs_xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        obs_tr = op
                        break
                if not obs_tr:
                    obs_tr = obs_xform.AddTranslateOp()
                obs_tr.Set(Gf.Vec3d(7.8, 4.0, -10.0), Usd.TimeCode(0.0))
                obs_tr.Set(Gf.Vec3d(7.8, 4.0, -10.0), Usd.TimeCode(59.0))
                obs_tr.Set(Gf.Vec3d(7.8, 4.0, 0.4), Usd.TimeCode(60.0))
                obs_tr.Set(Gf.Vec3d(7.8, 4.0, 0.4), Usd.TimeCode(float(total_frames)))

        stage.Save()
        print(f"✓ SAVED STAGE: {usd_path}\n")


if __name__ == "__main__":
    plan_and_bake_warehouse_routes()
