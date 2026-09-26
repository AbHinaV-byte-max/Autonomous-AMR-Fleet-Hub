"""
Phase 3B: Enhanced Collision Tracer
=====================================
Extends the basic CollisionChecker with full diagnostic records per collision event:
  - timestamp, robot pair, distance, threshold
  - both robots' current nav cells
  - waiting_on state
  - last replan event
  - path snapshot

Usage:
  from _collision_tracer import CollisionTracer
  tracer = CollisionTracer(threshold_m=0.6)
  tracer.check_frame(frame, t, positions, agents)
  tracer.print_report()
  assert tracer.n_collision_events == 0
"""

import math
from typing import List, Dict, Tuple, Optional, Any
from itertools import combinations


class CollisionTracer:
    """
    Per-frame geometric collision checker with full diagnostic trace.
    Phase 3B requirement: every collision frame must record:
      timestamp, pair, distance, threshold,
      both cells, waiting_on, deadlock state, last replan
    """

    def __init__(self, threshold_m: float = 0.6, nav_map=None):
        self.threshold     = threshold_m
        self.nav_map       = nav_map
        self.n_frames      = 0
        self.n_pairs       = 0
        self.min_distance  = float("inf")
        self.n_collision_frames = 0
        self.n_collision_events = 0
        self.collision_log: List[Dict[str, Any]] = []
        self.distance_log: List[Dict[str, Any]] = []   # all pairs, all frames (for min/max)
        self._was_colliding: Dict[Tuple[str, str], bool] = {}
        self._last_replan: Dict[str, Optional[Dict]] = {}  # robot_id -> last replan info

    # ------------------------------------------------------------------
    def notify_replan(self, robot_id: str, t: float, start_nav, goal_nav,
                      path_len: int, reason: str = "normal"):
        """Call this when a path replan occurs so we can record it in collision traces."""
        self._last_replan[robot_id] = {
            "t": round(t, 3),
            "start_nav": start_nav,
            "goal_nav": goal_nav,
            "path_len": path_len,
            "reason": reason,
        }

    # ------------------------------------------------------------------
    def check_frame(self, frame: int, t: float,
                    positions: Dict[str, Tuple[float, float]],
                    agents=None):
        """
        Check all robot pairs for this frame.
        positions: {robot_id: (world_x, world_y)}
        agents:    list of DecentralizedAMRAgent (optional; provides nav cells / waiting_on)
        """
        self.n_frames += 1
        frame_collision = False
        ids = sorted(positions.keys())
        pairs = list(combinations(ids, 2))
        self.n_pairs = len(pairs)

        # Build agent state lookup
        agent_lookup: Dict[str, Any] = {}
        if agents:
            for a in agents:
                if a.robot_id in positions:
                    agent_lookup[a.robot_id] = a

        for r1, r2 in pairs:
            p1, p2 = positions[r1], positions[r2]
            dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])

            if dist < self.min_distance:
                self.min_distance = dist

            pair = (r1, r2)
            was = self._was_colliding.get(pair, False)

            if dist < self.threshold:
                frame_collision = True
                if not was:
                    # New collision event — build detailed trace
                    self.n_collision_events += 1

                    def _nav(rid):
                        if self.nav_map:
                            x, y = positions[rid]
                            return self.nav_map.world_to_nav(x, y)
                        return "N/A"

                    def _waiting(rid):
                        a = agent_lookup.get(rid)
                        return a.waiting_on if a else "N/A"

                    def _path(rid):
                        a = agent_lookup.get(rid)
                        if a and a.current_path_nav:
                            return a.current_path_nav[:4]
                        return []

                    entry = {
                        "frame": frame,
                        "t": round(t, 3),
                        "pair": f"{r1}-{r2}",
                        "distance": round(dist, 4),
                        "threshold": self.threshold,
                        r1: {
                            "pos": (round(p1[0], 4), round(p1[1], 4)),
                            "cell": _nav(r1),
                            "waiting_on": _waiting(r1),
                            "path_prefix": _path(r1),
                            "last_replan": self._last_replan.get(r1),
                        },
                        r2: {
                            "pos": (round(p2[0], 4), round(p2[1], 4)),
                            "cell": _nav(r2),
                            "waiting_on": _waiting(r2),
                            "path_prefix": _path(r2),
                            "last_replan": self._last_replan.get(r2),
                        },
                    }
                    self.collision_log.append(entry)
                    self._was_colliding[pair] = True
            else:
                self._was_colliding[pair] = False

        if frame_collision:
            self.n_collision_frames += 1

    # ------------------------------------------------------------------
    def print_report(self, prefix: str = ""):
        tag = f"[{prefix}] " if prefix else ""
        print(f"{tag}Collision threshold      : {self.threshold:.2f}m")
        print(f"{tag}Frames checked           : {self.n_frames:,}")
        print(f"{tag}Robot pairs/frame        : {self.n_pairs}")
        print(f"{tag}Min inter-robot distance : {self.min_distance:.4f}m")
        print(f"{tag}Collision frames         : {self.n_collision_frames}")
        print(f"{tag}Collision events         : {self.n_collision_events}")

        for ev in self.collision_log:
            r1_id, r2_id = ev["pair"].split("-", 1)
            print(f"\n{tag}[COLLISION]")
            print(f"{tag}  t={ev['t']}s  pair={ev['pair']}  distance={ev['distance']:.4f}m  "
                  f"threshold={ev['threshold']}m")
            for rid in [r1_id, r2_id]:
                rd = ev.get(rid, {})
                print(f"{tag}  {rid}:")
                print(f"{tag}    pos={rd.get('pos')}  cell={rd.get('cell')}")
                print(f"{tag}    waiting_on={rd.get('waiting_on')}")
                print(f"{tag}    path_prefix={rd.get('path_prefix')}")
                lr = rd.get("last_replan")
                if lr:
                    print(f"{tag}    last_replan: t={lr['t']}s  "
                          f"start={lr.get('start_nav')}  goal={lr.get('goal_nav')}  "
                          f"len={lr.get('path_len')}  reason={lr.get('reason')}")
                else:
                    print(f"{tag}    last_replan: None")

    # ------------------------------------------------------------------
    def summary_dict(self) -> Dict[str, Any]:
        return {
            "min_distance": round(self.min_distance, 4),
            "collision_frames": self.n_collision_frames,
            "collision_events": self.n_collision_events,
            "threshold": self.threshold,
            "collision_log": self.collision_log,
        }
