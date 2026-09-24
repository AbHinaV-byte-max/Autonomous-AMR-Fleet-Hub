import os
import sys
import time
import heapq

# Configure Omniverse USD Environment
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__)))
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

try:
    from pxr import Usd, UsdGeom, Gf  # type: ignore
    USD_AVAILABLE = True
except ImportError:
    Usd = UsdGeom = Gf = None  # type: ignore
    USD_AVAILABLE = False


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def space_time_a_star(start_pos, start_time, goal, obstacles, max_time=50):
    start = (start_pos[0], start_pos[1], start_time)
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)
        x, y, t = current

        if (x, y) == goal:
            break
                
        if t > start_time + max_time:
            continue

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (0, 0)]:
            nx, ny = x + dx, y + dy
            nt = t + 1
            
            if (nx, ny) in obstacles:
                continue

            next_node = (nx, ny, nt)
            new_cost = cost_so_far[current] + 1
            
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic((nx, ny), goal)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current

    goal_state = None
    for state in came_from:
        if (state[0], state[1]) == goal:
            goal_state = state
            break
            
    if not goal_state:
        return []
        
    path = []
    current = goal_state
    while current != start:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


class DynamicObstacle:
    def __init__(self, obstacle_id, pos, activation_time):
        self.obstacle_id = obstacle_id
        self.pos = pos
        self.activation_time = activation_time
        self.active = False


class EventLogger:
    _events = []

    @classmethod
    def log_event(cls, event_type, details=None):
        entry = {
            "timestamp": details.get("time", time.time()) if details else time.time(),
            "event_type": event_type,
            "details": details or {}
        }
        cls._events.append(entry)

    @classmethod
    def get_recent_events(cls, limit=50):
        return cls._events[-limit:] if limit > 0 else list(cls._events)

    @classmethod
    def clear(cls):
        cls._events = []


class FleetContext:
    current_time = 0
    agents = []
    blackboard = None
    network = None
    task_manager = None
    static_obstacles = set()
    stage = None
    simulation_status = "RUNNING"


class SharedBlackboard:
    def __init__(self):
        self.reservations = {}
        self.wfg = {}
        self.robot_positions = {}
        self.active_reservations_by_robot = {}
        self.dynamic_obstacles = []
        
    def add_dynamic_obstacle(self, obs):
        self.dynamic_obstacles.append(obs)
        
    def get_active_dynamic_obstacles(self):
        return [obs.pos for obs in self.dynamic_obstacles if obs.active]
        
    def add_reservation(self, res):
        self.active_reservations_by_robot[res.robot_id] = res
        for state in res.path:
            if state not in self.reservations:
                self.reservations[state] = []
            self.reservations[state].append(res)
            
    def remove_reservation(self, res):
        if res.robot_id in self.active_reservations_by_robot and self.active_reservations_by_robot[res.robot_id] == res:
            del self.active_reservations_by_robot[res.robot_id]
        for state in res.path:
            if state in self.reservations:
                if res in self.reservations[state]:
                    self.reservations[state].remove(res)
                if not self.reservations[state]:
                    del self.reservations[state]
                    
    def remove_all_reservations_for_robot(self, robot_id):
        res = self.active_reservations_by_robot.get(robot_id)
        if res:
            self.remove_reservation(res)
            return True
        return False

    def get_conflicts(self, res, current_pos):
        conflicts = set()
        
        for state in res.path:
            if state in self.reservations:
                for other_res in self.reservations[state]:
                    if other_res.robot_id != res.robot_id:
                        conflicts.add(other_res)
        
        if res.path:
            next_step = res.path[0]
            for other_id, other_pos in self.robot_positions.items():
                if other_id == res.robot_id:
                    continue
                if (next_step[0], next_step[1]) == other_pos:
                    other_res = self.active_reservations_by_robot.get(other_id)
                    if other_res and other_res.path:
                        other_next = other_res.path[0]
                        if (other_next[0], other_next[1]) == current_pos:
                            conflicts.add(other_res)
                            
        return list(conflicts)

    def set_waiting(self, waiter_id, blocking_id):
        if blocking_id:
            self.wfg[waiter_id] = blocking_id
        else:
            if waiter_id in self.wfg:
                del self.wfg[waiter_id]
                
    def get_cycle(self, start_id):
        visited = []
        curr = start_id
        while curr in self.wfg:
            if curr in visited:
                cycle_start_idx = visited.index(curr)
                return visited[cycle_start_idx:]
            visited.append(curr)
            curr = self.wfg[curr]
        return None

    def update_position(self, robot_id, pos):
        self.robot_positions[robot_id] = pos


class CommunicationNetwork:
    def __init__(self):
        self.faults = []
        self.metrics = {
            "messages_sent": 0,
            "messages_received": 0,
            "messages_dropped": 0
        }
        
    def inject_fault(self, sender, receiver, start_time, end_time=float('inf')):
        self.faults.append({
            "sender": sender,
            "receiver": receiver,
            "start": start_time,
            "end": end_time
        })
        
    def can_communicate(self, sender, receiver, current_time):
        for fault in self.faults:
            if fault["sender"] == sender and fault["receiver"] == receiver:
                if fault["start"] <= current_time <= fault["end"]:
                    return False
        return True

    def broadcast_state(self, agents, current_time):
        for sender in agents:
            if sender.failed:
                continue
            for receiver in agents:
                if sender == receiver or receiver.failed:
                    continue
                
                self.metrics["messages_sent"] += 1
                if self.can_communicate(sender.name, receiver.name, current_time):
                    self.metrics["messages_received"] += 1
                    receiver.receive_peer_state(sender.name, sender.pos, current_time)
                else:
                    self.metrics["messages_dropped"] += 1


class Reservation:
    def __init__(self, robot_id, path, emergency, safety_constraint, request_time, urgency, remaining_cost):
        self.robot_id = robot_id
        self.path = path
        self.emergency = emergency
        self.safety_constraint = safety_constraint
        self.request_time = request_time
        self.urgency = urgency
        self.remaining_cost = remaining_cost
        self.valid = False

    def compare_priority(self, other):
        if self.emergency != other.emergency:
            return self.emergency
        if self.safety_constraint != other.safety_constraint:
            return self.safety_constraint
        if self.request_time != other.request_time:
            return self.request_time < other.request_time
        if self.urgency != other.urgency:
            return self.urgency < other.urgency
        if self.remaining_cost != other.remaining_cost:
            return self.remaining_cost < other.remaining_cost
class EdgeAI:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.latencies = []

    def advise(self, amr_state):
        start_t = time.perf_counter()
        recommendation = "PROCEED"
        confidence = 0.95
        risk_level = "LOW"
        reason = "Nominal trajectory, clear path"

        if "simulated_intent" in amr_state and amr_state["simulated_intent"]:
            recommendation = amr_state["simulated_intent"]
            confidence = 0.85
            risk_level = "LOW"
            reason = f"Simulated intent: {recommendation}"
        elif amr_state.get("is_in_safe_mode"):
            recommendation = "WAIT"
            confidence = 0.99
            risk_level = "CRITICAL"
            reason = "SAFE_MODE active due to communication failure"
        elif amr_state.get("comm_status") == "DEGRADED":
            recommendation = "WAIT"
            confidence = 0.80
            risk_level = "MEDIUM"
            reason = "Peer communication degraded; uncertain proximity"
        elif amr_state.get("conflicts"):
            recommendation = "YIELD"
            confidence = 0.90
            risk_level = "HIGH"
            reason = "Imminent space-time conflict detected"
        elif amr_state.get("blocked"):
            recommendation = "REROUTE"
            confidence = 0.95
            risk_level = "HIGH"
            reason = "Dynamic obstacle blocking planned trajectory"
        elif amr_state.get("deadlock"):
            recommendation = "YIELD"
            confidence = 0.92
            risk_level = "HIGH"
            reason = "Deadlock cycle detected in wait-for graph"
        elif amr_state.get("battery", 100.0) < 15.0:
            recommendation = "CHARGE"
            confidence = 0.95
            risk_level = "MEDIUM"
            reason = "Battery level low"

        end_t = time.perf_counter()
        latency_ms = (end_t - start_t) * 1000.0
        self.latencies.append(latency_ms)

        return {
            "recommendation": recommendation,
            "confidence": confidence,
            "risk_level": risk_level,
            "reason": reason,
            "latency_ms": latency_ms
        }


class SafetyArbiter:
    metrics = {
        "ai_decisions": 0,
        "ai_agreements": 0,
        "ai_disagreements": 0
    }

    def __init__(self, robot_id):
        self.robot_id = robot_id

    def arbitrate(self, ai_advice, safety_context):
        SafetyArbiter.metrics["ai_decisions"] += 1
        rec = ai_advice["recommendation"]
        final_action = rec
        arbiter_reason = "AI recommendation approved by safety constraints"
        agreement = True

        if safety_context.get("is_in_safe_mode") and rec != "WAIT":
            final_action = "STOP"
            arbiter_reason = "Hard safety rule: SAFE_MODE enforces full stop"
            agreement = False
        elif safety_context.get("has_active_conflict") and rec == "PROCEED":
            final_action = "WAIT"
            arbiter_reason = "Hard safety rule: Active reservation conflict requires WAIT/YIELD"
            agreement = False
        elif safety_context.get("is_blocked") and rec == "PROCEED":
            final_action = "REROUTE"
            arbiter_reason = "Hard safety rule: Path obstructed by dynamic obstacle"
            agreement = False
        elif safety_context.get("is_in_deadlock") and rec == "PROCEED":
            final_action = "YIELD"
            arbiter_reason = "Hard safety rule: Deadlock cycle requires yield and replan"
            agreement = False
        elif not safety_context.get("has_reservation") and rec == "PROCEED":
            final_action = "WAIT"
            arbiter_reason = "Hard safety rule: Cannot proceed without granted reservation"
            agreement = False

        if agreement:
            SafetyArbiter.metrics["ai_agreements"] += 1
            print(f"[{self.robot_id}] AI_SAFETY_AGREEMENT")
            print(f"[{self.robot_id}] ai_recommendation={rec}")
            print(f"[{self.robot_id}] final_action={final_action}")
            print(f"[{self.robot_id}] confidence={ai_advice['confidence']:.2f}")
            print(f"[{self.robot_id}] reason={arbiter_reason}")
            EventLogger.log_event("AI_SAFETY_AGREEMENT", {
                "robot": self.robot_id,
                "ai_recommendation": rec,
                "final_action": final_action,
                "confidence": ai_advice["confidence"],
                "reason": arbiter_reason
            })
        else:
            SafetyArbiter.metrics["ai_disagreements"] += 1
            print(f"[{self.robot_id}] AI_SAFETY_DISAGREEMENT")
            print(f"[{self.robot_id}] ai_recommendation={rec}")
            print(f"[{self.robot_id}] final_action={final_action}")
            print(f"[{self.robot_id}] override_reason={arbiter_reason}")
            EventLogger.log_event("AI_SAFETY_DISAGREEMENT", {
                "robot": self.robot_id,
                "ai_recommendation": rec,
                "final_action": final_action,
                "override_reason": arbiter_reason
            })

        return {
            "final_action": final_action,
            "agreement": agreement,
            "arbiter_reason": arbiter_reason
        }


class AMR:
    def __init__(self, name, start_pos, stage, blackboard, t_degraded=2, t_safe_mode=4):
        self.name = name
        self.pos = start_pos
        self.stage = stage
        self.blackboard = blackboard
        self.t_degraded = t_degraded
        self.t_safe_mode = t_safe_mode
        self.prim_path = f"/World/P_DYNEX_Depot/Robots/{name}"
        if stage:
            self.prim = stage.GetPrimAtPath(self.prim_path)
            self.xform = UsdGeom.Xformable(self.prim)
            self.translate_op = None
            for op in self.xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    self.translate_op = op
                    break
            if not self.translate_op:
                self.translate_op = self.xform.AddTranslateOp()
        
        self.current_res = None
        self.goal = None
        self.idle = True
        self.failed = False
        self.urgency = 0
        self.emergency = False
        self.safety_constraint = False
        self.obstacles = set()
        
        self.peer_states = {}
        self.is_in_safe_mode = False
        self.edge_ai = EdgeAI(self.name)
        self.safety_arbiter = SafetyArbiter(self.name)
        self.simulated_ai_intent = None
        
        self.blackboard.update_position(self.name, self.pos)

    def receive_peer_state(self, peer_id, peer_pos, current_time):
        if peer_id in self.peer_states:
            prev_status = self.peer_states[peer_id]["status"]
            if prev_status in ["DEGRADED", "SAFE_MODE"]:
                print(f"[{self.name}] COMMUNICATION_RESTORED")
                print(f"[{self.name}] peer={peer_id}")
                if prev_status == "SAFE_MODE":
                    self.is_in_safe_mode = False
        
        self.peer_states[peer_id] = {
            "last_message_timestamp": current_time,
            "last_known_position": peer_pos,
            "status": "CONNECTED"
        }

    def check_communication_health(self, current_time):
        for peer_id, state in self.peer_states.items():
            age = current_time - state["last_message_timestamp"]
            
            if age >= self.t_safe_mode and state["status"] != "SAFE_MODE":
                state["status"] = "SAFE_MODE"
                self.is_in_safe_mode = True
                print(f"[{self.name}] COMMUNICATION_SAFE_MODE")
                print(f"[{self.name}] robot={self.name}")
                print(f"[{self.name}] peer={peer_id}")
                print(f"[{self.name}] last_seen_age={age}")
                print(f"[{self.name}] action=STOP")
                
                if self.current_res:
                    self.blackboard.remove_reservation(self.current_res)
                    self.current_res = None
            
            elif age >= self.t_degraded and state["status"] == "CONNECTED":
                state["status"] = "DEGRADED"
                print(f"[{self.name}] COMMUNICATION_DEGRADED")
                print(f"[{self.name}] robot={self.name}")
                print(f"[{self.name}] peer={peer_id}")
                print(f"[{self.name}] last_seen_age={age}")
                
                print(f"[{self.name}] STALE_PEER_STATE")
                print(f"[{self.name}] robot={self.name}")
                print(f"[{self.name}] peer={peer_id}")
                print(f"[{self.name}] last_known_position={state['last_known_position']}")

    def get_safety_margins(self):
        margins = set()
        for peer_id, state in self.peer_states.items():
            if state["status"] == "DEGRADED":
                px, py = state["last_known_position"]
                # Add 1-cell radius margin around last known position for degraded peers
                for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (0, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                    margins.add((px + dx, py + dy))
        return margins

    def update_usd(self):
        self.blackboard.update_position(self.name, self.pos)
        if self.stage:
            pos_3d = Gf.Vec3d(float(self.pos[0]), float(self.pos[1]), 0.035)
            self.translate_op.Set(pos_3d)

    def trigger_failure(self, current_time, task_manager):
        if self.failed:
            return
            
        print(f"\n[{self.name}] ROBOT_FAILURE_DETECTED")
        print(f"[{self.name}] robot={self.name}")
        print(f"[{self.name}] task={self.goal}")
        print(f"[{self.name}] time={current_time}")
        
        self.failed = True
        print(f"\n[{self.name}] FAILURE_NOTICE")
        print(f"[{self.name}] sender={self.name}")
        print(f"[{self.name}] status=FAILED")
        
        res_count = len(self.current_res.path) if self.current_res else 0
        if self.blackboard.remove_all_reservations_for_robot(self.name):
            print(f"\n[{self.name}] RESERVATIONS_RELEASED")
            print(f"[{self.name}] robot={self.name}")
            print(f"[{self.name}] count={res_count}")
        self.current_res = None
        self.blackboard.set_waiting(self.name, None)
        
        if self.goal:
            failed_task = self.goal
            self.goal = None
            print(f"\n[{self.name}] TASK_UNASSIGNED")
            print(f"[{self.name}] task={failed_task}")
            print(f"[{self.name}] TASK_REASSIGNMENT_STARTED")
            print(f"[{self.name}] task={failed_task}")
            task_manager.add_task(failed_task)
            
        fail_obs = DynamicObstacle(f"FAILED_ROBOT_{self.name}", self.pos, current_time)
        fail_obs.active = True
        self.blackboard.add_dynamic_obstacle(fail_obs)

    def plan_and_request(self, current_time, obstacles):
        if self.failed or self.is_in_safe_mode:
            return
            
        self.obstacles = obstacles
        if not self.goal:
            return
            
        if self.current_res:
            self.blackboard.remove_reservation(self.current_res)
            self.current_res = None
            
        # Include degraded safety margins as temporary obstacles
        safety_margins = self.get_safety_margins()
        full_obstacles = obstacles.union(safety_margins)
        
        path = space_time_a_star(self.pos, current_time, self.goal, full_obstacles)
        
        if not path:
            print(f"[{self.name}] RESERVATION_REJECTED: No physical path to {self.goal}")
            if safety_margins:
                print(f"[{self.name}] UNCERTAIN_CONFLICT_ZONE")
                print(f"[{self.name}] robot={self.name}")
                print(f"[{self.name}] action=WAIT")
            self.yield_path(current_time, blocking_robot=None) 
            return
            
        print(f"[{self.name}] NEW_PATH_PLANNED")
        print(f"[{self.name}] robot={self.name}")
        print(f"[{self.name}] task={self.goal}")
        
        print(f"[{self.name}] RESERVATION_REQUEST: Planning path to {self.goal} at t={current_time}")
            
        res = Reservation(
            robot_id=self.name,
            path=path,
            emergency=self.emergency,
            safety_constraint=self.safety_constraint,
            request_time=current_time,
            urgency=self.urgency,
            remaining_cost=len(path)
        )
        
        self.current_res = res
        self.blackboard.add_reservation(res)
        self.resolve_conflicts(current_time)

    def check_deadlock(self, current_time):
        cycle = self.blackboard.get_cycle(self.name)
        if cycle and self.name in cycle:
            print(f"\n[{self.name}] DEADLOCK_SUSPECTED")
            print(f"[{self.name}] robots={cycle}")
            print(f"[{self.name}] DEADLOCK_CONFIRMED")
            print(f"[{self.name}] cycle={'->'.join(cycle) + '->' + cycle[0]}")
            
            selected_robot = min(cycle)
            
            if self.name == selected_robot:
                print(f"[{self.name}] DEADLOCK_RECOVERY_STARTED")
                print(f"[{self.name}] selected_robot={selected_robot}")
                print(f"[{self.name}] DEADLOCK_RECOVERY_ACTION")
                print(f"[{self.name}] action=YIELD_AND_REPLAN")
                
                temp_obstacles = set(self.obstacles)
                for r_id in cycle:
                    if r_id != self.name:
                        temp_obstacles.add(self.blackboard.robot_positions[r_id])
                        
                old_path = [s[:2] for s in self.current_res.path] if self.current_res else []
                
                if self.current_res:
                    self.blackboard.remove_reservation(self.current_res)
                    self.current_res = None
                
                self.blackboard.set_waiting(self.name, None)
                
                path = space_time_a_star(self.pos, current_time, self.goal, temp_obstacles)
                if path:
                    new_path = [s[:2] for s in path]
                    print(f"[{self.name}] REPLAN")
                    print(f"[{self.name}] robot={self.name}")
                    print(f"[{self.name}] old_path={old_path}")
                    print(f"[{self.name}] new_path={new_path}")
                    
                    self.current_res = Reservation(
                        robot_id=self.name,
                        path=path,
                        emergency=self.emergency,
                        safety_constraint=self.safety_constraint,
                        request_time=current_time,
                        urgency=self.urgency,
                        remaining_cost=len(path)
                    )
                    self.blackboard.add_reservation(self.current_res)
                    self.current_res.valid = True
                    print(f"[{self.name}] RESERVATION_GRANTED")
                    print(f"[{self.name}] robot={self.name}")
                    print(f"[{self.name}] DEADLOCK_RESOLVED")
                    print(f"[{self.name}] robots={cycle}\n")
                else:
                    print(f"[{self.name}] REPLAN FAILED - Remaining WAITING")
                    target = cycle[(cycle.index(self.name) + 1) % len(cycle)]
                    self.yield_path(current_time, blocking_robot=target)
            return True
        return False

    def is_wait_path(self):
        return self.current_res and len(self.current_res.path) == 1 and (self.current_res.path[0][0], self.current_res.path[0][1]) == self.pos

    def resolve_conflicts(self, current_time):
        if not self.current_res:
            return
            
        conflicts = self.blackboard.get_conflicts(self.current_res, self.pos)
        
        if conflicts:
            print(f"[{self.name}] CONFLICT_DETECTED with {[c.robot_id for c in conflicts]}")
            
            for other_res in conflicts:
                if self.current_res.compare_priority(other_res):
                    next_step = self.current_res.path[0] if self.current_res.path else None
                    if next_step and self.blackboard.robot_positions[other_res.robot_id] == (next_step[0], next_step[1]):
                        is_waiting = True
                        if other_res.path and (other_res.path[-1][0], other_res.path[-1][1]) != (next_step[0], next_step[1]):
                            is_waiting = False
                            
                        if is_waiting:
                            print(f"[{self.name}] Higher priority but physically blocked by yielding {other_res.robot_id}. YIELDing.")
                            self.yield_path(current_time, blocking_robot=other_res.robot_id)
                            other_res.valid = False 
                            return
                        else:
                            print(f"[{self.name}] Higher priority than {other_res.robot_id}. Invalidating their reservation.")
                            other_res.valid = False
                    else:
                        print(f"[{self.name}] Higher priority than {other_res.robot_id}. Invalidating their reservation.")
                        other_res.valid = False
                else:
                    print(f"[{self.name}] Lower priority than {other_res.robot_id}. YIELDing.")
                    self.yield_path(current_time, blocking_robot=other_res.robot_id)
                    other_res.valid = False 
                    return 
                    
            print(f"[{self.name}] CONFLICT_RESOLVED: Won all negotiations.")
            self.current_res.valid = True
            if not self.is_wait_path():
                self.blackboard.set_waiting(self.name, None)
            print(f"[{self.name}] RESERVATION_GRANTED")
            print(f"[{self.name}] robot={self.name}")
        else:
            self.current_res.valid = True
            if not self.is_wait_path():
                self.blackboard.set_waiting(self.name, None)
            print(f"[{self.name}] RESERVATION_GRANTED (No conflicts).")
            print(f"[{self.name}] robot={self.name}")
            
    def yield_path(self, current_time, blocking_robot):
        if self.current_res:
            self.blackboard.remove_reservation(self.current_res)
            
        wait_path = [(self.pos[0], self.pos[1], current_time + 1)]
        self.current_res = Reservation(
            robot_id=self.name,
            path=wait_path,
            emergency=self.emergency,
            safety_constraint=self.safety_constraint,
            request_time=current_time,
            urgency=self.urgency,
            remaining_cost=heuristic(self.pos, self.goal) if self.goal else 0
        )
        self.current_res.valid = True
        self.blackboard.add_reservation(self.current_res)
        self.blackboard.set_waiting(self.name, blocking_robot)
        print(f"[{self.name}] YIELD: Reverted to wait state at {self.pos}")
        
    def check_blockage(self, current_time):
        if not self.current_res or not self.current_res.path:
            return False
            
        active_dyn_obs = self.blackboard.get_active_dynamic_obstacles()
        
        for state in self.current_res.path:
            pos = (state[0], state[1])
            if pos in active_dyn_obs:
                obs_id = next((obs.obstacle_id for obs in self.blackboard.dynamic_obstacles if obs.pos == pos and obs.active), "UNKNOWN")
                print(f"[{self.name}] BLOCKAGE_DETECTED")
                print(f"[{self.name}] robot={self.name}")
                print(f"[{self.name}] obstacle={obs_id}")
                print(f"[{self.name}] location={pos}")
                print(f"[{self.name}] current_timestep={current_time}")
                print(f"[{self.name}] original_goal={self.goal}")
                return True
        return False

    def step(self, current_time):
        if self.failed:
            return
            
        self.check_communication_health(current_time)
        
        # Step 17: Edge-AI Advisory & Safety Arbitration
        amr_state = {
            "pos": self.pos,
            "goal": self.goal,
            "is_in_safe_mode": self.is_in_safe_mode,
            "comm_status": "SAFE_MODE" if self.is_in_safe_mode else ("DEGRADED" if any(s["status"] == "DEGRADED" for s in self.peer_states.values()) else "CONNECTED"),
            "peer_states": self.peer_states,
            "conflicts": self.blackboard.get_conflicts(self.current_res, self.pos) if self.current_res else [],
            "deadlock": bool(self.blackboard.get_cycle(self.name)),
            "blocked": self.check_blockage(current_time) if self.current_res else False,
            "battery": getattr(self, "battery", 100.0),
            "simulated_intent": self.simulated_ai_intent
        }
        
        ai_advice = self.edge_ai.advise(amr_state)
        safety_context = {
            "is_in_safe_mode": self.is_in_safe_mode,
            "has_reservation": self.current_res is not None,
            "reservation_valid": self.current_res.valid if self.current_res else False,
            "has_active_conflict": bool(amr_state["conflicts"]),
            "is_blocked": amr_state["blocked"],
            "is_in_deadlock": amr_state["deadlock"]
        }
        
        arbitration = self.safety_arbiter.arbitrate(ai_advice, safety_context)
        final_action = arbitration["final_action"]
        
        if final_action == "STOP" or self.is_in_safe_mode:
            return
            
        if self.check_deadlock(current_time):
            pass

        if self.check_blockage(current_time):
            print(f"[{self.name}] OLD_PATH_INVALIDATED")
            self.blackboard.remove_reservation(self.current_res)
            self.current_res = None
            self.idle = False
            print(f"[{self.name}] REROUTING_STARTED")
            print(f"[{self.name}] robot={self.name}")
            
            all_obstacles = self.obstacles.union(set(self.blackboard.get_active_dynamic_obstacles()))
            self.plan_and_request(current_time, all_obstacles)
            
            if self.current_res and self.current_res.path and not self.is_wait_path():
                print(f"[{self.name}] ALTERNATIVE_PATH_FOUND")
                print(f"[{self.name}] REROUTE_COMPLETED")
            else:
                print(f"[{self.name}] NO_SAFE_REROUTE")
                print(f"[{self.name}] WAITING_FOR_CLEARANCE")

        if not self.current_res or not self.current_res.valid:
            if not self.idle:
                self.plan_and_request(current_time, self.obstacles)
            return

        if self.current_res.path:
            next_state = self.current_res.path[0]
            if next_state[2] == current_time + 1:
                self.current_res.path.pop(0)
                self.pos = (next_state[0], next_state[1])
                self.update_usd()
                print(f"[{self.name}] Moved to {self.pos}")
            
        if not self.current_res.path:
            if self.goal and self.pos == self.goal:
                print(f"[{self.name}] Reached goal {self.goal}! Now idle.")
                print(f"[{self.name}] TASK_COMPLETED")
                print(f"[{self.name}] task={self.goal}")
                print(f"[{self.name}] robot={self.name}")
                self.idle = True
                self.goal = None
                self.blackboard.remove_reservation(self.current_res)
                self.current_res = None
                self.blackboard.set_waiting(self.name, None)
            elif not self.idle:
                self.plan_and_request(current_time + 1, self.obstacles)


class TaskManager:
    def __init__(self, tasks):
        self.tasks = tasks
        
    def add_task(self, task):
        self.tasks.append(task)
        
    def allocate_tasks(self, agents, current_time, obstacles, is_reassignment=False):
        if not self.tasks:
            return
            
        idle_agents = [agent for agent in agents if agent.idle and not agent.failed]
        
        for agent in idle_agents:
            if not self.tasks:
                break
                
            best_task_idx = 0
            best_dist = float('inf')
            
            for i, task in enumerate(self.tasks):
                dist = heuristic(agent.pos, task)
                if dist < best_dist:
                    best_dist = dist
                    best_task_idx = i
                    
            assigned_task = self.tasks.pop(best_task_idx)
            agent.goal = assigned_task
            agent.idle = False
            agent.urgency = current_time
            if is_reassignment:
                print(f"\n[{agent.name}] TASK_REASSIGNED")
                print(f"[{agent.name}] task={assigned_task}")
                print(f"[{agent.name}] robot={agent.name}")
            else:
                print(f">>> Task Allocator: Assigned task {assigned_task} to {agent.name}")
            
            all_obstacles = agent.obstacles.union(set(agent.blackboard.get_active_dynamic_obstacles()))
            agent.plan_and_request(current_time, all_obstacles)


def simulate_fleet():
    usd_file = r"C:\Users\goruv\Downloads\simulation5.usd"
    print(f"Opening stage {usd_file}...")
    stage = Usd.Stage.Open(usd_file)
    if not stage:
        print("Failed to open stage")
        sys.exit(1)

    # Note: I will keep Phase 1-4 intact and just add Phase 5-7.
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    static_obstacles = set()
    
    # --- PHASE 1: Regression Test (STEP 12) ---
    print("\n===========================================")
    print("PHASE 1: Deterministic Conflict Scenario (Step 12 Regression)")
    print("===========================================")
    agents = [
        AMR("AMR_01", (-12, 10), stage, blackboard),
        AMR("AMR_02", (-10, 12), stage, blackboard)
    ]
    agents[0].urgency = 0
    agents[1].urgency = 1
    
    task_manager = TaskManager([
        (-8, 10),
        (-10, 8)
    ])
        
    for step in range(8):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        task_manager.allocate_tasks(agents, current_time, static_obstacles)
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)
            
    # Reset for Phase 2
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    
    # --- PHASE 2: Deadlock Test (STEP 13) ---
    print("\n===========================================")
    print("PHASE 2: Deterministic Deadlock Scenario (Step 13)")
    print("===========================================")
    agents = [
        AMR("AMR_01", (-11, -11), stage, blackboard),
        AMR("AMR_02", (-10, -11), stage, blackboard)
    ]
    agents[0].urgency = 0
    agents[1].urgency = 1
    agents[0].goal = (-10, -11) 
    agents[1].goal = (-11, -11) 
    agents[0].idle = False
    agents[1].idle = False
    
    for x in range(-15, -8):
        for y in range(-15, -8):
            if (x, y) not in [(-11, -11), (-10, -11), (-11, -10)]: 
                static_obstacles.add((x, y))

    network.broadcast_state(agents, 10)
    for amr in agents:
        amr.plan_and_request(10, static_obstacles)
        
    for step in range(12):
        current_time = step + 10 
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)
            
    # Reset for Phase 3
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    static_obstacles = set()
    
    # --- PHASE 3: Dynamic Blocked Aisle (STEP 14) ---
    print("\n===========================================")
    print("PHASE 3: Dynamic Blocked Aisle Scenario (Step 14)")
    print("===========================================")
    agents = [
        AMR("AMR_01", (0, 0), stage, blackboard),
        AMR("AMR_02", (0, -2), stage, blackboard),
        AMR("AMR_03", (0, -3), stage, blackboard)
    ]
    agents[0].goal = (4, 0)
    agents[1].goal = (2, -2)
    agents[2].goal = (2, -3)
    
    for a in agents:
        a.idle = False
        
    for x in range(0, 5):
        static_obstacles.add((x, 2))
        static_obstacles.add((x, -1))
    static_obstacles.add((0, 1))
    static_obstacles.add((4, 1))
    
    dyn_obs = DynamicObstacle("OBSTACLE_01", (3, 0), activation_time=2)
    blackboard.add_dynamic_obstacle(dyn_obs)
    
    network.broadcast_state(agents, 0)
    for amr in agents:
        amr.plan_and_request(0, static_obstacles)
        
    for step in range(8):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        
        for obs in blackboard.dynamic_obstacles:
            if current_time >= obs.activation_time and not obs.active:
                obs.active = True
                print(f">>> Dynamic Obstacle {obs.obstacle_id} ACTIVATED at {obs.pos}")
                
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)

    # Reset for Phase 4
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    static_obstacles = set()
    
    # --- PHASE 4: Robot Failure (STEP 15) ---
    print("\n===========================================")
    print("PHASE 4: Robot Failure & Task Reassignment (Step 15)")
    print("===========================================")
    
    agents = [
        AMR("AMR_01", (6, 10), stage, blackboard),
        AMR("AMR_02", (6, 8), stage, blackboard),
        AMR("AMR_03", (6, 6), stage, blackboard)
    ]
    
    task_manager = TaskManager([(10, 10), (10, 8), (10, 6)])
    
    for step in range(12):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        
        if current_time == 2:
            agents[0].trigger_failure(current_time, task_manager)
            
        task_manager.allocate_tasks(agents, current_time, static_obstacles, is_reassignment=(current_time >= 2))
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and not amr.failed and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)

    # --- PHASE 5: Temporary Communication Degradation (STEP 16 - Test 1) ---
    print("\n===========================================")
    print("PHASE 5: Temporary Communication Degradation (Step 16 - Test 1)")
    print("===========================================")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    static_obstacles = set()
    
    print("\n[SCENARIO LOG]")
    print("scenario_id=S7_TEMP_DEGRADED")
    print("seed=42")
    print("robot_count=2")
    print("T_DEGRADED=2")
    print("T_SAFE_MODE=4")
    print("failure_start_time=2")
    print("failure_end_time=5")
    print("sender=AMR_02")
    print("receiver=AMR_01")
    
    agents = [
        AMR("AMR_01", (20, 20), stage, blackboard, t_degraded=2, t_safe_mode=4),
        AMR("AMR_02", (20, 24), stage, blackboard, t_degraded=2, t_safe_mode=4)
    ]
    agents[0].goal = (20, 22)
    agents[1].goal = (20, 22)
    agents[0].idle = False
    agents[1].idle = False
    
    # Inject fault: AMR_02 to AMR_01 messages dropped from t=2 to t=5
    network.inject_fault("AMR_02", "AMR_01", 2, 5)
    
    for amr in agents:
        amr.plan_and_request(0, static_obstacles)
        
    for step in range(8):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)
            
    print(f"\nMetrics for Phase 5:")
    print(f"messages_sent: {network.metrics['messages_sent']}")
    print(f"messages_received: {network.metrics['messages_received']}")
    print(f"messages_dropped: {network.metrics['messages_dropped']}")

    # --- PHASE 6: Persistent Communication Failure (STEP 16 - Test 2) ---
    print("\n===========================================")
    print("PHASE 6: Persistent Communication Failure (Step 16 - Test 2)")
    print("===========================================")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    static_obstacles = set()
    
    print("\n[SCENARIO LOG]")
    print("scenario_id=S8_PERSISTENT_FAIL")
    print("seed=42")
    print("robot_count=2")
    print("T_DEGRADED=2")
    print("T_SAFE_MODE=4")
    print("failure_start_time=2")
    print("failure_end_time=inf")
    print("sender=AMR_02")
    print("receiver=AMR_01")
    
    agents = [
        AMR("AMR_01", (30, 30), stage, blackboard, t_degraded=2, t_safe_mode=4),
        AMR("AMR_02", (30, 34), stage, blackboard, t_degraded=2, t_safe_mode=4)
    ]
    agents[0].goal = (30, 32)
    agents[1].goal = (30, 32)
    agents[0].idle = False
    agents[1].idle = False
    
    # Inject persistent fault
    network.inject_fault("AMR_02", "AMR_01", 2, float('inf'))
    
    for amr in agents:
        amr.plan_and_request(0, static_obstacles)
        
    for step in range(8):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)
            
    print(f"\nMetrics for Phase 6:")
    print(f"messages_sent: {network.metrics['messages_sent']}")
    print(f"messages_received: {network.metrics['messages_received']}")
    print(f"messages_dropped: {network.metrics['messages_dropped']}")
    
    # --- PHASE 7: Partial Fleet Communication (STEP 16 - Test 3) ---
    print("\n===========================================")
    print("PHASE 7: Partial Fleet Communication (Step 16 - Test 3)")
    print("===========================================")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    static_obstacles = set()
    
    print("\n[SCENARIO LOG]")
    print("scenario_id=S9_PARTIAL_COMM")
    print("seed=42")
    print("robot_count=3")
    print("T_DEGRADED=2")
    print("T_SAFE_MODE=4")
    
    agents = [
        AMR("AMR_01", (40, 40), stage, blackboard, t_degraded=2, t_safe_mode=4),
        AMR("AMR_02", (40, 42), stage, blackboard, t_degraded=2, t_safe_mode=4),
        AMR("AMR_03", (40, 44), stage, blackboard, t_degraded=2, t_safe_mode=4)
    ]
    agents[0].goal = (40, 41)
    agents[1].goal = (40, 43)
    agents[2].goal = (40, 45)
    for a in agents: a.idle = False
    
    # AMR_02 to AMR_01 is dropped permanently after t=2, AMR_03 is fine
    network.inject_fault("AMR_02", "AMR_01", 2, float('inf'))
    
    for amr in agents:
        amr.plan_and_request(0, static_obstacles)
        
    for step in range(7):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)
            
    print(f"\nMetrics for Phase 7:")
    print(f"messages_sent: {network.metrics['messages_sent']}")
    print(f"messages_received: {network.metrics['messages_received']}")
    print(f"messages_dropped: {network.metrics['messages_dropped']}")

    # --- PHASE 8: Edge-AI Intersection Conflict Agreement (STEP 17 - Test 1) ---
    print("\n===========================================")
    print("PHASE 8: Edge-AI Intersection Conflict Agreement (Step 17 - Test 1)")
    print("===========================================")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    static_obstacles = set()
    
    print("\n[SCENARIO LOG]")
    print("scenario_id=S10_EDGE_AI_AGREEMENT_CONFLICT")
    print("seed=42")
    print("robot_count=2")
    
    agents = [
        AMR("AMR_01", (40, 40), stage, blackboard),
        AMR("AMR_02", (41, 39), stage, blackboard)
    ]
    agents[0].goal = (42, 40)
    agents[1].goal = (41, 41)
    for a in agents: a.idle = False
    
    for amr in agents:
        amr.plan_and_request(0, static_obstacles)
        
    for step in range(4):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)

    # --- PHASE 9: Edge-AI Disagreement on Active Conflict (STEP 17 - Test 2) ---
    print("\n===========================================")
    print("PHASE 9: Edge-AI Disagreement on Active Conflict (Step 17 - Test 2)")
    print("===========================================")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    
    print("\n[SCENARIO LOG]")
    print("scenario_id=S11_EDGE_AI_DISAGREEMENT_CONFLICT")
    print("seed=42")
    print("robot_count=2")
    
    agents = [
        AMR("AMR_01", (40, 40), stage, blackboard),
        AMR("AMR_02", (41, 39), stage, blackboard)
    ]
    agents[0].goal = (42, 40)
    agents[1].goal = (41, 41)
    for a in agents: a.idle = False
    
    # AMR_02 simulates aggressive AI that attempts to PROCEED despite conflicts
    agents[1].simulated_ai_intent = "PROCEED"
    
    for amr in agents:
        amr.plan_and_request(0, static_obstacles)
        
    for step in range(3):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)

    # --- PHASE 10: Dynamic Obstacle Reroute Agreement (STEP 17 - Test 3) ---
    print("\n===========================================")
    print("PHASE 10: Dynamic Obstacle Reroute Agreement (Step 17 - Test 3)")
    print("===========================================")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    
    print("\n[SCENARIO LOG]")
    print("scenario_id=S12_EDGE_AI_REROUTE_OBSTACLE")
    print("seed=42")
    print("robot_count=1")
    
    agent = AMR("AMR_01", (40, 40), stage, blackboard)
    agent.goal = (44, 40)
    agent.idle = False
    agents = [agent]
    
    agent.plan_and_request(0, static_obstacles)
    
    # Dynamic obstacle placed at (42, 40) at t=1
    dyn_obs = DynamicObstacle("PALLET_DROP", (42, 40), 1)
    blackboard.add_dynamic_obstacle(dyn_obs)
    
    for step in range(5):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        if current_time == 1:
            dyn_obs.active = True
            print(f"[ENVIRONMENT] Dynamic obstacle activated at (42, 40)")
            
        network.broadcast_state(agents, current_time)
        agent.step(current_time)

    # --- PHASE 11: Communication Degradation Agreement (STEP 17 - Test 4) ---
    print("\n===========================================")
    print("PHASE 11: Communication Degradation Agreement (Step 17 - Test 4)")
    print("===========================================")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    
    print("\n[SCENARIO LOG]")
    print("scenario_id=S13_EDGE_AI_COMM_DEGRADED")
    print("seed=42")
    print("robot_count=2")
    
    agents = [
        AMR("AMR_01", (40, 40), stage, blackboard, t_degraded=2, t_safe_mode=5),
        AMR("AMR_02", (40, 43), stage, blackboard, t_degraded=2, t_safe_mode=5)
    ]
    agents[0].goal = (40, 42)
    agents[1].goal = (40, 41)
    for a in agents: a.idle = False
    
    # Drop packets from AMR_02 to AMR_01 from t=1 to t=3
    network.inject_fault("AMR_02", "AMR_01", 1, 3)
    
    for amr in agents:
        amr.plan_and_request(0, static_obstacles)
        
    for step in range(5):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        for amr in agents:
            amr.step(current_time)

    # --- PHASE 12: Safe Mode Disagreement (STEP 17 - Test 5) ---
    print("\n===========================================")
    print("PHASE 12: Safe Mode Disagreement (Step 17 - Test 5)")
    print("===========================================")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    
    print("\n[SCENARIO LOG]")
    print("scenario_id=S14_EDGE_AI_SAFE_MODE_DISAGREEMENT")
    print("seed=42")
    print("robot_count=2")
    
    agents = [
        AMR("AMR_01", (40, 40), stage, blackboard, t_degraded=1, t_safe_mode=2),
        AMR("AMR_02", (40, 42), stage, blackboard, t_degraded=1, t_safe_mode=2)
    ]
    agents[0].goal = (40, 41)
    agents[1].goal = (40, 43)
    for a in agents: a.idle = False
    
    # AMR_01 tries to PROCEED even during safe mode
    agents[0].simulated_ai_intent = "PROCEED"
    network.inject_fault("AMR_02", "AMR_01", 1, float('inf'))
    
    for amr in agents:
        amr.plan_and_request(0, static_obstacles)
        
    for step in range(4):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        for amr in agents:
            amr.step(current_time)

    # --- PHASE 13: Deadlock Recovery Agreement (STEP 17 - Test 6) ---
    print("\n===========================================")
    print("PHASE 13: Deadlock Recovery Agreement (Step 17 - Test 6)")
    print("===========================================")
    blackboard = SharedBlackboard()
    network = CommunicationNetwork()
    
    print("\n[SCENARIO LOG]")
    print("scenario_id=S15_EDGE_AI_DEADLOCK_AGREEMENT")
    print("seed=42")
    print("robot_count=2")
    
    agents = [
        AMR("AMR_01", (10, 10), stage, blackboard),
        AMR("AMR_02", (13, 10), stage, blackboard)
    ]
    agents[0].goal = (13, 10)
    agents[1].goal = (10, 10)
    for a in agents: a.idle = False
    
    corridor_obs = {(x, y) for x in range(9, 15) for y in [9, 11]}
    
    for amr in agents:
        amr.plan_and_request(0, corridor_obs)
        
    for step in range(6):
        current_time = step
        print(f"\n--- Timestep {current_time} ---")
        network.broadcast_state(agents, current_time)
        
        resolved_all = False
        while not resolved_all:
            resolved_all = True
            for amr in agents:
                if not amr.idle and amr.current_res and not amr.current_res.valid:
                    amr.resolve_conflicts(current_time)
                    resolved_all = False
                    
        for amr in agents:
            amr.step(current_time)

    # --- GLOBAL METRICS & LATENCY REPORT ---
    print("\n===========================================")
    print("STEP 17: EDGE-AI & SAFETY ARBITER SUMMARY")
    print("===========================================")
    print(f"Total AI Decisions: {SafetyArbiter.metrics['ai_decisions']}")
    print(f"Total AI-Safety Agreements: {SafetyArbiter.metrics['ai_agreements']}")
    print(f"Total AI-Safety Disagreements: {SafetyArbiter.metrics['ai_disagreements']}")
    
    all_latencies = []
    for a in agents:
        all_latencies.extend(a.edge_ai.latencies)
        
    if all_latencies:
        min_lat = min(all_latencies)
        max_lat = max(all_latencies)
        avg_lat = sum(all_latencies) / len(all_latencies)
        print(f"AI Decision Latency: Min={min_lat:.4f}ms, Avg={avg_lat:.4f}ms, Max={max_lat:.4f}ms")
    print("===========================================\n")

if __name__ == "__main__":
    simulate_fleet()
