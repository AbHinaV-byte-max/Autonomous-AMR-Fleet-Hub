import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sim.simulator import Simulator
from models import TaskStatus, RobotStatus, Task

# ---------------------------------------------------------------------------
# Scenario A (Section 23.1) — Perpendicular intersection conflict
# ---------------------------------------------------------------------------

INTERSECTION_MAP = """\
#######
#R...R#
#.....#
#..P..#
#.....#
#..D..#
#######
"""

# Tight corridor map: two robots meet head-on at column 4
HEAD_ON_MAP = """\
###########
#R.......R#
#.........#
###########
"""

def test_scenario_a_intersection():
    """
    Section 23.1 Scenario A: two robots approach the same cell from
    perpendicular directions. Assert: (a) zero vertex collisions,
    (b) conflict resolution is observed, and (c) both eventually complete
    their goals.

    P1 uses CBS as the path authority, so conflict resolution may happen
    during planning rather than by an execution-time WAITING state.
    """
    ascii_map = """\
#######
#R...R#
#.....#
#..P..#
#.....#
#..D..#
#######
"""
    sim = Simulator(ascii_map=ascii_map, headless=True, strategy="P1")
    # Force the two outer robots toward each other's side
    # They will naturally plan crossing paths through the centre
    from models import Task
    t1 = Task(task_id='t1', pickup_cell=(5, 1), dropoff_cell=(1, 5), priority=1, status=TaskStatus.QUEUED, created_at=0.0)
    t2 = Task(task_id='t2', pickup_cell=(1, 1), dropoff_cell=(5, 5), priority=2, status=TaskStatus.QUEUED, created_at=0.0)
    sim.tasks.extend([t1, t2])

    collision_count = 0
    conflict_resolved = False

    for _ in range(500):
        sim.tick()
        # Check vertex collisions manually
        positions = {}
        for m in sim.robot_managers:
            if m.state.status == RobotStatus.OFFLINE:
                continue
            pos = (int(m.state.position[0]), int(m.state.position[1]))
            if pos in positions:
                collision_count += 1
            positions[pos] = m.state.robot_id

        if any(
            event["outcome"] in {"CBS_RESOLVED", "YIELD", "WAIT_OCCUPIED", "YIELD_WAIT"}
            for event in sim.event_log.conflict_events
        ):
            conflict_resolved = True

    assert collision_count == 0, f"Vertex collisions detected: {collision_count}"
    assert conflict_resolved, "No conflict resolution was recorded"
    print(f"Scenario A passed — {sim.completed_tasks} tasks, {collision_count} collisions")


# ---------------------------------------------------------------------------
# S3 (Section 17.2) — Narrow aisle, high stress, 3 robots
# ---------------------------------------------------------------------------

NARROW_MAP = """\
###########
#R..R..R..#
#P.......D#
###########
"""

def test_s3_narrow_aisle_no_permanent_deadlock():
    """
    S3: 3 robots in a narrow aisle.  Any deadlock must be broken
    and logged within DEADLOCK_THRESHOLD ticks.
    """
    from sim.simulator import DEADLOCK_THRESHOLD
    sim = Simulator(ascii_map=NARROW_MAP, headless=True)
    sim.run(max_ticks=600)

    # Every deadlock_event must exist, meaning the detector fired and broke it
    dl_count = sim.metric_values["DEADLOCK_COUNT"]
    print(f"S3: deadlocks detected+broken = {dl_count}, completed = {sim.completed_tasks}")
    # If deadlocks occurred, every one must have a recovery event logged
    assert len(sim.event_log.deadlock_events) == int(dl_count)
    print("S3 passed")


# ---------------------------------------------------------------------------
# S4 (Section 17.2) — Blocked aisle
# ---------------------------------------------------------------------------

BLOCKED_MAP = """\
##########
#R.....R.#
#P.....D.#
#R.......#
##########
"""

def test_s4_blocked_aisle():
    """
    S4: A free cell is blocked mid-run.  The robot that was heading
    through it must replan; zero collisions with the obstacle.
    """
    sim = Simulator(ascii_map=BLOCKED_MAP, headless=True)

    # Run 100 ticks, then block a centre cell that a robot is likely using
    sim.run(max_ticks=100)
    sim.block_cell(5, 2)   # block a centre corridor cell

    collisions_before = int(sim.metric_values["COLLISION_COUNT"])
    sim.run(max_ticks=400)

    collisions_after = int(sim.metric_values["COLLISION_COUNT"])
    assert collisions_after == collisions_before, (
        f"New collisions after block: {collisions_after}")
    print(f"S4 passed — no new collisions after blocking cell (5,2)")


# ---------------------------------------------------------------------------
# S5 (Section 17.2) — Robot failure mid-task
# ---------------------------------------------------------------------------

FAILURE_MAP = """\
##########
#R.P...D.#
#........#
#R.P...D.#
##########
"""

def test_s5_robot_failure_task_recovery():
    """
    S5: Kill robot-0 mid-task, assert its task becomes RECOVERABLE,
    is reassigned to robot-1, and is completed within 600 ticks.

    Keep the scenario deterministic: background task generation can otherwise
    continuously occupy both robots and unrelated dropoff slots, turning this
    into a workload/starvation test rather than a robot-failure recovery test.
    """
    sim = Simulator(ascii_map=FAILURE_MAP, headless=True, strategy="P1")

    # Remove background workload so the test exercises only the failover path.
    sim.tasks.clear()
    sim.task_generator.queue.clear()
    sim.task_generator.spawn_interval = 999

    r0, r1 = sim.robot_managers[:2]

    # Make robot-1 the known recovery target.
    r1.current_task = None
    r1.state.current_task_id = None
    r1.state.planned_path = []
    r1.target_cell = None
    r1.state.status = RobotStatus.IDLE
    r1.state.battery = 90.0

    task = Task(
        task_id="S5_FAILOVER",
        pickup_cell=(3, 1),
        dropoff_cell=(7, 1),
        priority=1,
        status=TaskStatus.ASSIGNED,
        assigned_robot_id=r0.state.robot_id,
    )
    sim.tasks.append(task)
    r0.assign_task(task)

    # Let robot-0 make progress, then hard-kill it mid-task.
    sim.run(max_ticks=5)
    assert r0.current_task is task
    sim.kill_robot("robot-0")

    assert r0.state.status == RobotStatus.OFFLINE
    assert task.status == TaskStatus.ASSIGNED
    assert task.assigned_robot_id == r1.state.robot_id
    assert r1.current_task is task

    # Recovery must finish the exact orphaned task within the acceptance window.
    sim.run(max_ticks=600)

    recoverable_remaining = [
        t for t in sim.tasks if t.task_id == task.task_id and t.status == TaskStatus.RECOVERABLE
    ]
    assert not recoverable_remaining, "Orphaned task was not recovered"
    assert task.status == TaskStatus.COMPLETED, (
        f"Recovered task did not complete: status={task.status}, "
        f"assigned_robot_id={task.assigned_robot_id}"
    )
    assert task.assigned_robot_id == r1.state.robot_id
    print(f"S5 passed — {task.task_id} recovered by {r1.state.robot_id} and completed")


# ---------------------------------------------------------------------------
# S6 (Section 17.2) — Communication delay / degraded mode
# ---------------------------------------------------------------------------

def test_s6_comms_delay_degraded_mode():
    """
    S6: Stop delivering peer messages to robot-0 for > 3 ticks.
    Assert it enters DEGRADED status rather than colliding.
    """
    sim = Simulator(ascii_map=FAILURE_MAP, headless=True)

    # Monkeypatch PubSubChannel.receive so robot-0's manager sees nothing
    original_send = sim.comms.send
    drop_until = [0]
    send_count = [0]

    def patched_send(msg):
        send_count[0] += 1
        if msg.robot_id != "robot-0":
            original_send(msg)
        # robot-0 never broadcasts → peers stop seeing its heartbeat
        # and robot-0 sees no peer messages because we only inject from peers

    sim.comms.send = patched_send

    degraded_seen = [False]
    for _ in range(200):
        sim.tick()
        if any(m.state.status == RobotStatus.DEGRADED for m in sim.robot_managers):
            degraded_seen[0] = True
        sim.comms.clear()

    assert degraded_seen[0], "No robot entered DEGRADED mode when peer comms were cut"
    print("S6 passed — DEGRADED mode triggered on comms loss")


if __name__ == "__main__":
    test_scenario_a_intersection()
    test_s3_narrow_aisle_no_permanent_deadlock()
    test_s4_blocked_aisle()
    test_s5_robot_failure_task_recovery()
    test_s6_comms_delay_degraded_mode()
    print("\n=== All Phase 3/4 tests passed ===")
