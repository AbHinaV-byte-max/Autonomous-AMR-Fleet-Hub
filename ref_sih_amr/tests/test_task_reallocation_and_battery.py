import sys, os
import pytest
from sim.simulator import Simulator
from models import RobotStatus, TaskStatus, Task

GRID_MAP = """\
#######
#R...R#
#..P..#
#..D.C#
#..S..#
#######
"""

def test_task_reallocation_on_robot_shutdown():
    sim = Simulator(ascii_map=GRID_MAP, headless=True, strategy="B0")
    
    r0 = sim.robot_managers[0]
    r1 = sim.robot_managers[1]
    
    # Give r0 a specific task and make r1 available/idle
    task = Task("FAILOVER_TASK", pickup_cell=(3, 2), dropoff_cell=(3, 3), priority=1, status=TaskStatus.ASSIGNED, assigned_robot_id=r0.state.robot_id)
    sim.tasks.append(task)
    r0.assign_task(task)
    
    r1.current_task = None
    r1.state.current_task_id = None
    r1.state.planned_path = []
    r1.target_cell = None
    r1.state.status = RobotStatus.IDLE
    r1.state.battery = 85.0
    
    # Kill r0
    sim.kill_robot(r0.state.robot_id)
    
    # r0 should be OFFLINE
    assert r0.state.status == RobotStatus.OFFLINE
    
    # r1 should have automatically taken over FAILOVER_TASK!
    assert r1.current_task is not None
    assert r1.current_task.task_id == "FAILOVER_TASK"
    assert task.assigned_robot_id == r1.state.robot_id


def test_battery_drain_and_auto_recharge_failover():
    sim = Simulator(ascii_map=GRID_MAP, headless=True, strategy="B0")
    
    # Clear any background startup tasks so only our test task exists
    sim.tasks.clear()
    sim.task_generator.queue.clear()
    
    r0 = sim.robot_managers[0]
    r1 = sim.robot_managers[1]
    
    # Set r0 battery near critical threshold with an active task
    r0.state.battery = 20.1
    task = Task("BATTERY_TASK", pickup_cell=(3, 2), dropoff_cell=(3, 3), priority=1, status=TaskStatus.ASSIGNED, assigned_robot_id=r0.state.robot_id)
    sim.tasks.append(task)
    r0.assign_task(task)
    
    # Make r1 idle and ready to take over
    r1.current_task = None
    r1.state.current_task_id = None
    r1.state.planned_path = []
    r1.target_cell = None
    r1.state.status = RobotStatus.IDLE
    r1.state.battery = 90.0
    
    # Run ticks so battery dips below 20.0%
    sim.tick()
    
    # r0 must shed the task but travel to the fixed charger first.
    assert r0.state.status == RobotStatus.MOVING
    assert r0.post_task_mode == "CHARGER"
    assert r0.current_task is None
    assert r1.current_task is not None
    assert r1.current_task.task_id == "BATTERY_TASK"
    assert task.assigned_robot_id == r1.state.robot_id

    # The charger is a real map resource: reach it before entering CHARGING.
    assert r0.target_cell in sim.charger_cells
    assert r0.state.planned_path
    for _ in range(30):
        if r0.state.status == RobotStatus.CHARGING:
            break
        sim.tick()
    assert r0.state.status == RobotStatus.CHARGING
    initial_charge = r0.state.battery
    sim.tick()
    assert r0.state.battery > initial_charge, "Battery should increase while CHARGING"


def test_completed_robot_is_reassigned_without_waiting_for_periodic_allocator():
    sim = Simulator(ascii_map=GRID_MAP, headless=True, strategy="B0")
    sim.tasks.clear()
    sim.task_generator.queue.clear()
    sim.task_generator.spawn_interval = 999

    r0 = sim.robot_managers[0]
    r1 = sim.robot_managers[1]

    # Leave only r0 eligible so the follow-up task must return to the robot
    # that just finished its delivery.
    r1.state.status = RobotStatus.OFFLINE
    r1.current_task = None
    r1.state.current_task_id = None
    r1.state.planned_path = []
    r1.target_cell = None

    completed = Task(
        "COMPLETED_NOW",
        pickup_cell=(3, 2),
        dropoff_cell=(3, 3),
        priority=1,
        status=TaskStatus.IN_PROGRESS,
        assigned_robot_id=r0.state.robot_id,
    )
    follow_up = Task(
        "FOLLOW_UP",
        pickup_cell=(3, 2),
        dropoff_cell=(3, 3),
        priority=1,
        status=TaskStatus.QUEUED,
    )
    sim.tasks.extend([completed, follow_up])

    r0.current_task = completed
    r0.state.current_task_id = completed.task_id
    r0.state.status = RobotStatus.MOVING
    r0.state.position = (3.0, 3.0)
    r0.target_cell = completed.dropoff_cell
    r0.state.planned_path = []

    sim.tick()

    assert completed.status == TaskStatus.COMPLETED
    # The delivery cell is released and the robot immediately leaves it for
    # the fixed staging bay instead of taking another task at the dock.
    assert r0.current_task is None
    assert r0.post_task_mode == "STAGING"
    assert r0.state.status == RobotStatus.MOVING
    assert r0.target_cell in sim.staging_cells
    assert r0.state.planned_path
    assert follow_up.status == TaskStatus.QUEUED

    # Movement starts on the following simulation tick.
    sim.tick()
    assert (int(r0.state.position[0]), int(r0.state.position[1])) != (3, 3)

    # Once staged, the robot can be dispatched again by the normal allocator.
    # Stop as soon as the follow-up leaves QUEUED; it may complete quickly after
    # assignment, so asserting current_task at an arbitrary later tick is brittle.
    for _ in range(10):
        sim.tick()
        if follow_up.status != TaskStatus.QUEUED:
            break
    assert follow_up.assigned_robot_id == r0.state.robot_id
    assert follow_up.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS, TaskStatus.COMPLETED)


def test_completed_robot_routes_to_fixed_staging_bay():
    sim = Simulator(
        ascii_map="""\
########
#R.....#
#..P...#
#..D.S.#
#....C.#
########
""",
        headless=True,
        strategy="B0",
    )
    sim.tasks.clear()
    sim.task_generator.queue.clear()

    robot = sim.robot_managers[0]
    task = Task(
        "STAGE_AFTER_DELIVERY",
        pickup_cell=(3, 2),
        dropoff_cell=(3, 3),
        priority=1,
        status=TaskStatus.IN_PROGRESS,
        assigned_robot_id=robot.state.robot_id,
    )
    sim.tasks.append(task)
    robot.current_task = task
    robot.state.current_task_id = task.task_id
    robot.state.position = (3.0, 3.0)
    robot.state.status = RobotStatus.MOVING
    robot.target_cell = task.dropoff_cell
    robot.state.planned_path = []

    sim.tick()

    assert task.status == TaskStatus.COMPLETED
    assert robot.current_task is None
    assert robot.post_task_mode == "STAGING"
    assert robot.state.status == RobotStatus.MOVING
    assert robot.target_cell in sim.staging_cells
    assert robot.state.planned_path
    sim.tick()
    assert (int(robot.state.position[0]), int(robot.state.position[1])) != (3, 3)


def test_completed_robot_does_not_leave_long_future_reservation():
    sim = Simulator(ascii_map=GRID_MAP, headless=True, strategy="B0")
    sim.tasks.clear()
    sim.task_generator.queue.clear()

    r0 = sim.robot_managers[0]
    task = Task(
        "DELIVERY_DONE",
        pickup_cell=(3, 2),
        dropoff_cell=(3, 3),
        priority=1,
        status=TaskStatus.IN_PROGRESS,
        assigned_robot_id=r0.state.robot_id,
    )
    r0.current_task = task
    r0.state.current_task_id = task.task_id
    r0.state.status = RobotStatus.MOVING
    r0.state.timestamp = 10.0
    r0.state.position = (3.0, 3.0)
    r0.target_cell = task.dropoff_cell
    r0.state.planned_path = []

    r0._handle_arrival()

    assert task.status == TaskStatus.COMPLETED
    assert r0.state.status == RobotStatus.IDLE
    assert r0.reservation_table.get_claimer((3, 3), 10.0) == r0.state.robot_id
    assert r0.reservation_table.get_claimer((3, 3), 11.0) is None
    assert r0.reservation_table.get_claimer((3, 3), 209.0) is None


def test_allocator_does_not_stack_live_tasks_on_same_dropoff_slot():
    sim = Simulator(
        ascii_map="""\
#########
#R.....R#
#..D.D..#
#.......#
#########
""",
        headless=True,
        strategy="B0",
    )
    sim.tasks.clear()
    sim.task_generator.queue.clear()

    r0, r1 = sim.robot_managers[:2]
    r0.state.status = RobotStatus.MOVING
    active = Task(
        "ACTIVE_DELIVERY",
        pickup_cell=(3, 2),
        dropoff_cell=(3, 2),
        priority=1,
        status=TaskStatus.IN_PROGRESS,
        assigned_robot_id=r0.state.robot_id,
    )
    r0.current_task = active
    r0.state.current_task_id = active.task_id
    r0.target_cell = active.dropoff_cell

    r1.state.status = RobotStatus.IDLE
    r1.current_task = None
    r1.state.current_task_id = None
    r1.target_cell = None

    same_slot = Task(
        "SAME_SLOT",
        pickup_cell=(4, 2),
        dropoff_cell=(3, 2),
        priority=1,
        status=TaskStatus.QUEUED,
    )
    other_slot = Task(
        "OTHER_SLOT",
        pickup_cell=(4, 2),
        dropoff_cell=(5, 2),
        priority=1,
        status=TaskStatus.QUEUED,
    )
    sim.tasks.extend([same_slot, other_slot])

    sim._allocate()

    assert r1.current_task is not None
    assert r1.current_task.task_id == "OTHER_SLOT"
    assert same_slot.status == TaskStatus.QUEUED
    assert same_slot.assigned_robot_id is None


def test_allocator_does_not_assign_two_queued_tasks_to_same_free_dropoff():
    sim = Simulator(
        ascii_map="""\
#########
#R.....R#
#..D....#
#.......#
#########
""",
        headless=True,
        strategy="B0",
    )
    sim.tasks.clear()
    sim.task_generator.queue.clear()

    r0, r1 = sim.robot_managers[:2]
    for robot in (r0, r1):
        robot.state.status = RobotStatus.IDLE
        robot.current_task = None
        robot.state.current_task_id = None
        robot.state.planned_path = []
        robot.target_cell = None
        robot.state.battery = 90.0

    same_slot_a = Task(
        "SAME_FREE_SLOT_A",
        pickup_cell=(3, 2),
        dropoff_cell=(3, 2),
        priority=1,
        status=TaskStatus.QUEUED,
        created_at=1.0,
    )
    same_slot_b = Task(
        "SAME_FREE_SLOT_B",
        pickup_cell=(4, 2),
        dropoff_cell=(3, 2),
        priority=2,
        status=TaskStatus.QUEUED,
        created_at=2.0,
    )
    sim.tasks.extend([same_slot_a, same_slot_b])

    sim._allocate()

    assigned = [r.current_task for r in (r0, r1) if r.current_task is not None]
    assert len(assigned) == 1
    assert assigned[0].task_id == same_slot_a.task_id
    assert same_slot_b.status == TaskStatus.QUEUED
    assert same_slot_b.assigned_robot_id is None
