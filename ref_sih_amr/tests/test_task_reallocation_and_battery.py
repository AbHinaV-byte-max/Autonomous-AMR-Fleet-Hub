import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import pytest
from sim.simulator import Simulator
from models import RobotStatus, TaskStatus, Task

GRID_MAP = """\
#######
#R...R#
#..P..#
#..D..#
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
    
    # r0 should now be CHARGING, its task shed and handed over to r1
    assert r0.state.status == RobotStatus.CHARGING
    assert r0.current_task is None
    assert r1.current_task is not None
    assert r1.current_task.task_id == "BATTERY_TASK"
    assert task.assigned_robot_id == r1.state.robot_id
    
    # Continue ticking to verify charging increases battery level
    initial_charge = r0.state.battery
    sim.tick()
    assert r0.state.battery > initial_charge, "Battery should increase while CHARGING"
