import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import load_map
from robot.planner import AStarPlanner
from allocator.hungarian import HungarianAllocator
from sim.simulator import Simulator
from models import RobotState, Task, TaskStatus, RobotStatus

def test_astar_optimal():
    ascii_map = """
#####
#...#
#.#.#
#...#
#####
"""
    grid = load_map(ascii_map)
    planner = AStarPlanner()
    path = planner.plan((1, 1), (3, 3), grid)
    
    assert len(path) == 5, f"Expected path length 5 (4 steps + start cell if we included it, wait AStarPlanner doesn't include start unless start==goal or it just returns the steps). Let's check length."
    # AStarPlanner returns the sequence of cells excluding the start cell, but wait, the implementation:
    # It reconstructs from goal, so it includes goal. Does it include start? 
    # Yes, came_from[start_int] = None. So start is included.
    # From (1,1) to (3,3): (1,1)->(1,2)->(1,3)->(2,3)->(3,3). Length is 5.
    assert len(path) == 5, f"Expected path length 5, got {len(path)}"
    
    for x, y in path:
        assert grid.get_cell(x, y) != '#', f"Path hits obstacle at {x}, {y}"
    print("test_astar_optimal passed!")

def test_hungarian_allocator():
    ascii_map = """
#####
#...#
#...#
#...#
#####
"""
    grid = load_map(ascii_map)
    planner = AStarPlanner()
    allocator = HungarianAllocator(planner, grid)
    
    robots = [
        RobotState("r1", 0.0, (1, 1), 0.0, 0.0, 100, None, 0, status=RobotStatus.IDLE),
        RobotState("r2", 0.0, (1, 3), 0.0, 0.0, 100, None, 0, status=RobotStatus.IDLE),
    ]
    tasks = [
        Task("t1", (2, 1), (3, 3), 1, TaskStatus.QUEUED),
        Task("t2", (2, 3), (3, 1), 1, TaskStatus.QUEUED),
    ]
    
    assignments = allocator.allocate(robots, tasks)
    assert len(assignments) == 2
    assert assignments["r1"] == "t1"
    assert assignments["r2"] == "t2"
    print("test_hungarian_allocator passed!")

def test_phase2_sim():
    ascii_map = """
##########
#R..P...R#
#..####..#
#D..R...D#
##########
"""
    sim = Simulator(ascii_map=ascii_map, headless=True)
    sim.run(max_ticks=1000)
    
    assert sim.completed_tasks >= 5, f"Expected at least 5 tasks completed, got {sim.completed_tasks}"
    # Verify bounds
    for manager in sim.robot_managers:
        x, y = manager.state.position
        assert 0 <= x < sim.grid_map.width
        assert 0 <= y < sim.grid_map.height
    print(f"test_phase2_sim passed! Completed {sim.completed_tasks} tasks.")

if __name__ == "__main__":
    test_astar_optimal()
    test_hungarian_allocator()
    test_phase2_sim()
