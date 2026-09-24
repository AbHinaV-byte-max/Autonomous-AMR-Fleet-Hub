import sys
import os

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sim.simulator import Simulator

def test_phase1():
    # Simple map with 3 robots, pickups, dropoffs, and free cells
    ascii_map = """
##########
#R..P...R#
#........#
#D..R...D#
##########
"""
    
    # 1. Initialize Simulator
    sim = Simulator(ascii_map=ascii_map, headless=True)
    
    # Assert initial state
    assert len(sim.robots) == 3, "Should spawn exactly 3 robots"
    assert sim.grid_map.width == 10
    assert sim.grid_map.height == 5
    
    # 2. Run simulation for 500 ticks
    sim.run(max_ticks=500)
    
    # 3. Assertions
    # (a) no crash - if we reached here, there was no crash
    # (b) at least 5 tasks reach COMPLETED
    assert sim.completed_tasks >= 5, f"Expected at least 5 completed tasks, but got {sim.completed_tasks}"
    
    # (c) all robots stay within map bounds
    for r in sim.robots:
        x, y = r.position
        assert 0 <= x < sim.grid_map.width, f"Robot {r.robot_id} x-coord out of bounds: {x}"
        assert 0 <= y < sim.grid_map.height, f"Robot {r.robot_id} y-coord out of bounds: {y}"
        
    print(f"Phase 1 Test Passed! Completed {sim.completed_tasks} tasks in 500 ticks.")

if __name__ == "__main__":
    test_phase1()
