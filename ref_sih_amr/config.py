"""
Configuration and map loader for the warehouse grid map.
Format:
# = shelf/obstacle
. = free cell
P = pickup
D = dropoff
R = robot spawn
"""
from typing import List, Tuple

class GridMap:
    def __init__(self, grid: List[List[str]]):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0]) if self.height > 0 else 0

    def get_cell(self, x: int, y: int) -> str:
        if 0 <= y < self.height and 0 <= x < self.width:
            return self.grid[y][x]
        return '#' # out of bounds is obstacle

    def find_all(self, char: str) -> List[Tuple[int, int]]:
        """Find all (x, y) coordinates of cells containing the specified character."""
        positions = []
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y][x] == char:
                    positions.append((x, y))
        return positions

def load_map(ascii_map: str) -> GridMap:
    """Parses an ASCII map into a GridMap."""
    grid = []
    for line in ascii_map.strip().splitlines():
        # Remove whitespace if needed, but keeping it simple for now
        row = list(line.strip())
        if row:
            grid.append(row)
    return GridMap(grid)
