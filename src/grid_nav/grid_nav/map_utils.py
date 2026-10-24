"""Pure-Python helpers (no ROS imports) so they can be unit-tested anywhere.

Text map convention (edit the constants below if your map_example.md differs):
    #  wall
    .  free cell
    R  robot start
    G  goal (optional)
The FIRST text row is the TOP of the map. ROS OccupancyGrid row 0 is the
BOTTOM, so we flip the rows (this is the classic "upside-down map" bug).
"""
from dataclasses import dataclass, field
import math

WALL = '#'
FREE = '.'
ROBOT = 'R'
GOAL = 'G'


@dataclass
class GridMap:
    width: int
    height: int
    walls: set = field(default_factory=set)   # {(x, y)}  y grows UPWARD
    start: tuple = None                         # (x, y) or None
    goal: tuple = None                          # (x, y) or None
    resolution: float = 1.0                     # metres per cell


def _tokens(line):
    """'# . . #' -> ['#','.','.','#'];  '#..#' -> ['#','.','.','#']"""
    parts = line.split()
    if len(parts) > 1 and all(len(p) == 1 for p in parts):
        return parts
    return list(line.strip())


def parse_text_map(text, resolution=1.0):
    rows = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith('```') or line.startswith('//'):
            continue  # allow markdown fences / comments
        rows.append(_tokens(line))
    if not rows:
        raise ValueError('map is empty')
    width = len(rows[0])
    for i, r in enumerate(rows):
        if len(r) != width:
            raise ValueError(f'row {i} has {len(r)} cells, expected {width}')
    height = len(rows)
    gm = GridMap(width, height, resolution=resolution)
    for r_idx, row in enumerate(rows):
        y = height - 1 - r_idx            # flip: first text row = top
        for x, ch in enumerate(row):
            if ch == WALL:
                gm.walls.add((x, y))
            elif ch == ROBOT:
                gm.start = (x, y)
            elif ch == GOAL:
                gm.goal = (x, y)
            elif ch != FREE:
                raise ValueError(f'unknown symbol {ch!r} at row {r_idx}, col {x}')
    return gm


def occupancy_data(gm):
    """Row-major list for nav_msgs/OccupancyGrid.data (0 free, 100 occupied)."""
    return [100 if (x, y) in gm.walls else 0
            for y in range(gm.height) for x in range(gm.width)]


def cell_to_world(cell, resolution=1.0):
    """Centre of a cell, in metres."""
    return ((cell[0] + 0.5) * resolution, (cell[1] + 0.5) * resolution)


def world_to_cell(x, y, resolution=1.0):
    return (math.floor(x / resolution), math.floor(y / resolution))


def path_to_cells(points_xy, resolution=1.0):
    """World points -> list of grid cells, consecutive duplicates removed."""
    cells = []
    for x, y in points_xy:
        c = world_to_cell(x, y, resolution)
        if not cells or cells[-1] != c:
            cells.append(c)
    return cells


def render(gm, path=None, robot=None, goal=None):
    """ASCII picture, top row printed first."""
    path = set(path or [])
    goal = goal or gm.goal
    lines = []
    for y in range(gm.height - 1, -1, -1):
        row = []
        for x in range(gm.width):
            c = (x, y)
            if robot is not None and c == robot:
                row.append('R')
            elif goal is not None and c == goal:
                row.append('G')
            elif c in gm.walls:
                row.append('#')
            elif c in path:
                row.append('*')
            else:
                row.append('.')
        lines.append(' '.join(row))
    return '\n'.join(lines)
