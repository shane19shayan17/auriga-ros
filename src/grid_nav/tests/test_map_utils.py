from collections import deque
import os

from grid_nav.map_utils import (parse_text_map, occupancy_data, cell_to_world,
                                world_to_cell, path_to_cells, render)

MAP = os.path.join(os.path.dirname(__file__), '..', 'maps', 'map_example.txt')


def load():
    with open(MAP) as f:
        return parse_text_map(f.read())


def bfs(gm, s, g):
    q, seen = deque([s]), {s}
    while q:
        c = q.popleft()
        if c == g:
            return True
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + dx, c[1] + dy)
            if 0 <= n[0] < gm.width and 0 <= n[1] < gm.height \
                    and n not in gm.walls and n not in seen:
                seen.add(n)
                q.append(n)
    return False


def test_size_and_symbols():
    gm = load()
    assert (gm.width, gm.height) == (20, 20)
    assert gm.start == (0, 1)        # 'R' is on text row 18 -> y = 19-18 = 1
    assert gm.goal == (5, 10)


def test_solvable_4_connected():
    gm = load()
    assert bfs(gm, gm.start, gm.goal)


def test_occupancy_flip():
    gm = parse_text_map("#.\n..")      # wall is on the TOP-left
    data = occupancy_data(gm)
    assert data == [0, 0, 100, 0]      # row y=0 first (bottom), wall in y=1


def test_conversions():
    assert cell_to_world((3, 4)) == (3.5, 4.5)
    assert world_to_cell(3.5, 4.5) == (3, 4)
    assert path_to_cells([(0.5, 0.5), (0.6, 0.5), (1.5, 0.5)]) == [(0, 0), (1, 0)]


def test_spaced_format_and_fences():
    gm = parse_text_map("```\n# . R\n. . G\n```")
    assert (gm.width, gm.height) == (3, 2)
    assert gm.start == (2, 1) and gm.goal == (2, 0)


def test_render_roundtrip_has_robot():
    gm = load()
    assert 'R' in render(gm, robot=gm.start)
