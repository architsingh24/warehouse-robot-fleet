"""Warehouse grid map. Cells are (row, col). 0 = free, 1 = shelf/obstacle."""
import random

H, W = 20, 30
CELL = 0.5  # metres per cell

SHELF_COLS = [(2, 3), (6, 7), (10, 11), (14, 15), (18, 19), (22, 23), (26, 27)]
SHELF_ROWS = [(3, 8), (11, 16)]
DROP_STATIONS = [(1, 4), (1, 8), (1, 12), (1, 16), (1, 20), (1, 24)]
HOMES = [(18, 4), (18, 8), (18, 12), (18, 16), (18, 20), (18, 24), (18, 6), (18, 14)]


def build_grid():
    grid = [[0] * W for _ in range(H)]
    for r0, r1 in SHELF_ROWS:
        for c0, c1 in SHELF_COLS:
            for r in range(r0, r1 + 1):
                for c in range(c0, c1 + 1):
                    grid[r][c] = 1
    return grid


def pickup_cells(grid):
    """Free cells inside the shelf rows that touch a shelf (picking positions)."""
    cells = []
    for r0, r1 in SHELF_ROWS:
        for r in range(r0, r1 + 1):
            for c in range(W):
                if grid[r][c] == 0:
                    for dr, dc in ((0, 1), (0, -1)):
                        cc = c + dc
                        if 0 <= cc < W and grid[r][cc] == 1:
                            cells.append((r, c))
                            break
    return cells


def make_tasks(grid, n_robots, n_tasks, seed):
    """Return tasks[i] = [(pickup, drop), ...] for robot i. Same seed -> same tasks."""
    rng = random.Random(seed)
    picks = pickup_cells(grid)
    return [[(rng.choice(picks), rng.choice(DROP_STATIONS)) for _ in range(n_tasks)]
            for _ in range(n_robots)]


def cell_to_world(cell):
    r, c = cell
    return (c * CELL + CELL / 2, (H - 1 - r) * CELL + CELL / 2)


def render_ascii(grid):
    return "\n".join("".join("#" if v else "." for v in row) for row in grid)


if __name__ == "__main__":
    g = build_grid()
    print(render_ascii(g))
    print("pickup cells:", len(pickup_cells(g)))
