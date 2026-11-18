"""Path planners: grid A* / Dijkstra and space-time A* with a reservation table."""
import heapq
import math
from itertools import count

DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclid(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def zero(a, b):
    return 0.0


HEURISTICS = {"manhattan": manhattan, "euclidean": euclid, "dijkstra": zero}


def in_free(grid, c):
    r, q = c
    return 0 <= r < len(grid) and 0 <= q < len(grid[0]) and grid[r][q] == 0


def astar(grid, start, goal, heuristic="manhattan"):
    """Returns (path, nodes_expanded). path is None if unreachable."""
    h = HEURISTICS[heuristic]
    tie = count()
    openq = [(h(start, goal), next(tie), 0, start)]
    g_best = {start: 0}
    parent = {start: None}
    closed = set()
    expanded = 0
    while openq:
        _, _, g, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1], expanded
        for dr, dc in DIRS:
            nxt = (cur[0] + dr, cur[1] + dc)
            if not in_free(grid, nxt):
                continue
            ng = g + 1
            if ng < g_best.get(nxt, 1e18):
                g_best[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(openq, (ng + h(nxt, goal), next(tie), ng, nxt))
    return None, expanded


class Reservations:
    """Reservation table shared by prioritised planning."""

    def __init__(self):
        self.v = set()      # (cell, t)
        self.e = set()      # (from, to, t): move from->to during [t, t+1]
        self.park = {}      # cell -> earliest time it is permanently occupied
        self.tmax = 0

    def move_ok(self, a, b, t):
        if (b, t + 1) in self.v:
            return False
        if (b, a, t) in self.e:       # head-on swap
            return False
        p = self.park.get(b)
        if p is not None and t + 1 >= p:
            return False
        return True

    def stay_ok(self, cell, t0, t1):
        for t in range(t0, t1 + 1):
            if (cell, t) in self.v:
                return False
            p = self.park.get(cell)
            if p is not None and t >= p:
                return False
        return True

    def park_ok(self, cell, t):
        """True if `cell` can be occupied permanently from time t."""
        if cell in self.park:
            return False
        return all((cell, tt) not in self.v for tt in range(t, self.tmax + 1))

    def add_path(self, path, t0=0):
        for i, c in enumerate(path):
            self.v.add((c, t0 + i))
            if i + 1 < len(path):
                self.e.add((c, path[i + 1], t0 + i))
        self.tmax = max(self.tmax, t0 + len(path))

    def add_park(self, cell, t):
        self.park[cell] = min(t, self.park.get(cell, 10 ** 9))


def st_astar(grid, start, goal, t0, res, dwell=0, max_t=500, forever=False):
    """Space-time A*. Returns (path, expanded); path[i] is the cell at time t0+i.
    The path ends once the robot has reached `goal` and the dwell window is free."""
    tie = count()
    s = (start, t0)
    openq = [(manhattan(start, goal), next(tie), 0, s)]
    parent = {s: None}
    closed = set()
    expanded = 0
    while openq:
        _, _, g, state = heapq.heappop(openq)
        if state in closed:
            continue
        closed.add(state)
        expanded += 1
        cell, t = state
        if cell == goal and res.stay_ok(goal, t, t + dwell) and (
                not forever or res.park_ok(goal, t)):
            path = []
            cur = state
            while cur is not None:
                path.append(cur[0])
                cur = parent[cur]
            return path[::-1], expanded
        if t + 1 > max_t:
            continue
        for dr, dc in DIRS + [(0, 0)]:
            nxt = (cell[0] + dr, cell[1] + dc)
            if not in_free(grid, nxt):
                continue
            if nxt == cell:
                ok = (cell, t + 1) not in res.v and not (
                    cell in res.park and t + 1 >= res.park[cell])
            else:
                ok = res.move_ok(cell, nxt, t)
            if not ok:
                continue
            ns = (nxt, t + 1)
            if ns in closed:
                continue
            if ns not in parent:
                parent[ns] = state
            heapq.heappush(openq, (g + 1 + manhattan(nxt, goal), next(tie), g + 1, ns))
    return None, expanded
