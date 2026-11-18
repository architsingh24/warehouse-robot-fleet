"""Fleet simulation: three coordination modes on the same task set.

mode "blind"    : independent A*, executed blindly (conflicts are only counted).
mode "reactive" : independent A*, a robot waits if the next cell is occupied.
mode "reactive_replan": as reactive, but after waiting 3+ ticks it replans around the other robots.
mode "reserved" : prioritised space-time A* with a reservation table.

One tick = 1 s (a robot moves one 0.5 m cell per tick).
"""
import time
from planner import astar, st_astar, Reservations
from warehouse import HOMES

DWELL = 2          # ticks spent picking / dropping
MAX_TICKS = 700
DEADLOCK_TICKS = 20


def waypoints_for(i, tasks_i):
    """Sequence of (cell, kind) the robot must visit: pick, drop, ..., home."""
    wp = []
    for pick, drop in tasks_i:
        wp.append((pick, "pick"))
        wp.append((drop, "drop"))
    wp.append((HOMES[i], "home"))
    return wp


def count_conflicts(paths):
    """Count vertex and swap conflict events (a continuing conflict counts once)."""
    n = len(paths)
    T = max(len(p) for p in paths)
    pos = [p + [p[-1]] * (T - len(p)) for p in paths]
    prev = set()
    events = 0
    for t in range(T):
        now = set()
        for i in range(n):
            for j in range(i + 1, n):
                if pos[i][t] == pos[j][t]:
                    now.add((i, j, "v"))
                elif t + 1 < T and pos[i][t] == pos[j][t + 1] and pos[i][t + 1] == pos[j][t]:
                    now.add((i, j, "s"))
        events += len(now - prev)
        prev = now
    return events


def _summarise(paths, drop_times, n_tasks_total, plan_time, deadlock, failed):
    T = max(len(p) for p in paths)
    moves = sum(1 for p in paths for a, b in zip(p, p[1:]) if a != b)
    waits = sum(1 for p in paths for a, b in zip(p, p[1:]) if a == b)
    done = len(drop_times)
    makespan = T - 1
    return {
        "collisions": count_conflicts(paths),
        "deadlock": int(deadlock),
        "failed_plans": failed,
        "tasks_done": done,
        "tasks_total": n_tasks_total,
        "makespan_s": makespan,
        "tasks_per_min": round(done * 60.0 / max(makespan, 1), 3),
        "moves": moves,
        "waits": waits,
        "plan_time_s": round(plan_time, 4),
    }


# ----------------------------------------------------------------- blind
def run_blind(grid, tasks, heuristic="manhattan"):
    t_start = time.perf_counter()
    paths, drop_times = [], []
    for i, tasks_i in enumerate(tasks):
        cur = HOMES[i]
        path = [cur]
        for cell, kind in waypoints_for(i, tasks_i):
            seg, _ = astar(grid, cur, cell, heuristic)
            path.extend(seg[1:])
            cur = cell
            if kind in ("pick", "drop"):
                path.extend([cur] * DWELL)
                if kind == "drop":
                    drop_times.append(len(path) - 1)
        paths.append(path)
    plan_time = time.perf_counter() - t_start
    n_total = sum(len(t) for t in tasks)
    return _summarise(paths, drop_times, n_total, plan_time, False, 0), paths


# -------------------------------------------------------------- reactive
def run_reactive(grid, tasks, heuristic="manhattan", replan_after=None):
    n = len(tasks)
    t_start = time.perf_counter()
    plans = []   # per robot: list of (cell, kind) targets
    pos = [HOMES[i] for i in range(n)]
    wp = [waypoints_for(i, tasks[i]) for i in range(n)]
    idx = [0] * n
    route = [[] for _ in range(n)]
    dwell = [0] * n
    finished = [False] * n
    blocked = [0] * n
    hist = [[pos[i]] for i in range(n)]
    drop_times = []
    idle = 0
    deadlock = False
    plan_time = 0.0

    def start_leg(i):
        nonlocal plan_time
        t0 = time.perf_counter()
        seg, _ = astar(grid, pos[i], wp[i][idx[i]][0], heuristic)
        plan_time += time.perf_counter() - t0
        route[i] = seg[1:]

    for i in range(n):
        start_leg(i)

    for t in range(1, MAX_TICKS):
        if all(finished):
            break
        new = list(pos)
        progressed = False
        for i in range(n):
            if finished[i]:
                continue
            if dwell[i] > 0:
                dwell[i] -= 1
                progressed = True
                if dwell[i] == 0:
                    if wp[i][idx[i]][1] == "drop":
                        drop_times.append(t)
                    idx[i] += 1
                    if idx[i] >= len(wp[i]):
                        finished[i] = True
                    else:
                        start_leg(i)
                continue
            if route[i]:
                nxt = route[i][0]
                if all(new[j] != nxt for j in range(n) if j != i):
                    new[i] = nxt
                    route[i].pop(0)
                    progressed = True
                    blocked[i] = 0
                else:
                    blocked[i] += 1
                    if replan_after is not None and blocked[i] >= replan_after + i:
                        t0 = time.perf_counter()
                        g2 = [row[:] for row in grid]
                        for j in range(n):
                            if j != i:
                                g2[pos[j][0]][pos[j][1]] = 1
                        goal = wp[i][idx[i]][0]
                        if g2[goal[0]][goal[1]] == 0:
                            seg, _ = astar(g2, pos[i], goal, heuristic)
                            if seg is not None and len(seg) > 1:
                                route[i] = seg[1:]
                        plan_time += time.perf_counter() - t0
                        blocked[i] = 0
            if not route[i] and new[i] == wp[i][idx[i]][0] and dwell[i] == 0:
                kind = wp[i][idx[i]][1]
                if kind == "home":
                    finished[i] = True
                    progressed = True
                else:
                    dwell[i] = DWELL
                    progressed = True
        pos = new
        for i in range(n):
            hist[i].append(pos[i])
        idle = 0 if progressed else idle + 1
        if idle >= DEADLOCK_TICKS:
            deadlock = True
            break
    n_total = sum(len(t) for t in tasks)
    return _summarise(hist, drop_times, n_total, plan_time, deadlock, 0), hist


# -------------------------------------------------------------- reserved
def run_reserved(grid, tasks, max_t=MAX_TICKS):
    n = len(tasks)
    t_start = time.perf_counter()
    res = Reservations()
    for i in range(n):
        res.v.add((HOMES[i], 0))       # every robot occupies its home at t = 0
    paths, drop_times = [], []
    failed = 0
    for i in range(n):
        cur = HOMES[i]
        path = [cur]
        t = 0
        ok = True
        for cell, kind in waypoints_for(i, tasks[i]):
            dw = DWELL if kind in ("pick", "drop") else 0
            seg, _ = st_astar(grid, cur, cell, t, res, dwell=dw, max_t=max_t,
                              forever=(kind == "home"))
            if seg is None:
                failed += 1
                ok = False
                break
            path.extend(seg[1:])
            t = len(path) - 1
            cur = cell
            if dw:
                path.extend([cur] * DWELL)
                t = len(path) - 1
                if kind == "drop":
                    drop_times.append(t)
        res.add_path(path)
        res.add_park(path[-1], len(path) - 1)
        paths.append(path)
    plan_time = time.perf_counter() - t_start
    n_total = sum(len(t) for t in tasks)
    return _summarise(paths, drop_times, n_total, plan_time, False, failed), paths


MODES = {
    "blind": run_blind,
    "reactive": run_reactive,
    "reactive_replan": lambda g, t: run_reactive(g, t, replan_after=3),
    "reserved": run_reserved,
}
