"""Differential-drive (Pioneer P3-DX) heading controller and kinematic simulation."""
import math
from warehouse import cell_to_world

WHEEL_R = 0.0975   # m
AXLE = 0.331       # m
V_MAX = 0.6        # m/s used in experiments
W_MAX = 2.0        # rad/s


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def wheel_speeds(v, w):
    """Unicycle (v, w) -> left/right wheel angular speeds in rad/s."""
    return ((v - w * AXLE / 2) / WHEEL_R, (v + w * AXLE / 2) / WHEEL_R)


def command(pose, target, kw, last):
    """Heading P-controller. Returns (v, w)."""
    x, y, th = pose
    dx, dy = target[0] - x, target[1] - y
    dist = math.hypot(dx, dy)
    err = wrap(math.atan2(dy, dx) - th)
    w = clip(kw * err, -W_MAX, W_MAX)
    v = V_MAX * max(0.0, math.cos(err)) ** 2
    if last:
        v *= min(1.0, dist / 0.3)
    return v, w


def simplify(cells):
    """Drop collinear cells so the robot only needs the corner waypoints."""
    if len(cells) < 3:
        return list(cells)
    out = [cells[0]]
    for a, b, c in zip(cells, cells[1:], cells[2:]):
        if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
            out.append(b)
    out.append(cells[-1])
    return out


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = clip(((p[0] - ax) * dx + (p[1] - ay) * dy) / L2, 0, 1)
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def follow(cells, kw=2.0, dt=0.05, tol=0.12, tol_last=0.05, t_max=300.0):
    """Simulate following a cell path. Returns dict of metrics and the trace."""
    pts = [cell_to_world(c) for c in simplify(cells)]
    ref = [cell_to_world(c) for c in cells]
    x, y = pts[0]
    th = math.atan2(pts[1][1] - y, pts[1][0] - x) if len(pts) > 1 else 0.0
    k = 1
    t = 0.0
    trace = [(t, x, y, th)]
    max_cte, sum_cte = 0.0, 0.0
    while k < len(pts) and t < t_max:
        last = k == len(pts) - 1
        v, w = command((x, y, th), pts[k], kw, last)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th = wrap(th + w * dt)
        t += dt
        trace.append((t, x, y, th))
        cte = min(seg_dist((x, y), ref[i], ref[i + 1]) for i in range(len(ref) - 1))
        max_cte = max(max_cte, cte)
        sum_cte += cte
        if math.hypot(pts[k][0] - x, pts[k][1] - y) < (tol_last if last else tol):
            k += 1
    return {
        "kw": kw,
        "reached": k >= len(pts),
        "time_s": round(t, 2),
        "max_cte_m": round(max_cte, 4),
        "mean_cte_m": round(sum_cte / len(trace), 4),
    }, trace
