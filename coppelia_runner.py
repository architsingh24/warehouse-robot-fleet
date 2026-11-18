"""Replay the reserved (space-time A*) plan on Pioneer P3-DX robots in CoppeliaSim.

Prerequisites (on the team's machine, not in this chat sandbox):
  pip install coppeliasim-zmqremoteapi-client
  CoppeliaSim 4.5+ running (ZeroMQ remote API add-on is enabled by default).

Usage:
  python coppelia_runner.py --robots 3 --seed 0 --model "<path to pioneer p3dx.ttm>"

NOTE: this file could not be executed in the environment where it was written
(no CoppeliaSim installed). Object names below match the standard P3-DX model
('leftMotor', 'rightMotor'); verify them in the scene hierarchy on your machine.
"""
import argparse
import csv
import math
import time

from warehouse import build_grid, make_tasks, cell_to_world, HOMES, H, W, CELL
from fleet import run_reserved
from controller import command, wheel_speeds, wrap, simplify

TICK = 1.5        # seconds of simulated time per plan step (slack for turning)
DT = 0.05         # CoppeliaSim step


def build_scene(sim, grid, model_path, n_robots):
    """Create shelves as static cuboids and load n P3-DX robots at their homes."""
    for r in range(H):
        for c in range(W):
            if grid[r][c]:
                x, y = cell_to_world((r, c))
                h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [CELL, CELL, 0.8], 0)
                sim.setObjectPosition(h, sim.handle_world, [x, y, 0.4])
                sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
                sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
    robots = []
    for i in range(n_robots):
        base = sim.loadModel(model_path)
        x, y = cell_to_world(HOMES[i])
        sim.setObjectPosition(base, sim.handle_world, [x, y, 0.14])
        left = sim.getObject("./leftMotor", {"index": 0, "proxy": base})
        right = sim.getObject("./rightMotor", {"index": 0, "proxy": base})
        robots.append((base, left, right))
    return robots


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=3)
    ap.add_argument("--tasks", type=int, default=2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--kw", type=float, default=2.0)
    ap.add_argument("--model", default="models/robots/mobile/pioneer p3dx.ttm")
    ap.add_argument("--log", default="coppelia_log.csv")
    args = ap.parse_args()

    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient()
    sim = client.require("sim")
    client.setStepping(True)

    grid = build_grid()
    tasks = make_tasks(grid, args.robots, args.tasks, args.seed)
    metrics, paths = run_reserved(grid, tasks)
    print("plan:", metrics)

    sim.stopSimulation()
    time.sleep(0.5)
    robots = build_scene(sim, grid, args.model, args.robots)
    sim.startSimulation()

    log = []
    t = 0.0
    T_end = (max(len(p) for p in paths)) * TICK + 5
    while t < T_end:
        step = int(t / TICK) + 1          # time-indexed target keeps the schedule
        for i, (base, left, right) in enumerate(robots):
            path = paths[i]
            tgt_cell = path[min(step, len(path) - 1)]
            tx, ty = cell_to_world(tgt_cell)
            x, y, _ = sim.getObjectPosition(base, sim.handle_world)
            th = sim.getObjectOrientation(base, sim.handle_world)[2]
            last = step >= len(path) - 1
            v, w = command((x, y, th), (tx, ty), args.kw, last)
            if math.hypot(tx - x, ty - y) < 0.04:
                v, w = 0.0, 0.0
            wl, wr = wheel_speeds(v, w)
            sim.setJointTargetVelocity(left, wl)
            sim.setJointTargetVelocity(right, wr)
            log.append((round(t, 2), i, round(x, 3), round(y, 3), round(th, 3)))
        client.step()
        t += DT
    sim.stopSimulation()

    with open(args.log, "w", newline="") as f:
        csv.writer(f).writerows([("t", "robot", "x", "y", "theta")] + log)
    print("log written to", args.log)


if __name__ == "__main__":
    main()
