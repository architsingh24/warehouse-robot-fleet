"""Run all experiments, write CSV files and plots to ./results."""
import csv
import os
import random
import statistics as st
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from warehouse import build_grid, make_tasks, pickup_cells, DROP_STATIONS, HOMES
from planner import astar
from fleet import MODES
from controller import follow

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
os.makedirs(OUT, exist_ok=True)
GRID = build_grid()


def write_csv(name, rows):
    with open(os.path.join(OUT, name), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


# -------------------------------------------------- E1: planner benchmark
def exp_planner(n_pairs=300, seed=1):
    rng = random.Random(seed)
    free = [(r, c) for r in range(len(GRID)) for c in range(len(GRID[0])) if GRID[r][c] == 0]
    pairs = [(rng.choice(free), rng.choice(free)) for _ in range(n_pairs)]
    pairs = [(a, b) for a, b in pairs if a != b]
    rows, summary = [], []
    for h in ("manhattan", "euclidean", "dijkstra"):
        nodes, times, lens = [], [], []
        for a, b in pairs:
            t0 = time.perf_counter()
            path, n = astar(GRID, a, b, h)
            dt = time.perf_counter() - t0
            nodes.append(n)
            times.append(dt * 1000)
            lens.append(len(path) - 1)
            rows.append({"heuristic": h, "start": a, "goal": b, "nodes": n,
                         "time_ms": round(dt * 1000, 4), "path_len": len(path) - 1})
        summary.append({"method": "Dijkstra" if h == "dijkstra" else f"A* ({h})",
                        "heuristic": h,
                        "mean_nodes": round(st.mean(nodes), 1),
                        "mean_time_ms": round(st.mean(times), 3),
                        "mean_path_len": round(st.mean(lens), 2),
                        "pairs": len(pairs)})
    write_csv("planner_raw.csv", rows)
    write_csv("planner_summary.csv", summary)
    return summary


# ------------------------------------------------------ E2: fleet sweep
def exp_fleet(robot_counts=(1, 3, 5, 8), n_tasks=3, seeds=range(20)):
    rows = []
    for n in robot_counts:
        for seed in seeds:
            tasks = make_tasks(GRID, n, n_tasks, seed)
            for mode, fn in MODES.items():
                m, _ = fn(GRID, tasks)
                m.update({"robots": n, "seed": seed, "mode": mode})
                rows.append(m)
    write_csv("fleet_raw.csv", rows)
    summary = []
    for n in robot_counts:
        for mode in MODES:
            sel = [r for r in rows if r["robots"] == n and r["mode"] == mode]
            summary.append({
                "robots": n, "mode": mode, "runs": len(sel),
                "mean_collisions": round(st.mean(r["collisions"] for r in sel), 2),
                "deadlock_rate": round(st.mean(r["deadlock"] for r in sel), 2),
                "failed_plans": sum(r["failed_plans"] for r in sel),
                "mean_tasks_done": round(st.mean(r["tasks_done"] for r in sel), 2),
                "tasks_total": sel[0]["tasks_total"],
                "mean_makespan_s": round(st.mean(r["makespan_s"] for r in sel), 1),
                "mean_tasks_per_min": round(st.mean(r["tasks_per_min"] for r in sel), 3),
                "mean_waits": round(st.mean(r["waits"] for r in sel), 1),
                "mean_plan_time_s": round(st.mean(r["plan_time_s"] for r in sel), 4),
            })
    write_csv("fleet_summary.csv", summary)
    return summary


# ------------------------------------------------ E3: controller gains
def exp_controller():
    path, _ = astar(GRID, HOMES[0], (1, 24), "manhattan")
    rows, traces = [], {}
    for kw in (1.5, 4.0):
        m, tr = follow(path, kw=kw)
        rows.append(m)
        traces[kw] = tr
    write_csv("controller_summary.csv", rows)
    return rows, traces, path


# ----------------------------------------------------------------- plots
def plots(planner, fleet, ctrl_traces, ctrl_path):
    modes = list(MODES)
    colors = {"blind": "#c0504d", "reactive": "#e0a030", "reactive_replan": "#4a7fb5", "reserved": "#2a7f62"}
    robots = sorted({r["robots"] for r in fleet})

    fig, axs = plt.subplots(1, 4, figsize=(17, 3.8))
    for ax, key, title in zip(
            axs, ["mean_collisions", "deadlock_rate", "mean_makespan_s", "mean_tasks_per_min"],
            ["Collision events per run", "Deadlock rate", "Makespan (s)", "Tasks per minute (completed)"]):
        for m in modes:
            ys = [next(r[key] for r in fleet if r["robots"] == n and r["mode"] == m) for n in robots]
            ax.plot(robots, ys, marker="o", label=m, color=colors[m])
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("Number of robots")
        ax.set_xticks(robots)
        ax.grid(alpha=0.3)
    axs[0].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_fleet_comparison.png"), dpi=160)
    plt.close(fig)

    fig, axs = plt.subplots(1, 3, figsize=(11, 3.4))
    labels = [r["method"] for r in planner]
    for ax, key, title in zip(axs, ["mean_nodes", "mean_time_ms", "mean_path_len"],
                              ["Mean nodes expanded", "Mean planning time (ms)", "Mean path length (cells)"]):
        ax.bar(range(len(planner)), [r[key] for r in planner], color=["#2a7f62", "#4a7fb5", "#999999"])
        ax.set_xticks(range(len(planner)))
        ax.set_xticklabels(labels, rotation=15, fontsize=8)
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_planner_benchmark.png"), dpi=160)
    plt.close(fig)

    from warehouse import cell_to_world, H, W, CELL
    fig, ax = plt.subplots(figsize=(8, 5.4))
    for r in range(len(GRID)):
        for c in range(len(GRID[0])):
            if GRID[r][c]:
                x, y = cell_to_world((r, c))
                ax.add_patch(plt.Rectangle((x - CELL / 2, y - CELL / 2), CELL, CELL, color="#bbbbbb"))
    ref = [cell_to_world(c) for c in ctrl_path]
    ax.plot(*zip(*ref), "k--", lw=1, label="A* path")
    for kw, col in ((1.5, "#4a7fb5"), (4.0, "#c0504d")):
        tr = ctrl_traces[kw]
        ax.plot([p[1] for p in tr], [p[2] for p in tr], color=col, lw=1.4, label=f"Kw = {kw}")
    ax.set_xlim(0, W * CELL)
    ax.set_ylim(0, H * CELL)
    ax.set_aspect("equal")
    ax.set_title("Controller tracking of an A* path", fontsize=10)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_controller_tracking.png"), dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    p = exp_planner()
    print("PLANNER"); [print(r) for r in p]
    f = exp_fleet()
    print("FLEET"); [print(r) for r in f]
    c, traces, path = exp_controller()
    print("CONTROLLER"); [print(r) for r in c]
    plots(p, f, traces, path)
    print("done ->", OUT)
