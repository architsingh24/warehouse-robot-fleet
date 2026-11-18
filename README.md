# Warehouse Mobile Robot Fleet with A* Path Planning (CSAI4014 AISR)

## Files
- warehouse.py   grid map (20x30 cells, 0.5 m), task generator, cell->world mapping
- planner.py     A*, Dijkstra, heuristics, space-time A* + reservation table
- fleet.py       fleet modes: blind, reactive, reactive_replan, reserved
- controller.py  P3-DX differential-drive heading controller + kinematic simulation
- experiments.py runs all experiments, writes results/*.csv and results/*.png
- coppelia_runner.py replays the reserved plan in CoppeliaSim (ZeroMQ remote API)

## Run
    python experiments.py                       # needs matplotlib; ~8 s
    pip install coppeliasim-zmqremoteapi-client
    # start CoppeliaSim, then:
    python coppelia_runner.py --robots 3 --seed 0 --model "<path>/pioneer p3dx.ttm"

coppelia_runner.py was NOT executed where it was written (no CoppeliaSim). Test it first.
