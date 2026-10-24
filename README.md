# Robotic Car Painting Cell — MuJoCo Simulation

- **Name:** Vidya Sagar
- **Software used:** MuJoCo 3.x (simulation and visualisation) with Python 3 (NumPy, SciPy)
- **Robot selected:** 6-axis industrial robot with ABB IRB 6700-150/3.20-class kinematics, mounted inverted on a 2-axis overhead gantry (X travel 6.4 m + Z lift 1.4 m) → 8 controlled axes

Practical assessment for the **Yuktii AI Labs — Robotics Simulation Intern** position.

## What the simulation shows

The car body (4800 × 1900 × 1600 mm) is already parked in an 8 × 5 m paint booth. The robot:

1. starts folded in its park posture under the gantry beam,
2. paints **7 surfaces** — roof, hood, front, right side, rear, left side and trunk lid — with zig-zag spray strokes,
3. checks every motion for collisions and automatically re-plans any move that would hit the car, the booth or itself,
4. returns to park.

| Assessment requirement | Where it is |
|---|---|
| Car body | `models/paint_cell.xml` — body, cabin, bumpers, glass, wheels |
| Industrial robot | IRB 6700-class 6-axis arm on an overhead gantry |
| Painting end-effector | rotary-bell spray gun, tool point 350 mm from the flange |
| Cell layout | 8 × 5 m booth with transparent walls and a gantry frame |
| Robot movement around the vehicle | gantry carries the arm along and around the car |
| Painting trajectory | `paintcell/paths.py` — ray-cast raster strokes, 250 mm standoff |
| Collision checking | `paintcell/collision.py`, `paintcell/rrt.py` — 30 mm safety margin, automatic re-planning; live clearance shown in the viewer |
| Painting different surfaces | all 7 exterior surfaces, paint shown building up on the car |

## Results

| Metric | Value |
|---|---|
| Surfaces painted | 7 |
| Spray path | 126 m in 52 strokes |
| Paint coverage (simulated) | **97.2 %** (95.4–99.1 % per surface) |
| Gun standoff while spraying | 250.1 ± 2.3 mm (target 250 mm) |
| Path accuracy (IK error) | ≤ 0.5 mm |
| Collisions in the final program | **0** of 7,385 samples |
| Minimum clearance | 37.2 mm |
| Colliding moves rejected and re-planned | 5 |
| Cycle time | 411 s (316 s spraying) |

All numbers are written to `output/results.json` by `plan_program.py`.

## How to run

```bash
pip install -r requirements.txt
python view_simulation.py    # watch the simulation (a planned program is already in output/)
python plan_program.py       # optional: re-plan and re-verify from scratch (~1 min)
```

To see just the cell: `python -m mujoco.viewer --mjcf models/paint_cell.xml`

The simulation starts as soon as the viewer opens and runs once to the end (2× speed by default; `--speed 1` for real time). The camera is free: left-drag to rotate, right-drag to pan, scroll to zoom. To run it again, close the window and run the command again.

## How it works

1. **Path planning** (`paths.py`). Parallel rays are cast at each surface (`mj_ray`). Every hit gives a surface point and its normal. The gun target is the point + 250 mm along the normal, pointing back at the panel. Points are ordered into zig-zag rows 150 mm apart (50 % overlap with a 300 mm spray fan), with 100 mm of overspray past each edge. The gun switches off over glass.
2. **Inverse kinematics** (`ik.py`). Damped least squares on the MuJoCo Jacobian. A spray bell is round, so the task needs 5 DOF. The 3 spare DOF are used in the null space: the gantry follows the stroke and the arm keeps a constant shape.
3. **Program building** (`planner.py`). For each surface, many IK start postures are tried and each is tracked over the whole surface. The planner keeps the one that reaches every point within 2 mm with the most clearance. Strokes run at a constant 0.4 m/s.
4. **Collision checking** (`collision.py`, `rrt.py`). MuJoCo contact detection with a 30 mm margin runs on every planned pose. A move between surfaces is tried first as a direct joint move; if it collides, the planner tries a retract, a detour via the park posture, and finally RRT-Connect.
5. **Verification** (`analysis.py`). Every sample of the final program is checked again with exact distances. Paint is ray-cast from the actual gun pose, and film thickness is computed with a Gaussian spray model to get coverage.

## Files

```
models/paint_cell.xml     booth, gantry, robot, spray gun, car
paintcell/cell.py         model loading, joint bookkeeping, park posture
paintcell/paths.py        surface scanning and raster strokes
paintcell/ik.py           damped-least-squares IK
paintcell/collision.py    contact check + exact clearance
paintcell/rrt.py          RRT-Connect planner
paintcell/planner.py      builds the time-stamped program
paintcell/analysis.py     collision sweep, paint deposition, coverage
paintcell/render.py       paint, spray and red collision highlighting in the viewer
plan_program.py           step 1: plan and verify
view_simulation.py        step 2: live viewer
```

## Limitations

* Kinematic simulation: joint positions are set from the plan; motor dynamics are not simulated.
* The car is built from simple boxes, so the panels are flat.
* The paint model is geometric, not calibrated against real spray data.
