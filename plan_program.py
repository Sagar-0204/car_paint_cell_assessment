"""Step 1: plan the painting program and verify it.

    python plan_program.py        (takes about 1 minute)

Saves to ./output:
    program.npz    time-stamped joint trajectory + paint data (read by view_simulation.py)
    results.json   coverage, collisions, clearance, cycle time ...
"""
import json
import time
from pathlib import Path

import numpy as np

from paintcell import analysis, paths
from paintcell.cell import Cell
from paintcell.collision import CONTACT_MARGIN
from paintcell.planner import Planner, PAINT_SPEED, KINDS

OUT = Path(__file__).resolve().parent / "output"


def main():
    OUT.mkdir(exist_ok=True)
    t0 = time.time()
    cell = Cell()

    print("== 1. Planning painting program ==")
    planner = Planner(cell)
    prog = planner.build()
    print(f"   {len(prog['t'])} samples, cycle time {prog['t'][-1]:.1f} s")

    print("== 2. Collision check of every sample ==")
    dmin, ncon, pairs = analysis.collision_sweep(cell, prog)
    i_min = int(np.argmin(dmin))
    print(f"   samples in contact: {int((ncon > 0).sum())},  minimum clearance "
          f"{dmin[i_min]*1000:.1f} mm ({pairs[i_min][0]} <-> {pairs[i_min][1]})")

    print("== 3. Paint deposition and coverage ==")
    dots = analysis.deposit(cell, prog)
    film = analysis.film_thickness(cell, dots)
    for k, v in film.items():
        print(f"   {k:11s} coverage {v['coverage']*100:5.1f} %")

    t = prog["t"]
    dt = np.diff(t, append=t[-1])
    kind = prog["kind"]
    on = dots[:, 8] > 0
    total_cov = np.average([v["coverage"] for v in film.values()],
                           weights=[len(v["points"]) for v in film.values()])
    results = dict(
        cycle_time_s=round(float(t[-1]), 1),
        painting_time_s=round(float(dt[kind == KINDS.index("paint")].sum()), 1),
        paint_speed_m_s=PAINT_SPEED,
        standoff_m=paths.STANDOFF, fan_width_m=paths.FAN_WIDTH, row_pitch_m=paths.ROW_PITCH,
        surfaces=planner.stats,
        coverage_pct={k: round(v["coverage"] * 100, 1) for k, v in film.items()},
        total_coverage_pct=round(float(total_cov) * 100, 1),
        samples=int(len(t)),
        samples_in_contact=int((ncon > 0).sum()),
        contact_margin_mm=CONTACT_MARGIN * 1000,
        min_clearance_mm=round(float(dmin[i_min]) * 1000, 1),
        min_clearance_pair=list(pairs[i_min]),
        rejected_motions=planner.events,
        mean_standoff_mm=round(float(dots[on, 7].mean()) * 1000, 1),
        standoff_std_mm=round(float(dots[on, 7].std()) * 1000, 2),
        peak_joint_speed_ratio=analysis.joint_speed_ratio(prog),
    )
    np.savez_compressed(OUT / "program.npz", **prog, clearance=dmin, dots=dots)
    (OUT / "results.json").write_text(json.dumps(results, indent=2))

    print(f"== Done in {time.time() - t0:.0f} s. Results in {OUT}")
    for k in ["total_coverage_pct", "samples_in_contact", "min_clearance_mm", "cycle_time_s"]:
        print(f"   {k:20s} {results[k]}")
    print("Next: python view_simulation.py")


if __name__ == "__main__":
    main()
