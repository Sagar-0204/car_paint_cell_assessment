"""Step 2: play the painting program in the MuJoCo viewer.

    python view_simulation.py              (run plan_program.py first)
    python view_simulation.py --speed 1    real time (default is 2x)

The simulation starts as soon as the window opens and runs once to the end.
Camera: left-drag rotate, right-drag pan, scroll zoom.
To run it again, close the window and run the command again.
"""
import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from paintcell.cell import Cell
from paintcell.collision import CollisionChecker
from paintcell.render import PaintLayer, add_spray, highlight_collision

OUT = Path(__file__).resolve().parent / "output"
PRETTY = {"roof": "roof", "hood": "hood", "trunk": "trunk lid", "front": "front face",
          "rear": "rear face", "right_side": "right side", "left_side": "left side"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=2.0, help="playback speed (1 = real time)")
    speed = ap.parse_args().speed

    f = OUT / "program.npz"
    if not f.exists():
        raise SystemExit("output/program.npz not found - run  python plan_program.py  first.")
    z = np.load(f)
    T, Q, spray, clearance = z["t"], z["q"], z["spray"], z["clearance"]
    labels = [str(s) for s in z["labels"]]
    label_idx = z["label_idx"]
    spray_cum = np.cumsum(spray)
    duration = float(T[-1])

    cell = Cell()
    cc = CollisionChecker(cell)
    paint = PaintLayer(z["dots"])
    m, d = cell.m, cell.d
    base = (m.geom_rgba.copy(), m.geom_matid.copy())

    with mujoco.viewer.launch_passive(m, d) as v:
        # starting view; after this the camera is free (mouse controlled)
        v.cam.lookat[:] = [0, 0, 1.2]
        v.cam.azimuth, v.cam.elevation, v.cam.distance = 60, -28, 11
        start = time.time()
        while v.is_running():
            t = min((time.time() - start) * speed, duration)
            i = int(np.clip(np.searchsorted(T, t) - 1, 0, len(T) - 1))
            with v.lock():
                # robot pose at time t (interpolated between planned samples)
                cell.set(np.array([np.interp(t, T, Q[:, j]) for j in range(Q.shape[1])]))
                highlight_collision(m, cc, base)          # links turn red within 30 mm
                v.user_scn.ngeom = 0
                paint.add_to_scene(v.user_scn, t)         # paint deposited so far
                if spray[i]:
                    p, axis = cell.tcp_pose()
                    add_spray(v.user_scn, p, axis)        # spray plume while the gun is on
            op = labels[label_idx[i]]
            for k, w in PRETTY.items():
                op = op.replace(k, w)
            info = "\n".join([
                "Robotic car painting cell",
                f"Operation : {op}",
                f"Spray gun : {'ON' if spray[i] else 'off'}",
                f"Painted   : {spray_cum[i] / spray_cum[-1] * 100:5.1f} % of program",
                f"Clearance : {clearance[i] * 1000:5.0f} mm to car / booth / itself",
                f"Time      : {t:6.1f} s / {duration:.0f} s"])
            v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                         info, None))
            v.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
