"""Post-planning verification: collision sweep, paint deposition, film thickness / coverage, joint speeds."""
import numpy as np
from scipy.spatial import cKDTree

from . import paths
from .cell import VEL_LIMITS, ARM_JOINTS
from .collision import CollisionChecker, SAFETY_MARGIN


def collision_sweep(cell, prog, log=print):
    """Signed clearance and contact count for every program sample."""
    cc = CollisionChecker(cell)
    n = len(prog["t"])
    dmin = np.zeros(n)
    ncon = np.zeros(n, int)
    pair = []
    for i in range(n):
        cell.set(prog["q"][i])
        d, a, b = cc.clearance()
        dmin[i] = d
        ncon[i] = len(cc.contacts())
        pair.append((cell.m.geom(a).name, cell.m.geom(b).name))
        if i % 2000 == 0:
            log(f"   collision sweep {i}/{n}")
    return dmin, ncon, pair


def deposit(cell, prog):
    """Ray-cast the spray axis from the ACTUAL TCP onto the car for every
    spraying sample.  Returns array [t, x, y, z, nx, ny, nz, standoff, on_panel].  When the gun is aimed just past an edge (overspray) or at glass,
    the spray centre is placed at the nominal standoff (on_panel = 0): its fan still
    reaches the panel edge."""
    from .paths import cast, box_normal, STANDOFF
    out = []
    for i in np.where(prog["spray"])[0]:
        cell.set(prog["q"][i])
        p, a = cell.tcp_pose()
        gid, hit = cast(cell, p, a)
        if gid in cell.paint_geoms and np.linalg.norm(hit - p) < 2 * STANDOFF:
            n = box_normal(cell.m, cell.d, gid, hit)
            out.append([prog["t"][i], *hit, *n, np.linalg.norm(hit - p), 1])
        else:
            c = p + STANDOFF * a
            out.append([prog["t"][i], *c, *(-a), STANDOFF, 0])
    return np.array(out)


def film_thickness(cell, dots, sigma=paths.FAN_WIDTH / 4):
    """Relative dry-film thickness on every surface.

    Spray model: each spray sample deposits a Gaussian footprint
    (sigma = fan width / 4, i.e. the 300 mm fan holds ~95 % of the paint).
    Thickness at a surface point = sum of footprints of all spray samples on the
    same face.  It is normalised by the ideal value for an infinite flat panel
    painted with the same raster (spacing STEP along the stroke, ROW_PITCH between
    strokes), so 1.0 = nominal film build.

    Returns {surface: dict(points, rel, coverage, mean, cv, within25)}.
    """
    ideal = 2 * np.pi * sigma ** 2 / (paths.STEP * paths.ROW_PITCH)
    cell.set(cell.home())
    out = {}
    for patch in paths.PATCHES:
        pts = paths.surface_samples(cell, patch)
        want = np.array(patch["normal"], float)
        sel = dots[(dots[:, 4:7] @ want) > 0.99][:, 1:4]
        tree = cKDTree(sel)
        rel = np.zeros(len(pts))
        for i, nb in enumerate(tree.query_ball_point(pts[:, :3], 4 * sigma)):
            if nb:
                r2 = np.sum((sel[nb] - pts[i, :3]) ** 2, axis=1)
                rel[i] = np.exp(-r2 / (2 * sigma ** 2)).sum() / ideal
        out[patch["name"]] = dict(
            points=pts, rel=rel,
            coverage=float(np.mean(rel >= 0.5)),          # at least half nominal film
            mean=float(rel.mean()),
            cv=float(rel.std() / rel.mean()),
            within25=float(np.mean(np.abs(rel - 1) <= 0.25)))
    return out


def joint_speed_ratio(prog):
    """Peak |dq/dt| / limit per joint ."""
    t, q = prog["t"], prog["q"]
    dt = np.diff(t)
    ok = dt > 1e-6
    v = np.abs(np.diff(q, axis=0)[ok] / dt[ok, None])
    return dict(zip(ARM_JOINTS, (v.max(axis=0) / VEL_LIMITS).round(3).tolist()))
