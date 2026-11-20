"""Lift-and-turn sweep, IK only (no simulator), for the turn about the axis through both hands.

    python sweep2.py X0S DIRS ZHIS BETAS [JAWS] [GXS]      e.g.  python sweep2.py 0.06,0.10 -1,1 0.42,0.48 0,10 1,-1 0

Both hands pinch the seat plate at its side edges (jaw closing across the plate's thickness, fingers pointing in, tilted
BETA deg forward). The turn is about the world y axis through both grips, so each hand only translates and rolls about
its own approach axis: the wrist's last joint (+-120 deg) takes the 180 deg turn if the jaw starts on the right side.
The chair starts upright on the floor facing -x (front toward the arms), turns seat down through 180 deg, and lands with
its seat on the tabletop and its backrest hanging over the table's near edge, as classroom chairs are put up.
Each arm is solved alone along the whole path from several seeds; every chair corner is checked against floor and table.
"""
from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena import ik, station  # noqa: E402

TOP = station.TABLE_TOP
# Flash Furniture Whitney YU-YCX-003 (retailer listings): seat height 10.75 in, seat 10 x 10 in, back 11 in wide, 20 in high,
# footprint 12.5 x 14 in. Plate thickness 12 mm is assumed. Community Playthings Woodcrest table on 12 in legs.
SZ, ST, HW, HD, BW, BT = 0.273, 0.012, 0.127, 0.127, 0.140, 0.508
FX, FY = 0.178, 0.159          # feet half-footprint (14 in deep, 12.5 in wide)
TABLE_H = 0.305
IN = 0.018                     # the TCP sits this far in from the plate's side edge
GAP = 0.012                    # backrest's outer face to the table's near edge, landed


def Ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def chair_points(gx):
    """Corner points of the chair (facing -x in the world, upright), relative to the grip axis point A0 = (gx, 0, seat mid)."""
    zs = SZ - ST / 2
    pts = []
    # chair frame front +x; world = Rz(180): (x, y) -> (-x, -y)
    for x in (-HD, HD):
        for y in (-HW, HW):
            for z in (SZ - ST, SZ):
                pts.append((x, y, z))
    for y in (-BW, BW):
        for z in (SZ, BT):
            pts.append((-HD - 0.008, y, z))
    for x in (-FX, FX):
        for y in (-FY, FY):
            pts.append((x, y, 0.0))
    P = np.array(pts, float)
    P[:, 0] *= -1
    P[:, 1] *= -1
    return P - np.array([gx, 0.0, zs])


def plan(x0, d, zr, zhi, xt, gx=0.0, n_turn=24):
    """Grip-axis waypoints (A, theta) and the table's near edge. Lift straight up to zr, turn through 180 deg while the axis
    goes to (xt, zhi), slide over the table at zhi, lower onto it. d = +1 turns the chair's top away from the arms, -1 toward them."""
    zs = SZ - ST / 2
    a0 = np.array([x0 + gx, 0.0, zs])
    xe = x0 + FX + 0.02                # table near edge: clear of the chair's rear feet at the start
    # landed (theta 180): seat top on the tabletop, backrest's outer face GAP outside the near edge
    ax_land = xe - GAP + (HD + 0.008 - gx)
    a1 = np.array([a0[0], 0.0, zr])
    a2 = np.array([min(xt, ax_land), 0.0, zhi])
    a3 = np.array([ax_land, 0.0, zhi])
    a4 = np.array([ax_land, 0.0, TABLE_H + ST / 2])
    wps = [(a0, 0.0)]
    wps += [(a0 + (a1 - a0) * k / 6, 0.0) for k in range(1, 7)]
    for k in range(1, n_turn + 1):
        f = k / n_turn
        s = f * f * (3 - 2 * f)
        wps.append((a1 + (a2 - a1) * s, d * math.pi * s))
    wps += [(a2 + (a3 - a2) * k / 8, d * math.pi) for k in range(1, 9)]
    wps += [(a3 + (a4 - a3) * k / 8, d * math.pi) for k in range(1, 9)]
    return wps, xe


def clearance(wps, xe, gx):
    P0 = chair_points(gx)
    worst_floor, worst_table = 9.0, 9.0
    for A, th in wps[1:-1]:
        P = (Ry(th) @ P0.T).T + A
        worst_floor = min(worst_floor, float(P[:, 2].min()))
        over = P[P[:, 0] > xe + 0.003]
        if len(over):
            worst_table = min(worst_table, float(over[:, 2].min() - TABLE_H))
    return worst_floor, worst_table


def hand_rot(sgn, beta, jaw):
    a = np.array([math.sin(beta), -sgn * math.cos(beta), 0.0])
    j = np.array([0.0, 0.0, jaw])
    return np.column_stack([j, np.cross(a, j), a])


SEEDS = [[j1, 1.0, 1.0, -0.6, j5, j6] for j1 in (-0.5, 0.0, 0.5) for j5 in (-1.2, 0.0, 1.2) for j6 in (-1.7, 0.0, 1.7)]


def solve_arm(side, sgn, wps, beta, jaw):
    base = np.asarray(station.BASES[side], float)
    G = hand_rot(sgn, beta, jaw)
    off = np.array([0.0, sgn * (HW - IN), 0.0])
    p0 = wps[0][0] + off + [0, 0, TOP]
    starts = []
    for s in SEEDS:
        q, pe, re = ik.solve(p0, ik.matrix_to_quat_xyzw(G), np.array(s), base_pos=base, iters=80)
        if pe < 0.002 and re < 0.02 and not any(np.abs(q - o).max() < 0.2 for o in starts):
            starts.append(q)
    best = None
    for q0 in starts:
        q = q0
        worst, wdeg, step, rows = 0.0, 0.0, 0.0, []
        for A, th in wps:
            R = Ry(th)
            qq, pe, re = ik.solve(A + R @ off + [0, 0, TOP], ik.matrix_to_quat_xyzw(R @ G), q, base_pos=base, iters=50)
            st = float(np.degrees(np.abs(qq - q)).max())
            step, q = max(step, st), qq
            worst, wdeg = max(worst, pe * 1000), max(wdeg, math.degrees(re))
            rows.append(np.degrees(qq))
        if best is None or (worst + wdeg) < (best[0] + best[1]):
            best = (worst, wdeg, step, rows)
    return best


def main():
    """python sweep2.py X0S DIRS ZRS ZHIS XTS [BETAS] [JAWS]   (comma lists); prints every layout whose chair clears floor and table."""
    L = lambda i, dflt: [float(v) for v in sys.argv[i].split(",")] if len(sys.argv) > i else dflt
    x0s, dirs, zrs, zhis, xts = L(1, [0.1]), [int(v) for v in L(2, [1])], L(3, [0.45]), L(4, [0.45]), L(5, [0.0])
    betas, jaws = [math.radians(b) for b in L(6, [0.0])], L(7, [1.0, -1.0])
    print("TOP", round(TOP, 4), "bases", station.BASES, flush=True)
    for x0 in x0s:
        for d in dirs:
            for zr in zrs:
                for zhi in zhis:
                    for xt in xts:
                        wps, xe = plan(x0, d, zr, zhi, xt)
                        fl, tb = clearance(wps, xe, 0.0)
                        if fl < 0.01 or tb < 0.005:
                            continue
                        for beta in betas:
                            for jaw in jaws:
                                Lr = solve_arm("left", 1.0, wps, beta, jaw)
                                Rr = solve_arm("right", -1.0, wps, beta, -jaw)
                                tag = (f"x0 {x0:.2f} dir {d:+d} zr {zr:.2f} zhi {zhi:.2f} xt {xt:.2f} beta {math.degrees(beta):+3.0f} jaw {jaw:+.0f}"
                                       f" edge {xe:.3f} land_ax {wps[-1][0][0]:.3f} floor {fl*1000:+.0f} table {tb*1000:+.0f} mm")
                                if Lr is None or Rr is None:
                                    print(tag, "| no grasp solution", flush=True)
                                    continue
                                print(f"{tag} | L {Lr[0]:5.1f} mm {Lr[1]:4.1f} deg step {Lr[2]:5.1f} | R {Rr[0]:5.1f} mm {Rr[1]:4.1f} deg step {Rr[2]:5.1f}"
                                      f" | WORST {max(Lr[0], Rr[0]):5.1f} mm", flush=True)


if __name__ == "__main__":
    main()
