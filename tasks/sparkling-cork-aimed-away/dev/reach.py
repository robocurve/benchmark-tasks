"""Reach and hold check (no simulation): the left hand holds the 375 mL bottle by its body, lifts it off the coaster and
tilts it 45 degrees toward a heading; the right hand holds the stopper from above along the bottle's axis, pulls it
32 mm out and carries it to the dish. Usage: python dev/reach.py"""
import math

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY

TOP = 0.7475
C = np.array([0.34, 0.02])  # coaster
GRIP_Z = 0.004 + 0.090  # grip height above the table (bottle base on the 4 mm coaster floor, grip 90 mm up)
LIFT = 0.035
H = 0.2435
DISH = np.array([0.20, -0.30])


def frame(approach, close):
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * float(np.dot(close, z))
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def rot(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


SEEDS = (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0], [0.5, 1.2, 1.0, -0.5, 0.0, 1.5], [-0.5, 1.2, 1.0, -0.5, 0.0, -1.5],
         [0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [-0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [0.6, 1.8, 1.6, -0.2, 0.0, 0.0],
         [-0.6, 1.8, 1.6, -0.2, 0.0, 0.0], [0.0, 1.2, 0.8, 0.6, 0.0, 0.0], [0.0, 0.9, 1.4, -1.2, 0.0, 0.0])


def err(arm, p, r):
    best = (1e9, None)
    for q0 in SEEDS:
        q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(r), np.array(q0, float),
                             base_pos=station.BASES[arm], iters=150)
        e = pe * 1000 + math.degrees(re)
        if e < best[0]:
            best = (e, q)
    return best


def main():
    print("bases", station.BASES)
    for h_deg in (-80.0, -65.0, -95.0):
        h = math.radians(h_deg)
        u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])  # tilt axis: z goes toward h
        for a_deg in (-30.0, -10.0, 10.0, -50.0):
            a = math.radians(a_deg)
            d = np.array([math.cos(a), math.sin(a), 0.0])
            r0 = frame(d, (math.sin(a), -math.cos(a), 0.0))
            g0 = np.array([C[0], C[1], TOP + GRIP_Z]) - 0.004 * d
            rows = [("left", "grip", g0, r0), ("left", "lifted", g0 + [0, 0, LIFT], r0)]
            for t in (15, 30, 45, 55):
                rows.append(("left", f"tilt {t}", g0 + [0, 0, LIFT], rot(u, math.radians(t)) @ r0))
            bad = []
            worst = 0.0
            for arm, lab, p, r in rows:
                e, q = err(arm, p, r)
                worst = max(worst, e)
                if e > 3:
                    bad.append(f"{lab}:{e:.0f}")
            # the right hand on the stopper at 45 degrees
            ax = rot(u, math.radians(45)) @ np.array([0, 0, 1.0])
            grip = np.array([C[0], C[1], TOP + GRIP_Z + LIFT])
            lip = grip + (H - 0.090) * ax
            rr = []
            for roll in (0, 45, 90, 135, -45, -90):
                c = rot(ax, math.radians(roll)) @ u
                r1 = frame(-ax, c)
                es = []
                for lab, p in (("above", lip + 0.08 * ax), ("on knob", lip + 0.018 * ax), ("pulled", lip + 0.050 * ax),
                               ("clear", lip + 0.09 * ax)):
                    es.append(err("right", p, r1)[0])
                rr.append(f"roll{roll}:{max(es):.0f}")
            print(f"h {h_deg:5.0f} approach {a_deg:5.0f}: left worst {worst:6.1f} bad {bad} | lip "
                  f"({lip[0]:.3f},{lip[1]:.3f},{lip[2] - TOP:.3f}) right {' '.join(rr)}")
    rdish = frame((0, 0, -1.0), (1.0, 0, 0))
    for z in (0.10, 0.04):
        print("right over dish z", z, round(err("right", (DISH[0], DISH[1], TOP + z), rdish)[0], 1))
    # setting the bottle back down upright
    for a_deg in (-30.0, -10.0):
        a = math.radians(a_deg)
        d = np.array([math.cos(a), math.sin(a), 0.0])
        r0 = frame(d, (math.sin(a), -math.cos(a), 0.0))
        g0 = np.array([C[0], C[1], TOP + GRIP_Z]) - 0.004 * d
        print("left back off", a_deg, round(err("left", g0 - 0.12 * d, r0)[0], 1))


if __name__ == "__main__":
    main()
