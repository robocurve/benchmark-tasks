"""Scoring of the insole task: a pure function of measured geometry (no simulator imports).

All lengths in metres, in the shoe frame (origin on the table under the inside of the heel counter, +x toward the
toe, +z up); heights of the insole are taken above the shoe's bed at the same x (the bed has a 10 mm heel-to-toe
drop, a toe spring and a heel cup). See README "Scoring" for the real-trial check of every condition.
"""
import numpy as np

UPRIGHT_DEG = 10.0        # shoe sole within 10 deg of flat on the table
SOLE_ON_TABLE = 0.010     # shoe-frame origin within 10 mm of its resting height (sole down on the table)
HANDS_CLEAR = 0.030       # every gripper point >= 3 cm outside the shoe's outline box and from the insole
MARK_FROM_TOE = 0.020     # inside: the pen mark 20 mm from the insole's toe tip is under the laced vamp
HEEL_GAP = 0.006          # home: heel end <= 6 mm from the heel counter (insole 4 mm shorter than the inside) ...
HEEL_DOWN = 0.005         # ... and its edge down on the bed (underside <= 5 mm above it), not up on the counter
TOE_GAP = 0.010           # home: toe end <= 10 mm from the end of the toe box
PRESS_AT = (0.030, 0.100, 0.185)   # flat: heel, arch and ball marks, measured from the insole's heel end
FLAT = 0.003              # flat: the underside at each mark <= 3 mm above the bed (a press moves it <= 3 mm)


def _at_arc(points, arc, s):
    """Point at arc length ``s`` along a polyline given with its cumulative arc lengths."""
    return np.array([np.interp(s, arc, points[:, k]) for k in range(3)])


def measure(line, under, cloud, shoe_pos, shoe_R, fingers, *, li, throat, half_in, bed, top_h, thick, shoe_rest_z,
            shoe_box):
    """line: (N+1, 3) insole centre line heel -> toe at mid-thickness (world); under: (N+1, 3) the same line on the
    underside; cloud: (M, 3) points on its underside (world); shoe_pos/shoe_R: shoe-frame origin and rotation (world);
    fingers: (K, 3) gripper points (world); bed(x), top_h(x): bed height above the table and the upper's top edge
    (collar, then vamp) above the bed; shoe_box: (lo, hi) of the shoe's outline in the shoe frame."""
    R = np.asarray(shoe_R, float)
    loc = lambda p: (np.asarray(p, float) - shoe_pos) @ R
    ln, un, cl, fg = loc(line), loc(under), loc(cloud), loc(fingers)
    lo, hi = np.asarray(shoe_box[0]) - HANDS_CLEAR, np.asarray(shoe_box[1]) + HANDS_CLEAR
    in_box = np.all((fg > lo) & (fg < hi), axis=1)
    d_ins = np.min(np.linalg.norm(fg[:, None, :] - np.vstack([ln, cl])[None], axis=2))
    xc = lambda x: np.clip(x, 0.0, li)
    up = lambda p: p[:, 2] - np.array([bed(x) for x in xc(p[:, 0])])        # height above the bed
    width_ok = np.all(np.abs(cl[:, 1]) <= np.array([half_in(x) for x in xc(cl[:, 0])]) + 0.005)
    above_top = up(cl) + thick - np.array([top_h(x) for x in xc(cl[:, 0])])  # > 0: sticks out above the upper
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(ln, axis=0), axis=1))]
    mark = _at_arc(ln, arc, arc[-1] - MARK_FROM_TOE)
    press = [float(up(_at_arc(un, arc, s)[None])[0]) for s in PRESS_AT]
    heel_pts = cl[np.argsort(cl[:, 0])[:3]]
    tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    return dict(
        tilt_deg=tilt,
        shoe_dz=float(shoe_pos[2] - shoe_rest_z),
        shoe_x=float(shoe_pos[0]), shoe_y=float(shoe_pos[1]),
        hands_in_box=int(in_box.sum()),
        hands_to_insole=float(d_ins),
        x_min=float(cl[:, 0].min()), x_max=float(cl[:, 0].max()),
        above_top=float(above_top.max()),
        up_max=float(up(cl).max()),
        width_ok=bool(width_ok),
        forward=bool(np.all(np.diff(ln[:, 0]) > 0)),
        mark_x=float(mark[0]),
        heel_gap=float(ln[0, 0]),
        heel_z=float(up(heel_pts).max()),
        toe_x=float(ln[-1, 0]),
        toe_gap=float(li - ln[-1, 0]),
        press_heel=press[0], press_arch=press[1], press_ball=press[2],
        throat=float(throat), li=float(li),
    )


def score(m):
    """0 / 0.25 inside / 0.5 + home / 1.0 + flat. Every level needs the shoe upright on the table and both hands off."""
    ok = abs(m["tilt_deg"]) <= UPRIGHT_DEG and abs(m["shoe_dz"]) <= SOLE_ON_TABLE
    hands = m["hands_in_box"] == 0 and m["hands_to_insole"] >= HANDS_CLEAR
    if not (ok and hands):
        return 0.0, "shoe not upright on the table" if not ok else "a gripper is within 3 cm of the shoe or insole"
    inside = (m["x_min"] >= -0.005 and m["x_max"] <= m["li"] + 0.005 and m["width_ok"] and m["above_top"] <= 0.0
              and m["forward"] and m["mark_x"] >= m["throat"])
    if not inside:
        return 0.0, "insole not inside the shoe below the collar, unfolded, with its 20 mm toe mark under the vamp"
    home = m["heel_gap"] <= HEEL_GAP and m["heel_z"] <= HEEL_DOWN and m["toe_gap"] <= TOE_GAP
    if not home:
        return 0.25, "inside, not home (heel gap %.1f mm, heel %.1f mm up, toe gap %.1f mm)" % (
            m["heel_gap"] * 1e3, m["heel_z"] * 1e3, m["toe_gap"] * 1e3)
    worst = max(m["press_heel"], m["press_arch"], m["press_ball"])
    if worst > FLAT:
        return 0.5, "home, not flat (underside %.1f / %.1f / %.1f mm above the bed at heel / arch / ball)" % (
            m["press_heel"] * 1e3, m["press_arch"] * 1e3, m["press_ball"] * 1e3)
    return 1.0, "home and flat"


if __name__ == "__main__":  # engine-free checks of the score levels
    base = dict(tilt_deg=1.0, shoe_dz=0.0, hands_in_box=0, hands_to_insole=0.2, x_min=0.0005, x_max=0.268,
                above_top=-0.03, up_max=0.001, width_ok=True, forward=True, mark_x=0.247, heel_gap=0.001,
                heel_z=0.0005, toe_x=0.268, toe_gap=0.003, press_heel=0.0005, press_arch=0.001, press_ball=0.001,
                throat=0.13, li=0.271)
    cases = {
        "seated flat": ({}, 1.0),
        "raised at the arch": ({"press_arch": 0.012}, 0.5),
        "raised at the ball": ({"press_ball": 0.004}, 0.5),
        "heel on the counter": ({"above_top": 0.004, "heel_z": 0.06}, 0.0),
        "toe short, flat": ({"toe_gap": 0.03, "heel_gap": -0.002}, 0.25),
        "heel 8 mm out": ({"heel_gap": 0.008}, 0.25),
        "heel edge 4 mm up": ({"heel_z": 0.004}, 1.0),
        "heel edge 6 mm up": ({"heel_z": 0.006}, 0.25),
        "toe at the toe box, heel 4 mm": ({"heel_gap": 0.004, "toe_gap": 0.0}, 1.0),
        "folded back": ({"forward": False}, 0.0),
        "toe mark in the opening": ({"mark_x": 0.12}, 0.0),
        "shoe on its side": ({"tilt_deg": 85.0}, 0.0),
        "shoe lifted": ({"shoe_dz": 0.05}, 0.0),
        "still holding": ({"hands_in_box": 2}, 0.0),
        "hand near insole": ({"hands_to_insole": 0.01}, 0.0),
    }
    bad = 0
    for name, (chg, want) in cases.items():
        got, why = score({**base, **chg})
        bad += got != want
        print(("ok  " if got == want else "FAIL"), name, got, why)
    raise SystemExit(bad)
