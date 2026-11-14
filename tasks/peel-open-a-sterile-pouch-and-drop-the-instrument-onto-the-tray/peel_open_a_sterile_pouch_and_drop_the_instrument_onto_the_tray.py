"""On the station table a pair of sterile forceps waits in a peel pouch standing upright in a holder: a white paper
sheet on the left and a clear film sheet on the right, sealed together round three edges, the forceps lying inside
on the bottom seal. Above the seal the two sheets' top flaps are folded out, left and right, each ending in a stiff
pull tab. Beyond the holder lies a sterile instrument tray lined with a green drape; on either side stands a waste
bin.

Task: transfer the sterile forceps from their pouch onto the drape with aseptic technique. Only the forceps, the
pouch's inside and the drape are sterile: the forceps may touch nothing but the pouch's inside and the drape, and
nothing but the forceps may touch the drape or the tray. Then discard both pouch sheets in the waste bins and take
both arms back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=peel_open_a_sterile_pouch_and_drop_the_instrument_onto_the_tray.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
import yam_gripper_hull  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     segment, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the pouch: two sheets standing upright, wide along x, the paper on the left (+y) of the film -------------------
PW, PH = 0.16, 0.060  # wide (x), tall (z) up to the seal's top
ST = 0.002  # each sheet's thickness
GAP = 0.010  # between the sheets' inner faces
OFF = GAP / 2 + ST / 2  # each sheet's middle from the pouch's middle plane
LIP = 0.002  # the bottom seal: a strip across the gap on each sheet
FLAP = (0.030, 0.120, 0.004)  # each top flap, folded out: wide (x), out from its sheet (y), thick (z)
TAB = (0.006, 0.020, 0.035)  # the pull tab at each flap's end: thick (x), long (y), tall (z)
TAB_UP = 0.018  # each tab's top this far above its flap's top
STAND_X, STAND_BASE = 0.15, 0.006
P_BOT = TOP + STAND_BASE  # the pouch's bottom, standing in the holder
SEAL_K, SEAL_OPEN = 600.0, 0.012  # N/m holding the sheets together; peeled open once pulled this far apart (~7 N)

# -- the forceps: lying along x between the sheets, resting on the bottom seal --------------------------------------
F_LEN, F_T = 0.124, 0.004

# -- the tray (the sterile field) and the bins ------------------------------------------------------------------------
TRAY_C = np.array([0.36, 0.0])
TRAY_IN = (0.18, 0.30)  # inside the rim, x by y
TRAY_BASE, DRAPE, RIM_H, RIM_T = 0.003, 0.002, 0.015, 0.004
DRAPE_TOP = TOP + TRAY_BASE + DRAPE
RIM_TOP = TOP + TRAY_BASE + RIM_H
BIN_C = {"left": np.array([0.21, 0.25]), "right": np.array([0.21, -0.25])}
BIN_IN, BIN_H, BIN_T = (0.20, 0.16), 0.035, 0.004  # inside, x by y; wall height and thickness

WHITE_PAPER, FILM_BLUE, TAB_PAPER, TAB_FILM = (0.93, 0.93, 0.90), (0.45, 0.65, 0.85), (0.20, 0.45, 0.80), \
    (0.90, 0.50, 0.15)
STEEL, GREEN, TRAY_STEEL, BIN_GREY, ACRYLIC, PINK = (0.78, 0.80, 0.83), (0.15, 0.45, 0.30), (0.62, 0.64, 0.66), \
    (0.35, 0.37, 0.40), (0.80, 0.86, 0.90), (0.85, 0.40, 0.55)

# -- scoring ----------------------------------------------------------------------------------------------------------
GRIP_TOUCH = 0.003  # a gripper collision-mesh point this close to a prop touches it (points are 4 mm apart)
TOUCH = 0.0015  # a prop's surface point this close to another surface touches it
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _stand() -> Body:
    w, wall_h, slot = 0.012, 0.022, 0.024
    shapes = [BoxShape((0.0, 0.0, STAND_BASE / 2), (0.20, 0.06, STAND_BASE), ACRYLIC)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((0.0, s * (slot / 2 + w / 2), STAND_BASE + wall_h / 2), (0.20, w, wall_h), ACRYLIC))
    return Body.build("holder", pos=(STAND_X, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, color=ACRYLIC)


def _sheet(name: str, paper: bool) -> Body:
    """A sheet; origin at its bottom middle. The paper (left) also carries the bottom seal and the end seals, both
    across the gap; the film is clear (an invisible sheet with blue seal lines printed round its edges). Each has its
    top flap folded out, ending in a pull tab."""
    s = 1.0 if paper else -1.0
    inward = -s * OFF  # from this sheet's middle to the pouch's middle plane
    shapes = []
    if paper:
        shapes.append(BoxShape((0.0, 0.0, PH / 2), (PW, ST, PH), WHITE_PAPER))
        shapes.append(BoxShape((0.0, inward, LIP / 2), (PW - 0.004, GAP, LIP), WHITE_PAPER))
        for e in (-1.0, 1.0):
            shapes.append(BoxShape((e * (PW / 2 - 0.002), inward, PH / 2), (0.004, GAP, PH), WHITE_PAPER))
        face = s * (ST / 2 + 0.0002)  # printed lines and a process indicator on the paper's outside
        for k in range(4):
            shapes.append(BoxShape((-0.05 + k * 0.03, face, PH * 0.7), (0.020, 0.0004, 0.003), FILM_BLUE,
                                   collide=False))
        shapes.append(BoxShape((0.065, face, PH * 0.7), (0.010, 0.0004, 0.008), PINK, collide=False))
        color = TAB_PAPER
    else:
        shapes.append(BoxShape((0.0, 0.0, PH / 2), (PW, ST, PH), None))
        shapes.append(BoxShape((0.0, inward, LIP / 2), (PW - 0.004, GAP, LIP), None))
        face = s * (ST / 2 + 0.0003)
        shapes += [BoxShape((0.0, face, 0.005), (PW, 0.0005, 0.010), FILM_BLUE, collide=False),
                   BoxShape((0.0, face, PH - 0.003), (PW, 0.0005, 0.006), FILM_BLUE, collide=False)]
        for e in (-1.0, 1.0):
            shapes.append(BoxShape((e * (PW / 2 - 0.005), face, PH / 2), (0.010, 0.0005, PH), FILM_BLUE,
                                   collide=False))
        color = TAB_FILM
    shapes.append(BoxShape((0.0, s * FLAP[1] / 2, PH + FLAP[2] / 2), FLAP, WHITE_PAPER if paper else FILM_BLUE))
    shapes.append(BoxShape((0.0, s * (FLAP[1] - TAB[1] / 2), PH + FLAP[2] + TAB_UP - TAB[2] / 2), TAB, color))
    return Body.build(name, pos=(STAND_X, s * OFF, P_BOT), mass=0.012, shapes=shapes, friction=(1.2, 1.1),
                      color=color, contact_offset=0.001, com=(0.0, 0.0, PH / 2))


def _forceps() -> Body:
    """Dressing forceps: two steel blades joined at the back end (-x), lying in the pouch's middle plane."""
    b = F_T / 2
    x0, x1 = -F_LEN / 2, F_LEN / 2
    shapes = [segment((x0, 0.0, b), (x1, 0.0, b), F_T, STEEL),
              segment((x0 + 0.006, 0.0, b + 0.003), (x1, 0.0, 0.018), F_T, STEEL),
              BoxShape((x0 + 0.004, 0.0, 0.005), (0.010, F_T, 0.008), STEEL)]
    for k in range(5):  # serrated grip on the upper blade
        shapes.append(BoxShape((k * 0.006, 0.0, 0.0105 + k * 0.0006), (0.002, F_T + 0.0006, 0.0015),
                               (0.55, 0.57, 0.60), collide=False))
    return Body.build("forceps", pos=(STAND_X, 0.0, P_BOT + LIP + 0.0005), mass=0.03, shapes=shapes,
                      friction=(0.3, 0.25), slick=True, color=STEEL, contact_offset=0.001)


def _tray() -> Body:
    lx, ly = TRAY_IN
    shapes = [BoxShape((0.0, 0.0, TRAY_BASE / 2), (lx + 2 * RIM_T, ly + 2 * RIM_T, TRAY_BASE), TRAY_STEEL),
              BoxShape((0.0, 0.0, TRAY_BASE + DRAPE / 2), (lx, ly, DRAPE), GREEN)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (lx + RIM_T) / 2, 0.0, RIM_H / 2 + TRAY_BASE / 2), (RIM_T, ly + 2 * RIM_T, RIM_H),
                               TRAY_STEEL))
        shapes.append(BoxShape((0.0, s * (ly + RIM_T) / 2, RIM_H / 2 + TRAY_BASE / 2), (lx, RIM_T, RIM_H), TRAY_STEEL))
    return Body.build("tray", pos=(TRAY_C[0], TRAY_C[1], TOP), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.8, 0.7), color=TRAY_STEEL)


def _bin(side: str) -> Body:
    lx, ly = BIN_IN
    shapes = [BoxShape((0.0, 0.0, BIN_T / 2), (lx + 2 * BIN_T, ly + 2 * BIN_T, BIN_T), BIN_GREY)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (lx + BIN_T) / 2, 0.0, BIN_H / 2), (BIN_T, ly + 2 * BIN_T, BIN_H), BIN_GREY))
        shapes.append(BoxShape((0.0, s * (ly + BIN_T) / 2, BIN_H / 2), (lx, BIN_T, BIN_H), BIN_GREY))
    shapes.append(BoxShape((-(lx / 2 + BIN_T + 0.0002), 0.0, BIN_H * 0.6), (0.0004, 0.06, 0.012), (0.85, 0.75, 0.15),
                           collide=False))  # a yellow label on its front
    c = BIN_C[side]
    return Body.build(f"{side}_bin", pos=(c[0], c[1], TOP), mass=1.0, shapes=shapes, kinematic=True, color=BIN_GREY)


OBJECTS = [_stand(), _tray(), _bin("left"), _bin("right"), _sheet("paper", True), _sheet("film", False), _forceps()]
# The seal: the film slides off the paper (to the right) against a stiff spring; every other way the two are held
# together. Pulled 12 mm apart, it peels open: the task file switches the joint off.
slider("seal", "paper", "film", anchor=(STAND_X, 0.0, P_BOT + PH / 2), axis="Y", limits=(-0.03, 0.0),
       stiffness=SEAL_K, damping=2.0, target=0.0)

_SPEC = {o.name: o for o in OBJECTS}


def _set_seal(on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    joint = UsdPhysics.Joint(get_current_stage().GetPrimAtPath(bodykit._SLIDER_PATHS["seal"]))
    attr = joint.GetJointEnabledAttr() or joint.CreateJointEnabledAttr(True)
    attr.Set(bool(on))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"open": False, "contaminated": "", "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("POUCH_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _set_seal(False)  # rebuilt whole, from the reset poses
    _set_seal(True)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _box_dist(local, lo, hi) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0)))


def _box_dists(local, lo, hi) -> np.ndarray:
    return np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1)


_HULL = {k: np.array(getattr(yam_gripper_hull, k.upper()), float).reshape(-1, 3) / 1000.0
         for k in ("gripper", "tip_left", "tip_right")}


def _gripper_points(base, origin) -> np.ndarray:
    """Points on both grippers' collision meshes (gripper body and both fingers), world frame."""
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        for link, local in _HULL.items():
            i = next(j for j, n in enumerate(robot.body_names) if n == link or n.endswith("/" + link))
            g = poses[i]
            pts.append(g[:3] - origin + local @ rotation(g[3:7]).T)
    return np.concatenate(pts)


def _grid(lo, hi, n) -> np.ndarray:
    axes = [np.linspace(a, b, k) for a, b, k in zip(lo, hi, n)]
    return np.array(np.meshgrid(*axes, indexing="ij")).reshape(3, -1).T


def _sheet_points(paper: bool) -> np.ndarray:
    """Points on every surface of a sheet (panel, folded flap, pull tab), in its own frame."""
    s = 1.0 if paper else -1.0
    out = [_grid((-PW / 2, -ST / 2, 0.0), (PW / 2, ST / 2, PH), (11, 2, 9))]
    fy = sorted((0.0, s * FLAP[1]))
    out.append(_grid((-FLAP[0] / 2, fy[0], PH), (FLAP[0] / 2, fy[1], PH + FLAP[2]), (3, 13, 2)))
    tc = np.array([0.0, s * (FLAP[1] - TAB[1] / 2), PH + FLAP[2] + TAB_UP - TAB[2] / 2])
    out.append(_grid(tc - np.array(TAB) / 2, tc + np.array(TAB) / 2, (2, 3, 4)))
    return np.concatenate(out)


_SHEET_PTS = {"paper": _sheet_points(True), "film": _sheet_points(False)}
_TRAY_OUT = (TRAY_IN[0] / 2 + RIM_T, TRAY_IN[1] / 2 + RIM_T)
# the holder's three blocks, world axis-aligned (it never moves): (centre, half size)
_HOLDER = [((STAND_X, 0.0, TOP + STAND_BASE / 2), (0.10, 0.03, STAND_BASE / 2))] + \
    [((STAND_X, s * 0.018, TOP + STAND_BASE + 0.011), (0.10, 0.006, 0.011)) for s in (-1.0, 1.0)]


def _on_sterile_field(p, margin) -> np.ndarray:
    """Which points touch the tray: the drape inside the rim, or the rim itself."""
    dx = np.abs(p[:, 0] - TRAY_C[0]) - TRAY_IN[0] / 2
    dy = np.abs(p[:, 1] - TRAY_C[1]) - TRAY_IN[1] / 2
    inside = (dx <= 0.0) & (dy <= 0.0)
    on_rim = ~inside & (dx <= RIM_T + margin) & (dy <= RIM_T + margin)
    return (inside & (p[:, 2] < DRAPE_TOP + margin)) | (on_rim & (p[:, 2] < RIM_TOP + margin))


def _in_bin(p) -> bool:
    return any(abs(p[0] - c[0]) <= BIN_IN[0] / 2 and abs(p[1] - c[1]) <= BIN_IN[1] / 2 for c in BIN_C.values()) \
        and p[2] < TOP + BIN_H


def _forceps_contaminated(fw, sheets) -> str:
    """What non-sterile surface the forceps' surface points touch, if any: the table, the holder, a bin, the tray's
    rim, or the outside of the pouch (a sheet's outer face, its folded flap or its pull tab)."""
    outside_tray = (np.abs(fw[:, 0] - TRAY_C[0]) > _TRAY_OUT[0]) | (np.abs(fw[:, 1] - TRAY_C[1]) > _TRAY_OUT[1])
    if np.any(outside_tray & (fw[:, 2] < TOP + TOUCH)):
        return "table"
    for c, h in _HOLDER:
        if np.any(_box_dists(fw - np.array(c), -np.array(h), np.array(h)) < TOUCH):
            return "holder"
    for c in BIN_C.values():
        h = np.array([BIN_IN[0] / 2 + BIN_T, BIN_IN[1] / 2 + BIN_T, BIN_H / 2])
        if np.any(_box_dists(fw - np.array([c[0], c[1], TOP + BIN_H / 2]), -h, h) < TOUCH):
            return "bin"
    dx = np.abs(fw[:, 0] - TRAY_C[0]) - TRAY_IN[0] / 2
    dy = np.abs(fw[:, 1] - TRAY_C[1]) - TRAY_IN[1] / 2
    rim = ~((dx <= 0.0) & (dy <= 0.0)) & (dx <= RIM_T + TOUCH) & (dy <= RIM_T + TOUCH)
    if np.any(rim & (fw[:, 2] < RIM_TOP + TOUCH)):
        return "tray rim"
    for name, (p, r) in sheets.items():
        s = 1.0 if name == "paper" else -1.0
        q = (fw - p) @ r
        outer = (np.abs(q[:, 0]) <= PW / 2 + TOUCH) & (q[:, 2] >= 0.002) & (q[:, 2] <= PH + TOUCH) & \
            (s * q[:, 1] >= ST / 2 - 0.0005) & (s * q[:, 1] <= ST / 2 + TOUCH)
        if np.any(outer):
            return f"outside of the {name}"
        fy = sorted((0.0, s * FLAP[1]))
        if np.any(_box_dists(q, np.array([-FLAP[0] / 2, fy[0], PH]),
                             np.array([FLAP[0] / 2, fy[1], PH + FLAP[2]])) < TOUCH):
            return f"{name}'s flap"
        tc = np.array([0.0, s * (FLAP[1] - TAB[1] / 2), PH + FLAP[2] + TAB_UP - TAB[2] / 2])
        if np.any(_box_dists(q, tc - np.array(TAB) / 2, tc + np.array(TAB) / 2) < TOUCH):
            return f"{name}'s pull tab"
    return ""


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pp, pr = _pose(base, origin, "paper")
    fp, fr = _pose(base, origin, "film")
    kp, kr = _pose(base, origin, "forceps")
    apart = float((pp - fp) @ pr[:, 1]) - 2 * OFF  # how far the film has been pulled off the paper
    if not _EP["open"] and apart >= SEAL_OPEN:
        _EP["open"] = True
        _set_seal(False)
    _EP["tcps"] = _tcps(base, origin)
    lo, hi = _SPEC["forceps"].local_bounds()
    fw = kp + _F_PTS(lo, hi) @ kr.T  # the forceps' surface points, world
    gw = _gripper_points(base, origin)
    sheets = {"paper": (pp, pr), "film": (fp, fr)}
    if not _EP["contaminated"]:
        why = ""
        if np.any(_box_dists((gw - kp) @ kr, lo, hi) < GRIP_TOUCH):
            why = "a gripper touched the forceps"
        elif np.any(_on_sterile_field(gw, GRIP_TOUCH)):
            why = "a gripper touched the tray"
        else:
            for name, (p, r) in sheets.items():
                if np.any(_on_sterile_field(p + _SHEET_PTS[name] @ r.T, TOUCH)):
                    why = f"the {name} touched the tray"
                    break
        if not why:
            what = _forceps_contaminated(fw, sheets)
            if what:
                why = f"the forceps touched the {what}"
        if why:
            _EP["contaminated"] = why
    k_mid = kp + kr @ ((lo + hi) / 2)
    on_tray = abs(k_mid[0] - TRAY_C[0]) <= TRAY_IN[0] / 2 and abs(k_mid[1] - TRAY_C[1]) <= TRAY_IN[1] / 2 \
        and k_mid[2] < DRAPE_TOP + 0.015
    mids = {n: p + r @ np.array([0.0, 0.0, PH / 2]) for n, p, r in (("paper", pp, pr), ("film", fp, fr))}
    near = float(np.min(_box_dists((gw - kp) @ kr, lo, hi)))
    _EP["s"] = {"apart": apart, "on_tray": on_tray, "binned": all(_in_bin(m) for m in mids.values()), "near": near,
                "k_mid": k_mid, "mids": mids}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[pouch] t={_DEBUG['steps'] / 20:5.1f}s apart={apart * 1000:5.1f}mm open={_EP['open']} "
                  f"forceps={np.round(k_mid * 1000).astype(int).tolist()} on_tray={on_tray} "
                  f"gripper-forceps={near * 1000:.0f}mm contaminated={_EP['contaminated']!r} "
                  f"paper={np.round(mids['paper'] * 1000).astype(int).tolist()} "
                  f"film={np.round(mids['film'] * 1000).astype(int).tolist()} binned={_EP['s']['binned']} "
                  f"leftTCP={np.round(tl * 1000).astype(int).tolist()} rightTCP={np.round(tr * 1000).astype(int).tolist()} "
                  f"score={score(None)}", file=sys.stderr, flush=True)


def _F_PTS(lo, hi) -> np.ndarray:
    if "f" not in _FP:
        _FP["f"] = _grid(lo, hi, (13, 3, 4))
    return _FP["f"]


_FP: dict = {}


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once anything breaks asepsis, at any moment (latched): a gripper (any part of its collision meshes, within
    3 mm) touching the forceps or the tray; any surface of the pouch touching the tray (drape or rim, within 1.5 mm);
    or the forceps touching anything non-sterile (within 1.5 mm): the table, the holder, a bin, the tray's rim, a
    sheet's outer face, flap or pull tab. Otherwise 1.0 with the pouch peeled open (its sheets pulled at least 12 mm
    apart at some point), the forceps lying on the drape inside the tray's rim, each sheet in a waste bin and both
    arms back where they started; 0.5 with the pouch peeled open and the forceps on the drape; 0.25 with the pouch
    peeled open at some point; else 0.0."""
    st = _EP["s"]
    if _EP["contaminated"] or not _EP["open"]:
        return 0.0
    if not st.get("on_tray"):
        return 0.25
    if st.get("binned") and _arms_home():
        return 1.0
    return 0.5


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the forceps and both sheets still: the trial ends there."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("forceps", "paper", "film"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands come straight down on the pull tabs, fingers closing across them (along x): the left on the paper's tab,
# the right on the film's. Together they lift the pouch out of the holder and carry it over the tray, then pull the
# sheets apart, the left to the left and the right to the right, in one line: the seal gives, and the forceps drop
# onto the drape. Each hand carries its sheet to the waste bin on its side and drops it in, and both arms go back to
# where they started.

L_GRIP = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x
R_GRIP = grip_rotation(math.pi / 2)
TAB_OPEN = -0.010  # fingers opened ~1 cm wider than the tab
TAB_IN = -0.004  # the TCP this far below a tab's top: 4 mm above it, the fingertips pinching ~9 mm deep
LIFT = 0.050
CARRY = TRAY_C[0] - STAND_X
PULL = 0.045  # each hand pulls this far (aiming past the arm's lag)
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _tab(side: str) -> np.ndarray:
    s = 1.0 if side == "left" else -1.0
    return np.array([STAND_X, s * (OFF + FLAP[1] - TAB[1] / 2), P_BOT + PH + FLAP[2] + TAB_UP - TAB_IN])


def _hand(side: str, who: str, extra: float) -> list[Waypoint]:
    s = 1.0 if side == "left" else -1.0
    grip = L_GRIP if side == "left" else R_GRIP
    tab = _tab(side)
    up = tab + [0.0, 0.0, LIFT]
    over = up + [CARRY, 0.0, 0.0]
    pulled = over + [0.0, s * PULL, 0.0]
    drop = np.array([BIN_C[side][0], BIN_C[side][1] + s * 0.09, over[2]])  # the sheet hangs ~11 cm inward of the tab
    out = _lead_in(side, tab + [0.0, 0.0, 0.04], grip, f"over {who}'s pull tab", TAB_OPEN)
    out += [_w(1.0, tab, TAB_OPEN, "down round it", grip),
            _w(0.6, tab, FINGER_CLOSED, "take it", grip),
            _w(1.5, up, FINGER_CLOSED, "lift the pouch out of the holder", grip),
            _w(2.5, over, FINGER_CLOSED, "carry it over the tray", grip),
            _w(0.5, over, FINGER_CLOSED, "hold it there", grip),
            _w(2.0, pulled, FINGER_CLOSED, "peel the sheets apart: the forceps drop onto the drape", grip),
            _w(1.0, pulled, FINGER_CLOSED, "hold while the forceps settle", grip),
            _w(2.5, drop, FINGER_CLOSED, "carry the sheet over the waste bin on this side", grip),
            _w(0.6, drop, FINGER_OPEN, "drop it in", grip),
            _w(0.8, drop + [0.0, 0.0, 0.04], FINGER_OPEN, "rise", grip)]
    out += _go_home(side, drop + [0.0, 0.0, 0.04], grip, FINGER_OPEN)
    out.append(_w(extra, out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out


def demo():
    return {"left": _hand("left", "the paper", 3.0), "right": _hand("right", "the film", 3.0)}
