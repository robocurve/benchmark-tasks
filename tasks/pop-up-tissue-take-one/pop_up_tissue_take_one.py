"""On the station table lies a flat pop-up tissue box at full size (Kleenex Professional flat box 21400: printed
paperboard, 22.5 x 12.1 x 5.1 cm; 40 g with a light spacer and its last two sheets) and, to the right, a tray. Centred
in the box's top is the usual oval opening (8.9 x 5.7 cm), covered by clear plastic film with one slit (8.3 cm) along
the oval's long axis, which is also the box's. One film edge of the slit is fixed; the other is a light sprung strip
that presses whatever passes through the slit against the fixed edge (about 0.14 N). The top tissue stands 25 mm up out
of the slit. Inside, it runs down through the slit, bends round the leading edge of the next tissue and lies under it,
interleaved as in a real pop-up box; the next tissue's leading edge waits just inside the slit, pressed against the top
one.

Both tissues are real soft bodies (PhysX surface-deformable cloth, 550-675 mesh nodes each, creased as folded),
not rigid parts, and nothing in the box is scripted: pull the top tissue up and the film slit squeezes it against
the next one, so friction drags the next tissue's edge up through the slit after it; once the top one's end has
passed, the film holds the next one standing there, ready. The same squeeze drags on the box (only 40 g): pulled
one-handed it rocks a little (in sim about 1.6 deg and 8 mm) but stays put, so the demo's steadying hand is a
precaution. A single pinch on the thin sheet creeps out of the fingers under that drag, so the demo pulls hand over
hand.

The job: take one tissue out and put it on the tray, leaving the box where it is with the next tissue ready to pull.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/pop-up-tissue-take-one/pop_up_tissue_take_one.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (the box and its sprung film strip, per-episode memory, the grippers' TCPs)
import softkit as sk  # noqa: E402  (the two tissues: PhysX surface-deformable cloth, read as mesh nodes)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: log the box pose and both tissues' heights every 10 steps to ~/dbg/b18-tissue.jsonl

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
XB, YB = 0.230, -0.060  # the centre of the box and of its oval opening and slit, on the table (the box's origin)
IN = 0.0254
# A real flat pop-up box at full size (Kleenex Professional flat box 21400: 8.875 x 4.75 x 2.0 in) lying with its
# long axis along x. Box-local geometry: origin on the box's bottom at its centre; the oval opening is centred in
# the top, its long axis along x (3.5 x 2.25 in), and the film's slit (3.25 in) runs along that long axis at y = 0.
BX, BY, BH = 8.875 * IN, 4.75 * IN, 2.0 * IN  # 225 x 121 x 51 mm
WALL, LID = 0.003, 0.003
Y0, Y1 = -BY / 2, BY / 2  # the box's long sides
ZT, ZL = BH, BH - LID  # lid top, lid underside
OVAL_A, OVAL_B = 3.5 * IN / 2, 2.25 * IN / 2  # the oval opening's half length (x) and half width (y)
SLX = 3.25 * IN / 2  # film slit half length
FACE = -0.003  # the fixed film edge of the slit
LIP_T, LIP_H = 0.004, 0.010  # sprung film edge: thickness (y), depth (z, from the lid top down)
D0 = 0.006  # sprung edge travel at spawn (both tissues in the slit)
SLOT = (-0.003, 0.010)  # the slit's open gap in y, between the fixed edge and the film beyond the sprung edge
FILM_T = 0.001
STACK_TOP = BH - 0.020  # top of the spacer under the last two sheets, 20 mm under the lid top (rigid, part of the box)
BOX_KG = 0.040
CARD, WHITE, FILM, TRAY_C = (0.30, 0.50, 0.72), (0.97, 0.97, 0.95), (0.80, 0.84, 0.88), (0.55, 0.55, 0.58)
PINK = (0.98, 0.70, 0.78)  # pale pink tissues (a common facial-tissue tint), easy to see against the box and tray


def _oval_x(y: float) -> float:
    """Half length of the oval opening at lateral offset y (0 outside it)."""
    return OVAL_A * math.sqrt(max(0.0, 1.0 - (y / OVAL_B) ** 2))


def _box_boxes():
    """Walls, the spacer, the paperboard lid with its oval opening (a staircase of thin strips) and the plastic film
    over the opening, slit along the long axis (the film's fixed edge of the slit is a 10 mm deep lip)."""
    b = [mk.box((BX - 2 * WALL, BY - 2 * WALL, STACK_TOP), at=(0, 0, STACK_TOP / 2), color=WHITE)]
    for s in (-1, 1):
        b.append(mk.box((WALL, BY, ZL), at=(s * (BX / 2 - WALL / 2), 0, ZL / 2)))
        b.append(mk.box((BX - 2 * WALL, WALL, ZL), at=(0, s * (BY / 2 - WALL / 2), ZL / 2)))
    zl = ZL + LID / 2
    for s in (-1, 1):  # lid: the two ends beyond the oval and the two long sides beside it
        b.append(mk.box((BX / 2 - OVAL_A, BY, LID), at=(s * (BX / 2 + OVAL_A) / 2, 0, zl)))
        b.append(mk.box((2 * OVAL_A, BY / 2 - OVAL_B, LID), at=(0, s * (BY / 2 + OVAL_B) / 2, zl)))
    cut = [-0.006, *SLOT]  # the fixed lip's back face and the slit's open gap
    edges = sorted([e for e in (-OVAL_B + k * OVAL_B / 10 for k in range(21)) if min(abs(e - c) for c in cut) > 0.001]
                   + cut)
    zf = ZT - FILM_T / 2
    for y0, y1 in zip(edges[:-1], edges[1:]):
        ym, h = (y0 + y1) / 2, y1 - y0
        xe = _oval_x(ym)
        for s in (-1, 1):  # paperboard round the oval, band by band
            if OVAL_A - xe > 0.0005:
                b.append(mk.box((OVAL_A - xe, h, LID), at=(s * (OVAL_A + xe) / 2, ym, zl)))
        if -0.006 <= ym <= SLOT[1]:  # the slit and its fixed lip: film only beyond the slit's ends
            for s in (-1, 1):
                if xe - SLX > 0.0005:
                    b.append(mk.box((xe - SLX, h, FILM_T), at=(s * (xe + SLX) / 2, ym, zf), color=FILM))
        else:
            b.append(mk.box((2 * xe, h, FILM_T), at=(0, ym, zf), color=FILM))
    b.append(mk.box((2 * SLX, FACE + 0.006, LIP_H), at=(0, (FACE - 0.006) / 2, ZT - LIP_H / 2), color=FILM))
    return b


def _arc(cy, cz, r, a0, a1, n=8):
    return [(cy + r * math.cos(a0 + (a1 - a0) * k / n), cz + r * math.sin(a0 + (a1 - a0) * k / n))
            for k in range(n + 1)]


ZC = ZT - LIP_H  # centre of both bends
TAB = 0.025  # the top tissue stands this far above the lid
# cross-sections (box-local y, z): top tissue = tab, down the slit, round the next one's edge, flat under it
PATH_1 = tuple([(-0.001, ZT + TAB)] + _arc(0.007, ZC, 0.008, math.pi, 1.5 * math.pi) + [(0.037, ZC - 0.008)])
PATH_2 = tuple([(0.003, ZT - 0.001)] + _arc(0.007, ZC, 0.004, math.pi, 1.5 * math.pi) + [(0.053, ZC - 0.004)])
SHEET_W = 0.070
SHEET_DX = 0.002  # the sheets sit 2 mm forward of the slit's middle, 4 mm inside its front end
CLOTH = dict(kind="cloth", friction=3.0, edge=0.003, color=PINK)

kit = mk.Kit()
kit.part("box", (XB, YB, TOP), _box_boxes(), mass=BOX_KG, color=CARD)
kit.part("film", (XB, YB + FACE + 0.002 + LIP_T / 2, TOP + ZT - LIP_H / 2), [mk.box((2 * SLX, LIP_T, LIP_H))],
         mass=0.003, color=FILM)
kit.slider("film_slide", "box", "film", (0, 1, 0), 0.0, 0.012, start=D0, stiffness=20.0, rest=0.0)
TISSUE_1 = sk.Soft("tissue_1", pos=(XB + SHEET_DX, YB, TOP), size=(SHEET_W, sk.path_length(PATH_1), 0.001),
                   profile=PATH_1, **CLOTH)
TISSUE_2 = sk.Soft("tissue_2", pos=(XB + SHEET_DX, YB, TOP), size=(SHEET_W, sk.path_length(PATH_2), 0.001),
                   profile=PATH_2, **CLOTH)

# the tray, right of the box
TRX, TRY = 0.260, -0.250
TRAY_IN = (0.150, 0.120)
TRAY_T, RIM_H, RIM_W = 0.006, 0.012, 0.004
TRAY_TOP = TOP + TRAY_T


def _tray():
    out = [Box("tray_base", pos=(TRX, TRY, TOP + TRAY_T / 2), size=(TRAY_IN[0] + 2 * RIM_W, TRAY_IN[1] + 2 * RIM_W,
                                                                    TRAY_T), color=TRAY_C)]
    zc = TRAY_TOP + RIM_H / 2
    for s in (-1, 1):
        out.append(Box(f"tray_rim_x{'np'[s > 0]}", pos=(TRX + s * (TRAY_IN[0] / 2 + RIM_W / 2), TRY, zc),
                       size=(RIM_W, TRAY_IN[1] + 2 * RIM_W, RIM_H), color=TRAY_C))
        out.append(Box(f"tray_rim_y{'np'[s > 0]}", pos=(TRX, TRY + s * (TRAY_IN[1] / 2 + RIM_W / 2), zc),
                       size=(TRAY_IN[0], RIM_W, RIM_H), color=TRAY_C))
    return out


OBJECTS = [*kit.objects(), TISSUE_1, TISSUE_2, *_tray()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring, from the box's pose and the tissues' mesh nodes ---------------------------------------------------
# Latched for the trial (score 0) only for consequential events, at any step:
TIP_DEG = 45.0  # the box tipped over: tilted more than 45 deg
OFF_LIFT = 0.050  # the box off the table: its lowest corner more than 5 cm above the table (carried up with the sheet)
OFF_DROP = 0.020  # ... or its bottom more than 2 cm below the table top (fallen off the edge)
# (the second tissue spilled out of the box too: none of its nodes left inside, see next_out)
# A brief lift or tilt below those that settles costs nothing: the box's FINAL placement is judged instead.
SPOT_TOL = 0.015  # at the end, the box within 1.5 cm of where it stood ...
YAW_TOL = 10.0  # ... turned less than 10 deg ...
UPRIGHT_TOL = 5.0  # ... upright (tilted less than 5 deg) ...
REST_TOL = 0.005  # ... and resting on the table (lowest corner within 5 mm of it)
READY_H = 0.010  # the next tissue stands at least 10 mm above the lid ...
IN_FRAC = 0.25  # ... with at least a quarter of its nodes still under the lid (it is still in the box)
TRAY_FRAC = 0.80  # on the tray: 80 % of the tissue's nodes over the tray's floor, none higher than ...
TRAY_ABOVE = 0.040  # ... 4 cm over it (lying there, not held)
CLEAR = 0.03  # let go: both TCPs at least 3 cm from every tissue node and from the box
STILL = 0.02  # m/s, every node (from node displacement between control steps)
M = "puto"  # this task's key in the per-episode memory
_TEST_TCPS = None  # self-test stand-in for the TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"tipped": False, "off_table": False, "two_out": False}
    return m


def _box_frame(objects):
    pos, q = kit.pose(objects, "box")
    r = mk.quat_to_mat(q)
    return np.asarray(pos, dtype=float), r


def box_tilt_deg(objects) -> float:
    _, r = _box_frame(objects)
    return float(math.degrees(math.acos(max(-1.0, min(1.0, r[2, 2])))))


def box_yaw_deg(objects) -> float:
    """How far the box has turned about the vertical from its start (deg)."""
    _, r = _box_frame(objects)
    return float(abs(math.degrees(math.atan2(r[1, 0], r[0, 0]))))


def box_lift(objects) -> float:
    """How far the box's lowest corner is above the table (m)."""
    pos, r = _box_frame(objects)
    corners = np.array([(sx * BX / 2, sy, 0.0) for sx in (-1, 1) for sy in (Y0, Y1)]) @ r.T + pos
    return float(corners[:, 2].min() - TOP)


def local(objects, name) -> np.ndarray:
    """A tissue's nodes in the box frame (origin on its bottom under the slit's centre, slit along x)."""
    pos, r = _box_frame(objects)
    return (sk.nodes(objects[name]) - pos) @ r


def in_box(p) -> np.ndarray:
    """Nodes inside the box, under the lid."""
    return (np.abs(p[:, 0]) < BX / 2) & (p[:, 1] > Y0) & (p[:, 1] < Y1) & (p[:, 2] < ZL)


def first_out(objects) -> bool:
    """The top tissue is out: none of its nodes is inside the box under the lid."""
    return not in_box(local(objects, "tissue_1")).any()


def on_tray(objects, name) -> bool:
    p = sk.nodes(objects[name])
    over = (np.abs(p[:, 0] - TRX) <= TRAY_IN[0] / 2) & (np.abs(p[:, 1] - TRY) <= TRAY_IN[1] / 2)
    return bool(over.mean() >= TRAY_FRAC and p[:, 2].max() <= TRAY_TOP + TRAY_ABOVE)


def next_height(objects) -> float:
    """How far the next tissue's highest node stands above the lid (m, box frame)."""
    return float(local(objects, "tissue_2")[:, 2].max() - ZT)


def next_ready(objects) -> bool:
    """The next tissue stands at least 10 mm above the lid and is still in the box (a quarter of it under the lid)."""
    p = local(objects, "tissue_2")
    return bool(p[:, 2].max() - ZT >= READY_H and in_box(p).mean() >= IN_FRAC)


def next_out(objects) -> bool:
    """The next tissue has left the box: none of its nodes inside under the lid."""
    return not in_box(local(objects, "tissue_2")).any()


def released(objects) -> bool:
    """Both TCPs at least 3 cm from every node of both tissues and from the box."""
    tc = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    return (sk.clear_of_tcps(objects["tissue_1"], CLEAR, tc) and sk.clear_of_tcps(objects["tissue_2"], CLEAR, tc)
            and _box_clear(objects, tc))


def _box_clear(objects, tc) -> bool:
    st = objects["box"]
    lo, hi = np.asarray(st.low) - CLEAR, np.asarray(st.high) + CLEAR
    return not any(np.all((np.asarray(t) >= lo) & (np.asarray(t) <= hi)) for t in tc)


def still(objects) -> bool:
    b = objects["box"]
    return (sk.motion(objects["tissue_1"], "tissue_1") < STILL and sk.motion(objects["tissue_2"], "tissue_2") < STILL
            and float(np.linalg.norm(b.lin_vel)) < STILL)


def box_in_place(objects) -> bool:
    """The box's final placement: within 1.5 cm of its spot, turned < 10 deg, upright (< 5 deg) and resting on the
    table (lowest corner within 5 mm of it)."""
    pos, _ = _box_frame(objects)
    return bool(math.hypot(pos[0] - XB, pos[1] - YB) <= SPOT_TOL and box_yaw_deg(objects) <= YAW_TOL
                and box_tilt_deg(objects) <= UPRIGHT_TOL and abs(box_lift(objects)) <= REST_TOL)


def score(objects) -> float:
    """0, latched, only after a consequential event: the box tipped over (tilted > 45 deg), off the table (lowest
    corner > 5 cm above it, or bottom > 2 cm below its top), or the next tissue spilled out of the box too. A brief
    lift or tilt short of those that settles costs nothing. 0 at the end unless the box is back in place (within
    1.5 cm of its spot, turned < 10 deg, tilted < 5 deg, resting on the table) and the top tissue is out. Otherwise
    0.25 (top tissue out), + 0.25 if it lies on the tray, + 0.5 if the next tissue stands >= 10 mm above the lid with
    a quarter of it still under the lid. Halved unless both grippers are 3 cm clear of the tissues and the box and
    everything is still."""
    m = _mem()
    if box_tilt_deg(objects) > TIP_DEG:
        m["tipped"] = True
    pos, _ = _box_frame(objects)
    if box_lift(objects) > OFF_LIFT or pos[2] < TOP - OFF_DROP:
        m["off_table"] = True
    if next_out(objects):
        m["two_out"] = True
    if DEBUG and mk._EP.get("env") is not None and mk._EP.get("step", 0) % 10 == 0:
        _debug(objects)
    if m["tipped"] or m["off_table"] or m["two_out"]:
        return 0.0
    if not box_in_place(objects) or not first_out(objects):
        return 0.0
    base = 0.25 + (0.25 if on_tray(objects, "tissue_1") else 0.0) + (0.5 if next_ready(objects) else 0.0)
    ok = released(objects) and still(objects)
    return round(base if ok else base / 2, 6)


def _debug(objects) -> None:
    import json
    import time

    pos, _ = _box_frame(objects)
    p1, p2 = local(objects, "tissue_1"), local(objects, "tissue_2")
    rec = {"k": mk._EP.get("step"), "t": round(time.time(), 3), "box": np.round(pos, 4).tolist(),
           "lift_mm": round(box_lift(objects) * 1000, 1), "tilt": round(box_tilt_deg(objects), 2),
           "film_mm": round(float(((objects["film"].pos - pos) @ _box_frame(objects)[1])[1] - 0.001) * 1000, 2),
           "t1": {"zmax": round(float(p1[:, 2].max() - ZT) * 1000, 1), "zmin": round(float(p1[:, 2].min() - ZT) * 1000,
                                                                                    1),
                  "in": round(float(in_box(p1).mean()), 3), "tray": on_tray(objects, "tissue_1"),
                  "xy": np.round(np.r_[sk.nodes(objects["tissue_1"])[:, :2].min(0),
                                       sk.nodes(objects["tissue_1"])[:, :2].max(0)], 3).tolist()},
           "t2": {"zmax": round(float(p2[:, 2].max() - ZT) * 1000, 1), "in": round(float(in_box(p2).mean()), 3),
                  "ylo": round(float(p2[:, 1].min()) * 1000, 1), "yhi": round(float(p2[:, 1].max()) * 1000, 1)},
           "m1": round(sk.motion(objects["tissue_1"], "tissue_1"), 4),
           "m2": round(sk.motion(objects["tissue_2"], "tissue_2"), 4),
           "rel": released(objects), "mem": dict(_mem()), "tcps": [np.round(t, 4).tolist() for t in mk.tcps()]}
    with open(str(Path.home() / "dbg" / "b18-tissue.jsonl"), "a") as fh:
        fh.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    """Top tissue on the tray, next one ready, box back in place and never tipped or off the table, let go and
    still; held 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Top-tissue nodes still in the box, plus how far the next tissue is short of standing 10 mm up (m)."""
    return float(in_box(local(objects, "tissue_1")).sum() + max(0.0, READY_H - next_height(objects)))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry. The left hand comes down first, fingers closed, and presses the lid's near
# left corner (108 mm behind the slit's middle, as far from the pulling hand as the box allows) to steady it. The
# right hand waits beside the box, then comes over the standing tab with the fingers part open (30 mm), goes
# down until the pad tips are 6 mm above the lid (the pads then cover the tab from 6 to 22 mm up) and closes to a 2 mm
# jaw gap, a firm pinch on the thin tissue (closing fully on a single sheet squeezes it out of the jaws in this
# simulator). It pulls straight up hand over hand (two 3 cm strokes, each time sliding the eased pads back down to
# 20 mm above the lid and pinching again, then 10 cm), which draws the whole tissue out of the slit and, through the
# slit's squeeze, the next tissue's edge up after it. Then the left hand lets go of
# the box, the right carries the tissue to the tray, lowers it until it touches and opens; both arms park.

TIP = 0.013  # pad tips below the TCP
GRIP = -0.0010  # finger target: 2 mm jaw gap
APPROACH = -0.015  # 30 mm jaw gap coming down round the tab
PRESS = (XB - 0.108, YB + 0.045)  # the lid's near left corner, 5 mm in from the near end, 15 mm from the side
PINCH_DX = 0.006  # the right hand pinches the tab 6 mm forward of the slit's middle (4 mm from the sheet's)
DROP_Y = TRY - 0.025  # the sheet's creased lower end folds over toward +y as it lands
PULL = 0.106  # (single-stroke pull height, kept for the controls)
STAGE = 0.030  # each of the two short strokes
EASE = -0.003  # 6 mm jaw gap: the pads slide down the sheet without letting it out from between them
REGRIP = 0.020  # pad tips this high above the lid for the re-pinch (the next tissue's edge stands <= ~17 mm)
LAST = 0.100  # the final stroke
L_PARK, R_PARK = (0.150, 0.250, 0.950), (0.200, -0.250, TOP + 0.160)
NAIVE = False  # True: one-handed, the right hand pulls the same way and nobody holds the box (sim check)


def demo():
    """Hold the box down with the left hand; pull the top tissue straight up and out with the right; lay it on the
    tray."""
    c = FINGER_CLOSED
    g = (XB + PINCH_DX, YB - 0.001, TOP + ZT + 0.006 + TIP)
    R, L = [], []

    def w(arm, dur, pos, f, label, yaw=0.0):
        arm.append(Waypoint(dur, tuple(float(v) for v in pos), f, label, yaw))

    w(R, 3.0, (g[0] + 0.04, g[1] - 0.04, g[2] + 0.06), APPROACH, "beside the box, fingers part open")
    w(R, 1.5, (g[0], g[1], g[2] + 0.06), APPROACH, "over the tab once the left hand is down")
    w(R, 1.5, g, APPROACH, "down round the tab")
    w(R, 0.5, g, APPROACH, "settle")
    w(R, 1.0, g, GRIP, "pinch the tab")
    w(R, 0.5, g, GRIP, "settle")
    # Hand over hand: the thin sheet creeps through the pads under the slit's drag, so pull 3 cm, ease the pads to a
    # 6 mm gap, slide them back down the sheet to 20 mm above the lid (clear of the next tissue's edge) and pinch
    # again; twice, then the long pull that draws the rest out.
    lo = (g[0], g[1], TOP + ZT + REGRIP + TIP)
    for k in (1, 2):
        top = (g[0], g[1], (g[2] if k == 1 else lo[2]) + STAGE)
        w(R, 2.0, top, GRIP, f"pull {k}: 3 cm up")
        w(R, 0.4, top, EASE, "ease the pads")
        w(R, 1.0, lo, EASE, "slide them down the sheet")
        w(R, 0.6, lo, GRIP, "pinch again")
        w(R, 0.3, lo, GRIP, "settle")
    w(R, 6.5, (lo[0], lo[1], lo[2] + LAST), GRIP, "pull straight up, out of the slit")
    w(R, 1.0, (lo[0], lo[1], lo[2] + LAST), GRIP, "hold")
    held = sum(x.duration for x in R)
    w(R, 2.5, (TRX, DROP_Y, lo[2] + LAST), GRIP, "carry to the tray")
    w(R, 1.5, (TRX, DROP_Y, TRAY_TOP + 0.078), GRIP, "lower until it touches")
    w(R, 1.0, (TRX, DROP_Y, TRAY_TOP + 0.078), FINGER_OPEN, "let go")
    w(R, 1.0, (TRX, DROP_Y, TRAY_TOP + 0.150), FINGER_OPEN, "up")
    w(R, 2.0, R_PARK, FINGER_OPEN, "park")
    total = sum(x.duration for x in R) + 1.0
    w(R, total - sum(x.duration for x in R), R_PARK, FINGER_OPEN, "wait")
    if NAIVE:
        w(L, 2.0, L_PARK, FINGER_OPEN, "park")
        w(L, total - 2.0, L_PARK, FINGER_OPEN, "park")
        return {"left": L, "right": R}
    lid = TOP + ZT + TIP
    w(L, 2.5, (*PRESS, lid + 0.05), c, "over the lid's left side, fingers closed")
    w(L, 1.5, (*PRESS, lid - 0.004), c, "press the box down")
    w(L, held - 4.0, (*PRESS, lid - 0.004), c, "hold the box while the tissue is pulled")
    w(L, 1.0, (*PRESS, lid + 0.08), c, "lift off")
    view = Tilt(VIEW_ROT)
    w(L, 2.0, VIEW, c, "to the viewing pose beside the box", view)
    w(L, total - sum(x.duration for x in L), VIEW, c, "watch the next tissue standing in the slit", view)
    return {"left": L, "right": R}


# The viewing pose. Once the box is let go, the left hand goes to a pose 10 cm off the box's long side (TCP 10 cm
# above the table), pointing across the box (-y) and 30 deg down with the fingers closed, so that its wrist camera
# looks at the slit face-on (2.5 deg off the standing tab's middle). The top camera looks along the slit and sees a
# standing tab edge-on. The hand stays 9 cm clear of the box. yam_arena's Waypoint carries only a down grip's yaw, so
# Tilt (a float that also carries a full TCP rotation) and an in-memory patch of yam_arena.demo.grip_rotation pass
# the tilted rotation to the planner. Nothing here touches the scene or the scoring.
VIEW = (XB, YB + BY / 2 + 0.100, TOP + 0.100)
_EL = math.radians(30.0)
VIEW_ROT = np.column_stack([(1.0, 0.0, 0.0), (0.0, -math.sin(_EL), math.cos(_EL)),
                            (0.0, -math.cos(_EL), -math.sin(_EL))])


class Tilt(float):
    """A Waypoint yaw that also carries a full TCP rotation (used by the patched grip_rotation below)."""

    def __new__(cls, rot):
        self = super().__new__(cls, 0.0)
        self.rot = np.asarray(rot, dtype=float)
        return self


def _patch_grip_rotation() -> None:
    import yam_arena.demo as _demo

    if getattr(_demo, "_tilt_patch", False):
        return
    base = _demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot.copy() if rot is not None else base(yaw)

    _demo.grip_rotation = grip_rotation
    _demo._tilt_patch = True


_patch_grip_rotation()


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built states (one trial each, fed frame by frame)."""
    global _TEST_TCPS
    r1, r2 = sk.rest_nodes(TISSUE_1), sk.rest_nodes(TISSUE_2)
    o = np.array([XB, YB, TOP])

    def on_the_tray(p, dx=0.0):  # the tissue lying crumpled flat in the tray
        q = p - p.mean(0)
        q = q[:, [0, 1, 2]] * np.array([1.0, 0.0, 0.0]) + np.c_[np.zeros(len(q)), (q[:, 1] + q[:, 2]) * 0.8,
                                                             np.zeros(len(q))]
        return q + np.array([TRX + dx, TRY, TRAY_TOP + 0.004])

    def presented(p, up=0.016):  # the next tissue drawn up `up` along its path through the slit
        q = p - o
        lead = q[:, 1] < 0.0035
        q[lead, 2] += up
        near = (q[:, 1] >= 0.0035) & (q[:, 1] < 0.007 + up)  # what followed it now runs up the slit too
        q[near, 1] = 0.003
        q[near, 2] = ZT - 0.001 + up - (q[near, 1] - 0.003)
        q[:, 1] = np.where(q[:, 1] > 0.007 + up, q[:, 1] - up, q[:, 1])
        return q + o

    def state(p1, p2, box_dz=0.0, box_tilt=0.0, box_dxy=(0.0, 0.0), box_yaw=0.0, moving=False):
        h, y = math.radians(box_tilt) / 2, math.radians(box_yaw) / 2  # tilt about x, then turn about z
        qt, qy = (math.sin(h), 0.0, 0.0, math.cos(h)), (0.0, 0.0, math.sin(y), math.cos(y))
        rb = mk.quat_to_mat(qy) @ mk.quat_to_mat(qt)
        qb = mk.mat_to_quat(rb)
        bpos = np.array([XB + box_dxy[0], YB + box_dxy[1], TOP + box_dz])
        st = kit.fake_states(moved={"box": (tuple(bpos), tuple(qb))})
        v = np.zeros_like(p1)
        if moving:
            v[:, 2] = -0.05
        # the next tissue sits in the box and moves with it
        st["tissue_1"], st["tissue_2"] = sk.fake(TISSUE_1, p1, v), sk.fake(TISSUE_2, (p2 - o) @ rb.T + bpos)
        return st

    def lin(a, b, n=4):
        return [a + (b - a) * k / n for k in range(n + 1)]

    out_held = r1 + np.array([0.0, 0.0, 0.110])  # pulled straight up, still in the jaws
    tray = on_the_tray(r1)
    up2 = presented(r2)
    on_lid = r1 - o + np.array([0.0, -0.040, 0.0])
    on_lid[:, 2] = ZT + 0.003
    on_lid += o
    tcp_tab = [(XB, YB, TOP + ZT + 0.13)]
    cases = {  # name: (frames [(t1, t2, kwargs)], TCPs at the end, score)
        "start: top tissue standing in the slit": ([(r1, r2, {})], [], 0.0),
        "demo: out, on the tray, next one standing 16 mm, box held": (
            [(r1, r2, {}), (out_held, up2, {}), (tray, up2, {})], [], 1.0),
        "out and on the tray but the next fell back into the box": ([(r1, r2, {}), (tray, r2, {})], [], 0.5),
        "out, dropped on the lid, next one ready": ([(r1, r2, {}), (on_lid, up2, {})], [], 0.5 + 0.25),
        "out and still held in the jaws over the box": ([(r1, r2, {}), (out_held, up2, {})], tcp_tab, 0.375),
        "on the tray, still settling": ([(r1, r2, {}), (tray, up2, {"moving": True})], [], 0.5),
        "pulled half out, end still in the box": ([(r1, r2, {}), (r1 + np.array([0.0, -0.02, 0.005]), up2, {})],
                                                   [], 0.0),
        "brief lift: box rode up 6 mm with the sheet, then settled on its spot": (
            [(r1, r2, {}), (out_held, up2, {"box_dz": 0.006}), (tray, up2, {})], [], 1.0),
        "brief tilt: box tilted 6 deg mid-pull, then settled on its spot": (
            [(r1, r2, {}), (out_held, up2, {"box_tilt": 6.0}), (tray, up2, {})], [], 1.0),
        "box rode up 13 mm and tilted 8.5 deg, fell back onto its spot": (
            [(r1, r2, {}), (out_held, up2, {"box_dz": 0.013, "box_tilt": 8.5}), (tray, up2, {})], [], 1.0),
        "brief lift, but the box settled 3 cm off its spot": (
            [(r1, r2, {}), (out_held, up2, {"box_dz": 0.006}), (tray, up2, {"box_dxy": (0.0, -0.03)})], [], 0.0),
        "box dragged 3 cm off its spot": ([(r1, r2, {}), (tray, up2, {"box_dxy": (0.0, -0.03)})], [], 0.0),
        "box left turned 15 deg on its spot": ([(r1, r2, {}), (tray, up2, {"box_yaw": 15.0})], [], 0.0),
        "box left leaning 8 deg (propped up)": ([(r1, r2, {}), (tray, up2, {"box_tilt": 8.0, "box_dz": 0.01})],
                                                  [], 0.0),
        "tips over: box on its side (90 deg) at the end": (
            [(r1, r2, {}), (out_held, up2, {"box_tilt": 90.0, "box_dz": 0.06}), (tray, up2,
                                                                               {"box_tilt": 90.0, "box_dz": 0.06})],
            [], 0.0),
        "tips over (60 deg), then set back upright on its spot (latched)": (
            [(r1, r2, {}), (out_held, up2, {"box_tilt": 60.0}), (tray, up2, {})], [], 0.0),
        "box carried 6 cm up off the table with the sheet, then dropped back on its spot (latched)": (
            [(r1, r2, {}), (out_held, up2, {"box_dz": 0.06}), (tray, up2, {})], [], 0.0),
        "box knocked off the table": ([(r1, r2, {}), (tray, up2, {"box_dz": -0.30})], [], 0.0),
        "both tissues pulled out (pinched together), both on the tray": (
            [(r1, r2, {}), (tray, on_the_tray(r2, 0.01), {})], [], 0.0),
        "next one dragged out, then stuffed back in (latched)": (
            [(r1, r2, {}), (tray, on_the_tray(r2, 0.01), {}), (tray, up2, {})], [], 0.0),
    }
    try:
        for name, (frames, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for p1, p2, kw in frames[:-1]:
                score(state(p1, p2, **kw))
            _TEST_TCPS = tcps
            p1, p2, kw = frames[-1]
            got = score(state(p1, p2, **kw))
            assert abs(got - want) < 1e-9, (name, got, want, first_out(state(p1, p2, **kw)),
                                            next_height(state(p1, p2, **kw)))
            assert success(state(p1, p2, **kw)) == (want >= 1.0), name
        for name, key in (("tips over (60 deg), then set back upright on its spot (latched)", "tipped"),
                          ("box carried 6 cm up off the table with the sheet, then dropped back on its spot (latched)",
                           "off_table"), ("box knocked off the table", "off_table"),
                          ("both tissues pulled out (pinched together), both on the tray", "two_out")):
            mk.new_episode()
            _TEST_TCPS = []
            for p1, p2, kw in cases[name][0]:
                score(state(p1, p2, **kw))
            assert _mem()[key] and sum(_mem().values()) == 1, (name, _mem())  # zeroed for the right reason
        mk.new_episode()
        _TEST_TCPS = []
        score(state(out_held, up2, box_tilt=60.0))
        mk.new_episode()
        assert score(state(tray, up2)) == 1.0, "a new trial forgets the last one's tip-over"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n = self_test()
    print(f"scoring: {n} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
