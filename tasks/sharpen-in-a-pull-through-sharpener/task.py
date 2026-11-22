"""On the station table: a chef's knife standing edge-down in a low wooden dock on the right, and a small pull-through
knife sharpener on a wet plastic cutting board in front of the robot. The sharpener has a handle on its left end and
two V-slots: a coarse carbide slot (dark) and, further right, a fine ceramic slot (white).

Sharpen the knife and put it back in its dock. The manufacturer's instructions, not stated in the instruction: hold the
sharpener's handle firmly so it cannot slip; set the heel of the blade in the coarse slot first and pull the knife
toward you from heel to tip, a few times (3 to 5), then do the same in the fine slot; always pull, never saw back and
forth. The sharpener weighs 100 g and stands on slippery feet on a wet board, while the abrasive grips the blade, so a
pull drags the sharpener along unless the other arm holds its handle.

Scored each step from poses and the knife-on-sharpener contact force: each pass through a slot (how far the edge
travelled from heel toward tip while pressed into the slot, in which direction, and whether the sharpener stayed put),
where the holding hand is, and where the knife ends up.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import kitchenkit as kk  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Sharpen the chef's knife with the pull-through sharpener, then put the knife back in its dock."

STEEL = (0.66, 0.67, 0.70)
BLACK = (0.08, 0.08, 0.09)
DARK_WOOD = (0.36, 0.24, 0.14)
BODY_C = (0.20, 0.30, 0.55)
COARSE_C, FINE_C = (0.15, 0.15, 0.16), (0.95, 0.95, 0.93)
BOARD_C = (0.92, 0.92, 0.90)

# -- the chef's knife --------------------------------------------------------------------------------------------------
# Knife frame: origin at the heel of the edge, x along the edge toward the tip, z up the blade toward the spine,
# y the blade's normal. The edge is straight for 150 mm and then rises 30 mm over the last 50 mm to the point.
BLADE_T, BLADE_H, HEEL_L = 0.002, 0.045, 0.150
TIP_RUN, TIP_RISE = 0.050, 0.030
_TIP_A = math.atan2(TIP_RISE, TIP_RUN)
TIP_L = math.hypot(TIP_RUN, TIP_RISE)
EDGE_LEN = HEEL_L + TIP_L  # 208 mm
HANDLE_L, HANDLE_T, HANDLE_H, HANDLE_X0 = 0.105, 0.016, 0.024, -0.020
HANDLE_C = (HANDLE_X0 - HANDLE_L / 2, 0.0, BLADE_H - HANDLE_H / 2)
SHELL = 0.002  # the handle is a hollow shell, so the knife balances at its bolster as a real one does
BOLSTER = (-0.010, 0.020, 0.021)  # centre x, length, height
KNIFE_GRASP = np.array([-0.020, 0.0, 0.034])  # the demo pinches bolster and handle front from above (knife frame)
BLADE_CONTACT = (0.001, 0.0)
GRIP_CONTACT = (0.005, 0.0)  # handles: a 5 mm contact offset keeps tools from creeping in the YAM pinch
BLADE_FRICTION = (0.50, 0.45)  # static, dynamic: dry steel
_tip_c = np.array([HEEL_L + TIP_RUN / 2, 0.0, TIP_RISE / 2]) + 0.0075 * np.array([-math.sin(_TIP_A), 0.0, math.cos(_TIP_A)])
KNIFE_GEOMS = [
    kk.Geom("box", (HEEL_L / 2, 0.0, BLADE_H / 2), (HEEL_L, BLADE_T, BLADE_H), contact=BLADE_CONTACT,
            friction=BLADE_FRICTION),
    kk.Geom("box", tuple(_tip_c), (TIP_L, BLADE_T, 0.015), kk.quat_axis((0, 1, 0), -_TIP_A), contact=BLADE_CONTACT,
            friction=BLADE_FRICTION),
    kk.Geom("box", (HEEL_L + 0.020, 0.0, BLADE_H - 0.005), (0.040, BLADE_T, 0.010), contact=BLADE_CONTACT,
            friction=BLADE_FRICTION),
    kk.Geom("box", (0.1635, 0.0, 0.0260), (0.029, BLADE_T, 0.0180), color=STEEL, contact=BLADE_CONTACT,
            friction=BLADE_FRICTION),
    kk.Geom("box", (BOLSTER[0], 0.0, BLADE_H - BOLSTER[2] / 2), (BOLSTER[1], HANDLE_T, BOLSTER[2]), color=STEEL,
            contact=GRIP_CONTACT),
    kk.Geom("box", (HANDLE_C[0], 0.0, HANDLE_C[2] + (HANDLE_H - SHELL) / 2), (HANDLE_L, HANDLE_T, SHELL), color=BLACK,
            contact=GRIP_CONTACT),
    kk.Geom("box", (HANDLE_C[0], 0.0, HANDLE_C[2] - (HANDLE_H - SHELL) / 2), (HANDLE_L, HANDLE_T, SHELL), color=BLACK,
            contact=GRIP_CONTACT),
    kk.Geom("box", (HANDLE_C[0], (HANDLE_T - SHELL) / 2, HANDLE_C[2]), (HANDLE_L, SHELL, HANDLE_H - 2 * SHELL),
            color=BLACK, contact=GRIP_CONTACT),
    kk.Geom("box", (HANDLE_C[0], -(HANDLE_T - SHELL) / 2, HANDLE_C[2]), (HANDLE_L, SHELL, HANDLE_H - 2 * SHELL),
            color=BLACK, contact=GRIP_CONTACT),
]
TOOL_RIGID = {"solver_position_iteration_count": 32, "solver_velocity_iteration_count": 4}
KNIFE_MASS = 0.20


def _groove(length, width, depth, bottom_w, top_w, z0, color=None):
    """A V-groove along x: two plates leaning outward (inner faces from the bottom edge to the top edge) and solid
    blocks either side; the groove's bottom is at z0."""
    va = math.atan2((top_w - bottom_w) / 2, depth)
    return [*[kk.Geom("box", (0.0, sd * ((bottom_w + top_w) / 4 + 0.004 / math.cos(va)), z0 + depth / 2),
                      (length, 0.008, depth / math.cos(va)), kk.quat_axis((1, 0, 0), -sd * va), color=color)
              for sd in (1, -1)],
            *[kk.Geom("box", (0.0, sd * (width / 2 - 0.006), z0 + depth / 2), (length, 0.012, depth), color=color)
              for sd in (1, -1)]]


# the dock: a 5 cm wooden stand with a V-groove along its top; the knife stands in it edge-down, handle toward the robot
DOCK_H, DOCK_SLOT_W, DOCK_SLOT_D, DOCK_W, DOCK_L, DOCK_TOP_W = 0.050, 0.004, 0.012, 0.050, 0.230, 0.016
KNIFE_POS = (0.30, -0.24, TOP + DOCK_H + 0.0005)  # the heel of the edge, resting on the groove's floor
DOCK_XY = (KNIFE_POS[0] - 0.005 + DOCK_L / 2, KNIFE_POS[1])  # the dock starts 5 mm behind the heel
DOCK = kk.part("dock", (*DOCK_XY, TOP), [
    kk.Geom("box", (0.0, 0.0, DOCK_H / 2), (DOCK_L, DOCK_W, DOCK_H)),
    *_groove(DOCK_L, DOCK_W, DOCK_SLOT_D, DOCK_SLOT_W, DOCK_TOP_W, DOCK_H),
], mass=None, color=DARK_WOOD, contact=(0.002, 0.0))
KNIFE = kk.part("knife", KNIFE_POS, KNIFE_GEOMS, mass=KNIFE_MASS, color=STEEL, rigid=TOOL_RIGID)

# -- the wet cutting board (static, slippery) ---------------------------------------------------------------------------
BOARD_XY, BOARD = (0.36, 0.0), (0.24, 0.34, 0.015)
BOARD_TOP = TOP + BOARD[2]
BOARD_PART = kk.part("board", (*BOARD_XY, TOP), [kk.Geom("box", (0.0, 0.0, BOARD[2] / 2), BOARD, color=BOARD_C)],
                     mass=None, color=BOARD_C, friction=(0.05, 0.05), contact=(0.002, 0.0))

# -- the sharpener (frame: centre of its feet's underside; long axis along y, slots along x) ----------------------------
FOOT_H = 0.004
BASE = (0.060, 0.220, 0.020)
BASE_TOP = FOOT_H + BASE[2]  # the slots' floor (the abrasive), in the sharpener frame
SH_HANDLE_C, SH_HANDLE = (0.0, 0.065, BASE_TOP + 0.0175), (0.035, 0.080, 0.035)
SLOT_L, SLOT_D, SLOT_W, SLOT_TOP_W, HOUSING_W = 0.050, 0.018, 0.004, 0.014, 0.034
SLOTS = {"coarse": -0.035, "fine": -0.075}  # slot centre lines (sharpener y)
SLOT_X = 0.0  # the slots' centre (sharpener x)
SH_XY = (0.40, 0.0)
SH_POS = (*SH_XY, BOARD_TOP + 0.0005)
SH_MASS = 0.10
SH_GEOMS = [
    *[kk.Geom("box", (sx * 0.022, sy * 0.095, FOOT_H / 2), (0.012, 0.020, FOOT_H), color=BLACK, friction=(0.10, 0.10))
      for sx in (1, -1) for sy in (1, -1)],
    kk.Geom("box", (0.0, 0.0, FOOT_H + BASE[2] / 2), BASE, color=BODY_C, friction=(0.90, 0.80)),
    kk.Geom("box", SH_HANDLE_C, SH_HANDLE, color=BODY_C, contact=GRIP_CONTACT, friction=(0.9, 0.8)),
]
for _k, _y in SLOTS.items():
    SH_GEOMS += [kk.Geom(g.kind, (g.center[0], g.center[1] + _y, g.center[2]), g.size, g.quat_xyzw, color=BODY_C,
                         friction=(0.6, 0.5))
                 for g in _groove(SLOT_L, HOUSING_W, SLOT_D, SLOT_W, SLOT_TOP_W, BASE_TOP)]
    SH_GEOMS.append(kk.Geom("box", (SLOT_X, _y, BASE_TOP + 0.0005), (SLOT_L, SLOT_W, 0.001),
                            color=COARSE_C if _k == "coarse" else FINE_C, collide=False))
_RIGID = {"solver_position_iteration_count": 32, "solver_velocity_iteration_count": 4}
SHARPENER = kk.part("sharpener", SH_POS, SH_GEOMS, mass=SH_MASS, color=BODY_C, rigid=_RIGID, contact=(0.002, 0.0))
HOUSINGS = {k: ((SLOT_X, y, BASE_TOP + SLOT_D / 2), (SLOT_L, HOUSING_W, SLOT_D)) for k, y in SLOTS.items()}

OBJECTS = [DOCK, KNIFE, BOARD_PART, SHARPENER]
kk.watch_contact("knife", "sharpener")  # the scorer reads the knife-on-sharpener contact force each step

# -- scoring -----------------------------------------------------------------------------------------------------------
TOUCH = 0.5  # N: the blade is on the abrasive
PRESS = 1.5  # N: pressed into the slot (a sharpening contact)
PRESS_SHARE = 0.80  # of a pass's touching steps
STROKE_MIN = 0.120  # m: the edge travels at least this far from heel toward tip through the slot while pressed
PUSH_TOL = 0.010  # m: backward (tip-to-heel) travel tolerated within a pass (jitter)
SAW = 0.020  # m: more backward travel than this in one contact is sawing back and forth
GAP_STEPS = 3  # control steps off the abrasive that still belong to the same pass
STILL = 0.020  # m: the sharpener moved no more than this during a pass (held, it gives about 1 cm)
PASSES = 3  # passes needed per slot
HAND_KEEP = 0.015  # m: the holding hand's pads no closer than this to a slot housing
SH_TIP = 20.0  # deg: the sharpener tipped
ALIGN = 0.94  # cos: the blade along the slot and upright (about 20 deg)
DOCK_UPRIGHT, DOCK_HEADING = 30.0, 15.0  # deg: blade from upright; point from straight away from the robot
DOCK_HEEL_RANGE = 0.080  # m: the heel within the groove's first 8 cm (from the dock's near end)
DOCK_LIFT = 0.006  # m: the heel no higher than this above the groove floor
KNIFE_NEAR = 0.020  # m: a pad this close to the handle has the knife (for the drop rule)
_HANDLE_BOX = ((HANDLE_C[0] + 0.01, 0.0, HANDLE_C[2]), (HANDLE_L + 0.02, HANDLE_T, HANDLE_H))
_S: dict = {}
_FALL = kk.FallWatch(fall=0.03, steps=10)


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, cur=None, passes=[], coarse=0, fine=0, fine_early=0,
              saw=False, hand=False, tipped=False, dropped=False, returned=False)
    _FALL.reset()


_reset()
kk.RESET_CALLBACKS.append(_reset)


def slot_of(knife, sh):
    """(slot name, s) if the blade lies along a slot, upright, with its edge down in the groove; s is how far along the
    edge (m from the heel) the slots' centre line crosses it. (None, None) otherwise."""
    (p, R), (q, Rs) = knife, sh
    heel = Rs.T @ (p - q)
    ax, up = Rs.T @ R[:, 0], Rs.T @ R[:, 2]
    if ax[0] < ALIGN or up[2] < ALIGN:
        return None, None
    s = (SLOT_X - heel[0]) / ax[0]
    if not -0.010 <= s <= EDGE_LEN:
        return None, None
    pt = heel + ax * min(max(s, 0.0), HEEL_L)
    for k, y in SLOTS.items():
        if abs(pt[1] - y) <= SLOT_TOP_W / 2 and pt[2] <= BASE_TOP + SLOT_D:
            return k, float(s)
    return None, None


def _judge(c) -> dict:
    travel = c["s_max"] - c["s_min"] if c["press"] else 0.0
    share = c["press"] / max(c["touch"], 1)
    ok = travel >= STROKE_MIN and share >= PRESS_SHARE and c["back"] <= PUSH_TOL and c["shift"] <= STILL
    return {"slot": c["slot"], "travel": round(travel, 4), "press_share": round(share, 2), "back": round(c["back"], 4),
            "sharpener_moved": round(c["shift"], 4), "valid": bool(ok)}


def _close(c) -> None:
    j = _judge(c)
    _S["passes"].append(j)
    if j["valid"]:
        if j["slot"] == "coarse":
            _S["coarse"] += 1
        elif _S["coarse"] >= PASSES:
            _S["fine"] += 1
        else:
            _S["fine_early"] += 1


def _in_dock(knife) -> bool:
    p, R = knife
    near_end = DOCK_XY[0] - DOCK_L / 2
    heading = math.degrees(math.atan2(abs(R[1, 0]), R[0, 0]))
    return (near_end <= p[0] <= near_end + DOCK_HEEL_RANGE and abs(p[1] - DOCK_XY[1]) <= DOCK_TOP_W / 2
            and p[2] - (TOP + DOCK_H) <= DOCK_LIFT and kk.tilt_deg(R) <= DOCK_UPRIGHT and heading <= DOCK_HEADING)


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    knife = kk.pose_of("knife", KNIFE_POS)
    sh = kk.pose_of("sharpener", SH_POS)
    force = float(kk.STATE.contacts.get(("knife", "sharpener"), 0.0))
    slot, s = slot_of(knife, sh)
    c = _S["cur"]
    if slot is not None and force >= TOUCH:
        if c is not None and c["slot"] != slot:
            _close(c)
            c = None
        if c is None:
            c = dict(slot=slot, s_prev=s, s_min=s, s_max=s, back=0.0, touch=0, press=0, shift=0.0, gap=0,
                     sh0=np.array(sh[0]))
        if s < c["s_prev"]:
            c["back"] += c["s_prev"] - s
        c["s_prev"] = s
        c["touch"] += 1
        if force >= PRESS:
            c["press"] += 1
            c["s_min"], c["s_max"] = min(c["s_min"], s), max(c["s_max"], s)
        c["shift"] = max(c["shift"], float(np.linalg.norm((sh[0] - c["sh0"])[:2])))
        c["gap"] = 0
        if c["back"] > SAW:
            _S["saw"] = True
    elif c is not None:
        c["gap"] += 1
        if c["gap"] > GAP_STEPS:
            _close(c)
            c = None
    _S["cur"] = c
    left = kk.ARMS[0]
    if any(kk.near(left, cc, sz, sh, HAND_KEEP) for cc, sz in HOUSINGS.values()):
        _S["hand"] = True
    if kk.tilt_deg(sh[1]) > SH_TIP:
        _S["tipped"] = True
    handled = any(kk.near(a, *_HANDLE_BOX, knife, KNIFE_NEAR) for a in kk.STATE.fingers)
    if _FALL.update(knife[0][2], handled):
        _S["dropped"] = True
    _S["returned"] = _in_dock(knife) and not any(kk.near(a, *_HANDLE_BOX, knife, 0.010) for a in kk.STATE.fingers)
    coarse, fine = _S["coarse"], _S["fine"]
    if c is not None:  # a pass still in progress counts if it already qualifies
        j = _judge(c)
        if j["valid"]:
            if j["slot"] == "coarse":
                coarse += 1
            elif coarse >= PASSES:
                fine += 1
    _S["last"] = {"force": round(force, 3), "slot": slot, "s": None if s is None else round(s, 4),
                  "coarse_passes": coarse, "fine_passes": fine, "fine_before_coarse": _S["fine_early"],
                  "passes": list(_S["passes"]), "sawing": _S["saw"], "hand_at_slots": _S["hand"],
                  "sharpener_tipped": _S["tipped"], "knife_dropped": _S["dropped"], "knife_in_dock": _S["returned"],
                  "sharpener_shift": round(float(np.linalg.norm((sh[0] - np.array(SH_POS))[:2])), 4)}
    if _S["saw"] or _S["hand"] or _S["tipped"] or _S["dropped"]:
        res = (0.0, False)
    else:
        sc = 0.5 * min(coarse, PASSES) / PASSES + 0.5 * min(fine, PASSES) / PASSES
        done = coarse >= PASSES and fine >= PASSES and _S["returned"]
        if not _S["returned"]:
            sc = min(sc, 0.75)
        res = (round(sc, 3), done)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """Three valid passes in the coarse slot, then three in the fine slot, and the knife back in its dock. A valid pass:
    the edge travels at least 12 cm from heel toward tip through the slot, pressed in (>= 1.5 N on 80% of its touching
    steps), with no more than 1 cm of backward travel, while the sharpener moves no more than 2 cm."""
    return _update(objects)[1]


def score(objects) -> float:
    """1/6 per valid coarse pass and per valid fine pass made after three coarse ones (up to three each), capped at 0.75
    while the knife is not back in its dock; 0 for sawing (more than 2 cm of tip-to-heel travel in one contact), the
    holding hand within 1.5 cm of a slot, the sharpener tipped past 20 deg, or the knife dropped."""
    return _update(objects)[0]


def report() -> dict:
    return dict(_S.get("last", {}))
