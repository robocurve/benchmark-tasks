"""On the station table: a chef's knife after use, its bolster resting on a small knife rest on the wet steel draining
sheet, the blade lying flat off the rest with its edge facing away from the robot; a folded towel pad on a stand to the
right.

Wipe the knife clean. Knife-care practice, not stated in the instruction: hold the knife by its handle with the edge
facing away, wipe the flat of the blade from the heel toward the tip from the spine side, and never run the towel across
the cutting edge. The towel's passes count only while a gripper holds the knife by its handle, so one arm holds it
while the other wipes (left on its rest, the knife does not slide away under the towel: the hold is the rule, not a
physical necessity).

Scored each step from poses and the towel-on-blade contact force: which part of the blade the pressed towel has passed
over, while the knife was held by its handle, and in which direction, how near any finger came to the cutting edge,
whether the towel crossed the edge, and whether the knife was dropped.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import kitchenkit as kk  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Wipe the knife clean with the towel pad."

STEEL, BLADE_C, HANDLE_C = (0.75, 0.76, 0.79), (0.82, 0.83, 0.86), (0.12, 0.10, 0.09)
REST_C, TOWEL_C, STAND_C, SMEAR_C = (0.45, 0.30, 0.20), (0.92, 0.92, 0.88), (0.30, 0.45, 0.70), (0.55, 0.42, 0.25)

# -- the wet steel draining sheet --------------------------------------------------------------------------------------
SHEET_XY, SHEET = (0.30, 0.0), (0.26, 0.40, 0.003)
SHEET_TOP = TOP + SHEET[2]
SHEET_PART = kk.part("sheet", (*SHEET_XY, TOP), [kk.Geom("box", (0.0, 0.0, SHEET[2] / 2), SHEET, color=STEEL)],
                     mass=None, color=STEEL, friction=(0.08, 0.08), contact=(0.002, 0.0))

# -- the knife (frame: the blade's heel on the handle's axis; handle along +y, blade along -y, edge toward +x) ---------
BLADE_L, BLADE_W, BLADE_T = 0.160, 0.035, 0.002
SPINE_X = -0.010  # the spine's line (the blade's back), x in the knife frame
EDGE_X = SPINE_X + BLADE_W  # the cutting edge's line
HANDLE_L, HANDLE_W, HANDLE_T = 0.100, 0.022, 0.016
KNIFE_GEOMS = [
    kk.Geom("box", ((SPINE_X + EDGE_X) / 2, -BLADE_L / 2, 0.0), (BLADE_W, BLADE_L, BLADE_T), color=BLADE_C),
    kk.Geom("box", (0.0, HANDLE_L / 2, 0.0), (HANDLE_W, HANDLE_L, HANDLE_T), color=HANDLE_C, friction=(1.0, 0.9)),
    # food smeared along the blade (visual only)
    *[kk.Geom("box", (SPINE_X + 0.012 + 0.006 * (i % 2), -0.020 - 0.035 * i, BLADE_T / 2 + 0.0003), (0.010, 0.012, 0.0006),
              color=SMEAR_C, collide=False) for i in range(4)],
]
REST_XY, REST, REST_H = (0.30, 0.012), (0.040, 0.025), 0.060  # under the bolster end of the handle (y 0..0.025)
REST_PART = kk.part("knife_rest", (*REST_XY, SHEET_TOP), [kk.Geom("box", (0.0, 0.0, REST_H / 2), (REST[0], REST[1], REST_H))],
                    mass=None, color=REST_C, friction=(0.5, 0.4), contact=(0.002, 0.0))
_RIGID = {"solver_position_iteration_count": 24, "solver_velocity_iteration_count": 4}
KNIFE_POS = (REST_XY[0], 0.0, SHEET_TOP + REST_H + HANDLE_T / 2 + 0.0005)
KNIFE = kk.part("knife", KNIFE_POS, KNIFE_GEOMS, mass=0.15, color=BLADE_C, friction=(0.6, 0.5), rigid=_RIGID,
                contact=(0.002, 0.0))

# -- the towel pad on its stand -----------------------------------------------------------------------------------------
TOWEL = (0.025, 0.030, 0.040)  # x, y, z
STAND_XY, STAND_H = (0.18, -0.24), 0.030
TOWEL_POS = (*STAND_XY, TOP + STAND_H + 0.0005)
TOWEL_PART = kk.part("towel", TOWEL_POS, [kk.Geom("box", (0.0, 0.0, TOWEL[2] / 2), TOWEL, color=TOWEL_C)],
                     mass=0.03, color=TOWEL_C, friction=(0.8, 0.7), rigid=_RIGID, contact=(0.002, 0.0))
STAND_PART = kk.part("stand", (*STAND_XY, TOP), [kk.Geom("box", (0.0, 0.0, STAND_H / 2), (0.05, 0.05, STAND_H))],
                     mass=None, color=STAND_C, friction=(0.5, 0.4), contact=(0.002, 0.0))

OBJECTS = [SHEET_PART, REST_PART, KNIFE, STAND_PART, TOWEL_PART]
kk.watch_contact("towel", "knife")  # the scorer reads the towel-on-blade contact force each step

# -- scoring -----------------------------------------------------------------------------------------------------------
WIPE_FORCE, WIPE_SPEED = 0.3, 0.005  # N, m/s: the towel pressed on the blade and moving along it
BINS = 16  # 1 cm bins along the blade, heel to tip
FULL_COVER, PART_COVER = 0.90, 0.50
EDGE_KEEP = 0.008  # m: no gripper pad this close to the cutting edge
DROP_FALL, DROP_STEPS = 0.05, 10
KNIFE_NEAR = 0.020
DT = 0.05
_KNIFE_C, _KNIFE_S = (0.0, (HANDLE_L - BLADE_L) / 2, 0.0), (BLADE_W + 0.02, HANDLE_L + BLADE_L, HANDLE_T)
_EDGE_C, _EDGE_S = (EDGE_X, -BLADE_L / 2, 0.0), (0.002, BLADE_L, BLADE_T)  # a strip along the cutting edge
_HANDLE_C, _HANDLE_S = (0.0, HANDLE_L / 2, 0.0), (HANDLE_W, HANDLE_L, HANDLE_T)
HOLD_DIST = 0.008  # m: a pad point of each finger this close to the handle, the handle between them (kk.holds)
_S: dict = {}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, bins=np.zeros(BINS, bool), to_tip=0.0, to_heel=0.0, prev_s=None,
              edge=False, crossed=False, dropped=False, unheld=False, fall=kk.FallWatch(DROP_FALL, DROP_STEPS))


_reset()
kk.RESET_CALLBACKS.append(_reset)


def towel_on_blade(knife, towel) -> dict:
    """The towel's footprint in the knife frame: its centre along the blade (s, from the heel toward the tip), its x
    extent, and its height over the blade's top face."""
    (p, R), (q, Rt) = knife, towel
    c = R.T @ (q + Rt @ np.array([0.0, 0.0, 0.0]) - p)  # the towel's underside centre, knife frame
    return {"s": float(-c[1]), "x_lo": float(c[0] - TOWEL[0] / 2), "x_hi": float(c[0] + TOWEL[0] / 2),
            "gap": float(c[2] - BLADE_T / 2)}


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    knife = kk.pose_of("knife", KNIFE_POS)
    towel = kk.pose_of("towel", TOWEL_POS)
    tb = towel_on_blade(knife, towel)
    force = float(kk.STATE.contacts.get(("towel", "knife"), 0.0))
    s = tb["s"]
    ds = 0.0 if _S["prev_s"] is None else s - _S["prev_s"]
    _S["prev_s"] = s
    on = force >= WIPE_FORCE and abs(tb["gap"]) <= 0.006 and -TOWEL[1] / 2 <= s <= BLADE_L + TOWEL[1] / 2
    held = any(kk.holds(a, _HANDLE_C, _HANDLE_S, knife, HOLD_DIST) for a in kk.STATE.fingers)
    if on and not held:
        _S["unheld"] = True  # a pass on a knife nobody holds by the handle: it does not count
    if on and held and abs(ds) / DT >= WIPE_SPEED:
        edges = np.linspace(0.0, BLADE_L, BINS + 1)
        mids = (edges[:-1] + edges[1:]) / 2
        _S["bins"] |= (mids >= s - TOWEL[1] / 2) & (mids <= s + TOWEL[1] / 2)
        if ds > 0:
            _S["to_tip"] += ds
        else:
            _S["to_heel"] += -ds
    if on and tb["x_hi"] > EDGE_X:  # pressed on the blade and reaching over the cutting edge
        _S["crossed"] = True
    if any(kk.near(a, _EDGE_C, _EDGE_S, knife, EDGE_KEEP) for a in kk.STATE.fingers):
        _S["edge"] = True
    handled = kk.touched_by_any(_KNIFE_C, _KNIFE_S, knife, KNIFE_NEAR)
    if _S["fall"].update(knife[0][2], handled):
        _S["dropped"] = True
    cover = float(_S["bins"].mean())
    heel_to_tip = _S["to_tip"] > _S["to_heel"]
    _S["last"] = {"force": round(force, 3), "towel_s": round(s, 4), "towel_x": [round(tb["x_lo"], 4), round(tb["x_hi"], 4)],
                  "towel_gap": round(tb["gap"], 4), "coverage": round(cover, 3), "heel_to_tip": bool(heel_to_tip),
                  "to_tip": round(_S["to_tip"], 4), "to_heel": round(_S["to_heel"], 4),
                  "knife_tilt": round(kk.tilt_deg(knife[1]), 2), "held": bool(held), "wiped_unheld": bool(_S["unheld"])}
    if _S["edge"] or _S["crossed"] or _S["dropped"]:
        res = (0.0, False)
    elif cover >= FULL_COVER and heel_to_tip:
        res = (1.0, True)
    elif cover >= PART_COVER:
        res = (0.5, False)
    elif cover > 0.0:
        res = (0.25, False)
    else:
        res = (0.0, False)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """The pressed towel has passed over at least 90% of the blade's length (1 cm bins, heel to tip), its travel along
    the blade mostly toward the tip, with no finger pad within 8 mm of the cutting edge, the towel never reaching over
    the edge while pressed on the blade, and the knife not dropped."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 the whole blade wiped heel to tip; 0.5 at least half the blade wiped (or all of it, tip to heel); 0.25 less
    than half; 0 if a finger came within 8 mm of the edge, the towel reached over the edge while pressed on the blade,
    the knife was dropped, or nothing was wiped."""
    return _update(objects)[0]


def report() -> dict:
    return {**_S.get("last", {}), "edge_breached": _S.get("edge"), "towel_crossed_edge": _S.get("crossed"),
            "drop_flag": _S.get("dropped")}
