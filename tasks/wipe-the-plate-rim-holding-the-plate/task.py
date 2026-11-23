"""On the station table: a plated main course on the wet stainless-steel pass (the food, a whole English muffin as a
stand-in, in the middle of a square porcelain plate), three round sauce smudges at the plate's rim (near side, right side,
and far-right at the rim band's inner corner, partly on the plate's flat middle), and a folded towel pad on a small stand to the right.

Clean up the plate's rim before it goes out. Plating standard: wipe the rim clean before a plate leaves the kitchen, and
do not touch the food. Wet porcelain on a wet steel pass slides when a towel is dragged across it, so the plate has to
be held still by its rim with one arm while the other wipes. The instruction does not state how.

Scored each step from poses and contact forces: how much of each smudge's area the pressed towel has slid over (every
smudge must be wiped over its whole area), how far the plate moved, and whether any part of either gripper or the towel
touched the food or came within 1 cm of it. The trial ends once all three smudges are clean, the plate is steady and
the towel has been off the plate (not touching it, 2 cm above the rim) for 1 s; the score counts clean smudges.
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

INSTRUCTION = "Clean up the plate's rim with the towel pad before the plate goes out to the guest."

PORCELAIN = (0.95, 0.95, 0.93)
SAUCE = (0.62, 0.08, 0.05)  # tomato ketchup, as in the real trials
MUFFIN = (0.80, 0.62, 0.38)
STEEL = (0.72, 0.73, 0.76)
TOWEL_C = (0.85, 0.85, 0.92)

# -- the wet steel pass (static, slippery) ------------------------------------------------------------------------------
PASS_XY, PASS = (0.24, 0.0), (0.30, 0.42, 0.040)
PASS_TOP = TOP + PASS[2]
PASS_PART = kk.part("pass_shelf", (*PASS_XY, TOP), [kk.Geom("box", (0.0, 0.0, PASS[2] / 2), PASS, color=STEEL)],
                    mass=None, color=STEEL, friction=(0.02, 0.02), contact=(0.002, 0.0))

# -- the plate (frame: centre of its underside): base, a flat rim band, a raised lip at the edge ------------------------
PLATE, PLATE_T = 0.236, 0.010
LIP_W, LIP_H = 0.016, 0.012
BAND = 0.037  # the flat rim band inside the lip, where the smudges are (the towel pad fits inside it: a wider pad
# landed on the lip, and the lip took the press and twisted the plate in the hold)
SMUDGE_R = PLATE / 2 - LIP_W - BAND / 2  # smudge centres from the plate's centre
SMUDGES = {"far": (SMUDGE_R * 0.7071, -SMUDGE_R * 0.7071), "right": (0.0, -SMUDGE_R), "near": (-SMUDGE_R, 0.0)}  # plate
# frame (x, y): all on the half away from the holding hand (a smudge on the far side brought the wiping arm into it)
SAUCE_D = 0.030  # m: each smudge is a round patch of sauce 3 cm across (spread through a 3 cm stencil in real trials)
_lz = PLATE_T + LIP_H / 2
SAUCE_TILE = 0.0025  # m: the sauce's tile size, equal to the scoring grid (GRID below)
_sn = int(round(SAUCE_D / 2 / SAUCE_TILE))
SAUCE_TILES = [(i * SAUCE_TILE, j * SAUCE_TILE) for i in range(-_sn, _sn + 1) for j in range(-_sn, _sn + 1)
               if i * i + j * j <= _sn * _sn]  # same order as _DISC below
N_SOLID = 5  # PLATE_GEOMS: the base and four lip walls come first, then the sauce tiles smudge by smudge
PLATE_GEOMS = [
    kk.Geom("box", (0.0, 0.0, PLATE_T / 2), (PLATE, PLATE, PLATE_T), color=PORCELAIN),
    *[kk.Geom("box", (sx * (PLATE - LIP_W) / 2, 0.0, _lz), (LIP_W, PLATE, LIP_H), color=PORCELAIN, friction=(0.9, 0.8))
      for sx in (1, -1)],
    *[kk.Geom("box", (0.0, sy * (PLATE - LIP_W) / 2, _lz), (PLATE - 2 * LIP_W, LIP_W, LIP_H), color=PORCELAIN,
              friction=(0.9, 0.8)) for sy in (1, -1)],
    # the sauce, drawn as 2.5 mm tiles, one over each scoring sample point (visual only): a tile disappears once its
    # point is wiped, and a smudge's last tiles once it counts as clean, so the cameras see the rim come clean
    *[kk.Geom("box", (sx + dx, sy + dy, PLATE_T + 0.0005), (SAUCE_TILE, SAUCE_TILE, 0.001), color=SAUCE, collide=False)
      for sx, sy in SMUDGES.values() for dx, dy in SAUCE_TILES],
]
PLATE_XY = (0.24, 0.0)
PLATE_POS = (*PLATE_XY, PASS_TOP + 0.0005)
_RIGID = {"solver_position_iteration_count": 16, "solver_velocity_iteration_count": 4}
PLATE_PART = kk.part("plate", PLATE_POS, PLATE_GEOMS, mass=0.45, color=PORCELAIN, friction=(0.10, 0.10), rigid=_RIGID,
                     contact=(0.002, 0.0))

# -- the food (frame: centre of its underside): the stand-in used in the real trials, one whole English muffin, 9.5 cm
# across and 3 cm tall, centred on the plate. It collides and rests loose on the plate, so whatever touches it pushes
# it and the contact force is read. Fifteen box slats make the disc; the keep-out below is measured to every slat.
FOOD_D, FOOD_H, FOOD_MASS = 0.095, 0.030, 0.057
FOOD_GEOMS = kk.disk(FOOD_D / 2, FOOD_H, 0.0, k=15, color=MUFFIN)
FOOD_POS = (*PLATE_XY, PLATE_POS[2] + PLATE_T + 0.0005)
FOOD_PART = kk.part("food", FOOD_POS, FOOD_GEOMS, mass=FOOD_MASS, color=MUFFIN, friction=(0.8, 0.7), rigid=_RIGID,
                    contact=(0.002, 0.0))
FOOD_BOXES = [(np.asarray(g.center, float), np.asarray(g.size, float)) for g in FOOD_GEOMS]

# -- the towel pad on its stand (frame: centre of the pad's underside) --------------------------------------------------
TOWEL = (0.030, 0.030, 0.025)
STAND_XY, STAND_H = (0.15, -0.24), 0.030
TOWEL_POS = (*STAND_XY, TOP + STAND_H + 0.0005)
TOWEL_PART = kk.part("towel", TOWEL_POS, [kk.Geom("box", (0.0, 0.0, TOWEL[2] / 2), TOWEL, color=TOWEL_C)],
                     mass=0.03, color=TOWEL_C, friction=(0.8, 0.7), rigid=_RIGID, contact=(0.002, 0.0))  # a damp towel grips
# porcelain (at 0.3 / 0.25 it glided over the rim and an unheld plate crept only 5 mm)
STAND_PART = kk.part("stand", (*STAND_XY, TOP), [kk.Geom("box", (0.0, 0.0, STAND_H / 2), (0.05, 0.05, STAND_H),
                                                         color=STEEL)],
                     mass=None, color=STEEL, friction=(0.6, 0.5), contact=(0.002, 0.0))
OBJECTS = [PASS_PART, PLATE_PART, FOOD_PART, TOWEL_PART, STAND_PART]

# -- contact forces read each step -------------------------------------------------------------------------------------
kk.watch_contact("towel", "plate")  # the towel pressing on the plate
_CHAIN = "Geometry/base/link1/link2/link3/link4"  # each arm's links, under its prim in the env
ARM_LINKS = {"forearm link4": _CHAIN, "wrist link5": f"{_CHAIN}/link5", "gripper body": f"{_CHAIN}/link5/gripper",
             "finger tip_left": f"{_CHAIN}/link5/gripper/tip_left",
             "finger tip_right": f"{_CHAIN}/link5/gripper/tip_right"}
FOOD_WATCH = {"towel": "towel", **{f"{side} {name}": f"{prim}/{path}"
                                   for side, prim in (("left", "LeftArm"), ("right", "RightArm"))
                                   for name, path in ARM_LINKS.items()}}  # who -> the body whose push on the food is read
for _b in FOOD_WATCH.values():
    kk.watch_contact("food", _b)

# -- scoring -----------------------------------------------------------------------------------------------------------
PRESS = 1.2  # N: the towel presses on the plate with at least this force (firm contact for the rigid pad; the
# scripted wipe's force controller targets 1.2 N and holds a median of 1.3-1.45 N while rubbing)
RUB_SPEED = 0.010  # m/s: and moves at least this fast relative to the plate: it is wiping
GRID = 0.0025  # m: sample points on each smudge, a square grid over its whole disc (113 points per smudge)
PAD_GAP = 0.002  # m: a point is under the pad's pressed underside when inside its 3 x 3 cm footprint and this close to it
WIPE_TRAVEL = 0.010  # m: a point is wiped once the pressed, moving pad has slid this far over it in total (a step's
# travel counts for the points under the pad at both the start and the end of the step)
CLEAN_FRAC = 0.95  # a smudge is clean when at least this share of its points is wiped
STILL_MOVE, MAX_MOVE, MAX_YAW = 0.010, 0.030, 5.0  # m, m, deg: held still while wiped (from the first towel touch) / pushed off (from the start)
LIFT_MAX = 0.020
FOOD_TOUCH = 0.02  # N: any watched link of either arm, or the towel, pushing on the food this hard (peak in a step)
FOOD_MARGIN = 0.010  # m: finger pads and the towel's surface closer than this to any food slat
DT = 0.05
OFF_GAP, OFF_HOLD = 0.020, 1.0  # m, s: the towel off the plate (no contact, underside this far above the rim) this long

_n = int(round(SAUCE_D / 2 / GRID))
_DISC = np.array([(i * GRID, j * GRID) for i in range(-_n, _n + 1) for j in range(-_n, _n + 1) if i * i + j * j <= _n * _n])
assert GRID == SAUCE_TILE and np.allclose(_DISC, SAUCE_TILES)  # one sauce tile per sample point
SMUDGE_PTS = {k: np.column_stack([_DISC[:, 0] + sx, _DISC[:, 1] + sy, np.full(len(_DISC), PLATE_T)])
              for k, (sx, sy) in SMUDGES.items()}  # plate frame, on the rim's surface
_g = np.array(np.meshgrid(np.linspace(-TOWEL[0] / 2, TOWEL[0] / 2, 13), np.linspace(-TOWEL[1] / 2, TOWEL[1] / 2, 13),
                          np.linspace(0.0, TOWEL[2], 11), indexing="ij")).reshape(3, -1).T
TOWEL_SURF = _g[(np.abs(np.abs(_g[:, 0]) - TOWEL[0] / 2) < 1e-9) | (np.abs(np.abs(_g[:, 1]) - TOWEL[1] / 2) < 1e-9)
                | (np.abs(_g[:, 2]) < 1e-9) | (np.abs(_g[:, 2] - TOWEL[2]) < 1e-9)]  # towel frame, every 2.5 mm

# the previous version's rules, kept only as diagnostics in report() (not scored): a smudge counted as clean after 10
# steps of rubbing (0.8 N) with the towel's centre within 2 cm of its centre; the food check used a 7 x 7 x 4 cm box
_OLD_OVER, _OLD_RUB_STEPS, _OLD_PRESS = 0.020, 10, 0.8
_OLD_FOOD_C, _OLD_FOOD_S = (0.0, 0.0, PLATE_T + 0.02), (0.07, 0.07, 0.04)
_S: dict = {}
_VIS: dict = {"root": None, "hidden": set(), "error": None}  # the plate's prim path and the hidden sauce tiles


def _tile(stage, k: int, i: int):
    idx = N_SOLID + list(SMUDGES).index(k) * len(SAUCE_TILES) + i
    return stage.GetPrimAtPath(f"{_VIS['root']}/geometry/g{idx:02d}")


def _hide_tiles(wiped: dict) -> None:
    """Hide the sauce tile over each wiped point (visual only; scoring never depends on it)."""
    try:
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        if _VIS["root"] is None:
            roots = sorted(str(p.GetPath()) for p in stage.Traverse()
                           if p.GetName() == "plate" and p.GetChild("geometry").IsValid())
            _VIS["root"] = roots[0]
        for k, mask in wiped.items():
            for i in np.flatnonzero(mask):
                if (k, int(i)) not in _VIS["hidden"]:
                    UsdGeom.Imageable(_tile(stage, k, int(i))).MakeInvisible()
                    _VIS["hidden"].add((k, int(i)))
    except Exception as e:  # noqa: BLE001
        _VIS["error"] = repr(e)[:200]


def _show_tiles() -> None:
    if not _VIS["hidden"] or _VIS["root"] is None:
        _VIS["hidden"] = set()
        return
    try:
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        for k, i in _VIS["hidden"]:
            UsdGeom.Imageable(_tile(stage, k, i)).MakeVisible()
    except Exception as e:  # noqa: BLE001
        _VIS["error"] = repr(e)[:200]
    _VIS["hidden"] = set()


def _reset() -> None:
    _show_tiles()
    _S.update(step=None, result=(0.0, False), last={}, prev=None, max_move=0.0, max_yaw=0.0, lifted=False, wipe0=None,
              max_slide=0.0, travel={k: np.zeros(len(p)) for k, p in SMUDGE_PTS.items()},
              was_under={k: np.zeros(len(p), bool) for k, p in SMUDGE_PTS.items()}, food=False, food_at=None,
              food_force={}, gap=(np.inf, None, None), old_rub={k: 0 for k in SMUDGES}, old_food=None, off_s=0.0)


_reset()
kk.RESET_CALLBACKS.append(_reset)


def _peak(pair) -> float:
    """A watched pair's largest contact force over the step's physics substeps (N)."""
    peaks = getattr(kk.STATE, "peaks", None) or {}
    return float(peaks.get(pair, kk.STATE.contacts.get(pair, 0.0)))


def food_gap(pts, food_pose) -> float:
    """The smallest distance from points (n, 3) to any slat of the food at ``food_pose`` (m)."""
    return min(float(kk.box_dist(pts, c, s, food_pose).min()) for c, s in FOOD_BOXES)


def under_pad(pts, plate, towel) -> np.ndarray:
    """Which rim points (plate frame) lie under the towel pad's pressed underside: inside its footprint, within PAD_GAP."""
    loc = (plate[0] + pts @ plate[1].T - towel[0]) @ towel[1]
    return ((np.abs(loc[:, 0]) <= TOWEL[0] / 2) & (np.abs(loc[:, 1]) <= TOWEL[1] / 2)
            & (np.abs(loc[:, 2]) <= PAD_GAP))


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    plate = kk.pose_of("plate", PLATE_POS)
    towel = kk.pose_of("towel", TOWEL_POS)
    food = kk.pose_of("food", FOOD_POS)
    force = float(kk.STATE.contacts.get(("towel", "plate"), 0.0))
    rel = (towel[0] - plate[0]) @ plate[1]  # the towel's frame in the plate's frame
    travel = 0.0 if _S["prev"] is None else float(np.linalg.norm((rel - _S["prev"])[:2]))
    speed = travel / DT
    _S["prev"] = rel
    wiping = force >= PRESS and speed >= RUB_SPEED
    off = force < 0.05 and float(rel[2]) - PLATE_T >= OFF_GAP  # rel: the towel's underside centre, plate frame
    _S["off_s"] = _S["off_s"] + DT if off else 0.0
    under = {}
    for k, pts in SMUDGE_PTS.items():
        u = under_pad(pts, plate, towel)
        under[k] = int(u.sum())
        if wiping:  # the pad slid over the points it covered at the start and at the end of this step
            _S["travel"][k][u & _S["was_under"][k]] += travel
        _S["was_under"][k] = u
    cover = {k: float(np.mean(v >= WIPE_TRAVEL)) for k, v in _S["travel"].items()}
    cleaned = [k for k in SMUDGES if cover[k] >= CLEAN_FRAC]
    _hide_tiles({k: (v >= WIPE_TRAVEL) | (k in cleaned) for k, v in _S["travel"].items()})

    move = float(np.linalg.norm(plate[0][:2] - np.asarray(PLATE_XY)))
    yaw = float(np.degrees(np.arctan2(plate[1][1, 0], plate[1][0, 0])))
    if _S["wipe0"] is None and force > 0.0:  # the towel's first touch: from here on the plate must not slide
        _S["wipe0"] = (plate[0][:2].copy(), yaw)
    if _S["wipe0"] is not None:
        _S["max_slide"] = max(_S["max_slide"], float(np.linalg.norm(plate[0][:2] - _S["wipe0"][0])))
        _S["max_yaw"] = max(_S["max_yaw"], abs(yaw - _S["wipe0"][1]))
    _S["max_move"] = max(_S["max_move"], move)
    if plate[0][2] - PLATE_POS[2] > LIFT_MAX:
        _S["lifted"] = True

    # the food: a push from any watched link of either arm or from the towel, or finger pads / towel within 1 cm of it
    touch = {w: f for w, path in FOOD_WATCH.items() if (f := _peak(("food", path))) >= FOOD_TOUCH}
    for w, f in touch.items():
        _S["food_force"][w] = round(max(_S["food_force"].get(w, 0.0), f), 3)
    gaps = {f"{arm.split('_')[0]} finger pads": food_gap(pts, food) for arm, pts in kk.STATE.fingers.items()}
    gaps["towel"] = food_gap(towel[0] + TOWEL_SURF @ towel[1].T, food)
    w_min = min(gaps, key=gaps.get)
    if gaps[w_min] < _S["gap"][0]:
        _S["gap"] = (round(gaps[w_min], 4), w_min, kk.STATE.step)
    who = [f"{w} touched it ({f:.2f} N)" for w, f in touch.items()] + [f"{w} within 1 cm" for w, g in gaps.items()
                                                                        if g < FOOD_MARGIN]
    if who and not _S["food"]:
        _S["food_at"] = (kk.STATE.step, who)
    if who:
        _S["food"] = True

    # diagnostics only (not scored): what the previous version's rules would have counted
    old_wiping = force >= _OLD_PRESS and speed >= RUB_SPEED
    for k, (sx, sy) in SMUDGES.items():
        if old_wiping and float(np.hypot(rel[0] - sx, rel[1] - sy)) <= _OLD_OVER:
            _S["old_rub"][k] += 1
    old = [a for a in kk.STATE.fingers if kk.near(a, _OLD_FOOD_C, _OLD_FOOD_S, plate, 0.010)]
    if float(kk.box_dist(towel[0][None] + towel[1] @ np.array([0.0, 0.0, TOWEL[2] / 2]), _OLD_FOOD_C,
                         np.asarray(_OLD_FOOD_S) + np.asarray(TOWEL), plate)[0]) <= 0.0:
        old.append("towel")
    if old and _S["old_food"] is None:
        _S["old_food"] = (kk.STATE.step, old)

    _S["last"] = {"force": round(force, 3), "speed": round(speed, 4), "under_pad": under,
                  "coverage": {k: round(v, 3) for k, v in cover.items()}, "cleaned": cleaned,
                  "plate_moved": round(_S["max_move"], 4), "plate_slid": round(_S["max_slide"], 4),
                  "plate_yaw": round(_S["max_yaw"], 2), "lifted": _S["lifted"],
                  "food_touched": _S["food"], "food_at": _S["food_at"], "food_contact_max_N": dict(_S["food_force"]),
                  "food_gap_min": _S["gap"], "contact_errors": kk.contact_errors(),
                  "sauce_tiles_hidden": {k: sum(1 for kk_, _ in _VIS["hidden"] if kk_ == k) for k in SMUDGES},
                  "visual_error": _VIS["error"], "towel_off_s": round(_S["off_s"], 2),
                  "old_rules_not_scored": {"centre_rub_steps": dict(_S["old_rub"]),
                                           "would_clean": [k for k, n in _S["old_rub"].items() if n >= _OLD_RUB_STEPS],
                                           "food_box_hit": _S["old_food"]}}
    if _S["food"] or _S["lifted"] or _S["max_move"] > MAX_MOVE:
        res = (0.0, False)
    else:
        frac = round(len(cleaned) / len(SMUDGES), 2)
        steady = _S["max_slide"] <= STILL_MOVE and _S["max_yaw"] <= MAX_YAW
        done = frac == 1.0 and _S["off_s"] >= OFF_HOLD - 1e-9  # the trial ends once the towel is also off the plate
        res = (frac, done) if steady else (min(frac, 0.5), False)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """All three smudges clean: at least 95% of each smudge's sample points (a 2.5 mm grid over its whole 3 cm disc)
    wiped, a point being wiped once the towel pad's pressed underside (contact >= 1.2 N, moving >= 1 cm/s relative to
    the plate) has slid at least 1 cm over it in total, counting the steps it was under the pad from start to end; the plate kept within 1 cm and 5 deg of where it was when the towel first
    touched it (gripping the lip may nudge it); the food untouched and kept 1 cm clear; the plate not lifted; and then
    the towel off the plate (not touching it, its underside 2 cm above the rim) for 1 s, so the trial ends with the rim
    in view."""
    return _update(objects)[1]


def score(objects) -> float:
    """One third per clean smudge, capped at 0.5 if the plate slid more than 1 cm or turned more than 5 deg once the
    towel touched it; 0 if the plate ended up more than 3 cm from its start or was lifted more than 2 cm, or if the food
    was touched (a contact force of 0.02 N or more between the food and the towel or any of the forearm, wrist, gripper
    body and finger links of either arm) or a finger pad or the towel came within 1 cm of any part of the food."""
    return _update(objects)[0]


def report() -> dict:
    return dict(_S.get("last", {}))
