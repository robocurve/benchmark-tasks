"""On the station table, a bundle of seven cables lies across two beech blocks, held together by
a black nylon cable tie between the blocks. The tie's head sits on the bundle's near side. A pair of flush cutters with
red handles lies in a grey tool tray to the right.

Task: cut the tie off without nicking the cables.
- One hand presses the bundle down on its left block so that it cannot roll.
- The other takes the flush cutters from the tray by their handles and snips the tie's strap right beside its head.
  Anywhere else, the strap lies tight on the insulation, and closing the jaws there nicks a cable.
- The cutters go back in the tray, the hand lets go of the bundle, and both arms go back to where they started.

Why both hands: the light bundle rolls or slides away as soon as the cutter's jaws touch it, unless it is held.

Run it:  inspect-robots run --embodiment yam_arena -E task=cut_a_cable_tie_off_a_bundle_without_nicking_the_cables.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset,  # noqa: E402
                     on_step, rotation, set_glue_on)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the blocks and the bundle: the bundle along y, across two beech blocks ------------------------------------------
BX = 0.30  # the bundle's line
BLOCK_Y = (0.12, -0.16)
BLOCK = (0.06, 0.05, 0.05)
BLOCK_TOP = TOP + BLOCK[2]
CABLE_R = 0.0035
BUNDLE_L = 0.38
# seven cables: one in the middle and six round it (the bundle's origin on the middle one's axis), lying with two of
# the outer six underneath
CABLES = ((0.0, 0.0), *[(2 * CABLE_R * math.cos(math.radians(30 + 60 * k)),
                         2 * CABLE_R * math.sin(math.radians(30 + 60 * k))) for k in range(6)])  # (x, z) of each axis
COLOURS = ((0.07, 0.07, 0.08), (0.80, 0.12, 0.10), (0.15, 0.35, 0.80), (0.95, 0.80, 0.15), (0.92, 0.92, 0.90),
           (0.45, 0.30, 0.18), (0.20, 0.60, 0.25))
ZC = BLOCK_TOP + 2 * CABLE_R * 0.5 + CABLE_R  # the bundle's centre height, resting on the blocks
BUNDLE_R = 3 * CABLE_R
BUNDLE_MASS = 0.12

# -- the tie: a strap round the bundle at TIE_Y, its head on the near (-x) side ---------------------------------------
TIE_Y = -0.04
STRAP_R = BUNDLE_R + 0.0007  # the strap's middle, from the bundle's centre
STRAP_T, STRAP_W = 0.0014, 0.004
HEAD_DEG = 30.0  # the head's place round the bundle: up from the near side (-x), toward +z
HEAD = (0.007, 0.006, 0.006)  # along the strap, across it (y), outward
SEGMENTS = 14

# -- the flush cutters: two halves on a pivot, lying flat, jaws toward +x; each half's handle is on the side opposite
# its jaw, so squeezing the handles together closes the jaws ----------------------------------------------------------
TRAY_PIVOT = np.array([0.20, -0.25, TOP + 0.035])  # the pivot, the cutters lying in the tray
JAW_TIP = 0.026  # the jaws' tips, ahead of the pivot
GRIP_X = -0.050  # where a hand holds the handles, behind the pivot (each half balances there)
JAW_GAP = 0.008  # the jaws' opening at the tips
CUT_DEG = 10.0  # the halves turned this far together: closed (they cut what is between the tips)
SPRING = 0.03  # N m per degree, opening ...
PRELOAD_DEG = 10.0  # ... pulling toward 10 degrees past open, against the open stop: 0.3 N m holds them open
CUTTER_MASS = 0.06  # each half

# -- the tray and the bin -----------------------------------------------------------------------------------------------
TRAY_FLOOR = TOP + 0.005

# -- scoring ------------------------------------------------------------------------------------------------------------
# The strap is severed only when it is truly between the blades as they close: the jaws' mouth across the strap (its
# middle within ACROSS_IN of the mouth's middle line, so all 4 mm of its width lie between the 8 mm-apart edges), the
# tips reaching at least to the strap's inner face (no farther than TIPS_PAST from the bundle's axis), the cutters
# pointing into the bundle (within AIM_DEG of the inward radial) and their pivot axis along the strap's run (the jaws
# closing across its width, not along it).
ACROSS_IN = JAW_GAP / 2 - STRAP_W / 2 + 0.001
TIPS_PAST = STRAP_R - STRAP_T / 2 + 0.0004
AIM_DEG = 50.0
SAFE_ARC = 0.012  # cut within this (along the strap) of the head's middle: beside the head, where it stands off
NICK_DEPTH = 0.0015  # a tip pressed this far into any cable's insulation as the jaws close: a nicked cable
OFF_BELOW = 0.015  # the cut tie off the bundle: its middle this far below the bundle's axis ...
OFF_AWAY = 0.03  # ... or this far from it sideways
LIFTED = 0.02  # the cutters lifted this far out of the tray: taken
BACK_WITHIN = 0.03  # back in the tray: each half within this of where it lay
ON_BLOCKS = 0.025  # the bundle still across its blocks: its centre within this of its line and not below the blocks
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0
HIDE = {"cable_tie": np.array([0.55, -0.45, TOP - 0.15]), "cut_tie": np.array([0.55, -0.40, TOP - 0.15])}

BLACK, RED, BLUE, BEECH, STEEL, GREY, DARK, NYLON, PVC_RED = (0.07, 0.07, 0.08), (0.80, 0.12, 0.10), \
    (0.15, 0.35, 0.80), (0.82, 0.66, 0.45), (0.62, 0.64, 0.66), (0.50, 0.52, 0.55), (0.22, 0.23, 0.25), \
    (0.10, 0.10, 0.11), (0.85, 0.15, 0.12)


def _quat_y(a: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _blocks() -> Body:
    shapes = [BoxShape((0.0, y, BLOCK[2] / 2), BLOCK, BEECH) for y in BLOCK_Y]
    return Body.build("blocks", pos=(BX, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, color=BEECH,
                      friction=(0.7, 0.6))


def _bundle() -> Body:
    shapes = []
    for (x, z), c in zip(CABLES, COLOURS):
        shapes.append(CylShape((x, 0.0, z), CABLE_R, BUNDLE_L, c, axis="Y"))
        for s in (1.0, -1.0):  # the stripped ends
            shapes.append(CylShape((x, s * (BUNDLE_L / 2 + 0.006), z), 0.0013, 0.012, (0.80, 0.50, 0.25),
                                   collide=False, axis="Y"))
    return Body.build("cable_bundle", pos=(BX, 0.0, ZC), mass=BUNDLE_MASS, shapes=shapes, color=BLACK,
                      friction=(0.7, 0.6), com=(0.0, 0.0, 0.0))


def _strap(skip: int = 0) -> list:
    """The strap's segments round the bundle (in the tie's frame: origin at the bundle's centre at TIE_Y), leaving out
    ``skip`` segments on the far side of the head (a cut tie)."""
    shapes = []
    seg = 2 * math.pi * STRAP_R / SEGMENTS * 1.08
    for k in range(SEGMENTS):
        a = math.radians(HEAD_DEG) + 2 * math.pi * (k + 0.5) / SEGMENTS
        if k < skip:
            continue
        d = np.array([-math.cos(a), 0.0, math.sin(a)])
        # a segment's long side runs round the bundle: turned about y so that its local z points along d
        shapes.append(BoxShape(tuple((d * STRAP_R).tolist()), (seg, STRAP_W, STRAP_T), NYLON,
                               quat_xyzw=_quat_y(math.atan2(d[0], d[2]))))
    return shapes


def _head() -> BoxShape:
    a = math.radians(HEAD_DEG)
    d = np.array([-math.cos(a), 0.0, math.sin(a)])
    c = d * (STRAP_R + STRAP_T / 2 + HEAD[2] / 2)
    return BoxShape(tuple(c.tolist()), (HEAD[0], HEAD[1], HEAD[2]), NYLON, quat_xyzw=_quat_y(math.atan2(d[0], d[2])))


def _tie() -> Body:
    return Body.build("cable_tie", pos=(BX, TIE_Y, ZC), mass=0.002, shapes=[*_strap(), _head()], color=NYLON,
                      friction=(0.6, 0.5))


def _cut_tie() -> Body:
    """The tie once cut (the strap open beside its head), kept out of sight until then."""
    return Body.build("cut_tie", pos=tuple(HIDE["cut_tie"].tolist()), mass=0.002, shapes=[*_strap(skip=2), _head()],
                      color=NYLON, friction=(0.6, 0.5), ignore=("cable_bundle",))


def _handle(side: float) -> list:
    """A handle running back from the pivot on the ``side`` (+1: +y) of the cutters, with a red PVC sleeve."""
    p0, p1 = np.array([-0.008, side * 0.004]), np.array([-0.108, side * 0.0175])
    d = p1 - p0
    yaw = math.atan2(d[1], d[0])
    mid = (p0 + p1) / 2
    s0 = p0 + d * 0.30  # the sleeve, over the back 70 %
    smid = (s0 + p1) / 2
    return [BoxShape((float(mid[0]), float(mid[1]), 0.0), (float(np.linalg.norm(d)), 0.006, 0.008), STEEL, yaw=yaw),
            BoxShape((float(smid[0]), float(smid[1]), 0.0), (float(np.linalg.norm(p1 - s0)), 0.010, 0.014), PVC_RED,
                     yaw=yaw)]


def _jaw(side: float) -> list:
    """A jaw ahead of the pivot on the ``side`` of the cutters, its cutting edge JAW_GAP / 2 from the middle line."""
    w = 0.006
    y = side * (JAW_GAP / 2 + w / 2)
    return [BoxShape((0.012, y, 0.0), (0.020, w, 0.006), STEEL),
            BoxShape((0.0235, side * (JAW_GAP / 2 + 0.002), 0.0), (0.005, 0.004, 0.003), STEEL)]  # the tip


def _cutter_a() -> Body:
    """The half with the pivot: its jaw on +y, its handle on -y."""
    shapes = [CylShape((0.0, 0.0, 0.0), 0.009, 0.010, STEEL), *_jaw(1.0), *_handle(-1.0)]
    return Body.build("cutters", pos=tuple(TRAY_PIVOT.tolist()), mass=CUTTER_MASS, shapes=shapes, color=PVC_RED,
                      friction=(1.2, 1.0), com=(GRIP_X, -0.0097, 0.0))


def _cutter_b() -> Body:
    """The other half: its jaw on -y, its handle on +y."""
    shapes = [CylShape((0.0, 0.0, 0.0055), 0.006, 0.002, DARK, collide=False), *_jaw(-1.0), *_handle(1.0)]
    return Body.build("cutters_b", pos=tuple(TRAY_PIVOT.tolist()), mass=CUTTER_MASS, shapes=shapes, color=PVC_RED,
                      friction=(1.2, 1.0), com=(GRIP_X, 0.0097, 0.0))


def _tray() -> Body:
    """A grey ABS tray with two rests across it, one under the cutters' jaws and one under their handle ends."""
    tx, ty = float(TRAY_PIVOT[0]) - 0.04, float(TRAY_PIVOT[1])
    o = np.array([tx, ty, TOP])
    rest_h = float(TRAY_PIVOT[2]) - 0.007 - TOP
    shapes = [BoxShape((0.0, 0.0, 0.0025), (0.18, 0.10, 0.005), GREY)]
    for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        size = (0.004, 0.10, 0.025) if sx else (0.18, 0.004, 0.025)
        shapes.append(BoxShape((sx * 0.088, sy * 0.048, 0.0125), size, GREY))
    for x in (0.045, -0.063):  # under the jaws and pivot, and under the handles' ends (pivot at x = 0.04 here)
        shapes.append(BoxShape((x, 0.0, rest_h / 2), (0.022, 0.07, rest_h), DARK))
    return Body.build("tool_tray", pos=tuple(o.tolist()), mass=1.0, shapes=shapes, kinematic=True, color=GREY,
                      friction=(0.6, 0.5))


OBJECTS = [_blocks(), _bundle(), _tie(), _cut_tie(), _tray(), _cutter_a(), _cutter_b()]
glue("tie_on", "cable_bundle", "cable_tie", anchor=(BX, TIE_Y, ZC))
# the cutters' pivot: cutters_b turns against cutters about z; squeezing the handles turns it positive
hinge("cutter_pivot", "cutters", "cutters_b", anchor=tuple(TRAY_PIVOT.tolist()), axis="Z", limits_deg=(0.0, 16.0),
      spring=SPRING, damping=0.0005, target_deg=-PRELOAD_DEG)


# -- followed after every control step ----------------------------------------------------------------------------

def _fresh() -> dict:
    return {"cut": False, "nicked": False, "taken": False, "closed_before": False, "tcps": None, "s": {},
            "steps": 0, "mouth_ring": None, "aim": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TIE_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    set_glue_on("tie_on", True)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _place(base, origin, name, at, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), *quat_xyzw]], dtype=torch.float32, device=asset.device))
    asset.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=torch.float32, device=asset.device))


def _pose(base, origin, name) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return t[:3] - origin, rotation(t[3:7]), t[3:7]


def _mouth_on_strap(mouth, tpos, trot) -> tuple[float, float, float]:
    """Where the jaws' tips are against the strap: (radial gap to the strap's middle, offset across it, arc distance
    along it from the head's middle), all in m."""
    q = trot.T @ (np.asarray(mouth, float) - tpos)  # in the tie's frame (its y along the bundle)
    r = math.hypot(float(q[0]), float(q[2]))
    a = math.atan2(float(q[2]), float(-q[0]))  # up from the near side, as HEAD_DEG
    da = (a - math.radians(HEAD_DEG) + math.pi) % (2 * math.pi) - math.pi
    return abs(r - STRAP_R), abs(float(q[1])), abs(da) * STRAP_R


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    apos, arot, _ = _pose(base, origin, "cutters")
    bpos, brot, _ = _pose(base, origin, "cutters_b")
    rel = arot.T @ brot
    closed = math.degrees(math.atan2(float(rel[1, 0]), float(rel[0, 0])))  # the halves turned together
    mouth = apos + arot @ np.array([JAW_TIP - 0.001, 0.0, 0.0])
    tpos, trot, tq = _pose(base, origin, "cable_tie")
    gap, across, arc = _mouth_on_strap(mouth, tpos, trot)
    _EP["mouth_ring"] = (gap, across, arc)
    bun, brot_b, _ = _pose(base, origin, "cable_bundle")
    tips = [apos + arot @ np.array([JAW_TIP, JAW_GAP / 2, dz]) for dz in (-0.0015, 0.0015)]  # the tips' inner corners
    tips += [bpos + brot @ np.array([JAW_TIP, -JAW_GAP / 2, dz]) for dz in (-0.0015, 0.0015)]
    tq_b = [brot_b.T @ (t - bun) for t in tips]  # the tips' corners in the bundle's frame
    tip_r = min(math.hypot(float(q[0]), float(q[2])) for q in tq_b)
    dent = max((CABLE_R - math.hypot(float(q[0]) - cx, float(q[2]) - cz) for q in tq_b for cx, cz in CABLES
                if abs(float(q[1])) < BUNDLE_L / 2), default=-1.0)  # deepest a tip corner is in a cable
    _EP["dent"] = dent
    q_mouth = brot_b.T @ (mouth - bun)
    inward = -np.array([q_mouth[0], 0.0, q_mouth[2]]) / max(math.hypot(float(q_mouth[0]), float(q_mouth[2])), 1e-6)
    aim = math.degrees(math.acos(max(-1.0, min(1.0, float(inward @ (brot_b.T @ arot[:, 0]))))))
    pivot_along = math.degrees(math.asin(min(1.0, abs(float((brot_b.T @ arot[:, 2])[1])))))  # pivot axis vs bundle
    _EP["aim"] = (tip_r, aim, pivot_along)
    _EP["qm"] = trot.T @ (mouth - tpos)  # the mouth in the tie's frame (debug)
    if not _EP["cut"] and closed >= CUT_DEG and not _EP["closed_before"]:
        _EP["closed_before"] = True
        if dent >= NICK_DEPTH:
            _EP["nicked"] = True  # a tip pressed into a cable's insulation as the jaws closed
        if (across <= ACROSS_IN and gap <= 0.004 and tip_r <= TIPS_PAST and aim <= AIM_DEG
                and pivot_along <= AIM_DEG):  # the strap is between the blades: severed
            _EP["cut"] = True
            if arc > SAFE_ARC:
                _EP["nicked"] = True  # the strap lies tight on the insulation there: the jaws nicked a cable
            set_glue_on("tie_on", False)
            _place(base, origin, "cut_tie", tpos + np.array([0.0, 0.0, 0.002]), tuple(float(v) for v in tq))
    if closed < CUT_DEG * 0.5:
        _EP["closed_before"] = False
    # keep the parts that are out of play out of sight
    if _EP["cut"]:
        _place(base, origin, "cable_tie", HIDE["cable_tie"])
    else:
        _place(base, origin, "cut_tie", HIDE["cut_tie"])
    if apos[2] > TRAY_PIVOT[2] + LIFTED and bpos[2] > TRAY_PIVOT[2] + LIFTED:
        _EP["taken"] = True
    _EP["tcps"] = _tcps(base, origin)
    ctp, _, _ = _pose(base, origin, "cut_tie")
    off = _EP["cut"] and (float(ctp[2]) < float(bun[2]) - OFF_BELOW
                          or math.hypot(float(ctp[0] - bun[0]), float(ctp[2] - bun[2])) > OFF_AWAY
                          or abs(float(ctp[1] - bun[1])) > BUNDLE_L / 2)
    grip = apos + arot @ np.array([GRIP_X, 0.0, 0.0])
    press = bun + np.array([0.0, BLOCK_Y[0], 0.0])
    clear = all(float(np.linalg.norm(t - q)) >= LET_GO for t in _EP["tcps"] for q in (grip, press))
    back = (float(np.linalg.norm(apos - TRAY_PIVOT)) <= BACK_WITHIN
            and float(np.linalg.norm(bpos - TRAY_PIVOT)) <= BACK_WITHIN)
    on_blocks = abs(float(bun[0]) - BX) <= ON_BLOCKS and float(bun[2]) >= ZC - 0.006
    _EP["s"] = {"closed": closed, "clear": clear, "back": back, "on_blocks": on_blocks, "off": off}
    _EP["steps"] += 1
    if _DEBUG["on"] and _EP["steps"] % 10 == 0:
        tl, tr = _EP["tcps"]
        print(f"[tie] t={_EP['steps'] / 20:5.1f}s closed={closed:5.1f}deg mouth: gap={gap * 1000:5.1f}mm "
              f"across={across * 1000:5.1f}mm arc={arc * 1000:5.1f}mm tip_r={tip_r * 1000:5.1f}mm aim={aim:4.0f} "
              f"pivot_along={pivot_along:4.0f} dent={_EP['dent'] * 1000:4.1f}mm qm={np.round(_EP['qm'] * 1000, 1).tolist()} off={off} cut={_EP['cut']} nicked={_EP['nicked']} "
              f"taken={_EP['taken']} back={back} on_blocks={on_blocks} clear={clear} "
              f"bundle={np.round((bun - [BX, 0.0, ZC]) * 1000).astype(int).tolist()} "
              f"rTCP-grip={np.round((tr - grip) * 1000).astype(int).tolist()} "
              f"lTCP-press={np.round((tl - press) * 1000).astype(int).tolist()} score={score(None)}",
              file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if a cable was ever nicked: the jaws closed on the strap away from the head, or closed with a tip corner
    pressed 1.5 mm or more into any cable. Else 1.0 once the strap was severed beside the head (between the blades as they closed) and
    the tie has come off the bundle, the cutters lie back in the tray, the bundle still lies across its blocks, both
    hands have let go (each TCP at least 4 cm from the cutters' handles and from the bundle above its left block) and
    both arms are back where they started; 0.5 once the strap was severed beside the head and the tie is off the
    bundle; 0.25 if the cutters were taken out of the tray at some point; else 0.0."""
    s = _EP["s"]
    if _EP["nicked"] or not s:
        return 0.0
    if _EP["cut"] and s["off"]:
        if s["back"] and s["on_blocks"] and s["clear"] and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if _EP["taken"] else 0.0


_HOME: dict = {}


def _arms_home() -> bool:
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
    """Full score with the cutters and the bundle still: the trial ends there."""
    return (score(objects) >= 1.0 and objects["cutters"].resting() and objects["cutters_b"].resting()
            and objects["cable_bundle"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the bundle above its left block, fingers closed, and presses it down. The right
# comes straight down round the cutters' handles, fingers closing across them just enough to hold them, lifts them out
# of the tray and carries them, jaws first, level with the strap just below the tie's head; it pushes the jaws in until
# they touch the bundle, the strap between the tips, and squeezes the handles: the tie springs off. It carries the
# cutters back, lays them in the tray and lets go. The left lets go of the bundle, and both arms go back to where they
# started.

HANDLE_GRIP = grip_rotation(0.0)  # straight down, fingers closing along y, across both handles
PRESS_GRIP = grip_rotation(0.0)  # straight down, fingers closed, their long side along the bundle
HANDLE_SPREAD = 0.0294  # the handles' outer faces apart, where they are held
OPEN_AROUND = -(HANDLE_SPREAD / 2 + 0.007)
HOLD = -(HANDLE_SPREAD / 2 - 0.0015)  # a light squeeze: holds them, the jaws still open
TIP = 0.013  # closed fingertips below the TCP
PRESS_IN = 0.010
CUT_DEG_AT = HEAD_DEG - 20.0  # the jaws go to the strap here, just below the head
PUSH_IN = 0.004  # aimed this far past the strap, into the bundle
AIM_DY = 0.003  # measured: the held cutters end 3 mm toward -y of where they are aimed; aim this much the other way
BUNDLE_SIT = 0.003  # measured: the bundle rests this much above ZC on its blocks
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


def _cut_point(push: float) -> np.ndarray:
    """The jaws' tips at the strap just below the head, pushed ``push`` past it into the bundle."""
    a = math.radians(CUT_DEG_AT)
    return np.array([BX - (STRAP_R - push) * math.cos(a), TIE_Y + AIM_DY, ZC + BUNDLE_SIT + (STRAP_R - push) * math.sin(a)])


def _tcp_for_tips(tips) -> np.ndarray:
    """The right TCP holding the cutters (lying flat, jaws toward +x) with their tips at ``tips``."""
    return np.asarray(tips, float) + [GRIP_X - (JAW_TIP - 0.001), 0.0, 0.0]


def _right(press_on: float) -> tuple[list[Waypoint], float]:
    pick = TRAY_PIVOT + [GRIP_X, 0.0, 0.0]
    out = _lead_in("right", pick + [0.0, 0.0, 0.05], HANDLE_GRIP, "over the cutters' handles", OPEN_AROUND)
    out += [_w(1.0, pick, OPEN_AROUND, "down round the handles", HANDLE_GRIP),
            _w(0.6, pick, HOLD, "hold them: a light squeeze", HANDLE_GRIP),
            _w(1.0, pick + [0.0, 0.0, 0.05], HOLD, "lift them out of the tray", HANDLE_GRIP)]
    ready = _tcp_for_tips(_cut_point(-0.025))
    out += [_w(1.6, ready + [0.0, 0.0, 0.03], HOLD, "carry them to the tie, jaws first", HANDLE_GRIP),
            _w(0.8, ready, HOLD, "level with the strap below the tie's head", HANDLE_GRIP)]
    t = sum(w.duration for w in out)
    if t < press_on + 0.3:
        out.append(_w(press_on + 0.3 - t, ready, HOLD, "wait for the bundle to be held", HANDLE_GRIP))
    at = _tcp_for_tips(_cut_point(PUSH_IN))
    out += [_w(1.2, at, HOLD, "jaws onto the strap, right beside the head", HANDLE_GRIP),
            _w(0.6, at, FINGER_CLOSED, "squeeze: snip the strap", HANDLE_GRIP),
            _w(0.5, at, FINGER_CLOSED, "the tie springs off", HANDLE_GRIP),
            _w(0.5, at, HOLD, "ease the squeeze", HANDLE_GRIP),
            _w(0.8, ready + [0.0, 0.0, 0.03], HOLD, "draw the cutters back", HANDLE_GRIP),
            _w(1.6, pick + [0.0, 0.0, 0.03], HOLD, "carry them back to the tray", HANDLE_GRIP),
            _w(0.8, pick + [0.0, 0.0, 0.003], HOLD, "lay them on the rests", HANDLE_GRIP),
            _w(0.5, pick + [0.0, 0.0, 0.003], OPEN_AROUND, "let go", HANDLE_GRIP),
            _w(0.8, pick + [0.0, 0.0, 0.05], OPEN_AROUND, "rise", HANDLE_GRIP)]
    done = sum(w.duration for w in out)
    out += _go_home("right", pick + [0.0, 0.0, 0.05], HANDLE_GRIP, OPEN_AROUND)
    return out, done


def _press(dz: float) -> np.ndarray:
    top = ZC + BUNDLE_R
    return np.array([BX, BLOCK_Y[0], top + TIP - dz])


def _left(right_done: float) -> tuple[list[Waypoint], float]:
    out = _lead_in("left", _press(-0.04), PRESS_GRIP, "over the bundle, above its left block", FINGER_CLOSED)
    out += [_w(1.0, _press(0.0), FINGER_CLOSED, "down onto the bundle", PRESS_GRIP),
            _w(0.6, _press(PRESS_IN), FINGER_CLOSED, "press it down: it cannot roll", PRESS_GRIP)]
    press_on = sum(w.duration for w in out)
    t = press_on
    out.append(_w(max(right_done - 1.5 - t, 0.5), _press(PRESS_IN), FINGER_CLOSED,
                  "hold it while the tie is cut", PRESS_GRIP))
    out += [_w(0.8, _press(-0.05), FINGER_CLOSED, "let go of the bundle", PRESS_GRIP)]
    out += _go_home("left", _press(-0.05), PRESS_GRIP, FINGER_CLOSED)
    return out, press_on


def demo():
    _, press_on = _left(0.0)
    right, done = _right(press_on)
    left, _ = _left(done)
    total = max(sum(w.duration for w in right), sum(w.duration for w in left)) + 3.0
    for arm in (left, right):
        arm.append(_w(total - sum(w.duration for w in arm), arm[-1].pos, arm[-1].finger,
                      "hold still: the trial ends once the full score has held 1 s with both arms home", arm[-1].yaw))
    return {"left": left, "right": right}
