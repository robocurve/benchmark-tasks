"""On the station table: a stainless mixing bowl with a batch of bread dough's ingredients (flour, and a pool of
water and egg in a well in the middle of it). A wooden spoon with a built-up grip stands in a spoon clip on the bowl's
near rim, its head down in the batch.

The flour and the liquid have to be mixed into a dough. A bowl standing loose on a worktop turns and slides when a
spoon is dragged through stiff dough in it, so one hand steadies the bowl by its grip tab while the other stirs.

Mixing is judged as a baker judges it: by whether dry flour is left. Each flour bead becomes "wetted" once the spoon
has worked it into liquid or into flour that is already wetted: while the spoon's head is moving through the batch
(faster than 3 cm/s), within 4 cm of it, in contact with wet material and sliding against it at more than 2 cm/s,
for 1 cm in total over the trial. Flour that lies against the liquid without being worked stays dry, and so does
flour shaken or carried round with the bowl.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import doughkit as K  # noqa: E402
from doughkit import Beads, Bowl, Fixed, Spoon, bead_states, on_reset, on_step, reset_beads, tint_beads  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Mix the flour and the liquid in the bowl into a dough with the wooden spoon."

# -- layout ---------------------------------------------------------------------------------------------------------
LOT = os.environ.get("DOUGH_LOT", "A")
LOTS = {"A": dict(bowl=(0.330, -0.040))}
if LOT not in LOTS:
    raise ValueError(f"DOUGH_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
BOWL_XY = np.array(LOTS[LOT]["bowl"])

# -- the bowl: a 2.5 qt stainless mixing bowl ----------------------------------------------------------------------
R_BOTTOM, R_TOP, BOWL_H, WALL, BASE = 0.065, 0.100, 0.085, 0.005, 0.006
TAB = (0.040, 0.005, 0.025)             # the grip tab on the bowl's left side: 4 cm out from the rim, 5 mm thick, 2.5 cm tall
CLIP = (0.022, 0.014, 0.020)            # the spoon clip on the near rim: 2.2 cm out, 14 mm notch, posts 2 cm tall
BOWL_MASS = 0.32
BOWL_FRICTION = (0.25, 0.20)            # steel on a laminate worktop

# -- the spoon: a 26 cm beech spoon with a built-up silicone grip ---------------------------------------------------
HANDLE = (0.012, 0.009, 0.184)
HEAD = (0.045, 0.010, 0.060)
SPOON_LEN = HANDLE[2] + 0.016 + HEAD[2]          # 0.260
GRIP = (0.024, 0.030, 0.050)                     # the built-up grip: across, deep, long
SPOON_MASS = 0.052                               # 40 g beech spoon and a 12 g silicone grip
SLANT = math.radians(48)                         # the spoon in its clip: this far from vertical
GRIP_FROM_TIP = 0.199                            # the grip's middle, this far from the head's tip
PINCH_FROM_TIP = 0.199                           # where a hand takes the grip (used by the demo)

# -- the batch -----------------------------------------------------------------------------------------------------
BEAD_R = 0.006
FLOUR_MASS, WET_MASS = 0.0015, 0.0035   # per bead: 160 x 1.5 g = 240 g flour; 40 x 3.5 g = 140 g water and egg
N_FLOUR, N_WET = 160, 40
WORKED = 0.02                           # m/s relative speed in contact: the flour is being worked
RUB = 0.01                              # m of rubbing against wet material (summed over the trial) wets a flour bead
REACH = 0.04                            # only flour within this of the spoon's head is being worked by it ...
HEAD_MOVING = 0.03                      # ... and only while the head is moving through the batch faster than this (m/s)
DT = 1.0 / 20                           # s per control step
CONTACT = 2 * BEAD_R + 0.002            # centres closer than this are touching
GRACE = 20                              # control steps after a reset before anything counts (the batch settles)
SPILL_MAX = 0.05                        # more than this share of the batch's mass out of the bowl: 0
TIPPED = math.radians(30)               # the bowl tilted further than this: 0
FULL = 0.95                             # this share of the flour wetted earns full marks


# The spoon stands in its clip on the bowl's near rim, leaning toward the arms 48 degrees from vertical, its head's tip
# on the bowl's floor 2.45 cm short of the middle.
_U = np.array([-math.sin(SLANT), 0.0, math.cos(SLANT)])      # along the spoon, toward the handle's end
SPOON_TIP = np.array([BOWL_XY[0] - 0.0245, BOWL_XY[1], TOP + BASE + 0.009])
SPOON_TOP = tuple(float(v) for v in SPOON_TIP + SPOON_LEN * _U)
_R = np.column_stack([np.array([0.0, 1.0, 0.0]), np.cross(_U, [0.0, 1.0, 0.0]), _U])
_w = math.sqrt(1 + np.trace(_R)) / 2
SPOON_QUAT = (float((_R[2, 1] - _R[1, 2]) / (4 * _w)), float((_R[0, 2] - _R[2, 0]) / (4 * _w)),
              float((_R[1, 0] - _R[0, 1]) / (4 * _w)), float(_w))
SPOON_AXIS = _U
GRIP_AT = SPOON_LEN - GRIP_FROM_TIP                    # from the handle's top end
PINCH_AT = SPOON_LEN - PINCH_FROM_TIP


def _in_spoon(p, clearance=BEAD_R + 0.002) -> bool:
    """Whether a bead at ``p`` would overlap the spoon's head, neck or handle."""
    local = _R.T @ (np.asarray(p) - np.asarray(SPOON_TOP))
    for centre, size in (((0.0, 0.0, -HANDLE[2] / 2), HANDLE), ((0.0, 0.0, -HANDLE[2] - 0.008), (0.020, 0.009, 0.016)),
                         ((0.0, 0.0, -HANDLE[2] - 0.016 - HEAD[2] / 2), HEAD)):
        d = np.maximum(np.abs(local - np.asarray(centre)) - np.asarray(size) / 2, 0.0)
        if np.linalg.norm(d) < clearance:
            return True
    return False


def _lattice(z, r_max, spacing, offset):
    pts = []
    n = int(r_max / spacing) + 2
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x = (i + 0.5 * (j % 2) + offset) * spacing
            y = j * spacing * math.sqrt(3) / 2
            if math.hypot(x, y) <= r_max:
                pts.append((x, y, z))
    return pts


def _batch():
    """Flour in two layers on the bowl's floor with a well in the top one, the liquid in the well."""
    floor = TOP + BASE
    sp = 2 * BEAD_R + 0.0008
    flour, wet = [], []
    z = floor + BEAD_R + 0.0005
    layer = 0
    while len(flour) < N_FLOUR:
        r_wall = R_BOTTOM + (z - floor) * (R_TOP - R_BOTTOM) / (BOWL_H - BASE) - BEAD_R - 0.002
        for p in _lattice(z, r_wall, sp, 0.25 * (layer % 2)):
            if len(flour) >= N_FLOUR:
                break
            if _in_spoon((BOWL_XY[0] + p[0], BOWL_XY[1] + p[1], p[2])):
                continue
            if layer >= 1 and math.hypot(p[0], p[1]) < 0.036:
                wet.append(p)                                      # the well (overlaps were skipped above)
                continue
            flour.append(p)
        z += sp * 0.82
        layer += 1
    while len(wet) < N_WET:                                        # the rest of the liquid on top of the well
        for p in _lattice(z, 0.040, sp, 0.25 * (layer % 2)):
            if len(wet) < N_WET and not _in_spoon((BOWL_XY[0] + p[0], BOWL_XY[1] + p[1], p[2])):
                wet.append(p)
        z += sp * 0.82
        layer += 1
    wet = wet[:N_WET]
    pos = [(BOWL_XY[0] + x, BOWL_XY[1] + y, zz) for x, y, zz in flour + wet]
    kinds = [0] * len(flour) + [1] * len(wet)
    return tuple(tuple(float(v) for v in p) for p in pos), tuple(kinds)


POSITIONS, KINDS = _batch()
KINDS_NP = np.array(KINDS)
FLOUR = np.flatnonzero(KINDS_NP == 0)
MASSES = np.where(KINDS_NP == 0, FLOUR_MASS, WET_MASS)


def tab_point() -> np.ndarray:
    """The middle of the grip tab (world, at the start)."""
    return np.array([BOWL_XY[0], BOWL_XY[1] + R_TOP + WALL / 2 + TAB[0] / 2, TOP + BOWL_H - TAB[2] / 2 - 0.002])


def on_spoon(d: float) -> np.ndarray:
    """World point (at the start) on the spoon's centre line, ``d`` from the handle's top end toward the head."""
    return np.asarray(SPOON_TOP) - d * SPOON_AXIS


OBJECTS = [
    Bowl("bowl", pos=(float(BOWL_XY[0]), float(BOWL_XY[1]), TOP), size=(2 * R_TOP + 0.02, 2 * R_TOP + 0.02, BOWL_H),
         mass=BOWL_MASS, color=(0.80, 0.81, 0.83), r_bottom=R_BOTTOM, r_top=R_TOP, height=BOWL_H, wall=WALL,
         base=BASE, friction=BOWL_FRICTION, tab=TAB, clip=CLIP),
    Spoon("spoon", pos=SPOON_TOP, quat=SPOON_QUAT, size=(HEAD[0], HEAD[1], SPOON_LEN), mass=SPOON_MASS,
          color=(0.80, 0.62, 0.40), handle=HANDLE, head=HEAD, grip=GRIP, grip_at=GRIP_AT),
    Beads("dough", pos=(float(BOWL_XY[0]), float(BOWL_XY[1]), TOP - 0.05), size=(0.01, 0.01, 0.01),
          positions=POSITIONS, kinds=KINDS, radius=BEAD_R, masses=(FLOUR_MASS, WET_MASS)),
]


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(step=0, last_head=None, kinds=KINDS_NP.copy(), rubbed=np.zeros(len(KINDS)), spilled=False, tipped=False, spill_share=0.0, in_bowl=None,
                  bowl_tilt=0.0)


_clear()


def in_bowl(pos, bowl_pos, bowl_rot) -> np.ndarray:
    """Which beads are inside the bowl (in its own frame: above the base, below the rim, inside the wall)."""
    local = (pos - bowl_pos) @ bowl_rot
    z = local[:, 2]
    r_wall = R_BOTTOM + np.clip(z - BASE, 0, None) * (R_TOP - R_BOTTOM) / (BOWL_H - BASE) + 0.004
    return (z > -0.002) & (z < BOWL_H + 0.03) & (np.hypot(local[:, 0], local[:, 1]) < r_wall)


def head_centre(spoon_pos, spoon_quat) -> np.ndarray:
    """The middle of the spoon's head (the handle's top end is the body's origin; the head is at the other end)."""
    return np.asarray(spoon_pos) - (HANDLE[2] + 0.016 + HEAD[2] / 2) * K._rotation(spoon_quat)[:, 2]


def wet_step(kinds, rubbed, pos, vel, head, head_speed=1.0):
    """One control step of mixing by the spoon. While the spoon's head is moving faster than HEAD_MOVING, each dry
    flour bead within REACH of it, in contact with wet material (liquid or wetted flour) and sliding against it faster
    than WORKED, adds that sliding distance to its total; at RUB in total it is wetted. Returns the new kinds and
    totals."""
    kinds, rubbed = kinds.copy(), rubbed.copy()
    if head_speed < HEAD_MOVING:
        return kinds, rubbed
    dry = np.flatnonzero((kinds == 0) & (np.linalg.norm(pos - head, axis=1) < REACH))
    wet = np.flatnonzero(kinds != 0)
    if len(dry) == 0 or len(wet) == 0:
        return kinds, rubbed
    d = np.linalg.norm(pos[dry][:, None, :] - pos[wet][None, :, :], axis=2)
    dv = np.linalg.norm(vel[dry][:, None, :] - vel[wet][None, :, :], axis=2)
    slide = np.where((d < CONTACT) & (dv > WORKED), dv, 0.0).max(axis=1)
    rubbed[dry] += slide * DT
    kinds[dry[rubbed[dry] >= RUB]] = 2
    return kinds, rubbed


@on_reset
def _reset(environment) -> None:
    _clear()
    reset_beads(environment)


@on_step
def _watch(environment) -> None:
    _STATE["step"] += 1
    pos, vel = bead_states(environment)
    bowl_pos, bowl_quat = K.body_pose(environment, "bowl")
    rot = K._rotation(bowl_quat)
    tilt = math.acos(max(-1.0, min(1.0, float(rot[2, 2]))))
    inside = in_bowl(pos, bowl_pos, rot)
    _STATE["in_bowl"] = inside
    _STATE["centroid"] = pos.mean(axis=0)
    _STATE["flour_moved"] = float(np.median(np.linalg.norm(pos[FLOUR] - np.asarray(POSITIONS)[FLOUR], axis=1)))
    _STATE["spoon_root"] = K.body_pose(environment, "spoon")
    _STATE["bowl_tilt"] = tilt
    if _STATE["step"] <= GRACE:
        return
    _STATE["tipped"] = _STATE["tipped"] or tilt > TIPPED
    share = float(MASSES[~inside].sum() / MASSES.sum())
    _STATE["spill_share"] = max(_STATE["spill_share"], share)
    _STATE["spilled"] = _STATE["spilled"] or share > SPILL_MAX
    head = head_centre(*_STATE["spoon_root"])
    last = _STATE.get("last_head")
    speed = 0.0 if last is None else float(np.linalg.norm(head - last)) / DT
    _STATE["last_head"] = head
    new, _STATE["rubbed"] = wet_step(_STATE["kinds"], _STATE["rubbed"], pos, vel, head, speed)
    if (new != _STATE["kinds"]).any():
        _STATE["kinds"] = new
        tint_beads([int(k) for k in new])


# -- scoring --------------------------------------------------------------------------------------------------------
def wetted_share() -> float:
    """The share (by mass) of the flour that is wetted and in the bowl."""
    kinds = _STATE["kinds"]
    inside = _STATE["in_bowl"] if _STATE["in_bowl"] is not None else np.ones(len(kinds), bool)
    return float(((kinds[FLOUR] == 2) & inside[FLOUR]).sum() / len(FLOUR))


def score(objects) -> float:
    """0 if the bowl ever tipped past 30 degrees or more than 5% of the batch was ever out of the bowl. Otherwise
    the share of the flour wetted and in the bowl, divided by 0.95, at most 1."""
    if _STATE["tipped"] or _STATE["spilled"]:
        return 0.0
    return min(1.0, wetted_share() / FULL)


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    return {"wetted_share": round(wetted_share(), 3), "spill_share_max": round(_STATE["spill_share"], 3),
            "bowl_tilt_deg": round(math.degrees(_STATE["bowl_tilt"]), 1), "tipped": _STATE["tipped"],
            "spilled": _STATE["spilled"], "score": round(score(objects), 3),
            "batch_cm": [round(100 * float(v), 2) for v in _STATE.get("centroid", np.zeros(3))],
            "rub_cm": [round(100 * float(v), 2) for v in np.percentile(_STATE["rubbed"][FLOUR], [50, 90, 100])],
            "flour_moved_cm": round(100 * float(_STATE.get("flour_moved", 0.0)), 2),
            "spoon_root": [[round(float(v), 4) for v in a] for a in _STATE.get("spoon_root", (np.zeros(3), np.zeros(4)))]}


def demo(objects, seed):
    return {}
