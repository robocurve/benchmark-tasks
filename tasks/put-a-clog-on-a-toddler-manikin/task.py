"""On the station table (standing in for a nursery floor): a toddler-size care manikin sits on a child's chair facing the
robot, its feet dangling 7.5 cm above the floor. Its left foot already wears a foam clog. The other clog of the pair
stands on the floor in front of it, its heel strap back on the heel rim, the way a worn clog is left.

The job is what a nursery worker or a parent does many times a day: put the clog on the bare right foot, with the heel
strap behind the heel, and let go.

The strap makes the back of the clog a ring: the foot cannot be slid in from behind (the strap is in the way) or from
the front (the vamp is). It has to go in from above, toes first and pointing down, and the heel has to come down in
front of the strap. The manikin's lower leg swings freely at the knee, so its foot only points down while something
holds the knee bent: one hand bends the knee back and controls it while the other brings the clog under the toes and
follows the foot forward as the knee is straightened.

The clog's place changes with ``CLOG_LOT`` (A or B). ``CLOG_STRAP=forward`` starts the loose clog with its strap swung
forward onto the vamp (for the control run that slips the clog on without the strap).

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

import clogkit as K  # noqa: E402
from clogkit import Clog, Leg, Manikin, Strap, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Put the clog on the manikin's bare right foot, with the clog's heel strap behind the heel, then let go "
               "of the clog and the leg.")

# -- layout (m; x away from the robot, y left, heights above the table) ---------------------------------------------------
LOT = os.environ.get("CLOG_LOT", "A")
KNEE = np.array([0.400, 0.000, 0.300])          # the middle of the manikin's right knee
CLOG_XY = np.array({"A": (0.105, 0.000), "B": (0.125, -0.045)}[LOT])   # the loose clog's inside toe end
STRAP_FORWARD = os.environ.get("CLOG_STRAP", "back") == "forward"
STRAP_ON_VAMP = math.radians(-152.0)             # the strap's turn when it lies forward on the vamp
SEAT_TOP = KNEE[2] - 0.036                       # the thighs lie on the seat
SEAT_FRONT = KNEE[0] + 0.110                     # the chair's front edge: the manikin sits at the front of the seat
SEAT = (0.230, 0.340, 0.015)                     # depth, width, thickness
CHAIR_Y = KNEE[1] - K.LEG_GAP / 2
WOOD = (0.76, 0.62, 0.44)

MANIKIN, LEG, CLOG, STRAP = "manikin", "leg", "clog", "strap"

# -- scoring -----------------------------------------------------------------------------------------------------------
TOES_IN = 0.045          # the toes are under the vamp once their tip is this close to the inside toe end
TOES_SIDE = 0.020        # ... and this close to the clog's midline
TOES_UP = 0.035          # ... and no higher than this above the footbed
TOES_DOWN = 0.010        # ... nor more than this below its level (a clog that hangs on the foot tilts heel down,
                         # which carries the level of its footbed above the toes' tip)
EDGE_SLACK = 0.005       # the shank's axis may come up this far in front of the vamp's back edge
SIDE = 0.025             # ... and this far off the clog's midline
ANKLE_UP = 0.075         # the ankle's middle is at most this far above the footbed (the sole within 4 cm of it)
OFF_FLOOR = 0.005        # the clog hangs clear of the floor by at least this
CLEAR = 0.030            # both grippers this far from the clog and the leg at the end
STILL, STILL_TURN = 0.02, 0.2   # m/s for the clog, rad/s for the leg


def clog_start() -> np.ndarray:
    return np.array([CLOG_XY[0], CLOG_XY[1], TOP + K.SOLE_T + 0.0005])


def knee_world() -> np.ndarray:
    return np.array([KNEE[0], KNEE[1], TOP + KNEE[2]])


def _chair() -> list:
    x0, top = SEAT_FRONT, TOP + SEAT_TOP
    parts = [Box("seat", pos=(x0 + SEAT[0] / 2, CHAIR_Y, top - SEAT[2] / 2), size=SEAT, color=WOOD),
             Box("chair_back", pos=(x0 + SEAT[0] - 0.0075, CHAIR_Y, top + 0.130), size=(0.015, SEAT[1], 0.260), color=WOOD)]
    leg_h = SEAT_TOP - SEAT[2]
    for i, dx in enumerate((0.030, SEAT[0] - 0.030)):
        for j, s in enumerate((-1.0, 1.0)):
            parts.append(Box(f"chair_leg_{i}{j}", pos=(x0 + dx, CHAIR_Y + s * (SEAT[1] / 2 - 0.015), TOP + leg_h / 2),
                             size=(0.030, 0.030, leg_h), color=WOOD))
    return parts


def _objects() -> list:
    k, c = knee_world(), clog_start()
    manikin = Manikin(MANIKIN, pos=tuple(float(v) for v in k), size=(0.30, 0.30, 0.75), leg=LEG)
    leg = Leg(LEG, pos=tuple(float(v) for v in k), size=(0.13, 0.06, 0.26))
    clog = Clog(CLOG, pos=tuple(float(v) for v in c), size=(0.16, 0.08, 0.08), strap=STRAP)
    pivot = c + (K.PIVOT[0], 0.0, K.PIVOT[1])
    strap = Strap(STRAP, pos=tuple(float(v) for v in pivot), size=(0.07, 0.09, 0.01),
                  quat=K.turn_y(STRAP_ON_VAMP if STRAP_FORWARD else K.STRAP_REST))
    return [*_chair(), manikin, leg, clog, strap]


OBJECTS = _objects()


# -- what the trial looks like now ------------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(steps=0, toes=False, through=False, off_floor=False, released=False, still=False, flex=0.0, flex_max=0.0,
                  toe=(0.0, 0.0, 0.0), heel_x=0.0, cross=(0.0, 0.0), bar=(0.0, 0.0), ankle_up=0.0, lean=0.0, low=0.0)


_clear()


def toes_under(f: dict) -> bool:
    """The toes are under the vamp."""
    toe = f["toe"]
    return bool(toe[0] <= TOES_IN and abs(toe[1]) <= TOES_SIDE and -TOES_DOWN <= toe[2] <= TOES_UP)


def through_ring(f: dict) -> bool:
    """The leg comes up through the clog's opening in front of the strap, the heel in front of the crossbar, the foot
    down in the clog."""
    cx, cy = f["cross"]
    bar_front = f["bar"][0] - K.ROD
    return bool(f["shank_up"] and K.VAMP - EDGE_SLACK <= cx <= bar_front and abs(cy) <= SIDE
                and f["heel"][0] <= bar_front and f["ankle_up"] <= ANKLE_UP)


def score_from(toes: bool, through: bool, off_floor: bool, released: bool, still: bool) -> float:
    """Pure scoring rule. ``toes``: the toes are under the vamp. ``through``: the leg comes up through the opening in
    front of the heel strap, with the foot down in the clog. ``off_floor``: the clog hangs clear of the floor.
    ``released``: both grippers are clear of the clog and the leg. ``still``: clog and leg are at rest."""
    if not toes:
        return 0.0
    on_foot = off_floor and released and still
    if through:
        return 1.0 if on_foot else 0.75
    return 0.5 if on_foot else 0.25


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    st = object_states(environment, [LEG, CLOG, STRAP])
    leg, clog, strap = (st[n] for n in (LEG, CLOG, STRAP))
    f = K.fit(clog.pos, clog.quat_xyzw, leg.pos, leg.quat_xyzw, strap.pos, strap.quat_xyzw)
    s["toes"], s["through"] = toes_under(f), through_ring(f)
    s["low"] = float(clog.low[2]) - TOP
    s["off_floor"] = bool(s["low"] >= OFF_FLOOR)
    hands = [p for side in K.gripper_points(environment) for p in side]
    lo, hi = K._local_bounds(K._SPECS[CLOG])
    near_clog = min(float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - q, q - hi))))
                    for q in (K.to_local(p, clog.pos, clog.quat_xyzw) for p in hands))
    knee, ankle = np.asarray(leg.pos, float), K.place(leg.pos, leg.quat_xyzw, (0.0, 0.0, -K.SHANK))
    heel = K.place(leg.pos, leg.quat_xyzw, (K.HEEL_BACK, 0.0, K.SOLE + 0.02))
    toe = K.place(leg.pos, leg.quat_xyzw, (K.TOE_X, 0.0, K.SOLE + 0.01))
    near_leg = min(min(K.segment_distance(p, knee, ankle), K.segment_distance(p, heel, toe)) for p in hands) - 0.032
    s["released"] = bool(near_clog >= CLEAR and near_leg >= CLEAR)
    s["still"] = bool(float(np.linalg.norm(clog.lin_vel)) < STILL and float(np.linalg.norm(leg.ang_vel)) < STILL_TURN)
    s["flex"] = K.knee_flex(leg.quat_xyzw)
    s["flex_max"] = max(s["flex_max"], s["flex"])
    s["toe"], s["heel_x"], s["cross"], s["bar"] = tuple(f["toe"]), float(f["heel"][0]), tuple(f["cross"]), tuple(f["bar"])
    s["ankle_up"], s["lean"] = f["ankle_up"], f["lean"]


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """Scored at the end of the trial, in the clog's own frame (x back from its inside toe end, z up from its footbed).
    0 unless the toes are under the vamp: the tip of the toes within 4.5 cm of the inside toe end, within 2 cm of the
    clog's midline, and between 1 cm below and 3.5 cm above the level of the footbed. Then 0.25. 0.5: and the clog
    hangs on the foot: it is at least 5 mm clear of the floor, at rest (under 2 cm/s, the leg under 0.2 rad/s), and both grippers are at least
    3 cm from clog and leg. 0.75 instead: the leg comes up through the opening in front of the heel strap (the shank's
    axis, at the height of the strap's pivot, lies between 5 mm in front of the vamp's back edge and the front of the
    strap's crossbar, within 2.5 cm of the midline; the heel's back is in front of the crossbar; the ankle's middle is
    no more than 7.5 cm above the footbed), but the clog is still held, moving or on the floor. 1.0: through the
    opening in front of the strap, and hanging on the foot, let go and at rest."""
    s = _STATE
    return score_from(s["toes"], s["through"], s["off_floor"], s["released"], s["still"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    s = _STATE
    mm = lambda v: [round(1000 * float(x), 1) for x in v]   # noqa: E731
    return {"score": score(objects), "lot": LOT, "toes_under": s["toes"], "through_ring": s["through"],
            "off_floor": s["off_floor"], "released": s["released"], "still": s["still"], "toe_mm": mm(s["toe"]),
            "heel_x_mm": round(1000 * s["heel_x"], 1), "cross_mm": mm(s["cross"]), "bar_mm": mm(s["bar"]),
            "ankle_up_mm": round(1000 * s["ankle_up"], 1), "lean_deg": round(math.degrees(s["lean"]), 1),
            "clog_low_mm": round(1000 * s["low"], 1), "knee_deg": round(math.degrees(s["flex"]), 1),
            "knee_max_deg": round(math.degrees(s["flex_max"]), 1)}


def demo(objects=None, seed=None):
    return {}
