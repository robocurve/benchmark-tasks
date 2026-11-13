"""Cold-chain receiving check. On the station table: three identical sealed steel cans of a refrigerated food
(7.3 cm across, 11 cm tall, 450 g) stand in a line on the robots' left, 15 cm left of the midline, 20, 28.5 and 37
cm in front of the arms, each with a coloured dot on its lid (blue nearest the robots, then green, then yellow). On
the midline 25 cm out lies the grey plate of a contact thermometer (10 x 10 cm); a can standing on it, not held, is
measured, and the reading is shown in degrees C (one decimal) by a red three-digit LED display on a dark box beyond
the plate. A red reject mat (12 x 12 cm) lies on the robots' right.

One can has warmed up (hidden which, and by how much). The plate's sensor follows the can standing on it slowly
(first-order lag, time constant 4 s) and each display update, every 0.5 s, adds about 0.8 C of noise; with the plate
empty, or the can still held, the sensor relaxes toward room temperature (22 C).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out        (script.txt; WARMCUP_SEED picks the hidden draw, default 0)
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import _part_state, on_reset, on_step, part  # noqa: E402
from seg7 import Seg7  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "One of these three chilled cans from the same delivery has warmed up. Find the warmest can with the thermometer "
    "and put it on the red reject mat, then let go. A can is measured while it stands on the grey plate and is not "
    "held; the display beside the plate shows the reading in degrees C.")

# -- layout ------------------------------------------------------------------------------------------------------------
STEEL, DARK, GREY, RED = (0.78, 0.79, 0.80), (0.08, 0.08, 0.09), (0.55, 0.56, 0.58), (0.80, 0.12, 0.10)
DOTS = {"can_blue": (0.15, 0.35, 0.85), "can_green": (0.15, 0.65, 0.25), "can_yellow": (0.95, 0.80, 0.10)}
CAN_R, CAN_H, CAN_MASS = 0.0365, 0.11, 0.45
HOME = {"can_blue": (0.20, 0.15), "can_green": (0.285, 0.15), "can_yellow": (0.37, 0.15)}
CANS = tuple(HOME)
PAD_XY, PAD_HALF, PAD_H = (0.25, 0.0), 0.05, 0.004
MAT_XY, MAT_HALF = (0.30, -0.16), 0.06
DISP_XY = (0.38, 0.0)
DISP_BOX = (0.08, 0.13, 0.04)     # housing x, y, z
DISP = Seg7(centre=(0.0, 0.0, DISP_BOX[2] + 0.001), digits=3, height=0.045, dp_after=2, up="+x")


def _can(name):
    return part(name, pos=(*HOME[name], TOP), mass=CAN_MASS, color=STEEL, friction=0.9,
                cylinders=[((0.0, 0.0, CAN_H / 2), CAN_R, CAN_H, STEEL)],
                boxes=[((0.0, 0.0, CAN_H + 0.0005), (0.022, 0.022, 0.001), DOTS[name])])


_housing = [((0.0, 0.0, DISP_BOX[2] / 2), DISP_BOX, (0.20, 0.20, 0.22)),
            ((0.0, 0.0, DISP_BOX[2] + 0.0003), (DISP_BOX[0] - 0.01, DISP_BOX[1] - 0.01, 0.0006), DARK)]
DISPLAY = part("display", pos=(*DISP_XY, TOP), mass=6.0, color=DARK, friction=0.9,
               boxes=[*_housing, *DISP.boxes(first=len(_housing))])
PAD = Box(name="pad", pos=(*PAD_XY, TOP + PAD_H / 2), size=(2 * PAD_HALF, 2 * PAD_HALF, PAD_H), color=GREY)
MAT = Box(name="mat", pos=(*MAT_XY, TOP + 0.001), size=(2 * MAT_HALF, 2 * MAT_HALF, 0.002), color=RED)
OBJECTS = [*(_can(n) for n in CANS), DISPLAY, PAD, MAT]

# -- hidden draw (fixed per seed, so a trial can be reproduced) ---------------------------------------------------------
ROOM, REFRESH, DT = 22.0, 0.5, 0.05
TAU = float(os.environ.get("WARMCUP_TAU", "4.0"))         # s, sensor time constant
SIGMA = float(os.environ.get("WARMCUP_SIGMA", "0.8"))     # C, reading noise per display update
TOP_RANGE = (6.0, 9.0)                                     # C, the warmed can
GAP = tuple(float(x) for x in os.environ.get("WARMCUP_GAP", "1.5,2.0").split(","))  # C, between neighbours in rank
SEED = int(os.environ.get("WARMCUP_SEED", "0"))


def draw(seed: int) -> dict[str, float]:
    rng = np.random.default_rng(1000 + seed)
    top = float(rng.uniform(TOP_RANGE[0], TOP_RANGE[1]))
    g1, g2 = (float(rng.uniform(*GAP)) for _ in range(2))
    temps = [top, top - g1, top - g1 - g2]
    order = rng.permutation(3)
    return {CANS[int(order[i])]: temps[i] for i in range(3)}


TEMPS = draw(SEED)
WARMEST = max(TEMPS, key=TEMPS.get)

# -- scoring rules (pure functions of the displayed readings; shared with checks.py) --------------------------------------
MIN_SETTLED = 12
Z_PROOF = 3.0


def settled_from(times, values, min_n: int = MIN_SETTLED):
    """Earliest index i from which the display has stopped drifting: a straight line through all of readings[i:]
    (at least ``min_n`` of them), and one through the first ``min_n`` of them, both have slopes within 2 of their
    own standard errors of zero. None if there is no such i. Uses only displayed values and their times."""
    def flat(t, v):
        t = np.asarray(t, float) - t[0]
        v = np.asarray(v, float)
        slope, icpt = np.polyfit(t, v, 1)
        resid = v - (slope * t + icpt)
        s2 = float(resid @ resid) / (len(v) - 2)
        se = math.sqrt(s2 / float(((t - t.mean()) ** 2).sum())) if s2 > 0 else 0.0
        return abs(slope) <= 2 * se

    n = len(values)
    for i in range(0, n - min_n + 1):
        if flat(times[i:i + min_n], values[i:i + min_n]) and flat(times[i:], values[i:]):
            return i
    return None


def winner_z(samples: dict[str, list[float]]):
    """(proven warmest or None, z): the highest mean beats every other can by more than Z_PROOF standard errors of
    the difference, with the SD pooled from the readings' own scatter about their can's mean."""
    if any(len(v) < 2 for v in samples.values()):
        return None, 0.0
    means = {c: float(np.mean(v)) for c, v in samples.items()}
    resid = np.concatenate([np.asarray(v) - means[c] for c, v in samples.items()])
    dof = len(resid) - len(samples)
    sd = float(np.sqrt(resid @ resid / dof)) if dof > 0 else 0.0
    top = max(means, key=means.get)
    if sd == 0:
        return top, float("inf")
    z = min((means[top] - means[c]) / (sd * math.sqrt(1 / len(samples[top]) + 1 / len(samples[c])))
            for c in samples if c != top)
    return (top if z > Z_PROOF else None), float(z)


def sessions(readings):
    """Stretches of consecutive readings with one can standing untouched on the pad: [(can, [(t, value), ...])]."""
    out = []
    for t, can, v in readings:
        if can is None:
            continue
        if out and out[-1][0] == can and t - out[-1][1][-1][0] <= REFRESH + 1e-6:
            out[-1][1].append((t, v))
        else:
            out.append((can, [(t, v)]))
    return out


def settled(readings) -> dict[str, list[float]]:
    out = {c: [] for c in CANS}
    for can, rs in sessions(readings):
        i = settled_from([r[0] for r in rs], [r[1] for r in rs])
        if i is not None:
            out[can] += [r[1] for r in rs[i:]]
    return out


# -- simulation of the thermometer ----------------------------------------------------------------------------------------
TOUCH = 0.09    # a gripper TCP closer than this to a can's centre counts as holding or touching it
_STATE: dict = {}


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(t=0.0, sensor=ROOM, next=0.0, readings=[], shown=None, failed=False,
                  noise=np.random.default_rng(2000 + SEED), near=set(), gate=None, disp0=None)
    DISP.reset_cache()
    if environment is not None:
        _show(ROOM)


_reset()


def _show(value: float) -> None:
    tenths = min(999, max(0, int(round(value * 10))))
    text = f"{tenths:3d}" if tenths >= 100 else f" {tenths // 10:1d}{tenths % 10:1d}"   # " 68" shows 6.8
    DISP.show("display", text)
    _STATE["shown"] = text


def _tcps(environment):
    from yam_arena import ik, station

    base = environment.unwrapped
    out = []
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        q = np.asarray(getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].detach().cpu())[:6]
        out.append(ik.tcp_pose(q, station.BASES[side])[0])
    return out


def _upright(st) -> bool:
    return st.rotation()[2, 2] > math.cos(math.radians(10))


def on_pad(st) -> bool:
    return (abs(st.pos[0] - PAD_XY[0]) <= PAD_HALF and abs(st.pos[1] - PAD_XY[1]) <= PAD_HALF
            and abs(st.pos[2] - (TOP + PAD_H)) < 0.006 and _upright(st))


def on_mat(st) -> bool:
    return (abs(st.pos[0] - MAT_XY[0]) <= MAT_HALF and abs(st.pos[1] - MAT_XY[1]) <= MAT_HALF
            and abs(st.pos[2] - (TOP + 0.002)) < 0.006 and _upright(st))


@on_step
def _thermometer(environment) -> None:
    sts = {n: _part_state(environment, s) for n, s in ((o.name, o) for o in OBJECTS[:3])}
    tcps = _tcps(environment)
    near = {n for n, st in sts.items()
            if any(np.linalg.norm(tcp - (st.pos + [0, 0, CAN_H / 2])) < TOUCH for tcp in tcps)}
    _STATE["near"] = near
    on = [n for n, st in sts.items() if on_pad(st) and n not in near]
    can = on[0] if len(on) == 1 else None
    target = TEMPS[can] if can else ROOM
    _STATE["sensor"] += (target - _STATE["sensor"]) * (1 - math.exp(-DT / TAU))
    _STATE["t"] += DT
    t = _STATE["t"]
    if t + 1e-9 >= _STATE["next"]:
        _STATE["next"] += REFRESH
        value = round(_STATE["sensor"] + float(_STATE["noise"].normal(0.0, SIGMA)), 1)
        _show(value)
        _STATE["readings"].append((round(t, 3), can, value))


# -- score --------------------------------------------------------------------------------------------------------------
TIP = math.radians(30)        # a can leaning more than this has fallen over
DISP_MOVE = 0.01              # the display box pushed more than this


@on_step
def _gates(environment) -> None:
    """Latched zero gates: a can knocked over or off the table, or the display box shoved, at any time."""
    for spec in OBJECTS[:3]:
        st = _part_state(environment, spec)
        if st.rotation()[2, 2] < math.cos(TIP) or st.pos[2] < TOP - 0.05:
            _STATE["gate"] = _STATE.get("gate") or f"{spec.name} fell over or off the table"
    d = _part_state(environment, DISPLAY)
    if _STATE.get("disp0") is None:
        _STATE["disp0"] = d.pos.copy()
    if np.linalg.norm(d.pos[:2] - _STATE["disp0"][:2]) > DISP_MOVE:
        _STATE["gate"] = _STATE.get("gate") or "display box moved"


def _update(objects):
    """(score, success) from the whole-run record of displayed readings and the state now."""
    if _STATE.get("gate"):
        return 0.0, False
    mat = [n for n in CANS if on_mat(objects[n])]
    if any(n != WARMEST for n in mat):
        return 0.0, False                        # a can that is still cold rejected: gate
    s = settled(_STATE["readings"])
    if not all(s[c] for c in CANS):
        return 0.0, False
    proven, _ = winner_z(s)
    if proven != WARMEST:
        return 0.0, False                        # readings taken but not a proof: no credit for effort
    if mat != [WARMEST]:
        return 0.5, False
    released = WARMEST not in _STATE["near"] and objects[WARMEST].resting()
    return (1.0, True) if released else (0.75, False)


def success(objects) -> bool:
    """The readings prove which can is warmest, it alone stands on the red mat, released and at rest; held 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """0 if a gate tripped (a can fell over or off the table, the display box moved more than 1 cm, or a can other
    than the warmest is on the mat at the end). Otherwise: 0 unless the settled readings prove the warmest can; 0.5
    they prove it; 0.75 it also stands alone on the red mat; 1.0 also released (both grippers'
    TCPs at least 9 cm from its centre) and at rest. Doing nothing scores 0."""
    return _update(objects)[0]


def demo():
    """Open-loop plans for Inspect's ``yam_scripted`` policy, used only for baseline checks (checks.md), chosen by
    WARMCUP_INSPECT_PLAN: "none" (default: both arms hold their ready pose, i.e. do nothing) or "one_can" (the left
    arm sets the blue can on the plate and leaves it there). The full-score demo is script.txt (closed loop)."""
    from yam_arena.demo import pick_place_waypoints

    if os.environ.get("WARMCUP_INSPECT_PLAN", "none") == "one_can":
        x, y = HOME["can_blue"]
        grip = TOP + CAN_H - 0.03
        return {"left": pick_place_waypoints((x, y, grip), (*PAD_XY, grip + PAD_H + 0.003), travel_z=grip + 0.10)}
    return {}


def report(objects) -> dict:
    s = settled(_STATE["readings"])
    proven, z = winner_z(s)
    return {"seed": SEED, "temps": {k: round(v, 2) for k, v in TEMPS.items()}, "warmest": WARMEST, "proven": proven,
            "z": round(z, 2), "settled_n": {c: len(v) for c, v in s.items()},
            "settled_mean": {c: round(float(np.mean(v)), 2) if v else None for c, v in s.items()},
            "on_mat": [n for n in CANS if on_mat(objects[n])], "gate": _STATE.get("gate"),
            "n_readings": len(_STATE["readings"]), "rubric_score": _update(objects)[0],
            "cans": {n: {"pos_mm": [round(float(v) * 1000, 1) for v in objects[n].pos],
                         "tilt_deg": round(math.degrees(math.acos(min(1.0, objects[n].rotation()[2, 2]))), 1),
                         "resting": bool(objects[n].resting()), "held_or_touched": n in _STATE["near"]}
                     for n in CANS},
            "readings": _STATE["readings"]}
