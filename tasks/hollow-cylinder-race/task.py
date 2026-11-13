"""On the station table: a wooden two-lane ramp lies along the table on the robot's left half and runs downhill toward
the right arm. Its two lanes are 30 and 40 cm in front of the arm bases. An orange start gate crosses both lanes near
the ramp top; an orange push button in front of the ramp opens and closes it. Past the foot of the ramp the lanes run
flat over a black finish line to a blue foam stop. Three grey cylinders (5 cm across, 6 cm long, 211 g) with a
coloured band round each end (red, green, blue) lie in cradles in front of the ramp, axes pointing away from the
robot. One is a thin-walled brass tube, one a thick-walled aluminium tube and one a solid glass-epoxy rod; their
ends are capped and they look and weigh the same. A red bin stands in front of the foam stop.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import Slider, _ENV, _part_state, on_reset, on_step, part, set_target, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
DT = 1 / 20  # control step (s)

INSTRUCTION = (
    "The three grey cylinders with red, green and blue end bands look the same and weigh the same. One is a "
    "thin-walled tube, one a thick-walled tube and one a solid rod. Put the thin-walled tube in the red bin and "
    "leave the other two out of it. The orange button opens and closes the start gate at the top of the two-lane "
    "ramp.")

WOOD, DARKWOOD = (0.76, 0.60, 0.42), (0.55, 0.42, 0.28)
GREY, BAND_GREY, BLACK = (0.62, 0.64, 0.68), (0.62, 0.64, 0.68), (0.05, 0.05, 0.05)
WHITE = (0.95, 0.95, 0.95)
ORANGE, RED, BLUE_FOAM, DARK = (0.95, 0.55, 0.10), (0.80, 0.08, 0.08), (0.30, 0.45, 0.65), (0.22, 0.22, 0.24)
BAND = {"red": (0.85, 0.10, 0.10), "green": (0.10, 0.65, 0.20), "blue": (0.12, 0.30, 0.85)}
NAMES = ("red", "green", "blue")

# -- cylinders ---------------------------------------------------------------------------------------------------------
R = 0.025          # outer radius (m)
CYL_L = 0.060      # length
MASS = 0.211       # kg, all three
BAND_W = 0.010     # coloured tape band at each end
# I_axis = k m R^2.  Real build (see README): thin-walled brass tube (wall 2.9 mm) k = 0.88; thick-walled aluminium tube
# (bore 28.2 mm) k = 0.65; solid glass-epoxy (G10) rod k = 0.50; 2 mm ABS caps on all three. Same outer size, mass, finish.
K_OF = {"thin": 0.88, "thick": 0.65, "solid": 0.50}


def inertia(kind: str) -> tuple[float, float, float]:
    k = K_OF[kind]
    i_ax = k * MASS * R * R
    i_t = MASS * (6 * k * R * R + CYL_L * CYL_L) / 12  # tube: (3 (R^2 + r_i^2) + L^2) / 12 with R^2 + r_i^2 = 2 k R^2
    return (i_ax, i_t, i_t)


# -- layout (world x forward, y left; z above the table) ---------------------------------------------------------------
THETA = math.radians(15.0)
TAN = math.tan(THETA)
LANE_X = (0.30, 0.40)
TRACK_CX = 0.35
TRACK_W = 0.23
Y_TOP, Y_BOT = 0.25, 0.02       # ramp runs downhill toward -y
Y_GATE = 0.16
FINISH_Y = -0.16
STOP_Y = -0.29                   # far wall of the catch pit
PIT_Y = -0.20                    # near edge of the catch pit
PIT_FLOOR = 0.003                # foam floor of the pit, 12 mm below the running board
BOARD_Y1 = 0.262
BOARD_T = 0.015
WALL_H = 0.012
MU_WALL = 0.05                   # edge rails and gate faced with PTFE (slick) strip
START_YAW_MAX = math.radians(3.0)  # a cylinder counts as a starter only when squared to its lane within 3 deg
GATE_TRAVEL = 0.03

RACK_X = 0.19
CRADLE_Y = {"red": 0.12, "green": 0.02, "blue": -0.08, "spare": -0.18}
BIN_XY, BIN_IN = (0.18, -0.30), 0.10   # inner size
BUTTON_XY = (0.18, 0.30)


def surface(y: float) -> float:
    """Height of the running surface above the table at ``y``."""
    return BOARD_T + float(np.clip(y - Y_BOT, 0.0, Y_TOP - Y_BOT)) * TAN


def _lx(x: float) -> float:
    return x - TRACK_CX


def _track_boxes():
    deg = math.degrees(THETA)
    b = []
    # catch pit: a foam floor 12 mm below the board, so a cylinder that bounces off the far wall cannot roll back out
    b.append(((0.0, (STOP_Y + PIT_Y) / 2, PIT_FLOOR / 2), (TRACK_W, PIT_Y - STOP_Y, PIT_FLOOR), BLUE_FOAM))
    # flat board in strips: wood, finish line, wood (all flush)
    for y0, y1, col in ((PIT_Y, FINISH_Y - 0.003, WOOD),
                        (FINISH_Y - 0.003, FINISH_Y + 0.003, BLACK), (FINISH_Y + 0.003, BOARD_Y1, WOOD)):
        b.append(((0.0, (y0 + y1) / 2, BOARD_T / 2), (TRACK_W, y1 - y0, BOARD_T), col))
    # sloped plank (8 mm), its top face on the running surface
    lp = (Y_TOP - Y_BOT) / math.cos(THETA)
    ym, zm = (Y_TOP + Y_BOT) / 2, surface((Y_TOP + Y_BOT) / 2)
    b.append(((0.0, ym + 0.004 * math.sin(THETA), zm - 0.004 * math.cos(THETA)), (TRACK_W, lp, 0.008), WOOD, deg))
    # support block under the ramp top and an end board above it
    b.append(((0.0, Y_TOP - 0.006, (surface(Y_TOP) - 0.006) / 2), (TRACK_W, 0.012, surface(Y_TOP) - 0.006), DARKWOOD))
    b.append(((0.0, Y_TOP + 0.006, (surface(Y_TOP) + 0.03) / 2), (TRACK_W, 0.008, surface(Y_TOP) + 0.03), DARKWOOD))
    # lane walls: outer near, middle divider, outer far; a flat part and a sloped part each
    # lanes are painted: a white centre line between them (0.3 mm), and low edge rails along both sides of the board
    # that only a badly skewed cylinder reaches
    b.append(((0.0, (PIT_Y + Y_BOT) / 2, BOARD_T + 0.00015), (0.004, Y_BOT - PIT_Y, 0.0003), WHITE))
    b.append(((0.0, ym - 0.0001 * math.sin(THETA), zm + 0.00015 * math.cos(THETA)), (0.004, lp, 0.0003), WHITE, deg))
    walls = [(TRACK_CX - TRACK_W / 2 + 0.002, 0.004), (TRACK_CX + TRACK_W / 2 - 0.002, 0.004)]
    for wx, ww in walls:
        y0, y1 = PIT_Y, Y_BOT + 0.004   # the walls end at the pit, which is open across both lanes
        b.append(((_lx(wx), (y0 + y1) / 2, BOARD_T + WALL_H / 2), (ww, y1 - y0, WALL_H), DARKWOOD, 0.0, MU_WALL))
        n_off = (WALL_H - 0.008) / 2  # from the plank's mid-plane to the wall's centre, along the normal
        b.append(((_lx(wx), ym + 0.004 * math.sin(THETA) - n_off * math.sin(THETA),
                   zm - 0.004 * math.cos(THETA) + n_off * math.cos(THETA) + 0.004),
                  (ww, lp, WALL_H + 0.008), DARKWOOD, deg, MU_WALL))
    # far wall of the pit (5 cm tall), lined with foam
    b.append(((0.0, STOP_Y - 0.004, 0.025), (TRACK_W, 0.008, 0.05), DARKWOOD))
    b.append(((0.0, STOP_Y + 0.003, 0.0165), (TRACK_W, 0.006, 0.027), BLUE_FOAM))
    return b


_GATE_Z = surface(Y_GATE) + 0.011
TRACK = part("track", pos=(TRACK_CX, 0.0, TOP), mass=20.0, color=WOOD, friction=0.9, boxes=_track_boxes(),
             sliders=(Slider("gate", rest=(0.0, Y_GATE, _GATE_Z), boxes=(((0.0, Y_GATE, _GATE_Z), (TRACK_W, 0.006, 0.022),
                                                                          ORANGE, 0.0, MU_WALL),),
                             travel=GATE_TRAVEL, rate=3000.0, damping=60.0, mass=0.05, axis="Z", sign=-1),))


def _rack_boxes():
    b = [((0.0, 0.0, 0.002), (0.08, 0.36, 0.004), DARK)]
    for name, y in CRADLE_Y.items():
        for s in (-1, 1):
            b.append(((0.0, y - 0.0 + s * 0.019, 0.004 + 0.004), (0.07, 0.004, 0.008), (0.30, 0.30, 0.33)))
    return b


RACK = part("rack", pos=(RACK_X, 0.03, TOP), mass=5.0, color=DARK, friction=0.9,
            boxes=[(c, s, col) for (c, s, col) in [((cx, cy - 0.03, cz), sz, cl) for (cx, cy, cz), sz, cl in _rack_boxes()]])

_BW = 0.004
BIN = part("bin", pos=(*BIN_XY, TOP), mass=20.0, color=RED, friction=0.9, boxes=[
    ((0, 0, 0.002), (BIN_IN + 2 * _BW, BIN_IN + 2 * _BW, 0.004), RED),
    ((BIN_IN / 2 + _BW / 2, 0, 0.017), (_BW, BIN_IN + 2 * _BW, 0.03), RED),
    ((-BIN_IN / 2 - _BW / 2, 0, 0.017), (_BW, BIN_IN + 2 * _BW, 0.03), RED),
    ((0, BIN_IN / 2 + _BW / 2, 0.017), (BIN_IN, _BW, 0.03), RED),
    ((0, -BIN_IN / 2 - _BW / 2, 0.017), (BIN_IN, _BW, 0.03), RED)])

PRESS = 0.004
BUTTON = part("button", pos=(*BUTTON_XY, TOP), mass=5.0, color=DARK, friction=0.9, boxes=[
    ((0, 0, 0.015), (0.07, 0.07, 0.03), DARK)],
    sliders=(Slider("cap", rest=(0, 0, 0.040), boxes=(((0, 0, 0.040), (0.036, 0.036, 0.02), ORANGE),),
                    travel=0.008, rate=400.0, damping=10.0, axis="Z", sign=-1),))


BODY_KIND = {"cyl_a": "thin", "cyl_b": "thick", "cyl_c": "solid"}   # fixed per body; colours and places are dealt per trial
CRADLE_Z = 0.012 + math.sqrt(R * R - 0.017 ** 2)                  # centre height resting on the rail edges


def _cylinder(body: str, start: str):
    """One cylinder body with its own fixed inertia. Its collider is grey, with two grey rims where the tape goes;
    over each rim are three coloured tape overlays (visual only), and the reset shows the pair of the colour this
    body wears in the trial."""
    x, y = RACK_X, CRADLE_Y[start]
    shapes = [((0.0, 0.0, 0.0), R - 0.0005, CYL_L - 2 * BAND_W, GREY, "X"),
              ((CYL_L / 2 - BAND_W / 2, 0.0, 0.0), R, BAND_W, GREY, "X"),
              ((-CYL_L / 2 + BAND_W / 2, 0.0, 0.0), R, BAND_W, GREY, "X")]
    for c in NAMES:
        for s in (1, -1):
            shapes.append(((s * (CYL_L / 2 - BAND_W / 2), 0.0, 0.0), R + 0.0003, BAND_W - 0.0002, BAND[c], "X", "visual"))
    return part(body, pos=(x, y, TOP + CRADLE_Z + 0.002), mass=MASS, color=GREY, friction=1.0,
                boxes=[((0.0, 0.0, 0.0), (0.01, 0.01, 0.01), GREY)],     # tiny inner box (a Part needs one)
                cylinders=shapes, inertia=inertia(BODY_KIND[body]), ang_damping=0.0, can_sleep=False)


CYLS = {b: _cylinder(b, start) for b, start in zip(BODY_KIND, NAMES)}
OBJECTS = [TRACK, RACK, BIN, BUTTON, *CYLS.values()]


def body_name(colour: str) -> str:
    """The body wearing this colour's tape bands in the current trial (what the cameras show)."""
    return _STATE["body_of"][colour]

# -- runtime -----------------------------------------------------------------------------------------------------------
GAP_MIN = 0.020        # s: a finish-time gap counts as a result only above this (set from the noise study, checks.md)
RACE_WINDOW = 4.0      # s after the gate opens within which a run is timed
HELD_DIST = 0.06       # a cylinder within 6 cm of a gripper's TCP counts as held
_STATE: dict = {}


def hidden_kinds(seed) -> dict[str, str]:
    """Which cylinder is which, from the trial seed (a random permutation)."""
    rng = np.random.default_rng(0 if seed is None else int(seed))
    kinds = list(rng.permutation(["thin", "thick", "solid"]))
    return dict(zip(NAMES, kinds))


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(step=0, gate_open=False, down=False, presses=0, trials=[], prev={}, failed=False, fell=None, held=set(),
                  kinds=hidden_kinds(_ENV.get("seed")))
    _STATE["body_of"] = {c: next(b for b, k in BODY_KIND.items() if k == _STATE["kinds"][c]) for c in NAMES}
    if environment is not None:
        _deal(environment)
        set_target(environment, "track", "gate", 0.0)



def _deal(environment) -> None:
    """Put each body in the cradle of the colour it wears this trial, at rest, and show that colour's tape bands."""
    from robokit import set_visible

    for ci, c in enumerate(NAMES):
        b = _STATE["body_of"][c]
        write_pose(environment, b, (RACK_X, CRADLE_Y[c], TOP + CRADLE_Z + 0.001), (0, 0, 0, 1))
    for b in BODY_KIND:
        worn = next(c for c in NAMES if _STATE["body_of"][c] == b)
        for ci, c in enumerate(NAMES):
            for k in (3 + 2 * ci, 4 + 2 * ci):
                set_visible(b, f"body/cyl_{k}", c == worn)

_reset()


def thin_name() -> str:
    return next(n for n, k in _STATE["kinds"].items() if k == "thin")


def _tcps(environment):
    from yam_arena import ik, station

    base = environment.unwrapped
    out = []
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        q = np.asarray(getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].detach().cpu())[:6]
        out.append(ik.tcp_pose(q, station.BASES[side])[0])
    return out


def axis_yaw(st) -> float:
    """Yaw of the cylinder's axis (sign-free, in (-90, 90] deg), 0 = pointing away from the robot."""
    a = st.rotation()[:, 0]
    return (math.atan2(a[1], a[0]) + math.pi / 2) % math.pi - math.pi / 2


def start_lane(st) -> int | None:
    """Lane index if the cylinder rests behind the gate in that lane, squared to it."""
    p = st.pos
    for i, x in enumerate(LANE_X):
        if (abs(p[0] - x) < 0.02 and Y_GATE < p[1] < Y_GATE + 0.08 and abs(axis_yaw(st)) <= START_YAW_MAX
                and np.linalg.norm(st.lin_vel) < 0.05):
            return i
    return None


@on_step
def _monitor(environment) -> None:
    s = _STATE
    s["step"] += 1
    t = s["step"] * DT
    states = {n: _part_state(environment, CYLS[body_name(n)]) for n in NAMES}
    tcps = _tcps(environment)
    s["held"] = {n for n, st in states.items() if min(np.linalg.norm(st.pos - c) for c in tcps) < HELD_DIST}
    fell = [n for n, st in states.items() if st.pos[2] < TOP - 0.05]
    if fell and not s["failed"]:
        s["failed"] = True
        s["fell"] = (fell, s["step"])
    # button -> gate
    travel = -_part_state(environment, BUTTON).sliders["cap"]
    if travel >= PRESS and not s["down"]:
        s["down"] = True
        s["presses"] += 1
        s["gate_open"] = not s["gate_open"]
        set_target(environment, "track", "gate", -GATE_TRAVEL if s["gate_open"] else 0.0)
        if s["gate_open"]:
            lanes = {n: start_lane(states[n]) for n in NAMES if n not in s["held"]}
            starters = {n: ln for n, ln in lanes.items() if ln is not None}
            if starters and len(set(starters.values())) == len(starters):
                s["trials"].append({"t0": t, "lanes": starters, "cross": {}, "done": False})
    elif travel < PRESS / 2:
        s["down"] = False
    # finish-line crossings, interpolated between control steps
    for tr in s["trials"]:
        if tr["done"]:
            continue
        for n in tr["lanes"]:
            y_prev, y = s["prev"].get(n, states[n].pos[1]), states[n].pos[1]
            if n not in tr["cross"] and y_prev >= FINISH_Y > y:
                frac = (y_prev - FINISH_Y) / max(1e-9, y_prev - y)
                tr["cross"][n] = round((t - DT + frac * DT) - tr["t0"], 4)
        if len(tr["cross"]) == len(tr["lanes"]) or t - tr["t0"] > RACE_WINDOW:
            tr["done"] = True
    s["prev"] = {n: states[n].pos[1] for n in NAMES}


def slower_pairs() -> set[tuple[str, str]]:
    """(slower, faster) pairs shown by two-lane races, or by timed solo runs, with a gap above GAP_MIN."""
    out = set()
    solo: dict[str, float] = {}
    for tr in _STATE["trials"]:
        c = tr["cross"]
        if len(tr["lanes"]) == 2 and len(c) == 2:
            a, b = tr["lanes"]
            if abs(c[a] - c[b]) > GAP_MIN:
                out.add((a, b) if c[a] > c[b] else (b, a))
        elif len(tr["lanes"]) == 1 and c:
            n = next(iter(c))
            solo[n] = min(solo.get(n, 9.0), c[n])
    for a in solo:
        for b in solo:
            if solo[a] - solo[b] > GAP_MIN:
                out.add((a, b))
    return out


def proven_slowest() -> str | None:
    """The cylinder shown slower than each of the other two, directly or through a chain."""
    rel = slower_pairs()
    for _ in NAMES:
        rel |= {(a, d) for a, b in rel for c, d in rel if b == c and a != d}
    for x in NAMES:
        if all((x, z) in rel for z in NAMES if z != x):
            return x
    return None


def in_bin(objects) -> set[str]:
    out = set()
    for n in NAMES:
        p = objects[body_name(n)].pos
        if (n not in _STATE["held"] and abs(p[0] - BIN_XY[0]) < BIN_IN / 2 and abs(p[1] - BIN_XY[1]) < BIN_IN / 2
                and p[2] < TOP + 0.06):
            out.add(n)
    return out


def _update(objects) -> tuple[float, bool]:
    if _STATE["failed"]:
        return 0.0, False
    binned = in_bin(objects)
    thin = thin_name()
    if binned - {thin}:
        return 0.0, False                   # a wrong cylinder in the bin: the wrong part passed on
    if proven_slowest() != thin:
        return 0.0, False
    still = all(objects[body_name(n)].resting() for n in NAMES)
    if binned == {thin} and still and not _STATE["held"]:
        return 1.0, True
    return 0.5, False


def success(objects) -> bool:
    """The thin-walled tube shown slowest by the timed runs, alone in the bin, released, everything at rest; held 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 on success; 0.5 if the runs show the thin-walled tube slowest but it is not (yet) alone in the bin; 0 if
    another cylinder is in the bin, a cylinder left the table, or the runs do not show it."""
    return _update(objects)[0]


def report(objects) -> dict:
    """Plain summary for result.json (scorer's view)."""
    return {"kinds": dict(_STATE["kinds"]), "thin": thin_name(), "trials": _STATE["trials"],
            "slower_pairs": sorted(slower_pairs()), "proven_slowest": proven_slowest(), "in_bin": sorted(in_bin(objects)),
            "presses": _STATE["presses"], "failed": _STATE["failed"], "fell": _STATE.get("fell"),
            "final_pos": {n: [round(float(v), 3) for v in objects[body_name(n)].pos] for n in NAMES}}


def debug_line(objects) -> str:
    """One status line for debugging runs (DEMO_DEBUG=1); not used by any policy."""
    s = _STATE
    pos = " ".join(f"{n}=({objects[body_name(n)].pos[0]:.3f},{objects[body_name(n)].pos[1]:.3f},{objects[body_name(n)].pos[2] - TOP:.3f})"
                   for n in NAMES)
    return (f"presses={s['presses']} gate_open={s['gate_open']} held={sorted(s['held'])} trials="
            f"{[(t['lanes'], t['cross']) for t in s['trials']]} {pos}")
