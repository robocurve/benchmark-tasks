"""An emergency-stop button stands head up on the bench, its locking ring two turns on the end of its thread; a steel
panel stands upright, side-on, on the midline, with a 22.3 mm hole and the notch for the button's tab at the top of it.
The ring has to come off, the button go through the hole from the front with its tab in the notch, and the ring go back
on behind the panel and be turned on by hand until it is hand-tight against the panel.

How it is built. The panel is fixed; its hole is a ring of sixteen staves round a 22.3 mm bore with the notch (3.2 mm
wide) cut into the top one. The button (the operator) is one free body: mushroom head, bezel, gasket, threaded barrel
with its tab, and a plain stem behind the thread. The ring is one free body of twelve staves. While the ring is on the
thread it is welded ("on") to a hidden turner that turns freely ("turn") on a hidden carrier, which slides along the
barrel ("along"); every control step the carrier's stops are put where the turn has brought it, 1 mm a turn: the
thread. Unscrewed past the thread's end the weld is cut and the ring is free; a free ring held on the barrel's end in
line with it (within 1 mm and 5 degrees) and turned on while it is not drawn away is welded on again there
("on_again"). When the ring's front face reaches the panel's back on a seated operator it leaves the thread: it is
welded ("clamp") to a hidden tightener hinged to the world on the hole's axis there, and the seat's stops close at
the panel (the ring drawn against the panel's back pulls the flange home); the tightener turns on against a spring
("tight"): the tightening, read in the scene at 2.05 N m/rad. Out of the panel the ring runs to the tab's end, where
the thread runs out, and turns there without going further. While no hand is closed on the ring the tightener is
welded where it stands ("set": the thread's friction holds a tightened ring); the most it was turned is kept. The
operator enters the hole only on a hidden seat that slides along the hole's axis ("seat"): it is welded to it
("seated") when its barrel arrives in line with the hole (within SEAT_OFF and SEAT_TILT) with its tab within
SEAT_TURN of the notch, and let go when it has been pulled back out.
"""

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder, hinge, slide, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = (
    "Mount the emergency-stop button in the panel hole with its tab in the notch, then screw its locking ring on from "
    "behind until it is hand-tight, and let go."
)
RUN_LIMIT_STEPS = 2500  # 125 s at 20 Hz: the recorded full run f1b (101.05 s) plus 24 %
TOP = TABLE_TOP[SCENE]

RED, BLACK, DARK, RUBBER, STEEL, PANEL, WHITE = (0.80, 0.06, 0.05), (0.07, 0.07, 0.08), (0.20, 0.20, 0.22), (0.12, 0.12, 0.13), (0.62, 0.64, 0.67), (0.78, 0.79, 0.77), (0.95, 0.95, 0.93)
GRAVITY = 9.81
X, Y, Z = np.eye(3)


def _q(columns) -> tuple:
    return mk.matrix_to_quat(np.column_stack(columns))


def _staves(inner: float, outer: float, low: float, high: float, count: int, axes=(X, Y, Z), offset: float = 0.0, skip=(), sides=None) -> list:
    """Boxes round axis ``axes[2]`` between ``inner`` and ``outer`` radius (flats), from ``low`` to ``high`` along it;
    stave k is centred at angle offset + 2 pi k / count from ``axes[0]`` towards ``axes[1]``. ``sides``: stave ->
    (from, to), its extent across (m, towards growing angle) where it is not the usual one, centred."""
    a0, a1, a2 = (np.asarray(a, dtype=float) for a in axes)
    width = 2 * outer * math.tan(math.pi / count) * 1.05
    shapes = []
    for k in range(count):
        if k in skip:
            continue
        a = offset + 2 * math.pi * k / count
        across = (sides or {}).get(k, (-width / 2, width / 2))
        radial, tangent = math.cos(a) * a0 + math.sin(a) * a1, -math.sin(a) * a0 + math.cos(a) * a1
        middle = (inner + outer) / 2 * radial + (across[0] + across[1]) / 2 * tangent + (low + high) / 2 * a2
        shapes.append(box((outer - inner, across[1] - across[0], high - low), pos=tuple(middle), quat=_q([radial, tangent, a2])))
    return shapes


# --- the panel: a 2 mm steel plate 110 x 230 mm, upright on the midline in a fixed frame with two feet, side-on to the
# robot: its front (y -) faces the right arm, its back (y +) the left arm. Hole (IDEC HW: 22.3 mm, the notch 3.2 mm wide,
# 24.1 mm across hole and notch) centred at (0.25, 0), 16 cm over the bench, the notch at the top ---
HOLE = np.array([0.25, 0.0, TOP + 0.16])
PANEL_T, PANEL_W, PANEL_H = 0.002, 0.110, 0.230
HOLE_R, NOTCH_W, NOTCH_TO = 0.02230 / 2, 0.0032, 0.0241 - 0.02230 / 2  # the notch reaches 12.95 mm over the hole's centre
FRONT_Y, BACK_Y = -PANEL_T / 2, PANEL_T / 2
WINDOW = 0.016  # half the square left open round the hole by the plate's four outer parts; the staves fill it
_plate_axes = (X, Z, -Y)  # the hole's staves: radial in the plate's plane (x, z), along the hole's axis (a right-handed frame)
_TOP_STAVE = 4  # of sixteen staves from +x towards +z: the one at the top, where the notch is
_STAVE_W = 2 * 0.0235 * math.tan(math.pi / 16) * 1.05
# The staves either side of the notch, cut back on the notch's side: square boxes overlap their neighbours at the bore,
# and these two reached 0.27 mm past the notch's middle (run e1d: the tab stopped 2 mm short of seated). Their corners
# at the bore now stand at the notch's sides, 1.6 mm from its middle.
_NEXT = math.radians(67.5)
_CUT = (HOLE_R * math.cos(_NEXT) - NOTCH_W / 2) / math.sin(_NEXT)  # m across from the stave's middle to its notch-side edge
_panel = [
    box((PANEL_W / 2 - WINDOW, PANEL_T, PANEL_H), pos=(-(PANEL_W / 2 + WINDOW) / 2, 0.0, 0.0)),
    box((PANEL_W / 2 - WINDOW, PANEL_T, PANEL_H), pos=((PANEL_W / 2 + WINDOW) / 2, 0.0, 0.0)),
    box((2 * WINDOW, PANEL_T, PANEL_H / 2 - WINDOW), pos=(0.0, 0.0, -(PANEL_H / 2 + WINDOW) / 2)),
    box((2 * WINDOW, PANEL_T, PANEL_H / 2 - WINDOW), pos=(0.0, 0.0, (PANEL_H / 2 + WINDOW) / 2)),
    *_staves(HOLE_R, 0.0235, -PANEL_T / 2, PANEL_T / 2, 16, axes=_plate_axes, skip=(_TOP_STAVE,),
             sides={_TOP_STAVE - 1: (-_STAVE_W / 2, _CUT), _TOP_STAVE + 1: (-_CUT, _STAVE_W / 2)}),
    *_staves(NOTCH_TO, 0.0235, -PANEL_T / 2, PANEL_T / 2, 16, axes=_plate_axes)[_TOP_STAVE:_TOP_STAVE + 1],  # the top stave above the notch
    # the notch's sides, from the bore's flat (11.15 mm) up into the top stave (run e1c: reaching 0.3 mm lower, their
    # corners stood 0.03 mm into the barrel's 11 mm and the barrel stopped at the panel's front)
    box((0.0011, PANEL_T, NOTCH_TO - HOLE_R + 0.0003), pos=(-(NOTCH_W / 2 + 0.00055), 0.0, (HOLE_R + NOTCH_TO) / 2 + 0.00015)),
    box((0.0011, PANEL_T, NOTCH_TO - HOLE_R + 0.0003), pos=(NOTCH_W / 2 + 0.00055, 0.0, (HOLE_R + NOTCH_TO) / 2 + 0.00015)),
]
_frame_parts = [  # two posts at the plate's ends, each on a foot along y
    box((0.010, 0.010, 0.160 + PANEL_H / 2), pos=(-(PANEL_W / 2 + 0.005), 0.0, (PANEL_H / 2 - 0.160) / 2), color=DARK),
    box((0.010, 0.010, 0.160 + PANEL_H / 2), pos=(PANEL_W / 2 + 0.005, 0.0, (PANEL_H / 2 - 0.160) / 2), color=DARK),
    box((0.020, 0.140, 0.008), pos=(-(PANEL_W / 2 + 0.005), 0.0, -0.160 + 0.004), color=DARK),
    box((0.020, 0.140, 0.008), pos=(PANEL_W / 2 + 0.005, 0.0, -0.160 + 0.004), color=DARK),
]
body("panel", _panel + _frame_parts, pos=tuple(HOLE), color=PANEL, friction=(0.4, 0.35))

# --- the operator (IDEC HW1B-V4 emergency-stop switch, 40 mm mushroom, the contact block off), in its own frame: the
# origin on its axis at the gasket's rear face (on the panel's front when seated), z towards the head, x towards its
# tab (the TOP marking). Sizes not in the maker's catalogue are assumed (README) ---
HEAD_R, HEAD_FROM, HEAD_TO = 0.020, 0.0065, 0.032  # the head stands 32 mm in front of the panel
BEZEL_R, BEZEL_FROM = 0.015, 0.0005
GASKET_T = 0.0005
THREAD_R, THREAD_FROM, THREAD_TO = 0.011, -0.0115, 0.0005  # M22 x 1, 12 mm behind the bezel
STEM_R, STEM_FROM = 0.010, -0.0195  # a plain stem 20 mm across, 8 mm long behind the thread
TAB_W, TAB_H, TAB_FROM = 0.003, 0.001, -0.002  # the anti-rotation tab: 3 mm wide, 1 mm high, from the panel's back to the bezel
OPERATOR_MASS = float(os.environ.get("ESTOP_OPERATOR_MASS", "0.040"))
OPERATOR_START = np.array([0.30, -0.28, TOP - STEM_FROM])  # standing on its stem's end
# The tab's direction from the arms' line (x), by the trial's seed (seed % 4; no seed: the first). Under inspect-robots,
# --seed 0 gives the trial seed 1415158438: the third start. The third was 210 degrees until the scripted plan was found
# not to reach it (the right wrist past its stop at the hole); 345 is reached like the other three (demo.py --quick).
START_ANGLES = tuple(math.radians(a) for a in (30.0, 120.0, 345.0, 300.0))
_tab_mid = THREAD_R + TAB_H / 2 - 0.0001
_operator = [
    cylinder(HEAD_R, HEAD_TO - HEAD_FROM, pos=(0.0, 0.0, (HEAD_FROM + HEAD_TO) / 2), color=RED),
    cylinder(BEZEL_R, HEAD_FROM - BEZEL_FROM, pos=(0.0, 0.0, (BEZEL_FROM + HEAD_FROM) / 2), color=BLACK),
    cylinder(BEZEL_R, GASKET_T, pos=(0.0, 0.0, GASKET_T / 2), color=RUBBER, friction=(0.9, 0.8)),
    cylinder(THREAD_R, THREAD_TO - THREAD_FROM, pos=(0.0, 0.0, (THREAD_FROM + THREAD_TO) / 2), color=DARK),
    box((TAB_H + 0.0002, TAB_W, THREAD_TO - TAB_FROM), pos=(_tab_mid, 0.0, (TAB_FROM + THREAD_TO) / 2), color=DARK),
    cylinder(STEM_R, THREAD_FROM - STEM_FROM, pos=(0.0, 0.0, (STEM_FROM + THREAD_FROM) / 2), color=DARK),
    box((0.0004, 0.006, 0.003), pos=(BEZEL_R + 0.0001, 0.0, (BEZEL_FROM + HEAD_FROM) / 2), color=WHITE, solid=False),  # TOP marking on the tab's side
]
_op_rot0 = mk.axis_angle(Z, START_ANGLES[0])
body("operator", _operator, pos=tuple(OPERATOR_START), quat=mk.matrix_to_quat(_op_rot0), mass=OPERATOR_MASS, color=BLACK, friction=(0.6, 0.5))


def on_operator(local, rotation=None, position=None) -> np.ndarray:
    """A point of the operator (its frame) in the world, as built or at the pose given."""
    rotation = _op_rot0 if rotation is None else rotation
    position = OPERATOR_START if position is None else position
    return position + rotation @ np.asarray(local, dtype=float)


# --- the locking ring (IDEC HW9Z-LN: black polyamide, 28.4 mm across, 5 mm high, M22 x 1; 2 g), its frame on the
# operator's axis at its front face, z as the operator's. It starts two turns on the end of the thread ---
RING_IN, RING_OUT, RING_H = 0.0115, 0.0142, 0.005
RING_REAL = 0.002
RING_MASS = float(os.environ.get("ESTOP_RING_MASS", "0.050"))  # for the solver (inertia); the weight over the real one carried
LEAD = 0.001  # m a turn
A_END, A_START, A_TOUCH = THREAD_FROM, THREAD_FROM + 0.002, TAB_FROM  # the ring's front face along the operator's axis: off the thread; at the start; on the panel's back
RING_AT = on_operator((0.0, 0.0, A_START))
_ring = [*_staves(RING_IN, RING_OUT, -RING_H, 0.0, 12, offset=math.pi / 12)]
_ring += [box((0.0025, 0.0016, 0.0006), pos=(0.0128 * math.cos(a), 0.0128 * math.sin(a), -RING_H - 0.0001), color=DARK, solid=False) for a in (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)]  # the four notches in its back face, 0.6 mm deep (drawn only)
body("ring", _ring, pos=tuple(RING_AT), quat=mk.matrix_to_quat(_op_rot0), mass=RING_MASS, color=BLACK, friction=(0.7, 0.6))

# --- the hidden bodies: the thread (carrier, turner) and the tightener on the operator, the seat in the panel. Each
# is four small boxes round its own axis, inside a part (the operator's barrel and stem, the hole), so that it turns
# with an inertia near its neighbours'. Run e1b had each as one 2 mm box, some 1000 times under the parts it joins,
# and the solver dropped the turning through them: the thread turned against the torque put on the ring ---
HIDDEN_MASS = 0.05
NAMES = ("panel", "operator", "ring", "carrier", "turner", "tightener", "seat")
HIDDEN = ("carrier", "turner", "tightener", "seat")
GHOST_SHAPES = 4


def _ghost(radius: float, side: float) -> list:
    return [box((side, side, 0.002), pos=(radius * math.cos(a), radius * math.sin(a), 0.0), passes=NAMES, color=STEEL)
            for a in (math.pi / 4 + k * math.pi / 2 for k in range(GHOST_SHAPES))]  # fmt: skip


for _name in ("carrier", "turner"):  # 8 mm off the axis: inside the barrel (11 mm) and the stem (10 mm) wherever the thread takes them
    body(_name, _ghost(0.008, 0.0016), pos=tuple(RING_AT), quat=mk.matrix_to_quat(_op_rot0), mass=HIDDEN_MASS)
SEATED_ROT = np.column_stack([Z, -X, -Y])  # seated: the head towards the right arm (z -> -y), the tab at the top (x -> +z)
SEAT_AT = HOLE + np.array([0.0, FRONT_Y, 0.0])  # the operator's origin when seated: on the panel's front
TIGHTENER_AT = SEAT_AT + SEATED_ROT @ np.array([0.0, 0.0, A_TOUCH])  # on the hole's axis at the panel's back: where a seated operator's ring meets it
body("tightener", _ghost(0.008, 0.0016), pos=tuple(TIGHTENER_AT), quat=mk.matrix_to_quat(SEATED_ROT), mass=HIDDEN_MASS)  # inside the barrel
body("seat", _ghost(0.008, 0.0016), pos=tuple(SEAT_AT), quat=mk.matrix_to_quat(SEATED_ROT), mass=HIDDEN_MASS)  # in the hole, inside the barrel when seated

# The thread: the ring welded to a turner that turns freely on a carrier, whose stops along the barrel are moved every
# control step to where the turn has brought it, 1 mm a turn (thread()). On a seated operator the ring leaves the
# thread at the panel's back for the tightener, a hidden disc hinged to the world on the hole's axis there: welded to
# it, the ring turns on against its spring, and the seat's stops close at the panel (the ring drawn against the panel's
# back pulls the flange home, within 0.2 mm of it then, and holds it). A spring
# holds the stiffness it is built with in this simulator only where the body it hangs from is held once: with a free
# hinge and a weld side by side there the tightening spring held at 0.547 of it, as with two welds there (runs e3b,
# e3c); hung from the seated operator, which its seat and its touch on the panel both hold, at 0.284 (run e1g). Hung
# from the world it cannot be held twice. A second joint beside the spring itself changes nothing (e3c): the lock.
# On an operator that is not in the panel the ring runs to the tab's end and turns there without going further.
PLAY = 0.00001  # m either way the thread's axial stops leave the carrier (PhysX refuses equal stops)
_axis_op = _op_rot0 @ Z
slide("along", "carrier", "operator", axis=tuple(_axis_op), limits=(-PLAY, PLAY))
# Its damping, 0.01 N m s/rad (0.04 N m at the 4 rad/s of a hand's turn), keeps a ring spun freely under half a turn a
# control step, where the turn is read (run e1h: 0.5 N m on a free ring at the tab's end, at 0.004, spun it past that
# and the count ran backwards)
hinge("turn", "turner", "carrier", anchor=tuple(RING_AT), axis=tuple(_axis_op), limits=None, damping=0.01)
TIGHT_K = 2.0 / math.radians(30.0)  # N m/rad: 2.0 N m at 30 degrees past the touch (the spec's placeholder)
TIGHT_MAX = math.radians(60.0)
# The spring as read in this scene with known loads on the ring (run e1j, seated, the hold off): 2.09 N m/rad at
# 0.1 N m to 2.00 at 2.0 N m, 0.547 of the stiffness it is built with (a fraction met where a body is held twice:
# runs e3b, e3c; here the doubling is not found). The level is read at this rate.
TIGHT_READ = 2.05
_axis_seated = SEATED_ROT @ Z  # the hole's axis, towards the head of a seated operator (-y)
hinge("tight", "tightener", None, anchor=tuple(TIGHTENER_AT), axis=tuple(_axis_seated), limits=(0.0, TIGHT_MAX), spring=TIGHT_K, rest=0.0, damping=0.02)
# The thread's hold on a tightened ring: the tightener welded to the world where it stands while no hand is closed on
# the ring, the weld cut while one is (hand_on_ring()). (A second hinge beside the spring whose lower stop followed the
# tightening pushed the angle on: 0.5 N m crept to 27 degrees and 1.0 N m to the stop, run e1i; without it the spring
# read 2.09 to 2.00 N m/rad from 0.1 to 2.0 N m, steady, run e1j.)
# A weld made during a trial keeps its frame where its anchor sat on its child as built (mechkit.make), so every one is
# anchored at its child's own origin. Anchored away from it, it is a long lever that the solver holds loosely: one
# anchored 0.56 m from its child as built let the two come 2.6 mm apart while welded (e2n); the seat's, anchored in
# the hole 0.33 m from the operator as built, gave 0.24 mm under the scorer's 10 N (e1j; 0.03 mm anchored on it, e1l).
weld("set", "tightener", None, anchor=tuple(TIGHTENER_AT), made=False)
weld("on", "ring", "turner", anchor=tuple(RING_AT))  # the ring on the thread as the trial starts
weld("on_again", "ring", "turner", anchor=tuple(RING_AT), made=False)  # and after it has been taken off and started again
weld("clamp", "ring", "tightener", anchor=tuple(RING_AT), made=False)  # against the panel's back (or the tab's end)
weld("op_hold", "operator", None, anchor=tuple(OPERATOR_START), made=False)  # trials.py only: the operator held where it stands
SEAT_OUT = -0.030
# N: the seated button rests in the bore and on its gasket: its weight (0.39 N) times nylon on steel (about 0.6). Without
# it nothing held a seated button left alone: jolted as the hand let go, it coasted out at 1.35 mm/s (check e2i seated).
SEAT_FRICTION = 0.25
slide("seat", "seat", None, axis=(0.0, 1.0, 0.0), limits=(SEAT_OUT, 0.0), friction=SEAT_FRICTION)
weld("seated", "operator", "seat", anchor=tuple(OPERATOR_START), made=False)
OBJECTS = list(mk.MECH.bodies.values())
PARTS = ("operator", "ring")
BOXES = {name: [s.bounds() for s in mk.MECH.bodies[name].shapes if s.solid] for name in PARTS}
CARRIED = {"ring": RING_MASS - RING_REAL, **dict.fromkeys(HIDDEN, HIDDEN_MASS)}

# --- the gates ---
RING_OFF, RING_TILT = 0.001, math.radians(5.0)  # a free ring is taken on by the thread within 1 mm and 5 degrees of the barrel's line
RING_REACH = 0.001  # its front face within this of the thread's end
START_SPIN, START_AWAY = 0.2, 0.002  # rad/s the ring turns on (screwing) and m/s it may come away at, to start
SEAT_OFF = float(os.environ.get("ESTOP_SEAT_OFF", "0.001"))  # m: the barrel's line within this of the hole's centre
SEAT_TILT = math.radians(float(os.environ.get("ESTOP_SEAT_TILT", "5.0")))
SEAT_TURN = math.radians(5.0)  # the tab within this of the notch
# m: the gate is read while the thread's rear end is from 8 mm in front of the panel's front (the stem's end at it) to
# all the way through: a barrel goes in only to 9.5 mm before the tab, turned off the notch, stands on the panel's face
# (and a little further leaning in the bore's play, 10.3 mm in run e1h); an installer turns it there under a light push
# until the tab drops in (the designer, 2026-11-20 06:07)
SEAT_ZONE = (STEM_FROM - THREAD_FROM, -THREAD_FROM + 0.0005)
SEAT_LOOSE = -0.025  # m: pulled back this far, the operator is out of the hole and free again

# --- the rules ---
SEATED_GAP = 0.001  # m: the gasket within this of the panel's front: seated
TURNS_ON = 1.0  # turns on the barrel behind the panel for 0.5
TOUCH_TOL = 0.0002  # m: the ring's front face this near the panel's back: against it
CLAMP_CLEAR = 0.00002  # m the clamped ring's front face is put behind the panel's back: against it, not into it
# Hand-tight: what fingers put on a ring 28.4 mm across, about 25 to 50 N at its 14 mm radius, 0.35 to 0.7 N m (an
# estimate, README). The level is 0.68 N m, at the top of that range and under the 1.13 N m at which this gripper's
# pinch on the ring slips (the tighten trial, run f1a: 31.6 degrees past the touch by the fifth grip and no further over
# four more): 19.0 degrees past the touch at the scene's 2.05 N m/rad. The maker's final torque with its ring wrench is
# a separate step outside this task (README).
SLIP_NM = 1.13
LEVEL_NM = 0.68
LEVEL_ANGLE = LEVEL_NM / TIGHT_READ
PULL_N, PULL_GAP = 10.0, 0.0002  # the scorer's pull on the head (N) and the gap the flange may show under it (m)
CLEAR, RESTING, SETTLE_S, ON_BENCH = 0.03, 0.02, 0.25, 0.02
SIDES = ("left", "right")
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    STATE.clear()
    STATE.update(seed=seed, carried=False, ring_on=True, tight=False, lock=0.0, slip=0.0, at_end=False, seated=False, track=[], events=[])
    for name in HIDDEN:
        for shape in range(GHOST_SHAPES):
            mk.show(name, shape, False)
    return STATE


def start_angle(seed) -> float:
    return START_ANGLES[(seed or 0) % len(START_ANGLES)]


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is None:
        return
    mk.set_limits("along", lower=-PLAY, upper=PLAY)  # the ring where it starts on the thread (run e1b: stops left 3 mm open let it drop first)
    mk.set_limits("seat", lower=SEAT_OUT, upper=0.0)
    turn = start_angle(seed) - START_ANGLES[0]
    if turn:  # the operator and what rides on it turned about its own (upright) axis
        about = mk.axis_angle(Z, turn)
        for name in ("operator", "ring", "carrier", "turner", "tightener"):
            position, rotation = mk.built_pose(name)
            mk.write_pose(name, OPERATOR_START + about @ (position - OPERATOR_START), mk.matrix_to_quat(about @ rotation))


# --- where things are ---
def axis_of(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(a point, the direction) of body ``name``'s z axis in the world: the operator's, the ring's."""
    position, rotation = mk.pose(name)
    return position, rotation @ Z


def ring_along() -> float:
    """Where the ring's front face is along the operator's axis (m, the operator's frame; THREAD_FROM at the thread's end)."""
    p, r = mk.pose("operator")
    return float((r.T @ (mk.pose("ring")[0] - p))[2])


def ring_off_axis() -> tuple[float, float]:
    """(how far the ring's centre is from the operator's axis, m; the angle between their axes, rad)."""
    p, r = mk.pose("operator")
    q, s = mk.pose("ring")
    local = r.T @ (q - p)
    return float(math.hypot(local[0], local[1])), float(math.acos(max(-1.0, min(1.0, float((r @ Z) @ (s @ Z))))))


def seat_reading() -> dict:
    """The operator against the hole: its axis's distance from the hole's centre at the panel's plane (m), its tilt
    from the hole's axis (rad), its tab's turn from the notch (rad), how far its thread's rear end is through the
    panel's front (m), how far its gasket is from the panel's front (m)."""
    p, r = mk.pose("operator")
    axis = r @ Z  # towards the head: -y when in line
    tilt = float(math.acos(max(-1.0, min(1.0, float(-axis[1])))))
    k = (FRONT_Y - p[1]) / axis[1] if abs(axis[1]) > 1e-6 else 0.0  # the axis line at the panel's front plane
    at_plane = p + k * axis
    off = float(math.hypot(at_plane[0] - HOLE[0], at_plane[2] - HOLE[2]))
    tab = r @ X
    tab_flat = tab - (tab @ axis) * axis
    turn = float(math.atan2(-tab_flat[0], tab_flat[2])) if np.linalg.norm(tab_flat) > 1e-9 else math.pi  # from straight up
    rear = p + r @ np.array([0.0, 0.0, THREAD_FROM])
    return {"off": off, "tilt": tilt, "turn": turn, "rear in": float(rear[1] - FRONT_Y), "gasket gap": float(FRONT_Y - (p + r @ np.array([0.0, 0.0, 0.0]))[1])}


def turns_on() -> float:
    """Turns of thread the ring has on the barrel (0 off it)."""
    return max(0.0, (ring_along() - A_END) / LEAD) if STATE.get("ring_on") else 0.0


# --- the mechanisms, once per control step ---
def ring_weld() -> str:
    """The weld that holds the ring at present: "clamp" (to the tightener) once it is against the panel's back; on the
    thread "on" as the trial starts, "on_again" once it has been taken off and started again (a weld that was built
    can be cut but not made again)."""
    if STATE.get("tight"):
        return "clamp"
    return "on_again" if mk.is_cut("on") else "on"


# A hand is on the ring when its fingers are closed to the ring's size (an opening of at most HAND_SHUT of the gripper's
# 94 mm: 32 mm round the 28.4 mm ring) with its grasp point within HAND_NEAR of the ring's front face's middle (9 to
# 13.5 mm as the scripts hold it). While a hand is on a tightened ring, the thread's hold is off and the ring turns
# against the spring; once the hand opens, the ring is held where it stands (the thread's grip on a real ring).
HAND_SHUT, HAND_NEAR = 0.34, 0.020


def hand_on_ring() -> bool:
    if not mk.MECH.grippers:
        return False
    q, _ = mk.pose("ring")
    return any(opening <= HAND_SHUT and float(np.linalg.norm(np.asarray(point) - q)) <= HAND_NEAR for point, opening in mk.MECH.grippers.values())


def thread() -> None:
    if STATE["ring_on"]:
        if STATE["tight"]:  # on the tightener: held where it stands unless a hand is closed on the ring; the most it was turned kept
            STATE["lock"] = max(STATE["lock"], min(mk.joint("tight"), TIGHT_MAX))
            turned = hand_on_ring()
            if turned and not mk.is_cut("set"):
                mk.cut("set")
            elif not turned and mk.is_cut("set"):
                mk.make("set")
            return
        a = A_START + LEAD * (mk.joint("turn") - STATE["slip"]) / (2 * math.pi)
        if a >= A_TOUCH - TOUCH_TOL and STATE["seated"]:  # against the panel's back: off the thread, onto the tightener
            _, s = mk.pose("ring")
            mk.cut(ring_weld())
            mk.write_pose("ring", TIGHTENER_AT - CLAMP_CLEAR * _axis_seated, mk.matrix_to_quat(s))  # at most TOUCH_TOL on, in line as the thread held it
            mk.make("clamp")
            mk.set_limits("seat", lower=-PLAY, upper=0.0)  # drawn against the panel's back, the ring pulls the flange home (within TOUCH_TOL of it now) and holds it
            STATE.update(tight=True)
            STATE["events"].append((mk.now(), "ring against the panel's back"))
            return
        if a > A_TOUCH:  # out of the panel: at the tab's end the thread runs out, and the ring turns there
            STATE["slip"] += (a - A_TOUCH) * 2 * math.pi / LEAD
            a = A_TOUCH
        if a >= A_TOUCH - TOUCH_TOL and not STATE["at_end"]:
            STATE["events"].append((mk.now(), "ring at the tab's end"))
        STATE["at_end"] = a >= A_TOUCH - TOUCH_TOL
        if a <= A_END:  # unscrewed off the thread's end: the ring is free
            mk.cut(ring_weld())
            STATE["ring_on"] = False
            STATE["events"].append((mk.now(), "ring off the thread"))
        else:
            mk.set_limits("along", lower=a - A_START - PLAY, upper=a - A_START + PLAY)
        return
    # A free ring is taken on again when it is held on the thread's end in line with it and turned on (screwing) while
    # not drawn away. Its bore clears the barrel, so a ring pushed on fast can be past the thread's end by the next
    # control step (run e1c: 12 mm in one step): found in line on the barrel beyond the end, it is caught as well, put
    # back where the thread's first turn stops it. (A push alone counted as well up to run e2a, where the ring, just
    # unscrewed in the fingers, was taken on and off again within 0.05 s on the noise of the solver's velocities.)
    off, tilt = ring_off_axis()
    a = ring_along()
    if off <= RING_OFF and tilt <= RING_TILT and A_END - RING_REACH <= a <= A_TOUCH:
        p, r = mk.pose("operator")
        q, s = mk.pose("ring")
        v, w = mk.velocity("ring")
        u, o = mk.velocity("operator")
        spin = float((w - o) @ (r @ Z))  # rad/s about the barrel, + screwing on
        closing = float((v - u) @ (r @ Z))  # m/s along it, + towards the head
        if (spin > START_SPIN and closing > -START_AWAY) or a > A_END + RING_REACH:
            fwd = s @ X
            fwd = fwd - (fwd @ (r @ Z)) * (r @ Z)
            fwd /= np.linalg.norm(fwd)
            rot = np.column_stack([fwd, np.cross(r @ Z, fwd), r @ Z])
            at = A_START + mk.joint("along")  # where the turner stands on the barrel: the thread's end, as it let the ring go
            mk.write_pose("ring", p + r @ np.array([0.0, 0.0, at]), mk.matrix_to_quat(rot))
            mk.make("on_again")
            STATE["ring_on"] = True
            STATE["events"].append((mk.now(), "ring taken on by the thread"))


def seat() -> None:
    if STATE["seated"]:
        if mk.joint("seat") <= SEAT_LOOSE:
            mk.cut("seated")
            mk.write_pose("seat", SEAT_AT, mk.matrix_to_quat(SEATED_ROT))  # the hidden seat back in the hole, out of the hands' way
            STATE["seated"] = False
            STATE["events"].append((mk.now(), "operator pulled out of the hole"))
        return
    g = seat_reading()
    STATE["closest"] = min(STATE.get("closest", (9.9,))[0], g["off"]), g
    if SEAT_ZONE[0] <= g["rear in"] <= SEAT_ZONE[1] and g["off"] <= SEAT_OFF and g["tilt"] <= SEAT_TILT and abs(g["turn"]) <= SEAT_TURN:
        p, _ = mk.pose("operator")
        depth = float(p[1] - SEAT_AT[1])  # m along +y from seated (negative: in front of it)
        at = SEAT_AT + np.array([0.0, depth, 0.0])
        mk.write_pose("operator", at, mk.matrix_to_quat(SEATED_ROT))
        mk.write_pose("seat", at, mk.matrix_to_quat(SEATED_ROT))
        mk.make("seated")
        STATE["seated"] = True
        STATE["events"].append((mk.now(), f"operator taken into the hole ({1000 * g['off']:.2f} mm off, {math.degrees(g['tilt']):.1f} deg tilted, tab {math.degrees(g['turn']):.1f} deg off)"))


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        if not STATE["carried"]:
            for name, kilograms in CARRIED.items():
                mk.push(name, force=(0.0, 0.0, kilograms * GRAVITY))
            STATE["carried"] = True
        thread()
        seat()
    track()


# --- rest, read from travel ---
def _marks() -> dict:
    return {name: mk.pose(name)[0].copy() for name in PARTS}


def track() -> None:
    t, kept = mk.now(), STATE.setdefault("track", [])
    kept.append((t, _marks()))
    while len(kept) > 2 and kept[1][0] <= t - SETTLE_S:
        kept.pop(0)


def speeds() -> dict:
    kept = STATE.get("track", [])
    if len(kept) < 2 or kept[-1][0] <= kept[0][0]:
        return {name: None for name in PARTS}
    (t0, then), (t1, now) = kept[0], kept[-1]
    return {name: float(np.linalg.norm(now[name] - then[name])) / (t1 - t0) for name in now}


def at_rest() -> bool:
    return all(v is not None and v < RESTING for v in speeds().values())


# --- the score ---
def on_bench() -> bool:
    return all(mk.pose(name)[0][2] >= TOP - ON_BENCH for name in PARTS)


def flange_gap() -> float:
    """How far the operator's gasket stands off the panel's front (m); inf if the operator is not in the hole."""
    return max(0.0, -mk.joint("seat")) if STATE.get("seated") else math.inf


def seated() -> bool:
    """The operator in the hole with its tab in the notch, its gasket within 1 mm of the panel's front."""
    return flange_gap() <= SEATED_GAP


def ring_gap() -> float:
    """How far the ring's front face is from where it meets the panel's back (the tab's end) on the barrel (m)."""
    return max(0.0, A_TOUCH - ring_along()) if STATE.get("ring_on") else math.inf


def tightened() -> float:
    """Rad the ring has been turned past its touch on the panel's back and holds."""
    return float(STATE.get("lock", 0.0)) if STATE.get("tight") else 0.0


def distance_to(name: str, point) -> float:
    """How far ``point`` is from body ``name``'s solid shapes (their boxes; m, 0 inside one)."""
    position, rotation = mk.pose(name)
    local = rotation.T @ (np.asarray(point, dtype=float) - position)
    return min(float(np.linalg.norm(np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0))) for low, high in BOXES[name])


def hands_clear() -> bool:
    """Both grippers' grasp points 3 cm or more from operator and ring."""
    return all(distance_to(name, mk.gripper(side)[0]) >= CLEAR for side in SIDES for name in PARTS)


def level() -> float:
    if not on_bench():
        return 0.0
    if not seated():
        return 0.0
    if not (STATE.get("ring_on") and turns_on() >= TURNS_ON):
        return 0.25
    if not STATE.get("tight"):
        return 0.5
    return 0.75


def score(objects) -> float:
    """By where things are at the end. 0 if a part has left the bench. The operator seated in the hole with its tab
    in the notch and its gasket within 1 mm of the panel: 0.25; and the ring at least one turn on the barrel behind
    the panel: 0.5; and the ring against the panel's back: 0.75; and the ring hand-tight (turned LEVEL_ANGLE past its
    touch, where the thread's hold keeps it), the gasket within 0.2 mm of the panel, both grippers clear and
    everything at rest: 1.0. The scorer's 10 N pull on the head is not made at the end: a
    tightened ring holds the flange home (trials.py seat: the flange's gap under that pull)."""
    found = level()
    if found == 0.75 and tightened() >= LEVEL_ANGLE and flange_gap() <= PULL_GAP and hands_clear() and at_rest():
        return 1.0
    return found


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {"events": [(round(t, 2), e) for t, e in STATE.get("events", [])]}
    if mk.MECH.poses:
        out.update({"ring on": STATE.get("ring_on"), "turns on": round(turns_on(), 2), "ring along (mm)": round(1000 * ring_along(), 2), "seated": STATE.get("seated"),
                    "tight (deg)": round(math.degrees(tightened()), 1), "seat": {k: round(v * (1000 if k in ("off", "rear in", "gasket gap") else 57.2958), 2) for k, v in seat_reading().items()},
                    "speeds (mm/s)": {k: None if v is None else round(1000 * v, 2) for k, v in speeds().items()}})  # fmt: skip
        if mk.MECH.grippers:
            out.update({"hands clear": hands_clear(), "at rest": at_rest(), "score": score(None)})
    return out
