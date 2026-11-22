"""Put a gas cooktop's burner knobs back on after cleaning. A section of a gas cooktop stands on the table: its
enamelled steel top panel, 42 mm above the table, with two burners at the back and, along the front, two burner
valve stems sticking 18 mm up through the panel, 100 mm apart, both at OFF. Each stem is a 1/4 in (6.35 mm) brass
shaft with a flat (a "D" shaft). The two black knobs that came off them lie upright on the panel beside them, each
turned away from OFF.

Task: put each knob back on a stem, pushed fully home, its pointer at OFF, without turning any burner on. A knob
goes onto its stem only with the flat in its bore lined up with the flat on the stem, so it has to be turned while
it rests on the stem's tip until it drops on. The valve is push-to-turn: its stem is locked at OFF by a lug in a
notch of the valve body until the stem is pushed 3.5 mm in, against its spring (about 10 N). Pressed lightly, a
knob that is turned on the stem turns on its own, or stops once it has dropped on; pressed hard, it pushes the stem
in, and the turn it is given carries the stem round off OFF: the gas is on.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from knobkit import Body, BoxShape, CylShape, LoftShape, as_numpy, joint, on_reset, on_step, rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Put the two burner knobs back on their valve stems, each pushed fully home with its pointer at OFF, "
               "and leave every burner off.")

MM = 1e-3

# -- the valve (a push-to-turn gas cock, as on a household gas cooktop or range) -------------------------------------
# Numbers marked "estimate" are to be replaced by values measured on the valves and knobs used (README.md,
# "Measuring the valve and the knob").
STEM_R = 6.35 * MM / 2      # 1/4 in stem
STEM_FLAT = 4.8 * MM - STEM_R   # the flat, from the axis (4.8 mm across the flat to the round side; estimate)
STEM_OUT = 18.0 * MM        # the stem's tip above the panel at rest (estimate; set by how the valve is mounted)
STEM_MASS = 0.030           # the stem and the plug it turns (estimate)
TRAVEL = 4.5 * MM           # how far the stem can be pushed in (estimate)
UNLOCK = 3.5 * MM           # pushed this far in, its lug is below the notch and it can turn (estimate)
SPRING_PRELOAD = 6.0        # N: the stem's spring at rest (estimate)
SPRING_RATE = 1200.0        # N/m (estimate): so the stem unlocks at 6 + 1200 x 0.0035 = 10.2 N
UNLOCK_FORCE = SPRING_PRELOAD + SPRING_RATE * UNLOCK
VALVE_DRAG = 0.05 / 360.0   # N m s/deg: the greased plug's drag, 0.05 N m at one turn a second (estimate)
OFF_STOP_DEG = -2.0         # OFF is the clockwise end of the valve's travel ...
MAX_TURN_DEG = 300.0        # ... and LO the counter-clockwise end
# The lug (on the stem, pointing to the front at OFF) and the notch it sits in (in the valve body's lip, under the
# panel): the lug is 2.0 mm wide, 2.5 mm tall, from 2.5 to 7.5 mm off the axis; the notch is 2.4 mm wide; the lip
# runs from 2 to 6 mm under the panel's top, so a lug sitting from 2.5 to 5 mm under it clears the lip once the
# stem is 3.5 mm in.
LUG = dict(r0=2.5 * MM, r1=7.5 * MM, w=2.0 * MM, top=-2.5 * MM, bottom=-5.0 * MM)
NOTCH_W = 2.4 * MM
LIP = dict(r0=4.0 * MM, r1=9.0 * MM, top=-2.0 * MM, bottom=-6.0 * MM)

# -- the knob (a household gas-cooktop burner knob: a moulded skirt with a grip fin, a D bore and a spring clip) -----
KNOB_R = 22.0 * MM          # 44 mm across
KNOB_H = 33.0 * MM          # skirt 14 mm, top 3 mm, fin 16 mm
KNOB_MASS = 0.022           # estimate
BORE_R = 3.45 * MM          # the bore's round part (0.275 mm clear of the stem)
BORE_DEPTH = 14.0 * MM      # the stem's tip bottoms here: seated, the skirt stands 18 - 14 = 4 mm off the panel
LEAD = 1.0 * MM             # lead-in chamfer at the bore's mouth
FLAT_CLEAR = 0.25 * MM      # the bore's own flat is this far clear of the stem's flat
FLAT_Z = (3.5 * MM, 7.5 * MM)   # the bore's moulded flat, from 3.5 to 7.5 mm up the bore (round below: it centres)
CLIP_IN = 0.45 * MM         # the steel spring clip stands this far proud of the stem's flat when free ...
CLIP_PRELOAD = 7.0          # N (estimate) ... and presses it with this; pushing the knob on takes about 5 N
CLIP_RATE = 5000.0          # N/m
CLIP_TRAVEL = 0.6 * MM
CLIP_Z = (8.0 * MM, 13.5 * MM)
CLIP_MASS = 0.0015
SEATED_GAP = STEM_OUT - BORE_DEPTH            # 4 mm: the skirt's gap to the panel with the knob home
ENGAGED_GAP = STEM_OUT - CLIP_Z[0]            # 10 mm: the flat found, resting on the clip

# -- layout --------------------------------------------------------------------------------------------------------
PANEL_Z = TOP + 0.042       # the cooktop panel's top
COOK_X = (0.20, 0.47)       # the cooktop section, front to back (x) ...
COOK_Y = 0.20               # ... and 400 mm across (y +-0.20)
COOK_C = np.array([(COOK_X[0] + COOK_X[1]) / 2, 0.0, PANEL_Z])
STEMS = [(0.265, 0.050), (0.265, -0.050)]     # left, right
HOLE = 6.0 * MM             # half the square hole round each stem in the panel
BURNERS = [(0.385, 0.10), (0.385, -0.10)]
KNOB_START = [(0.235, 0.140, -60.0), (0.235, -0.140, 60.0)]   # x, y, pointer (deg from OFF, counter-clockwise +)
OFF_DIR = 0.0               # OFF: the pointer to the back of the cooktop (+x); the lug points to the front

STEEL = (0.20, 0.20, 0.22)      # black-enamelled panel
BOX = (0.55, 0.45, 0.32)
BRASS = (0.78, 0.65, 0.30)
BLACK = (0.06, 0.06, 0.06)
WHITE = (0.95, 0.95, 0.93)
RED = (0.80, 0.12, 0.10)
GREY = (0.45, 0.45, 0.47)


def _ring(r_in, r_out, n, z0, z1, a0=0.0, chamfer=0.0, color=BLACK, skip=None):
    """n lofted segments making a ring (inner faces a regular n-gon of inradius r_in), optionally with a chamfer
    at z0 opening the inside by ``chamfer`` over that height."""
    out = []
    h = math.pi / n
    for k in range(n):
        a = a0 + 2 * h * k
        if skip is not None and skip(a):
            continue

        def poly(ri):
            ci = ri / math.cos(h)
            co = r_out / math.cos(h)
            return [(ci * math.cos(a - h), ci * math.sin(a - h)), (co * math.cos(a - h), co * math.sin(a - h)),
                    (co * math.cos(a + h), co * math.sin(a + h)), (ci * math.cos(a + h), ci * math.sin(a + h))]
        levels = [(z0, poly(r_in + chamfer)), (z0 + chamfer, poly(r_in)), (z1, poly(r_in))] if chamfer else \
            [(z0, poly(r_in)), (z1, poly(r_in))]
        out.append(LoftShape(tuple(levels), color=color))
    return out


def _rect(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def d_polygon(r=STEM_R, flat=STEM_FLAT, n=20):
    """The stem's D section: round, with its flat facing +x at ``flat`` from the axis (counter-clockwise)."""
    t0 = math.acos(flat / r)
    return [(r * math.cos(t), r * math.sin(t)) for t in np.linspace(t0, 2 * math.pi - t0, n)]


def _cooktop() -> Body:
    cx = COOK_C[0]
    x0, x1 = COOK_X[0] - cx, COOK_X[1] - cx
    hx = STEMS[0][0] - cx
    t = 0.001
    shapes = [BoxShape(((x0 + hx - HOLE) / 2, 0, -t / 2), (hx - HOLE - x0, 2 * COOK_Y, t), STEEL),
              BoxShape(((hx + HOLE + x1) / 2, 0, -t / 2), (x1 - hx - HOLE, 2 * COOK_Y, t), STEEL)]
    ys = sorted(s[1] for s in STEMS)
    edges = [-COOK_Y] + [v for y in ys for v in (y - HOLE, y + HOLE)] + [COOK_Y]
    for a, b in zip(edges[0::2], edges[1::2]):
        shapes.append(BoxShape((hx, (a + b) / 2, -t / 2), (2 * HOLE, b - a, t), STEEL))
    zb = TOP - PANEL_Z
    shapes.append(BoxShape(((x0 + x1) / 2, 0, (zb - t) / 2), (x1 - x0 - 0.004, 2 * COOK_Y - 0.004, -zb - t), BOX,
                           collide=False))                       # the valve box under the panel (not reachable)
    shapes.append(BoxShape((x0 + 0.002, 0, (zb - t) / 2), (0.004, 2 * COOK_Y, -zb - t), BOX))  # its front face
    for bx, by in BURNERS:
        shapes.append(CylShape((bx - cx, by, 0.006), 0.045, 0.012, GREY))
        shapes.append(CylShape((bx - cx, by, 0.016), 0.035, 0.008, BLACK))
    for sx, sy in STEMS:
        lx, ly = sx - cx, sy
        # The valve body's lip under the panel, with the notch the stem's lug sits in at OFF (the lug points to
        # the front, -x). The ring leaves out 30 deg either side of the notch; two walls make the notch's sides.
        zc, zh = (LIP["top"] + LIP["bottom"]) / 2, LIP["top"] - LIP["bottom"]
        rc, rl = (LIP["r0"] + LIP["r1"]) / 2, LIP["r1"] - LIP["r0"]
        for k in range(15):
            a = math.pi + math.radians(40 + 20 * k)
            shapes.append(BoxShape((lx + rc * math.cos(a), ly + rc * math.sin(a), zc),
                                   (rl, 2 * LIP["r1"] * math.tan(math.radians(10)), zh), BRASS, yaw=a))
        for s in (-1, 1):
            shapes.append(BoxShape((lx - rc, ly + s * (NOTCH_W / 2 + 1.0 * MM), zc), (rl, 2.0 * MM, zh), BRASS))
        # The panel's printed marks: OFF (white, behind the knob) and HI (red, a quarter turn counter-clockwise).
        shapes.append(BoxShape((lx + 0.030, ly, 0.0001), (0.008, 0.0015, 0.0002), WHITE, collide=False))
        shapes.append(BoxShape((lx, ly + 0.030, 0.0001), (0.0015, 0.008, 0.0002), RED, collide=False))
    return Body.build("cooktop", pos=tuple(COOK_C), mass=5.0, kinematic=True, color=STEEL, shapes=shapes,
                      friction=(0.4, 0.35), contact_offset=0.001)


def _stem(i: int) -> Body:
    poly = d_polygon()
    tip = [(0.9 * x, 0.9 * y) for x, y in poly]
    shapes = [LoftShape(((-0.012, tuple(poly)), (STEM_OUT - 0.3 * MM, tuple(poly)), (STEM_OUT, tuple(tip))), BRASS),
              BoxShape((-(LUG["r0"] + LUG["r1"]) / 2, 0.0, (LUG["top"] + LUG["bottom"]) / 2),
                       (LUG["r1"] - LUG["r0"], LUG["w"], LUG["top"] - LUG["bottom"]), BRASS),
              CylShape((0.0, 0.0, -0.022), 0.006, 0.020, BRASS, collide=False)]
    sx, sy = STEMS[i]
    return Body.build(f"stem_{i}", pos=(sx, sy, PANEL_Z), mass=STEM_MASS, color=BRASS, shapes=shapes,
                      friction=(0.3, 0.25), contact_offset=0.0008, solver_iterations=32, damping=(0.0, 0.5))


def _knob(i: int) -> Body:
    shapes = _ring(BORE_R, 6.5 * MM, 12, 0.0, BORE_DEPTH, chamfer=LEAD)              # the bore's boss
    fx = STEM_FLAT + FLAT_CLEAR
    shapes.append(LoftShape(((FLAT_Z[0], tuple(_rect(fx + 0.3 * MM, 5.5 * MM, -3.2 * MM, 3.2 * MM))),
                             (FLAT_Z[0] + 0.3 * MM, tuple(_rect(fx, 5.5 * MM, -3.2 * MM, 3.2 * MM))),
                             (FLAT_Z[1], tuple(_rect(fx, 5.5 * MM, -3.2 * MM, 3.2 * MM)))), BLACK))  # its flat
    shapes += _ring(KNOB_R - 1.5 * MM, KNOB_R, 16, 0.0, BORE_DEPTH)                  # the skirt
    shapes.append(CylShape((0, 0, BORE_DEPTH + 1.5 * MM), KNOB_R, 3.0 * MM, BLACK))  # the top
    shapes.append(BoxShape((0, 0, 25.0 * MM), (36.0 * MM, 9.0 * MM, 16.0 * MM), BLACK))   # the grip fin
    shapes.append(BoxShape((15.0 * MM, 0, KNOB_H + 0.1 * MM), (6.0 * MM, 2.0 * MM, 0.2 * MM), WHITE, collide=False))
    shapes.append(BoxShape((19.5 * MM, 0, 17.1 * MM), (4.0 * MM, 1.5 * MM, 0.2 * MM), WHITE, collide=False))
    x, y, a = KNOB_START[i]
    h = math.radians(a) / 2
    return Body.build(f"knob_{i}", pos=(x, y, PANEL_Z + 0.0005), mass=KNOB_MASS, color=BLACK, shapes=shapes,
                      friction=(0.5, 0.4), contact_offset=0.0008, solver_iterations=32, com=(0.0, 0.0, 0.016),
                      start_xyzw=(0.0, 0.0, math.sin(h), math.cos(h)), damping=(0.0, 0.05))


CLIP_X0 = STEM_FLAT - CLIP_IN
CLIP_ZC = (CLIP_Z[0] + CLIP_Z[1]) / 2


def _clip(i: int) -> Body:
    """The knob's steel spring clip: a leaf along the bore's flat side, sprung toward the axis (a separate body on a
    sliding joint in the knob). Frame: origin on its face at rest, x outward, z up the bore."""
    lo, hi = CLIP_Z[0] - CLIP_ZC, CLIP_Z[1] - CLIP_ZC
    shapes = [LoftShape(((lo, tuple(_rect(0.7 * MM, 1.0 * MM, -2.4 * MM, 2.4 * MM))),
                         (lo + 1.0 * MM, tuple(_rect(0.0, 1.0 * MM, -2.4 * MM, 2.4 * MM))),
                         (hi, tuple(_rect(0.0, 1.0 * MM, -2.4 * MM, 2.4 * MM)))), STEEL_CLIP)]
    x, y, a = KNOB_START[i]
    r = rotation((0.0, 0.0, math.sin(math.radians(a) / 2), math.cos(math.radians(a) / 2)))
    p = np.array([x, y, PANEL_Z + 0.0005]) + r @ np.array([CLIP_X0, 0.0, CLIP_ZC])
    h = math.radians(a) / 2
    return Body.build(f"clip_{i}", pos=tuple(p), mass=CLIP_MASS, color=STEEL_CLIP, shapes=shapes,
                      friction=(0.3, 0.25), contact_offset=0.0008, solver_iterations=32,
                      start_xyzw=(0.0, 0.0, math.sin(h), math.cos(h)))


STEEL_CLIP = (0.70, 0.70, 0.72)

OBJECTS = [_cooktop(), _stem(0), _stem(1), _knob(0), _clip(0), _knob(1), _clip(1)]

for _i, (_sx, _sy) in enumerate(STEMS):
    # The stem: held on its axis by the valve body (a sliding and turning fit); its spring pushes it out against its
    # stop; it turns from OFF (clockwise end) to LO; the plug's grease drags it. Whether it can turn at all is left
    # to the contact between its lug and the lip's notch (the joint lets the two collide).
    joint(f"valve_{_i}", "cooktop", f"stem_{_i}", anchor0=(_sx - COOK_C[0], _sy, 0.0), anchor1=(0.0, 0.0, 0.0),
          axis0=(0.0, 0.0, 1.0), axis1=(0.0, 0.0, 1.0),
          free={"transX": (-TRAVEL, 0.0), "rotX": (OFF_STOP_DEG, MAX_TURN_DEG)},
          drives={"transX": (SPRING_RATE, 2.0, SPRING_PRELOAD / SPRING_RATE), "rotX": (0.0, VALVE_DRAG, 0.0)},
          collide=True)
    # The clip slides only across the bore, between its seat (0) and 0.6 mm out, pressed in by its spring.
    joint(f"clip_{_i}", f"knob_{_i}", f"clip_{_i}", anchor0=(CLIP_X0, 0.0, CLIP_ZC), anchor1=(0.0, 0.0, 0.0),
          free={"transX": (0.0, CLIP_TRAVEL)}, drives={"transX": (CLIP_RATE, 1.0, -CLIP_PRELOAD / CLIP_RATE)})

# -- scoring numbers ---------------------------------------------------------------------------------------------------
GAS_ON_DEG = 10.0          # a stem turned more than this from OFF has left its OFF notch: the gas is on
CENTRE = 1.5 * MM          # a knob is on a stem: its bore's axis within 1.5 mm of the stem's at the mouth ...
TILT_ENGAGED = 5.0         # ... tilted no more than 5 degrees (3 seated)
TILT_SEATED = 3.0
GAP_SEATED = 5.0 * MM      # seated: the skirt within 5 mm of the panel (4 mm when home)
GAP_ENGAGED = 12.0 * MM    # the flat found: the knob has dropped onto the stem's flat (skirt within 12 mm)
POINTER_SEATED = 5.0       # degrees from OFF
POINTER_ENGAGED = 10.0
CREDIT_SEATED = 0.45
CREDIT_ENGAGED = 0.2
CLEAR = 0.02               # let go: every part of both grippers at least 2 cm from both knobs
STILL = 0.001              # still: nothing moved more than 1 mm over the last 1 s
HOLD_STEPS = 20            # 1 s at 20 Hz


# -- geometry (pure; used both in simulation and by the offline tests) ------------------------------------------------

def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def yaw_deg(r) -> float:
    r = np.asarray(r, float)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def stem_turn_deg(stem_r) -> float:
    """How far a stem is turned from OFF, counter-clockwise (toward ON) positive."""
    return wrap_deg(yaw_deg(stem_r) - OFF_DIR)


def knob_on_stem(knob_pos, knob_r, stem_xy) -> dict:
    """Where a knob is against a stem: its bore's offset from the stem's axis at the mouth (m), its tilt (deg), the
    skirt's gap to the panel (m) and its pointer's angle from OFF (deg)."""
    p, r = np.asarray(knob_pos, float), np.asarray(knob_r, float)
    off = float(math.hypot(p[0] - stem_xy[0], p[1] - stem_xy[1]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))
    return {"off": off, "tilt": tilt, "gap": float(p[2] - PANEL_Z), "pointer": wrap_deg(yaw_deg(r) - OFF_DIR)}


def knob_credit(g: dict) -> float:
    if g["off"] > CENTRE:
        return 0.0
    if g["tilt"] <= TILT_SEATED and g["gap"] <= GAP_SEATED and abs(g["pointer"]) <= POINTER_SEATED:
        return CREDIT_SEATED
    if g["tilt"] <= TILT_ENGAGED and g["gap"] <= GAP_ENGAGED and abs(g["pointer"]) <= POINTER_ENGAGED:
        return CREDIT_ENGAGED
    return 0.0


def knob_clearance(points, knob_pos, knob_r) -> float:
    """The least distance (m) from the grippers' points to a knob (a 44 mm cylinder 33 mm tall, from its mouth)."""
    if points is None or len(points) == 0:
        return float("inf")
    local = (np.asarray(points, float) - np.asarray(knob_pos, float)) @ np.asarray(knob_r, float)
    radial = np.maximum(np.hypot(local[:, 0], local[:, 1]) - KNOB_R, 0.0)
    axial = np.maximum(np.maximum(-local[:, 2], local[:, 2] - KNOB_H), 0.0)
    return float(np.hypot(radial, axial).min())


def knob_points(knob_pos, knob_r) -> np.ndarray:
    """Two points on a knob (its mouth and its pointer's tip), for the stillness check."""
    p, r = np.asarray(knob_pos, float), np.asarray(knob_r, float)
    return np.vstack([p, p + r @ np.array([0.018, 0.0, KNOB_H])])


# -- the score, step by step -------------------------------------------------------------------------------------------

_EP: dict = {}
_DEBUG = bool(os.environ.get("KNOB_DEBUG"))


@on_reset
def _reset_state() -> None:
    _EP.clear()
    _EP.update(gas=False, held=0, recent=[], score=0.0, done=False, steps=0, info={}, max_turn=0.0)


_reset_state()


def observe(knobs, stems, gripper_points) -> None:
    """One control step. ``knobs``: a (pos, rotation matrix) per knob; ``stems``: a (pos, rotation matrix) per stem;
    ``gripper_points``: the grippers' points (world, m)."""
    turns = [stem_turn_deg(r) for _, r in stems]
    _EP["max_turn"] = max(_EP["max_turn"], *(abs(t) for t in turns))
    _EP["gas"] |= any(abs(t) > GAS_ON_DEG for t in turns)
    geo = [[knob_on_stem(kp, kr, s) for s in STEMS] for kp, kr in knobs]
    credit = [max(knob_credit(geo[k][s]) for k in range(len(knobs))) for s in range(len(STEMS))]
    # One knob cannot count on both stems: they are 100 mm apart and a knob is on a stem only within 1.5 mm of it.
    snap = np.vstack([knob_points(kp, kr) for kp, kr in knobs] + [np.asarray(sp, float) for sp, _ in stems])
    recent = _EP["recent"]
    recent.append(snap)
    del recent[:-(HOLD_STEPS + 1)]
    still = len(recent) == HOLD_STEPS + 1 and all(float(np.abs(r - snap).max()) <= STILL for r in recent)
    clear = all(knob_clearance(gripper_points, kp, kr) >= CLEAR for kp, kr in knobs)
    seated = all(c >= CREDIT_SEATED for c in credit)
    done = bool(seated and clear and still and not _EP["gas"])
    _EP["held"] = _EP["held"] + 1 if done else 0
    if _EP["gas"]:
        score = 0.0
    elif _EP["held"] >= HOLD_STEPS:
        score = 1.0
    else:
        score = float(sum(credit))
    _EP.update(score=score, done=done, steps=_EP["steps"] + 1,
               info={"credit": credit, "turns": turns, "still": still, "clear": clear, "geo": geo,
                     "depths": [float(PANEL_Z - sp[2]) for sp, _ in stems]})


def success(objects) -> bool:
    """True only on steps where score() is 1.0: both knobs seated and at OFF, no stem ever turned off OFF, both
    grippers 2 cm clear and everything still, for 1 s. The runtime then ends the episode after its own 1 s of this."""
    return _EP["score"] >= 1.0


def score(objects) -> float:
    """0 for the whole trial once any valve stem has turned more than 10 degrees from OFF (the gas is on). Otherwise
    1.0 once both knobs have been seated with their pointers at OFF, both grippers 2 cm clear and everything still,
    for 1 s (20 steps in a row); else, at the end, per stem: 0.45 for a knob seated on it (bore within 1.5 mm of the
    stem's axis, tilt within 3 deg, skirt within 5 mm of the panel, pointer within 5 deg of OFF) or 0.2 for a knob
    on its flat but not pushed home (tilt within 5 deg, skirt within 12 mm, pointer within 10 deg)."""
    return float(_EP["score"])


# -- in the simulator: read the state and score ---------------------------------------------------------------------

_HULLS: dict = {}


def _gripper_points(base, origin) -> np.ndarray:
    """Points on both grippers (the vertices of their collision hulls, world frame)."""
    if not _HULLS:
        from yam_arena.analysis import link_hulls

        hulls = link_hulls()
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in hulls[link]])
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        for j, n in enumerate(robot.body_names):
            link = n.rsplit("/", 1)[-1]
            if link in _HULLS:
                out.append((_HULLS[link] @ rotation(poses[j][3:7]).T) + poses[j][:3] - origin)
    return np.vstack(out) if out else np.zeros((0, 3))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    knobs = [pose(f"knob_{i}") for i in range(len(STEMS))]
    stems = [pose(f"stem_{i}") for i in range(len(STEMS))]
    observe(knobs, stems, _gripper_points(base, origin))
    if _DEBUG and _EP["steps"] % 10 == 0:
        inf = _EP["info"]
        msg = [f"[knob] step {_EP['steps']} score {_EP['score']} held {_EP['held']} gas {_EP['gas']} "
               f"still {inf['still']} clear {inf['clear']} credit {inf['credit']} "
               f"turns {np.round(inf['turns'], 2).tolist()} depths(mm) {np.round(1000 * np.array(inf['depths']), 2)}"]
        for k, row in enumerate(inf["geo"]):
            msg.append(f"  knob{k} " + " | ".join(f"stem{s}: off {1000 * g['off']:.2f} tilt {g['tilt']:.1f} "
                                                   f"gap {1000 * g['gap']:.2f} ptr {g['pointer']:.1f}"
                                                   for s, g in enumerate(row)))
        print("\n".join(msg), file=sys.stderr, flush=True)
