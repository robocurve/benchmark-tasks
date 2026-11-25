"""On the station table, a stand-in for a car's rear seat: a rigid seat cushion with its backrest behind it and, in
the gap between them (the seat bight), the car's two ISOFIX low anchorages: 6 mm round steel bars 280 mm apart
(UN Regulation No. 145), each inside a plastic insertion guide. On the cushion, its back 190 mm from the backrest and
turned 2 deg, stands a high-back ISOFIX booster seat (KIDFIX i-SIZE type, 6.5 kg) with its ISOFIX locking-arm bracket
pulled out 100 mm. Each arm ends in a connector with a spring-loaded catch; a window on each armrest shows red while
that side is not latched and green once it is. Fitting it takes the manual's two steps: push the connectors onto the
bars until both click, then hold the red ISOFIX adjustment button (front of the seat) in and push the booster back
until it sits flat against the backrest; released, the button locks the arms again.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Fit the booster seat to the car seat's ISOFIX anchors." --scorer task_score --max-steps 1600 ...
"""

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from closedloop import grip_rot, placeholder_demo  # noqa: E402  (hooks yam_scripted to run demo_script)
from mechkit import Joint, Mech, box, cyl, hull, lamp, on_reset, on_step, released, set_lamp  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)

FABRIC, DARKFAB, STEEL, GUIDE, SHELL, ARMC = ((0.30, 0.31, 0.34), (0.20, 0.21, 0.23), (0.70, 0.71, 0.73),
                                              (0.08, 0.08, 0.09), (0.16, 0.17, 0.19), (0.55, 0.56, 0.58))
RED, GREEN = (0.85, 0.10, 0.08), (0.10, 0.75, 0.22)


def _qz(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def _qy(a):
    return (math.cos(a / 2), 0.0, math.sin(a / 2), 0.0)


# -- the car seat (fixed). x is toward the backrest (away from the robot), y left, z up.
CUSH_TOP = TOP + 0.040  # rigid stand-in for the seat cushion's surface
BACK_X = 0.790  # front face of the seat backrest (as far out as the table allows: the arms push hardest here)
BIGHT = 0.045  # height of the gap under the backrest's front face (the seat bight)
BAR_X, BAR_Z = BACK_X + 0.020, CUSH_TOP + 0.020  # ISOFIX bar centres: 20 mm into the bight, 20 mm above the cushion
BAR_R, BAR_LEN = 0.003, 0.036  # 6 mm round bar (UN R145), 36 mm exposed between the guide walls
BAR_Y = 0.140  # +- 140 mm: the two bars' centres 280 mm apart
G_IN_Y, G_IN_Z, G_WALL, G_DEPTH = 0.036, 0.034, 0.003, 0.030  # guide sleeve inside 36 x 34 mm, 3 mm walls
FLARE = 0.006  # the sleeve opening widens 6 mm per side over its first 10 mm (lead-in)


def _guide(sy):
    """Insertion guide around one bar: a sleeve from the backrest face into the bight, with a flared mouth."""
    cx = BACK_X + G_DEPTH / 2
    y0 = sy * BAR_Y
    zc = BAR_Z - TOP  # the car seat Mech sits at the table top
    out = [
        box((cx, y0, zc + G_IN_Z / 2 + G_WALL / 2), (G_DEPTH, G_IN_Y + 2 * G_WALL, G_WALL), GUIDE),
        box((cx, y0, zc - G_IN_Z / 2 - G_WALL / 2), (G_DEPTH, G_IN_Y + 2 * G_WALL, G_WALL), GUIDE),
        *[box((cx + 0.005, y0 + s * (G_IN_Y / 2 + G_WALL / 2), zc), (G_DEPTH - 0.010, G_WALL, G_IN_Z), GUIDE)
          for s in (-1, 1)],
    ]
    a = math.atan2(FLARE, 0.010)
    for s in (-1, 1):  # side flares: plates from the mouth (wide) to 10 mm in (sleeve width)
        out.append(box((BACK_X + 0.005, y0 + s * (G_IN_Y / 2 + FLARE / 2 + G_WALL / 2), zc),
                       (math.hypot(0.010, FLARE), G_WALL, G_IN_Z), GUIDE, _qz(-s * a)))  # mouth end further out
    out.append(cyl((BAR_X, y0, zc), BAR_R, BAR_LEN + 2 * G_WALL, STEEL, axis="y"))  # the anchorage bar
    return out


CAR_SEAT = Mech("car_seat", pos=(0.0, 0.0, TOP), mass=None, friction=0.4, shapes=[
    box(((0.15 + BACK_X) / 2, 0.0, 0.020), (BACK_X - 0.15, 0.80, 0.040), FABRIC),  # cushion x 0.15 .. BACK_X
    box((BACK_X + 0.050, 0.0, 0.040 + BIGHT + 0.2575), (0.100, 0.80, 0.515), FABRIC),  # backrest above the bight
    box((BACK_X + 0.080, 0.0, 0.040 + BIGHT / 2), (0.040, 0.80, BIGHT), DARKFAB),  # back of the bight
    *[s for sy in (-1, 1) for s in _guide(sy)],
])
CAR_SEAT_SHAPES = [(s.at, s.size) for s in CAR_SEAT.shapes]

# -- the booster seat (free). Its own frame: origin at the middle of its rear bottom edge, x toward its back.
B_DEPTH, B_WIDTH, B_SEAT_H = 0.360, 0.440, 0.100  # seat cushion
B_MASS = 6.5
ARM_Y = BAR_Y  # the locking arms line up with the car's bars
SLOT_Z = BAR_Z - CUSH_TOP  # 0.020: slot centre height when the booster sits on the cushion
THROAT_X, MOUTH_X = 0.017, 0.045  # connector throat bottom and mouth, measured behind the booster's back face
SLOT = 0.008  # gap between the jaws (6 mm bar)
LATCH_AT = (0.030, 0.0, SLOT_Z + SLOT / 2)  # catch plunger origin: at the upper jaw face, over the throat
LATCH_OPEN = 0.009  # m: the plunger's lift stop
EXT = 0.100  # locking arm travel: they start pulled out 100 mm and slide back into the seat while the button is held
ARM_FREE = 3.0  # N to slide the arm bracket while the ISOFIX adjustment button is held
LOCK_PLAY = 0.0003  # m: with the button out, the bracket is pinned where it is to +-0.3 mm (its ratchet)
BUTTON_AT = (-B_DEPTH, 0.0, 0.080)  # ISOFIX adjustment button, centre of the cushion's front face, 20 mm below its top
BUTTON_TRAVEL, BUTTON_DOWN = 0.008, 0.004  # m: full stroke; pressed (arms released) past this


def _arms():
    """Both locking arms in their joint frame (origin on the centre line at the booster's back face and slot height,
    arms pulled out EXT): 30 x 28 mm bars at y +-ARM_Y from 120 mm inside the seat to the connectors, joined by a
    cross bar under the seat so they slide as one bracket."""
    e = EXT
    out = [box((-0.110, 0.0, -0.004), (0.020, 2 * ARM_Y + 0.030, 0.012), ARMC)]  # cross bar, inside the seat
    for sy in (1, -1):
        y = sy * ARM_Y
        out += [
            box(((-0.120 + THROAT_X + e) / 2, y, 0.0), (THROAT_X + e + 0.120, 0.030, 0.028), ARMC),  # arm
            box((0.031 + e, y, SLOT / 2 + 0.005), (0.028, 0.030, 0.010), ARMC),  # upper jaw
            box((0.031 + e, y, -SLOT / 2 - 0.005), (0.028, 0.030, 0.010), ARMC),  # lower jaw
        ]
    return out


def _catch():
    """Spring catch in the joint frame (origin LATCH_AT): a plunger sliding straight up and down out of the upper jaw,
    with a 43 deg ramp toward the mouth and a square face toward the throat 1 mm in front of the seated bar. The bar
    entering pushes it up on the ramp; once the bar is past, its spring drops it straight down behind the bar."""
    px, pz = LATCH_AT[0], LATCH_AT[2]
    zb, zt = SLOT_Z - SLOT / 2 + 0.0005, SLOT_Z + SLOT / 2 + 0.008  # 0.5 mm above the lower jaw .. inside the jaw
    xz = [(0.024, zb), (0.028, zb), (0.036, SLOT_Z + SLOT / 2), (0.036, zt), (0.024, zt)]
    pts = [(x - px, s * 0.012, z - pz) for x, z in xz for s in (-1, 1)]
    return [hull((0.0, 0.0, 0.0), pts, STEEL)]


BOOSTER = Mech("booster", pos=(BACK_X - 0.020 - EXT - 0.070, 0.0, CUSH_TOP + 0.0005), yaw=math.radians(2.0), mass=B_MASS,
               friction=0.4, shapes=[
    box((-B_DEPTH / 2, 0.0, B_SEAT_H / 2), (B_DEPTH, B_WIDTH, B_SEAT_H), SHELL),  # seat cushion
    box((-0.045, 0.0, B_SEAT_H + 0.25), (0.090, B_WIDTH, 0.50), SHELL),  # backrest
    *[box((-0.20, s * (B_WIDTH / 2 - 0.02), B_SEAT_H + 0.035), (0.28, 0.04, 0.07), SHELL) for s in (-1, 1)],  # armrests
    box((-0.11, 0.0, B_SEAT_H + 0.50), (0.14, 0.30, 0.12), SHELL),  # headrest
    *lamp("ind_l", (-0.12, B_WIDTH / 2 - 0.02, B_SEAT_H + 0.0715), 0.010, {"red": RED, "green": GREEN}),  # on the armrests
    *lamp("ind_r", (-0.12, -B_WIDTH / 2 + 0.02, B_SEAT_H + 0.0715), 0.010, {"red": RED, "green": GREEN}),
], joints=[
    Joint("arms", "slide", axis="x", at=(0.0, 0.0, SLOT_Z), low=-EXT, high=0.0, hold=ARM_FREE, damping=20.0,
          mass=0.6, shapes=_arms()),
    *[Joint(f"catch_{n}", "slide", axis="z", parent="arms", at=(LATCH_AT[0] + EXT, sy * ARM_Y, LATCH_AT[2] - SLOT_Z),
            low=0.0, high=LATCH_OPEN, spring=40.0, damping=0.5, mass=0.004, shapes=_catch())
      for n, sy in (("l", 1), ("r", -1))],
    Joint("button", "slide", axis="x", at=BUTTON_AT, low=0.0, high=BUTTON_TRAVEL, spring=1500.0, damping=5.0,
          mass=0.01, shapes=[box((-0.006, 0.0, 0.0), (0.012, 0.060, 0.025), RED)]),  # stands 12 mm proud, red
])
OBJECTS = [CAR_SEAT, BOOSTER]

# -- scoring
STILL_SPEED, STILL_SPIN = 0.005, 0.05
CAUGHT = 0.002  # m: the catch plunger counts as down (closed behind the bar) within this of its stop
_TRIAL = {}


FLUSH = 0.005  # back face within 5 mm of the car's backrest at both sides: pushed home


@on_reset
def _new_trial(env):
    _TRIAL.clear()
    _ADJ.clear()
    _bracket_limits(env, -EXT, 0.0)


_ADJ = {}


def _bracket_limits(env, lo, hi):
    import torch

    asset = env.unwrapped.scene["booster"]
    i = [n.removesuffix("_joint") for n in asset.joint_names].index("arms")
    lim = torch.tensor([[[float(lo), float(hi)]]], dtype=torch.float32, device=asset.device)
    asset.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=[i], warn_limit_violation=False)


@on_step
def _adjustment_button(env):
    """The mechanism, not scoring: the locking-arm bracket's ratchet. With the ISOFIX adjustment button out, the
    bracket is pinned where it is (its slide limits closed to +-LOCK_PLAY around its position), so no push or pull moves
    it; while the button is held in, the limits open to the full travel and the bracket slides at ARM_FREE."""
    asset = env.unwrapped.scene["booster"]
    names = [n.removesuffix("_joint") for n in asset.joint_names]
    q = asset.data.joint_pos[0].tolist()
    pressed = q[names.index("button")] > BUTTON_DOWN
    if _ADJ.get("pressed") != pressed:
        if pressed:
            _bracket_limits(env, -EXT, 0.0)
        else:
            a = q[names.index("arms")]
            _bracket_limits(env, max(-EXT, a - LOCK_PLAY), min(0.0, a + LOCK_PLAY))
        _ADJ["pressed"] = pressed


def latched(b, n):
    """True when side ``n``'s bar sits in its connector's throat (behind the catch) and the catch is down."""
    sy = 1 if n == "l" else -1
    lx, ly, lz = b.to_local((BAR_X, sy * BAR_Y, BAR_Z))
    lx -= EXT + b.joints["arms"]  # measured from the connector, wherever the arms have slid to
    in_throat = (THROAT_X - 0.0005 <= lx <= 0.0255 and abs(lz - SLOT_Z) <= SLOT / 2 and abs(ly - sy * ARM_Y) <= 0.015)
    return bool(in_throat and b.joints[f"catch_{n}"] <= CAUGHT)


def state(objects):
    b = objects["booster"]
    tilt = math.degrees(math.acos(float(np.clip(b.rotation()[2, 2], -1.0, 1.0))))
    gap = max(BACK_X - float(b.to_world((0.0, sy * (B_WIDTH / 2 - 0.02), 0.07))[0]) for sy in (1, -1))
    return {"l": latched(b, "l"), "r": latched(b, "r"), "tilt": tilt, "low": float(b.origin[2]), "gap": gap,
            "flush": gap <= FLUSH}


def score(objects):
    """Per side, latched = the bar in the connector's throat with the catch down behind it (the window turns green).
    Flush = the booster's back face within 5 mm of the car's backrest at both sides. 0 for the rest of the trial if the
    booster tips more than 30 deg or drops off the cushion. Else, at the end: 1.0 both latched and flush, the booster
    let go and at rest; 0.75 both latched and flush but still held or moving; 0.5 both latched, not pushed flush;
    0.25 one side only; 0 none."""
    s = state(objects)
    b = objects["booster"]
    set_lamp(BOOSTER, "ind_l", "green" if s["l"] else "red")
    set_lamp(BOOSTER, "ind_r", "green" if s["r"] else "red")
    if s["tilt"] > 30.0:
        _TRIAL.setdefault("hazard", "booster tipped over")
    if s["low"] < CUSH_TOP - 0.02:
        _TRIAL.setdefault("hazard", "booster fell off the seat")
    if os.environ.get("ISOFIX_DEBUG"):
        _debug(objects, s)
    if "hazard" in _TRIAL:
        return 0.0
    n = int(s["l"]) + int(s["r"])
    if n == 0:
        return 0.0
    if n == 1:
        return 0.25
    if not s["flush"]:
        return 0.5
    let_go = released(b, lo=(-B_DEPTH - 0.012, -B_WIDTH / 2, 0.0), hi=(MOUTH_X, B_WIDTH / 2, B_SEAT_H + 0.56))
    return 1.0 if let_go and b.resting(STILL_SPEED, STILL_SPIN) else 0.75


def success(objects):
    return score(objects) >= 1.0


demo = placeholder_demo

# -- the scripted demo, in the booster manual's two steps. (1) Both hands, fingers closed and level, push the front
# ends of the two armrests; each hand drives its own corner toward where it is when both connectors are on their bars,
# until both catches click (the guides' flared mouths take out the 2 deg turn). (2) The hand on the side already
# nearer home holds the ISOFIX adjustment button in (arms free to slide) while the other pushes its armrest, until
# the booster's back is flush with the backrest; the button hand lets go first (the arms lock where they are).
# The armrest front ends are the push points because they are the booster's only front faces at hand height.
LEVEL = grip_rot(0.0, math.pi / 2)  # fingers level, pointing at the backrest (push force along the fingers)
PRESS = grip_rot(0.0, math.pi / 2)  # level for the button too: a level hand pushes hardest (force along the fingers)
ARM_FRONT_X = -0.340  # front ends of the armrests (x -0.34 .. -0.06 in the booster frame)
PUSH_Z = 0.135  # contact height: middle of the armrest front ends (z 0.10 .. 0.17)
PUSH_Y = B_WIDTH / 2 - 0.02  # on the armrests' centre lines
OVER = 0.050  # command this far past the goal (the arms sag under load)
INTO = 0.060  # and this far into the face ahead of the corner: a firm push
CLICK_BACK = BAR_X - 0.020 - EXT  # booster back face when both connectors are on their bars (arms still out)


def _tcp(tip, rot=None):
    rot = LEVEL if rot is None else rot
    return np.asarray(tip, float) - PAD_TIP_PAST_TCP * rot[:, 2]


def _front(b, sy):
    return np.asarray(b.to_world((ARM_FRONT_X, sy * PUSH_Y, PUSH_Z)), float)


def _button(b):
    return np.asarray(b.to_world((BUTTON_AT[0] - 0.012, BUTTON_AT[1], BUTTON_AT[2])), float)  # its face


def _pusher(ctx, sy, back_goal, even=False):
    """Target for the hand on armrest ``sy``: 60 mm into its front end, never past where it is with the booster's back
    at ``back_goal`` (+ OVER). ``even``: also never ahead of the other armrest's front end (it only helps the side that lags) (step 2,
    where the other side is pushed through the button at the middle)."""
    def fn():
        bb = ctx.obj("booster")
        want = _front(bb, sy) + INTO * np.asarray(bb.rotation()[:, 0], float)
        want[0] = min(want[0], back_goal + ARM_FRONT_X + OVER)
        if even:
            want[0] = min(want[0], _front(bb, -sy)[0])
        want[1], want[2] = sy * PUSH_Y, CUSH_TOP + PUSH_Z  # where the armrest is with the booster centred: the
        # hand's grip on the armrest end drags it back toward the centre line if it drifts sideways
        return _tcp(want)
    return fn


def demo_script(ctx):
    """Step 1: both closed hands push the armrests' front ends until both connectors click onto their bars (the guides
    take out the 2 deg turn). Step 2: the hand on the side already nearer home holds the adjustment button in while the
    other pushes its armrest until the booster's back is flush; button released first (arms lock), then back away."""
    L, R = ctx.left, ctx.right
    b = ctx.obj("booster")
    x_axis = np.asarray(b.rotation()[:, 0], float)
    pre = {sy: _front(b, sy) - 0.05 * x_axis for sy in (1, -1)}
    yield from ctx.both(L.move(_tcp(pre[1]) + [0, 0, 0.04], 2.0, FINGER_CLOSED, rot=LEVEL),
                        R.move(_tcp(pre[-1]) + [0, 0, 0.04], 2.0, FINGER_CLOSED, rot=LEVEL))
    yield from ctx.both(L.move(_tcp(pre[1]), 1.0, rot=LEVEL), R.move(_tcp(pre[-1]), 1.0, rot=LEVEL))
    for _ in range(12):  # step 1 in 1 s stages; stop once both catches have clicked
        yield from ctx.both(L.servo(_pusher(ctx, 1, CLICK_BACK), 1.0, rot=LEVEL, max_speed=0.05),
                            R.servo(_pusher(ctx, -1, CLICK_BACK), 1.0, rot=LEVEL, max_speed=0.05))
        if not ctx.offline:
            bb = ctx.obj("booster")
            if latched(bb, "l") and latched(bb, "r"):
                break
    # step 2: the hand on the side that is further home goes to the button; the other pushes its armrest
    bb = ctx.obj("booster")
    yaw = math.atan2(float(bb.rotation()[1, 0]), float(bb.rotation()[0, 0]))
    ahead = 1 if yaw <= 0 else -1  # turned clockwise: its left side is nearer home
    B, P = (L, R) if ahead == 1 else (R, L)
    yield from B.move(B.pos - [0.06, 0.0, 0.0], 0.8, rot=LEVEL)
    ax = np.asarray(bb.rotation()[:, 0], float)
    yield from B.move(_tcp(_button(bb) - 0.04 * ax, PRESS) + [0, 0, 0.04], 1.2, rot=PRESS)
    yield from B.move(_tcp(_button(bb) - 0.02 * ax, PRESS), 0.8, rot=PRESS)

    def presser():
        b2 = ctx.obj("booster")
        want = _button(b2) + 0.060 * np.asarray(b2.rotation()[:, 0], float)  # 8 mm stroke, then push the booster
        want[0] = min(want[0], BACK_X + BUTTON_AT[0] - 0.012 + 0.060 + 0.02)
        want[1] = 0.0
        return _tcp(want, PRESS)

    yield from B.servo(presser, 0.8, rot=PRESS, max_speed=0.05)
    for _ in range(10):
        if ahead == 1:
            yield from ctx.both(B.servo(presser, 1.0, rot=PRESS, max_speed=0.06),
                                P.servo(_pusher(ctx, -1, BACK_X, even=True), 1.0, rot=LEVEL, max_speed=0.025))
        else:
            yield from ctx.both(P.servo(_pusher(ctx, 1, BACK_X, even=True), 1.0, rot=LEVEL, max_speed=0.025),
                                B.servo(presser, 1.0, rot=PRESS, max_speed=0.06))
        if not ctx.offline and state(ctx.states())["flush"]:
            break
    yield from ctx.both(L.hold(0.5), R.hold(0.5))
    yield from B.move(B.pos - [0.06, 0.0, 0.0], 0.8, rot=PRESS)  # button out first: the arms lock
    yield from P.hold(0.5)
    if os.environ.get("ISOFIX_PULLTEST") and not ctx.offline:
        yield from _pull_test(ctx)
    yield from P.move(P.pos - [0.08, 0.0, 0.0], 1.0, rot=LEVEL)
    yield from ctx.both(L.move(L.pos + [0.0, 0.0, 0.12], 1.0, rot=LEVEL), R.move(R.pos + [0.0, 0.0, 0.12], 1.0, rot=LEVEL))
    yield from ctx.both(L.home(), R.home())  # from above the booster, so the swing home cannot sweep it
    yield from ctx.wait(1.5)


PULL_OPEN = -0.030  # pads 60 mm apart round the 40 mm armrest


def _pull_test(ctx):
    """Retention check, run only with ISOFIX_PULLTEST=1 (never in a scored demo): take each armrest's front end in
    the hand and pull the latched booster 30 mm back toward the robot, hold, report how far it came, let go."""
    L, R = ctx.left, ctx.right
    b0 = ctx.obj("booster")
    yield from ctx.both(L.move(L.pos - [0.06, 0.0, 0.0], 1.0, PULL_OPEN, rot=LEVEL),
                        R.move(_tcp(_front(b0, -1) - [0.06, 0.0, 0.0]), 1.5, PULL_OPEN, rot=LEVEL))
    grab = {sy: _front(b0, sy) + 0.030 * np.asarray(b0.rotation()[:, 0], float) for sy in (1, -1)}
    yield from ctx.both(L.move(_tcp(grab[1]), 1.2, rot=LEVEL), R.move(_tcp(grab[-1]), 1.2, rot=LEVEL))
    yield from ctx.both(L.grip(FINGER_CLOSED, 0.8), R.grip(FINGER_CLOSED, 0.8))
    yield from ctx.both(L.hold(0.4), R.hold(0.4))
    print(f"ISOFIXPULL holding l {L.holding(0.030)} r {R.holding(0.030)}", flush=True)
    yield from ctx.both(L.move(L.pos - [0.03, 0.0, 0.0], 1.5, rot=LEVEL), R.move(R.pos - [0.03, 0.0, 0.0], 1.5, rot=LEVEL))
    yield from ctx.both(L.hold(1.0), R.hold(1.0))
    b1 = ctx.obj("booster")
    print(f"ISOFIXPULL moved {(b0.origin[0] - b1.origin[0]) * 1000:.1f} mm latched l {latched(b1, 'l')} "
          f"r {latched(b1, 'r')}", flush=True)
    yield from ctx.both(L.move(L.pos + [0.03, 0.0, 0.0], 1.0, rot=LEVEL), R.move(R.pos + [0.03, 0.0, 0.0], 1.0, rot=LEVEL))
    yield from ctx.both(L.grip(PULL_OPEN, 0.6), R.grip(PULL_OPEN, 0.6))


def _debug(objects, s):
    n = _debug.n = getattr(_debug, "n", 0) + 1
    if n % 10:
        return
    b = objects["booster"]
    lb = {k: np.round((b.to_local((BAR_X, sy * BAR_Y, BAR_Z)) - [EXT + b.joints["arms"], 0, 0]) * 1000, 1)
          for k, sy in (("l", 1), ("r", -1))}
    print(f"ISOFIXDBG step {n} origin ({b.origin[0]:.3f},{b.origin[1]:.3f},{b.origin[2] - CUSH_TOP:.4f}) "
          f"tilt {s['tilt']:.1f} arms {b.joints['arms'] * 1000:.0f}mm "
          f"button {b.joints['button'] * 1000:.1f}mm gap {s['gap'] * 1000:.1f}mm catch l {b.joints['catch_l']:.2f} r {b.joints['catch_r']:.2f} "
          f"bar_local_mm l {lb['l']} r {lb['r']} latched l {s['l']} r {s['r']} hazard {_TRIAL.get('hazard')}",
          flush=True)
