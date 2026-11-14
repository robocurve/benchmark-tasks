"""On the station table: one arm of a small delta robot with its parallel-rod pair off. Near the robot, a blue-grey tray
holds the pair: two matt black 20 mm tubes, 226 mm long, joined by two extension springs, with a white cup housing at each
end (four cups, mouths facing each other). To the left (+y), the upper arm's end: a light grey boss with two chrome 15 mm
balls 56 mm apart, on a grey cast arm that rises to a black motor on a post. To the right (-y), lower down, the dark grey
hexagonal platform on a service post, with a light grey lug and two more chrome balls. The pair goes back on: spread
against its springs, set between the two ball pairs, and let close so each cup holds a ball.

Requires -E object_jitter=0: the start pose decides the scored positions.

Run it:  inspect-robots run --embodiment yam_arena -E task=refit_a_delta_robots_sprung_parallel_rod_pair.py \\
           -E object_jitter=0 --instruction "Take the black rod pair from the tray, clip its four cups onto the arm's and
           the platform's balls, then let go." --scorer task_score ...
"""

import dataclasses
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import closedloop  # noqa: E402, F401
import mechkit  # noqa: E402
from belimits import tilt_deg  # noqa: E402,F401
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, ball, box, cyl, extrude, on_reset, released, ring_profile, rod  # noqa: E402

from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips the pair: let it open the grippers and park clear before the episode ends

BLACK, WHITE, CHROME = (0.05, 0.05, 0.06), (0.97, 0.97, 0.95), (0.86, 0.88, 0.92)
TRAY_C, LIGHT, DARK, STEEL, CAST = (0.42, 0.55, 0.70), (0.80, 0.81, 0.83), (0.27, 0.28, 0.31), (0.62, 0.64, 0.67), (0.60, 0.61, 0.63)


def _look(shape):
    """A look-only part: shown, never collides."""
    return dataclasses.replace(shape, collide=False)


# -- the fixed ball pairs ------------------------------------------------------------------------------------------------
CX = 0.260  # both ball pairs are centred here; ball axes run along x
BALL_R, BALL_HALF = 0.0075, 0.028  # 15 mm balls, 56 mm apart
ARM_Y, ARM_Z = 0.160, 0.110  # the upper arm's ball pair (z above the table top)
PLAT_Y, PLAT_Z = -0.080, 0.040  # the platform's ball pair
SLOPE = float(np.arctan2(ARM_Z - PLAT_Z, ARM_Y - PLAT_Y))  # 16.3 deg: how the seated pair leans
BALLS = np.array([(CX + sx * BALL_HALF, y, TOP + z) for y, z in ((ARM_Y, ARM_Z), (PLAT_Y, PLAT_Z)) for sx in (-1, 1)])


def _ball_pair(y, z):
    return [cyl((CX, y, z), 0.004, 0.045, STEEL, axis="x")] + [ball((CX + sx * BALL_HALF, y, z), BALL_R, CHROME) for sx in (-1, 1)]


# Upper arm end: boss with the stud and two balls; the cast arm, motor and post behind it are look-only.
MOTOR_Y, MOTOR_Z = ARM_Y + 0.040, ARM_Z + 0.130
_ARM_OUTLINE = [(0.008, -0.008), (0.030, -0.008), (0.052, 0.100), (0.020, 0.112), (0.020, 0.008), (0.008, 0.008)]  # (y, z)
ARM_END = Mech("upper_arm_end", pos=(0.0, 0.0, TOP), mass=None, friction=0.2, shapes=[
    box((CX, ARM_Y, ARM_Z), (0.032, 0.016, 0.016), LIGHT),
    *_ball_pair(ARM_Y, ARM_Z),
    _look(extrude((CX, ARM_Y, ARM_Z), _ARM_OUTLINE, 0.020, CAST, axis="x")),
    _look(cyl((CX, MOTOR_Y, MOTOR_Z), 0.040, 0.070, BLACK, axis="x")),
    _look(cyl((CX - 0.039, MOTOR_Y, MOTOR_Z), 0.046, 0.008, CAST, axis="x")),  # motor flange
    _look(box((CX, MOTOR_Y, (MOTOR_Z - 0.036) / 2), (0.030, 0.030, MOTOR_Z - 0.036), BLACK)),
])

# Platform: hexagon 100 mm across flats on a service post, one working lug (+y flat) and two look-only lugs.
HEX_C, HEX_Z = (CX, PLAT_Y - 0.075), (PLAT_Z - 0.010, PLAT_Z + 0.010)
_HEX = [(0.05 / np.cos(np.pi / 6) * np.cos(a), 0.05 / np.cos(np.pi / 6) * np.sin(a)) for a in np.arange(6) * np.pi / 3]


def _spare_lug(deg):
    a = np.radians(deg)
    n, t = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    c = np.array(HEX_C) + 0.065 * n
    q = (float(np.cos((a - np.pi / 2) / 2)), 0.0, 0.0, float(np.sin((a - np.pi / 2) / 2)))
    out = [box((c[0], c[1], PLAT_Z), (0.032, 0.032, 0.016), LIGHT, tilt=q)]
    for s in (-1, 1):
        b = c + 0.010 * n + s * BALL_HALF * t
        out.append(ball((b[0], b[1], PLAT_Z), BALL_R, CHROME))
        tip = b + 0.012 * n
        out.append(rod((b[0] + s * 0.014 * t[0], b[1] + s * 0.014 * t[1], PLAT_Z),
                       (tip[0] + s * 0.014 * t[0], tip[1] + s * 0.014 * t[1], PLAT_Z + 0.045), 0.010, BLACK))
    return [_look(s) for s in out]


PLATFORM = Mech("platform", pos=(0.0, 0.0, TOP), mass=None, friction=0.2, shapes=[
    extrude((HEX_C[0], HEX_C[1], PLAT_Z), _HEX, 0.020, DARK, axis="z"),
    box((CX, PLAT_Y - 0.010, PLAT_Z), (0.032, 0.032, 0.016), LIGHT),
    *_ball_pair(PLAT_Y, PLAT_Z),
    cyl((HEX_C[0], HEX_C[1], (0.008 + HEX_Z[0]) / 2), 0.020, HEX_Z[0] - 0.008, STEEL),
    box((HEX_C[0], HEX_C[1], 0.004), (0.080, 0.080, 0.008), STEEL),
    *_spare_lug(210), *_spare_lug(330),
])

# -- the tray ---------------------------------------------------------------------------------------------------------------
TRAY_X, TRAY_Y, REST_Z = 0.155, 0.000, 0.030  # tray centre; the tube axes rest 30 mm above the table
_N = 0.012  # half the notch width
_CRADLE = [(-0.045, 0.0), (0.045, 0.0), (0.045, 0.022), (0.028 + _N, 0.022), (0.028 + _N, 0.012), (0.028 - _N, 0.012),
           (0.028 - _N, 0.022), (-0.028 + _N, 0.022), (-0.028 + _N, 0.012), (-0.028 - _N, 0.012), (-0.028 - _N, 0.022),
           (-0.045, 0.022)]  # (x, z): 90 x 22 mm with two 24 mm notches 10 mm deep
TRAY = Mech("tray", pos=(0.0, 0.0, TOP), mass=None, friction=0.6, shapes=[
    box((TRAY_X, TRAY_Y, 0.004), (0.110, 0.300, 0.008), TRAY_C),
    *[extrude((TRAY_X, y, 0.008), [(z, x) for x, z in _CRADLE], 0.012, TRAY_C, axis="y") for y in (TRAY_Y - 0.045, TRAY_Y + 0.045)],
])

# -- the rod pair (free) ------------------------------------------------------------------------------------------------------
# Frame: origin at the middle of rod A's tube axis, y along the tubes, x toward rod B. Rod B rides an x slide that stands
# for the two extension springs: q = seat spacing minus 44 mm, 0 (coils closed) to 50 mm (rated stretch).
TUBE_R, TUBE_L, HALF_L = 0.010, 0.226, 0.125
SEAT_IN = 0.006  # a rod's seat points lie 6 mm inside its tube axis, toward the other rod
CLOSED = 0.056  # tube axes this far apart at q = 0 (seats 44 mm apart)
CUP_R, CUP_DEPTH, CUP_FLOOR = 0.014, 0.009, 0.004
BALL_IN_CUP = 0.0086  # a seated 15 mm ball's centre above the cone's narrow row (7.5 mm / sin 45 deg minus 2 mm)
Q_SEATED, Q_MAX, SPRING = 0.012, 0.050, 120.0
_FLIP = (0.0, 0.0, 0.0, 1.0)  # half a turn about z


def _rod_shapes(sign):
    """One rod: tube, two cup housings with their mouths toward +x (sign 1) or -x (sign -1), look-only spring coils."""
    tilt = None if sign > 0 else _FLIP
    x0 = sign * (SEAT_IN - BALL_IN_CUP)
    out = [cyl((0.0, 0.0, 0.0), TUBE_R, TUBE_L, BLACK, axis="y")]
    for y in (-HALF_L, HALF_L):
        kw = {} if tilt is None else {"tilt": tilt}
        out.append(ring_profile((x0, y, 0.0), [(0.002, CUP_R, 0.0), (0.011, CUP_R, CUP_DEPTH)], WHITE, axis="x", sides=12,
                                floor=CUP_FLOOR, **kw))
    for y in (-HALF_L + 0.030, HALF_L - 0.030):
        out.append(_look(cyl((sign * 0.025, y, 0.0), 0.0035 if sign > 0 else 0.0025, 0.050, STEEL, axis="x")))
    return out


PAIR_START = (TRAY_X - CLOSED / 2, TRAY_Y, TOP + REST_Z)
# Friction 0.4: acetal cups must slide down the chrome balls to centre (a 45 degree cone locks at 1.0).
PAIR = Mech("rod_pair", pos=PAIR_START, mass=0.055, friction=0.4, shapes=_rod_shapes(1), joints=[
    Joint("spread", "slide", axis="x", at=(CLOSED, 0.0, 0.0), low=0.0, high=Q_MAX, spring=SPRING, damping=1.0, mass=0.055,
          friction=0.4, shapes=_rod_shapes(-1))])

OBJECTS = [TRAY, ARM_END, PLATFORM, PAIR]

# -- scoring ----------------------------------------------------------------------------------------------------------------
ON_TOL = 0.0025  # a cup is on when its seat point is within 2.5 mm of a ball centre
SPREAD_Q = 0.078 - 0.044  # seats 78 mm apart: the cup mouths clear the balls' outer faces (71 mm) by 3 mm a side
OVER_Q = 0.092 - 0.044  # seats 92 mm apart: 2 mm short of the springs' rated stretch (94 mm, the slide's stop)
OFF_TRAY = 0.040  # the whole pair this far above the table: clear of the 30 mm cradles
DROP_Z, DROP_V = 0.010, 0.6
STILL_V, STILL_STEPS, RELEASE_CLEAR = 0.01, 20, 0.03
OFFLINE_FINGERS_CLEAR = True  # what the release test returns with no simulator (the checker flips it for a held pair)


class _Latch:
    def __init__(self):
        self.clear()

    def clear(self):
        self.spread = self.dropped = self.over = False
        self.still = 0
        self.prev = None
        self.seen = None


LATCH = _Latch()


@on_reset
def _reset_latch(env=None):
    LATCH.clear()


def seat_points(p, q=None) -> np.ndarray:
    """World positions of the four seat points (where a ball centre sits in each cup): rod A -y, +y, rod B -y, +y."""
    q = float(p.joints.get("spread", 0.0)) if q is None else q
    return np.array([p.to_world((x, y, 0.0)) for x in (SEAT_IN, CLOSED + q - SEAT_IN) for y in (-HALF_L, HALF_L)])


def cups_on(p) -> int:
    """How many balls have a seat point within 2.5 mm of their centre."""
    s = seat_points(p)
    return int(sum(np.min(np.linalg.norm(s - b, axis=1)) <= ON_TOL for b in BALLS))


def _fingers_clear(p) -> bool:
    """Both grippers 3 cm or more from the pair (its box in rod A's frame). Offline replays have no fingers to measure."""
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    q = float(p.joints.get("spread", 0.0))
    return released(p, RELEASE_CLEAR, lo=(-CUP_R, -HALF_L - CUP_R, -CUP_R), hi=(CLOSED + q + CUP_R, HALF_L + CUP_R, CUP_R))


def _update(objects):
    p = objects["rod_pair"]
    if LATCH.seen is objects:  # success() and score() are both called on one dict each step: count the step once
        return p
    LATCH.seen = objects
    d = float(np.linalg.norm(np.asarray(p.origin, float) - np.asarray(PAIR_START, float)))
    v = float(np.linalg.norm(p.lin_vel))
    if LATCH.prev is not None and LATCH.prev > 0.03 and d <= 0.005 and v < 0.01:
        LATCH.clear()  # the pair teleported back to the tray: a trial reset
        LATCH.seen = objects
    LATCH.prev = d
    q = float(p.joints.get("spread", 0.0))
    low = float(p.low[2]) - TOP
    if q >= SPREAD_Q and low >= OFF_TRAY:
        LATCH.spread = True
    if q >= OVER_Q:
        LATCH.over = True
    if low < DROP_Z or v > DROP_V:
        LATCH.dropped = True
    LATCH.still = LATCH.still + 1 if (v < STILL_V and _fingers_clear(p)) else 0
    return p


def is_released() -> bool:
    return LATCH.still >= STILL_STEPS


def score(objects):
    """First row that fits. 0: the pair was dropped (any part under 10 mm above the table, or faster than 0.6 m/s) or
    overstretched (seats 92 mm or more apart) at any time. 1.0, 0.75, 0.5, 0.25: released (both grippers 3 cm or more
    from the pair and the pair under 1 cm/s for the last 1 s) with 4, 3, 2, 1 cups on their balls. 0.25: the pair was
    spread to 78 mm or more off the tray at some time. Else 0. A pair still held never counts as released."""
    p = _update(objects)
    if LATCH.dropped or LATCH.over:
        return 0.0
    n = cups_on(p) if is_released() else 0
    if n:
        return 0.25 * n
    return 0.25 if LATCH.spread else 0.0


def success(objects):
    return score(objects) == 1.0


# The pinch zones: a box round each ball pair, where the sprung cups snap shut. The hands work on the tubes, never there.
# The look-only motor and its post are listed too, so the offline sweep shows no arm part passes through them.
ARM_KEEP_OUT = [
    {"name": "pinch zone: upper arm ball pair", "x": (CX - 0.07, CX + 0.07), "y": (ARM_Y - 0.025, ARM_Y + 0.025), "z_top": TOP + ARM_Z + 0.04},
    {"name": "pinch zone: platform ball pair", "x": (CX - 0.07, CX + 0.07), "y": (PLAT_Y - 0.025, PLAT_Y + 0.025), "z_top": TOP + PLAT_Z + 0.04},
    {"name": "motor and post", "x": (CX - 0.045, CX + 0.045), "y": (MOTOR_Y - 0.045, MOTOR_Y + 0.045), "z_top": TOP + MOTOR_Z + 0.045},
]

# -- the scripted demo (both arms, closed loop) -------------------------------------------------------------------------------
demo = placeholder_demo

GRIP_Y = 0.080  # each hand grips its tube 80 mm from mid-length (closer and the two wrists touch): left on rod B (+y), right on rod A (-y)
OPEN40 = FINGER_OPEN * 40.0 / 94.0  # pads 40 mm apart
CLOSE8 = FINGER_OPEN * 8.0 / 94.0  # an 8 mm command on a 20 mm tube
OPEN60 = FINGER_OPEN * 60.0 / 94.0  # pads 60 mm apart to let go: 20 mm a side round the tube
GRIP_LOW = 0.006  # tool point this far below the tube axis, so the tube sits deeper between the pads
WAY_X = 0.140  # the hands travel to and from the tray along this x, on the robot side of both ball pairs
FEED = 1.2  # feed-forward on the spread command for the arms' give
HOVER_Z, LIFT_Z, OVER = 0.110, 0.080, 0.030
Q_OPEN = 0.040  # spread to an 84 mm seat spacing: mouths 6 mm outside the balls
SEAT_C = np.array([CX, (ARM_Y + PLAT_Y) / 2, TOP + (ARM_Z + PLAT_Z) / 2])  # the seated pair's centre
YAW = {"left": -np.pi / 2, "right": np.pi / 2}


def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rotvec(r):
    """Rotation vector of a rotation matrix (small angles)."""
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / 2.0
    n = float(np.linalg.norm(w))
    return w if n < 1e-9 else w * (np.arcsin(min(1.0, n)) / n)


def _exp(w):
    a = float(np.linalg.norm(w))
    if a < 1e-9:
        return np.eye(3)
    k = np.asarray(w) / a
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * K @ K


def _clamp(v, m):
    n = float(np.linalg.norm(v))
    return v if n <= m else v * (m / n)


def demo_script(ctx):
    """Both arms grip the tubes in the tray, lift the pair level, spread it to an 84 mm seat spacing, lean it to the
    16.3 degree slope of the two ball pairs, carry it over them, lower it so the cup mouths pass outside the balls, bring
    the rods together until the cups sit on the balls, open both grippers and park."""
    L, R = ctx.left, ctx.right
    rot0 = {"left": grip_rot(yaw=YAW["left"]), "right": grip_rot(yaw=YAW["right"])}
    live = ctx.states() is not ctx.states()  # an offline replay hands back the same start poses every time

    # 1. grip both tubes in the tray
    def to_tube(arm, x, y):
        p = np.array([x, y, TOP + REST_Z - GRIP_LOW])
        r = rot0[arm.side]
        sg = 1.0 if arm.side == "left" else -1.0
        # come in from the robot's side of the table, round the motor and the pinch zones, never across them
        yield from arm.move((WAY_X, sg * 0.290, TOP + 0.200), 2.0, OPEN40, rot=r)
        yield from arm.move((WAY_X, sg * 0.100, TOP + 0.160), 2.0, OPEN40, rot=r)
        yield from arm.move(p + np.array([0.0, 0.0, HOVER_Z - REST_Z]), 1.5, OPEN40, rot=r)
        yield from arm.trim_to_measured(p + np.array([0.0, 0.0, HOVER_Z - REST_Z]), tol=0.002, max_comp=0.03, max_duration=0.8)
        yield from arm.move(p, 2.5, OPEN40, rot=r)
        yield from arm.trim_to_measured(p, tol=0.0015, max_comp=0.03, step_mps=0.03, max_duration=0.8)
        yield from arm.grip(CLOSE8, 0.8)
        yield from arm.hold(0.3)
        if not arm.holding(0.013):  # pads did not stop on the tube (they read about 16 mm on it): open, go 3 mm lower, try once more
            yield from arm.grip(OPEN40, 0.5)
            yield from arm.move(p + np.array([0.0, 0.0, -0.003]), 0.6, OPEN40, rot=r)
            yield from arm.grip(CLOSE8, 0.8)
            yield from arm.hold(0.3)

    yield from ctx.both(to_tube(L, TRAY_X + CLOSED / 2, TRAY_Y + GRIP_Y), to_tube(R, TRAY_X - CLOSED / 2, TRAY_Y - GRIP_Y))
    if not (L.holding(0.013) and R.holding(0.013)):
        yield from ctx.both(L.home(3.0), R.home(3.0))
        return

    # The pair is steered as one body: a wanted pose (centre between the rods at mid-length, lean, q) gives both grip
    # points; while the pair hangs free in the hands, its live pose and q trim that command.
    cur = {"c": np.array([TRAY_X, TRAY_Y, TOP + REST_Z]), "a": 0.0, "q": 0.0}
    trim = {"t": np.zeros(3), "w": np.zeros(3), "q": 0.0}

    def command(c, a, q, pose_fb=True, q_fb=True):
        rd = _Rx(a)
        if live:
            st = ctx.obj("rod_pair")
            qa = float(st.joints.get("spread", 0.0))
            if pose_fb:
                ca = st.to_world(((CLOSED + qa) / 2, 0.0, 0.0))
                trim["t"] = _clamp(trim["t"] + 0.10 * (c - ca), 0.03)
                trim["w"] = _clamp(trim["w"] + 0.10 * _rotvec(rd @ st.rotation().T), 0.12)
            if q_fb:  # the arms give several cm under the springs' pull: overdrive until q itself is right
                trim["q"] = float(np.clip(trim["q"] + 0.15 * (q - qa), -0.03, 0.10))
        rc = _exp(trim["w"]) @ rd
        half = (CLOSED + q * (1.0 + FEED) + trim["q"]) / 2
        cc = c + trim["t"]
        L._set_cartesian(cc + rc @ np.array([half, GRIP_Y, -GRIP_LOW]), rc @ rot0["left"], CLOSE8)
        R._set_cartesian(cc + rc @ np.array([-half, -GRIP_Y, -GRIP_LOW]), rc @ rot0["right"], CLOSE8)

    def go(c=None, a=None, q=None, dur=1.0):
        c0, a0, q0 = cur["c"].copy(), cur["a"], cur["q"]
        c1 = c0 if c is None else np.asarray(c, float)
        a1 = a0 if a is None else a
        q1 = q0 if q is None else q
        n = max(1, round(dur / ctx.dt))
        for i in range(1, n + 1):
            s = closedloop.ik.smoothstep(i / n)
            cur.update(c=c0 + s * (c1 - c0), a=a0 + s * (a1 - a0), q=q0 + s * (q1 - q0))
            command(cur["c"], cur["a"], cur["q"])
            yield

    def q_now():
        return float(ctx.obj("rod_pair").joints.get("spread", 0.0)) if live else cur["q"]

    def settle(max_s=4.0, tol=0.0015):
        """Hold the wanted pose until the pair's centre, lean and q have been within tolerance for 0.5 s."""
        good = 0
        for _ in range(round(max_s / ctx.dt)):
            command(cur["c"], cur["a"], cur["q"])
            yield
            if not live:
                break
            st = ctx.obj("rod_pair")
            qa = float(st.joints.get("spread", 0.0))
            err = np.linalg.norm(cur["c"] - st.to_world(((CLOSED + qa) / 2, 0.0, 0.0)))
            turn = np.linalg.norm(_rotvec(_Rx(cur["a"]) @ st.rotation().T))
            good = good + 1 if (err < tol and abs(qa - cur["q"]) < tol and turn < 0.012) else 0
            if good >= 10:
                break

    # 2. lift level, clear of the cradles
    yield from go(c=(TRAY_X, TRAY_Y, TOP + LIFT_Z), dur=2.0)
    # 3. spread against the springs, reading q, to an 84 mm seat spacing
    yield from go(q=Q_OPEN, dur=3.0)
    yield from settle(2.0)
    # 4. lean to the slope and rise, carry over the balls, then lower with every seat point beside its ball
    over = SEAT_C + np.array([0.0, 0.0, OVER])
    yield from go(c=(TRAY_X, SEAT_C[1], over[2]), a=SLOPE, dur=2.5)
    yield from go(c=over, dur=3.0)
    yield from settle(3.0)
    yield from go(c=SEAT_C, dur=3.0)
    yield from settle(2.0)
    # 5. close: q falls 10 mm/s, then 4 mm/s below a 62 mm spacing (each grip half that), until q stops falling for 0.5 s
    best, stall = q_now(), 0
    while cur["q"] > 0.006:
        cur["q"] -= (0.010 if cur["q"] > 0.018 else 0.004) * ctx.dt
        command(cur["c"], cur["a"], cur["q"], pose_fb=False)  # the balls now set the pose: only q is trimmed
        yield
        qn = q_now()
        if qn < best - 0.0002:
            best, stall = qn, 0
        else:
            stall += 1
        if (stall >= 10 and qn < 0.016) or (not live and cur["q"] <= Q_SEATED):
            break
    yield from ctx.both(L.hold(0.5), R.hold(0.5))

    # 6. let go: open to 40 mm, rise 6 cm, leave by the robot side, park
    def leave(arm):
        for _ in range(2):  # take the arm's own push off the tube first, so opening does not jolt the pair
            yield from arm.move(arm.measured_pos.copy(), 0.4, CLOSE8, rot=arm.rot.copy())
        yield from arm.grip(OPEN60, 1.0)
        yield from arm.hold(0.3)
        sg = 1.0 if arm.side == "left" else -1.0
        yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.06]), 1.5, OPEN60, rot=arm.rot.copy())
        # leave the way it came: back toward the robot, then out to its own side, then the ready pose
        yield from arm.move((WAY_X, sg * 0.100, TOP + 0.160), 1.5, OPEN60, rot=rot0[arm.side])
        yield from arm.move((WAY_X, sg * 0.290, TOP + 0.200), 1.5, OPEN60, rot=rot0[arm.side])
        yield from arm.home(2.0)

    yield from ctx.both(leave(L), leave(R))
    yield from ctx.wait(2.0)
