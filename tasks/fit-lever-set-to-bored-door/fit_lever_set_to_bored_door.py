"""Fit a passage lever set to a bored door section and screw it up with the outside lever held level.

A painted solid-core door section (1-3/8 in thick) stands on a steel base plate, bored for a tubular latch (2-1/8 in
cross bore, 2-3/8 in backset), with the latch already fitted. The outside half of the lever set (lever, rose, flat
spindle blade and two hollow mounting posts) hangs on its holder to the robot's right; the interior lever base (rose,
square hub, two exposed #8-32 screws) hangs on its holder to the robot's left, and a stubby ratcheting #2 Phillips
screwdriver lies on its rest. The outside half's blade and posts go through the latch, the base goes over the blade
and the posts' ends, and both screws are driven while the outside lever is held level: let go, the outside half
droops in its posts' play and the screws lock it crooked. All dimensions are metres.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import lockset_runtime as LR  # noqa: E402
from yam_parts import Part, ball, box, cyl  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_OPEN, READY, SHOULDER, WRIST  # noqa: E402
import yam_arena.demo as motion  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = 0.7475

# ---- door, bores and latch (door frame: origin at the cross bore's centre, on the door's centre plane) ----------
T = 0.0349  # 1-3/8 in door
X0, Z0 = 0.290, 0.120  # cross-bore centre (x from the arm-base line; z above the table top)
LEVER_SIGN = -1  # the levers point along x by this sign, away from the latch edge
BACKSET = 0.0603  # 2-3/8 in: the latch edge is at x = X0 - LEVER_SIGN * BACKSET
DOOR_WIDTH = 0.2503
BORE_R = 0.027  # 2-1/8 in cross bore
DOOR_BOTTOM, DOOR_TOP = -Z0 + 0.010, -Z0 + 0.260  # door section 250 tall, on a 10 mm plate
POST_Z = 0.019  # posts and screws 19 mm above and below the spindle (estimate)
POST_R = 0.0032  # 6.4 mm hollow posts (estimate)
LATCH_HOLE = 0.00395  # half-width of the latch frame's post holes: 0.75 mm radial play (estimate)
LATCH_DEPTH = 0.020  # the latch frame's post holes run 20 mm through the door's core
BLADE_X, BLADE_Z = 0.00125, 0.00475  # half-sizes of the flat spindle blade (2.5 x 9.5 mm, estimate)
CAM_PLAY = 0.0006  # the blade's play in the latch cam's slot
CAM_MOUTH_Y = -0.006  # the cam's slot starts 6 mm short of the door's centre plane (door frame y)

WHITE, STEEL, BRONZE, BRASS, DARK = (0.93, 0.93, 0.90), (0.62, 0.63, 0.66), (0.36, 0.22, 0.12), (0.78, 0.62, 0.30), (0.22, 0.22, 0.24)
BLACK, YELLOW, BLUE = (0.08, 0.08, 0.08), (0.95, 0.75, 0.10), (0.20, 0.35, 0.75)


def channel(cx, cz, hx, hz, y0, y1, wall=0.0025, color=STEEL):
    """A rectangular tube along y: inner half-widths hx (x) and hz (z), from y0 to y1."""
    yc, ly = (y0 + y1) / 2, y1 - y0
    return [box((cx - hx - wall / 2, yc, cz), (wall, ly, 2 * hz + 2 * wall), color),
            box((cx + hx + wall / 2, yc, cz), (wall, ly, 2 * hz + 2 * wall), color),
            box((cx, yc, cz - hz - wall / 2), (2 * hx, ly, wall), color),
            box((cx, yc, cz + hz + wall / 2), (2 * hx, ly, wall), color)]


def make_door():
    s = []
    h = T
    e = -LEVER_SIGN  # the edge's side of the bore
    edge = e * BACKSET
    far = edge + LEVER_SIGN * DOOR_WIDTH
    xa, xb = min(edge, far), max(edge, far)
    zc, zh = (DOOR_BOTTOM + DOOR_TOP) / 2, DOOR_TOP - DOOR_BOTTOM
    # wood around the 54 mm cross bore (its corners filled to approximate the round bore)
    s.append(box(((xa - BORE_R) / 2, 0, zc), (-BORE_R - xa, h, zh)))
    s.append(box(((BORE_R + xb) / 2, 0, zc), (xb - BORE_R, h, zh)))
    s.append(box((0, 0, (BORE_R + DOOR_TOP) / 2), (2 * BORE_R, h, DOOR_TOP - BORE_R)))
    s.append(box((0, 0, (DOOR_BOTTOM - BORE_R) / 2), (2 * BORE_R, h, -BORE_R - DOOR_BOTTOM)))
    for sx in (-1, 1):
        for sz in (-1, 1):
            s.append(box((sx * 0.023, 0, sz * 0.023), (0.008, h, 0.008)))
    # the tubular latch: case from the edge bore into the cross bore, frame with two post holes and the cam's slot
    s.append(box((e * 0.0170, 0, 0), (0.0200, 0.022, 0.022), STEEL))
    for sz in (-1, 1):
        s += channel(0.0, sz * POST_Z, LATCH_HOLE, LATCH_HOLE, -LATCH_DEPTH / 2, LATCH_DEPTH / 2)
    s += channel(0.0, 0.0, BLADE_X + CAM_PLAY, BLADE_Z + CAM_PLAY, CAM_MOUTH_Y, -CAM_MOUTH_Y, wall=0.002, color=BRASS)
    # faceplate (25.4 x 57 mm, mortised flush, 1 mm shown proud) and the bolt standing 13 mm out
    s.append(box((edge + e * 0.0005, 0, 0), (0.001, 0.0254, 0.057), BRASS))
    s.append(box((edge + e * 0.0075, 0, 0), (0.013, 0.010, 0.022), STEEL))
    # steel base plate under the section
    s.append(box(((xa + xb) / 2, 0, DOOR_BOTTOM - 0.005), (xb - xa + 0.040, 0.050, 0.010), DARK))
    return Part("door", (X0, 0.0, TOP + Z0), s, color=WHITE, friction=(0.40, 0.35))


# ---- the outside half (frame: origin at the rose's back face, on the spindle; seated = the door's frame shifted to
# the outside face; the spindle points +y, the lever -x, toward the robot) -----------------------------------------
ROSE_R = 0.03335  # 2-5/8 in rose
LEVER_LEN, LEVER_OUT = 0.107, 0.053  # 4-7/32 in long, 2-3/32 in projection
LEVER_HALF_Z = 0.008  # 16 mm tall handle (estimate)
LEVER_GRIP = 0.050  # the hand pinches the lever this far out from the hub's axis
POST_LEN = T + 0.005  # the posts reach 5 mm into the base's holes
BLADE_LEN = T + 0.004
OUT_MASS, IN_MASS, DRIVER_MASS = 0.30, 0.10, 0.15  # estimates


def make_outside(pos):
    s = [cyl((0, -0.0025, 0), ROSE_R, 0.005, "y"),
         cyl((0, -0.0075, 0), 0.024, 0.005, "y"),
         cyl((0, -0.026, 0), 0.009, 0.032, "y")]
    # the lever (from 9 mm behind the hub's axis to its tip) as three boxes, the middle one where the hand holds it
    xa, xb = sorted((LEVER_SIGN * LEVER_LEN, -LEVER_SIGN * 0.009))
    g = LEVER_SIGN * LEVER_GRIP
    for x0, x1 in ((xa, g - 0.015), (g - 0.015, g + 0.015), (g + 0.015, xb)):
        s.append(box(((x0 + x1) / 2, -(LEVER_OUT - 0.006), 0), (x1 - x0, 0.012, 2 * LEVER_HALF_Z)))
    for sz in (-1, 1):
        s.append(cyl((0, (POST_LEN - POST_R) / 2, sz * POST_Z), POST_R, POST_LEN - POST_R, "y", STEEL))
        s.append(ball((0, POST_LEN - POST_R, sz * POST_Z), POST_R, STEEL))
    s.append(box((0, (BLADE_LEN - 0.004) / 2, 0), (2 * BLADE_X, BLADE_LEN - 0.004, 2 * BLADE_Z), STEEL))
    s.append(box((0, BLADE_LEN - 0.002, 0), (0.0015, 0.004, 0.0065), STEEL))
    return Part("outside", pos, s, mass=OUT_MASS, color=BRONZE, friction=(0.40, 0.35), contact_offset=0.001)


# ---- the interior lever base (frame: origin at its back face, on the hub's axis; the hub points +y) ---------------
BASE_HOLE = 0.0035  # half-width of its post holes: 0.3 mm radial play on the posts (estimate)
BASE_PLATE = 0.006
HUB_X, HUB_Z, HUB_Y0, HUB_Y1 = 0.012, 0.010, 0.009, 0.044  # square hub the interior lever slides onto (estimate)
HEAD_R, HEAD_Y0, HEAD_Y1 = 0.0040, 0.009, 0.0114  # #8 pan heads on the rose


def make_inside(pos):
    p, hh, w = BASE_PLATE, BASE_HOLE, 0.0235
    sx, sz = BLADE_X + 0.0004, BLADE_Z + 0.0004
    yc = p / 2
    s = [box((-(w + hh) / 2, yc, 0), (w - hh, p, 2 * w)), box(((w + hh) / 2, yc, 0), (w - hh, p, 2 * w))]
    lo, hi = POST_Z - hh, POST_Z + hh  # 0.0155, 0.0225
    for sg in (-1, 1):
        s.append(box((0, yc, sg * (hi + w) / 2), (2 * hh, p, w - hi)))
        s.append(box((0, yc, sg * (lo + sz) / 2), (2 * hh, p, lo - sz)))
    for sg in (-1, 1):
        s.append(box((sg * (sx + hh) / 2, yc, 0), (hh - sx, p, 2 * sz)))
    s.append(cyl((0, p + 0.0015, 0), ROSE_R, 0.003, "y"))
    s.append(box((0, (HUB_Y0 + HUB_Y1) / 2, 0), (2 * HUB_X, HUB_Y1 - HUB_Y0, 2 * HUB_Z)))
    for sg in (-1, 1):
        s.append(cyl((0, (HEAD_Y0 + HEAD_Y1) / 2, sg * POST_Z), HEAD_R, HEAD_Y1 - HEAD_Y0, "y", STEEL))
    return Part("inside", pos, s, mass=IN_MASS, color=BRONZE, friction=(0.40, 0.35), contact_offset=0.001)


# ---- stubby ratcheting screwdriver (frame: origin at the bit's tip; the handle runs +y) --------------------------
SHAFT = 0.095  # bit and shaft out of the handle (200 mm overall)
HANDLE_X, HANDLE_Z, HANDLE_Y0, HANDLE_Y1 = 0.013, 0.0175, SHAFT + 0.005, SHAFT + 0.105  # flats 26 apart, 35 across
DRIVER_GRIP = (HANDLE_Y0 + HANDLE_Y1) / 2  # TCP across the handle's middle


def make_driver(pos):
    s = [cyl((0, 0.003, 0), 0.0020, 0.006, "y", STEEL), cyl((0, 0.0155, 0), 0.00318, 0.019, "y", STEEL),
         cyl((0, (0.025 + SHAFT) / 2, 0), 0.0035, SHAFT - 0.025, "y", STEEL),
         cyl((0, SHAFT + 0.0025, 0), 0.010, 0.005, "y", BLACK),
         box((0, (HANDLE_Y0 + HANDLE_Y1) / 2, 0), (2 * HANDLE_X, HANDLE_Y1 - HANDLE_Y0, 2 * HANDLE_Z), YELLOW)]
    return Part("driver", pos, s, mass=DRIVER_MASS, color=YELLOW, friction=(0.50, 0.40), contact_offset=0.001)


# ---- start layout --------------------------------------------------------------------------------------------------
OUT_SEATED = np.array((X0, -T / 2, TOP + Z0))
IN_SEATED = np.array((X0, T / 2, TOP + Z0))
OUT_START = np.array((0.340, -0.061, TOP + Z0))  # beside the door's outside face, lever toward the robot
IN_START = np.array((0.340, 0.080, TOP + Z0))
DRIVER_START = np.array((0.225, 0.035, TOP + 0.060))  # bit tip; the handle's flats face +-x (x 225: clear of
# the left gripper's housing while it carries and places the base at x 290)
HOLDER_PLAY = 0.0015  # the outside holder's post holes are 1.5 mm loose above the posts


def make_holder_out():
    """A grey steel stand whose block takes the outside half's posts and blade (the half hangs on its posts)."""
    hy0, hy1 = 0.025, 0.037  # post ends at y 0.0399 (half frame), block 3 mm short of them... in the half's frame
    s = []
    for sz in (-1, 1):
        # holes whose floor carries the post: centred POST_Z + HOLDER_PLAY/2 so the post sits on the floor at POST_Z
        s += channel(0.0, sz * POST_Z + HOLDER_PLAY / 2, POST_R + HOLDER_PLAY / 2, POST_R + HOLDER_PLAY / 2, hy0, hy1, 0.004, DARK)
    s += channel(0.0, 0.0, BLADE_X + 0.001, BLADE_Z + 0.001, hy0, hy1, 0.003, DARK)
    col_lo, col_hi = -Z0 + 0.006, -POST_Z - POST_R - 0.004  # from the foot's top to the lower post channel
    s.append(box((0, (hy0 + hy1) / 2, (col_lo + col_hi) / 2), (0.020, hy1 - hy0, col_hi - col_lo), DARK))
    s.append(box((0, 0.001, -Z0 + 0.003), (0.080, 0.060, 0.006), DARK))
    return Part("holder_out", OUT_START, s, color=DARK)


PEG_R = 0.0031


def make_holder_in():
    """A grey stand with two pegs that go 4.5 mm into the base's post holes (the base hangs on them)."""
    s = []
    for sz in (-1, 1):
        zc = sz * POST_Z + (BASE_HOLE - PEG_R)  # the hole's top wall rests on the peg
        s.append(cyl((0, -0.0040, zc), PEG_R, 0.0170, "y", DARK))
        s.append(ball((0, 0.0045 - PEG_R, zc), PEG_R, DARK))
    s.append(box((0, -0.0165, 0), (0.016, 0.008, 0.060), DARK))  # narrow: clear of the hand at the bore
    s.append(box((0, -0.0165, -0.030 - (Z0 - 0.036) / 2), (0.016, 0.008, Z0 - 0.036)))
    s.append(box((0, -0.0165, -Z0 + 0.003), (0.080, 0.060, 0.006), DARK))
    return Part("holder_in", IN_START, s, color=DARK)


def make_rest():
    """A narrow block the screwdriver's handle lies on (the fingers close on the handle's flats either side)."""
    top = DRIVER_START[2] - TOP - HANDLE_Z
    return Part("rest", (DRIVER_START[0], DRIVER_START[1], TOP),
                [box((0, (HANDLE_Y0 + HANDLE_Y1) / 2, top / 2), (0.016, HANDLE_Y1 - HANDLE_Y0 - 0.010, top)),
                 box((0, (HANDLE_Y0 + HANDLE_Y1) / 2, 0.003), (0.050, HANDLE_Y1 - HANDLE_Y0 + 0.010, 0.006))], color=BLUE)


OBJECTS = [make_door(), make_holder_out(), make_holder_in(), make_rest(),
           make_outside(OUT_START), make_inside(IN_START), make_driver(DRIVER_START)]
# The pads pinch the lever, the hub and the handle from the side, which the kinematic replay cannot model, and the
# demo presses the halves onto the door: those contacts are advisory offline and the GPU demo decides.
PRESSED_PARTS = ("outside", "inside", "driver")

# ---- scoring -----------------------------------------------------------------------------------------------------
SEAT_TOL = 0.0005  # rose rim within 0.5 mm of its door face all round
LEVEL_FULL, LEVEL_PART = 1.0, 3.0  # degrees
RELEASE_CLEAR, RELEASE_S = 0.030, 2.0
STILL_SPEED, STILL_SPIN = 0.005, 0.05  # offline fixtures only (m/s, rad/s)
STILL_WINDOW_S, STILL_MOVE, STILL_TURN_DEG = 1.0, 0.0002, 0.1  # on the simulator: under 0.2 mm and 0.1 deg in 1 s
TEST_TURNS = None  # offline fixtures set the screws' turns (the thread model runs only in the simulator)
TEST_GRIPPERS = None
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
CONTROL_HZ = 20


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run_lockset"):
        return cls._rc_run_lockset
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        LR.MODEL.clear()
        result = original_reset(self, *args, **kwargs)
        LR.MODEL.clear()  # whatever the settle steps did, every trial starts with loose screws
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        try:
            LR.install(self._env)
        except Exception as e:  # noqa: BLE001
            print(f"LOCKSET_HOOK failed: {e!r}", flush=True)
        return result

    cls._rc_run_lockset = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena.constants import ARM_JOINTS
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        q = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0]
        q = q.detach().cpu().numpy() if hasattr(q, "detach") else np.asarray(q)
        out[arm] = ik.tcp_pose(np.asarray(q)[ids], station.BASES[arm])[0]
    return out


def released(states):
    hands = grippers()
    if hands is None:
        return True  # offline the arms are not known: assumed clear
    for st in states:
        low, high = np.asarray(st.low), np.asarray(st.high)
        for tcp in hands.values():
            if np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) < RELEASE_CLEAR:
                return False
    return True


def held_for(key, cond):
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _MEMORY_EPISODE = _RUN["episode"]
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    prev, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or prev is None or step not in (prev, prev + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def _rot(st):
    return st.rotation()


def _origin(st):
    return np.asarray(st.origin, float)


def fit_terms(o_pos, o_R, i_pos, i_R):
    """Every scored quantity from the two halves' poses (environment frame, metres)."""
    ang = np.linspace(0, 2 * math.pi, 8, endpoint=False)
    rim_o = [o_pos + o_R @ np.array((0.030 * math.cos(a), 0.0, 0.030 * math.sin(a))) for a in ang]
    rim_i = [i_pos + i_R @ np.array((0.022 * math.cos(a), 0.0, 0.022 * math.sin(a))) for a in ang]
    gap_o = max(-T / 2 - p[1] for p in rim_o)  # how far the rose's rim stands off the outside face
    gap_i = max(p[1] - T / 2 for p in rim_i)
    lever = LEVER_SIGN * o_R[:, 0]  # from the hub to the lever's tip
    away = lever[0] * LEVER_SIGN > 0.9  # away from the latch edge
    spindle = o_R[:, 1]
    hub_off = math.hypot(o_pos[0] - X0, o_pos[2] - (TOP + Z0))
    blade_tip = o_pos + o_R @ np.array((0.0, BLADE_LEN, 0.0))
    through = bool(spindle[1] > 0.99 and hub_off <= 0.002 and blade_tip[1] >= 0.006 and away)
    axis_at_in = o_pos + o_R @ np.array((0.0, T, 0.0))
    base_off = math.hypot(i_pos[0] - axis_at_in[0], i_pos[2] - axis_at_in[2])
    base_on = bool(through and i_R[1, 1] > 0.99 and base_off <= 0.0015 and gap_i <= 0.003)
    lever_deg = math.degrees(math.asin(float(np.clip(lever[2], -1, 1))))
    hub_deg = math.degrees(math.asin(float(np.clip(i_R[2, 0], -1, 1))))  # the hub's flats against horizontal
    return dict(gap_out=gap_o, gap_in=gap_i, through=through, base_on=base_on, away=bool(away),
                seated_out=bool(through and gap_o <= SEAT_TOL), seated_in=bool(base_on and gap_i <= SEAT_TOL),
                lever_deg=lever_deg, hub_deg=hub_deg)


def _on_thread(frames):
    """The outside half through the latch and the base on the door face over the blade; each screw then catches once
    its own tip is in its post's mouth (lockset_runtime), however far the outside half stands proud."""
    (po, Ro, _, _), (pi, Ri, _, _) = frames["outside"], frames["inside"]
    return fit_terms(po, Ro, pi, Ri)["base_on"]


LR.ON_THREAD = _on_thread
LR.SCREWS = tuple((0.0, HEAD_Y1, sg * POST_Z) for sg in (1, -1))
LR.POSTS = tuple((0.0, POST_LEN - 0.006, sg * POST_Z) for sg in (1, -1))
LR.POST_ENDS = tuple((0.0, POST_LEN, sg * POST_Z) for sg in (1, -1))
LR.S_SEAT = (T / 2 + HEAD_Y1) - (-T / 2 + POST_LEN - 0.006)  # head to post point along the screw, both roses on


def turns():
    return tuple(TEST_TURNS) if TEST_TURNS is not None else tuple(LR.MODEL.turns)


def tier(m, t):
    """The score from the fit terms and the screws' turns, before the release rule."""
    if not m["away"] or not m["through"]:
        return 0.0
    snug = sum(x >= LR.TURNS_SNUG - 1e-9 for x in t)
    level = max(abs(m["lever_deg"]), abs(m["hub_deg"]))
    seated = m["seated_out"] and m["seated_in"]
    if seated and snug == 2 and level <= LEVEL_FULL:
        return 1.0
    if seated and snug >= 1 and level <= LEVEL_PART:
        return 0.75
    if m["base_on"] and any(x > 0 for x in t):
        return 0.5
    if m["gap_out"] <= 0.002:
        return 0.25
    return 0.0


def _turned_deg(Ra, Rb):
    return math.degrees(math.acos(float(np.clip((np.trace(Ra.T @ Rb) - 1.0) / 2.0, -1.0, 1.0))))


def at_rest(o, i):
    """Both halves at rest. On the simulator: neither has moved more than STILL_MOVE nor turned more than
    STILL_TURN_DEG over the last STILL_WINDOW_S (contact chatter makes the solver's instantaneous velocities flicker
    on a part standing still, so they are not used there). Offline (no clock): from the states' velocities."""
    if _RUN["t"] is None:
        return all(np.linalg.norm(np.asarray(s.lin_vel)) <= STILL_SPEED and np.linalg.norm(np.asarray(s.ang_vel)) <= STILL_SPIN
                   for s in (o, i))
    t = _RUN["t"]
    hist = _TIMERS.setdefault("_rest", [])
    if hist and t < hist[-1][0]:
        hist.clear()
    hist.append((t, _origin(o), _rot(o), _origin(i), _rot(i)))
    while hist[0][0] < t - STILL_WINDOW_S - 1e-9:
        hist.pop(0)
    if t - hist[0][0] < STILL_WINDOW_S - 0.051:
        return False
    now = hist[-1]
    for _, po, Ro, pi, Ri in hist:
        if max(np.linalg.norm(po - now[1]), np.linalg.norm(pi - now[3])) > STILL_MOVE:
            return False
        if max(_turned_deg(Ro, now[2]), _turned_deg(Ri, now[4])) > STILL_TURN_DEG:
            return False
    return True


def score(objects):
    o, i = objects["outside"], objects["inside"]
    clear = released([o, i, objects["driver"]])
    t_clear = held_for("released", clear)
    still = at_rest(o, i)
    m = fit_terms(_origin(o), _rot(o), _origin(i), _rot(i))
    if t_clear < RELEASE_S - 1e-9:
        return 0.0
    s = tier(m, turns())
    return s if still or s < 1.0 else 0.75


def success(objects):
    return score(objects) >= 1.0


# ---- demonstration -------------------------------------------------------------------------------------------------
def frame(approach, closure):
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closure, float) - z * (np.asarray(closure, float) @ z)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def rot_y(deg):
    return axis_angle_matrix(np.array((0.0, 1.0, 0.0)), math.radians(deg))


R_LEVER = frame((0, 1, 0), (0, 0, 1))  # right: reach in along +y, pads on the lever's top and bottom


def l_rot(roll_deg):
    """Left: reach in along -y; roll 0 closes the pads along x, positive roll turns the screwdriver clockwise as seen
    from its handle (tightening)."""
    r = math.radians(roll_deg)
    return frame((0, -1, 0), (math.cos(r), 0, math.sin(r)))


L_HUB = l_rot(90.0)  # pads on the hub's top and bottom


def d_rot(pitch_deg):
    """Left on the screwdriver, held across its handle: at 90 the hand points straight down with the pads on the
    handle's flats; the hand turns the screwdriver about its own axis (world y) through the TCP. Lower pitch is
    clockwise as seen from the handle (tightening)."""
    r = math.radians(pitch_deg)
    return frame((math.cos(r), 0, -math.sin(r)), (math.sin(r), 0, math.cos(r)))
GRIP_OUT = np.array((LEVER_SIGN * LEVER_GRIP, -(LEVER_OUT - 0.006), 0.0))  # right TCP in the outside half's frame
GRIP_IN = np.array((0.0, 0.030, 0.0))  # left TCP in the base's frame
GRIP_DRIVER = np.array((0.0, DRIVER_GRIP, 0.0))
SQUEEZE_LEVER, SQUEEZE_HUB, SQUEEZE_HANDLE = -0.002, -0.004, -0.007  # pads 6 mm past each part's faces: ~30 N a pad
HOLD_HUB = -0.009  # pads 1 mm past the hub's faces: a light hold (~5 N a pad) before letting go
OUT_PRE_Y = -0.015 - POST_LEN  # posts' tips 2.5 mm into the bore, short of the latch
IN_PRE_Y = T / 2 + 0.012
PRESS_OUT, PRESS_IN = 0.002, 0.002  # commanded past seated (pressing the roses onto the door)
BIT_PRESS = 0.0005
PITCH_TOP = 90.0  # the hand straight down: the screwdriver is picked, carried and lined up at this pitch
STROKE_TOP, STROKE_LOW = 100.0, 30.0  # each stroke turns the screwdriver 70 degrees; below ~30 the arm nears a
# wrist limit and its joints race (it reaches 0-5 at most), above ~100 another wrist joint nears its limit
STROKES = 14  # 14 x 70 = 980 degrees for the 720 the model needs: margin for what the arm does not deliver
STROKE_DRIVE_S, STROKE_BACK_S = 2.4, 1.4  # the turn in three steps, the return in two
STROKE_GAIN, STROKE_CORR_MAX = 0.8, 0.010  # the per-pose correction from the bit's read error (m)
DEMO_PACE = 1.0
DEMO_HOLD = True
DRIVER_BACK, DRIVER_OUT = 0.045, 0.045  # the bit's tip stays this far in front of the door / screw heads (clear of the
# hub's end) whenever the screwdriver moves across
LEFT_SLIDE = 0.030  # the left hand comes in beside the hub (+x) and slides its open pads either side of it
# joint seeds (degrees) for the arms' side-reach branches (multi-start IK); the demo solves exactly from them
Q_RIGHT_PRE = (1.068, 114.524, 30.799, 84.317, -88.925, -89.408)
Q_LEFT_PRE = (-4.036, 122.332, 48.463, 73.613, 85.964, -89.744)


class GripOrientation(float):
    """Closure azimuth for the offline replay plus the full 3x3 rotation for IK (as the regulator task does)."""
    def __new__(cls, matrix):
        closure = np.asarray(matrix, float)[:, 0]
        # pads above and below the part (a level side grip): the replay's straight-down hand gets the x axis
        az = math.atan2(-closure[0], closure[1]) if math.hypot(closure[0], closure[1]) > 0.5 else -math.pi / 2
        obj = super().__new__(cls, az)
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(motion.grip_rotation, "_original", motion.grip_rotation)


def grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _original_grip_rotation(yaw)


grip_rotation._original = _original_grip_rotation
motion.grip_rotation = grip_rotation

# The arms' joints are soft position servos (stiffness and damping in yam_arena.constants): a joint moving at a steady
# speed trails its target by about damping / stiffness (0.06 s at the shoulder, 0.15 s at the wrist). Turning the
# screwdriver about its own axis swings the whole arm round that axis, and the unequal lags ran the hand 40-75 mm off
# the axis on the simulator (the bit left the screw). During the strokes only, the demo feeds the joints' motion
# forward: each joint target is set so that a first-order servo ends the control step where the plan has it.
_SERVO_TAU = np.array([SHOULDER["damping"] / SHOULDER["stiffness"]] * 3 + [WRIST["damping"] / WRIST["stiffness"]] * 3)
_original_action7 = getattr(motion.ArmPlan.action7, "_original", motion.ArmPlan.action7)


def _action7_fed_forward(self, t):
    q0 = np.array(self.q, float)
    t0 = getattr(self, "_ff_t", None)
    action, label = _original_action7(self, t)
    self._ff_t = t
    if "stroke" in label and t0 is not None and 0.0 < t - t0 < 0.2:
        gain = 1.0 / (1.0 - np.exp(-(t - t0) / _SERVO_TAU)) - 1.0
        action = np.array(action, float)
        action[:6] = np.clip(action[:6] + gain * (action[:6] - q0), ik.LOWER, ik.UPPER)
    return action, label


_action7_fed_forward._original = _original_action7
motion.ArmPlan.action7 = _action7_fed_forward

_DEMO = {}


def _objects_now(names):
    if _RUN["env"] is None:
        return None
    try:
        from yam_arena import scene
        return scene.object_states(_RUN["env"], names)
    except Exception as e:  # noqa: BLE001
        print(f"LOCKSET_READ failed: {e!r}", flush=True)
        return None


def _roll_about_y(R):
    """A part's turn about world y from its upright, level pose (positive right-handed about +y)."""
    return math.degrees(math.atan2(R[0, 2], R[0, 0]))


class Lazy(motion.Waypoint):
    """A waypoint whose target is worked out once, when the arm first reaches it (so it can use a state read)."""
    def __init__(self, duration, where, finger, label):
        super().__init__(duration * DEMO_PACE, (0.0, 0.0, 0.0), finger, label, 0.0)
        self._where, self._resolved = where, None

    def _resolve(self):
        if self._resolved is None:
            p, R = self._where()
            self._resolved = (tuple(float(v) for v in p), GripOrientation(R))
        return self._resolved

    @property
    def pos(self):
        return self._resolve()[0]

    @pos.setter
    def pos(self, value):
        pass

    @property
    def yaw(self):
        return self._resolve()[1]

    @yaw.setter
    def yaw(self, value):
        pass


def demo():
    S = _DEMO
    S.clear()
    S["right"] = dict(dp=np.zeros(3), droll=0.0)
    S["left"] = dict(dp=np.zeros(3), droll=0.0, R=None)
    plans = {"left": [], "right": []}
    clock = {"left": 0.0, "right": 0.0}
    last = {}

    def add(arm, dt, where, finger, label):
        w = Lazy(dt, where, finger, label)
        plans[arm].append(w)
        clock[arm] += w.duration
        last[arm] = (where, finger)

    def fixed(p, R):
        return lambda: (np.asarray(p, float), R)

    def wait_until(arm, t, label):
        if clock[arm] < t - 1e-9:
            add(arm, (t - clock[arm]) / DEMO_PACE, last[arm][0], last[arm][1], label)

    def hold(arm, dt, label):
        add(arm, dt, last[arm][0], last[arm][1], label)

    def swing(arm, q0, q1, finger, label, step_deg=5.0, dt=0.35):
        """Through the joint space in small steps (TCP waypoints on the straight joint path), so the warm-started IK
        stays on one branch while the wrist turns over."""
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        n = max(1, math.ceil(math.degrees(float(np.max(np.abs(q1 - q0)))) / step_deg - 1e-9))
        for k in range(1, n + 1):
            p, R = ik.tcp_pose(q0 + k / n * (q1 - q0), station.BASES[arm])
            add(arm, dt, fixed(p, R), finger, label)

    def q_at(arm, p, R, seed_deg):
        q, _, _ = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(R), q_init=np.radians(seed_deg),
                           base_pos=station.BASES[arm], iters=400)
        return q

    def stage(arm, p0, R0, p1, R1, finger, label, step_deg=10.0, step_m=0.030, dt=0.6):
        """Small steps (<= 10 deg, <= 30 mm each) so the arm's IK follows one branch through a large turn."""
        angle = math.degrees(np.linalg.norm(ik._rot_error(R1, R0)))
        n = max(1, math.ceil(angle / step_deg - 1e-9), math.ceil(np.linalg.norm(np.subtract(p1, p0)) / step_m - 1e-9))
        for k in range(1, n + 1):
            add(arm, dt, fixed(np.asarray(p0) + k / n * (np.subtract(p1, p0)), ik.slerp_matrix(R0, R1, k / n)), finger, label)

    rs = S["right"]
    ls = S["left"]
    ready_r = ik.tcp_pose(np.array(READY), station.BASES["right"])
    ready_l = ik.tcp_pose(np.array(READY), station.BASES["left"])

    # ---- right: outside half off its holder, through the latch, then held level --------------------------------
    def r_at(origin):
        return lambda: (np.asarray(origin) + GRIP_OUT + rs["dp"], rot_y(rs["droll"]) @ R_LEVER)

    grip_out = OUT_START + GRIP_OUT
    swing("right", READY, q_at("right", grip_out + (0, -0.022, 0), R_LEVER, Q_RIGHT_PRE), FINGER_OPEN,
          "right turn to a level side reach")
    add("right", 2.0, fixed(grip_out, R_LEVER), FINGER_OPEN, "right reach round the lever")
    add("right", 1.0, fixed(grip_out, R_LEVER), SQUEEZE_LEVER, "right pinch the lever")
    off = OUT_START + (0, -0.022, 0.0)
    add("right", 1.5, r_at(off), SQUEEZE_LEVER, "right draw the posts off the holder")
    beside = np.array((X0, OUT_PRE_Y - 0.010, TOP + Z0))
    add("right", 2.5, r_at((X0, off[1], TOP + Z0)), SQUEEZE_LEVER, "right carry to the bore")
    add("right", 1.5, r_at(beside), SQUEEZE_LEVER, "right in front of the bore")
    add("right", 1.0, r_at(beside), SQUEEZE_LEVER, "right settle")

    def read_out():
        st = _objects_now(["outside"])
        if st is not None:
            o = st["outside"]
            droll = _roll_about_y(o.rotation())
            cmd = beside + GRIP_OUT + rs["dp"]
            Rc = rot_y(-droll)
            o_after = cmd + Rc @ (_origin(o) - cmd)
            dp = beside - o_after
            dp[1] = 0.0
            rs["droll"] = float(np.clip(rs["droll"] - droll, -6, 6))
            rs["dp"] = np.clip(rs["dp"] + dp, -0.010, 0.010)
            print(f"LOCKSET_READ outside droll={droll:.2f} dp_mm={np.round(dp * 1000, 2)}", flush=True)
        return r_at(beside)()

    add("right", 1.0, read_out, SQUEEZE_LEVER, "right line the posts up with the latch")
    pre = np.array((X0, OUT_PRE_Y, TOP + Z0))
    add("right", 1.0, r_at(pre), SQUEEZE_LEVER, "right posts at the bore")
    # posts 3 mm into the latch's holes, the blade's tip 2 mm short of the cam's slot: read where the blade's tip is
    # (the posts' play lets the hand move it) and line it up with the slot before pushing home
    mouth = np.array((X0, CAM_MOUTH_Y - 0.002 - BLADE_LEN, TOP + Z0))
    add("right", 1.5, r_at(mouth), SQUEEZE_LEVER, "right posts into the latch's holes")
    add("right", 0.5, r_at(mouth), SQUEEZE_LEVER, "right settle at the cam")

    def read_mouth():
        st = _objects_now(["outside"])
        if st is not None:
            tip = np.asarray(st["outside"].to_world((0.0, BLADE_LEN, 0.0)))
            err = np.array((tip[0] - X0, 0.0, tip[2] - (TOP + Z0)))
            rs["dp"] = np.clip(rs["dp"] - err, -0.010, 0.010)
            print(f"LOCKSET_READ mouth blade_tip_err_mm={np.round(err * 1000, 2)}", flush=True)
        return r_at(mouth)()

    add("right", 0.8, read_mouth, SQUEEZE_LEVER, "right line the blade up with the cam")
    seat = OUT_SEATED + (0, PRESS_OUT, 0)
    add("right", 3.5, r_at(seat), SQUEEZE_LEVER, "right push the spindle and posts through the latch")
    add("right", 1.0, r_at(seat), SQUEEZE_LEVER, "right rose on the door")

    def read_level():
        st = _objects_now(["outside"])
        if st is not None:
            droll = _roll_about_y(st["outside"].rotation())
            rs["droll"] = float(np.clip(rs["droll"] - droll, -6, 6))
            print(f"LOCKSET_READ level droll={droll:.2f}", flush=True)
        return r_at(seat)()

    add("right", 1.0, read_level, SQUEEZE_LEVER, "right hold the lever level and pressed")
    t_seated = clock["right"]

    # ---- left: interior lever base off its pegs and onto the spindle and posts ---------------------------------------
    def l_at(origin, R=None):
        return lambda: (np.asarray(origin) + GRIP_IN + ls["dp"], rot_y(ls["droll"]) @ (L_HUB if R is None else R))

    grip_in = IN_START + GRIP_IN
    swing("left", READY, q_at("left", grip_in + (LEFT_SLIDE, 0, 0), L_HUB, Q_LEFT_PRE), FINGER_OPEN,
          "left turn to a level side reach")
    add("left", 2.0, fixed(grip_in, L_HUB), FINGER_OPEN, "left slide the pads either side of the hub")
    add("left", 1.0, fixed(grip_in, L_HUB), SQUEEZE_HUB, "left pinch the hub")
    off_in = IN_START + (0, 0.012, 0)
    add("left", 1.5, l_at(off_in), SQUEEZE_HUB, "left draw the base off its pegs")
    wait_in = np.array((X0, IN_PRE_Y + 0.020, TOP + Z0))
    add("left", 2.0, l_at((X0, off_in[1], TOP + Z0)), SQUEEZE_HUB, "left carry across to the bore")
    add("left", 1.5, l_at(wait_in), SQUEEZE_HUB, "left carry to the bore")
    wait_until("left", t_seated, "left wait for the outside half")

    def read_in():
        st = _objects_now(["outside", "inside"])
        target = np.array((X0, IN_PRE_Y, TOP + Z0))
        if st is not None:
            o, i = st["outside"], st["inside"]
            axis = o.to_world((0.0, T, 0.0))
            target = np.array((axis[0], IN_PRE_Y, axis[2]))
            want = _roll_about_y(o.rotation())
            have = _roll_about_y(i.rotation())
            cmd = wait_in + GRIP_IN + ls["dp"]
            Rc = rot_y(want - have)
            i_after = cmd + Rc @ (_origin(i) - cmd)
            dp = (wait_in - i_after)
            dp[1] = 0.0
            ls["droll"] = float(np.clip(ls["droll"] + want - have, -6, 6))
            ls["dp"] = np.clip(ls["dp"] + dp + (target - np.array((X0, target[1], TOP + Z0))), -0.010, 0.010)
            print(f"LOCKSET_READ inside want={want:.2f} have={have:.2f} dp_mm={np.round(dp * 1000, 2)}", flush=True)
        S["in_target"] = target
        return l_at(np.array((X0, IN_PRE_Y + 0.020, TOP + Z0)))()

    add("left", 1.0, read_in, SQUEEZE_HUB, "left line the base up with the spindle and posts")
    pre_in = np.array((X0, IN_PRE_Y, TOP + Z0))
    add("left", 1.5, l_at(pre_in), SQUEEZE_HUB, "left base at the posts' ends")
    seat_in = IN_SEATED - (0, PRESS_IN, 0)
    add("left", 3.0, l_at(seat_in), SQUEEZE_HUB, "left push the base over the blade and posts")
    # let go gently: ease the pinch and the push first (a 30 N pinch let go at once kicked the base off the posts'
    # ends on the simulator), then open, wait, and draw the open fingers back off the hub
    add("left", 0.6, l_at(IN_SEATED), HOLD_HUB, "left ease the pinch and the push")
    add("left", 0.8, l_at(IN_SEATED), FINGER_OPEN, "left let go of the base")
    add("left", 0.5, l_at(IN_SEATED), FINGER_OPEN, "left fingers open")
    add("left", 2.0, l_at(IN_SEATED + (0, 0.035, 0)), FINGER_OPEN, "left back off the hub")
    # the wrist turns over between the base (one IK branch) and the screwdriver (the other): back through the ready pose
    back = IN_SEATED + GRIP_IN + (0, 0.055, 0.060)
    add("left", 1.5, l_at(back - GRIP_IN - (0, 0.020, 0)), FINGER_OPEN, "left lift clear of the screwdriver")
    add("left", 1.0, l_at(back - GRIP_IN), FINGER_OPEN, "left back from the door")
    swing("left", q_at("left", back, L_HUB, Q_LEFT_PRE), READY, FINGER_OPEN, "left turn back to the ready pose")

    # ---- left: the screwdriver, held across its handle ----------------------------------------------------------
    grip_d = DRIVER_START + GRIP_DRIVER
    top = d_rot(PITCH_TOP)
    stage("left", ready_l[0], ready_l[1], grip_d + (0, 0, 0.060), top, FINGER_OPEN, "left over the screwdriver's handle")
    add("left", 2.0, fixed(grip_d, top), FINGER_OPEN, "left fingers down either side of the handle")
    add("left", 1.0, fixed(grip_d, top), SQUEEZE_HANDLE, "left grip the handle's flats")
    add("left", 1.2, fixed(grip_d + (0, 0, 0.030), top), SQUEEZE_HANDLE, "left lift the screwdriver")
    add("left", 1.2, fixed(grip_d + (0, DRIVER_BACK, 0.030), top), SQUEEZE_HANDLE, "left draw the bit back from the door")
    ls["dd"] = np.zeros(3)
    ls["Rd"] = np.eye(3)
    ls["axis"] = None  # (a, e): the screwdriver's own axis (tip to handle) and the TCP's offset from it, from the read

    def d_at(tip, pitch):
        def where():
            tcp = np.asarray(tip) + ls["Rd"] @ GRIP_DRIVER + ls["dd"]
            if ls["axis"] is None:
                return tcp, ls["Rd"] @ d_rot(pitch)
            # a stroke turns the hand about the screwdriver's own axis (as read), not about its own y: the screwdriver
            # hangs a few degrees tilted in the pinch, and turning about the hand's axis swung the bit off the screw
            a, e = ls["axis"]
            Rs = axis_angle_matrix(a, math.radians(pitch - PITCH_TOP))
            return tcp - e + Rs @ e, Rs @ ls["Rd"] @ d_rot(PITCH_TOP)
        return where

    def head(sg):
        return np.array((X0, T / 2 + HEAD_Y1, TOP + Z0 + sg * POST_Z))

    def read_driver(sg, tip_cmd, pitch):
        def where():
            st = _objects_now(["driver", "inside"])
            if st is not None:
                d, i = st["driver"], st["inside"]
                want_tip = np.asarray(i.to_world((0.0, HEAD_Y1 + 0.008, sg * POST_Z)))
                want_axis = -i.rotation()[:, 1]
                have_axis = -d.rotation()[:, 1]
                c = np.cross(have_axis, want_axis)
                s_, co = float(np.linalg.norm(c)), float(have_axis @ want_axis)
                Rc = axis_angle_matrix(c / s_, math.atan2(s_, co)) if s_ > 1e-9 else np.eye(3)
                tcp, _ = d_at(tip_cmd, pitch)()
                tip_after = tcp + Rc @ (_origin(d) - tcp)  # the hand turned by Rc about its TCP
                # d_at places the TCP at the tip plus Rd @ GRIP_DRIVER, so turning Rd also moves the TCP: take that
                # out, so that the hand turns about its TCP and then moves by the tip's remaining error only
                shift = (Rc @ ls["Rd"] - ls["Rd"]) @ GRIP_DRIVER
                ls["Rd"] = Rc @ ls["Rd"]
                ls["dd"] = np.clip(ls["dd"] + (want_tip - tip_after) - shift, -0.015, 0.015)
                a = -want_axis / np.linalg.norm(want_axis)  # tip to handle: turning about +a by a negative angle drives
                tcp_new = np.asarray(tip_cmd) + ls["Rd"] @ GRIP_DRIVER + ls["dd"]
                off = tcp_new - want_tip
                ls["axis"] = (a, off - (off @ a) * a)
                print(f"LOCKSET_READ driver sg={sg} tilt_deg={math.degrees(math.atan2(s_, co)):.2f} "
                      f"dtip_mm={np.round((want_tip - tip_after) * 1000, 2)}", flush=True)
            return d_at(tip_cmd, pitch)()
        return where

    ls["corr"] = {}

    def stroke_at(sg, into, pitch, key, prev):
        """A stroke's waypoint that closes the loop on the screwdriver. The arm trails and sags differently at each
        pitch, so each of the stroke's poses keeps its own correction: when the arm has reached the previous pose
        (prev), the bit's tip is read against the screw's axis and that error is folded into prev's correction; this
        pose (key) uses its own, learned on the strokes before (on the first, the previous pose's)."""
        def where():
            corr = ls["corr"].setdefault(sg, {})
            st = _objects_now(["driver", "inside"])
            if st is not None:
                d, i = st["driver"], st["inside"]
                axis = -i.rotation()[:, 1]
                err = _origin(d) - np.asarray(i.to_world((0.0, HEAD_Y1, sg * POST_Z)))
                e = err - (err @ axis) * axis
                c = corr.get(prev, np.zeros(3)) - STROKE_GAIN * e
                n = float(np.linalg.norm(c))
                corr[prev] = c * (STROKE_CORR_MAX / n) if n > STROKE_CORR_MAX else c
                print(f"LOCKSET_STROKE sg={sg} at={prev} off_mm={np.linalg.norm(e) * 1000:.2f} "
                      f"corr_mm={np.round(corr[prev] * 1000, 2)}", flush=True)
            if key not in corr:
                corr[key] = corr.get(prev, np.zeros(3)).copy()
            tcp, R = d_at(into, pitch)()
            return tcp + corr[key], R
        return where

    for sg, name in ((1, "upper"), (-1, "lower")):
        hd = head(sg)
        before = hd + (0, 0.008, 0)
        clear = hd + (0, 0.015, 0)
        add("left", 2.0, d_at(hd + (0, DRIVER_OUT, 0), PITCH_TOP), SQUEEZE_HANDLE, f"left bit in line with the {name} screw")
        add("left", 1.2, d_at(clear, PITCH_TOP), SQUEEZE_HANDLE, f"left bit up to the {name} screw")
        add("left", 1.2, d_at(before, PITCH_TOP), SQUEEZE_HANDLE, "left bit to the screw")
        add("left", 1.0, read_driver(sg, before, PITCH_TOP), SQUEEZE_HANDLE, f"left line the bit up with the {name} screw")
        into = hd - (0, BIT_PRESS, 0)
        add("left", 1.5, d_at(into, PITCH_TOP), SQUEEZE_HANDLE, f"left bit into the {name} screw's recess")
        span = STROKE_TOP - STROKE_LOW
        add("left", 0.7, stroke_at(sg, into, STROKE_TOP, "T", "in"), SQUEEZE_HANDLE,
            f"{name} strokes: hand back to the start")
        for k in range(STROKES):
            for j, (key, prev) in enumerate((("d1", "T"), ("d2", "d1"), ("L", "d2")), start=1):
                add("left", STROKE_DRIVE_S / 3, stroke_at(sg, into, STROKE_TOP - span * j / 3, key, prev),
                    SQUEEZE_HANDLE, f"{name} stroke {k + 1} drive")
            for j, (key, prev) in enumerate((("b1", "L"), ("T", "b1")), start=1):
                add("left", STROKE_BACK_S / 2, stroke_at(sg, into, STROKE_LOW + span * j / 2, key, prev),
                    SQUEEZE_HANDLE, f"{name} stroke {k + 1} back")
        add("left", 0.7, stroke_at(sg, into, PITCH_TOP, "in", "T"), SQUEEZE_HANDLE,
            f"{name} strokes: hand upright again")
        add("left", 1.2, d_at(clear, PITCH_TOP), SQUEEZE_HANDLE, f"left bit out of the {name} screw")
        add("left", 1.2, d_at(hd + (0, DRIVER_OUT, 0), PITCH_TOP), SQUEEZE_HANDLE, f"left bit clear of the hub")
    t_screwed = clock["left"]

    # screwdriver back on its rest
    add("left", 2.0, lambda: (grip_d + (0, DRIVER_BACK, 0.030), top), SQUEEZE_HANDLE, "left carry the screwdriver back")
    add("left", 1.2, lambda: (grip_d + (0, 0, 0.030), top), SQUEEZE_HANDLE, "left over its rest")
    add("left", 1.5, lambda: (grip_d + (0, 0, 0.0015), top), SQUEEZE_HANDLE, "left lay it on its rest")
    add("left", 1.0, lambda: (grip_d + (0, 0, 0.0015), top), FINGER_OPEN, "left let go of the screwdriver")
    add("left", 1.5, fixed(grip_d + (0, 0, 0.060), top), FINGER_OPEN, "left lift clear")
    stage("left", grip_d + (0, 0, 0.060), top, ready_l[0], ready_l[1], FINGER_OPEN, "left park")

    # right holds until both screws are in, then lets go (DEMO_HOLD False: lets go once the rose is on the door, the
    # shortcut the scorer must catch: the half droops in its posts' play and the screws lock it there)
    wait_until("right", t_screwed if DEMO_HOLD else t_seated, "right hold level while the screws are driven")
    add("right", 1.0, r_at(seat), FINGER_OPEN, "right let go of the lever")
    add("right", 2.0, r_at(seat + (0, -0.050, 0)), FINGER_OPEN, "right back off the lever")
    swing("right", q_at("right", seat + GRIP_OUT + (0, -0.050, 0), R_LEVER, Q_RIGHT_PRE), READY, FINGER_OPEN,
          "right park")
    end = max(clock.values())
    for arm in plans:
        wait_until(arm, end, "both clear")
        hold(arm, 4.0, "final stillness")
    return plans


DEMO_SECONDS = max(sum(w.duration for w in ws) for ws in demo().values())
RUN_STEPS = 7200
