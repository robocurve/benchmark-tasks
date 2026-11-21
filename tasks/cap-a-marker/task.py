"""Cap a marker. An EXPO Low Odor chisel-tip dry-erase marker lies on the table with its cap posted on its back end,
as it is left while writing. Task: take the cap off the back end, turn it round, put it on the tip end until it clicks,
and leave the capped marker lying on the table.

The click and the posting fit are real contact, not rules: a 0.45 mm ridge runs round the barrel's nose, and inside the
cap's mouth three small rounded pads, each on a preloaded spring (a sprung slider joint), stand 0.35 mm inside that
ridge. A cap pushed straight on stops on the ridge with its mouth about 3 mm from the barrel's shoulder; only a push of
about 5 N along the marker drives the pads out over it, and the cap then sits home, its mouth against the shoulder,
held there by the pads behind the ridge. Posted, the same pads grip the barrel's narrower back end (pull-off about
1.2 N). The bore clears the ridge by 0.2 mm and the nose by 0.65 mm, with a 0.6 mm lead-in at the mouth.

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
from markerkit import Body, BoxShape, CylShape, as_numpy, on_reset, on_step, quat_xyzw, rotation, slider  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Take the cap off the back of the marker, put it on the tip until it clicks, and leave the capped marker lying on the table."

# -- the marker (barrel) ------------------------------------------------------------------------------------------------
# The marker is an EXPO Low Odor chisel-tip dry-erase marker (item 80001), listed by its maker at 4.81 in (122 mm) long
# capped, 0.75 in (19 mm) across at its widest (the cap with its pocket clip) and 0.02 lb (9 g). Everything finer
# (barrel 15 mm, the barrel/cap split, nose, ridge, nib, bore, clip) is an estimate of the same marker, to be checked
# with calipers on the marker used. Barrel frame: origin on the axis at the shoulder (where the cap's mouth seats), z
# along the axis toward the tip.
BODY_R = 0.0075            # the barrel, 15 mm across (estimate)
BODY_L = 0.077             # from the back end to the shoulder (77 mm + the 45 mm cap = 122 mm capped)
POST_R, POST_L = 0.00615, 0.010   # the back end's narrower 10 mm, which the cap is posted on
NOSE_R, NOSE_L = 0.00585, 0.010   # the nose the cap fits over
RIDGE_R = 0.0063           # the snap ridge round the nose: 0.45 mm proud of it ...
RIDGE_Z = (0.0030, 0.0042)  # ... from 3.0 to 4.2 mm in front of the shoulder
CONE_R, CONE_L = 0.0040, 0.008    # the nib holder, in front of the nose
NIB = (0.005, 0.002, 0.012)       # the chisel nib: 5 x 2 mm, 12 mm long
TIP_Z = NOSE_L + CONE_L + NIB[2]  # 30 mm: the nib's end, in front of the shoulder
BARREL_MASS = 0.0068
BARREL_FLATS = 16

# -- the cap --------------------------------------------------------------------------------------------------------------
# Cap frame: origin on the axis at the middle of its mouth, z along the axis into the cap (toward its closed end).
CAP_L = 0.045
CAP_R = 0.00825            # 16.5 mm across
BORE_R = 0.0065            # the bore: 0.2 mm clear of the ridge, 0.65 mm clear of the nose, 0.35 mm of the post
MOUTH_CHAMFER = 0.0006     # the mouth's inner edge is chamfered 0.6 mm at 45 degrees (a lead-in)
END_T = 0.003              # the closed end
STAVES = 12                # the tube, as 12 flat staves round the bore
# The pocket clip: a bar 4 mm wide standing 2.5 mm proud of the cap (so the cap is 19 mm across over it), from 12 to
# 44 mm in from the mouth, on the cap's -y side (cap frame). Modelled solid (no gap under it).
CLIP_W, CLIP_T, CLIP_Z = 0.004, 0.0025, (0.012, 0.044)
CAP_MASS = 0.0019          # the shell and clip; with its three pads, 2.2 g (9.0 g with the barrel)
# The snap ring: three rounded pads (1 mm radius, 3 mm long, across the bore) at 90, 210 and 330 degrees round it, their
# centres 1.9 mm into the mouth. At rest each pad's inner edge is at PAD_IN, 0.35 mm inside the ridge; it can be pushed
# out 0.5 mm. A spring holds it in: preloaded to PAD_PRELOAD and rising at PAD_K. These two numbers are estimates sized
# so that pushing the cap over the ridge takes about 5 N (README.md: "The click force"), TO BE REPLACED by the force
# measured on the real marker. On the back end the same ring grips the 12.3 mm post, 0.2 mm out, so the posted cap
# pulls off at about 1.2 N (also an estimate).
PAD_R, PAD_W = 0.0010, 0.003
PAD_IN = RIDGE_R - 0.00035
PAD_Z = 0.0019
PAD_ANGLES = (90.0, 210.0, 330.0)
PAD_TRAVEL = 0.0005
PAD_PRELOAD = 0.8          # N, estimate
PAD_K = 2000.0             # N/m, estimate
PAD_MASS = 0.0001
POST_DEFLECT = POST_R - PAD_IN     # the pads' travel while the cap is posted

# -- layout ---------------------------------------------------------------------------------------------------------------
# The marker lies across the table (along y) with its cap posted on its back end, as it is left after writing: the tip
# toward the right arm, the posted cap toward the left, its clip up. Posted, the cap's mouth sits against the step at
# the start of the post and the cap points back along the barrel.
BARREL_XYZW = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))    # body z along world -y (turned 90 degrees about x)
CAP_XYZW = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))      # body z along world +y; body -y (the clip) up
SHOULDER_Y = -0.07         # the shoulder; the nib's end at y -0.10, the back end at y 0.007, the cap's end at y 0.053
BARREL_START = (0.30, SHOULDER_Y, TOP + CAP_R + 0.0002)      # lying on the posted cap and the barrel's flats
CAP_START = (0.30, SHOULDER_Y + BODY_L - POST_L, TOP + CAP_R + 0.0002)   # the mouth, against the post's step

BARREL_COLOR = (0.15, 0.15, 0.17)
BAND = (0.10, 0.30, 0.80)
CAP_COLOR = (0.12, 0.35, 0.85)
NIB_COLOR = (0.20, 0.35, 0.80)
NOSE_COLOR = (0.70, 0.70, 0.72)


def _barrel() -> Body:
    shapes = [CylShape((0.0, 0.0, -BODY_L + POST_L / 2), POST_R, POST_L, BARREL_COLOR),
              CylShape((0.0, 0.0, -(BODY_L - POST_L) / 2), BODY_R, BODY_L - POST_L, BARREL_COLOR, collide=False),
              CylShape((0.0, 0.0, -0.030), BODY_R + 0.00005, 0.004, BAND, collide=False),
              CylShape((0.0, 0.0, (NOSE_L - 0.0015) / 2), NOSE_R, NOSE_L - 0.0015, NOSE_COLOR),
              CylShape((0.0, 0.0, NOSE_L - 0.00125), NOSE_R - 0.00025, 0.001, NOSE_COLOR),   # the nose's front edge,
              CylShape((0.0, 0.0, NOSE_L - 0.00035), NOSE_R - 0.0006, 0.0008, NOSE_COLOR),   # chamfered in two steps
              CylShape((0.0, 0.0, sum(RIDGE_Z) / 2), RIDGE_R, RIDGE_Z[1] - RIDGE_Z[0], NOSE_COLOR),
              CylShape((0.0, 0.0, NOSE_L + CONE_L / 2), CONE_R, CONE_L, NOSE_COLOR),
              BoxShape((0.0, 0.0, NOSE_L + CONE_L + NIB[2] / 2), NIB, NIB_COLOR)]
    # The barrel collides as a 16-sided tube (16.5 mm across its flats): a perfectly round simulated barrel creeps
    # (rolls) on the level table where a real one lies still; the flats keep it lying where it is put.
    wall, l = 0.006, BODY_L - POST_L
    for k in range(BARREL_FLATS):
        a = 2 * math.pi * k / BARREL_FLATS
        r = BODY_R - wall / 2
        shapes.append(BoxShape((r * math.cos(a), r * math.sin(a), -l / 2),
                               (wall, 2 * BODY_R * math.tan(math.pi / BARREL_FLATS), l), None, yaw=a))
    return Body.build("marker", pos=BARREL_START, mass=BARREL_MASS, shapes=shapes, color=BARREL_COLOR,
                      friction=(0.35, 0.3), contact_offset=0.0005, solver_iterations=32, damping=(0.05, 0.2),
                      start_xyzw=BARREL_XYZW, ccd=True)


def _cap() -> Body:
    t = CAP_R - BORE_R
    w = 2 * CAP_R * math.tan(math.pi / STAVES)
    shapes = []
    c = MOUTH_CHAMFER
    tm = CAP_R - (BORE_R + c)
    n = (math.sqrt(0.5), math.sqrt(0.5))                 # the chamfer face's normal into the wall, (radial, axial)
    for k in range(STAVES):
        a = 2 * math.pi * k / STAVES
        ca, sa = math.cos(a), math.sin(a)
        r = BORE_R + t / 2
        zc = (c + CAP_L - END_T) / 2
        shapes.append(BoxShape((r * ca, r * sa, zc), (t, w, CAP_L - END_T - c), CAP_COLOR, yaw=a))
        # the mouth: c deep, its bore c wider, and a 45-degree chamfer from it down to the bore
        rm = BORE_R + c + tm / 2
        shapes.append(BoxShape((rm * ca, rm * sa, c / 2), (tm, w, c), CAP_COLOR, yaw=a))
        th = 0.001
        rc, zc2 = BORE_R + c / 2 + n[0] * th / 2, c / 2 + n[1] * th / 2
        shapes.append(BoxShape((rc * ca, rc * sa, zc2), (th, 2 * (BORE_R + c) * math.tan(math.pi / STAVES),
                                                         c * math.sqrt(2)), CAP_COLOR, yaw=a, pitch=-math.pi / 4))
    shapes.append(CylShape((0.0, 0.0, CAP_L - END_T / 2), CAP_R, END_T, CAP_COLOR))
    shapes.append(BoxShape((0.0, -(CAP_R + CLIP_T / 2 - 0.0002), sum(CLIP_Z) / 2),
                           (CLIP_W, CLIP_T + 0.0004, CLIP_Z[1] - CLIP_Z[0]), CAP_COLOR))
    return Body.build("cap", pos=CAP_START, mass=CAP_MASS, shapes=shapes, color=CAP_COLOR, friction=(0.35, 0.3),
                      contact_offset=0.0005, solver_iterations=32, damping=(0.05, 0.2), start_xyzw=CAP_XYZW, ccd=True)


def _pad_rest(phi_deg: float) -> np.ndarray:
    """A pad's centre at rest, in the cap frame."""
    a = math.radians(phi_deg)
    return np.array([(PAD_IN + PAD_R) * math.cos(a), (PAD_IN + PAD_R) * math.sin(a), PAD_Z])


def _pad(i: int) -> Body:
    phi = PAD_ANGLES[i]
    r_cap = rotation(CAP_XYZW)
    a = math.radians(phi)
    out = np.array([math.cos(a), math.sin(a), 0.0]) * max(0.0, POST_DEFLECT)   # pushed out by the post
    pos = np.array(CAP_START) + r_cap @ (_pad_rest(phi) + out)
    rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    return Body.build(f"cap_pad_{i}", pos=tuple(pos), mass=PAD_MASS, color=CAP_COLOR, friction=(0.35, 0.3),
                      contact_offset=0.0005, solver_iterations=32, start_xyzw=quat_xyzw(r_cap @ rz), ccd=True,
                      shapes=[CylShape((0.0, 0.0, 0.0), PAD_R, PAD_W, None, axis="Y")])


PADS = [slider(f"snap_{i}", "cap", f"cap_pad_{i}", anchor0=tuple(_pad_rest(phi)), yaw0=math.radians(phi),
               travel=(0.0, PAD_TRAVEL), stiffness=PAD_K, target=-PAD_PRELOAD / PAD_K)
        for i, phi in enumerate(PAD_ANGLES)]

OBJECTS = [_barrel(), _cap(), *(_pad(i) for i in range(len(PAD_ANGLES)))]

# -- scoring numbers -------------------------------------------------------------------------------------------------
ON_ANGLE = 10.0            # the cap counts as on the tip end only within 10 degrees of in line with the barrel ...
ON_OFFSET = 0.002          # ... with its mouth's centre within 2 mm of the barrel's axis ...
OFF_CLEAR = 0.005          # 0.2: the cap is off the back end: the back end at least 5 mm clear of the cap's mouth
GAP_STARTED = 0.012        # 0.4: its mouth within 12 mm of the shoulder (the nib is inside, the bore over the nose)
GAP_RIDGE = 0.004          # 0.6: within 4 mm (pushed onto the snap ridge, where an unclicked cap stops, ~3 mm)
GAP_SEATED = 0.0005        # 0.8 / 1.0: within 0.5 mm (clicked home; only reachable past the ridge)
LEVELS = {"off": 0.2, "started": 0.4, "ridge": 0.6, "seated": 0.8}
LIE_GAP = 0.003            # lying on the table: both ends' lowest points within 3 mm of the table top
LIE_ANGLE = 10.0           # ... and the marker within 10 degrees of level
CLEAR = 0.02               # let go: every part of both grippers at least 2 cm from the marker and the cap
STILL = 0.001              # still: neither moved more than 1 mm over the last 1 s
HOLD_STEPS = 20            # 1 s at 20 Hz


# -- geometry (pure; used both in simulation and by the offline tests) ------------------------------------------------

def cap_fit(barrel_pos, barrel_r, cap_pos, cap_r) -> dict:
    """How the cap sits on the barrel's tip end. ``gap``: the cap's mouth from the shoulder along the barrel's axis
    (m; positive toward the tip); ``offset``: the mouth's centre from the barrel's axis (m); ``angle``: between the
    two axes (degrees; 0 when the cap points the way the tip does)."""
    bp, br = np.asarray(barrel_pos, float), np.asarray(barrel_r, float)
    cp, cr = np.asarray(cap_pos, float), np.asarray(cap_r, float)
    a = br[:, 2]
    d = cp - bp
    gap = float(d @ a)
    offset = float(np.linalg.norm(d - gap * a))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(a @ cr[:, 2])))))
    on = bool(angle <= ON_ANGLE and offset <= ON_OFFSET and gap >= -0.001)
    return {"gap": gap, "offset": offset, "angle": angle, "on": on}


def fit_level(fit: dict) -> float:
    """The credit the cap's fit on the tip end alone earns: 0.8 seated, 0.6 on the ridge, 0.4 started over the tip,
    else 0."""
    if not fit["on"]:
        return 0.0
    if fit["gap"] <= GAP_SEATED:
        return LEVELS["seated"]
    if fit["gap"] <= GAP_RIDGE:
        return LEVELS["ridge"]
    if fit["gap"] <= GAP_STARTED:
        return LEVELS["started"]
    return 0.0


def off_back_end(barrel_pos, barrel_r, cap_pos, cap_r) -> bool:
    """The cap is off the back end: the centre of the barrel's back end face is not inside the cap or within
    OFF_CLEAR of its mouth (it is at least 5 mm outside the mouth's plane, or beyond the cap's side)."""
    bp, br = np.asarray(barrel_pos, float), np.asarray(barrel_r, float)
    cp, cr = np.asarray(cap_pos, float), np.asarray(cap_r, float)
    back = bp - BODY_L * br[:, 2]
    d = back - cp
    depth = float(d @ cr[:, 2])                        # how far the back end is in past the cap's mouth
    side = float(np.linalg.norm(d - depth * cr[:, 2]))
    return bool(depth < -OFF_CLEAR or side > CAP_R + OFF_CLEAR or depth > CAP_L + OFF_CLEAR)


def _low(point, axis_r_vertical, radius) -> float:
    return float(point[2] - radius * axis_r_vertical)


def lying(barrel_pos, barrel_r, cap_pos, cap_r) -> bool:
    """The capped marker lies on the table: within LIE_ANGLE of level, and the lowest points of its back end and of
    the cap's closed end within LIE_GAP of the table top."""
    bp, br = np.asarray(barrel_pos, float), np.asarray(barrel_r, float)
    cp, cr = np.asarray(cap_pos, float), np.asarray(cap_r, float)
    a = br[:, 2]
    tilt = math.degrees(math.asin(min(1.0, abs(float(a[2])))))
    s = math.sqrt(max(0.0, 1.0 - float(a[2]) ** 2))
    back = _low(bp - BODY_L * a, s, BODY_R)
    c = cr[:, 2]
    sc = math.sqrt(max(0.0, 1.0 - float(c[2]) ** 2))
    end = min(_low(cp + CAP_L * c, sc, CAP_R), clip_low(cp, cr))   # it may rest on its clip
    return bool(tilt <= LIE_ANGLE and abs(back - TOP) <= LIE_GAP and abs(end - TOP) <= LIE_GAP)


def clip_low(cap_pos, cap_r) -> float:
    """The height of the lowest corner of the cap's pocket clip."""
    cr = np.asarray(cap_r, float)
    lo = []
    for sx in (-1, 1):
        for sz in CLIP_Z:
            local = np.array([sx * CLIP_W / 2, -(CAP_R + CLIP_T), sz])
            lo.append(float((np.asarray(cap_pos, float) + cr @ local)[2]))
    return min(lo)


def _segment_distance(points, a, b, radius) -> np.ndarray:
    pts = np.asarray(points, float)
    d = b - a
    t = np.clip((pts - a) @ d / (d @ d), 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * d), axis=1) - radius


def gripper_clearance(points, barrel_pos, barrel_r, cap_pos, cap_r) -> float:
    """The least distance (m) from the grippers' points to the marker (back end to nib) and to the cap."""
    if points is None or len(points) == 0:
        return float("inf")
    bp, a = np.asarray(barrel_pos, float), np.asarray(barrel_r, float)[:, 2]
    cp, c = np.asarray(cap_pos, float), np.asarray(cap_r, float)[:, 2]
    d1 = _segment_distance(points, bp - BODY_L * a, bp + TIP_Z * a, BODY_R)
    d2 = _segment_distance(points, cp, cp + CAP_L * c, CAP_R + CLIP_T)   # (the clip included)
    return float(min(d1.min(), d2.min()))


def marks(barrel_pos, barrel_r, cap_pos, cap_r) -> np.ndarray:
    """Points tracked for 'still': the marker's back end and shoulder, the cap's mouth and closed end."""
    bp, a = np.asarray(barrel_pos, float), np.asarray(barrel_r, float)[:, 2]
    cp, c = np.asarray(cap_pos, float), np.asarray(cap_r, float)[:, 2]
    return np.vstack([bp - BODY_L * a, bp, cp, cp + CAP_L * c])


# -- the score, step by step ------------------------------------------------------------------------------------------

_EP: dict = {}
_DEBUG = bool(os.environ.get("CAP_DEBUG"))


@on_reset
def _reset_state() -> None:
    _EP.clear()
    _EP.update(best=0.0, held=0, recent=[], score=0.0, done=False, steps=0, info={})


_reset_state()


def observe(barrel_pos, barrel_r, cap_pos, cap_r, gripper_points) -> float:
    """One control step: update what the episode has reached and work out the score."""
    fit = cap_fit(barrel_pos, barrel_r, cap_pos, cap_r)
    level = fit_level(fit)
    off = off_back_end(barrel_pos, barrel_r, cap_pos, cap_r)
    if off:
        level = max(level, LEVELS["off"])
    _EP["best"] = max(_EP["best"], level)
    snap = marks(barrel_pos, barrel_r, cap_pos, cap_r)
    recent = _EP["recent"]
    recent.append(snap)
    del recent[:-(HOLD_STEPS + 1)]
    still = len(recent) == HOLD_STEPS + 1 and all(float(np.abs(r - snap).max()) <= STILL for r in recent)
    seated = level >= LEVELS["seated"]
    lies = lying(barrel_pos, barrel_r, cap_pos, cap_r)
    clear = gripper_clearance(gripper_points, barrel_pos, barrel_r, cap_pos, cap_r) >= CLEAR
    done = bool(seated and lies and clear and still)
    _EP["held"] = _EP["held"] + 1 if done else 0
    score = 1.0 if _EP["held"] >= HOLD_STEPS else _EP["best"]
    _EP.update(score=score, done=done, steps=_EP["steps"] + 1,
               info={"fit": fit, "level": level, "off": off, "lying": lies, "clear": clear, "still": still})
    return score


def success(objects) -> bool:
    """True only on steps where score() is 1.0: the cap has sat clicked home on the tip end with the marker lying on
    the table, let go of and still, for 1 s. The runtime then ends the episode after its own 1 s of this."""
    return _EP["score"] >= 1.0


def score(objects) -> float:
    """1.0 once the cap has sat home on the tip end (mouth within 0.5 mm of the shoulder, within 10 degrees and 2 mm
    of in line) with the marker lying on the table (within 10 degrees of level; its back end and the cap's closed end
    or clip within 3 mm of the table), both grippers 2 cm clear and nothing moving more than 1 mm, for 1 s (20 steps in
    a row); otherwise the best reached at any point: 0.8 home on the tip end (within 0.5 mm), 0.6 on the ridge (within
    4 mm), 0.4 started over the tip (within 12 mm), 0.2 pulled off the back end (the back end 5 mm or more clear of the
    cap); else 0. A cap reversed, off-axis or posted on the back end again earns nothing more."""
    return float(_EP["score"])


# -- in the simulator ------------------------------------------------------------------------------------------------

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

    bp, br = pose("marker")
    cp, cr = pose("cap")
    observe(bp, br, cp, cr, _gripper_points(base, origin))
    if _DEBUG and _EP["steps"] % 10 == 0:
        from markerkit import slider_travel

        inf = _EP["info"]
        f = inf["fit"]
        pads = [round(1000 * slider_travel(env, s), 3) for s in PADS]
        print(f"[cap] step {_EP['steps']} score {_EP['score']} best {_EP['best']} gap {1000 * f['gap']:.2f} mm "
              f"offset {1000 * f['offset']:.2f} mm angle {f['angle']:.1f} pads(mm) {pads} lying {inf['lying']} "
              f"clear {inf['clear']} still {inf['still']} held {_EP['held']} marker {np.round(1000 * bp, 1).tolist()} "
              f"cap {np.round(1000 * cp, 1).tolist()}", file=sys.stderr, flush=True)
