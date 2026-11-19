"""A hand mixer standing on its heel rest on the station table, and its two beaters lying flat beside it.

The mixer's underside faces the robot. It has two beater sockets side by side: the larger one (on the mixer's left,
+y) takes only the beater with the collar; the smaller one (right, -y) takes the plain beater. A collar is wider than
the smaller socket's opening. Inside each socket a drive spindle with four slots sits at a random angle; the four fins
on a beater's stem enter only when they line up with the slots. Deeper in, a spring clip (a sprung steel ball) rides
over the stem's tip and drops into the stem's groove when the beater is fully in. The 0.9 kg mixer slides on the
table at about 4.4 N and the clip takes about 6 N to pass, so a push on a beater that is not braced moves the mixer.

The task: fit both beaters, each in its own socket, fully in and clipped, leave the mixer standing on its heel rest
where it was, and let go.

Everything works by contact, joints and springs: the socket openings, spindle slots, fins, collar and stem groove are
collision shapes; each spindle is a hinge held by a stiff spring (its gear train); each clip is a sprung slider.

Frame: x forward from the arm bases, y left, z up, metres. Mixer frame: origin on the table under the footprint
centre, +x from the socket face into the mixer.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
from bodykit import Body, BoxShape, CylShape, SphereShape, segment  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
CONTROL_HZ = 20.0
PROBE = os.environ.get("MIX_PROBE", "")  # "a": collared aligned in large, plain 20 deg off in small; "b": swapped
SUCCESS_HOLD_S = 1000.0 if PROBE else 1.0  # a probe runs its whole schedule

WHITE = (0.92, 0.92, 0.90)
GREY = (0.35, 0.35, 0.37)
STEEL = (0.78, 0.79, 0.80)
DARK = (0.15, 0.15, 0.16)
BRASS = (0.80, 0.62, 0.25)

# -- the mixer (mixer frame) -----------------------------------------------------------------------------------------
FACE_X = -0.040  # the socket face (the mixer's underside)
Z_S = 0.065  # socket axis height above the table
SOCK_Y = {"large": 0.019, "small": -0.019}
HOLE = {"large": 0.0145, "small": 0.0118}  # square openings in the 4 mm face plate (m, side)
PLATE_T = 0.004
SPIN_S = (0.004, 0.019)  # the spindle's slotted tube, depth from the face
BORE_S = (0.019, 0.038)  # the round bore beyond it
R_IN = 0.0036  # bore and spindle inner radius (stem 3.0 mm)
SLOT_W = 0.003  # four spindle slots, fins are 2.0 mm
D_FULL = 0.036  # stem tip depth with the collar or shoulder down on the spindle
CLIP_S = 0.030  # the clip ball's depth
CLIP_R0 = 0.004  # ball centre above the socket axis at rest (protrudes 1 mm into the stem's path)
BALL_R = 0.002
CLIP_K = float(os.environ.get("MIX_CLIP_K", "3000.0"))  # N/m
CLIP_PRE = float(os.environ.get("MIX_CLIP_PRE", "0.005"))  # m: 15 N preload against its stop
SPIN_K = 0.005  # N m/deg: the spindle's gear train holds its angle
MIXER_MASS = 0.90
CHAMBER = 0.012  # half-height of the socket chamber behind the face plate


def _rxq(a: float) -> tuple[float, float, float, float]:
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def _rx(a: float) -> np.ndarray:
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _rz(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _quat(r: np.ndarray) -> tuple[float, float, float, float]:
    return bodykit._quat_from_matrix(np.asarray(r, float))


def _plate_with_holes(x0, x1, y0, y1, z0, z1, holes, color) -> list:
    """A plate x0..x1 over y0..y1, z0..z1 with square holes [(cy, cz, side)], as a grid of boxes."""
    ys = sorted({y0, y1, *[c - s / 2 for c, _, s in holes], *[c + s / 2 for c, _, s in holes]})
    zs = sorted({z0, z1, *[c - s / 2 for _, c, s in holes], *[c + s / 2 for _, c, s in holes]})
    out = []
    for ya, yb in zip(ys[:-1], ys[1:]):
        for za, zb in zip(zs[:-1], zs[1:]):
            ym, zm = (ya + yb) / 2, (za + zb) / 2
            if any(abs(ym - cy) < s / 2 and abs(zm - cz) < s / 2 for cy, cz, s in holes):
                continue
            out.append(BoxShape(((x0 + x1) / 2, ym, zm), (x1 - x0, yb - ya, zb - za), color))
    return out


def _ring(s0, s1, r_in, thick, n, color, cy=0.0, cz=0.0, x0=0.0, phase=0.0) -> list:
    """A polygonal bore of ``n`` staves round the x axis through (cy, cz)."""
    w = 2 * r_in * math.tan(math.pi / n) + 0.0002
    rc = r_in + thick / 2
    out = []
    for k in range(n):
        a = phase + 2 * math.pi * k / n
        out.append(BoxShape((x0 + (s0 + s1) / 2, cy + rc * math.cos(a), cz + rc * math.sin(a)),
                            (s1 - s0, thick, w), color, quat_xyzw=_rxq(a)))
    return out


def _mixer_shapes() -> list:
    c0, c1 = Z_S - CHAMBER, Z_S + CHAMBER
    sh = [BoxShape((0.0225, 0.0, 0.005), (0.125, 0.080, 0.010), DARK),  # heel rest, x -0.040..0.085
          BoxShape((-0.005, 0.0, (0.010 + c0) / 2), (0.070, 0.090, c0 - 0.010), WHITE),  # housing below the chamber
          BoxShape((-0.005, 0.0, (c1 + 0.200) / 2), (0.070, 0.090, 0.200 - c1), WHITE),  # housing above (motor)
          BoxShape(((0.0005 + 0.030) / 2, 0.0, Z_S), (0.0295, 0.090, 2 * CHAMBER), WHITE),  # behind the bores
          BoxShape((-0.020, 0.0425, Z_S), (0.032, 0.005, 2 * CHAMBER), WHITE),  # chamber side walls
          BoxShape((-0.020, -0.0425, Z_S), (0.032, 0.005, 2 * CHAMBER), WHITE),
          # the handle loop behind the housing (standing on the heel rest, the handle is vertical)
          BoxShape((0.045, 0.0, 0.175), (0.030, 0.025, 0.020), GREY),  # top strut (the brace grips here)
          BoxShape((0.0725, 0.0, 0.1175), (0.025, 0.025, 0.135), GREY),  # handle bar
          BoxShape((0.045, 0.0, 0.060), (0.030, 0.025, 0.020), GREY),  # lower strut
          BoxShape((-0.005, 0.0, 0.2005), (0.060, 0.050, 0.001), GREY, collide=False)]  # speed panel mark
    holes = [(SOCK_Y[k], Z_S, HOLE[k]) for k in SOCK_Y]
    sh += _plate_with_holes(FACE_X, FACE_X + PLATE_T, -0.045, 0.045, c0, c1, holes, WHITE)
    for k, y in SOCK_Y.items():
        sh += _ring(BORE_S[0], BORE_S[1], R_IN, 0.0015, 8, GREY, cy=y, cz=Z_S, x0=FACE_X, phase=math.pi / 8)
        # a ring mark round the larger opening (as on the real face)
        if k == "large":
            for a in range(8):
                ang = a * math.pi / 4
                sh.append(BoxShape((FACE_X - 0.0003, y + 0.0095 * math.cos(ang), Z_S + 0.0095 * math.sin(ang)),
                                   (0.0006, 0.002, 0.004), GREY, collide=False, quat_xyzw=_rxq(ang + math.pi / 2)))
    return sh


def _spindle_shapes() -> list:
    """Four-slot drive spindle, frame at the socket face centre (x into the mixer), slot 0 along +y."""
    s0, s1 = SPIN_S
    sm, ln = (s0 + s1) / 2, s1 - s0
    out = []
    for k in range(4):
        a = k * math.pi / 2
        d, n = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        for side in (-1.0, 1.0):
            c = d * (0.0033 + 0.0065) / 2 + n * side * (SLOT_W / 2 + 0.0015)
            out.append(BoxShape((sm, c[0], c[1]), (ln, 0.0032, 0.003), BRASS, quat_xyzw=_rxq(a)))
        f = a + math.pi / 4
        out.append(BoxShape((sm, 0.0042 * math.cos(f), 0.0042 * math.sin(f)), (ln, 0.0012, 0.0024), BRASS,
                            quat_xyzw=_rxq(f)))
    return out


# -- the beaters (beater frame: origin at the stem tip, x along the stem toward the tip) --------------------------------
STEM_R = 0.003
GROOVE = (-0.0085, -0.0045)
COLLAR_X = (-0.036, -0.032)
FIN_X = (-0.032, -0.024)
FIN_R = 0.0050  # fins 10 mm across
COLLAR_R = 0.0065  # 13 mm collar: too wide for the 11.8 mm opening
SHOULDER_R = 0.0050
CAGE_X = (-0.190, -0.120)
CAGE_R = 0.020
GRIP_X = -0.105  # the gripper takes the stem here, just ahead of the blades (the hand keeps clear of the mixer)
BEATER_MASS = {"collared": 0.060, "plain": 0.055}


def _beater_shapes(kind: str) -> list:
    sh = [SphereShape((-0.003, 0.0, 0.0), STEM_R, STEEL),
          CylShape((-0.00375, 0.0, 0.0), STEM_R, 0.0015, STEEL, axis="X"),
          CylShape(((GROOVE[0] + GROOVE[1]) / 2, 0.0, 0.0), 0.0019, GROOVE[1] - GROOVE[0], STEEL, axis="X"),
          CylShape(((CAGE_X[1] + GROOVE[0]) / 2, 0.0, 0.0), STEM_R, GROOVE[0] - CAGE_X[1], STEEL, axis="X")]
    r = COLLAR_R if kind == "collared" else SHOULDER_R
    sh.append(CylShape(((COLLAR_X[0] + COLLAR_X[1]) / 2, 0.0, 0.0), r, COLLAR_X[1] - COLLAR_X[0],
                       BRASS if kind == "collared" else STEEL, axis="X"))
    for k in range(4):
        a = k * math.pi / 2
        rc = (0.0025 + FIN_R) / 2
        sh.append(BoxShape(((FIN_X[0] + FIN_X[1]) / 2, rc * math.cos(a), rc * math.sin(a)),
                           (FIN_X[1] - FIN_X[0], FIN_R - 0.0025, 0.002), STEEL, quat_xyzw=_rxq(a)))
    sh.append(CylShape(((CAGE_X[0] + CAGE_X[1]) / 2, 0.0, 0.0), 0.0025, CAGE_X[1] - CAGE_X[0], STEEL, axis="X"))
    for j in range(4):  # four wire blades at 45 degrees to the fins
        a = math.pi / 4 + j * math.pi / 2
        u = np.array([0.0, math.cos(a), math.sin(a)])
        p = [np.array([CAGE_X[1], 0, 0]) + 0.003 * u, np.array([-0.130, 0, 0]) + CAGE_R * u,
             np.array([-0.180, 0, 0]) + CAGE_R * u, np.array([CAGE_X[0], 0, 0]) + 0.003 * u]
        for p1, p2 in zip(p[:-1], p[1:]):
            sh.append(segment(tuple(p1), tuple(p2), 0.0025, STEEL))
    return sh


# -- nominal layout ---------------------------------------------------------------------------------------------------
MIXER0 = (0.300, 0.020)  # footprint centre (world x, y) of the nominal start
MIXER_YAW0 = 90.0  # degrees: the sockets face the right arm (-y), the handle the left arm
BEATER_LANES = (0.185, 0.115)  # stem lines (world x) of the two beaters; which beater lies in which is random
BEATER_TIP_Y = -0.065  # the beaters lie pointing +y (toward the mixer), tips here
REST_Z = 0.0160  # stem axis height at the cage when lying (blades at 45 degrees on the table)

SOCKETS = ("large", "small")
SOCKET_OF = {"collared": "large", "plain": "small"}


def _build() -> list:
    mx = (MIXER0[0], MIXER0[1], TOP + 0.0005)
    objs = [Body.build("mixer", pos=mx, mass=MIXER_MASS, shapes=_mixer_shapes(), color=WHITE, friction=(0.5, 0.45),
                       solver_iterations=32, com=(-0.005, 0.0, 0.090))]
    for k in SOCKETS:
        f = (mx[0] + FACE_X, mx[1] + SOCK_Y[k], mx[2] + Z_S)
        objs.append(Body.build(f"spindle_{k}", pos=f, mass=0.008, shapes=_spindle_shapes(), color=BRASS,
                               friction=(0.3, 0.25), solver_iterations=32, ignore=("mixer",)))
        bodykit.hinge(f"spin_{k}", "mixer", f"spindle_{k}", anchor=f, axis="X", spring=SPIN_K, damping=0.0005)
    for k in SOCKETS:
        c = (mx[0] + FACE_X + CLIP_S, mx[1] + SOCK_Y[k], mx[2] + Z_S + CLIP_R0)
        objs.append(Body.build(f"clip_{k}", pos=c, mass=0.004, shapes=[SphereShape((0, 0, 0), BALL_R, STEEL)],
                               color=STEEL, friction=(0.2, 0.15), solver_iterations=32,
                               ignore=("mixer", "spindle_large", "spindle_small")))
        bodykit.slider(f"clip_slide_{k}", "mixer", f"clip_{k}", anchor=c, axis="Z", limits=(0.0, 0.0025),
                       stiffness=CLIP_K, damping=2.0, target=-CLIP_PRE, max_force=100.0)
    for i, kind in enumerate(("collared", "plain")):
        objs.append(Body.build(f"beater_{kind}", pos=(BEATER_LANES[i], BEATER_TIP_Y, TOP + REST_Z + 0.002),
                               yaw=math.pi / 2,
                               mass=BEATER_MASS[kind], shapes=_beater_shapes(kind), color=STEEL, friction=(0.3, 0.25),
                               solver_iterations=32, com=(-0.125, 0.0, 0.0),
                               damping=(0.0, 30.0) if PROBE else (0.0, 0.05),  # probe: as if held (no free roll)
                               ignore=("beater_collared",) if kind == "plain" else ()))
    return objs


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]


# -- reading the mechanism from poses -----------------------------------------------------------------------------------
def socket_frame(mixer, k: str) -> tuple[np.ndarray, np.ndarray]:
    """(world point of the socket face centre, world rotation of the mixer)."""
    r = mixer.rotation()
    return np.asarray(mixer.pos, float) + r @ np.array([FACE_X, SOCK_Y[k], Z_S]), r


def in_socket(mixer, beater, k: str) -> dict:
    """The beater's stem against socket ``k``: tip depth, lateral offset, axis angle."""
    f, r = socket_frame(mixer, k)
    tip, axis = np.asarray(beater.pos, float), beater.rotation()[:, 0]
    rel = r.T @ (tip - f)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(axis @ r[:, 0])))))
    return {"depth": float(rel[0]), "lateral": float(math.hypot(rel[1], rel[2])), "angle": ang}


def clip_lift(mixer, clip, k: str) -> float:
    """How far the clip ball is pushed out of the bore from its stop (m)."""
    rest = np.array([FACE_X + CLIP_S, SOCK_Y[k], Z_S + CLIP_R0])
    return float((mixer.rotation().T @ (np.asarray(clip.pos, float) - np.asarray(mixer.pos, float)))[2] - rest[2])


def spindle_deg(mixer, spindle) -> float:
    rel = mixer.rotation().T @ spindle.rotation()
    return math.degrees(math.atan2(rel[2, 1], rel[1, 1]))


# -- episode state -------------------------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["MIX_DEBUG"], "w") if os.environ.get("MIX_DEBUG") else None


def _blank() -> None:
    start = _EP.get("start")
    _EP.clear()
    _EP.update(step=0, hands={}, report={}, st={}, start=start, probe={})


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def _hand_points(base) -> dict:
    """Points on each gripper's fingers (world), from the arm's own joint readings; and each TCP pose."""
    from bodykit import as_numpy
    from yam_arena import ik, station

    out, tcp = {}, {}
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        tcp[arm] = (np.asarray(p, float), np.asarray(r, float))
        _EP.setdefault("q", {})[arm] = q6
        pts = []
        for side in (-1.0, 1.0):
            for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                            (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
                pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
        out[arm] = pts
    return out, tcp


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    try:
        _EP["hands"], _EP["tcp"] = _hand_points(env.unwrapped)
    except Exception as err:  # noqa: BLE001
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    if PROBE:
        _probe(env, st)
    _EP["step"] += 1
    if _DBG is not None:
        _debug(st)


# -- scoring -------------------------------------------------------------------------------------------------------------
LATERAL_MAX = 0.0015  # m: the stem on the socket axis
ANGLE_MAX = 6.0  # degrees
ENTER_S = 0.006  # tip past the spindle mouth: the stem is in the socket
FINS_S = D_FULL - 0.006  # 30 mm: only reachable with the fins in the slots (they stop at 28 mm otherwise)
LATCH_S = D_FULL - 0.0015
LATCH_LIFT = 0.0003  # the clip ball back within 0.3 mm of its stop: in the groove
POINTS = {"entered": 0.1, "fins": 0.1, "clipped": 0.2, "mixer": 0.2}
MIXER_MOVE = 0.005  # braced: 0.4-1.8 mm; unbraced pushes slide it about 12 mm
MIXER_TILT = 5.0
CLEAR_MM = 20.0


def _beater_report(objects, kind: str) -> dict:
    """The collared beater counts only in the larger socket; the plain one in whichever socket it is in (the maker:
    a beater without a collar fits either opening)."""
    if kind == "plain":
        reps = [_socket_report(objects, kind, k) for k in SOCKETS]
        return max(reps, key=lambda r: (r["score"], r["depth_mm"]))
    return _socket_report(objects, kind, "large")


def _socket_report(objects, kind: str, k: str) -> dict:
    mixer = objects["mixer"]
    s = in_socket(mixer, objects[f"beater_{kind}"], k)
    lift = clip_lift(mixer, objects[f"clip_{k}"], k)
    on_axis = s["lateral"] <= LATERAL_MAX and s["angle"] <= ANGLE_MAX
    entered = on_axis and s["depth"] >= ENTER_S
    fins = entered and s["depth"] >= FINS_S
    clipped = fins and s["depth"] >= LATCH_S and lift <= LATCH_LIFT
    sc = POINTS["entered"] * entered + POINTS["fins"] * fins + POINTS["clipped"] * clipped
    return {"socket": k, "depth_mm": round(1000 * s["depth"], 2), "lateral_mm": round(1000 * s["lateral"], 2),
            "angle_deg": round(s["angle"], 2), "clip_lift_mm": round(1000 * lift, 2), "entered": bool(entered),
            "fins_in": bool(fins), "clipped": bool(clipped), "score": round(sc, 2)}


def _beater_capsules(b) -> list:
    p, r = np.asarray(b.pos, float), b.rotation()
    return [(p, p + r @ np.array([CAGE_X[1], 0, 0]), 0.006), (p + r @ np.array([CAGE_X[1], 0, 0]),
                                                              p + r @ np.array([CAGE_X[0], 0, 0]), CAGE_R + 0.002)]


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(1e-12, ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


MIXER_BOXES = [((-0.040, -0.045, 0.0), (0.030, 0.045, 0.200)), ((-0.040, -0.040, 0.0), (0.085, 0.040, 0.010)),
               ((0.030, -0.0125, 0.050), (0.085, 0.0125, 0.185))]


def clearance_mm(objects) -> float | None:
    hands = _EP.get("hands") or {}
    if not hands:
        return None
    mixer = objects["mixer"]
    caps = [c for kind in ("collared", "plain") for c in _beater_capsules(objects[f"beater_{kind}"])]
    best = float("inf")
    for pts in hands.values():
        for p in pts:
            for a, b, rad in caps:
                best = min(best, _seg_dist(p, a, b) - rad)
            loc = mixer.to_local(p)
            for lo, hi in MIXER_BOXES:
                d = np.maximum(np.maximum(np.array(lo) - loc, loc - np.array(hi)), 0.0)
                best = min(best, float(np.linalg.norm(d)))
    return 1000.0 * best


def report(objects) -> dict:
    out = {k: _beater_report(objects, k) for k in ("collared", "plain")}
    mixer = objects["mixer"]
    start = _EP.get("start") or MIXER0
    moved = float(np.hypot(mixer.pos[0] - start[0], mixer.pos[1] - start[1]))
    tilt = math.degrees(mixer.tilt())
    standing = tilt <= MIXER_TILT and moved <= MIXER_MOVE
    out["mixer"] = {"moved_mm": round(1000 * moved, 1), "tilt_deg": round(tilt, 2), "standing": bool(standing)}
    clr = clearance_mm(objects)
    out["clearance_mm"] = None if clr is None else round(clr, 1)
    clear = clr is None or clr >= CLEAR_MM
    out["grippers_clear"] = bool(clear)
    total = out["collared"]["score"] + out["plain"]["score"] + POINTS["mixer"] * standing
    out["score"] = round(total if clear else 0.0, 2)
    return out


def score(objects) -> float:
    """Per beater (collared: counted in the larger socket only; plain: in either socket): 0.1 stem entered (tip 6 mm past the
    face, on the axis), +0.1 fins in the slots (tip at least 30 mm deep), +0.2 clipped (tip within 1.5 mm of full
    depth and the clip ball back in the groove). +0.2 if the mixer stands on its heel rest (tilt at most 5 degrees)
    within 5 mm of where it started. 0 for the trial unless both grippers end at least 20 mm from the mixer and the
    beaters."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    m = objects["mixer"]
    return sum(max(0.0, D_FULL - in_socket(m, objects[f"beater_{k}"], SOCKET_OF[k])["depth"]) for k in
               ("collared", "plain"))


# -- the seeded start ----------------------------------------------------------------------------------------------------
VARY_MIXER = (0.030, 0.030, 20.0)  # x, y (m), yaw (degrees)
VARY_BEATER = (0.012, 0.008, 12.0)


def layout(seed: int | None) -> dict:
    """World poses (position, quaternion xyzw) of every loose body for a seed (None or 0: the nominal layout), plus
    the spindle angles ("_spin_<socket>", degrees) and the beater order."""
    rng = np.random.default_rng(seed) if seed else None
    u = (lambda a: float(rng.uniform(-a, a))) if rng is not None else (lambda a: 0.0)
    mx, my, myaw = MIXER0[0] + u(VARY_MIXER[0]), MIXER0[1] + u(VARY_MIXER[1]), math.radians(MIXER_YAW0 + u(VARY_MIXER[2]))
    spin = {k: (float(rng.uniform(0.0, 90.0)) if rng is not None else 0.0) for k in SOCKETS}
    swap = bool(rng.integers(2)) if rng is not None else False
    rm = _rz(myaw)
    pm = np.array([mx, my, TOP + 0.0005])
    pose = {"mixer": (pm, _quat(rm))}
    beat_rot = {}
    if PROBE:
        aligned = {"a": {"collared": "large", "plain": "small"}, "b": {"collared": "small", "plain": "large"}}[PROBE]
        off = {"a": {"collared": 0.0, "plain": 20.0}, "b": {"collared": 0.0, "plain": 0.0}}[PROBE]
        for kind, k in aligned.items():
            f = pm + rm @ np.array([FACE_X, SOCK_Y[k], Z_S])
            r = rm @ _rx(math.radians(spin[k] + off[kind]))
            pose[f"beater_{kind}"] = (f + r[:, 0] * 0.024, _quat(r))
    else:
        for i, kind in enumerate(("collared", "plain")):
            lane = BEATER_LANES[(i + swap) % 2]
            yaw = math.radians(90.0 + u(VARY_BEATER[2]))
            r = _rz(yaw)
            beat_rot[kind] = r
            pose[f"beater_{kind}"] = (np.array([lane + u(VARY_BEATER[1]), BEATER_TIP_Y + u(VARY_BEATER[0]),
                                                TOP + REST_Z + 0.002]), _quat(r))
    for k in SOCKETS:
        pose[f"spindle_{k}"] = (pm + rm @ np.array([FACE_X, SOCK_Y[k], Z_S]), _quat(rm @ _rx(math.radians(spin[k]))))
        pose[f"clip_{k}"] = (pm + rm @ np.array([FACE_X + CLIP_S, SOCK_Y[k], Z_S + CLIP_R0]), _quat(rm))
        pose[f"_spin_{k}"] = spin[k]
    pose["_swap"] = swap
    return pose


@bodykit.on_reset_env
def _lay_out(emb, seed):
    import torch
    from yam_arena import inspect_embodiment as ie

    base = emb._base
    lay = layout(seed)
    _blank()
    for k in SOCKETS:
        bodykit.set_hinge_drive(f"spin_{k}", target_deg=lay[f"_spin_{k}"])
    for name in NAMES:
        base.scene[name].permanent_wrench_composer.reset()
    for name, val in lay.items():
        if name.startswith("_"):
            continue
        pos, quat = val
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.asarray(pos, float), dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    _EP["start"] = (float(lay["mixer"][0][0]), float(lay["mixer"][0][1]))
    _EP["spin"] = {k: lay[f"_spin_{k}"] for k in SOCKETS}
    if _DBG is not None:
        _DBG.write(f"seed {seed}: mixer=({lay['mixer'][0][0]:.4f},{lay['mixer'][0][1]:.4f}) spin="
                   f"{_EP['spin']} swap={lay['_swap']}\n")
    return emb._observation()


# -- M1 probe: push each beater in along its axis with a rising force (and the same force back on the mixer) ----------
PROBE_RAMP_S, PROBE_PEAK = 4.0, 12.0


def _push_pair(env, st, forces: dict) -> None:
    """``forces``: {beater kind: (newtons along its stem, socket)}. Each beater is pushed along its stem and carried
    against gravity at its centre of mass (a hand holding it); the push comes back on the mixer at that socket's face."""
    import torch

    base = env.unwrapped
    dev = base.device
    origin = base.scene.env_origins[0]
    zero = torch.zeros((1, 1, 3), dtype=torch.float32, device=dev)
    for name in ("mixer", *[f"beater_{k}" for k in forces]):
        base.scene[name].permanent_wrench_composer.reset()

    def add(name, force, pt, torque=None):
        f = torch.tensor(np.asarray(force, float), dtype=torch.float32, device=dev).reshape(1, 1, 3)
        tq = zero if torque is None else torch.tensor(np.asarray(torque, float), dtype=torch.float32,
                                                      device=dev).reshape(1, 1, 3)
        p = (torch.tensor(np.asarray(pt, float), dtype=torch.float32, device=dev) + origin).reshape(1, 1, 3)
        base.scene[name].permanent_wrench_composer.add_forces_and_torques(f, tq, positions=p, body_ids=[0],
                                                                         is_global=True)

    for kind, (newtons, k) in forces.items():
        b = st[f"beater_{kind}"]
        axis = b.rotation()[:, 0]
        com = np.asarray(b.pos, float) + b.rotation() @ np.array([-0.125, 0.0, 0.0])
        add(f"beater_{kind}", newtons * axis + np.array([0.0, 0.0, 9.81 * BEATER_MASS[kind]]), com)
        if newtons:
            add("mixer", -newtons * axis, socket_frame(st["mixer"], k)[0])


def _probe(env, st) -> None:
    """1 s still; push rising to 12 N over 4 s, hold 1 s; nothing for 1 s; pull rising to -12 N over 4 s."""
    hz = int(CONTROL_HZ)
    t = _EP["step"] / CONTROL_HZ
    sock = {"a": {"collared": "large", "plain": "small"}, "b": {"collared": "small", "plain": "large"}}[PROBE]
    if t < 1.0:
        f = 1.0
    elif t < 1.0 + PROBE_RAMP_S:
        f = max(1.0, PROBE_PEAK * (t - 1.0) / PROBE_RAMP_S)
    elif t < 2.0 + PROBE_RAMP_S:
        f = PROBE_PEAK
    elif t < 3.0 + PROBE_RAMP_S:
        f = 0.0
    elif t < 3.0 + 2 * PROBE_RAMP_S:
        f = -PROBE_PEAK * (t - 3.0 - PROBE_RAMP_S) / PROBE_RAMP_S
    else:
        f = 0.0
    p = _EP["probe"]
    live = {}
    for kind, k in sock.items():
        s = in_socket(st["mixer"], st[f"beater_{kind}"], k)
        live[kind] = (f if -0.005 < s["depth"] < 0.045 and s["lateral"] < 0.004 else 0.0, k)
    _push_pair(env, st, live)
    for kind, k in sock.items():
        s = in_socket(st["mixer"], st[f"beater_{kind}"], k)
        lift = clip_lift(st["mixer"], st[f"clip_{k}"], k)
        rec = p.setdefault(kind, {"max_lift": 0.0})
        rec["max_lift"] = max(rec["max_lift"], lift)
        if f > 0 and "f_latch" not in rec and s["depth"] >= LATCH_S and lift <= LATCH_LIFT:
            rec["f_latch"] = f
        if f > 0:
            rec["depth_push"] = s["depth"]
        if f < 0 and rec.get("f_latch") is not None and "f_out" not in rec and s["depth"] < LATCH_S - 0.001:
            rec["f_out"] = f
        rec["depth_end"] = s["depth"]
    if _EP["step"] % hz == 0:
        line = f"PROBE t={t:4.1f} f={f:5.2f} " + " ".join(
            f"{kind}->{k}: depth={1000 * in_socket(st['mixer'], st[f'beater_{kind}'], k)['depth']:6.2f} "
            f"lift={1000 * clip_lift(st['mixer'], st[f'clip_{k}'], k):5.2f} "
            f"spin={spindle_deg(st['mixer'], st[f'spindle_{k}']):6.2f}" for kind, k in sock.items())
        print(line, flush=True)
        if t >= 3.0 + 2 * PROBE_RAMP_S + 0.9 and not p.get("done"):
            p["done"] = True
            print("MIX PROBE " + repr({kk: {a: (round(b, 5) if isinstance(b, float) else b) for a, b in v.items()}
                                      for kk, v in p.items() if isinstance(v, dict)}), flush=True)


def _debug(st) -> None:
    m = st["mixer"]
    parts = [f"{_EP['step']:4d}"]
    for kind in ("collared", "plain"):
        k = SOCKET_OF[kind]
        s = in_socket(m, st[f"beater_{kind}"], k)
        parts.append(f"{kind[0]}: d={1000 * s['depth']:7.2f} lat={1000 * s['lateral']:6.2f} ang={s['angle']:6.2f} "
                     f"lift={1000 * clip_lift(m, st[f'clip_{k}'], k):5.2f} spin={spindle_deg(m, st[f'spindle_{k}']):6.2f}")
    parts.append(f"mixer=({m.pos[0]:.4f},{m.pos[1]:.4f}) tilt={math.degrees(m.tilt()):5.2f}")
    rep = _EP.get("report") or {}
    parts.append(f"clr={rep.get('clearance_mm')} score={rep.get('score')}")
    _DBG.write(" ".join(parts) + "\n")
    _DBG.flush()


# -- the scripted demo (mixdemo.py plans each stage from the simulator's object poses) -----------------------------------
import mixdemo  # noqa: E402

demo = mixdemo.demo
plan_stage = mixdemo.plan_stage
LOOKS = 0 if PROBE else mixdemo.N_STAGES
mixdemo_task = True
mixdemo.bind(globals())
mixdemo.install()
