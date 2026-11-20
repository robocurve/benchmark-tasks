"""A hotel wardrobe's anti-theft hanger, put back on its security ring.

Hotel hangers have no hook: a chrome ball top on a short pin. The ball sits in the cup of a stamped steel security
ring (a 1.5 in A-ring) that stays on the closet rod. The ring is loose on the rod: it slides along it, swings about it
and, because its opening is 5 mm wider than the rod, turns up to about 30 degrees about the vertical, where it stays
(nothing turns it back). The cup is the ring's bottom tab bent out 90 degrees into a small shelf with a slot open at
its edge and a low lip; the hanger's pin can only enter that slot from the side, along the ring's normal, with the
ball held above the lip, and then the ball is let down into the cup. The ring weighs 12 g and moves at a touch, so
the usual way is to hold the ring still and square with one hand while the other brings the ball in.

On the station table: a closet rod on two uprights, across the table; the empty ring B; ring A 200 mm to its right
holding hanger A (the neighbour); hanger B lying flat on the table.

Frame: x forward from the arm bases, y left, z up, metres.
"""

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
from bodykit import Body, BoxShape, CylShape, SphereShape  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
CONTROL_HZ = 20.0

# -- the closet rod and its frame ---------------------------------------------------------------------------------------
ROD_X, ROD_Z = 0.40, TOP + 0.36  # rod axis: 400 mm in front of the arm bases, 360 mm above the table
ROD_R, ROD_L = 0.0165, 0.80  # 1-5/16 in (33 mm) closet rod
UP_Y, UP_T, UP_D = 0.40, 0.018, 0.18  # the two plywood uprights: at y +-400 mm, 18 mm thick, 180 mm deep
FRAME_MASS = 30.0
WOOD, WOOD_DARK, CHROME, BLACK = (0.82, 0.68, 0.48), (0.62, 0.47, 0.30), (0.80, 0.81, 0.83), (0.10, 0.10, 0.11)
BOX_C, BOX_LID = (0.72, 0.60, 0.45), (0.80, 0.68, 0.52)

# -- the security ring (NAHANCO HHSRA100 A-ring): ring frame at the opening's centre, plane x-z, the cup toward +y ----
RING_RI, RING_RO, RING_T = 0.01905, 0.0238, 0.0015  # 1.5 in opening; 1-7/8 in wide; 1.5 mm plate
RING_DROP = 0.0015  # the opening's centre sits this far below the rod's axis (its top edge 1 mm above the rod)
TAB_HW = 0.0107  # the tab: 21.4 mm wide
TAB_TOP, BEND_Z = -0.019, -0.0319  # the flat tab from the band down to the bend; the shelf's top face at BEND_Z
SLOT_HW, SLOT_TOP = 0.0014, -0.0265  # the slot: 2.8 mm wide, from the bend up the tab to here
SHELF_D, LIP_D, LIP_H = 0.0090, 0.0015, 0.0020  # shelf reaches 9 mm out from the plate face; its edge turned up 2 mm
MOUTH_R = 0.0010  # the slot mouth's corners rounded off 1 mm
RING_MASS = 0.012
BALL_R, PIN_R, PIN_L = 0.0035, 0.0009, 0.0515  # chrome ball 7 mm, pin 1.8 mm thick, 51.5 mm ball centre to wood
# the ball's seat in the cup (ring frame): between the plate face and the lip, resting on the slot's edges
SEAT = np.array([0.0, RING_T / 2 + (SHELF_D - LIP_D) / 2, BEND_Z + math.sqrt(BALL_R**2 - SLOT_HW**2)])

# -- the hanger (NAHANCO 60217BEC 17-1/2 in wooden suit hanger): frame at the ball's centre, plane x-z ------------------
HW_T = 0.0127  # 1/2 in thick
WOOD_TOP = -(PIN_L + BALL_R)  # top of the wooden crown
CROWN_HW, CROWN_H = 0.028, 0.030  # the hump's upper part: 56 mm wide, 30 mm tall
HUMP_HW, HUMP_H = 0.045, 0.017  # its lower part, where the shoulders leave it: 90 mm wide
HANG_HW, BAR_Z = 0.2223, WOOD_TOP - 0.153  # 445 mm tip to tip; the pant bar's axis
HANGER_MASS = 0.13
HANGER_SPIN_DAMP = 0.6  # 1/s: the ball's rub in the cup and the air stop a hung hanger turning and swinging within a few seconds
GRIP_Z = WOOD_TOP - 0.0105  # the hump's upper part, 18 mm above its wider lower part: where the demo pinches it
GRIP_ROLL = float(os.environ.get("HANGER_GRIP_ROLL", "-1"))  # which way round the pinching hand is turned
GRIP_IN = CROWN_HW - 0.004  # the fingertips 4 mm short of the hump's middle (the pin rises from the middle)

# -- the closed shoe box hanger B lies on (a men's shoe box with its shoes) --------------------------------------------
BOX_L, BOX_W, BOX_H, BOX_MASS = 0.330, 0.180, 0.120, 1.2
BOX_EDGE = 0.120  # the box's near edge this far from hanger B's ball (along the hanger): the hump sticks out past it

RING_B_Y, RING_A_Y = -0.02, -0.26  # reference ring positions on the rod (y)
SPECKS = {"ring_b": RING_B_Y, "ring_a": RING_A_Y}


def _box(lo, hi, color, collide: bool = True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _frame_shapes() -> list:
    zt = TOP - ROD_Z  # the table top, rod-relative
    s = [CylShape((0.0, 0.0, 0.0), ROD_R, ROD_L, WOOD, axis="Y")]
    for sy in (-1.0, 1.0):
        y0, y1 = sorted((sy * UP_Y, sy * (UP_Y + UP_T)))
        s.append(_box((-UP_D / 2, y0, zt), (UP_D / 2, y1, 0.045), WOOD_DARK))  # upright
        f0, f1 = sorted((sy * (UP_Y - 0.03), sy * (UP_Y + UP_T + 0.03)))
        s.append(_box((-0.11, f0, zt), (0.11, f1, zt + 0.018), WOOD_DARK))  # its foot on the table
    return s


def _ring_shapes() -> list:
    t = RING_T / 2
    s = []
    n = 20
    for k in range(n):  # the band, as 20 flat segments
        a = 2 * math.pi * (k + 0.5) / n
        if -0.5 * math.pi - 0.45 < a - 2 * math.pi * (a > math.pi) < -0.5 * math.pi + 0.45:
            continue  # the bottom of the band is the tab
        r = (RING_RI + RING_RO) / 2
        c = np.array([r * math.cos(a), 0.0, r * math.sin(a)])
        tan = np.array([-math.sin(a), 0.0, math.cos(a)])
        seg = 2 * math.pi * RING_RO / n * 1.05
        s.append(bodykit.plate(c - 0.5 * seg * tan, c + 0.5 * seg * tan, RING_RO - RING_RI, RING_T,
                               (0.0, 1.0, 0.0), CHROME))
    # the tab: solid above the slot, two legs beside it down to the bend
    s.append(_box((-TAB_HW, -t, SLOT_TOP), (TAB_HW, t, -RING_RI + 0.002), CHROME))
    s.append(_box((-0.016, -t, -0.0225), (0.016, t, TAB_TOP), CHROME))  # where the band meets the tab
    y_m = t + SHELF_D  # the slot's mouth
    zc = BEND_Z + (LIP_H - RING_T) / 2
    for sx in (-1.0, 1.0):
        x0, x1 = sorted((sx * SLOT_HW, sx * TAB_HW))
        m0, m1 = sorted((sx * (SLOT_HW + MOUTH_R), sx * TAB_HW))
        s.append(_box((x0, -t, BEND_Z), (x1, t, SLOT_TOP), CHROME))  # tab leg
        s.append(_box((x0, -t, BEND_Z - RING_T), (x1, y_m - MOUTH_R, BEND_Z), CHROME))  # shelf (bent out 90 degrees)
        s.append(_box((m0, y_m - MOUTH_R, BEND_Z - RING_T), (m1, y_m, BEND_Z), CHROME))  # ... its edge at the mouth
        s.append(_box((m0, y_m - LIP_D, BEND_Z), (m1, y_m, BEND_Z + LIP_H), CHROME))  # turned-up lip
        # the mouth's rounded corner, as a 45 degree face from the slot's side to the shelf's edge
        n_out = np.array([-sx, 1.0, 0.0]) / math.sqrt(2.0)
        p1 = np.array([sx * SLOT_HW, y_m - MOUTH_R, zc]) - 0.0003 * n_out
        p2 = np.array([sx * (SLOT_HW + MOUTH_R), y_m, zc]) - 0.0003 * n_out
        s.append(bodykit.plate(p1, p2, RING_T + LIP_H, 0.0006, n_out, CHROME))
    return s


def _hanger_shapes() -> list:
    t = HW_T / 2
    s = [SphereShape((0.0, 0.0, 0.0), BALL_R, CHROME),
         CylShape((0.0, 0.0, -(PIN_L + 0.004) / 2), PIN_R, PIN_L - 0.004 + 0.002, CHROME, axis="Z"),
         _box((-CROWN_HW, -t, WOOD_TOP - CROWN_H), (CROWN_HW, t, WOOD_TOP), BLACK),
         _box((-HUMP_HW, -t, WOOD_TOP - CROWN_H - HUMP_H), (HUMP_HW, t, WOOD_TOP - CROWN_H), BLACK)]
    for sx in (-1.0, 1.0):  # the two shoulders, the notched ends, the pant bar's end blocks
        s.append(bodykit.plate((sx * 0.035, 0.0, WOOD_TOP - 0.038), (sx * 0.205, 0.0, WOOD_TOP - 0.128), 0.026, HW_T,
                               (0.0, 1.0, 0.0), BLACK))
        x0, x1 = sorted((sx * 0.195, sx * HANG_HW))
        s.append(_box((x0, -t, BAR_Z - 0.008), (x1, t, WOOD_TOP - 0.118), BLACK))
    s.append(CylShape((0.0, 0.0, BAR_Z), 0.0055, 2 * 0.200, BLACK, axis="X"))  # pant bar
    return s


def _ring_pos(y: float) -> tuple:
    return (ROD_X, y, ROD_Z - RING_DROP)


def _build() -> list:
    frame = Body.build("frame", pos=(ROD_X, 0.0, ROD_Z), mass=FRAME_MASS, shapes=_frame_shapes(), color=WOOD,
                       friction=(1.0, 0.9), solver_iterations=32, damping=(0.5, 0.5))
    out = [frame]
    for name, y in SPECKS.items():
        car = Body.build(f"car_{name[-1]}", pos=_ring_pos(y), mass=0.01, inertia=(1e-6, 1e-6, 1e-6),
                         shapes=[_box((-0.002, -0.002, -0.002), (0.002, 0.002, 0.002), CHROME, collide=False)],
                         color=CHROME, ignore=("frame",))
        ring = Body.build(name, pos=_ring_pos(y), mass=RING_MASS, shapes=_ring_shapes(), color=CHROME,
                          friction=(0.4, 0.3), solver_iterations=32, contact_offset=0.001, damping=(0.0, 0.0),
                          ignore=(car.name,))
        out += [car, ring]
        # the ring slides along the rod (a light rub) and swings and turns on it (the rod stops the turn at ~30 deg)
        bodykit.slider(f"slide_{name[-1]}", "frame", car.name, anchor=_ring_pos(y), axis="Y", limits=(-0.30, 0.30),
                       damping=0.3)
        bodykit.ball(f"pivot_{name[-1]}", car.name, name, anchor=_ring_pos(y), swing_damping=0.0006,
                     twist_damping=0.002)
    out.append(Body.build("shoe_box", pos=(0.25, 0.25, TOP + BOX_H / 2), mass=BOX_MASS,
                          shapes=[_box((-BOX_L / 2, -BOX_W / 2, -BOX_H / 2), (BOX_L / 2, BOX_W / 2, BOX_H / 2), BOX_C),
                                  _box((-BOX_L / 2 - 0.002, -BOX_W / 2 - 0.002, BOX_H / 2 - 0.035),
                                       (BOX_L / 2 + 0.002, BOX_W / 2 + 0.002, BOX_H / 2), BOX_LID)],
                          color=BOX_C, friction=(0.7, 0.6), solver_iterations=16))
    for name in ("hanger_a", "hanger_b"):
        out.append(Body.build(name, pos=(0.25, 0.10 if name == "hanger_b" else -0.25, TOP + 0.10), mass=HANGER_MASS,
                              shapes=_hanger_shapes(), color=BLACK, friction=(0.9, 0.8), solver_iterations=32,
                              contact_offset=0.001, damping=(0.05, HANGER_SPIN_DAMP)))
    return out


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]
SPEC = {o.name: o for o in OBJECTS}


def _rx(a: float) -> np.ndarray:
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _rz(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _quat(r: np.ndarray) -> tuple:
    return bodykit._quat_from_matrix(r)


FLAT = _rx(math.radians(90.0))  # the hanger lying flat: its pin toward -y, the pant bar toward +y


# -- reading the scene ------------------------------------------------------------------------------------------------
def ball_in_ring(st, hanger: str, ring: str) -> np.ndarray:
    return st[ring].to_local(st[hanger].pos)


def seated(st, hanger: str, ring: str) -> bool:
    """The ball in the ring's cup: within 3 mm of the seat across the slot, between the plate and the lip, and resting
    (within 2.5 mm of the seat's height)."""
    b = ball_in_ring(st, hanger, ring)
    return (abs(b[0]) <= 0.003 and RING_T / 2 + 0.001 <= b[1] <= RING_T / 2 + SHELF_D - LIP_D
            and abs(b[2] - SEAT[2]) <= 0.0025)


def ring_yaw(st, ring: str) -> float:
    """The ring's turn about the vertical (degrees; 0 square to the rod)."""
    n = st[ring].rotation()[:, 1]
    return math.degrees(math.atan2(-n[0], n[1]))


def hanger_heading(st, hanger: str) -> float:
    """The hanger's shoulder line about the vertical (degrees, modulo 180; 0 along x, square to the rod)."""
    a = st[hanger].rotation()[:, 0]
    d = math.degrees(math.atan2(a[1], a[0]))
    return (d + 90.0) % 180.0 - 90.0


def lowest_z(st, name: str) -> float:
    lo, hi = SPEC[name].local_bounds()
    r, p = st[name].rotation(), np.asarray(st[name].pos, float)
    pts = [p + r @ np.array([x, y, z]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    return min(q[2] for q in pts)


def finger_points(p, r, half: float) -> list[np.ndarray]:
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> list:
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out.extend(finger_points(p, r, half))
    return out


def clearance(st, points, names=("hanger_b", "ring_b")) -> float:
    """Smallest distance (m) from the finger points to the named bodies' bounding boxes."""
    best = float("inf")
    for name in names:
        lo, hi = SPEC[name].local_bounds()
        for p in points:
            loc = st[name].to_local(p)
            d = np.maximum(0.0, np.maximum(lo - loc, loc - hi))
            best = min(best, float(np.linalg.norm(d)))
    return best


# -- episode state, the score -----------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["HANGER_DEBUG"], "w") if os.environ.get("HANGER_DEBUG") else None
HANDS_CLEAR = 0.03  # every finger point at least 30 mm from hanger B and ring B
STILL_STEPS = 40  # the last 2 s of the trial
STILL_BALL = 0.002  # hanger B's ball moves less than 2 mm over those 2 s
SQUARE_DEG = 15.0  # hanger B's shoulders within 15 degrees of square to the rod (as hanger A hangs at the start)
RING_MOVE = 0.015  # a ring within 15 mm of its start mark on the rod
TABLE_GAP = 0.02  # hanger B hangs at least 20 mm above the table


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, hist=[], st=None, clear=float("inf"), start=None, report={})


_blank()


@bodykit.on_reset
def _reset() -> None:
    start = _EP.get("start")
    _blank()
    _EP["start"] = start


def evaluate(st, hist, start) -> dict:
    """The score of a state (the end of the trial) and the 2 s before it."""
    b_ok = seated(st, "hanger_b", "ring_b")
    a_ok = seated(st, "hanger_a", "ring_a")
    a = np.array(hist) if hist else np.zeros((0, 5))
    still = len(a) >= STILL_STEPS and float(np.max(np.ptp(a[:, 0:3], axis=0))) <= STILL_BALL
    clear = len(a) >= STILL_STEPS and float(a[:, 3].min()) >= HANDS_CLEAR
    off_table = lowest_z(st, "hanger_b") >= TOP + TABLE_GAP
    sq = abs(hanger_heading(st, "hanger_b"))  # 0: shoulders square to the rod (along x)
    rb = abs(float(st["ring_b"].pos[1]) - start["ring_b"]) if start else 0.0
    ra = abs(float(st["ring_a"].pos[1]) - start["ring_a"]) if start else 0.0
    gate = b_ok and still and clear and off_table
    score = 0.0
    if gate:
        score = 0.6 + 0.2 * (sq <= SQUARE_DEG) + 0.1 * (rb <= RING_MOVE) + 0.1 * (a_ok and ra <= RING_MOVE)
    return dict(score=round(score, 3), seated_b=b_ok, still=still, hands_clear=clear, off_table=off_table,
                square_deg=round(sq, 1), ring_b_moved_mm=round(rb * 1000, 1), seated_a=a_ok,
                ring_a_moved_mm=round(ra * 1000, 1),
                ball_b_in_ring=[round(v * 1000, 1) for v in ball_in_ring(st, "hanger_b", "ring_b")],
                ring_b_yaw=round(ring_yaw(st, "ring_b"), 1))


def _track(st, clear: float) -> None:
    e = _EP
    e["hist"].append((*np.asarray(st["hanger_b"].pos, float).tolist(), clear, hanger_heading(st, "hanger_b")))
    if len(e["hist"]) > STILL_STEPS:
        e["hist"].pop(0)


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    try:
        clear = clearance(st, _hand_points(env.unwrapped))
    except Exception as err:  # noqa: BLE001
        clear = 0.0
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    _EP["clear"] = clear
    _track(st, clear)
    _EP["step"] += 1
    if _DBG is not None and _EP["step"] % 2 == 0:
        _debug(st)


def report(objects=None) -> dict:
    st = _EP.get("st")
    if st is None:
        return dict(score=0.0)
    return evaluate(st, _EP["hist"], _EP.get("start"))


def score(objects) -> float:
    """Read at the end of the trial. 0 unless hanger B's ball rests in ring B's cup (within 3 mm of the seat across
    the slot, between the plate and the lip, within 2.5 mm of the seat's height), hanger B is at least 20 mm above
    the table, every finger point has been at least 30 mm from hanger B and ring B for the last 2 s, and the ball has
    moved less than 2 mm in those 2 s. Then 0.6, +0.2 if hanger B's shoulders are within 15 degrees of square to the rod,
    +0.1 if ring B is within 15 mm of its start mark on the rod, +0.1 if hanger A is still seated in ring A and ring A
    is within 15 mm of its mark."""
    rep = report(objects)
    _EP["report"] = rep
    if _DBG is not None:
        _DBG.write(f"score report: {rep}\n")
        _DBG.flush()
    return float(rep["score"])


def distance(objects) -> float:
    st = _EP.get("st")
    if st is None:
        return 1.0
    return float(np.linalg.norm(ball_in_ring(st, "hanger_b", "ring_b") - SEAT))


def _debug(st) -> None:
    b = ball_in_ring(st, "hanger_b", "ring_b") * 1000
    _DBG.write(f"{_EP['step']:4d} ballB=({b[0]:6.1f},{b[1]:6.1f},{b[2]:6.1f})mm seatB={int(seated(st, 'hanger_b', 'ring_b'))} "
               f"seatA={int(seated(st, 'hanger_a', 'ring_a'))} ringB_y={st['ring_b'].pos[1]:.4f} "
               f"yawB={ring_yaw(st, 'ring_b'):5.1f} headB={hanger_heading(st, 'hanger_b'):6.1f} "
               f"headA={hanger_heading(st, 'hanger_a'):6.1f} clear={_EP['clear']:.3f} hB=({st['hanger_b'].pos[0]:.3f},{st['hanger_b'].pos[1]:.3f},{st['hanger_b'].pos[2]:.3f})\n")
    _DBG.flush()


# -- the seeded start ---------------------------------------------------------------------------------------------------
RING_SHIFT, RING_YAW_MIN, RING_YAW_MAX = 0.040, 8.0, 22.0
HANGER_DX, HANGER_DY, HANGER_YAW = 0.010, 0.020, 10.0
HANGER_B_XY = (0.31, 0.04)  # hanger B's ball, lying on the shoe box (reference)
FAIL = os.environ.get("HANGER_FAIL", "")  # wrong outcomes on purpose (README): fixed, nothing, nohold, holding


def layout(seed: int | None) -> dict:
    """Ring B's shift along the rod (m) and turn (degrees), hanger B's shift (m) and turn on the table (degrees)."""
    if not seed:
        return dict(ring_dy=0.0, ring_yaw=0.0, hx=0.0, hy=0.0, hyaw=0.0)
    rng = np.random.default_rng(seed)
    return dict(ring_dy=float(rng.uniform(-RING_SHIFT, RING_SHIFT)),
                ring_yaw=float(rng.choice([-1.0, 1.0]) * rng.uniform(RING_YAW_MIN, RING_YAW_MAX)),
                hx=float(rng.uniform(-HANGER_DX, HANGER_DX)), hy=float(rng.uniform(-HANGER_DY, HANGER_DY)),
                hyaw=float(rng.uniform(-HANGER_YAW, HANGER_YAW)))


def seat_pose(ring_p, ring_r) -> tuple[np.ndarray, np.ndarray]:
    """Where a seated hanger's ball is, hanging square from a ring at (ring_p, ring_r)."""
    return np.asarray(ring_p, float) + ring_r @ SEAT, ring_r


def poses(lay: dict) -> dict:
    """World poses (position, quaternion xyzw) of every body for a layout."""
    yb = RING_B_Y + lay["ring_dy"]
    rb = _rz(math.radians(lay["ring_yaw"]))
    pa, ra = seat_pose(_ring_pos(RING_A_Y), np.eye(3))
    rz = _rz(math.radians(lay["hyaw"]))
    hb = rz @ FLAT
    pb = np.array([HANGER_B_XY[0] + lay["hx"], HANGER_B_XY[1] + lay["hy"], TOP + BOX_H + HW_T / 2 + 0.0005])
    pbox = pb + rz @ np.array([0.0, BOX_EDGE + BOX_W / 2, -(HW_T / 2 + 0.0005) - BOX_H / 2])
    return {
        "frame": (np.array([ROD_X, 0.0, ROD_Z]), (0.0, 0.0, 0.0, 1.0)),
        "car_a": (np.asarray(_ring_pos(RING_A_Y)), (0.0, 0.0, 0.0, 1.0)),
        "ring_a": (np.asarray(_ring_pos(RING_A_Y)), (0.0, 0.0, 0.0, 1.0)),
        "car_b": (np.asarray(_ring_pos(yb)), (0.0, 0.0, 0.0, 1.0)),
        "ring_b": (np.asarray(_ring_pos(yb)), _quat(rb)),
        "hanger_a": (pa, _quat(ra)),
        "shoe_box": (pbox, _quat(rz)),
        "hanger_b": (pb, _quat(hb)),
    }


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put the bodies where this trial's seed says, let them settle, and return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    base = emb._base
    lay = layout(seed)
    _blank()
    for name, (pos, quat) in poses(lay).items():
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
    st = _scene.object_states(emb._env, NAMES)
    _EP["st"] = st
    _EP["layout"] = lay
    _EP["start"] = {"ring_b": float(st["ring_b"].pos[1]), "ring_a": float(st["ring_a"].pos[1])}
    if _DBG is not None:
        _DBG.write(f"seed {seed}: layout {lay}\n")
        _debug(st)
    return emb._observation()


# -- the scripted demo (reads object poses from the simulator to plan; moves the parts only through the grippers) ------
HOLD_X = float(os.environ.get("HANGER_HOLD_X", "0.0014"))  # the holder's fingertips reach this far onto the tab's front edge
HOLD_Z = float(os.environ.get("HANGER_HOLD_Z", "-0.021"))  # the fingertips this far below the opening's centre
HOLD_TILT = math.radians(float(os.environ.get("HANGER_HOLD_TILT", "28")))  # the holder comes in from the front, tilted down (its body rises clear of the carrying hand)
HOLD_OPEN = -0.015
HOLD_SQUEEZE = -0.0005  # 0.25 mm past contact on the 1.5 mm plate: firm, and it lets go without dragging the ring
PRE_Y = 0.028  # the ball waits this far out from its seat (along the ring's normal) before going in
OVER_Z = float(os.environ.get("HANGER_OVER_Z", "0.0055"))  # ... and this far above the seat, over the lip
DROP_GAP = 0.0004  # let go of the hanger this far above the seat
CROWN_SQUEEZE = 0.0  # full squeeze on the hump's two flat faces
CROWN_OPEN = -0.030
LIFT = 0.12  # lift the hanger this far off the shoe box before standing it up
APPROACH_BACK = 0.060  # the left hand comes down this far in front of its pinch point (clear of the hump) ...
APPROACH_UP = 0.060  # ... from this high above it, then slides in level


def ready_pose(arm: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def demo():
    """Both arms wait at the ready pose; the real plan is made on the first control step (hangerdemo.py)."""
    return {arm: [park(arm, 0.5, label="look")] for arm in ("left", "right")}


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) / np.linalg.norm(close)
    x = x - z * float(x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _wp(sec, pos, finger, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(np.asarray(pos, float).tolist()), float(finger), label, np.asarray(rot, float))


def _grasp_offset() -> tuple[np.ndarray, np.ndarray]:
    """The TCP's pose in the hanger's frame for the hump pinch: pointing along the hanger's +x (in from its front
    shoulder side), the fingers closing across the hanger's thickness on the hump's upper part, the fingertips GRIP_IN
    in from the hump's front edge. Hung, the hand points away from the robot with its wrist in front of the rod; lying
    on the shoe box, where the hump sticks out past the box's edge, the lower finger goes under the hump."""
    r = _frame((1.0, 0.0, 0.0), (0.0, GRIP_ROLL, 0.0))
    return np.array([-CROWN_HW + GRIP_IN - 0.013, 0.0, GRIP_Z]), r


def tcp_for_hanger(p_h, r_h) -> tuple[np.ndarray, np.ndarray]:
    off_p, off_r = _grasp_offset()
    return np.asarray(p_h, float) + r_h @ off_p, r_h @ off_r


def ball_target(ring_p, ring_y: float, out: float, up: float) -> np.ndarray:
    """A ball position in ring B's squared frame (the ring held square to the rod), ``out`` along its normal from the
    seat and ``up`` above it."""
    return np.asarray(ring_p, float) + np.array([0.0, SEAT[1] + out, SEAT[2] + up])


def plan_from_state(st, lay=None) -> dict[str, list[Waypoint]]:
    """Both hands' plans, from the start poses of ring B and hanger B."""
    if FAIL == "nothing":
        return {arm: [park(arm, 40.0, label="wait")] for arm in ("left", "right")}
    if FAIL == "fixed":  # wrong on purpose: the reference layout of the setup text, not the poses
        ref = poses(layout(0))
        rb_p, hb_p = np.asarray(ref["ring_b"][0]), np.asarray(ref["hanger_b"][0])
        rb_r = bodykit.rotation(ref["ring_b"][1])
        hb_r = bodykit.rotation(ref["hanger_b"][1])
    else:
        rb_p, rb_r = np.asarray(st["ring_b"].pos, float), st["ring_b"].rotation()
        hb_p, hb_r = np.asarray(st["hanger_b"].pos, float), st["hanger_b"].rotation()
    ring_p = np.array([ROD_X, rb_p[1], rb_p[2]])  # ring B squared to the rod, where it is along it
    _EP["plan_ring"] = ring_p.copy()

    # left hand: pinch hanger B's crown from above, lift, stand it up, bring the ball over the cup, in, down, let go
    g_p, g_r = tcp_for_hanger(hb_p, hb_r)
    up = np.array([0.0, 0.0, 1.0])
    # the swing from the ready pose ends high and well in front of the hump (the hand rings +-30 mm for about a second
    # after a long move), waits there to settle, comes down slowly, then slides in level along the hanger
    front = g_p - APPROACH_BACK * g_r[:, 2]
    left = [
        _wp(3.0, front + APPROACH_UP * up, CROWN_OPEN, "above, in front of the hump", g_r),
        _wp(1.0, front + APPROACH_UP * up, CROWN_OPEN, "above, in front of the hump", g_r),
        _wp(1.6, front, CROWN_OPEN, "in front of the hump", g_r),
        _wp(0.5, front, CROWN_OPEN, "in front of the hump", g_r),
        _wp(2.2, g_p, CROWN_OPEN, "fingers either side of the hump", g_r),
        _wp(0.8, g_p, CROWN_SQUEEZE, "pinch the hump", g_r),
        _wp(0.6, g_p + 0.003 * up, CROWN_SQUEEZE, "lift the hanger", g_r),
        _wp(1.8, g_p + LIFT * up, CROWN_SQUEEZE, "lift the hanger", g_r),
    ]
    p_pre = ball_target(ring_p, 0, PRE_Y, OVER_Z)
    t_pre_p, t_pre_r = tcp_for_hanger(p_pre, np.eye(3))
    # stand the hanger up on the way: hanger turns from lying flat to hanging square, about its crown
    lift_p = g_p + LIFT * up
    # first turn it, still lying flat, square to the rod (the wrist cannot roll it up from a turned start)
    left.append(_wp(1.0, lift_p, CROWN_SQUEEZE, "turn the hanger square", tcp_for_hanger(np.zeros(3), FLAT)[1]))
    for i, s in enumerate((0.33, 0.66, 1.0)):
        r = bodykit.rotation(_slerp_q(FLAT, np.eye(3), s))
        mid = lift_p + s * (t_pre_p + np.array([-0.06, 0.05, -0.02]) - lift_p)  # in front of and below the rod
        left.append(_wp(1.5, mid, CROWN_SQUEEZE, "stand the hanger up", tcp_for_hanger(np.zeros(3), r)[1]))
    left.append(_wp(1.0, t_pre_p + np.array([0.0, 0.0, 0.0]), CROWN_SQUEEZE, "ball out beside the cup", t_pre_r))
    left.append(_wp(0.8, t_pre_p, CROWN_SQUEEZE, "aim", t_pre_r))  # hangerdemo re-aims here from the ball's pose
    t_wait = sum(w.duration for w in left)
    p_in = ball_target(ring_p, 0, 0.0, OVER_Z)
    p_dn = ball_target(ring_p, 0, 0.0, DROP_GAP)
    in_p, _ = tcp_for_hanger(p_in, np.eye(3))
    dn_p, _ = tcp_for_hanger(p_dn, np.eye(3))

    # right hand: pinch ring B's tab from the front and hold the ring square to the rod
    ha = np.array([math.cos(HOLD_TILT), 0.0, -math.sin(HOLD_TILT)])  # in from the front, tilted down onto the tab
    hr = _frame(ha, (0.0, -1.0, 0.0))  # closing across the plate: the tab's front corner and the band above it
    h_p = ring_p + np.array([-TAB_HW + HOLD_X, 0.0, HOLD_Z]) - 0.013 * ha
    right = [
        park("right", 1.0, HOLD_OPEN, label="wait"),
        _wp(2.5, h_p - 0.045 * ha, HOLD_OPEN, "in front of the ring", hr),
        _wp(1.5, h_p, HOLD_OPEN, "fingers either side of the ring", hr),
        _wp(0.8, h_p, HOLD_SQUEEZE, "pinch the ring square", hr),
    ]
    t_hold = sum(w.duration for w in right)
    if FAIL == "nohold":  # wrong on purpose: the ring is never held
        right = [park("right", 1.0, HOLD_OPEN, label="wait")]
        t_hold = 0.0
    if t_hold > t_wait:
        left.insert(len(left) - 1, _wp(t_hold - t_wait, t_pre_p, CROWN_SQUEEZE, "ball out beside the cup", t_pre_r))
    left += [
        _wp(1.8, in_p, CROWN_SQUEEZE, "ball in over the cup", t_pre_r),
        _wp(1.2, in_p, CROWN_SQUEEZE, "steady", t_pre_r),
        _wp(1.0, dn_p, CROWN_SQUEEZE, "let the ball down into the cup", t_pre_r),
    ]
    if FAIL == "holding":  # wrong on purpose: the hand never lets go of the hanger
        left.append(_wp(25.0, dn_p, CROWN_SQUEEZE, "keep holding", t_pre_r))
    else:
        left += [
            _wp(0.8, dn_p, -0.040, "let go of the hanger", t_pre_r),
            _wp(1.6, dn_p - np.array([0.10, 0.0, 0.0]), -0.040, "back off", t_pre_r),
            _wp(1.0, dn_p - np.array([0.10, -0.08, 0.0]), -0.040, "back off up", t_pre_r),
            park("left", 2.0), park("left", 3.0),
        ]
    t_let = sum(w.duration for w in left[: len(left) - 3]) if FAIL != "holding" else sum(w.duration for w in left)
    t_r = sum(w.duration for w in right)
    if FAIL != "nohold":
        right.append(_wp(max(0.5, t_let - t_r), h_p, HOLD_SQUEEZE, "hold the ring", hr))
        right += [
            _wp(1.5, h_p, -0.007, "let go of the ring", hr),
            _wp(1.4, h_p - 0.06 * ha, -0.028, "back off", hr),
            _wp(1.0, h_p - 0.06 * ha + np.array([-0.05, 0.0, 0.06]), -0.028, "back off up", hr),
            park("right", 2.0), park("right", 3.0),
        ]
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    print(f"hangerdemo: ring B y {rb_p[1]:.4f} yaw {math.degrees(math.atan2(-rb_r[0, 1], rb_r[1, 1])):.1f} deg, "
          f"hanger B ({hb_p[0]:.3f}, {hb_p[1]:.3f}), plan {max(tl, tr):.1f} s", flush=True)
    return {"left": left, "right": right}


def _slerp_q(r0: np.ndarray, r1: np.ndarray, s: float) -> tuple:
    from yam_arena import ik

    return _quat(ik.slerp_matrix(r0, r1, s))


HUNG_MOVES = ("ball in over the cup", "steady", "let the ball down into the cup", "let go of the hanger", "keep holding",
              "back off", "back off up")


def regrip_waypoint(w, r_g, p_g):
    """A left-hand waypoint planned for the nominal pinch, re-planned for the pinch as it really is (the hanger's pose in
    the hand's frame: rotation r_g, ball position p_g), so the hanger itself still goes where the plan wanted it,
    hanging upright. Other waypoints: None (unchanged)."""
    if w.label not in HUNG_MOVES:
        return None
    off_p, _ = _grasp_offset()
    ball = np.asarray(w.pos, float) - off_p  # planned with the hanger upright: TCP = ball + offset
    r_new = np.asarray(r_g, float).T
    return dataclasses.replace(w, pos=tuple((ball - r_new @ np.asarray(p_g, float)).tolist()), yaw=r_new)


def aim_correction(st, stage: str = "pre") -> np.ndarray:
    """How far hanger B's ball is from where the plan wants it (world, m): "pre" beside the cup, "in" over it."""
    out = PRE_Y if stage == "pre" else 0.0
    want = np.asarray(st["ring_b"].pos, float) + st["ring_b"].rotation() @ (SEAT + np.array([0.0, out, OVER_Z]))
    return want - np.asarray(st["hanger_b"].pos, float)


def cup_shift(st) -> np.ndarray:
    """Where ring B's cup really is (from the ring's pose), minus where the plan put it (the ring hanging square at its
    start place), world, m."""
    now = np.asarray(st["ring_b"].pos, float) + st["ring_b"].rotation() @ SEAT
    return now - (np.asarray(_EP.get("plan_ring", st["ring_b"].pos), float) + SEAT)


import hangerdemo  # noqa: E402  (the demo's control hooks for yam_scripted)

hangerdemo.install()
hangerdemo_task = True
