"""On the station table, in front of the left arm: an aluminium bearing housing, 40 mm deep, 90 mm wide and 90 mm
tall, standing with its round through-bore horizontal and open toward the robot. A telescoping gauge has been left in
the bore, lightly locked: a T-shaped tool with an orange knurled handle and an end knob, whose yellow head carries two
spring-loaded plungers with rounded tips, one up and one down. The plungers' outward springs are weaker than the
lock, which holds them where they are; pushing hard on a tip slides its plunger in. The gauge's handle points out of
the bore and slopes down, resting near its end on a small yellow V-rest (a holding fixture), and its tips touch the
top and bottom of the bore. To the robot's right of the housing stands an empty black two-post stand with slots, for
a gauge lying level. Two more housings with different bores stand further right.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402  (before lots, so parts' reset hooks run before the lots' moves)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

ORANGE, YELLOW = (1.0, 0.45, 0.02), (1.0, 0.85, 0.05)  # the gauge's painted grip and head; the V-rest
ALU, BORE_FACE, STEEL, KNURL, TIP, BLACK = ((0.72, 0.74, 0.77), (0.86, 0.87, 0.89), (0.80, 0.81, 0.83),
                                             (0.45, 0.46, 0.48), (0.95, 0.80, 0.25), (0.08, 0.08, 0.09))

# -- housings: 40 (x, along the bore) x 90 (y) x 90 (z) mm, bore axis 55 mm above the base ----------------------
H_LEN, H_HALF_Y, AXIS_H, H_TOP = 0.040, 0.045, 0.055, 0.035  # 90 mm tall, top 35 mm above the axis
N_STAVE = 72  # the bore: 72 flat, 5-degree faces (across flats = the bore, across corners 0.1% more)
BORES = {"A": 0.040, "B": 0.044, "C": 0.048}


def _housing_shapes(d: float):
    r = d / 2
    sq = r + 0.004  # the square opening the four outer blocks leave, filled by the bore's staves
    out = [
        parts.box((H_LEN, H_HALF_Y - sq, H_TOP + AXIS_H), at=(0, s * (sq + H_HALF_Y) / 2, (H_TOP - AXIS_H) / 2),
                  color=ALU) for s in (1, -1)
    ] + [
        parts.box((H_LEN, 2 * sq, H_TOP - sq), at=(0, 0, (H_TOP + sq) / 2), color=ALU),
        parts.box((H_LEN, 2 * sq, AXIS_H - sq), at=(0, 0, -(AXIS_H + sq) / 2), color=ALU),
    ]
    for k in range(N_STAVE):
        a = 2 * math.pi * k / N_STAVE  # in the y-z plane; k = 18 is the top of the bore
        reach = sq / max(abs(math.cos(a)), abs(math.sin(a))) + 0.002
        t = reach - r
        w = 2 * reach * math.tan(math.pi / N_STAVE) * 1.1
        b = a - math.pi / 2  # turn about x: the box's +z (its thickness) points out along a
        out.append(parts.box((H_LEN, w, t), at=(0, (r + t / 2) * math.cos(a), (r + t / 2) * math.sin(a)),
                             color=BORE_FACE, quat=(math.sin(b / 2), 0.0, 0.0, math.cos(b / 2))))
    return out


def _housing_mass(d: float) -> float:
    vol = H_LEN * 2 * H_HALF_Y * (H_TOP + AXIS_H) - H_LEN * math.pi * (d / 2) ** 2
    return round(2700 * vol, 3)  # aluminium


WORK = (0.355, 0.200, TOP + AXIS_H)  # housing centre: its front face at x 0.335, bore axis 55 mm up
SPARES = ((0.380, -0.040, TOP + AXIS_H), (0.380, -0.160, TOP + AXIS_H))
HOUSINGS = {
    name: parts.Part(f"housing_{round(d * 1000)}", pos=p, shapes=_housing_shapes(d), mass=_housing_mass(d),
                     friction=(0.6, 0.5))
    for name, d, p in (("A", BORES["A"], WORK), ("B", BORES["B"], SPARES[0]), ("C", BORES["C"], SPARES[1]))
}

# -- the gauge (32-54 mm range). Its frame: origin at the head's centre, the handle along -x, the plungers along z --
R_TIP = 0.0025  # rounded tips, the plunger's own radius
SPAN_MAX, SPAN_MIN = 0.054, 0.032
# At the start the handle slopes down out of the bore until the rim of its end knob rests on the table:
# 127 mm (head centre to the knob's end) x sin + 6 mm (knob radius) x cos = the bore axis height.
_s = 0.0
for _ in range(50):
    _s = (0.055 - 0.003 - 0.006 * math.sqrt(1 - _s * _s)) / 0.127  # knob rim 3 mm above the table
START_UP = -math.asin(_s)  # about -21.4 degrees
SPRING = 120.0  # N/m: each plunger's outward spring, 0.4-1.3 N (the lock, about 2 N, holds it)


def wedged_span(d: float, up: float = START_UP) -> float:
    """Tip-to-tip span of a gauge tilted `up` whose tips just touch both walls of a bore `d` across."""
    return (d - 2 * R_TIP) / math.cos(up) + 2 * R_TIP - 0.00002


SPAN_A = wedged_span(BORES["A"])  # the plungers' joint 0: lot A's start
C_A = SPAN_A / 2 - R_TIP  # tip centre from the head centre at joint 0
GRIP_X = -0.075  # where the hand takes the handle, along the gauge's x


ROD_L = 0.012  # plunger rod, tip centre to its inner end (the two rods never meet: 1.5 mm apart fully in)


def _plunger(name, sign):
    c = C_A * sign
    return parts.Slider(
        name,
        shapes=[parts.cylinder(R_TIP, ROD_L, at=(0, 0, sign * (C_A - ROD_L / 2)), color=YELLOW, axis="z"),
                parts.sphere(R_TIP, at=(0, 0, c), color=TIP)],
        axis=(0.0, 0.0, -float(sign)),  # pushed in: toward the head
        travel=C_A - (SPAN_MIN / 2 - R_TIP),
        extend=(SPAN_MAX / 2 - R_TIP) - C_A,
        mass=0.007,
        spring=SPRING,
        target=-((SPAN_MAX / 2 - R_TIP) - C_A),  # the spring pushes the plunger fully out
        damping=0.2,
    )


# The lock: in each end of the head a small steel pad, on its own slide across the plunger, is pressed against the
# plunger's side by a stiff spring (the lock knob's setting). Contact friction between pad and plunger holds the span.
PAD_GAP = 0.0001  # pad face to plunger at the start (sliding pieces must not overlap at the start)
PAD_PRESS = 300.0  # N/m
PAD_PRELOAD = 0.0135  # m: the spring's rest point lies 13.5 mm past the plunger, so the pad presses with about 4 N
PAD_Z = 0.014  # pad centre from the head centre, along the plunger


def _pad(name, sign):
    return parts.Slider(
        name,
        shapes=[parts.box((0.0015, 0.003, 0.003), at=(R_TIP + PAD_GAP + 0.00075, 0, sign * PAD_Z), color=STEEL)],
        axis=(-1.0, 0.0, 0.0),  # toward the plunger
        travel=0.002,
        mass=0.008,
        spring=PAD_PRESS,
        target=PAD_PRELOAD,
        damping=2.0,
    )


def _qy(a):
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


HEAD0 = (WORK[0], WORK[1], WORK[2])  # the head at the bore's centre, mid-depth
GAUGE = parts.Part(
    "gauge",
    pos=HEAD0,
    quat=_qy(START_UP),  # turned about y: the handle's outer end goes down
    shapes=[
        parts.cylinder(0.0045, 0.032, at=(0, 0, 0), color=YELLOW, axis="z"),  # head, 9 mm across
        parts.cylinder(0.003, 0.021, at=(-0.0145, 0, 0), color=ORANGE, axis="x"),  # neck
        parts.cylinder(0.00475, 0.090, at=(-0.070, 0, 0), color=ORANGE, axis="x"),  # knurled handle, 9.5 mm
        parts.cylinder(0.006, 0.012, at=(-0.121, 0, 0), color=ORANGE, axis="x"),  # lock knob
    ],
    joints=[_plunger("top", 1), _plunger("bottom", -1), _pad("lock_top", 1), _pad("lock_bottom", -1)],
    mass=0.050,  # with the plungers and lock pads, 80 g
    friction=(0.5, 0.4),
)

# -- the stand: two slotted posts on a base, the gauge level along x with its head past the front post -----------
STAND_AXIS = 0.045  # a stowed handle's axis above the table
STAND_HEAD = (0.300, 0.080, TOP + STAND_AXIS)  # where a stowed gauge's head sits
SLOT_W, SLOT_DEPTH, POST_X = 0.012, 0.007, 0.010
POSTS = (-0.030, -0.100)  # post centres along the gauge's x
FLOOR_Z = STAND_AXIS - 0.00475  # slot floor above the table: a stowed handle's axis is 45 mm up


def _stand_shapes():
    out = [parts.box((0.110, 0.034, 0.006), at=(-0.061, 0, 0.003), color=BLACK)]
    for px in POSTS:
        out.append(parts.box((POST_X, 0.034, FLOOR_Z - 0.006), at=(px, 0, (FLOOR_Z + 0.006) / 2), color=BLACK))
        for s in (1, -1):
            out.append(parts.box((POST_X, 0.011, SLOT_DEPTH),
                                 at=(px, s * (SLOT_W / 2 + 0.0055), FLOOR_Z + SLOT_DEPTH / 2), color=BLACK))
    return out


STAND = parts.Part("stand", pos=(STAND_HEAD[0], STAND_HEAD[1], TOP), shapes=_stand_shapes())

# -- the V-rest: a small fixed V-block that carries the handle near its end at the start, so the gauge sits tilted
# in the bore with its weight on the rest, not hanging on its tips.
REST_AT = -0.105  # along the gauge's x: where the handle crosses the rest


def _rest():
    r_h, up = 0.00475, START_UP
    c = np.asarray(HEAD0) + parts._quat_to_matrix(_qy(up)) @ np.array([REST_AT, 0.0, 0.0])  # handle axis there
    # A cylinder tilted `up` lies in a 90-degree V with its axis this far above the V's bottom line:
    lift = math.sqrt(2) * r_h * math.sqrt((1 + 1 / math.cos(up) ** 2) / 2)
    # the V's bottom line above the table; the V is 4 mm long, so drop it by the handle's fall over 2 mm, plus 50 um
    z0 = c[2] - TOP - lift - 0.002 * math.tan(-up) - 0.00005
    shapes = [parts.box((0.004, 0.024, z0 - 0.002), at=(0, 0, (z0 - 0.002) / 2), color=YELLOW)]
    for s in (1, -1):
        b = s * math.pi / 4
        shapes.append(parts.box((0.004, 0.020, 0.004), at=(0, s * (0.00707 + 0.001414), z0 + 0.00707 - 0.001414),
                                color=YELLOW, quat=(math.sin(b / 2), 0.0, 0.0, math.cos(b / 2))))
    return parts.Part("v_rest", pos=(float(c[0]), float(c[1]), TOP), shapes=shapes)


REST = _rest()

OBJECTS = [GAUGE, HOUSINGS["A"], HOUSINGS["B"], HOUSINGS["C"], STAND, REST]
_NAMES = {k: h.name for k, h in HOUSINGS.items()}
SHORT = 0.0001  # each plunger is written 0.1 mm short of the walls; its spring pushes it out to touch


def _release_then_lock(emb) -> None:
    """The start, as an inspector leaves it: lock released, the springs push the plungers out against the bore walls,
    then the lock is set lightly."""
    base = emb._base
    pads = [j.name for j in GAUGE.joints if j.name.startswith("lock")]
    for name in pads:
        parts._set_target(base, "gauge", name, 0.0)
    emb._env.step(emb._ready)  # the pads let go
    lots.write_joints(emb)  # the gauge as it was left, tips just short of the walls
    for j in GAUGE.joints:
        if j.name not in pads:
            parts._set_target(base, "gauge", j.name, j.target)  # the plunger springs push out
    for _ in range(3):
        emb._env.step(emb._ready)
    for j in GAUGE.joints:
        if j.name in pads:
            parts._set_target(base, "gauge", j.name, j.target)


lots.AFTER_MOVES.append(_release_then_lock)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", bore=BORES["A"], place={_NAMES["A"]: WORK, _NAMES["B"]: SPARES[0], _NAMES["C"]: SPARES[1]},
             joints={"gauge": {"top": SHORT, "bottom": SHORT}}),
    *(lots.Lot(k, bore=BORES[k], place={_NAMES[k]: WORK, _NAMES[o1]: SPARES[0], _NAMES[o2]: SPARES[1]},
               joints={"gauge": {"top": (SPAN_A - wedged_span(BORES[k])) / 2 + SHORT,
                                 "bottom": (SPAN_A - wedged_span(BORES[k])) / 2 + SHORT}})
      for k, o1, o2 in (("B", "C", "A"), ("C", "A", "B"))),
)

# -- scoring -----------------------------------------------------------------------------------------------------
TOL = 0.00010  # span within 0.10 mm of the bore
CLEAR = 0.005  # no finger or gripper within 5 mm of the gauge
TIP_CLEAR = 0.001  # tips 1 mm or more from everything: the lock alone holds the span
EPS = 1e-6
ON_STAND_Z = 0.001  # handle axis within 1 mm above its resting height over both posts
LEVEL_DEG = 10.0


def span(gauge) -> float:
    """Tip to tip (m), from the two plunger positions."""
    return SPAN_A - gauge.joints.get("top", 0.0) - gauge.joints.get("bottom", 0.0)


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _tips(gauge):
    """World centres of the two tip spheres."""
    out = []
    for name in ("top", "bottom"):
        if name not in gauge.links:
            return []
        p, q = gauge.links[name]
        sign = 1 if name == "top" else -1
        out.append(np.asarray(p) + _rot(q) @ (np.array([0, 0, sign * C_A]) - GAUGE.joints[0 if sign > 0 else 1].origin))
    return out


def _gauge_boxes(gauge):
    poses = {"base": (np.asarray(gauge.pos), _rot(gauge.quat_xyzw))}
    for name, (p, q) in gauge.links.items():
        poses[name] = (np.asarray(p), _rot(q))
    return GAUGE.world_boxes(poses)


def _other_boxes(objects):
    out = []
    for o in OBJECTS:
        if o.name == "gauge" or o.name not in objects:
            continue
        st = objects[o.name]
        out += [(c, r, s) for _, c, r, s in o.world_boxes({"base": (np.asarray(st.pos), _rot(st.quat_xyzw))})]
    return out


def _box_dist(p, c, r, s) -> float:
    local = (np.asarray(p) - c) @ r
    q = np.abs(local) - np.asarray(s) / 2
    return float(np.linalg.norm(np.maximum(q, 0.0)) + min(q.max(), 0.0))


def _tips_free(objects) -> bool:
    tips = _tips(objects["gauge"])
    if len(tips) != 2:
        return False
    boxes = _other_boxes(objects)
    for t in tips:
        if t[2] - R_TIP < TOP + TIP_CLEAR - EPS:
            return False
        if any(_box_dist(t, c, r, s) - R_TIP < TIP_CLEAR - EPS for c, r, s in boxes):
            return False
    return True


def _out_of_housings(objects) -> bool:
    g = objects["gauge"]
    for h in HOUSINGS.values():
        hs = objects[h.name]
        if np.all(g.low < hs.high + EPS) and np.all(g.high > hs.low - EPS):
            return False
    return True


def _on_stand(objects) -> bool:
    g = objects["gauge"]
    r = _rot(g.quat_xyzw)
    axis = r @ np.array([1.0, 0.0, 0.0])  # from the handle toward the head; either way round on the stand
    if math.degrees(math.acos(min(1.0, abs(axis[0])))) > LEVEL_DEG + EPS:
        return False
    sx, sy = STAND_HEAD[0], STAND_HEAD[1]
    for px in POSTS:
        # the handle's axis where it crosses this post
        t = (sx + px - g.pos[0]) / axis[0]
        p = np.asarray(g.pos) + t * axis
        if not (-0.115 - EPS <= t <= -0.025 + EPS):  # the knurled handle is over the post
            return False
        if abs(p[1] - sy) > SLOT_W / 2 - 0.00475 + 0.0005 + EPS:
            return False
        if not (TOP + STAND_AXIS - 0.0005 - EPS <= p[2] <= TOP + STAND_AXIS + ON_STAND_Z + EPS):
            return False
    return True


# Released: an on_step callback measures, every step, how close both arms' fingers and grippers come to the gauge.
# Until it has run (offline, or if the arms cannot be read) the gauge counts as held: no credit (fail closed). The
# record lives on the shared parts module, so offline tests that import this file a second time see the same one.
_grip = parts.__dict__.setdefault("hand_clear_of_gauge", {"clear": False})


@parts.on_reset
def _reset_grip(sim) -> None:
    _grip["clear"] = False
    # Springs: plungers pushed out, lock pads pressed on (the first episode starts with the drive targets at 0).
    for j in GAUGE.joints:
        sim.set_target("gauge", j.name, j.target)


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    _grip["clear"] = False
    try:
        base = sim.env.unwrapped
        origin = parts._np(base.scene.env_origins[0])
        g = sim.states()["gauge"]
        hulls = []
        for arm in ("left_robot", "right_robot"):
            asset = base.scene[arm]
            poses = parts._np(asset.data.body_link_pose_w)[0]
            for i, n in enumerate(asset.body_names):
                if n in ("tip_left", "tip_right", "gripper"):
                    m = np.eye(4)
                    m[:3, :3] = _rot(poses[i, 3:])
                    m[:3, 3] = poses[i, :3] - origin
                    hulls += analysis.world_hulls({n: m})
        if not hulls:
            return
        _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in _gauge_boxes(g))
    except Exception as exc:  # unreadable arms: stay "held"
        print(f"[gauge] grip check failed: {exc}", file=sys.stderr)


def _measured(objects) -> bool:
    g = objects["gauge"]
    return abs(span(g) - lots.current().bore) <= TOL + EPS


def _left_alone(objects) -> bool:
    return bool(_grip["clear"] and objects["gauge"].resting() and _tips_free(objects)
                and _out_of_housings(objects))


def success(objects):
    """The gauge set to the bore, out of the housing, let go and still on its stand."""
    return bool(_measured(objects) and _left_alone(objects) and _on_stand(objects))


def score(objects):
    """1.0: set to the bore and stowed on the stand; 0.5: set to the bore and let go out of the housing elsewhere."""
    if not (_measured(objects) and _left_alone(objects)):
        return 0.0
    return 1.0 if _on_stand(objects) else 0.5


# -- demo (left arm; open loop from the published start, the same in every lot) ---------------------------------
TCP_UP = 0.006  # TCP above the handle's axis, along the hand: pad tips 7 mm below it
END_TILT = math.radians(8)  # the rock ends with the handle 8 degrees above level: 0.2-0.3 mm clear to draw out


def _hand(head, up, lean):
    """TCP for the head at `head`, the handle sloping up by `up` (rad), the hand leaned `lean` (rad, about y;
    negative swings the fingertips toward +x)."""
    r = _rot(_qy(up))
    grip = np.asarray(head) + r @ np.array([GRIP_X, 0.0, 0.0])
    hand_up = _rot(_qy(lean)) @ np.array([0.0, 0.0, 1.0])
    return tuple(float(v) for v in grip + TCP_UP * hand_up)


def _w(dur, head, up, finger, label):
    lean = up - END_TILT  # the hand turns with the gauge: straight down at the end of the rock
    deg = math.degrees(-lean)
    pos = _hand(head, up, lean)
    if abs(deg) < 1e-6:
        return tilt.Waypoint(dur, pos, finger, label)
    return tilt.Waypoint(dur, pos, finger, label, tilt=abs(deg), heading=0.0 if deg > 0 else math.pi)


def demo():
    h = np.asarray(HEAD0)
    above = h + np.array([0, 0, 0.06])
    face_out = h + np.array([-0.075, 0, 0])  # tips 40 mm clear of the face
    stand = np.asarray(STAND_HEAD)
    plan = [
        _w(2.0, above, START_UP, FINGER_OPEN, "over the handle"),
        _w(1.5, h, START_UP, FINGER_OPEN, "down to the handle"),
        _w(1.0, h, START_UP, FINGER_CLOSED, "grip"),
    ]
    n = 8
    for i in range(1, n + 1):
        up = START_UP + (END_TILT - START_UP) * i / n
        plan.append(_w(0.5, h, up, FINGER_CLOSED, f"rock {i}"))
    plan += [
        _w(0.5, h, END_TILT, FINGER_CLOSED, "past level"),
        _w(1.0, h, math.radians(-0.5), FINGER_CLOSED, "back to level"),
        _w(1.0, h, END_TILT, FINGER_CLOSED, "through level again"),
        _w(0.5, h + np.array([0, 0, 0.0015]), END_TILT, FINGER_CLOSED, "lift 1.5 mm (the loaded arm sags)"),
        _w(2.0, face_out + np.array([0, 0, 0.0015]), END_TILT, FINGER_CLOSED, "draw out"),
        _w(1.5, face_out + np.array([0, 0, 0.04]), 0.0, FINGER_CLOSED, "lift, level"),
        _w(2.0, stand + np.array([0, 0, 0.04]), 0.0, FINGER_CLOSED, "over the stand"),
        _w(1.5, stand + np.array([0, 0, 0.002]), 0.0, FINGER_CLOSED, "lower into the slots"),
        _w(0.5, stand + np.array([0, 0, 0.002]), 0.0, FINGER_CLOSED, "settle"),
        _w(1.5, stand + np.array([0, 0, 0.002]), 0.0, FINGER_OPEN, "let go"),
        _w(0.5, stand + np.array([0, 0, 0.002]), 0.0, FINGER_OPEN, "wait"),
        _w(2.0, stand + np.array([0, 0, 0.07]), 0.0, FINGER_OPEN, "straight up"),
        tilt.Waypoint(2.5, (0.20, 0.30, TOP + 0.20), FINGER_OPEN, "clear"),
    ]
    return {"left": plan}

