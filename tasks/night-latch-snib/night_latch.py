"""A rim night latch on a section of a front door: put the door on the latch.

A Yale P77 night latch is screwed to a 30 x 30 cm section of a 44 mm door that stands on the table, facing the robot,
its latch edge to the robot's left. The bolt is sprung and stands 13 mm out of the door's edge. Turning the knob draws
the bolt back against its spring; the snib (a 12 mm button on the case) can only slide down into its hold notch while
the bolt is drawn back, and then keeps it back. Pushed down with the bolt out, the same snib deadlocks the bolt out.

Everything works by contact, joints, springs and joint limits: a cam pin on the knob pushes the bolt's tail, the
bolt's spring pushes the knob back, and a stop on the snib enters one of two notches in the bolt's tail.

Frame: x forward from the midpoint between the arm bases, y to the robot's left, z up, metres. Sizes: latch_layout.py.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (multi-shape bodies and joints for yam_arena)
import latch_layout as L  # noqa: E402
from bodykit import Body, BoxShape, CylShape  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
CONTROL_HZ = 20.0

XF = L.CASE_FACE_X + L.CASE_D  # the door's face (x)
YK = L.KNOB_Y
Z0 = TOP + L.LATCH_H  # the knob's axis (z)
YE = YK + L.BACKSET  # the door's latch edge (y)
YS = YK + L.SNIB_FROM_KNOB  # the snib (y)

# ---- shapes, written in latch coordinates: u out of the door's face (toward the robot), y from the knob's axis toward
# the door's edge, z up from the knob's axis --------------------------------------------------------------------------


def _box(origin, u, y, z, color, collide: bool = True) -> BoxShape:
    lo = np.array([XF - u[1], YK + y[0], Z0 + z[0]]) - np.asarray(origin, float)
    hi = np.array([XF - u[0], YK + y[1], Z0 + z[1]]) - np.asarray(origin, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _xcyl(origin, u, y: float, z: float, r: float, color, collide: bool = True) -> CylShape:
    c = np.array([XF - (u[0] + u[1]) / 2, YK + y, Z0 + z]) - np.asarray(origin, float)
    return CylShape(tuple(c.tolist()), r, abs(u[1] - u[0]), color, collide=collide, axis="X")


DOOR_POS = (XF + L.DOOR_T / 2, YE - L.DOOR_W / 2, TOP + L.DOOR_H / 2)
KNOB_POS = (L.CASE_FACE_X, YK, Z0)  # on the knob's axis, at the case's face
BOLT_POS = (XF - L.BOLT_T / 2, YE, Z0)
SNIB_POS = (L.CASE_FACE_X, YS, Z0 + L.SNIB_FREE_Z)

EDGE = L.BACKSET  # the door's edge in latch y
NOTCH_OUT = L.SNIB_FROM_KNOB  # centre of the notch under the snib with the bolt out (deadlock)
NOTCH_BACK = NOTCH_OUT + L.THROW  # centre of the notch that comes under the snib with the bolt drawn back (hold)
RAIL_TOP = L.SNIB_FREE_Z - 0.001 - L.STOP_H - 0.0005  # the tail's top, 0.5 mm under the stop with the snib free
RAIL_FLOOR = RAIL_TOP - L.SNIB_TRAVEL - 0.0005  # the notches' floor
CAM0 = math.radians(L.CAM_START_DEG)
PIN_Y, PIN_Z = L.CAM_R * math.sin(CAM0), L.CAM_R * math.cos(CAM0)  # the cam pin at rest


def _door_shapes() -> list:
    o = DOOR_POS
    hy, hz = L.CASE_BODY_H / 2, L.CASE_H / 2
    far = EDGE - L.CASE_W  # the case's end away from the door's edge
    s = [
        BoxShape((0.0, 0.0, 0.0), (L.DOOR_T, L.DOOR_W, L.DOOR_H), L.WHITE),
        _box(o, (0.0, L.CASE_FLANGE_D), (far, EDGE), (-hz, hz), L.CASE_C),  # screw flanges
        _box(o, (0.0, L.CASE_D), (far + 0.003, EDGE), (-hy, hy), L.CASE_C),  # the raised case
        # the snib's slot (a dark mark; the button slides in it)
        _box(o, (L.CASE_D, L.CASE_D + 0.0003), (L.SNIB_FROM_KNOB - 0.003, L.SNIB_FROM_KNOB + 0.003),
             (L.SNIB_FREE_Z - L.SNIB_TRAVEL - 0.0025, L.SNIB_FREE_Z + 0.0025), L.DARK, collide=False),
    ]
    for zc in (-L.CASE_H / 2 + 0.006, L.CASE_H / 2 - 0.006):  # fixing screws (visual)
        s.append(_xcyl(o, (L.CASE_FLANGE_D, L.CASE_FLANGE_D + 0.0005), EDGE - 0.016, zc, 0.003, L.STEEL, collide=False))
    bx, by, bz = L.BRACKET
    for yc in (EDGE - 0.06, EDGE - L.DOOR_W + 0.06):  # two angle brackets on the far side hold the section to the table
        lo = np.array([XF + L.DOOR_T, YK + yc - by / 2, TOP]) - np.asarray(o, float)
        s.append(BoxShape(tuple((lo + np.array([bx, by, 0.003]) / 2).tolist()), (bx, by, 0.003), L.STEEL))
        s.append(BoxShape(tuple((lo + np.array([0.003, by, bz]) / 2).tolist()), (0.003, by, bz), L.STEEL))
    return s


def _knob_shapes() -> list:
    o = KNOB_POS
    u0 = L.CASE_D + L.KNOB_STEM_L
    u1 = u0 + L.KNOB_HEAD_T
    flat = (L.KNOB_L - L.KNOB_W) / 2  # half the straight part of the oval
    return [
        _xcyl(o, (L.CASE_D, u0), 0.0, 0.0, L.KNOB_STEM_R, L.CASE_C),
        _box(o, (u0, u1), (-L.KNOB_W / 2, L.KNOB_W / 2), (-flat, flat), L.CASE_C),
        _xcyl(o, (u0, u1), 0.0, flat, L.KNOB_W / 2, L.CASE_C),
        _xcyl(o, (u0, u1), 0.0, -flat, L.KNOB_W / 2, L.CASE_C),
        _xcyl(o, L.LAYER, PIN_Y, PIN_Z, L.CAM_PIN_R, L.STEEL),  # the cam pin, inside the case
    ]


def _bolt_shapes() -> list:
    o = BOLT_POS
    head_in = NOTCH_BACK + L.NOTCH_W / 2 + 0.0005  # the head's inner end is the hold notch's far wall
    tab_face = PIN_Y - L.CAM_PIN_R - L.CAM_PLAY
    return [
        _box(o, (0.0, L.BOLT_T), (head_in, EDGE + L.THROW), (-L.BOLT_H / 2, L.BOLT_H / 2), L.BRASS),  # the bolt's head
        _box(o, L.LAYER, (tab_face - 0.004, head_in), (RAIL_FLOOR - 0.002, RAIL_FLOOR), L.BRASS),  # the tail's spine
        _box(o, L.LAYER, (NOTCH_OUT - L.NOTCH_W / 2 - 0.0065, NOTCH_OUT - L.NOTCH_W / 2), (RAIL_FLOOR, RAIL_TOP), L.BRASS),
        _box(o, L.LAYER, (NOTCH_OUT + L.NOTCH_W / 2, NOTCH_BACK - L.NOTCH_W / 2), (RAIL_FLOOR, RAIL_TOP), L.BRASS),
        _box(o, L.LAYER, (tab_face - 0.004, tab_face), (RAIL_FLOOR, PIN_Z + L.CAM_PIN_R + 0.003), L.BRASS),  # the cam's tab
    ]


def _snib_shapes() -> list:
    o = SNIB_POS
    zb = L.SNIB_FREE_Z - 0.001  # the stop's top
    return [
        _xcyl(o, (L.CASE_D, L.CASE_D + L.SNIB_PROUD), L.SNIB_FROM_KNOB, L.SNIB_FREE_Z, L.SNIB_R, L.CASE_C),
        _box(o, L.LAYER, (L.SNIB_FROM_KNOB - L.STOP_W / 2, L.SNIB_FROM_KNOB + L.STOP_W / 2), (zb - L.STOP_H, zb), L.STEEL),
    ]


def _build() -> list:
    door = Body.build("door", pos=DOOR_POS, mass=8.0, shapes=_door_shapes(), color=L.WHITE, friction=(0.6, 0.5),
                      kinematic=True, solver_iterations=32)
    bolt = Body.build("bolt", pos=BOLT_POS, mass=L.BOLT_MASS, shapes=_bolt_shapes(), color=L.BRASS, friction=(0.2, 0.15),
                      solver_iterations=32, ignore=("door",))
    knob = Body.build("knob", pos=KNOB_POS, mass=L.KNOB_MASS, shapes=_knob_shapes(), color=L.CASE_C,
                      friction=(0.7, 0.6), solver_iterations=32, ignore=("door",))
    snib = Body.build("snib", pos=SNIB_POS, mass=L.SNIB_MASS, shapes=_snib_shapes(), color=L.CASE_C,
                      friction=(0.4, 0.3), solver_iterations=32, ignore=("door", "knob"))
    bodykit.slider("latch", "door", "bolt", anchor=BOLT_POS, axis="Y", limits=(-(L.THROW + L.BOLT_OVERTRAVEL), 0.0),
                   stiffness=L.BOLT_K, damping=3.0, target=L.BOLT_PRELOAD, max_force=60.0)
    bodykit.hinge("spindle", "door", "knob", anchor=KNOB_POS, axis="X", limits_deg=(0.0, L.KNOB_STOP_DEG),
                  spring=L.KNOB_SPRING, damping=L.KNOB_DAMPING)
    bodykit.slider("snib_slide", "door", "snib", anchor=SNIB_POS, axis="Z", limits=(-L.SNIB_TRAVEL, 0.0),
                   stiffness=L.SNIB_DETENT_K, damping=1.0, target=0.0, max_force=20.0)
    return [door, bolt, knob, snib]


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]


# ---- reading the latch from poses ---------------------------------------------------------------------------------------
def knob_deg(st) -> float:
    """The knob's turn from rest (degrees; its top away from the door's edge is positive)."""
    r = st["knob"].rotation()
    return math.degrees(math.atan2(r[2, 1], r[1, 1]))


def bolt_out_mm(st) -> float:
    """How far the bolt stands out of the door's edge (mm): what a ruler at the edge reads. 13 at rest."""
    return (L.THROW + float(st["bolt"].pos[1]) - BOLT_POS[1]) * 1000.0


def snib_down_mm(st) -> float:
    """How far the snib's button is below its free end (mm): 0 free, 6 at its hold end."""
    return (SNIB_POS[2] - float(st["snib"].pos[2])) * 1000.0


LATCH_BOX = (np.array([L.CASE_FACE_X - L.KNOB_STEM_L - L.KNOB_HEAD_T, YE - L.CASE_W, Z0 - L.CASE_H / 2]),
             np.array([XF, YE + L.THROW, Z0 + L.CASE_H / 2]))  # the case with its knob, snib and bolt


def finger_points(p, r, half: float) -> list:
    """Points on the two fingers of a gripper whose TCP is at ``p`` with rotation ``r`` and whose pads are each
    ``half`` m from the centre line: pad tips, pad middles and finger roots."""
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> dict:
    """Points on each gripper's fingers (world), from the arm's own joint readings."""
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = {}
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out[arm] = finger_points(p, r, half)
    return out


def clearance_mm(hands) -> float | None:
    """The smallest distance (mm) from either gripper's fingers to the latch. None if a gripper's position is missing
    or not a finite number (the scorer then counts the grippers as NOT clear)."""
    if not hands or set(hands) != {"left", "right"}:
        return None
    best = float("inf")
    for pts in hands.values():
        for p in pts:
            p = np.asarray(p, float)
            if not np.all(np.isfinite(p)):
                return None
            d = np.maximum(np.maximum(LATCH_BOX[0] - p, p - LATCH_BOX[1]), 0.0)
            best = min(best, float(np.linalg.norm(d)))
    return best * 1000.0 if math.isfinite(best) else None


# ---- episode state, the snib's detent, the score ----------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["LATCH_DEBUG"], "w") if os.environ.get("LATCH_DEBUG") else None
PROBE = os.environ.get("LATCH_PROBE", "")
NEED = int(round(L.HOLD_S * CONTROL_HZ))


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, st=None, hands=None, detent=0, on_latch=0, clear=0, report={}, knob_max=0.0)


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def state_now(st, hands) -> dict:
    out, down, clr = bolt_out_mm(st), snib_down_mm(st), clearance_mm(hands)
    back = out <= L.BACK_MAX_OUT * 1000.0
    held = down >= L.SNIB_HOLD_MIN * 1000.0
    return dict(bolt_out_mm=out, snib_down_mm=down, clearance_mm=clr, back=back, snib_hold=held,
                clear=clr is not None and clr >= L.CLEAR_M * 1000.0)


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    # the snib's detent: a spring seats the button at whichever end of its slot is nearer
    want = 1 if snib_down_mm(st) > L.SNIB_TRAVEL * 500.0 else 0
    if want != _EP["detent"]:
        bodykit.set_slider_drive("snib_slide", target=-L.SNIB_TRAVEL if want else 0.0)
        _EP["detent"] = want
    try:
        _EP["hands"] = _hand_points(env.unwrapped)
    except Exception as err:  # noqa: BLE001
        _EP["hands"] = None
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    now = state_now(st, _EP["hands"])
    on = now["back"] and now["snib_hold"]
    _EP["on_latch"] = _EP["on_latch"] + 1 if on else 0
    _EP["clear"] = _EP["clear"] + 1 if (on and now["clear"]) else 0
    _EP["knob_max"] = max(_EP["knob_max"], knob_deg(st))
    _EP["now"] = now
    _EP["step"] += 1
    if PROBE:
        _probe(env)
    if _DBG is not None:
        _debug(st, now)


def report(objects=None) -> dict:
    e = _EP
    now = dict(e.get("now") or {})
    if e["clear"] >= NEED:
        s = 1.0
    elif e["on_latch"] >= NEED:
        s = 0.5
    else:
        s = 0.0
    now.update(score=s, on_latch_steps=e["on_latch"], clear_steps=e["clear"], knob_max_deg=e["knob_max"])
    return now


def score(objects) -> float:
    """1.0: for the last 1 s of the run the bolt stands at most 3 mm out of the door's edge, the snib's button is at
    least 4 mm down (its hold end) and both grippers are at least 5 cm from the latch. 0.5: the same bolt and snib,
    but a gripper is within 5 cm (or its position is unknown). 0: anything else."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def distance(objects) -> float:
    st = _EP.get("st")
    return 0.0 if st is None else max(0.0, bolt_out_mm(st) / 1000.0)


def _debug(st, now) -> None:
    c = now["clearance_mm"]
    _DBG.write(f"{_EP['step']:4d} knob={knob_deg(st):6.2f} bolt_out={now['bolt_out_mm']:6.2f} "
               f"snib_down={now['snib_down_mm']:5.2f} clear={'none' if c is None else format(c, '6.1f')} "
               f"on={_EP['on_latch']:3d} ok={_EP['clear']:3d} score={report()['score']}\n")
    _DBG.flush()


# ---- M1 probe: the mechanism's truth table, driven by a torque on the knob and a force on the snib (no arms) -----------
PROBE_TORQUE, PROBE_PUSH = 0.30, 3.0  # N m on the knob, N on the snib


def _wrench(env, name: str, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)) -> None:
    import torch

    base = env.unwrapped
    obj = base.scene[name]
    obj.permanent_wrench_composer.reset()
    f = torch.tensor(force, dtype=torch.float32, device=base.device).reshape(1, 1, 3)
    t = torch.tensor(torque, dtype=torch.float32, device=base.device).reshape(1, 1, 3)
    if any(force) or any(torque):
        obj.permanent_wrench_composer.add_forces_and_torques(f, t, body_ids=[0], is_global=True)


# (from step, to step, knob torque in N m, snib force: -1 down, +1 up, 0 none, label)
FULL, PART = PROBE_TORQUE, 0.085
PROBE_PLAN = (
    (10, 50, FULL, 0, "A turn the knob only"),
    (50, 90, 0.0, 0, "B let the knob go"),
    (90, 120, 0.0, -1, "C snib down with the bolt out"),
    (120, 160, FULL, 0, "D turn the knob while deadlocked"),
    (160, 190, 0.0, 1, "E snib back up"),
    (190, 215, PART, 0, "F knob held part way"),
    (215, 245, PART, -1, "G knob held part way, snib pushed"),
    (245, 265, FULL, 0, "H knob turned all the way and held"),
    (265, 295, FULL, -1, "I knob held, snib down"),
    (295, 345, 0.0, 0, "J let everything go"),
    (345, 375, 0.0, 1, "K snib pushed up, knob not touched"),
    (375, 400, 0.0, 0, "L let go"),
)


def _probe(env) -> None:
    k, st = _EP["step"], _EP["st"]
    for a, b, torque, sn, label in PROBE_PLAN:
        if a <= k < b:
            _wrench(env, "knob", torque=(torque, 0.0, 0.0))
            _wrench(env, "snib", force=(0.0, 0.0, sn * PROBE_PUSH))
        if k == b - 1:
            print(f"LATCH PROBE {label}: knob {knob_deg(st):6.2f} deg, bolt out {bolt_out_mm(st):6.2f} mm, "
                  f"snib down {snib_down_mm(st):5.2f} mm", flush=True)


# ---- the scripted demo: a fixed plan for the fixed layout; it moves the parts only through the grippers --------------
VARIANT = os.environ.get("DEMO_VARIANT", "")  # "" full demo; onehand; snibfirst; nothing
TIP = 0.013  # pad tips past the TCP
TURN_DEG = float(os.environ.get("LATCH_TURN_DEG", "66"))
TILT_DEG = float(os.environ.get("LATCH_TILT_DEG", "25"))  # the left gripper leans back from straight down
GRIP = -(L.KNOB_W / 2 - float(os.environ.get("LATCH_SQUEEZE_MM", "6")) / 1000.0)  # finger target: 6 mm past contact
ROUND = -(L.KNOB_W / 2 + 0.010)  # fingers open 10 mm each side of the knob
SNIB_X = float(os.environ.get("LATCH_SNIB_X_MM", "10")) / 1000.0  # the left fingertip's centre this far off the case
OVER = float(os.environ.get("LATCH_OVER_MM", "1.5")) / 1000.0  # the push goes this far past the snib's hold end


def _rot(z, x) -> np.ndarray:
    """A tool rotation: approach along ``z``, fingers closing along ``x``."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _rx(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ready_pose(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def _knob_part(hold_s: float) -> list:
    """The right hand: grip the knob across its oval, turn it, hold, let go, back off."""
    r0 = _rot((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
    r1 = _rx(TURN_DEG) @ r0
    grab = np.array([L.CASE_FACE_X - L.KNOB_STEM_L - L.KNOB_HEAD_T - 0.002 + 0.0, YK, Z0])  # pad tips 1 mm short of the neck
    grab[0] = L.CASE_FACE_X - L.KNOB_STEM_L - 0.001 - TIP
    pre = grab - np.array([0.08, 0.0, 0.0])
    g = lambda p: tuple(np.asarray(p, float).tolist())  # noqa: E731
    return [
        Waypoint(2.5, g(pre), ROUND, "line up with the knob", r0),
        Waypoint(1.5, g(grab), ROUND, "fingers round the knob", r0),
        Waypoint(1.0, g(grab), GRIP, "grip the knob", r0),
        Waypoint(2.5, g(grab), GRIP, "turn the knob", r1),
        Waypoint(hold_s, g(grab), GRIP, "hold the knob turned", r1),
        Waypoint(0.8, g(grab), ROUND, "let go of the knob", r1),
        Waypoint(1.2, g(pre), ROUND, "back off", r1),
        park("right", 2.0, label="back to the ready pose"),
    ]


def _snib_part() -> list:
    """The left hand: closed fingers, leaning back from straight down, push the snib's button down with the fingertip."""
    a = math.radians(TILT_DEG)
    zt = np.array([math.sin(a), 0.0, -math.cos(a)])
    r = _rot(zt, (0.0, -1.0, 0.0))
    top_free = Z0 + L.SNIB_FREE_Z + L.SNIB_R  # the button's top, free
    # the slanted fingertip face meets the button's outer top edge; that edge is (SNIB_X - SNIB_PROUD) toward the door
    # from the fingertip's centre, where the face is this much higher than the centre:
    rise = (SNIB_X - L.SNIB_PROUD) * math.tan(a)

    def tcp(z_edge: float):  # TCP that puts the face on the button's outer top edge at height z_edge
        tip = np.array([L.CASE_FACE_X - SNIB_X, YS, z_edge - rise])
        return tuple((tip - TIP * zt).tolist())

    return [
        Waypoint(2.5, tcp(top_free + 0.070), FINGER_CLOSED, "over the snib", r),
        Waypoint(1.5, tcp(top_free + 0.012), FINGER_CLOSED, "down to the snib", r),
        Waypoint(1.2, tcp(top_free - L.SNIB_TRAVEL - OVER), FINGER_CLOSED, "push the snib down", r),
        Waypoint(0.6, tcp(top_free - L.SNIB_TRAVEL - OVER), FINGER_CLOSED, "hold the push", r),
        Waypoint(1.2, tcp(top_free + 0.070), FINGER_CLOSED, "lift off the snib", r),
        park("left", 2.0, FINGER_CLOSED, label="back to the ready pose"),
    ]


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


def demo():
    if VARIANT == "nothing" or PROBE:
        return {arm: [park(arm, 21.0 if PROBE else 1.0, label="wait")] for arm in ("left", "right")}
    snib = _snib_part()
    push_done = _dur(snib[:4])  # the snib is down and held
    if VARIANT == "onehand":  # the knob is turned, held 3 s and let go; the other hand never comes
        right = _knob_part(3.0)
        return {"left": [park("left", _dur(right), label="wait")], "right": right}
    if VARIANT == "snibfirst":  # wrong order: the snib is pushed with the bolt out, then the knob is tried
        right = _knob_part(1.5)
        return {"left": snib, "right": [park("right", _dur(snib) - 1.0, label="wait")] + right}
    # the full demo: the right hand turns and holds the knob; the left pushes the snib while it is held
    t_turned = 2.5 + 1.5 + 1.0 + 2.5  # the knob is turned at this time
    right = _knob_part(push_done + 0.3)  # holds until the snib is down, then lets go
    left = [park("left", t_turned, FINGER_CLOSED, label="wait")] + snib
    return {"left": left, "right": right}


import latchdemo  # noqa: E402  (steady hands for yam_scripted: closes the soft joints' gap to the planned pose)

latchdemo.install()
latchdemo_task = True
