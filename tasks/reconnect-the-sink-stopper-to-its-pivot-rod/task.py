"""On the station table, a little to the left of the middle: a chrome bathroom-sink pop-up drain body standing upright
in a grey bench stand, its round flange on top and its pivot port, a short round spout with a white seat washer,
pointing at the robot. The pop-up stopper sits closed in the drain, its chrome head on the flange. Nearer the robot
and to the right, the steel pivot rod with its white pivot ball lies front to back in two notched rests, its inner
end toward the drain.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

CHROME, BRASS_DARK, WHITE = (0.80, 0.81, 0.83), (0.45, 0.42, 0.36), (0.93, 0.93, 0.90)
STEEL, GREY, DARK = (0.60, 0.60, 0.62), (0.55, 0.57, 0.60), (0.22, 0.23, 0.25)
STEM_C = (0.20, 0.20, 0.22)


def _qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# -- drain body (fixed). Built in its own frame (axis vertical through the origin, the pivot port pointing along -y),
# then placed at (BX, BY) and turned so the port points along PORT_DIR (world, rad; 0 is away from the robot).
BX, BY = 0.33, 0.05
PORT_DIR = math.pi  # the port points at the robot
YAW_D = PORT_DIR + math.pi / 2  # the drain's turn
U = np.array([math.cos(PORT_DIR), math.sin(PORT_DIR), 0.0])  # outward along the port
RD = np.array([[math.cos(YAW_D), -math.sin(YAW_D), 0.0], [math.sin(YAW_D), math.cos(YAW_D), 0.0], [0.0, 0.0, 1.0]])
C0 = np.array([BX, BY, 0.0])
R_IN, R_OUT = 0.016, 0.020  # 32 mm bore, 40 mm outside
BASE_T = 0.008  # stand's base plate
HP = TOP + 0.054  # pivot port axis height
HF = TOP + 0.110  # flange top: the stopper's head rests here when closed
FLANGE_T, FLANGE_R = 0.004, 0.030
WIN = 0.006  # the opening through the wall is 12 mm tall
STAVES = 24
SEAT_IN, SEAT_OUT = R_OUT, R_OUT + 0.003  # seat plate (washer) from 20 to 23 mm off the axis
SEAT_HOLE = 0.0045  # 9 mm round hole through the seat
BORE = 0.008  # port bore 16 mm round
PORT_END = 0.040  # the port spout ends 40 mm from the drain axis


def _ring(rm, rc, n, yc, length, color, z=None):
    """n cylinders of radius rc along y, centred rm from a line along y at height z: a round hole of radius rm - rc."""
    z = HP if z is None else z
    return [parts.cylinder(rc, length, at=(rm * math.cos(2 * math.pi * k / n), yc, z + rm * math.sin(2 * math.pi * k / n)),
                           color=color, axis="y") for k in range(n)]


def _body_shapes():
    s = []
    z0, z1 = TOP + BASE_T, HF - FLANGE_T
    w = 2 * R_OUT * math.tan(math.pi / STAVES)
    rc = (R_IN + R_OUT) / 2
    for i in range(STAVES):
        a = -math.pi / 2 + i * 2 * math.pi / STAVES
        at_xy = (0.0 + rc * math.cos(a), 0.0 + rc * math.sin(a))
        cut = i in (0, 1, STAVES - 1)  # the three staves facing the port are cut by the opening
        spans = [(z0, HP - WIN), (HP + WIN, z1)] if cut else [(z0, z1)]
        for lo, hi in spans:
            s.append(parts.box((R_OUT - R_IN, w, hi - lo), at=(at_xy[0], at_xy[1], (lo + hi) / 2), color=CHROME,
                               quat=_qz(a)))
    fw = 2 * FLANGE_R * math.tan(math.pi / 12)
    for i in range(12):  # flange ring, 60 mm across, its hole the bore
        a = i * 2 * math.pi / 12
        rc_f = (R_IN + FLANGE_R) / 2
        s.append(parts.box((FLANGE_R - R_IN, fw, FLANGE_T), at=(0.0 + rc_f * math.cos(a), 0.0 + rc_f * math.sin(a),
                                                                  HF - FLANGE_T / 2), color=CHROME, quat=_qz(a)))
    # The kit has only solid cylinders, so each round hole along the port is a ring of overlapping cylinders
    # ("lobes") whose inner faces make the bore. Seat (washer): a 9 mm hole, 3 mm thick, 20 to 23 mm off the axis.
    yc = -(SEAT_IN + SEAT_OUT) / 2
    s += _ring(SEAT_HOLE + 0.002, 0.002, 14, yc, SEAT_OUT - SEAT_IN, WHITE)
    # Port spout: a 16 mm round bore from the seat out to 40 mm off the axis.
    s += _ring(BORE + 0.0022, 0.0022, 16, -(SEAT_OUT + PORT_END) / 2, PORT_END - SEAT_OUT, CHROME)
    # bench stand: base plate and a square clamp collar round the lower tube
    s.append(parts.box((0.10, 0.10, BASE_T), at=(0.0, 0.0, TOP + BASE_T / 2), color=DARK))
    ch, co = 0.022, 0.028
    zc = TOP + BASE_T + ch / 2
    s.append(parts.box((2 * co, co - R_OUT, ch), at=(0.0, 0.0 + (co + R_OUT) / 2, zc), color=GREY))
    s.append(parts.box((2 * co, co - R_OUT, ch), at=(0.0, 0.0 - (co + R_OUT) / 2, zc), color=GREY))
    s.append(parts.box((co - R_OUT, 2 * R_OUT, ch), at=(0.0 + (co + R_OUT) / 2, 0.0, zc), color=GREY))
    s.append(parts.box((co - R_OUT, 2 * R_OUT, ch), at=(0.0 - (co + R_OUT) / 2, 0.0, zc), color=GREY))
    return s


DRAIN = parts.Part("drain_body", pos=(BX, BY, 0.0), yaw=YAW_D, shapes=_body_shapes())

# -- pop-up stopper (loose). Part frame: origin at the centre of the head's underside, z up. In the frame the eye's
# hole runs along y; the stopper starts turned a quarter turn from facing the port, so the hole runs left to right.
HEAD_R, HEAD_T = 0.019, 0.009
KNOB_R, KNOB_H = 0.006, 0.012
FIN_SPAN, FIN_T, FIN_H = 0.0305, 0.002, 0.050
STEM_R = 0.0045
EYE_Z = -0.082  # eye centre below the head's underside: 26 mm below the port when closed
HOLE = 0.0045  # 9 mm round eye
TAB_W, TAB_T = 0.017, 0.005  # the eye's ring: 17 mm across, 5 mm thick along the hole
TAB_BAR = (TAB_W - 2 * HOLE) / 2  # 4 mm of ring round the hole
STOPPER_MASS = 0.055
START_YAW = YAW_D + math.pi / 2  # a quarter turn from facing the port


def _stopper_shapes():
    top_of_tab = EYE_Z + HOLE + TAB_BAR
    stem_len = -top_of_tab
    return [
        parts.cylinder(HEAD_R, HEAD_T, at=(0.0, 0.0, HEAD_T / 2), color=CHROME, axis="z"),
        parts.cylinder(KNOB_R, KNOB_H, at=(0.0, 0.0, HEAD_T + KNOB_H / 2), color=CHROME, axis="z"),
        parts.box((FIN_SPAN, FIN_T, FIN_H), at=(0.0, 0.0, -FIN_H / 2), color=STEM_C),
        parts.box((FIN_T, FIN_SPAN, FIN_H), at=(0.0, 0.0, -FIN_H / 2), color=STEM_C),
        parts.cylinder(STEM_R, stem_len, at=(0.0, 0.0, -stem_len / 2), color=STEM_C, axis="z"),
        *_ring(HOLE + 0.002, 0.002, 14, 0.0, TAB_T, STEM_C, z=EYE_Z),  # the eye: a 9 mm round hole
    ]


STOPPER = parts.Part("stopper", pos=(BX, BY, HF), yaw=START_YAW, mass=STOPPER_MASS, shapes=_stopper_shapes())

# -- pivot rod (loose). Part frame: origin at the pivot ball's centre, the rod along +y toward its inner tip.
ROD_R, ROD_L = 0.0025, 0.250
TIP = 0.042  # inner tip 42 mm in from the ball's centre
BALL_R, BALL_L = 0.007, 0.010  # the pivot ball, as a round barrel 14 mm across and 10 mm long
ROD_MASS = 0.040
SEATED = C0 + U * (SEAT_OUT + BALL_L / 2) + np.array([0.0, 0.0, HP])  # ball centre when its face is on the seat
_SHIFT = np.array([0.0, -0.10, 0.0])  # the rod starts 10 cm to the right of the port line
_S0 = SEATED + U * 0.060 + _SHIFT
ROD_START = (float(_S0[0]), float(_S0[1]), TOP + 0.030)
ROD = parts.Part(
    "pivot_rod", pos=ROD_START, yaw=YAW_D, mass=ROD_MASS,
    shapes=[parts.cylinder(ROD_R, ROD_L, at=(0.0, TIP - ROD_L / 2, 0.0), color=STEEL, axis="y"),
            parts.cylinder(BALL_R, BALL_L, at=(0.0, 0.0, 0.0), color=WHITE, axis="y")],
)

# -- rod rests (fixed): two notched blocks, the notch 6 mm wide
REST_TOP = ROD_START[2] - ROD_R
REST_YS = (TIP - 0.012, -0.180)  # along the rod from the ball's centre


def _rest_shapes():
    s = []
    x = 0.0
    for y in REST_YS:
        s.append(parts.box((0.020, 0.012, REST_TOP - TOP), at=(x, y, (REST_TOP + TOP) / 2 - ROD_START[2]), color=DARK))
        for sx in (-1, 1):
            s.append(parts.box((0.007, 0.012, 0.003), at=(x + sx * 0.0065, y, REST_TOP + 0.0015 - ROD_START[2]), color=DARK))
    return s


RESTS = parts.Part("rod_rests", pos=ROD_START, yaw=YAW_D, shapes=_rest_shapes())

OBJECTS = [DRAIN, STOPPER, ROD, RESTS]

# -- scoring ---------------------------------------------------------------------------------------------------
# Everything scored is checked from outside the drain, the way a person would check the repair at a real sink, once
# both parts have been let go (no gripper within 5 mm) and still for 1 s:
# - readings: the rod comes out of the spout's 16 mm bore, and its outer end is at most 199 mm from the spout's end
#   face, measured along the rod with a rule (196 mm with the ball home on the washer, so 3 mm of play); the stopper's
#   head is centred within 4 mm on the flange, upright within 6 degrees, and at most 40 mm above it (a rule or depth
#   gauge from the basin side);
# - then a movement test shows that the rod is through the stopper's eye. A rod under, over, beside or short of the
#   eye fails it.
# The pivot nut and the lift linkage are left out (see the fidelity note), so nothing but its seat keeps the ball in
# the port, and the test pushes at the rod's outer end, where the linkage's strap would clamp on.
DEPTH_MAX = 0.199  # spout end face to the rod's outer end, along the rod (196 mm with the ball on the washer)
GUIDE_OFF, GUIDE_TILT, GUIDE_LIFT = 0.004, math.radians(6.0), 0.040
RELEASE_GAP = 0.005
SUCCESS_HOLD_S = 1.0
HOLD_STEPS = 20  # 1 s at 20 Hz

# The movement test, right after the readings:
# 1. lift (1 s): an upward push at the rod's outer end, rising steadily to LIFT_N over 0.5 s and held 0.5 s. The rod
#    turns about its ball, its inner end goes down, and a stopper hung on it through the eye goes down with it: it
#    must drop at least MIN_DROP.
# 2. back (2 s): the push eases off steadily to nothing over 1 s, lowering the rod's outer end back, then nothing is
#    applied for 1 s so the parts come to rest; the rod's inner end has lifted the stopper again: it must have risen
#    at least MIN_DROP.
# 3. pull (0.5 s): a steady upward pull of PULL_N (about twice the stopper's weight) on the stopper's knob. A rod
#    through the eye catches it: it may rise at most MAX_PULL_RISE. A stopper resting on a rod under its eye, or on
#    nothing, lifts straight off; the pull stops as soon as it passes that limit.
# If a gripper comes within 5 mm of either part during the test or after it, the check starts again from the 1 s of
# stillness. The test's own pushes can shift the parts, so the readings are those taken just before it.
LIFT_N, PULL_N = 0.20, 1.00
LIFT_STEPS, BACK_STEPS, SETTLE_STEPS, PULL_STEPS = 20, 20, 20, 10
MIN_DROP, MAX_PULL_RISE = 0.003, 0.010


def _rod_axis(st):
    r = _rot(st.quat_xyzw)
    return np.asarray(st.pos, float), r[:, 1]


def _eye_frame(st):
    r = _rot(st.quat_xyzw)
    return np.asarray(st.pos, float) + r @ np.array([0.0, 0.0, EYE_Z]), r


def through_eye_by_pose(objects) -> bool:
    """Whether the rod's centreline passes through the eye's 9 mm hole, from the parts' poses. Not scored in the
    simulator, where the movement test decides; the run log prints it beside the test's result. Offline it stands in
    for the test's outcome (near_misses.py checks the test's readings separately)."""
    c, r = _eye_frame(objects["stopper"])
    p, d = _rod_axis(objects["pivot_rod"])
    a = r[:, 1]
    k = float(d @ a)
    if abs(k) < math.cos(math.radians(35.0)):
        return False
    s = float((c - p) @ a) / k
    q = p + s * d - c
    return bool(math.hypot(float(q @ r[:, 0]), float(q @ r[:, 2])) < HOLE and 0.0 < s < TIP)


def spout_reading(objects):
    """(distance from the spout's end face to the rod's outer end along the rod (m), the rod's offset from the port's
    axis where it crosses that face (m)), or None if the rod does not point into the spout."""
    p, d = _rod_axis(objects["pivot_rod"])
    p, d = RD.T @ (p - C0), RD.T @ d  # drain frame: the port along -y, its end face at y = -PORT_END
    if d[1] < math.cos(math.radians(35.0)):
        return None
    s = (-PORT_END - p[1]) / d[1]  # along the rod from the ball's centre to the end face
    x = p + s * d
    return float(s - (TIP - ROD_L)), float(math.hypot(x[0], x[2] - HP))


def in_port(objects) -> bool:
    """The rod comes out of the spout's bore (its centreline within the 16 mm bore, less the rod's radius)."""
    r = spout_reading(objects)
    return bool(r is not None and r[1] <= BORE - ROD_R + EPS)


def seated(objects) -> bool:
    r = spout_reading(objects)
    return bool(in_port(objects) and r[0] <= DEPTH_MAX + EPS)


def in_guide(objects) -> bool:
    st = objects["stopper"]
    p = np.asarray(st.pos, float)
    tilt = math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0)))
    return bool(math.hypot(p[0] - BX, p[1] - BY) <= GUIDE_OFF + EPS and tilt <= GUIDE_TILT + EPS
                and HF - 0.001 - EPS <= p[2] <= HF + GUIDE_LIFT + EPS)


# Release check: in the simulator an on_step hook measures, every step, the shortest distance from both arms' finger
# and gripper hulls to every shape of the rod and the stopper. Before the first measurement (no step since the reset)
# nothing is credited: the check fails closed. Offline (near_misses.py sets OFFLINE_FLAG; no simulator, no robot)
# each case stands for a state with the grippers away.
_GAP: dict[str, float] = {}
OFFLINE_FLAG = "STOPPER_TASK_OFFLINE_CHECK"
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
_SPECS = {o.name: o for o in OBJECTS}

# The test's state: phase, steps in the phase, heights, readings and result.
_TEST: dict = {}


def _new_test():
    _TEST.clear()
    _TEST.update(phase="wait", k=0)


@parts.on_reset
def _clear(sim):
    _GAP.clear()
    _new_test()
    _push(sim, None, None)


def _push(sim, rod_point, stopper_point, rod_share=1.0):
    """Set the test's pushes: rod_share x LIFT_N up at a world point on the rod, PULL_N up at the stopper's knob;
    None for no push. They act on every physics step until changed."""
    import torch

    base = sim.env.unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    dev = base.device
    for name, point, force in (("pivot_rod", rod_point, rod_share * LIFT_N), ("stopper", stopper_point, PULL_N)):
        comp = base.scene[name].permanent_wrench_composer
        if point is None:
            comp.reset()
            continue
        comp.set_forces_and_torques_index(
            forces=torch.tensor([[[0.0, 0.0, force]]], dtype=torch.float32, device=dev),
            positions=torch.tensor([[[*(np.asarray(point, float) + origin)]]], dtype=torch.float32, device=dev),
            body_ids=[0], env_ids=torch.tensor([0], dtype=torch.int32, device=dev), is_global=True,
        )


def _rod_end(st):
    p, d = _rod_axis(st)
    return p + (TIP - ROD_L + 0.002) * d  # 2 mm in from the outer end


def _knob(st):
    return np.asarray(st.pos, float) + _rot(st.quat_xyzw) @ np.array([0.0, 0.0, HEAD_T + KNOB_H])


@parts.on_step
def _record(sim):
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    states = sim.states()
    hands = []
    for arm in ("left_robot", "right_robot"):
        rob = base.scene[arm]
        poses = rob.data.body_link_pose_w
        poses = (poses.torch if hasattr(poses, "torch") else poses)[0].cpu().numpy().astype(float)
        links = {}
        for i, n in enumerate(rob.body_names):
            if n in _HAND_LINKS:
                t = np.eye(4)
                t[:3, :3] = _rot(poses[i, 3:])
                t[:3, 3] = poses[i, :3] - origin
                links[n] = t
        hands += analysis.world_hulls(links)
    for name in ("pivot_rod", "stopper"):
        st = states[name]
        boxes = _SPECS[name].world_boxes({"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))})
        _GAP[name] = min(float(analysis.hulls_to_box(hands, c, r, sz)[0]) for _, c, r, sz in boxes)
    _advance(sim, states)


def _advance(sim, states):
    """One control step of the movement test."""
    if not _TEST:
        _new_test()
    t = _TEST
    z = float(states["stopper"].pos[2])
    let_go = _released("pivot_rod") and _released("stopper")
    if t["phase"] != "wait" and not let_go:  # handled during or after the test: test again once let go and still
        _push(sim, None, None)
        _new_test()
        return
    if t["phase"] == "wait":
        ready = in_guide(states) and in_port(states) and _settled(states)
        t["k"] = t["k"] + 1 if ready else 0
        if t["k"] >= HOLD_STEPS:  # the readings from outside, then the test
            t.update(phase="lift", k=0, z0=z, seated=seated(states), depth=spout_reading(states)[0],
                     pose=through_eye_by_pose(states))
            _push(sim, _rod_end(states["pivot_rod"]), None, 1.0 / LIFT_STEPS)
        return
    t["k"] += 1
    if t["phase"] == "lift":  # the push rises steadily to LIFT_N, then holds for the rest of the second
        if t["k"] >= LIFT_STEPS:
            t.update(phase="back", k=0, z_low=z, drop=t["z0"] - z)
            _push(sim, _rod_end(states["pivot_rod"]), None, 1.0 - 1.0 / BACK_STEPS)
        else:
            _push(sim, _rod_end(states["pivot_rod"]), None, min(1.0, 2.0 * (t["k"] + 1) / LIFT_STEPS))
    elif t["phase"] == "back":  # the push eases off steadily to nothing, so the rod is lowered, not dropped
        if t["k"] >= BACK_STEPS:
            t.update(phase="settle", k=0)
            _push(sim, None, None)
        else:
            _push(sim, _rod_end(states["pivot_rod"]), None, max(0.0, 1.0 - (t["k"] + 1) / BACK_STEPS))
    elif t["phase"] == "settle":  # no push, so the rod and the stopper come to rest before the reading
        if t["k"] >= SETTLE_STEPS:
            t.update(phase="pull", k=0, z1=z, back=z - t["z_low"], rise=0.0)
            _push(sim, None, _knob(states["stopper"]))
    elif t["phase"] == "pull":
        t["rise"] = max(t["rise"], z - t["z1"])
        if t["k"] >= PULL_STEPS or t["rise"] > MAX_PULL_RISE:
            _push(sim, None, None)
            t.update(phase="done", k=0, passed=test_passes(t["drop"], t["back"], t["rise"]))
            print(f"[stopper test] t={sim.t:.2f} s: outer end {1e3 * t['depth']:.1f} mm from the spout, drop "
                  f"{1e3 * t['drop']:.1f} mm, back {1e3 * t['back']:.1f} mm, pull rise {1e3 * t['rise']:.1f} mm, "
                  f"passed {t['passed']}; through the eye by pose: {t['pose']}", flush=True)
        else:
            _push(sim, None, _knob(states["stopper"]))


def test_passes(drop, back, rise) -> bool:
    """The movement test's verdict from its three readings (m)."""
    return bool(drop >= MIN_DROP - EPS and back >= MIN_DROP - EPS and rise <= MAX_PULL_RISE + EPS)


def _released(name) -> bool:
    if name not in _GAP:
        return os.environ.get(OFFLINE_FLAG) == "1"
    return _GAP[name] >= RELEASE_GAP - EPS


def _rod_still(st) -> bool:
    """Still: moving under 1 cm/s and swinging under 0.5 rad/s. Spin about the rod's own axis is left out: a round
    rod turning in place about its centreline moves nothing (in the simulator it can keep a slow roll in the port)."""
    _, d = _rod_axis(st)
    w = np.asarray(st.ang_vel, float)
    return bool(np.linalg.norm(st.lin_vel) < 0.01 and np.linalg.norm(w - (w @ d) * d) < 0.5)


def _settled(objects) -> bool:
    return bool(_rod_still(objects["pivot_rod"]) and objects["stopper"].resting()
                and _released("pivot_rod") and _released("stopper"))


def _offline(objects) -> bool:
    """Offline only (no simulator, so no test can run): the parts' poses stand in for the test, read as it would be."""
    return bool(os.environ.get(OFFLINE_FLAG) == "1" and through_eye_by_pose(objects) and in_guide(objects)
                and in_port(objects) and _settled(objects))


def _result(objects):
    """(test passed, ball home) as the scorer last found them, or None. Once the test has passed, the result stands
    until a gripper comes within 5 mm of either part again, which restarts the check."""
    if not _TEST:  # no step since the reset
        return (True, seated(objects)) if _offline(objects) else None
    if _TEST.get("phase") != "done" or not _TEST.get("passed"):
        return None
    if not (_released("pivot_rod") and _released("stopper")):
        return None
    return True, bool(_TEST["seated"])


def success(objects):
    """Reconnected: the readings from outside and the movement test passed, with the ball home, and let go."""
    r = _result(objects)
    return bool(r is not None and r[1])


def score(objects):
    """1.0: the movement test passed and the ball was home (outer end at most 199 mm from the spout) when it began;
    0.5: the test passed but the ball was not home; else 0."""
    r = _result(objects)
    if r is None:
        return 0.0
    return 1.0 if r[1] else 0.5


# -- demo ------------------------------------------------------------------------------------------------------
TIP_DZ = 0.013  # finger tips below the TCP
LIFT = HP - (HF + EYE_Z)  # 26 mm: the lift that brings the eye level with the port
L_GRIP_Z = HF + 0.002 + TIP_DZ  # tips 2 mm above the flange, pads on the head's rim
L_OPEN = -(HEAD_R + 0.008)
L_CLOSE = -(HEAD_R - 0.003)
L_DZ = 0.0025  # carry compensation, set from traces
R_DZ = 0.0023
R_DY = -0.0008
R_DX = 0.0
GRIP_OUT = 0.100  # the right arm grips the rod 100 mm out from the ball's centre
R_PINCH = 0.009  # TCP above the rod's axis
R_OPEN = -0.012
TURN = -math.pi / 2
R_YAW = math.pi / 2 - (math.pi / 2 - PORT_DIR) % math.pi  # fingers close across the rod, in (-90, 90] deg


def demo():
    """Left arm: pinch the stopper's head, lift it 26 mm and turn it a quarter turn, hold it there, then let go once
    the rod is in. Right arm: pick the rod up from its rests, line it up with the port and push it straight in until
    the ball reaches its seat, let go and move away. Both then wait clear of the parts while the scorer runs its
    movement test. Open loop, from the published start layout."""
    hz = L_GRIP_Z
    left = [
        Waypoint(1.5, (BX, BY, hz + 0.03), L_OPEN, "over the stopper"),
        Waypoint(1.5, (BX, BY, hz), L_OPEN, "down round the head"),
        Waypoint(0.8, (BX, BY, hz), L_CLOSE, "pinch the head"),
        Waypoint(1.2, (BX, BY, hz + LIFT + L_DZ), L_CLOSE, "lift to the port"),
        Waypoint(1.5, (BX, BY, hz + LIFT + L_DZ), L_CLOSE, "quarter turn", TURN),
        Waypoint(5.5, (BX, BY, hz + LIFT + L_DZ), L_CLOSE, "hold while the rod goes in", TURN),
        Waypoint(0.6, (BX, BY, hz + LIFT + L_DZ), L_OPEN, "let go", TURN),
        Waypoint(1.5, (BX, BY + 0.07, hz + LIFT + L_DZ), L_OPEN, "slide off sideways, pads clear of the head", TURN),
        Waypoint(1.5, (BX - 0.08, BY + 0.10, hz + 0.03), L_OPEN, "away", TURN),
        Waypoint(14.0, (BX - 0.08, BY + 0.10, hz + 0.03), L_OPEN, "wait while the scorer tests the repair", TURN),
    ]
    sz = ROD_START[2]
    out = U[:2]  # outward along the port
    g0 = np.array(ROD_START[:2]) + out * GRIP_OUT  # grip point on the rod at its start
    zr = HP + R_PINCH + R_DZ
    fix = np.array([R_DX, R_DY])
    pre = SEATED[:2] + out * (0.062 + GRIP_OUT) + fix  # tip 6 mm short of the spout
    home = SEATED[:2] + out * (GRIP_OUT - 0.002) + fix  # pushed 2 mm past the seat
    back = home + out * 0.03
    away = home + out * 0.06 + np.array([-0.06, -0.06])

    def w(t, xy, z, f, label):
        return Waypoint(t, (float(xy[0]), float(xy[1]), z), f, label, R_YAW)

    right = [
        w(1.5, g0, sz + 0.06, R_OPEN, "over the rod"),
        w(1.2, g0, sz + R_PINCH, R_OPEN, "down"),
        w(0.8, g0, sz + R_PINCH, FINGER_CLOSED, "grip the rod"),
        w(1.2, g0, zr + 0.02, FINGER_CLOSED, "lift"),
        w(2.0, pre, zr, FINGER_CLOSED, "line up with the port"),
        w(0.5, pre, zr, FINGER_CLOSED, "wait for the stopper"),
        w(3.0, home, zr, FINGER_CLOSED, "push in to the seat"),
        w(0.6, home, zr, R_OPEN, "let go"),
        w(0.8, home, zr + 0.03, R_OPEN, "straight up"),
        w(1.0, back, zr + 0.05, R_OPEN, "back off"),
        w(5.0, away, zr + 0.06, R_OPEN, "away"),
        w(12.0, away, zr + 0.06, R_OPEN, "wait while the scorer tests the repair"),
    ]
    return {"left": left, "right": right}
