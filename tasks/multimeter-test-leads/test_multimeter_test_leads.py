"""Offline checks (no GPU): the task file loads, the meter's box colliders leave each jack's bore open and stay in the
mesh, the looks stay in their bounds, the scorer covers each outcome (with the meter where it starts, and moved and
turned), and the demo's reach and clearances. Run in the yam_arena environment:
python -m pytest -q test_multimeter_test_leads.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena import YAM_URDF
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load
from yam_arena.urdf import forward_kinematics

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "multimeter_test_leads.py")
T = TASK.module

import fingers  # noqa: E402  (the task file put its folder on the path)

DOWN_PIN = (0.0, -math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # the lead's +x up: its pin points down


def _qz(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def _qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.05, high=pos + 0.05)


METER0 = np.array([*T.METER_XY, T.PSU_TOP])  # the file's origin at the start


class Meter:
    """The meter placed with its file origin at ``origin`` turned ``yaw``: its scorer state and its jacks."""

    def __init__(self, origin=METER0, yaw=T.METER_YAW, tilt=None):
        self.origin, self.quat = np.asarray(origin, float), _qz(yaw)
        if tilt is not None:
            self.quat = _qmul(self.quat, tilt)
        self.rot = T._rot(self.quat)

    def world(self, local):
        return self.origin + self.rot @ np.asarray(local, float)

    def state(self):
        return _state(self.world(T.METER_CENTROID), self.quat)

    def jack(self, name, below_face=0.0):
        return self.world((T.JACK_X, T.JACK_Y[name], T.FACE - below_face))

    def axis_quat(self):
        """The lead's orientation with its pin down the meter's jack axis."""
        return _qmul(self.quat, np.asarray(DOWN_PIN))


def _lead(spec, tip_at, quat):
    rot = T._rot(quat)
    return _state(np.asarray(tip_at) - rot @ (np.array([spec.tip, 0.0, 0.0]) - np.asarray(spec.centroid)), quat)


def _lying(spec):
    return _state(np.asarray(spec.pos) + np.asarray(spec.centroid))


def scene(meter=None, black=None, red=None):
    """``black`` and ``red``: None (lying where they start) or (jack, depth)."""
    meter = meter or Meter()
    out = {"multimeter": meter.state()}
    for name, spec, where in (("black_lead", T.BLACK, black), ("red_lead", T.RED, red)):
        if where is None:
            out[name] = _lying(spec)
        else:
            jack, depth = where
            out[name] = _lead(spec, meter.jack(jack, depth), meter.axis_quat())
    return out


# -- the scene -------------------------------------------------------------------------------------------------------


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"multimeter", "black_lead", "red_lead"}


def _boxes_corners(box):
    centre, size = np.asarray(box[0]), np.asarray(box[1])
    turn = box[2] if len(box) > 2 else (0.0, 0.0, 0.0)
    rx, ry, rz = (math.radians(a) for a in turn)
    rot_x = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    rot_y = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    rot_z = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    rot = rot_z @ rot_y @ rot_x
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * 0.5
    return centre + (signs * size) @ rot.T


def test_meter_boxes_inside_the_mesh():
    lo, hi = np.asarray(T.METER_LO), np.asarray(T.METER_HI)
    for box in T.METER_BOXES:
        c = _boxes_corners(box)
        assert np.all(c >= lo - 1e-9) and np.all(c <= hi + 1e-9), box


def _point_in_box(p, box):
    """Whether the point is inside the (turned) box."""
    centre, size = np.asarray(box[0]), np.asarray(box[1])
    corners = _boxes_corners(box)
    axes = [corners[4] - corners[0], corners[2] - corners[0], corners[1] - corners[0]]
    d = np.asarray(p) - centre
    return all(abs(d @ (a / np.linalg.norm(a))) < s / 2 - 1e-7 for a, s in zip(axes, size))


@pytest.mark.parametrize("jack", ["a", "com", "v_ohm"])
def test_bore_is_open(jack):
    """A 4 mm pin's square goes down 19.5 mm into each bore without entering a box, and the bore has a floor."""
    yj, h = T.JACK_Y[jack], 0.002
    for z in np.linspace(T.FACE - 0.0195, T.FACE + 0.005, 30):
        for dx in (-h, 0.0, h):
            for dy in (-h, 0.0, h):
                p = (T.JACK_X + dx, yj + dy, z)
                assert not any(_point_in_box(p, b) for b in T.METER_BOXES), p
    assert any(_point_in_box((T.JACK_X, yj, T.FACE - T.BORE_DEPTH - 0.001), b) for b in T.METER_BOXES)
    # the face around the bore holds the plug's body up
    for d in (0.004, -0.004):
        assert any(_point_in_box((T.JACK_X + d, yj + d, T.FACE - 0.0002), b) for b in T.METER_BOXES)
    # the chamfer: 0.5 mm down, a point 0.5 mm outside the bore's wall is open, 1.5 mm outside is solid
    for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        for out, solid in ((0.0005, False), (0.0015, True)):
            r = T.BORE / 2 + out
            p = (T.JACK_X + sx * r, yj + sy * r, T.FACE - 0.0005)
            assert any(_point_in_box(p, b) for b in T.METER_BOXES) == solid, (p, solid)


def test_pin_fits_with_play():
    assert T.BORE - T.RED.pin_width >= 0.0003 - 1e-9  # 0.15 mm each side
    assert T.BORE_DEPTH > T.RED.pin_length  # the body sits on the face before the pin bottoms out
    assert T.RED.size[1] > T.BORE + 2 * T.CHAMFER


def test_pin_has_a_narrow_tip():
    """The pin's last 2 mm are 3 mm square: off the bore's axis by up to 0.65 mm, the tip still goes in."""
    spec = T.RED
    boxes = [p[1:] for p in spec.parts()]

    def solid(along, y, z):
        return any(_point_in_box((spec.tip + along, y, z), b) for b in boxes)

    assert solid(0.001, 0.0014, 0.0014) and not solid(0.001, 0.0016, 0.0)
    assert solid(0.0025, 0.0019, 0.0019) and not solid(-0.0001, 0.0, 0.0)
    assert (T.BORE - spec.tip_width) / 2 >= 0.00065 - 1e-9


def test_lead_bounds_match_the_looks():
    from leads import Lead  # noqa: F401
    spec = T.RED
    lo = np.array([spec.tip, -spec.size[1] / 2, -spec.size[2] / 2])
    hi = np.array([spec.end, spec.size[1] / 2, spec.size[2] / 2])
    import ast
    src = (HERE / "looks" / "make_looks.py").read_text()
    consts = {}
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            if node.targets[0].id in ("LEAD_LO", "LEAD_HI", "PSU_SIZE"):
                consts[node.targets[0].id] = ast.literal_eval(node.value)
    assert np.allclose(consts["LEAD_LO"], lo) and np.allclose(consts["LEAD_HI"], hi)
    assert tuple(consts["PSU_SIZE"]) == T.PSU_SIZE
    assert np.allclose(spec.centroid, (lo + hi) / 2)


def test_jacks_where_the_layout_says():
    assert T.jack_xy("com") == pytest.approx((0.40, -0.19))
    a, v = T.jack_xy("a"), T.jack_xy("v_ohm")
    assert a[1] > -0.19 > v[1]  # A on the robot's left, V ohm on its right
    assert abs(a[1] - v[1]) == pytest.approx(0.0406, abs=1e-4)
    # the meter's jack end is flush with the supply's front, and it lies on the supply
    assert T.METER_XY[0] - T.METER_HI[0] == pytest.approx(T.PSU_FRONT, abs=0.002)
    assert abs(T.METER_XY[1] - T.PSU_Y) < T.PSU_SIZE[1] / 2 - 0.045


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_start_scores_zero():
    s = scene()
    assert T.score(s) == 0.0 and not T.success(s)
    for name in T.LEADS:
        for jack in T.JACK_Y:
            assert T.depth_in(s, name, jack) == 0.0


MOVES = {
    "as placed": Meter(),
    "slid and turned": Meter(METER0 + (0.03, -0.02, 0.0), T.METER_YAW + 0.4),
    "tilted": Meter(METER0 + (0.0, 0.0, 0.01), T.METER_YAW, tilt=np.array([math.sin(0.1), 0.0, 0.0, math.cos(0.1)])),
}


@pytest.mark.parametrize("move", MOVES)
def test_ladder(move):
    m = MOVES[move]
    assert T.score(scene(m, black=("com", 0.018), red=("v_ohm", 0.018))) == 1.0
    assert T.success(scene(m, black=("com", 0.018), red=("v_ohm", 0.018)))
    assert T.score(scene(m, black=("com", 0.0121))) == 0.5
    assert T.score(scene(m, black=("com", 0.0115))) == 0.25
    assert T.score(scene(m, red=("v_ohm", 0.0051))) == 0.25
    assert T.score(scene(m, red=("v_ohm", 0.0045))) == 0.0
    assert T.score(scene(m, black=("com", 0.018), red=("v_ohm", 0.008))) == 0.75
    assert T.score(scene(m, black=("com", 0.007), red=("v_ohm", 0.007))) == 0.5
    assert not T.success(scene(m, black=("com", 0.018), red=("v_ohm", 0.008)))


def test_swapped_leads_score_zero():
    assert T.score(scene(black=("v_ohm", 0.018), red=("com", 0.018))) == 0.0
    assert T.score(scene(black=("com", 0.018), red=("com", 0.0))) == 0.5  # red resting on the black plug's jack


@pytest.mark.parametrize("who", ["black", "red"])
def test_a_lead_in_the_10_amp_jack_scores_zero(who):
    other = {"black": ("red", ("v_ohm", 0.018)), "red": ("black", ("com", 0.018))}[who]
    kw = {who: ("a", 0.018), other[0]: other[1]}
    assert T.score(scene(**kw)) == 0.0 and not T.success(scene(**kw))
    kw[who] = ("a", 0.0035)
    assert T.score(scene(**kw)) == 0.0
    kw[who] = ("a", 0.0025)  # resting at the mouth, in the chamfer: not in
    assert T.score(scene(**kw)) == 0.5


def test_pin_beside_a_jack_is_not_in():
    m = Meter()
    s = scene(m, black=("com", 0.018), red=("v_ohm", 0.018))
    off = m.rot @ np.array([0.0, T.BORE / 2 + 0.0015, 0.0])
    s["red_lead"] = _state(s["red_lead"].pos + off, s["red_lead"].quat_xyzw)
    assert T.score(s) == 0.5


def test_off_table_or_blown_up_scores_zero():
    s = scene(black=("com", 0.018), red=("v_ohm", 0.018))
    for name in ("multimeter", "black_lead", "red_lead"):
        fallen = dict(s)
        fallen[name] = _state((0.4, -0.2, T.TOP - 0.1))
        assert T.score(fallen) == 0.0
    s["red_lead"] = _state((np.nan, 0.0, 1.0))
    assert T.score(s) == 0.0


def test_moving_leads_are_not_success():
    s = scene(black=("com", 0.018), red=("v_ohm", 0.018))
    s["red_lead"] = _state(s["red_lead"].pos, s["red_lead"].quat_xyzw, vel=(0.05, 0.0, 0.0))
    assert T.score(s) == 1.0 and not T.success(s)


# -- the demo --------------------------------------------------------------------------------------------------------


def _plan():
    return ArmPlan(T.demo()["right"], BASES["right"])


def test_demo_reachable():
    plan = _plan()
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_demo_puts_the_pin_in_its_jack():
    """At "push in", the plug held as gripped (its axis AHEAD past the TCP, the body's front 3 cm below it) has its
    pin tip 1 mm deeper than seated, over the jack's axis."""
    steps = T.demo()["right"]
    pushes = [w for w in steps if w.label == "push in"]
    assert len(pushes) == 2
    for w, jack in zip(pushes, ("com", "v_ohm")):
        jx, jy = T.jack_xy(jack)
        axis = np.asarray(w.pos) + (T.AHEAD, 0.0, 0.0) - (*T.AIM, 0.0)
        assert axis[:2] == pytest.approx((jx, jy), abs=1e-9)
        assert max(abs(a) for a in T.AIM) <= 0.0025
        tip_z = w.pos[2] - T.ABOVE_BODY - T.RED.pin_length
        assert tip_z == pytest.approx(T.FACE_Z - T.RED.pin_length - T.PRESS)


def _arm_points(q):
    """Points on the right gripper's housing and fingers (world), for arm joints q and the finger joint."""
    poses = forward_kinematics(YAM_URDF, {**{f"joint{i + 1}": float(v) for i, v in enumerate(q[:6])},
                                          "joint7": q[6], "joint8": q[6]})
    base = np.asarray(BASES["right"])
    pts = []
    for link in ("gripper", "tip_left", "tip_right"):
        t = poses[link]
        pts.append(fingers.cloud(link, YAM_URDF) @ t[:3, :3].T + t[:3, 3] + base)
    return np.vstack(pts)


def _inside(points, lo, hi, margin=0.0):
    return np.all((points > np.asarray(lo) + margin) & (points < np.asarray(hi) - margin), axis=1)


def test_demo_clears_the_supply_and_the_meter():
    """No point of the gripper or its fingers enters the power supply, or the meter's body (its box, the face down),
    at any step of the plan: only the plug touches the meter."""
    plan = _plan()
    psu_lo = np.asarray(T.PSU.pos) - np.asarray(T.PSU_SIZE) / 2
    psu_hi = np.asarray(T.PSU.pos) + np.asarray(T.PSU_SIZE) / 2
    meter_lo = (T.METER_XY[0] - T.METER_HI[0], T.METER_XY[1] - T.METER_HI[1], T.PSU_TOP)
    meter_hi = (T.METER_XY[0] - T.METER_LO[0], T.METER_XY[1] - T.METER_LO[1], T.FACE_Z)
    worst = 1.0
    for k in range(int(plan.duration * 10) + 1):
        a, label = plan.action7(k / 10)
        finger = plan.target(k / 10)[2]
        pts = _arm_points(np.r_[plan.q, finger])
        assert not np.any(_inside(pts, psu_lo, psu_hi)), (k / 10, label, "power supply")
        assert not np.any(_inside(pts, meter_lo, meter_hi)), (k / 10, label, "meter")
        assert np.all(pts[:, 2] > T.TOP), (k / 10, label, "table")
        worst = min(worst, float(pts[:, 2].min() - T.TOP))
    assert worst > 0.0


def test_release_clears_the_plug_beside():
    """Let go of the red plug in V ohm: the fingers opened to FINGER_LOOSE stay clear of the black plug in COM."""
    fx = abs(T.FINGER_LOOSE)  # each pad's inner face from the plug's axis
    finger_thick = 0.0064  # the fingers' thickest part within 13 mm of the pad tips (URDF meshes)
    gap = abs(T.jack_xy("com")[1] - T.jack_xy("v_ohm")[1]) - T.PLUG[1] / 2
    assert gap - (fx + finger_thick) >= 0.002
    assert fx - T.PLUG[1] / 2 >= 0.0015
