"""Offline checks for the frame-corner-protectors task: the geometry, the scoring rules on hand-built states, and the
scripted demo's reach (inverse kinematics only, no Isaac Sim). Run from this folder with yam_arena importable:

    python -m pytest test_task.py -q
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task as T  # noqa: E402
from partkit import PartState  # noqa: E402
from yam_arena import ik, station, task_file  # noqa: E402
from yam_arena.constants import FINGER_OPEN, READY  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402

RUN_LIMIT_S = 1800 / 20
GRIPPER_SPAN = 0.094  # pads 94 mm apart fully open


def _quat_xyzw(yaw=0.0, roll=0.0):
    """Rotation about z by yaw, then about x by roll (xyzw)."""
    cy, sy, cr, sr = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(roll / 2), math.sin(roll / 2)
    # q = qz(yaw) * qx(roll)
    w = cy * cr
    x = cy * sr
    y = sy * sr
    z = sy * cr
    return np.array([x, y, z, w])


def _state(pos, yaw=0.0, roll=0.0, sliders=None, vel=0.0):
    return PartState(pos=np.asarray(pos, float), quat_xyzw=_quat_xyzw(yaw, roll), lin_vel=np.array([vel, 0, 0.0]),
                     ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), sliders=sliders or {})


def _frame_state(dz=0.0, roll=0.0):
    return _state((T.FRAME_AT[0], T.FRAME_AT[1], T.FB + dz), roll=roll)


def _seated_protector(name, frame=None, squeeze=0.0015, shift=(0.0, 0.0, 0.0), yaw_err=0.0):
    frame = frame or _frame_state()
    corner = T.corner_world(frame, name)
    # seated: the corner at the pocket's back, the frame's front 0.5 mm above the origin (the roof on the frame's back)
    pos = corner + np.array([0.0, 0.0, -T.CLEAR]) + np.asarray(shift)
    return _state(pos, yaw=T.YAW[name] + math.radians(yaw_err), sliders={f"floor_{name}": -squeeze})


def _objects(prots=None, frame=None):
    frame = frame or _frame_state()
    out = {T.FRAME.name: frame}
    for name, spec in T.PROTECTORS.items():
        out[spec.name] = (prots or {}).get(name) or _state((*T.start_xy(name), T.TOP + T.LIP), yaw=T.YAW[name],
                                                           sliders={f"floor_{name}": 0.0})
    return out


@pytest.fixture(autouse=True)
def _fresh_trial():
    T._reset()
    T._S["tcps"] = [np.array(T.PARK["left"]), np.array(T.PARK["right"])]
    yield


# -- the task file loads ------------------------------------------------------------------------------------------------

def test_task_file_loads():
    tf = task_file.load(HERE / "task.py")
    names = set(tf.names)
    assert {"frame", "block_left", "block_right"} <= names
    assert {f"protector_{n}" for n in T.CORNERS} <= names
    assert tf.scene == "station" and tf.score is not None and tf.demo is not None


# -- geometry ----------------------------------------------------------------------------------------------------------

def test_protector_fits_the_open_gripper():
    width = T.POCKET + T.WALL
    assert width < GRIPPER_SPAN - 0.016  # at least 8 mm of room either side when the pads come down


def test_pocket_squeezes_the_frame():
    # roof above the frame's back by CLEAR; the floor stands PROUD above its front: net squeeze 1 mm, about 4 N
    squeeze = T.PROUD - T.CLEAR
    assert squeeze == pytest.approx(0.0005)
    assert 1.0 <= T.FLOOR_RATE * squeeze <= 10.0  # it grips: many times the protector's 0.2 N weight
    assert squeeze < T.FLOOR_TRAVEL


def test_the_lips_leave_a_height_window_around_the_aim():
    """With the protector's origin at height z below the frame's front (the demo aims Z_AIM), the frame's front must
    meet the floor's lip above the spheres' middles (it then rides up and presses the floor down) and the frame's back
    must meet the roof's lip below the spheres' middles."""
    zb = -T.Z_AIM  # the frame's front above the protector's origin
    lo = T.PROUD - T.LEAD_R  # floor spheres' middles
    hi = T.ROOF + T.LEAD_R - T.THICK  # roof spheres' middles, less the frame's thickness
    assert lo + 0.001 < zb < hi - 0.001
    assert hi - lo >= 0.006


def test_lips_are_flush_with_the_floor_and_roof():
    (boxes, roof_sph), (floor_boxes, floor_sph) = T._protector_shapes()
    top = max(c[2] + s[2] / 2 for c, s, *_ in floor_boxes)
    assert top == pytest.approx(T.PROUD)
    assert all(c[2] + r == pytest.approx(T.PROUD) for c, r, _ in floor_sph)  # crests level with the floor
    assert all(c[2] - r == pytest.approx(T.ROOF) for c, r, _ in roof_sph)  # and with the roof
    # the flat floor and roof start at the crests: no step for the frame's edge to catch on
    inner = T.POCKET - T.LEAD_R
    assert all(abs(c[0] + inner) < 1e-9 or abs(c[1] + inner) < 1e-9 for c, _, _ in floor_sph + roof_sph)
    # along each edge the spheres are close enough that the frame's edge always rests on several
    xs = sorted({round(c[1], 6) for c, _, _ in floor_sph if abs(c[0] + inner) < 1e-9})
    assert max(b - a for a, b in zip(xs, xs[1:])) <= T.LEAD_R + 1e-9
    # no sphere pokes out of the block (past the other open face, where it would catch the frame's edge first)
    for c, r, _ in floor_sph + roof_sph:
        assert c[0] - r >= -T.POCKET - 1e-9 and c[1] - r >= -T.POCKET - 1e-9


def test_fitted_protectors_sit_on_the_right_corners():
    """The two pre-fitted protectors are boxes on the frame's body: each wraps its corner (walls outside it, floor and
    roof over its front and back), and none overlaps a block."""
    for name in T.FITTED:
        boxes = T._fitted_boxes(name)
        corner = np.array([T.ALL_CORNERS[name][0] * T.HALF[0], T.ALL_CORNERS[name][1] * T.HALF[1]])
        lo = np.min([np.array(c) - np.array(sz) / 2 for c, sz, _ in boxes], axis=0)
        hi = np.max([np.array(c) + np.array(sz) / 2 for c, sz, _ in boxes], axis=0)
        sx, sy = T.ALL_CORNERS[name]
        # the footprint runs WALL past the corner outward and POCKET inward, both ways
        assert sorted([corner[0] - sx * T.POCKET, corner[0] + sx * T.WALL]) == pytest.approx([lo[0], hi[0]])
        assert sorted([corner[1] - sy * T.POCKET, corner[1] + sy * T.WALL]) == pytest.approx([lo[1], hi[1]])
        assert lo[2] == pytest.approx(-T.LIP) and hi[2] == pytest.approx(T.ROOF + T.LIP)
        # nothing inside the frame's moulding at the corner: the pocket's floor and roof are outside its front and back
        for c, sz, _ in boxes:
            if abs(c[2] - T.THICK / 2) > 0.01:
                assert c[2] + sz[2] / 2 <= 1e-9 or c[2] - sz[2] / 2 >= T.THICK - 1e-9
    assert T.FRAME.mass == pytest.approx(T.FRAME_MASS + 2 * T.PROTECTOR_MASS)


def test_only_the_bare_corners_get_loose_protectors():
    assert set(T.CORNERS) == {"far_left", "near_left"} and set(T.PROTECTORS) == set(T.CORNERS)
    assert not set(T.CORNERS) & set(T.FITTED)


def test_no_collider_is_turned():
    """PhysX built turned boxes differently from how USD draws them (Brev, 2026-11-13): keep every protector and
    frame collider unturned."""
    for spec in [T.FRAME, *T.PROTECTORS.values()]:
        shapes = list(spec.boxes) + [b for s in spec.sliders for b in s.boxes]
        assert all(len(b) < 4 or tuple(b[3]) == (1.0, 0.0, 0.0, 0.0) for b in shapes), spec.name


def test_corners_overhang_the_blocks_past_the_pocket():
    for name in T.ALL_CORNERS:
        cx, cy = T.corner_xy(name)
        sx, sy = T.ALL_CORNERS[name]
        # the seated protector's footprint, world
        lo_x, hi_x = sorted([cx - sx * T.POCKET, cx + sx * T.WALL])
        lo_y, hi_y = sorted([cy - sy * T.POCKET, cy + sy * T.WALL])
        bx0, bx1 = T.BLOCK_X - T.BLOCK[0] / 2, T.BLOCK_X + T.BLOCK[0] / 2
        by = sy * T.BLOCK_Y
        by0, by1 = by - T.BLOCK[1] / 2, by + T.BLOCK[1] / 2
        overlap = hi_x > bx0 and lo_x < bx1 and hi_y > by0 and lo_y < by1
        assert not overlap, name
        assert min(abs(lo_x - bx1), abs(hi_x - bx0)) >= 0.02  # 2 cm of clearance from the block's end


def test_blocks_sit_under_the_moulding():
    rail_in, rail_out = T.HALF[1] - T.MOULD_W, T.HALF[1]
    assert T.BLOCK_Y - T.BLOCK[1] / 2 < rail_out and T.BLOCK_Y + T.BLOCK[1] / 2 > rail_in


def test_protectors_start_clear_of_frame_and_blocks():
    for name in T.CORNERS:
        x, y = T.start_xy(name)
        sy = T.CORNERS[name][1]
        nearest = y - sy * T.POCKET  # the protector's edge nearest the frame
        assert abs(nearest) > T.BLOCK_Y + T.BLOCK[1] / 2 + 0.01, name


def test_pinch_stays_clear_of_the_frame():
    """The fingers close along world y across the protector's full width, on the closed wall beyond the frame's end.
    The pads (27.4 mm wide along x) must stay off the frame's end along x, and the finger's wider part (40 mm, 25 mm
    up from its tips) must be above the frame's back; the pads must bear on at least 12 mm of the wall."""
    for name in T.CORNERS:
        seat = T.corner_xy(name)
        g = T.grip_world(name, seat)
        sx, sy = T.CORNERS[name]
        out = sx * (g[0] - seat[0])  # the pinch's middle, out past the frame's end
        assert out - T.FINGER_HALF_W >= 0.002, name
        tip = T.PINCH_UP - T.TIP
        if tip + 0.025 < T.THICK + 0.003:  # the finger's wider part is at the frame's height: keep it off the frame's end
            assert out - 0.020 >= 0.002, name
        on_wall = min(out + T.FINGER_HALF_W, T.WALL) - max(out - T.FINGER_HALF_W, 0.0)
        assert on_wall >= 0.012, (name, on_wall)
        lo, hi = sorted([seat[1] - sy * T.POCKET, seat[1] + sy * T.WALL])
        assert g[1] == pytest.approx((lo + hi) / 2)


# -- scoring -----------------------------------------------------------------------------------------------------------

def test_both_seated_and_let_go_scores_full():
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    objs = _objects(prots)
    assert set(T.protected(objs)) == set(T.CORNERS)
    assert T.score(objs) == 1.0 and T.success(objs)


def test_partial_credit_counts_corners():
    for k in range(2):
        names = list(T.CORNERS)[:k]
        objs = _objects({n: _seated_protector(n) for n in names})
        assert T.score(objs) == pytest.approx(0.5 * k)
    assert T.score(_objects({"near_left": _seated_protector("near_left")})) == 0.5


def test_moving_parts_cap_full_score():
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    prots["far_left"] = PartState(**{**prots["far_left"].__dict__, "lin_vel": np.array([0.05, 0, 0])})
    assert T.score(_objects(prots)) == 0.75


@pytest.mark.parametrize("shift", [(0.006, 0, 0), (0, 0.006, 0), (-0.006, -0.006, 0), (0, 0, 0.005)])
def test_not_pushed_home_is_not_seated(shift):
    name = "far_left"
    frame = _frame_state()
    # shift in the protector's own frame: out along its pocket's opening
    yaw = T.YAW[name]
    c, s = math.cos(yaw), math.sin(yaw)
    world = (c * shift[0] - s * shift[1], s * shift[0] + c * shift[1], shift[2])
    if shift[0] > 0 or shift[1] > 0:  # into the wall: impossible physically, but the rule allows only 2 mm
        world = tuple(-v for v in world[:2]) + (world[2],)
    prot = _seated_protector(name, frame, shift=world)
    assert not T.seated_on(frame, prot, name)


def test_unsqueezed_floor_is_not_seated():
    frame = _frame_state()
    prot = _seated_protector("near_left", frame, squeeze=0.0)
    assert not T.seated_on(frame, prot, "near_left")


def test_turned_protector_is_not_seated():
    frame = _frame_state()
    prot = _seated_protector("near_left", frame, yaw_err=10.0)
    assert not T.seated_on(frame, prot, "near_left")


def test_wrong_protector_orientation_does_not_count_on_another_corner():
    frame = _frame_state()
    # the far-left protector, as turned, put on the near-left corner's place: its pocket faces the wrong way
    prot = _seated_protector("far_left", frame)
    assert not T.seated_on(frame, prot, "near_left")


def test_still_held_does_not_count():
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    T._S["tcps"] = [T.protector_middle(prots["far_left"]) + np.array([0.0, 0.0, 0.02]), np.array(T.PARK["right"])]
    objs = _objects(prots)
    assert "far_left" not in T.protected(objs)
    assert T.score(objs) == 0.5


def test_fingers_pressing_the_face_is_not_let_go(monkeypatch):
    """The demo's press pose (closed fingers on the outer face) must not count as let go."""
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    wps = T._fit("near_left")
    press = next(w for w in wps if w.label.endswith("push it home with the fingers"))
    T._S["tcps"] = [np.array(press.pos), np.array(T.PARK["right"])]
    assert "near_left" not in T.protected(_objects(prots))


def test_block_distance():
    prot = _seated_protector("far_left")
    mid = T.protector_middle(prot)
    assert T.block_distance(prot, mid) == 0.0
    assert T.block_distance(prot, mid + np.array([0.0, 0.0, 0.10])) == pytest.approx(0.10 - (T.ROOF + T.LIP - T.ROOF / 2), abs=1e-6)


def test_measured_press_pose_is_not_let_go():
    """On Brev the near press held the TCP 24 mm off the seated protector's outer face."""
    prot = _seated_protector("near_left")
    face_y = T.corner_xy("near_left")[1] + T.WALL
    T._S["tcps"] = [np.array([0.185, face_y + 0.024, 0.797]), np.array(T.PARK["right"])]
    assert not T.released(prot)


def test_lifted_away_counts_as_let_go():
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    for name in T.CORNERS:
        wps = T._fit(name)
        lift = next(w for w in wps if w.label.endswith("lift away"))
        T._S["tcps"] = [np.array(lift.pos), np.array(T.PARK["right"])]
        assert T.released(prots[name]), name


def test_dropped_frame_scores_zero():
    prots = {n: _seated_protector(n) for n in T.CORNERS}
    T._S["dropped"] = True
    assert T.score(_objects(prots)) == 0.0


def test_frame_off_the_blocks_or_tilted_scores_zero():
    for frame in (_frame_state(dz=-0.02), _frame_state(roll=math.radians(5))):
        prots = {n: _seated_protector(n, frame) for n in T.CORNERS}
        assert T.score(_objects(prots, frame)) == 0.0


def test_untouched_scene_scores_zero():
    assert T.score(_objects()) == 0.0


# -- the demo ------------------------------------------------------------------------------------------------------------

VARIANTS = ["full", "nothing", "no_hold", "keep_holding", "one_only", "near_only"]


def _plans(variant, monkeypatch):
    monkeypatch.setenv("DEMO_VARIANT", variant)
    plan = T.demo()
    return {arm: ArmPlan(wps, station.BASES[arm], q_start=READY, finger_start=FINGER_OPEN) for arm, wps in plan.items()}


@pytest.mark.parametrize("variant", VARIANTS)
def test_demo_fits_the_run_limit(variant, monkeypatch):
    if variant == "keep_holding":  # keeps pressing until the trial ends, by design
        return
    plans = _plans(variant, monkeypatch)
    assert max(p.duration for p in plans.values()) < RUN_LIMIT_S - 5.0


@pytest.mark.parametrize("variant", VARIANTS)
def test_demo_is_reachable(variant, monkeypatch):
    plans = _plans(variant, monkeypatch)
    for arm, plan in plans.items():
        t = 0.0
        while t <= plan.duration + 0.5:
            plan.action7(t)
            t += 1 / 20
        assert plan.worst_ik_mm < 2.0, (variant, arm, plan.worst_ik_mm)
        assert plan.worst_ik_deg < 2.0, (variant, arm, plan.worst_ik_deg)


@pytest.mark.parametrize("variant", VARIANTS)
def test_arms_keep_apart(variant, monkeypatch):
    plans = _plans(variant, monkeypatch)
    if len(plans) < 2:
        return
    end = max(p.duration for p in plans.values()) + 0.5
    t, worst = 0.0, 9.0
    while t <= end:
        a = plans["left"].target(t)[0]
        b = plans["right"].target(t)[0]
        worst = min(worst, float(np.linalg.norm(np.asarray(a) - np.asarray(b))))
        t += 0.05
    assert worst > 0.15, (variant, worst)


def test_full_demo_holds_the_frame_during_every_push(monkeypatch):
    """Whenever the left arm is pushing a protector on, the right arm's fingertips are pressed on the frame's back."""
    plans = _plans("full", monkeypatch)
    z_press = T.FB + T.SLAB_Z[1] + T.TIP - T.PRESS
    for arm, other in (("left", "right"),):
        t = 0.0
        while t <= plans[arm].duration:
            label = plans[arm].target(t)[3]
            if "push it" in label or "press it into" in label or "settle" in label:
                pos = plans[other].target(t)[0]
                assert pos[2] == pytest.approx(z_press, abs=0.004), (arm, t, label, pos)
            t += 0.1


def test_release_lifts_clear_before_moving_on(monkeypatch):
    """After letting go, the gripper rises straight up until the pad tips are above the protector's top before it moves
    sideways (on Brev a 35 mm lift dragged the far protector back off its corner)."""
    top = T.FB - T.CLEAR + T.ROOF + T.LIP
    for name in T.CORNERS:
        wps = T._fit(name)
        i = next(k for k, w in enumerate(wps) if w.label.endswith(": up"))
        before, up = wps[i - 1], wps[i]
        assert up.pos[:2] == pytest.approx(before.pos[:2])
        assert up.pos[2] - T.TIP >= top + 0.010


def test_grip_rotation_is_straight_down():
    r = grip_rotation(0.0)
    assert np.allclose(r @ np.array([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0], atol=1e-6)
    _ = ik  # imported for the reach tests' warm start


def _box_face_line(spec, axis, top):
    """The two ends (s, z) of a slope box's top (or bottom) face, along ``axis``, from its centre, size and rotation."""
    centre, size, _, q = spec
    w, x, y, z = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    i = 0 if axis == "x" else 1
    half = np.array(size) / 2
    ends = []
    for sgn in (-1, 1):
        local = np.zeros(3)
        local[i] = sgn * half[i]
        local[2] = half[2] if top else -half[2]
        p = np.array(centre) + r @ local
        ends.append((p[i], p[2]))
    return sorted(ends)


def test_floor_rests_proud_in_the_part_frame():
    """partkit places slider shapes in the part's frame: at rest the floor's top is PROUD above the origin, and its
    lowest point (the lips' spheres) stays above the walls' bottom, so a protector lying on the table rests on its
    walls."""
    _, (floor_boxes, floor_sph) = T._protector_shapes()
    bottom = min([c[2] - s[2] / 2 for c, s, *_ in floor_boxes] + [c[2] - r for c, r, _ in floor_sph])
    assert bottom > -T.LIP + 0.0005


def test_object_poses_are_plain_floats():
    """torch.tensor() of numpy float64s makes a float64 tensor, which the simulator's pose writes reject."""
    for obj in T.OBJECTS:
        assert all(type(v) is float for v in obj.pos), (obj.name, [type(v) for v in obj.pos])
        assert type(obj.yaw) is float, obj.name
