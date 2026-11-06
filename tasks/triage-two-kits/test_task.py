"""Offline checks for triage-two-kits (NumPy and pytest only, no Isaac Sim).

    YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest -q test_task.py

YAM_ARENA_DIR is a checkout of the arena (tested at bd62771); it is not needed when yam_arena is installed.
"""

import itertools
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

for _candidate in (os.environ.get("YAM_ARENA_DIR"), os.environ.get("YAM_ARENA"), "~/yam-isaac-lab-arena"):
    if _candidate and Path(_candidate).expanduser().is_dir():
        sys.path.insert(0, str(Path(_candidate).expanduser()))
        break

from yam_arena import ik, station  # noqa: E402
from yam_arena.analysis import box_signed_distance, fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, ObjectState, load  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

HERE = Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
KITS = M.KIT_NAMES
HZ = 20.0
TOP = TABLE_TOP["station"]
GRIPPER_MAX_GAP = 0.094  # pads 94 mm apart at FINGER_OPEN
USABLE_GRIP = 0.084
JOINT4_TRAVEL_LIMIT = 0.14  # TCP height over the table past which joint4 can run out (arena stack_cubes.py)
KX, KY, KZ = M.KIT_SIZE
I3 = np.eye(3)


# -- states ---------------------------------------------------------------------------------------------------------


def _quat_x(deg):
    h = math.radians(deg) / 2
    return np.array([math.sin(h), 0.0, 0.0, math.cos(h)])


def kit(xy, bottom, on_side=False, vel=(0.0, 0.0, 0.0)):
    """A kit with its bottom at ``bottom``; ``on_side``: rolled 90 deg about its long (x) axis."""
    half = np.array([KX, KZ, KY]) / 2 if on_side else np.array([KX, KY, KZ]) / 2
    c = np.array([xy[0], xy[1], bottom + half[2]])
    q = _quat_x(90.0 if on_side else 0.0)
    return ObjectState(c, q, np.asarray(vel, float), np.zeros(3), c - half, c + half)


def fixed_states():
    out = {}
    for o in M.OBJECTS:
        if o.free:
            continue
        p = np.asarray(o.pos)
        h = np.asarray(o.size) / 2 if isinstance(o, Box) else np.array([o.radius, o.radius, o.height / 2])
        out[o.name] = ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), p - h, p + h)
    return out


def start_states():
    return {n: kit((M.KIT_X, y), TOP) for n, y in M.KIT_Y.items()}


def on_pad(colour, dx=0.0, dy=0.0):
    return kit((M.PAD_X + dx, M.CASUALTY_Y[colour] + dy), M.PAD_TOP)


def world(**kits):
    out = fixed_states()
    out.update(start_states())
    out.update(kits)
    return out


def placed(a, b):
    """kit_outer at pad ``a`` and kit_inner at pad ``b`` (None: left where it started)."""
    k = {}
    for name, colour, dx in ((KITS[0], a, -0.03), (KITS[1], b, 0.03)):
        if colour is not None:
            k[name] = on_pad(colour, dx=dx)  # two kits on one pad sit side by side along x
    return world(**k)


def spec_boxes():
    out = []
    for o in M.OBJECTS:
        p = np.asarray(o.pos)
        h = np.asarray(o.size) / 2 if isinstance(o, Box) else np.array([o.radius, o.radius, o.height / 2])
        out.append((o.name, p - h, p + h))
    return out


# -- 1. the file loads ----------------------------------------------------------------------------------------------


def test_loads_with_two_identical_loose_kits_and_everything_else_fixed():
    assert TASK.scene == "station" and TASK.table_top == TOP
    assert set(TASK.free_objects) == set(KITS) and len(KITS) == 2
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert TASK.docs.startswith("On the station table")
    loose = [o for o in M.OBJECTS if o.free]
    assert all(isinstance(o, Box) for o in loose)
    assert len({(o.size, o.mass, o.color) for o in loose}) == 1  # nothing tells the kits apart
    assert not any(isinstance(o, Cylinder) and o.free for o in M.OBJECTS)


def test_the_tag_mix_is_fixed_and_one_of_each_colour():
    assert sorted(M.CASUALTY_Y) == ["green", "red", "yellow"]
    left_to_right = sorted(M.CASUALTY_Y, key=lambda c: -M.CASUALTY_Y[c])
    assert left_to_right == ["green", "red", "yellow"]
    tags = {o.name: o.color for o in M.OBJECTS if o.name.startswith("tag_")}
    assert len(set(tags.values())) == 3
    assert M.KIT_COLOR not in tags.values() and M.BODY_COLOR not in tags.values()


# -- 2. the demo: reachable, timed, clear of everything ---------------------------------------------------------------


@pytest.fixture(scope="module")
def played():
    """Per arm: the ArmPlan and per control step (t, joints, finger target, TCP target, label)."""
    out = {}
    for side, wps in TASK.demo().items():
        arm = ArmPlan(wps, station.BASES[side])
        steps = []
        for k in range(round(arm.duration * HZ) + 5):
            t = (k + 1) / HZ
            pos, _, finger, label = arm.target(t)
            action, _ = arm.action7(t)
            steps.append((t, action[:6].copy(), finger, pos, label))
        out[side] = (arm, steps)
    return out


def test_demo_is_reachable_within_joint_limits_at_20_hz(played):
    assert set(played) == {"left", "right"}
    for side, (arm, steps) in played.items():
        joints = np.array([s[1] for s in steps])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(joints >= ik.LOWER) and np.all(joints <= ik.UPPER), side


def test_both_arm_plans_end_together_with_a_still_hold():
    plan = TASK.demo()
    ends = {side: sum(w.duration for w in wps) for side, wps in plan.items()}
    assert ends["left"] == pytest.approx(ends["right"])
    for wps in plan.values():
        last, before = wps[-1], wps[-2]
        assert last.label == "hold still" and last.duration >= 1.5
        assert tuple(last.pos) == tuple(before.pos) and last.finger == before.finger


def test_demo_fits_the_run_limit():
    run_limit = int(json.loads((HERE / "task.json").read_text())["run_limit"].split()[0].replace(",", ""))
    steps = max(sum(w.duration for w in wps) for wps in TASK.demo().values()) * HZ
    assert steps + 100 < run_limit / 3, (steps, run_limit)


def test_travel_height_stays_under_the_joint4_limit_and_over_the_scene():
    for wps in TASK.demo().values():
        assert max(w.pos[2] for w in wps) - TOP <= JOINT4_TRAVEL_LIMIT
    tallest = max(hi[2] for n, lo, hi in spec_boxes() if n not in KITS)  # the heads and tags, 35-39 mm
    assert M.TRAVEL_Z - M.GRIP_ABOVE_BOTTOM > tallest + 0.03  # a carried kit's bottom over everything fixed
    assert M.TRAVEL_Z - PAD_TIP_PAST_TCP > TOP + KZ + 0.03  # open fingertips over a standing kit


def test_each_release_is_followed_by_a_straight_rise_of_at_least_3_cm():
    for wps in TASK.demo().values():
        for i, w in enumerate(wps):
            if w.label == "release":
                up = wps[i + 1]
                assert up.pos[:2] == w.pos[:2] and up.pos[2] - w.pos[2] >= 0.03


def _hulls(side, joints, finger):
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = fk_link_poses_world(q, station.BASES[side])
    return world_hulls({k: v for k, v in poses.items() if k != "base"})


def _box_points(centre, size, n=5):
    g = np.linspace(-0.5, 0.5, n)
    local = np.array([[a, b, c] for a in g for b in g for c in g])
    return local[np.abs(local).max(axis=1) >= 0.5] * np.asarray(size) + np.asarray(centre)


def _min_to_boxes(points, boxes):
    best = (np.inf, "")
    for name, lo, hi in boxes:
        s = float(box_signed_distance(points, (lo + hi) / 2, I3, hi - lo).min())
        best = min(best, (s, name))
    return best


def _hulls_to_boxes(hulls, aabbs, boxes, reach=0.03):
    """(min signed distance from any hull vertex to any box, box name); only hulls whose bounding box comes within
    ``reach`` of a box are measured exactly (anything farther is reported as ``reach``)."""
    best = (reach, "")
    for name, lo, hi in boxes:
        gap = np.maximum(lo - aabbs[:, 1], aabbs[:, 0] - hi).max(axis=1)
        for i in np.flatnonzero(gap < reach):
            s = float(box_signed_distance(hulls[i][1].vertices, (lo + hi) / 2, I3, hi - lo).min())
            best = min(best, (s, name))
    return best


def _aabb_gap(a, b):
    """A lower bound on the distance between two hull sets: the smallest gap between their bounding boxes."""
    gap = np.maximum(b[None, :, 0] - a[:, None, 1], a[:, None, 0] - b[None, :, 1])
    return float(np.linalg.norm(np.maximum(gap, 0.0), axis=2).min())


HELD = ("close", "lift", "carry", "lower", "release")  # the kit is in the pads (or being let go)
# After a release the pads open beside the kit and rise straight up (tested above); the released kit is not
# checked against its own arm until the retreat ends.
NEXT_TO_OWN_KIT = (*HELD, "retreat")
OFFSETS = (0.0, -0.015, -0.020)


@pytest.fixture(scope="module")
def sweeps(played):
    """Play both plans kinematically, once for each grip offset (the carried kit hangs that far from the TCP in y;
    sim: 12-20 mm toward -y). Per offset: worst arm-to-scene gap, worst carried-kit gap, kit bottom-centres at the
    end. Also the closest the two arms come (bounding-box lower bound) and the lowest arm point over the table."""
    fixed = [(n, lo, hi) for n, lo, hi in spec_boxes() if n not in KITS]
    kits = {o: {n: np.array([M.KIT_X, y, TOP]) for n, y in M.KIT_Y.items()} for o in OFFSETS}
    worst_arm = {o: (np.inf, "") for o in OFFSETS}
    worst_kit = {o: (np.inf, "") for o in OFFSETS}
    closest, lowest = np.inf, np.inf
    for k in range(len(played["left"][1])):
        boxes = {}
        for side, (_, steps) in played.items():
            t, joints, finger, tcp, label = steps[k]
            name, _ = M.DEMO_ROUTE[side]
            held = label in NEXT_TO_OWN_KIT
            d = min(finger, -KY / 2) if label in ("lift", "carry", "lower") else finger  # pads stop on the kit
            h = _hulls(side, joints, d)
            aabbs = np.array([[x.vertices.min(0), x.vertices.max(0)] for _, x in h])
            boxes[side] = aabbs
            lowest = min(lowest, float(aabbs[:, 0, 2].min()))
            fixed_gap = _hulls_to_boxes(h, aabbs, fixed)
            for o in OFFSETS:
                others = [(n, b - [KX / 2, KY / 2, 0], b + [KX / 2, KY / 2, KZ]) for n, b in kits[o].items()
                          if not (held and n == name)]
                gap = min(fixed_gap, _hulls_to_boxes(h, aabbs, others))
                worst_arm[o] = min(worst_arm[o], (gap[0], f"{side} {label} t={t:.2f}: {gap[1]}"))
                if label in ("lift", "carry", "lower"):
                    c = np.asarray(tcp) + [0, o, -M.GRIP_ABOVE_BOTTOM + KZ / 2]
                    kg = _min_to_boxes(_box_points(c, M.KIT_SIZE), fixed + others)
                    worst_kit[o] = min(worst_kit[o], (kg[0], f"{side} {label} t={t:.2f}: {kg[1]}"))
                if label == "release":
                    kits[o][name] = np.array([tcp[0], tcp[1] + o, M.PAD_TOP])
        closest = min(closest, _aabb_gap(boxes["left"], boxes["right"]))
    return {o: (worst_arm[o], worst_kit[o], closest, lowest, kits[o]) for o in OFFSETS}


@pytest.mark.parametrize("offset", [0.0, -0.015, -0.020])
def test_arm_links_and_carried_kit_clear_the_scene(sweeps, offset):
    worst_arm, worst_kit, _, lowest, _ = sweeps[offset]
    assert worst_arm[0] > 0.005, worst_arm  # every arm link 5 mm off casualties, pads, tags and the other kit
    assert worst_kit[0] > 0.002, worst_kit  # the carried kit never touches anything fixed or the other kit
    assert lowest > TOP + 0.002, lowest - TOP


def test_the_two_arms_stay_apart(sweeps):
    _, _, closest, _, _ = sweeps[0.0]
    assert closest > 0.10, closest  # the right arm waits outboard until the left arm has parked


@pytest.mark.parametrize("offset", [0.0, -0.015, -0.020])
def test_demo_ends_on_red_and_yellow_with_full_score(sweeps, offset):
    *_, kits = sweeps[offset]
    states = world(**{n: kit(b[:2], b[2]) for n, b in kits.items()})
    assert M.placements(states) == ("red", "yellow")
    assert TASK.score(states) == 1.0 and TASK.success(states)
    for n, b in kits.items():  # each kit's centre keeps at least 20 mm inside its pad's footprint
        x0, x1, y0, y1 = M.pad_bounds(M.pad_of(states[n]))
        assert min(b[0] - x0, x1 - b[0], b[1] - y0, y1 - b[1]) >= 0.02, n


# -- 3. scoring: the goal and every branch ---------------------------------------------------------------------------


def test_start_scores_zero():
    s = world()
    assert M.placements(s) == (None, None) and TASK.score(s) == 0.0 and not TASK.success(s)
    assert TASK.distance(s) > 0.2


def test_goal_succeeds_with_full_score_and_zero_distance():
    for a, b in (("red", "yellow"), ("yellow", "red")):  # either kit may go to either casualty
        s = placed(a, b)
        assert TASK.success(s) and TASK.score(s) == 1.0 and TASK.distance(s) == pytest.approx(0.0)


EXPECTED = {
    ("red", "yellow"): 1.0,
    ("red", "red"): 0.6,
    ("red", None): 0.6,
    ("yellow", "yellow"): 0.4,
    ("yellow", None): 0.4,
    ("red", "green"): 0.2,
    ("yellow", "green"): 0.0,
    ("green", "green"): 0.0,
    ("green", None): 0.0,
    (None, None): 0.0,
}


@pytest.mark.parametrize("pair", sorted(EXPECTED, key=str))
def test_every_allocation_scores_as_stated(pair):
    for a, b in (pair, pair[::-1]):
        s = placed(a, b)
        assert TASK.score(s) == pytest.approx(EXPECTED[pair]), (a, b)
        assert TASK.success(s) == (sorted((a, b), key=str) == ["red", "yellow"])


def test_every_combination_is_covered():
    options = ("red", "yellow", "green", None)
    for a, b in itertools.product(options, repeat=2):
        assert tuple(sorted((a, b), key=lambda c: options.index(c))) in EXPECTED


def test_a_kit_on_its_side_or_off_centre_on_the_pad_still_counts():
    s = world(kit_outer=kit((M.PAD_X, 0.0), M.PAD_TOP, on_side=True),
              kit_inner=on_pad("yellow", dx=0.05, dy=-0.04))
    assert M.placements(s) == ("red", "yellow") and TASK.score(s) == 1.0


def test_kits_not_lying_on_a_pad_earn_nothing():
    held_above = kit((M.PAD_X, 0.0), M.PAD_TOP + M.ON_PAD_TOL + 0.005)
    on_casualty = kit((0.45, 0.0), TOP + M.BODY_H + M.TAG_SIZE[2])
    beside_pad = kit((M.PAD_X, M.CASUALTY_Y["red"] + M.PAD_HALF_W + 0.01), TOP)
    floor = ObjectState(np.array([0.5, 0.9, 0.05]), np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3),
                        np.array([0.45, 0.875, 0.0275]), np.array([0.55, 0.925, 0.0725]))
    for bad in (held_above, on_casualty, beside_pad, floor):
        s = world(kit_outer=bad, kit_inner=on_pad("yellow"))
        assert M.placements(s) == (None, "yellow") and TASK.score(s) == pytest.approx(0.4)


def test_a_kit_stacked_on_the_other_does_not_count_twice():
    s = world(kit_outer=on_pad("red"), kit_inner=kit((M.PAD_X, 0.0), M.PAD_TOP + KZ))
    assert M.placements(s) == ("red", None) and TASK.score(s) == pytest.approx(0.6)


def test_a_moving_kit_scores_but_does_not_succeed():
    s = world(kit_outer=on_pad("red"), kit_inner=kit((M.PAD_X, M.CASUALTY_Y["yellow"]), M.PAD_TOP, vel=(0.05, 0, 0)))
    assert TASK.score(s) == 1.0 and not TASK.success(s)


def test_distance_shrinks_toward_the_goal():
    half = world(kit_outer=on_pad("red"))
    assert TASK.distance(world()) > TASK.distance(half) > TASK.distance(placed("red", "yellow")) == pytest.approx(0.0)


# -- 4. construct validity: blind strategies do not reach full score --------------------------------------------------


def _nearest_pad(name):
    p = np.array([M.KIT_X, M.KIT_Y[name]])
    return min(M.CASUALTY_Y, key=lambda c: np.hypot(p[0] - M.PAD_X, p[1] - M.CASUALTY_Y[c]))


def test_the_green_pad_is_the_nearest_to_the_kits():
    centre = np.array([M.KIT_X, np.mean(list(M.KIT_Y.values()))])
    dist = {c: float(np.hypot(centre[0] - M.PAD_X, centre[1] - y)) for c, y in M.CASUALTY_Y.items()}
    assert min(dist, key=dist.get) == "green"
    assert max(dist, key=dist.get) == "yellow"  # the yellow casualty, who needs a kit, is the farthest


@pytest.mark.parametrize(
    "strategy, pair, expected",
    [
        ("the two pads nearest the kits", ("green", "red"), 0.2),
        ("each kit to its own nearest pad", (_nearest_pad("kit_outer"), _nearest_pad("kit_inner")), 0.2),
        ("each arm straight ahead (left arm to the left pad, right arm to the right pad)", ("green", "yellow"), 0.0),
        ("the first two pads from the robot's left", ("green", "red"), 0.2),
        ("both kits to the nearest casualty", ("green", "green"), 0.0),
    ],
)
def test_blind_strategies_fall_short(strategy, pair, expected):
    s = placed(*pair)
    assert TASK.score(s) == pytest.approx(expected), strategy
    assert not TASK.success(s)


def test_one_kit_to_each_casualty_is_impossible_and_the_kits_cannot_be_split():
    # Two rigid kits, three pads: every way of covering the green pad costs credit (all cases tested above).
    assert len(KITS) == 2 and len(M.CASUALTY_Y) == 3
    assert all(isinstance(o, Box) and o.free for o in M.KITS)


# -- 5. layout: no overlaps, supports, clearances, gripper fit ------------------------------------------------------


def test_objects_do_not_overlap_at_the_start():
    boxes = spec_boxes()
    for i, (a, alo, ahi) in enumerate(boxes):
        for b, blo, bhi in boxes[i + 1 :]:
            overlap = np.minimum(ahi, bhi) - np.maximum(alo, blo)
            assert np.any(overlap <= 1e-9), (a, b, overlap)


def test_everything_rests_on_the_table_or_a_fixture():
    fixed = [(n, lo, hi) for n, lo, hi in spec_boxes() if n not in KITS]
    for n, lo, _ in fixed:
        assert abs(lo[2] - TOP) < 1e-9 or any(abs(lo[2] - h[2]) < 1e-9 for _, _, h in fixed), n
    for o in M.KITS:
        bottom = o.pos[2] - o.size[2] / 2
        assert 0.0 <= bottom - TOP <= 0.002, o.name  # on the bare table
        assert not any(lo[0] <= o.pos[0] <= hi[0] and lo[1] <= o.pos[1] <= hi[1] for _, lo, hi in fixed)


def test_each_pad_belongs_to_one_casualty_and_pads_are_disjoint():
    for c, y in M.CASUALTY_Y.items():
        x0, x1, y0, y1 = M.pad_bounds(c)
        assert y0 < y < y1 and x1 < M.BODY_X0  # in front of its own casualty, short of the body
    ys = sorted(M.CASUALTY_Y.values())
    assert all(b - a > 2 * M.PAD_HALF_W + 0.05 for a, b in zip(ys, ys[1:]))  # 5 cm+ of bare table between pads
    for c in M.CASUALTY_Y:
        assert M.PAD_X1 - M.PAD_X0 >= KX + 0.02 and 2 * M.PAD_HALF_W >= KY + 0.04  # a kit fits with room


def test_twenty_mm_of_free_space_where_the_fingers_go_in():
    # Fingers close along world y (yaw 0); the open finger's outer face is 57 mm from the TCP.
    finger_outer = abs(FINGER_OPEN) + 0.0100
    ys = sorted(M.KIT_Y.values())
    for y in ys:
        for other in ys:
            if other != y:
                assert abs(other - y) - KY / 2 - finger_outer >= 0.02
    # nothing fixed within reach of the fingers around either kit
    for y in ys:
        lo = np.array([M.KIT_X - KX / 2 - 0.02, y - finger_outer - 0.02, TOP])
        hi = np.array([M.KIT_X + KX / 2 + 0.02, y + finger_outer + 0.02, TOP + 0.1])
        for n, blo, bhi in spec_boxes():
            if n not in KITS:
                assert np.any(np.minimum(hi, bhi) - np.maximum(lo, blo) <= 0), n


def test_kits_fit_the_gripper_with_a_centimetre_to_spare():
    assert KY + 0.01 <= 2 * abs(FINGER_OPEN) <= GRIPPER_MAX_GAP + 1e-6
    assert KY <= USABLE_GRIP - 0.01
    # gripped at GRIP_ABOVE_BOTTOM: the pads (27 mm from 13 mm below the TCP) sit on the kit, the tips above the
    # table, and the finger carriage (54 mm above the TCP) clears the kit's top
    assert M.GRIP_ABOVE_BOTTOM - PAD_TIP_PAST_TCP >= 0.01
    assert M.GRIP_ABOVE_BOTTOM + 0.014 <= KZ
    assert M.GRIP_ABOVE_BOTTOM + 0.054 > KZ + 0.02


# -- 6. the top camera sees what is scored in the first and last frames ---------------------------------------------

CAM = station.top_camera("calibrated")


def _project(points):
    rot = quat_xyzw_to_matrix(np.asarray(CAM.rot_xyzw))
    c = (np.asarray(points) - np.asarray(CAM.pos)) @ rot
    depth = -c[:, 2]
    lens = CAM.lens
    return np.c_[lens.cx + lens.fx * c[:, 0] / depth, lens.cy - lens.fy * c[:, 1] / depth], depth


def _hull2d(p):
    pts = sorted(map(tuple, np.round(p, 6)))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.array(lower[:-1] + upper[:-1])


def _inside(poly, pts):
    ok = np.ones(len(pts), bool)
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        ok &= (b[0] - a[0]) * (pts[:, 1] - a[1]) - (b[1] - a[1]) * (pts[:, 0] - a[0]) >= 0
    return ok


def _hidden(points, hulls):
    uv, depth = _project(points)
    hidden = np.zeros(len(points), bool)
    for _, h in hulls:
        huv, hdepth = _project(h.vertices)
        hidden |= _inside(_hull2d(huv), uv) & (hdepth.min() < depth)
    return hidden


def _top_points(lo, hi, n=7):
    g = np.linspace(0, 1, n)
    return np.array([[lo[0] + a * (hi[0] - lo[0]), lo[1] + b * (hi[1] - lo[1]), hi[2]] for a in g for b in g])


def test_the_top_camera_sees_tags_pads_and_kits_in_the_first_and_last_frames(played):
    ready = _hulls("left", READY, FINGER_OPEN) + _hulls("right", READY, FINGER_OPEN)
    last = {side: steps[-1] for side, (_, steps) in played.items()}
    parked = _hulls("left", last["left"][1], last["left"][2]) + _hulls("right", last["right"][1], last["right"][2])
    fixed = {n: (lo, hi) for n, lo, hi in spec_boxes()}
    scored = [fixed[f"tag_{c}"] for c in M.CASUALTY_Y] + [fixed[f"pad_{c}"] for c in M.CASUALTY_Y]
    start_kits = [(s.low, s.high) for s in start_states().values()]
    end_kits = [(s.low, s.high) for s in (on_pad("red"), on_pad("yellow"))]
    for frame, hulls, kits in (("first", ready, start_kits), ("last", parked, end_kits)):
        for lo, hi in [*scored, *kits]:
            pts = _top_points(lo, hi)
            uv, _ = _project(pts)
            assert np.all(uv[:, 0] > 10) and np.all(uv[:, 0] < CAM.lens.width - 10), frame
            assert np.all(uv[:, 1] > 10) and np.all(uv[:, 1] < CAM.lens.height - 10), frame
            assert not _hidden(pts, hulls).any(), (frame, lo, hi)


def test_tags_are_large_enough_to_read_at_320_x_176():
    # At 320 x 176 the image is half the 640 x 360 render: each tag should span at least 6 px each way.
    fixed = {n: (lo, hi) for n, lo, hi in spec_boxes()}
    for c in M.CASUALTY_Y:
        lo, hi = fixed[f"tag_{c}"]
        uv, _ = _project(_top_points(lo, hi, 3))
        span = (uv.max(0) - uv.min(0)) / 2
        assert span.min() >= 6, (c, span)


# -- task.json ------------------------------------------------------------------------------------------------------


def test_task_json_has_every_field_and_no_names():
    meta = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "fidelity", "scores"):
        assert meta.get(key), key
    # scores is filled after the Dream-1 run and media by the upload script
    assert meta["scores"] == "not yet run" or meta["scores"].startswith("Dream-1 run: "), meta["scores"]
    assert meta["category"] == "Societal relevance"
    text = json.dumps(meta).lower() + TASK.docs.lower() + (HERE / "README.md").read_text().lower()
    for word in ("robocurve sprint", "our team", "university", "laboratory"):
        assert word not in text, word
