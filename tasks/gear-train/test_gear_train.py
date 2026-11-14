"""Checks of gear_train.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_gear_train.py
"""

import functools
import math
import types
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.analysis import fk_link_poses_world
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("gear_train.py"))
M = TASK.module
GEARS = M.GEARS
THICK = 0.025


def gear(name, x, y, bottom=None, yaw_deg=0.0, tilt_deg=0.0, moving=False, v=None, w=None):
    """A gear centred at (x, y), the centre of its bottom face at ``bottom`` (default: lying on the table), turned
    ``yaw_deg`` about the vertical, then tilted ``tilt_deg`` about its own x. Its box is the gear's own box turned
    with it, as Arena reports it (rules.md: get_aabb_w). ``v`` and ``w`` set its linear (m/s) and angular (rad/s)
    velocity; ``moving`` is v = (0.05, 0, 0)."""
    a, b = math.radians(yaw_deg) / 2, math.radians(tilt_deg) / 2
    q = np.array([math.cos(a) * math.sin(b), math.sin(a) * math.sin(b), math.sin(a) * math.cos(b),
                  math.cos(a) * math.cos(b)])
    qx, qy, qz, qw = q
    rot = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                    [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                    [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    r = M.DIAMETER[name] / 2
    low_z = M.TOP + 0.001 if bottom is None else bottom
    centre = np.array([x, y, low_z + rot[2, 2] * THICK / 2])
    half = np.abs(rot) @ np.array([r, r, THICK / 2])
    lin = np.array([0.05, 0, 0]) if moving else np.zeros(3) if v is None else np.asarray(v, dtype=float)
    ang = np.zeros(3) if w is None else np.asarray(w, dtype=float)
    return ObjectState(pos=centre, quat_xyzw=q, lin_vel=lin, ang_vel=ang, low=centre - half, high=centre + half)


def start():
    return {n: gear(n, *M.START[n]) for n in GEARS}


def seated_gear(name, **kw):
    return gear(name, *M.shaft(name), bottom=M.SEAT, **kw)


def layout(**where):
    o = start()
    o.update(where)
    return o


ALL_SEATED = {n: seated_gear(n) for n in GEARS}


# -- the arms, by the URDF's forward kinematics --------------------------------------------------------------------
def arm_poses(side, q6, finger):
    q = {f"joint{i + 1}": float(q6[i]) for i in range(6)}
    q["joint7"] = q["joint8"] = float(finger)
    world = fk_link_poses_world(q, station.BASES[side])
    return {k: world[k] for k in M.ARM_LINKS}


READY_ARMS = {side: arm_poses(side, READY, FINGER_OPEN) for side in ("left", "right")}
DEMO = M.demo()["right"]
DEMO_END = np.cumsum([w.duration for w in DEMO])


@functools.lru_cache(maxsize=None)
def _demo_right(i):
    """The right arm's body poses when the demo's waypoint ``i`` ends (IK stepped every 0.05 s, as the demo plays)."""
    plan = ArmPlan(DEMO, station.BASES["right"])
    t_end = float(DEMO_END[i])
    for t in np.arange(0.0, t_end, 0.05):
        plan.action7(float(t))
    a, _ = plan.action7(t_end)
    return arm_poses("right", a[:6], plan.target(t_end)[2])


def demo_hand(label, nth=-1):
    """Both arms when the demo's right-arm waypoint ``label`` (its ``nth`` use, default the last) ends; the left at
    READY."""
    i = [k for k, w in enumerate(DEMO) if w.label == label][nth]
    return {"left": READY_ARMS["left"], "right": _demo_right(i)}


def shifted(hand, d):
    return {k: np.vstack([np.c_[t[:3, :3], t[:3, 3] + np.asarray(d, float)], [0, 0, 0, 1.0]]) for k, t in hand.items()}


@pytest.fixture(autouse=True)
def arms_parked():
    """Unless a test says otherwise, the hook saw both arms at READY, far from the gears."""
    M._HANDS["poses"] = READY_ARMS
    yield
    M._HANDS["poses"] = M._HANDS["memo"] = None


def test_the_start_scores_zero():
    o = start()
    assert M.score(o) == 0 and not M.success(o)


def test_no_gear_starts_nearest_its_own_shaft():
    for n in GEARS:
        d = {s: np.linalg.norm(np.array(M.START[n]) - M.shaft(s)) for s in GEARS}
        assert min(d, key=d.get) != n


def test_all_seated_succeeds_but_score_stays_below_1():
    # the trial pays 1.0 only when success() has held for 1 s; score() at the last step never does
    assert M.success(ALL_SEATED) and M.score(ALL_SEATED) == pytest.approx(0.8)


def test_score_never_reaches_1():
    states = [start(), ALL_SEATED, dict(ALL_SEATED, small_gear=seated_gear("small_gear", moving=True))]
    states += [layout(**{n: seated_gear(n) for n in pair}) for pair in M.PAIRS]
    assert max(M.score(o) for o in states) == pytest.approx(0.8)
    M._HANDS["poses"] = demo_hand("settle low")  # held: below the let-go train
    assert max(M.score(o) for o in states) == pytest.approx(0.6)


@pytest.mark.parametrize("name", GEARS)
def test_one_seated_gear_scores_0_1(name):
    assert M.score(layout(**{name: seated_gear(name)})) == pytest.approx(0.1)


def test_large_and_small_seated_without_the_medium_have_no_mesh():
    # the large and small shafts are too far apart to mesh: two gears down, no phase matched
    o = layout(large_gear=seated_gear("large_gear"), small_gear=seated_gear("small_gear"))
    assert M.score(o) == pytest.approx(0.2)


@pytest.mark.parametrize("pair", M.PAIRS)
def test_one_meshed_pair_scores_0_5(pair):
    assert M.score(layout(**{n: seated_gear(n) for n in pair})) == pytest.approx(0.5)


@pytest.mark.parametrize("name", GEARS)
@pytest.mark.parametrize("proud_mm, credit", [(-1.1, 0.0), (0.0, 0.1), (0.9, 0.1), (-0.9, 0.1), (1.1, 0.0),
                                              (8.0, 0.0), (19.0, 0.0)])
def test_height_on_the_shaft(name, proud_mm, credit):
    """Seated within 1 mm either way; stopped on teeth (7 to 10 mm proud) or resting on the tip (about 19 mm proud)
    scores nothing."""
    g = gear(name, *M.shaft(name), bottom=M.SEAT + proud_mm / 1000)
    assert M.score(layout(**{name: g})) == pytest.approx(credit)


@pytest.mark.parametrize("off_mm, credit", [(1.9, 0.1), (2.1, 0.0)])
def test_off_axis(off_mm, credit):
    x, y = M.shaft("medium_gear")
    g = gear("medium_gear", x + off_mm / 1000, y, bottom=M.SEAT)
    assert M.score(layout(medium_gear=g)) == pytest.approx(credit)


def test_a_gear_on_the_wrong_shaft_scores_nothing():
    o = layout(large_gear=gear("large_gear", *M.shaft("small_gear"), bottom=M.SEAT),
               small_gear=gear("small_gear", *M.shaft("large_gear"), bottom=M.SEAT))
    assert M.score(o) == 0


@pytest.mark.parametrize("tilt, credit", [(2.5, 0.1), (3.5, 0.0)])
def test_a_tilted_gear_is_not_seated(tilt, credit):
    assert M.score(layout(small_gear=seated_gear("small_gear", tilt_deg=tilt))) == pytest.approx(credit)


def test_yaw_does_not_matter():
    assert M.success({n: seated_gear(n, yaw_deg=37.0) for n in GEARS})


@pytest.mark.parametrize("name", GEARS)
def test_success_needs_every_gear_still(name):
    o = dict(ALL_SEATED, **{name: seated_gear(name, moving=True)})
    assert not M.success(o) and M.score(o) == pytest.approx(0.7)


# resting(): speed under 0.01 m/s and spin under 0.5 rad/s (yam_arena task_file.py), checked on both signs of each
# axis, including a gear spinning about its own shaft (z)
@pytest.mark.parametrize("axis", range(3))
@pytest.mark.parametrize("sign", (1.0, -1.0))
@pytest.mark.parametrize("kind, under, over", [("v", 0.009, 0.011), ("w", 0.49, 0.51)])
def test_stillness_limits_both_signs(axis, sign, kind, under, over):
    for mag, still in ((under, True), (over, False)):
        vec = np.zeros(3)
        vec[axis] = sign * mag
        o = dict(ALL_SEATED, medium_gear=seated_gear("medium_gear", **{kind: vec}))
        assert M.success(o) is still
        assert M.score(o) == pytest.approx(0.8 if still else 0.7)


@pytest.mark.parametrize("jammed", GEARS)
def test_two_seated_one_jammed_is_not_success(jammed):
    o = dict(ALL_SEATED, **{jammed: gear(jammed, *M.shaft(jammed), bottom=M.SEAT + 0.008)})
    # jammed medium: two gears down, no mesh; jammed end gear: one mesh
    assert not M.success(o) and M.score(o) == pytest.approx(0.2 if jammed == "medium_gear" else 0.5)


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["right"], station.BASES["right"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert plan.duration < 2400 / 20


@pytest.mark.parametrize("delay", (0.05, 0.10))
def test_start_delay_only_lengthens_the_first_move(delay):
    base, late = M.demo()["right"], M.demo(delay)["right"]
    assert late[0].duration == pytest.approx(base[0].duration + delay)
    assert late[1:] == base[1:]


def test_task_json_text_limits_and_ascii():
    import json

    here = Path(__file__).parent
    t = json.loads(here.joinpath("task.json").read_text())
    assert len(t["title"]) <= 120 and len(t["fidelity"]) <= 500 and len(t["context"]) <= 1000
    for f in ("task.json", "README.md", "gear_train.py", "test_gear_train.py"):
        assert here.joinpath(f).read_text().isascii(), f
    # the scoring text states the last-step score when the limit ends before the 1 s hold
    assert "0.8" in t["scoring"] and "0.7" in t["scoring"]


def test_shafts_where_the_setup_says():
    # 28.0, 33.0 and 36.1 cm out, 15 cm right; 50.6 and 30.5 mm apart (setup and README)
    want = {"large_gear": (0.2797, -0.15), "medium_gear": (0.3303, -0.15), "small_gear": (0.3608, -0.15)}
    for n, xy in want.items():
        assert M.shaft(n) == pytest.approx(np.array(xy), abs=1e-9)


@pytest.mark.parametrize("bottom_mm, credit", [(4.9, 0.0), (5.1, 0.1), (6.9, 0.1), (7.1, 0.0)])
def test_seat_height_is_6_mm_above_the_table(bottom_mm, credit):
    g = gear("large_gear", *M.shaft("large_gear"), bottom=M.TOP + bottom_mm / 1000)
    assert M.score(layout(large_gear=g)) == pytest.approx(credit)


def test_no_delay_by_default():
    assert all(w.duration == 2.0 for w in M.demo()["right"] if w.label.startswith("to "))


@pytest.mark.parametrize("name", GEARS)
@pytest.mark.parametrize("yaw", (0.0, 45.0, -45.0, 135.0))
@pytest.mark.parametrize("tilt", (2.9, -2.9))
def test_a_tilted_gear_is_judged_by_its_bottom_face_not_its_box_corner(name, yaw, tilt):
    """The turned box's lowest corner drops by up to r sin(tilt) sqrt(2) (2.3 mm for the large gear at 3 deg): a
    tilted gear proud by more than 1 mm must not pass, and a seated one tilted within 3 deg must."""
    x, y = M.shaft(name)
    proud = gear(name, x, y, bottom=M.SEAT + 0.0012, yaw_deg=yaw, tilt_deg=tilt)
    home = gear(name, x, y, bottom=M.SEAT + 0.0008, yaw_deg=yaw, tilt_deg=tilt)
    assert proud.low[2] < M.SEAT + 0.001  # the corner alone would read seated
    assert not M.seated({name: proud}, name)
    assert M.seated({name: home}, name)


def test_the_hunted_state_is_not_success():
    # hole hunt 2026-11-12: large gear tilted 2.99 deg at yaw 45, bottom face 3.19 mm proud, scored 0.8 and success
    o = dict(ALL_SEATED, large_gear=gear("large_gear", *M.shaft("large_gear"), bottom=M.SEAT + 0.00319,
                                         yaw_deg=45.0, tilt_deg=2.99))
    assert not M.success(o) and M.score(o) == pytest.approx(0.5)  # medium and small still mesh


# -- the hands: let go and clear -----------------------------------------------------------------------------------
def test_arms_at_ready_are_clear_of_the_start_and_the_seated_train():
    for o in (start(), ALL_SEATED):
        assert M.hand_clearance(o, READY_ARMS) == {"left": M.CLEAR_CAP, "right": M.CLEAR_CAP}


def test_no_arm_reading_is_not_success():
    """If the hook has not run since the reset (or is not wired), the hands count as not clear."""
    M._HANDS["poses"] = None
    assert M.hand_clearance(ALL_SEATED) is None
    assert not M.hands_clear(ALL_SEATED) and not M.success(ALL_SEATED)
    assert M.score(ALL_SEATED) == pytest.approx(M.ALL_HELD)


@pytest.mark.parametrize("moving", (False, True))
def test_a_seated_train_still_gripped_is_not_success_and_pays_0_6(moving):
    """The reviewer's hole (#914, same scorer shape): all three seated while the fingers still close on the small
    gear. Still or moving, a held train pays 0.6, under the let-go 0.8 / 0.7 and over one meshed pair (0.5)."""
    o = dict(ALL_SEATED, small_gear=seated_gear("small_gear", moving=moving))
    M._HANDS["poses"] = demo_hand("settle low")
    assert M.hand_clearance(o)["right"] <= 0.0
    assert not M.success(o) and M.score(o) == pytest.approx(0.6)
    assert 0.5 < M.ALL_HELD < M.ALL_MOVING < M.ALL_STILL < 1.0


@pytest.mark.parametrize("name", GEARS)
def test_a_closed_grip_at_each_pick_is_not_clear(name):
    """Closed on each gear where it lies (the demo's grips): no clearance from that gear."""
    gaps = M.hand_clearance(start(), demo_hand("grip", GEARS.index(name)))  # the demo picks in GEARS order
    assert gaps["right"] == pytest.approx(-M.SAMPLE) and gaps["left"] == M.CLEAR_CAP  # overlap reads -SAMPLE


def test_the_demo_lets_go_open_then_backs_off_clear():
    """At the end of the last release the fingers' target is fully open; at the end of "let go" the hand stands at
    least 10 mm off every gear; at the end of "back off" it is past the 50 mm cap."""
    assert DEMO[[w.label for w in DEMO].index("let go") - 1].finger == FINGER_OPEN
    let_go = M.hand_clearance(ALL_SEATED, demo_hand("let go"))
    assert let_go["right"] >= M.HAND_CLEAR and let_go["left"] == M.CLEAR_CAP
    assert M.hand_clearance(ALL_SEATED, demo_hand("back off")) == {"left": M.CLEAR_CAP, "right": M.CLEAR_CAP}
    M._HANDS["poses"] = demo_hand("back off")
    assert M.success(ALL_SEATED) and M.score(ALL_SEATED) == pytest.approx(0.8)


def test_only_the_last_release_changed():
    """The first two inserts end as before (release, then up 1.0 s); only the small gear's adds a fully open wait
    and a slower back-off."""
    labels = [w.label for w in DEMO]
    assert labels.count("release") == 3 and labels.count("up") == 2
    assert labels[-3:] == ["release", "let go", "back off"]
    assert labels.count("let go") == 1 and labels.count("back off") == 1
    for i, w in enumerate(DEMO):
        if w.label == "up":
            assert DEMO[i - 1].label == "release" and w.duration == 1.0 and w.finger == FINGER_OPEN
    let_go, back = DEMO[-2], DEMO[-1]
    assert let_go.pos == DEMO[-3].pos and let_go.finger == back.finger == FINGER_OPEN
    assert back.pos[:2] == DEMO[-3].pos[:2] and back.pos[2] == pytest.approx(M.TOP + 0.10)


def hand_over(name, h):
    """The demo's hand closed on the seated small gear ("settle low"), carried over gear ``name``'s shaft and raised
    ``h``: at h = 0 its closed pads sit inside that gear, as they sat inside the small one."""
    d = np.r_[M.shaft(name) - M.shaft("small_gear"), h]
    return shifted(demo_hand("settle low")["right"], d)


def _hand_at_gap(target, side):
    """That hand over the large gear, raised until its clearance is ``target`` (bisection), carried by ``side``; the
    other arm at READY."""
    lo, hi = 0.0, 0.08  # h: lo is too near, hi too far
    for _ in range(40):
        mid = (lo + hi) / 2
        g = M.hand_clearance(ALL_SEATED, {**READY_ARMS, side: hand_over("large_gear", mid)})[side]
        lo, hi = (lo, mid) if g > target else (mid, hi)
    return {**READY_ARMS, side: hand_over("large_gear", hi)}


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("gap_mm, clear", [(7.5, False), (9.5, False), (9.9, False), (10.1, True), (10.5, True),
                                           (12.5, True)])
def test_the_10_mm_clearance_either_side_of_the_limit_for_either_arm(side, gap_mm, clear):
    M._HANDS["poses"] = arms = _hand_at_gap(gap_mm / 1000, side)
    other = "left" if side == "right" else "right"
    assert M.hand_clearance(ALL_SEATED)[side] == pytest.approx(gap_mm / 1000, abs=5e-5)
    assert M.hand_clearance(ALL_SEATED)[other] == M.CLEAR_CAP
    assert M.hands_clear(ALL_SEATED) is clear and M.success(ALL_SEATED) is clear
    assert M.score(ALL_SEATED) == pytest.approx(0.8 if clear else 0.6)
    moving = dict(ALL_SEATED, large_gear=seated_gear("large_gear", moving=True))
    assert M.score(moving) == pytest.approx(0.7 if clear else 0.6)
    assert arms[other] is READY_ARMS[other]


@pytest.mark.parametrize("name", GEARS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_a_hand_on_any_gear_by_either_arm_is_not_clear(name, side):
    """The closed hand put on each gear as it held the small one, by either arm, the other arm parked."""
    M._HANDS["poses"] = {**READY_ARMS, side: hand_over(name, 0.0)}
    assert M.hand_clearance(ALL_SEATED)[side] <= 0.0
    assert not M.success(ALL_SEATED) and M.score(ALL_SEATED) == pytest.approx(0.6)


def _point_to_cylinder(p, r, length):
    """Exact distance from points (cylinder frame, axis z, centred) to a solid cylinder."""
    a = np.abs(p[:, 2]) - length / 2
    rad = np.linalg.norm(p[:, :2], axis=1) - r
    return np.linalg.norm(np.maximum(np.c_[a, rad], 0.0), axis=1) + np.minimum(np.maximum(a, rad), 0.0)


@pytest.mark.parametrize("label", ["let go", "back off", "above", "over shaft"])
def test_the_clearance_is_never_more_than_the_true_gap(label):
    """Its claim is a lower bound: no hull vertex may lie nearer the gears' cylinders than the reported clearance (a
    vertex's exact distance is at least the true gap to the hull)."""
    o = ALL_SEATED
    arms = demo_hand(label)
    gap = M.hand_clearance(o, arms)["right"]
    exact = np.inf
    for link, t in arms["right"].items():
        for verts, _ in M._arm_hulls()[link]:
            with np.errstate(all="ignore"):  # WHY: spurious matmul FP flags from macOS numpy (checked finite)
                v = verts @ t[:3, :3].T + t[:3, 3]
            assert np.isfinite(v).all()
            for n in GEARS:
                c = (o[n].low + o[n].high) / 2
                with np.errstate(all="ignore"):
                    local = (v - c) @ M._rot(o[n].quat_xyzw)
                assert np.isfinite(local).all()
                exact = min(exact, float(_point_to_cylinder(local, M.TIP_R[n], M.OVERALL).min()))
    assert 0.0 < gap <= min(exact, M.CLEAR_CAP)
    assert min(exact, M.CLEAR_CAP) - gap < 0.012  # and not uselessly loose here


def test_the_laid_points_cover_every_gear_cylinder_within_SAMPLE():
    rng = np.random.default_rng(0)
    for n in GEARS:
        pts, r, length = M._GEAR_POINTS[n], M.TIP_R[n], M.OVERALL
        t = rng.uniform(0, 2 * math.pi, 400)
        side = np.c_[r * np.cos(t), r * np.sin(t), rng.uniform(-length / 2, length / 2, 400)]
        rr = r * np.sqrt(rng.uniform(0, 1, 400))
        cap = np.c_[rr * np.cos(t), rr * np.sin(t), rng.choice([-length / 2, length / 2], 400)]
        q = np.vstack([side, cap])
        nearest = np.sqrt(((q[:, None, :] - pts[None, :, :]) ** 2).sum(-1)).min(axis=1)
        assert nearest.max() <= M.SAMPLE, (n, nearest.max())
        assert np.abs(pts[:, 2]).max() == pytest.approx(length / 2)
        assert np.linalg.norm(pts[:, :2], axis=1).max() == pytest.approx(r)


def test_the_cylinders_are_the_measured_sim_gears():
    # README measured table (tools/measure_assets.py on the instance): tip 62.05, 42.08, 21.97 mm; overall 25.02,
    # 25.05, 24.97 mm. The cylinder must contain each gear
    tip = {"large_gear": 62.05, "medium_gear": 42.08, "small_gear": 21.97}
    for n in GEARS:
        assert M.TIP_R[n] * 2000 == pytest.approx(tip[n], abs=1e-9)
    assert M.OVERALL * 1000 >= max(25.02, 25.05, 24.97)


def test_the_cylinder_turns_with_the_gear_and_sits_in_its_box_middle():
    """A large gear tilted onto its side: its points span the tip diameter vertically, about the box's middle."""
    g = gear("large_gear", 0.30, -0.30, bottom=M.TOP + 0.04, tilt_deg=90.0)
    pts = M.gear_points(dict(ALL_SEATED, large_gear=g))[:len(M._GEAR_POINTS["large_gear"])]
    mid = (g.low + g.high) / 2
    assert 2 * M.TIP_R["large_gear"] - 1e-4 < np.ptp(pts[:, 2]) <= 2 * M.TIP_R["large_gear"] + 1e-9
    assert pts.mean(axis=0) == pytest.approx(mid, abs=1e-6)


def test_every_arm_body_is_watched():
    assert set(M.ARM_LINKS) == {"base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left",
                                "tip_right"}
    assert set(M.ARMS) == {"left", "right"}
    assert M.HAND_CLEAR == 0.010


# -- the hook, on fake simulator objects ---------------------------------------------------------------------------
def quat(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return np.array([x, y, z, w])


def fake_env(arms, fingers=(FINGER_OPEN, FINGER_CLOSED), origin=(4.0, -2.0, 0.0)):
    """An Isaac Lab-like env: scene[left_robot / right_robot] with body poses (world, offset by the env origin) and
    joints, in a shuffled order, as numpy arrays."""
    origin = np.asarray(origin, float)
    scene = {"env_origins": np.array([origin])}
    for (side, key), f in zip(M.ARMS.items(), fingers):
        names = list(M.ARM_LINKS)[::-1] + ["camera_mount"]
        poses = np.array([np.r_[arms[side][n][:3, 3] + origin, quat(arms[side][n][:3, :3])] if n in arms[side]
                          else np.r_[origin, 0, 0, 0, 1.0] for n in names])
        joints = ["joint8", *[f"joint{i}" for i in range(1, 7)], "joint7"]
        data = types.SimpleNamespace(body_link_pose_w=poses[None], joint_pos=np.array([[f] * 8]))
        scene[key] = types.SimpleNamespace(body_names=names, joint_names=joints, data=data)

    class Scene(dict):
        env_origins = scene["env_origins"]

    return types.SimpleNamespace(unwrapped=types.SimpleNamespace(scene=Scene(scene)))


def test_read_arms_returns_task_frame_poses_and_fingers():
    arms = demo_hand("settle low")
    poses, finger = M.read_arms(fake_env(arms))
    for side in ("left", "right"):
        for n in M.ARM_LINKS:
            assert np.allclose(poses[side][n], arms[side][n], atol=1e-9), (side, n)
    assert finger == {"left": [FINGER_OPEN, FINGER_OPEN], "right": [FINGER_CLOSED, FINGER_CLOSED]}


@pytest.mark.parametrize("key", ["left_robot", "right_robot"])
def test_read_arms_refuses_a_missing_body(key):
    env = fake_env(READY_ARMS)
    env.unwrapped.scene[key].body_names[0] = "renamed"
    with pytest.raises(ValueError):
        M.read_arms(env)


def test_the_hook_runs_before_each_scored_step_and_reset_forgets():
    """arm_watch.py wraps the embodiment's _task_step (the step that reads the states and calls success/score) so the
    registered hooks see the env first, and its reset so the reading is forgotten."""
    import arm_watch

    seen = []

    class Embodiment:
        def __init__(self):
            self._env = fake_env(demo_hand("settle low"))

        def reset(self, scene, *, seed=None):
            return "obs"

        def step(self, action):
            return self._task_step()

        def _task_step(self):
            seen.append((M.success(ALL_SEATED), M.score(ALL_SEATED)))
            return "result"

    module = types.SimpleNamespace(YamArenaEmbodiment=Embodiment)
    arm_watch._patch_embodiment(module)
    assert M._watch_arms in arm_watch._STEP and M._forget_arms in arm_watch._RESET
    M._HANDS["poses"] = READY_ARMS  # a stale reading from before: the hook must replace it
    e = Embodiment()
    assert e.step(None) == "result"
    assert seen == [(False, pytest.approx(0.6))]  # the gripping hand, read by the hook, not the stale parked one
    e._env = fake_env(demo_hand("back off"))
    e.step(None)
    assert seen[-1] == (True, pytest.approx(0.8))
    assert e.reset(None) == "obs"
    assert M._HANDS["poses"] is None and M._HANDS["memo"] is None
    assert not M.success(ALL_SEATED) and M.score(ALL_SEATED) == pytest.approx(0.6)  # unknown after the reset
    arm_watch._patch_embodiment(module)  # patching again does not stack the hooks
    e._env = fake_env(demo_hand("back off"))
    n = M._HANDS["step"]
    e.step(None)
    assert M._HANDS["step"] == n + 1


def test_the_memo_is_per_step_and_per_gear_pose():
    """success(), score() and a logging copy share one clearance per step; a new step or moved gear recomputes."""
    M._HANDS["poses"], M._HANDS["step"] = demo_hand("back off"), 0
    assert M.hands_clear(ALL_SEATED)
    M._HANDS["poses"], M._HANDS["step"] = demo_hand("settle low"), 1  # next step, new reading
    assert not M.hands_clear(ALL_SEATED)
    held = M._HANDS["poses"]
    M._HANDS["poses"] = {k: dict(v) for k, v in held.items()}  # same step count, a new reading object
    assert not M.hands_clear(ALL_SEATED)
    M._HANDS["poses"] = demo_hand("let go")  # open around the seated small gear, about 16 mm off the train
    assert M.hand_clearance(ALL_SEATED)["right"] < 0.02
    assert M.hand_clearance(start())["right"] == M.CLEAR_CAP  # same step, same reading, the gears back at the start
