"""Checks of tie_rod_end.py's scoring, layout and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_tie_rod_end.py
"""

import importlib.util
import math
import types
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.analysis import fk_link_poses_world
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).resolve().parent
TASK = load(HERE / "tie_rod_end.py")
M = TASK.module
MECH = M.TIE_ROD
BALL_M = np.array([0.0, -M.L, M.JOINT_Z])  # the ball centre in the mechanism's frame at rest


def _spec(name):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


MESHES = _spec("make_meshes")


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def quat(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return np.array([x, y, z, w])


def link(pos, r=np.eye(3), moving=False, lin_vel=None, ang_vel=None):
    pos = np.asarray(pos, float)
    v = np.array([0.05, 0, 0]) if moving else np.zeros(3)
    return ObjectState(pos=pos, quat_xyzw=quat(r), lin_vel=v if lin_vel is None else np.asarray(lin_vel, float),
                       ang_vel=np.zeros(3) if ang_vel is None else np.asarray(ang_vel, float), low=pos, high=pos)


JOINT_W = np.array([*M.INNER, M.TOP + M.JOINT_Z])  # the rack's inner joint, world


def rod_link(yaw, rise, moving=False):
    """The rod turned ``yaw`` about the inner joint and its outer end ``rise`` above it (as ``M.ball_at``). A link's
    frame origin is the mechanism's origin carried by the link."""
    r = rz(yaw) @ rx(math.asin(-rise / M.L))
    return link(JOINT_W + r @ (M.MECH_ORIGIN - JOINT_W), r, moving)


def state(ball, r=np.eye(3), stud_moving=False, rod_moving=False, stud_vel=None, rod=None):
    """The tie rod with the stud's ball centre at ``ball`` (world) and the stud turned by ``r`` from hanging straight
    down at the start. ``rod`` is (yaw, rise) for the rod's pose (as ``M.ball_at``); by default it is at rest, which
    matters only for its speed and the hands' clearance."""
    origin = np.asarray(ball, float) - r @ BALL_M
    m = M.mechanism  # noqa: F841 (the module is loaded)
    st = ObjectState(pos=M.MECH_ORIGIN.copy(), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                     ang_vel=np.zeros(3), low=M.MECH_ORIGIN.copy(), high=M.MECH_ORIGIN.copy())
    from mechanism import MechanismState
    lin, ang = stud_vel if stud_vel else (None, None)
    rod_state = link(M.MECH_ORIGIN, moving=rod_moving) if rod is None else rod_link(*rod, moving=rod_moving)
    links = {"stud": link(origin, r, stud_moving, lin, ang), "rod": rod_state}
    return {"tie_rod": MechanismState(pos=st.pos, quat_xyzw=st.quat_xyzw, lin_vel=st.lin_vel, ang_vel=st.ang_vel,
                                      low=st.low, high=st.high, joints={}, links=links)}


SEAT_BALL = M.ball_at(M.SEAT_YAW, -M.SEAT_DROP)
SEAT_R = rz(M.SEAT_YAW)


def seated_at(dx=0.0, dy=0.0, dz=0.0, r=SEAT_R, **kw):
    """The stud seated, its ball moved by (dx, dy, dz) and the stud turned by ``r``; the rod in its seated pose."""
    kw.setdefault("rod", (M.SEAT_YAW, -M.SEAT_DROP))
    return state(SEAT_BALL + np.array([dx, dy, dz]), r, **kw)


# -- the arms, by the URDF's forward kinematics --------------------------------------------------------------------
def arm_poses(side, q6, finger):
    q = {f"joint{i + 1}": float(q6[i]) for i in range(6)}
    q["joint7"] = q["joint8"] = float(finger)
    world = fk_link_poses_world(q, station.BASES[side])
    return {k: world[k] for k in M.ARM_LINKS}


def plan_poses(waypoints, t_end, side="right"):
    """The arm's body poses at time ``t_end`` of a waypoint plan (IK stepped every 0.05 s, as the demo plays it)."""
    plan = ArmPlan(waypoints, station.BASES[side])
    for t in np.arange(0.0, t_end, 0.05):
        plan.action7(float(t))
    a, _ = plan.action7(t_end)
    return arm_poses(side, a[:6], plan.target(t_end)[2])


READY_ARMS = {side: arm_poses(side, READY, FINGER_OPEN) for side in ("left", "right")}
DEMO = M.demo()["right"]
DEMO_END = dict(zip(range(len(DEMO)), np.cumsum([w.duration for w in DEMO])))


def demo_hand(label, nth=0):
    """Both arms when the demo's right-arm waypoint ``label`` (its ``nth`` use) ends; the left at READY."""
    i = [k for k, w in enumerate(DEMO) if w.label == label][nth]
    return {"left": READY_ARMS["left"], "right": plan_poses(DEMO, float(DEMO_END[i]))}


def shifted(arms, side, dz):
    return {**arms, side: {k: np.vstack([np.c_[t[:3, :3], t[:3, 3] + [0, 0, dz]], [0, 0, 0, 1.0]])
                           for k, t in arms[side].items()}}


@pytest.fixture(autouse=True)
def arms_parked():
    """Unless a test says otherwise, the hook saw both arms at READY, far from the tie rod."""
    M._HANDS["poses"] = READY_ARMS
    yield
    M._HANDS["poses"] = None


def tilted(deg, about=rx):
    """Seated, but the stud turned ``deg`` about its taper's large end (kept on the hole's axis)."""
    top_m = M.TAPER_TOP_M - BALL_M
    r = SEAT_R @ about(math.radians(deg))
    ball = SEAT_BALL + SEAT_R @ top_m - r @ top_m
    return state(ball, r)


def test_the_start_scores_zero():
    o = state(M.START_BALL)
    assert M.score(o) == 0 and not M.success(o)


def test_the_hole_is_where_the_tie_rod_reaches_it_level():
    assert np.allclose(SEAT_BALL[:2], M.HOLE, atol=1e-9)
    top, _, tilt = M.stud_pose(seated_at())
    assert np.linalg.norm(top[:2] - M.HOLE) < 1e-9 and tilt < 1e-6
    assert abs((M.HOLE_TOP_Z - top[2]) - M.SINK) < 0.0001


def test_the_knuckle_mesh_matches_the_task():
    assert abs(MESHES.BOSS_TOP + MESHES.REST - M.BOSS_TOP) < 1e-9
    assert (MESHES.TAPER_LARGE_R, MESHES.TAPER_SMALL_R, MESHES.TAPER_L) == (M.TAPER_LARGE_R, M.TAPER_SMALL_R, M.TAPER_L)
    knuckle = next(o for o in TASK.objects if o.name == "knuckle")
    b = MESHES.bounds()
    centre = (np.array(b["lo"][:2]) + np.array(b["hi"][:2])) / 2
    assert np.allclose(np.array(knuckle.xy) - centre, M.HOLE)


def test_the_stud_seats_inside_the_taper():
    # each step's corner meets the cone at the seat, and the thread passes the hole's small end
    assert M.THREAD_R < M.TAPER_SMALL_R and M.CHAMFER < M.THREAD_R
    assert 0 < M.SINK < 0.001


def test_seated_square_succeeds():
    o = seated_at()
    assert M.success(o) and M.score(o) == M.TOP_SCORE == 0.8


def test_seated_with_the_tie_rod_swung_straight_is_the_same_seat():
    # the stud's turn about its own axis does not matter
    o = seated_at(r=np.eye(3))
    assert M.success(o)


@pytest.mark.parametrize("about", [rx, ry])
@pytest.mark.parametrize("deg, credit", [(1.9, 0.8), (2.1, 0.0), (5.9, 0.0), (12.0, 0.0)])
def test_tilt(about, deg, credit):
    """Square within 2 degrees; a stud cocked more than that earns nothing, seated or not."""
    assert M.score(tilted(deg, about)) == pytest.approx(credit)
    assert M.score(tilted(-deg, about)) == pytest.approx(credit)


@pytest.mark.parametrize("about", [rx, ry])
@pytest.mark.parametrize("deg, credit", [(1.9, 0.3), (2.1, 0.0), (4.0, 0.0), (5.9, 0.0)])
def test_a_stud_wedged_high_and_cocked_is_not_paid(about, deg, credit):
    """Raised 3 mm in the taper: square within 2 degrees it has started; cocked more, it is a jam and earns nothing."""
    r = SEAT_R @ about(math.radians(deg))
    top_m = M.TAPER_TOP_M - BALL_M
    ball = SEAT_BALL + SEAT_R @ top_m - r @ top_m + np.array([0, 0, 0.003])
    assert M.score(state(ball, r)) == pytest.approx(credit)


@pytest.mark.parametrize("dz_mm, credit", [(-0.4, 0.8), (0.4, 0.8), (-0.6, 0.0), (0.6, 0.3), (15.0, 0.3),
                                           (39.5, 0.3), (40.0, 0.0), (42.5, 0.0), (60.0, 0.0)])
def test_depth(dz_mm, credit):
    """Home within 0.5 mm either way. Higher, with the tip at least 2 mm below the hole's top (41.9 mm of stud below
    the top when seated), it has started; with less of it in the hole it scores nothing. Deeper than home is not a
    place a real stud can be, and scores nothing."""
    assert M.score(seated_at(dz=dz_mm / 1000)) == pytest.approx(credit)


@pytest.mark.parametrize("off_mm, credit", [(0.9, 0.8), (1.1, 0.3), (2.3, 0.3), (2.5, 0.0), (8.9, 0.0), (30.0, 0.0)])
@pytest.mark.parametrize("ang", [0.0, 2.0, 4.0])
def test_off_axis(off_mm, credit, ang):
    o = seated_at(dx=off_mm / 1000 * math.cos(ang), dy=off_mm / 1000 * math.sin(ang))
    assert M.score(o) == pytest.approx(credit)


def test_the_stud_on_the_hole_edge_scores_nothing():
    # the tip resting on the boss's top face beside the mouth
    o = seated_at(dx=0.012, dz=(M.TIP_M[2] - M.TAPER_TOP_M[2]) * -1 + M.SINK + 0.0005)
    assert M.score(o) == 0


@pytest.mark.parametrize("off_mm, down_mm", [(3.0, 1.0), (5.0, 0.1), (8.9, 0.05), (2.5, 1.9)])
def test_a_stud_on_the_mouth_rim_scores_nothing(off_mm, down_mm):
    """Square, the thread's end on the rim of the mouth: the 1 mm chamfer step inside, or the tip pressed a little in."""
    seated_tip_down = M.TAPER_L + M.THREAD_L + M.SINK  # the tip below the hole's top when seated
    o = seated_at(dx=off_mm / 1000, dz=seated_tip_down - down_mm / 1000)
    assert M.score(o) == 0


def test_a_dropped_end_scores_nothing():
    # let go short of the hole: the outer end falls past the steering arm, the stud hanging or swung
    o = state(M.ball_at(M.SEAT_YAW - 0.06, -0.07))
    assert M.score(o) == 0
    o = state(M.ball_at(0.0, -0.08), rx(math.radians(15)))
    assert M.score(o) == 0


def test_a_stud_deeper_than_its_seat_is_not_in_the_hole():
    # the taper cannot pass its own seat, so a stud lower than that is not one standing in the hole
    assert M.score(seated_at(dz=-0.0015)) == 0


def test_success_needs_the_stud_and_rod_still():
    assert not M.success(seated_at(stud_moving=True))
    assert not M.success(seated_at(rod_moving=True))


@pytest.mark.parametrize("kw", [{"stud_moving": True}, {"rod_moving": True}])
def test_seated_but_moving_scores_less_than_full(kw):
    """The last step's score() is the trial's score when success never held 1 s: a stud seated but still moving
    then earns 0.6, not the top tier."""
    assert M.score(seated_at(**kw)) == pytest.approx(0.6)
    assert M.score(seated_at()) == 0.8


def test_every_shape_has_a_positive_size():
    for link in (MECH.base, *MECH.links):
        for shape in link.shapes:
            assert all(v > 0 for v in shape.size), (link.name, shape.size)
    neck = MECH.links[3].shapes[0]
    assert abs(neck.size[1] - (M.BALL_TO_TAPER + M.HOUSING_LOW)) < 1e-12  # 15 mm, housing bottom to taper top


def test_stillness_is_judged_at_the_stud_not_the_frame_origin():
    """A link's velocity is reported at its frame's origin, 0.39 m from the stud. A slow turn of the stud about its
    taper moves that origin fast but the stud hardly at all: still. A stud sliding at 0.02 m/s is not."""
    o = seated_at()
    s = o["tie_rod"].links["stud"]
    w = np.array([0.0, 0.0, 0.2])
    arm = M._rot(s.quat_xyzw) @ M.TAPER_TOP_M
    assert np.linalg.norm(np.cross(w, arm)) > 0.05  # the origin moves fast
    assert M.success(seated_at(stud_vel=(-np.cross(w, arm), w)))
    assert not M.success(seated_at(stud_vel=(np.array([0.02, 0, 0]) - np.cross(w, arm), w)))
    assert not M.success(seated_at(stud_vel=(-np.cross(np.array([0, 0, 0.6]), arm), np.array([0, 0, 0.6]))))


def test_the_ball_moves_on_its_sphere():
    for yaw in (0.0, M.SEAT_YAW):
        for rise in (0.0, M.LIFT, -M.SEAT_DROP):
            b = M.ball_at(yaw, rise)
            j = np.array([*M.INNER, M.TOP + M.JOINT_Z])
            assert abs(np.linalg.norm(b - j) - M.L) < 1e-12


def test_a_joint_loop_is_refused():
    from mechanism import Joint, Link, mechanism
    links = (Link("a", 0.1, MECH.links[0].shapes), Link("b", 0.1, MECH.links[0].shapes))
    loop = mechanism("loop", (0, 0, 0), MECH.base, links,
                     (Joint("ja", "revolute", "a", (0, 0, 0), "Z", -1, 1, parent="b"),
                      Joint("jb", "revolute", "b", (0, 0, 0), "Z", -1, 1, parent="a")), M.MATERIALS)
    with pytest.raises(ValueError, match="loop"):
        loop._check()
    MECH._check()  # the tie rod's own chain passes


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["right"], station.BASES["right"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert plan.duration < 2400 / 20


# -- home, as the real parts define it -----------------------------------------------------------------------------
def test_home_is_the_matched_parts_hand_seat_read_under_the_arm():
    """Seated, the thread stands below the steering arm's bottom face by the thread's length plus the stepped taper's
    sink past flush: the protrusion a depth gauge on that face reads for the sim's matched parts."""
    o = seated_at()
    assert M.ARM_BOTTOM_Z == pytest.approx(M.HOLE_TOP_Z - M.TAPER_L)
    assert M.protrusion(o) == pytest.approx(M.P_HOME, abs=1e-9)
    assert M.P_HOME == pytest.approx(M.THREAD_L + M.SINK) and 0.0225 < M.P_HOME < 0.0227
    # the knuckle's mesh: its boss bottom is the arm's bottom face (the mesh is 1 mm low, its asset spawns 1 mm up)
    assert MESHES.BOSS_TOP - MESHES.TAPER_L + MESHES.REST == pytest.approx(M.ARM_BOTTOM_Z - M.TOP)


@pytest.mark.parametrize("dz_mm", [-0.3, 0.3, 2.0, -1.0])
def test_protrusion_follows_the_stud_both_ways(dz_mm):
    """A stud raised by d stands d less below the arm; lowered, d more."""
    assert M.protrusion(seated_at(dz=dz_mm / 1000)) == pytest.approx(M.P_HOME - dz_mm / 1000, abs=1e-9)


def test_score_never_pays_full():
    """Only success() held for 1 s pays 1.0; any one step's score() stays below it."""
    states = [seated_at(), seated_at(stud_moving=True), seated_at(dz=0.003), state(M.START_BALL), tilted(1.0)]
    assert max(M.score(o) for o in states) == M.TOP_SCORE < 1.0


# -- the hands -----------------------------------------------------------------------------------------------------
def test_arms_at_ready_are_clear_at_the_start_and_at_the_seat():
    for o in (state(M.START_BALL), seated_at()):
        gaps = M.hand_clearance(o, READY_ARMS)
        assert gaps == {"left": M.CLEAR_CAP, "right": M.CLEAR_CAP}


def test_a_closed_grip_on_the_housing_is_not_clear():
    gaps = M.hand_clearance(state(M.START_BALL), demo_hand("grip"))
    assert gaps["right"] <= 0.0 and gaps["left"] == M.CLEAR_CAP


def test_the_open_hand_beside_the_housing_is_not_counted_as_holding():
    """Open at the grip height, before closing: the pads stand about 25 mm off the housing [ARITH: (94 - 40) / 2 =
    27 mm, less the bound's slack]."""
    gap = M.hand_clearance(state(M.START_BALL), demo_hand("settle", 1))["right"]
    assert 0.020 < gap < 0.027


def seat_grip(finger, lift=0.0):
    """The right hand at the demo's grip on the housing with the stud seated, fingers at ``finger``, raised ``lift``."""
    b = M.ball_at(M.SEAT_YAW, -M.SEAT_DROP)
    pos = (float(b[0]), float(b[1]), float(b[2] + M.TCP_OVER_BALL + lift))
    w = [Waypoint(4.0, pos, finger, "at the seat", M.GRIP_YAW + M.SEAT_YAW)]
    return {"left": READY_ARMS["left"], "right": plan_poses(w, 4.0)}


def test_seated_still_but_gripped_is_not_success():
    """The scorer's old hole: the stud seated square and still while the fingers still hold the housing."""
    o = seated_at()
    M._HANDS["poses"] = seat_grip(M.GRIP_FINGER)
    assert M.hand_clearance(o)["right"] <= 0.0
    assert not M.success(o) and M.score(o) == pytest.approx(0.6)


def test_seated_after_letting_go_and_backing_off_succeeds():
    o = seated_at()
    # open at the release height, the pads stand about 25 mm off the housing [ARITH: 27 mm, less the 2 mm slack]
    assert 0.024 < M.hand_clearance(o, seat_grip(FINGER_OPEN))["right"] < 0.027
    for arms in (seat_grip(FINGER_OPEN), seat_grip(FINGER_OPEN, 0.06), demo_hand("clear")):
        M._HANDS["poses"] = arms
        assert M.hand_clearance(o)["right"] >= M.HAND_CLEAR
        assert M.success(o) and M.score(o) == M.TOP_SCORE


def _hand_at_gap(target, side="right"):
    """The hand, half closed 60 mm over its grip on the housing, lowered until its clearance is ``target``
    (bisection; its pads come down on the housing's top)."""
    o, top = seated_at(), seat_grip(M.GRIP_FINGER, 0.06)
    arms = {"left": top["right"], "right": top["right"]}  # either key may carry the near hand
    far = {"left": READY_ARMS["left"], "right": READY_ARMS["right"]}
    lo, hi = -0.06, 0.0  # dz: lo is too near, hi too far
    for _ in range(40):
        mid = (lo + hi) / 2
        g = M.hand_clearance(o, {**far, side: shifted(arms, side, mid)[side]})[side]
        lo, hi = (lo, mid) if g > target else (mid, hi)
    return o, {**far, side: shifted(arms, side, hi)[side]}


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("gap_mm, clear", [(7.5, False), (9.5, False), (10.5, True), (12.5, True)])
def test_the_10_mm_clearance_either_side_of_the_limit_for_either_arm(side, gap_mm, clear):
    o, arms = _hand_at_gap(gap_mm / 1000, side)
    M._HANDS["poses"] = arms
    assert M.hand_clearance(o)[side] == pytest.approx(gap_mm / 1000, abs=2e-4)
    assert M.hands_clear(o) is clear and M.success(o) is clear
    assert M.score(o) == pytest.approx(M.TOP_SCORE if clear else 0.6)


def test_no_arm_reading_is_not_success():
    """If the hook has not run since the reset (or is not wired), the hands count as not clear."""
    M._HANDS["poses"] = None
    o = seated_at()
    assert M.hand_clearance(o) is None and not M.success(o) and M.score(o) == pytest.approx(0.6)


def test_a_hand_on_the_rod_or_the_stud_counts_too():
    """A hand propping the rod far from the housing, or touching only the stud's thread under the arm."""
    o = seated_at()
    for name, m_point in (("rod", np.array([0.0, -0.15, M.JOINT_Z])), ("stud", M.TIP_M)):
        lk = o["tie_rod"].links[name]
        p = np.asarray(lk.pos) + M._rot(lk.quat_xyzw) @ m_point
        hand = {k: t.copy() for k, t in READY_ARMS["right"].items()}
        shift = p - hand["tip_left"][:3, 3]
        hand = {k: np.vstack([np.c_[t[:3, :3], t[:3, 3] + shift], [0, 0, 0, 1.0]]) for k, t in hand.items()}
        M._HANDS["poses"] = {"left": READY_ARMS["left"], "right": hand}
        assert M.hand_clearance(o)["right"] <= 0.0, name
        assert not M.success(o)


def _point_to_cylinder(p, shape):
    """Exact distance from points to a solid cylinder (mechanism frame)."""
    r, length = shape.size
    ax = "XYZ".index(shape.axis)
    q = p - np.asarray(shape.center)
    a = np.abs(q[:, ax]) - length / 2
    rad = np.linalg.norm(np.delete(q, ax, axis=1), axis=1) - r
    return np.linalg.norm(np.maximum(np.c_[a, rad], 0.0), axis=1) + np.minimum(np.maximum(a, rad), 0.0)


@pytest.mark.parametrize("label, nth", [("down", 0), ("settle", 1), ("over the housing", 0)])
def test_the_clearance_is_never_more_than_the_true_gap(label, nth):
    """Its claim is a lower bound: no hull vertex may lie nearer the parts than the reported clearance (a vertex's
    exact distance is at least the true gap)."""
    o, arms = state(M.START_BALL), demo_hand(label, nth)
    gap = M.hand_clearance(o, arms)["right"]
    shapes = [*M._rod_shapes(), *M._stud_shapes()]  # at rest the links' frames are the mechanism's
    exact = np.inf
    for linkname, t in arms["right"].items():
        for verts, _ in M._arm_hulls()[linkname]:
            with np.errstate(all="ignore"):  # WHY: spurious matmul FP flags from macOS numpy 2.2 (checked finite)
                v = verts @ t[:3, :3].T + t[:3, 3] - M.MECH_ORIGIN
            assert np.isfinite(v).all()
            exact = min(exact, min(float(_point_to_cylinder(v, s).min()) for s in shapes))
    assert gap <= exact
    assert exact - gap < 0.012  # and not uselessly loose here


def test_the_laid_points_cover_every_surface_within_SAMPLE():
    rng = np.random.default_rng(0)
    for shape in [*M._rod_shapes(), *M._stud_shapes()]:
        pts = M._surface(shape)
        r, length = shape.size
        ax = "XYZ".index(shape.axis)
        o = [k for k in range(3) if k != ax]
        t = rng.uniform(0, 2 * math.pi, 400)
        side = np.zeros((400, 3))
        side[:, ax] = rng.uniform(-length / 2, length / 2, 400)
        side[:, o[0]], side[:, o[1]] = r * np.cos(t), r * np.sin(t)
        cap = np.zeros((400, 3))
        rr = r * np.sqrt(rng.uniform(0, 1, 400))
        cap[:, ax] = rng.choice([-length / 2, length / 2], 400)
        cap[:, o[0]], cap[:, o[1]] = rr * np.cos(t), rr * np.sin(t)
        q = np.vstack([side, cap]) + np.asarray(shape.center)
        nearest = np.sqrt(((q[:, None, :] - pts[None, :, :]) ** 2).sum(-1)).min(axis=1)
        assert nearest.max() <= M.SAMPLE, (shape, nearest.max())


def test_every_arm_body_is_watched():
    assert set(M.ARM_LINKS) == {"base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left",
                                "tip_right"}
    assert set(M._PARTS) == {"rod", "stud"}
    assert len(M._PARTS["rod"]) > 5000 and len(M._PARTS["stud"]) > 1000


# -- the hook, on fake simulator objects ---------------------------------------------------------------------------
def _pose7(t, origin):
    return np.r_[t[:3, 3] + origin, quat(t[:3, :3])]


def fake_env(arms, fingers=(-0.015, FINGER_OPEN), origin=(4.0, -2.0, 0.0)):
    """An Isaac Lab-like env: scene[left_robot / right_robot] with body poses (world, offset by the env origin) and
    joints, in a shuffled order, as numpy arrays."""
    origin = np.asarray(origin, float)
    scene = {"env_origins": np.array([origin])}
    for (side, key), f in zip(M.ARMS.items(), fingers):
        names = list(M.ARM_LINKS)[::-1] + ["camera_mount"]
        poses = np.array([_pose7(arms[side][n], origin) if n in arms[side] else np.r_[origin, 0, 0, 0, 1.0]
                          for n in names])
        joints = ["joint8", *[f"joint{i}" for i in range(1, 7)], "joint7"]
        data = types.SimpleNamespace(body_link_pose_w=poses[None], joint_pos=np.array([[f] * 8]))
        scene[key] = types.SimpleNamespace(body_names=names, joint_names=joints, data=data)

    class Scene(dict):
        env_origins = scene["env_origins"]

    return types.SimpleNamespace(unwrapped=types.SimpleNamespace(scene=Scene(scene)))


def test_read_arms_returns_task_frame_poses_and_fingers():
    arms = demo_hand("grip")
    poses, finger = M.read_arms(fake_env(arms))
    for side in ("left", "right"):
        for n in M.ARM_LINKS:
            assert np.allclose(poses[side][n], arms[side][n], atol=1e-9), (side, n)
    assert finger == {"left": [-0.015, -0.015], "right": [FINGER_OPEN, FINGER_OPEN]}


def test_read_arms_refuses_a_missing_body():
    env = fake_env(READY_ARMS)
    env.unwrapped.scene["right_robot"].body_names[0] = "renamed"
    with pytest.raises(ValueError):
        M.read_arms(env)


def test_the_hook_runs_before_each_scored_step_and_reset_forgets():
    """mechanism.py wraps the embodiment's _task_step (the step that reads the states and calls success/score) so the
    registered hooks see the env first; the reset hook clears the reading."""
    import mechanism

    seen = []

    class Embodiment:
        def __init__(self):
            self._env = fake_env(seat_grip(M.GRIP_FINGER))

        def reset(self, scene, *, seed=None):
            return "obs"

        def step(self, action):
            return self._task_step()

        def close(self):
            return None

        def _task_step(self):
            o = seated_at()
            seen.append((M.success(o), M.score(o)))
            return "result"

    module = types.SimpleNamespace(YamArenaEmbodiment=Embodiment)
    mechanism._patch_embodiment(module)
    assert M._watch_arms in mechanism._STEP and M._forget_arms in mechanism._RESET
    M._HANDS["poses"] = READY_ARMS  # a stale reading from before: the hook must replace it
    e = Embodiment()
    assert e.step(None) == "result"
    assert seen == [(False, pytest.approx(0.6))]  # the gripping hand, read by the hook, not the stale parked one
    e._env = fake_env(seat_grip(FINGER_OPEN, 0.06))
    e.step(None)
    assert seen[-1] == (True, pytest.approx(M.TOP_SCORE))
    for fn in mechanism._RESET:
        fn()
    assert M._HANDS["poses"] is None
