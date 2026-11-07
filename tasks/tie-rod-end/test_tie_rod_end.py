"""Checks of tie_rod_end.py's scoring, layout and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_tie_rod_end.py
"""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
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


def state(ball, r=np.eye(3), stud_moving=False, rod_moving=False, stud_vel=None):
    """The tie rod with the stud's ball centre at ``ball`` (world) and the stud turned by ``r`` from hanging straight
    down at the start. The rod's own pose only matters for its speed."""
    origin = np.asarray(ball, float) - r @ BALL_M
    m = M.mechanism  # noqa: F841 (the module is loaded)
    st = ObjectState(pos=M.MECH_ORIGIN.copy(), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                     ang_vel=np.zeros(3), low=M.MECH_ORIGIN.copy(), high=M.MECH_ORIGIN.copy())
    from mechanism import MechanismState
    lin, ang = stud_vel if stud_vel else (None, None)
    links = {"stud": link(origin, r, stud_moving, lin, ang), "rod": link(M.MECH_ORIGIN, moving=rod_moving)}
    return {"tie_rod": MechanismState(pos=st.pos, quat_xyzw=st.quat_xyzw, lin_vel=st.lin_vel, ang_vel=st.ang_vel,
                                      low=st.low, high=st.high, joints={}, links=links)}


SEAT_BALL = M.ball_at(M.SEAT_YAW, -M.SEAT_DROP)
SEAT_R = rz(M.SEAT_YAW)


def seated_at(dx=0.0, dy=0.0, dz=0.0, r=SEAT_R, **kw):
    """The stud seated, its ball moved by (dx, dy, dz) and the stud turned by ``r``."""
    return state(SEAT_BALL + np.array([dx, dy, dz]), r, **kw)


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
    assert abs((M.HOLE_TOP_Z - top[2]) - M.SEAT_DEPTH) < 0.0001


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
    assert 0 < M.SEAT_DEPTH < 0.001


def test_seated_square_succeeds():
    o = seated_at()
    assert M.success(o) and M.score(o) == 1.0


def test_seated_with_the_tie_rod_swung_straight_is_the_same_seat():
    # the stud's turn about its own axis does not matter
    o = seated_at(r=np.eye(3))
    assert M.success(o)


@pytest.mark.parametrize("about", [rx, ry])
@pytest.mark.parametrize("deg, credit", [(1.9, 1.0), (2.1, 0.0), (5.9, 0.0), (12.0, 0.0)])
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


@pytest.mark.parametrize("dz_mm, credit", [(-0.4, 1.0), (0.4, 1.0), (-0.6, 0.0), (0.6, 0.3), (15.0, 0.3),
                                           (39.5, 0.3), (40.0, 0.0), (42.5, 0.0), (60.0, 0.0)])
def test_depth(dz_mm, credit):
    """Home within 0.5 mm either way. Higher, with the tip at least 2 mm below the hole's top (41.9 mm of stud below
    the top when seated), it has started; with less of it in the hole it scores nothing. Deeper than home is not a
    place a real stud can be, and scores nothing."""
    assert M.score(seated_at(dz=dz_mm / 1000)) == pytest.approx(credit)


@pytest.mark.parametrize("off_mm, credit", [(0.9, 1.0), (1.1, 0.3), (2.3, 0.3), (2.5, 0.0), (8.9, 0.0), (30.0, 0.0)])
@pytest.mark.parametrize("ang", [0.0, 2.0, 4.0])
def test_off_axis(off_mm, credit, ang):
    o = seated_at(dx=off_mm / 1000 * math.cos(ang), dy=off_mm / 1000 * math.sin(ang))
    assert M.score(o) == pytest.approx(credit)


def test_the_stud_on_the_hole_edge_scores_nothing():
    # the tip resting on the boss's top face beside the mouth
    o = seated_at(dx=0.012, dz=(M.TIP_M[2] - M.TAPER_TOP_M[2]) * -1 + M.SEAT_DEPTH + 0.0005)
    assert M.score(o) == 0


@pytest.mark.parametrize("off_mm, down_mm", [(3.0, 1.0), (5.0, 0.1), (8.9, 0.05), (2.5, 1.9)])
def test_a_stud_on_the_mouth_rim_scores_nothing(off_mm, down_mm):
    """Square, the thread's end on the rim of the mouth: the 1 mm chamfer step inside, or the tip pressed a little in."""
    seated_tip_down = M.TAPER_L + M.THREAD_L + M.SEAT_DEPTH  # the tip below the hole's top when seated
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
    then earns 0.6, not 1.0."""
    assert M.score(seated_at(**kw)) == pytest.approx(0.6)
    assert M.score(seated_at()) == 1.0


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
