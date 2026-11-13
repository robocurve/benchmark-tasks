"""Rules a single layout cannot show: the magnet law against its integral, the hang on a vertical wall, the layouts
the seed picks, the 1 s windows over control steps, and the demo's search stopping over each layout's screw."""
import itertools
import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena import station
from yam_arena.task_file import load

HELPERS = ("stud_runtime", "magnet_law", "yam_parts")
MODULE = next(p for p in sorted(Path(__file__).parent.glob("*.py"))
              if not p.name.startswith("test_") and p.stem not in HELPERS)
T = load(MODULE).module
import magnet_law as L  # noqa: E402  (the task folder is on the path after loading)
import stud_runtime  # noqa: E402
from yam_parts import PartState  # noqa: E402

EPISODES = itertools.count(10**6)
AWAY = {"left": (np.array([0.15, 0.40, T.TOP + 0.15]), 1.0), "right": (np.array([0.32, -0.305, T.TOP + 0.26]), 1.0)}
WEIGHT = T.FINDER_MASS * 9.81


@pytest.fixture(autouse=True)
def isolate():
    saved = dict(T._RUN)
    T._RUN.update(env=None, t=0.0, step=0, episode=next(EPISODES))
    T.TEST_GRIPPERS, T.TEST_LAYOUT = AWAY, None
    yield
    T.TEST_GRIPPERS = T.TEST_LAYOUT = None
    T.ACTIVE_LAYOUT = 0
    T._RUN.clear()
    T._RUN.update(saved)


def state(origin, R=np.eye(3), vel=(0.0, 0.0, 0.0)):
    origin = np.asarray(origin, float)
    low, high = T.FINDER.aabb(origin, R)
    from yam_arena.ik import matrix_to_quat_xyzw
    return PartState((low + high) / 2, matrix_to_quat_xyzw(R), np.asarray(vel, float), np.zeros(3), low, high, origin)


def hanging(k=0, dz=-0.0015, which=1):
    head = T.screw_heads(k)[0]
    return state(T.on_wall_origin(head[1], head[2] - T.TOP + dz, which))


def step(finder, dt=0.05):
    T._RUN["step"] += 1
    T._RUN["t"] += dt
    return T.score({"finder": finder})


# ---------------------------------------------------------------- the magnet law
def brute_field(rho, z, n=60, nt=180):
    """The disc's field by direct 2-D quadrature over both charged faces (independent of the ring formula)."""
    R, t, M = L.MAGNET_D / 2, L.MAGNET_T, L.MAGNET_BR / L.MU0
    r, w = np.polynomial.legendre.leggauss(n)
    r, w = R * (r + 1) / 2, R * w / 2
    th = np.arange(nt) * 2 * math.pi / nt
    rr, tt = np.meshgrid(r, th, indexing="ij")
    W = (w[:, None] * rr * 2 * math.pi / nt).ravel()
    src = np.column_stack([(rr * np.cos(tt)).ravel(), (rr * np.sin(tt)).ravel()])
    out = np.zeros(3)
    for z0, sign in ((0.0, -1.0), (t, 1.0)):
        d = np.column_stack([rho - src[:, 0], -src[:, 1], np.full(len(W), z - z0)])
        out += L.MU0 / (4 * math.pi) * sign * M * (d / np.linalg.norm(d, axis=1)[:, None] ** 3 * W[:, None]).sum(0)
    return out


@pytest.mark.parametrize("rho,z", [(0.0, -0.002), (0.003, -0.002), (0.006, -0.0025), (0.010, -0.004)])
def test_ring_field_matches_direct_integral(rho, z):
    b_r, b_z = L.magnet_field(np.array([rho]), np.array([z]))
    B = brute_field(rho, z)
    assert abs(b_r[0] - B[0]) < 2e-3 and abs(b_z[0] - B[2]) < 2e-3


def test_on_axis_field_closed_form():
    R, t, br = L.MAGNET_D / 2, L.MAGNET_T, L.MAGNET_BR
    for g in (0.001, 0.002, 0.005):
        closed = br / 2 * ((g + t) / math.hypot(g + t, R) - g / math.hypot(g, R))
        assert abs(L.magnet_field(np.array([0.0]), np.array([-g]))[1][0] - closed) < 1e-4


def test_pull_profile():
    fn0, fl0 = L.pull(0.0, 0.002)
    assert 1.3 < fn0 < 1.7 and abs(fl0) < 1e-6                   # about 1.5 N straight over the screw
    assert all(L.pull(r, 0.002)[1] > 0.2 for r in (0.002, 0.004, 0.006, 0.008))   # pulls toward the axis
    assert L.pull(0.010, 0.002)[0] < 0.05                         # nothing 10 mm off the axis
    assert L.pull(0.0, 0.004)[0] < L.pull(0.0, 0.003)[0] < L.pull(0.0, 0.002)[0]   # weaker with gap
    assert L.pull(0.021, 0.002) == (0.0, 0.0) and L.pull(0.0, 0.026) == (0.0, 0.0)
    for r, g in ((0.0035, 0.0021), (0.0071, 0.0034)):             # the table agrees with the integral
        assert np.allclose(L.pull(r, g), L.head_pull(r, g), atol=0.03)


def test_magnet_force_direction():
    face, axis = np.array([T.WALL_X - 0.001, 0.100, T.TOP + 0.130]), np.array([1.0, 0.0, 0.0])
    head = np.array([T.WALL_X + T.SCREW_SET, 0.104, T.TOP + 0.130])
    f = L.magnet_force(face, axis, head)
    assert f[0] > 0.8 and f[1] > 0.3 and abs(f[2]) < 1e-9        # onto the wall and toward the screw
    assert not L.magnet_force(head + [0.001, 0, 0], axis, head).any()   # head behind the face: nothing


def test_hangs_on_a_vertical_wall_with_margin():
    for mu, need in ((0.40, 1.5), (0.30, 1.4)):
        hold = max(mu * L.pull(r, 0.002)[0] + L.pull(r, 0.002)[1] for r in np.arange(0, 0.012, 0.0005))
        assert hold >= need * WEIGHT


def test_wrench_on_the_finder():
    stud_runtime._CFG = vars(T)
    head = T.screw_heads(0)[0]
    origin = T.on_wall_origin(head[1], head[2] - T.TOP)
    f, tq, detail = stud_runtime.finder_wrench(origin, np.eye(3))
    assert 1.3 < f[0] < 1.7 and abs(f[1]) < 1e-6
    lower = detail[1]
    assert abs(lower[0] - (T.SKIN + T.SCREW_SET)) < 1e-9 and lower[1] < 1e-6
    # pulled below the centre of mass and behind it: the torque turns the finder's lower end onto the wall
    assert tq[1] < 0
    f2, _, _ = stud_runtime.finder_wrench(origin + [0, 0.004, 0], np.eye(3))
    assert f2[1] < -0.3                                           # sideways back toward the screw
    far, _, _ = stud_runtime.finder_wrench(T.on_wall_origin(head[1] + 0.040, 0.134), np.eye(3))
    assert np.linalg.norm(far) < 1e-3


# ---------------------------------------------------------------- layouts
def test_layouts_by_seed():
    assert T.layout_for_seed(None) == 0
    assert [T.layout_for_seed(s) for s in range(8)] == [0, 1, 2, 3, 0, 1, 2, 3]
    for k in range(4):
        left, right = T.screw_heads(k)
        assert abs(left[1] - right[1] - T.STUD_SPACING) < 1e-9 and left[2] == right[2]
        for h in (left, right):
            assert T.WALL_Y0 + T.STUD_W / 2 <= h[1] <= T.WALL_Y1 - T.STUD_W / 2
            assert T.TOP + T.STUD_W + 0.050 < h[2] < T.TOP + T.WALL_H - T.CAP_T - 0.020   # on the stud, mid-board
    ys = sorted(T.LAYOUTS[k][0] for k in range(4))
    assert min(b - a for a, b in zip(ys, ys[1:])) >= 0.030          # no two sections share a stud's place
    T.TEST_LAYOUT, T.FORCE_LAYOUT = None, 2
    assert T.layout_index() == 2
    T.TEST_LAYOUT = 3
    assert T.layout_index() == 3
    T.FORCE_LAYOUT = None


def test_a_remembered_spot_scores_only_in_its_own_layout():
    T._RUN["t"] = None                                                   # single end-state reads, no history
    for k in range(4):
        finder = hanging(k)
        for j in range(4):
            T.TEST_LAYOUT = j
            expect = 1.0 if (T.LAYOUTS[j] == T.LAYOUTS[k]) else 0.0
            assert T.score({"finder": finder}) == expect


# ---------------------------------------------------------------- windows over control steps
def test_full_score_needs_a_second_hanging_still():
    finder = hanging()
    scores = [step(finder) for _ in range(30)]
    assert scores[0] == 0.5 and max(scores[:19]) == 0.5 and scores[-1] == 1.0


def test_sliding_down_never_reaches_full_score():
    head = T.screw_heads(0)[0]
    scores = []
    for i in range(40):
        z = head[2] - T.TOP - 0.0002 * i                                   # creeping 4 mm/s down the wall
        scores.append(step(state(T.on_wall_origin(head[1], z))))
    assert max(scores) == 0.5


def test_a_hand_nearby_holds_it_at_half():
    finder = hanging()
    T.TEST_GRIPPERS = {"left": (finder.origin + T.TCP_IN_F + [-0.025, 0, 0], 1.0), "right": AWAY["right"]}
    assert max(step(finder) for _ in range(40)) == 0.5
    T.TEST_GRIPPERS = AWAY
    scores = [step(finder) for _ in range(30)]
    assert scores[0] == 0.5 and scores[-1] == 1.0                       # the second starts when the hand is clear


def test_dropped_on_the_table_scores_zero_even_after_hanging():
    for _ in range(30):
        step(hanging())
    lying = state([0.20, 0.20, T.TOP + 0.020], T.rot("y", 90))
    assert step(lying) == 0.0


# ---------------------------------------------------------------- the demo's search
@pytest.mark.parametrize("k", range(4))
def test_demo_stops_over_each_layouts_screw(k):
    T.TEST_LAYOUT = k
    arm = ArmPlan(T.demo()["left"], np.array(station.BASES["left"]))
    t, labels = 0.0, []
    while t <= arm.duration + 1e-9:
        pos, _, _, label = arm.target(t)
        labels.append(label)
        if label in ("sweep the wall", "slide down to the second pass", "dwell on the screw"):
            assert pos[0] + T.HAND_FRONT <= T.WALL_X + T.PRESS_WALL - 0.002  # the pitched hand stays behind the back
            top = pos[2] - T.TCP_IN_F[2] + T.FINDER_TOP                     # the finder wholly on the wall's face
            assert top <= T.TOP + T.WALL_H - 0.030
        t += 0.05
    head = T.screw_heads(k)[0]
    mag = np.asarray(T._DEMO["hold"]) + T.V0 + T.LOWER_FACE_F
    assert math.hypot(mag[1] - head[1], mag[2] - head[2]) <= 0.0045
    skipped = [w for w in arm.waypoints if getattr(w, "_skip", None) and w._skip()]
    assert all(w.duration == T.SKIPPED_S for w in skipped)
    assert "let go" in labels and labels.index("let go") > labels.index("dwell on the screw")


def test_rows_cover_every_layout_height():
    for _, z in T.LAYOUTS:
        assert min(abs(z - r) for r in T.ROWS) <= T.CAPTURE_DZ
    assert abs(T.ROWS[0] - T.ROWS[1]) > 0.010 and L.pull(0.0125, 0.002)[0] < 0.02   # the other pass misses
