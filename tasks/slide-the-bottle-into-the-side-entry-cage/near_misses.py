"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so each case first
replays the step-side function observe() for 1.2 s of control steps with the given end state and the hands 50 mm away
(unless the case says otherwise), and then scores it. _run() afterwards drives observe() directly to check the 1 s
hold, the over-bent arm rule over the whole trial, unknown states, face sampling and lot B, and raises if anything
scores wrongly.
"""

import math

import numpy as np

import lots
import parts
import task
from task import AX, BX, BOTTLE, FLOOR, FRAME, WX

lots.use("A")


def q_axis(axis, spin=0.0):
    """Quaternion turning the bottle's own +y (base to cap) onto ``axis``, spun ``spin`` rad about it."""
    y = np.asarray(axis, float) / np.linalg.norm(axis)
    ref = BX - (BX @ y) * y
    if np.linalg.norm(ref) < 1e-6:
        ref = WX - (WX @ y) * y
    x = ref / np.linalg.norm(ref)
    z = np.cross(x, y)
    m = np.c_[x, y, z] @ parts._rot((0, 1, 0), spin)
    return tuple(parts._matrix_to_quat(m))


def tipped(deg, toward=WX):
    """The cage axis leaned ``deg`` toward ``toward``."""
    k = np.cross(AX, toward)
    return parts._rot(k, math.radians(deg)) @ AX


def bottle(along=0.0, w=0.0, b=0.0, axis=AX):
    """The bottle with its base centre ``along`` up the cage axis from the floor's middle, ``w`` across (toward the
    robot negative) and ``b`` away from the down tube."""
    return BOTTLE.state(pos=tuple(FLOOR + along * AX + w * WX + b * BX), quat_xyzw=q_axis(axis))


def arm(deg=0.0):
    return FRAME.state(joints={"front_arm": math.radians(deg)})


def replay(objects, gap=0.05, seconds=1.2):
    """Fill the trial record as the simulator would after ``seconds`` at 20 Hz with these states."""
    task.reset_record()
    for k in range(int(round(seconds * 20)) + 1):
        task.observe(objects, gap, k / 20.0)
    return task.score(objects)


def task_start():
    return {o.name: o.state() for o in task.OBJECTS}


def case(label, bt, fr, want, gap=0.05):
    objs = {"bottle": bt, "frame": fr}
    replay({**task_start(), **objs}, gap=gap)
    return (label, objs, want, want >= 1.0)


class _Lazy(list):
    """The checker scores each case right after building it, so each case is replayed just before it is checked."""

    def __init__(self, makers):
        super().__init__(range(len(makers)))
        self._makers = makers

    def __iter__(self):
        for m in self._makers:
            yield m()

    def __getitem__(self, i):
        return self._makers[i]()


ON_BENCH = BOTTLE.state()
MAKERS = [
    lambda: case("seated on the floor behind the shut arm, hands clear, 1 s still", bottle(), arm(), 1.0),
    lambda: case("base rim 1.0 mm above the floor (exact limit)", bottle(0.0010), arm(), 1.0),
    lambda: case("base rim 1.2 mm above the floor (not resting on it)", bottle(0.0012), arm(), 0.5),
    lambda: case("hung up 20 mm above the floor behind the shut arm", bottle(0.020), arm(), 0.5),
    lambda: case("hung up 35 mm up the cage", bottle(0.035), arm(), 0.0),
    lambda: case("base 1 mm into the floor (contact)", bottle(-0.0010), arm(), 1.0),
    lambda: case("base centre 3.0 mm off the cage axis (exact limit)", bottle(b=-0.0030), arm(), 1.0),
    lambda: case("base centre 3.5 mm off the cage axis", bottle(b=-0.0035), arm(), 0.5),
    lambda: case("axis 3 degrees off the cage's, rim on the floor", bottle(0.037 * math.sin(math.radians(3.0)),
                 axis=tipped(3.0)), arm(), 1.0),
    lambda: case("axis 4 degrees off the cage's", bottle(0.037 * math.sin(math.radians(4.0)), axis=tipped(4.0)), arm(),
                 0.5),
    lambda: case("seated, front arm 5.0 degrees open (exact limit)", bottle(), arm(5.0), 1.0),
    lambda: case("seated, front arm 6 degrees open", bottle(), arm(6.0), 0.0),
    lambda: case("pushed in part way, resting against the arm's tip (arm 8 degrees open)", bottle(0.0, w=-0.030),
                 arm(8.0), 0.0),
    lambda: case("on the floor 20 mm short, outside the shut arm", bottle(0.0, w=-0.020), arm(), 0.0),
    lambda: case("put in cap first (upside down) on the floor", bottle(0.185, axis=-AX), arm(), 0.0),
    lambda: case("lying on the bench where it started", ON_BENCH, arm(), 0.0),
    lambda: case("seated but a fingertip 4 mm from it (still held)", bottle(), arm(), 0.0, gap=0.004),
    lambda: case("seated, nearest hand point 5.5 mm away", bottle(), arm(), 1.0, gap=0.0055),
]
CASES = _Lazy(MAKERS)


def _run():
    start = task_start()
    good = {**start, "bottle": bottle(), "frame": arm()}
    assert replay(good, seconds=0.9) == 0.0, "0.9 s of stillness must not score"
    assert replay(good, seconds=1.0) == 1.0, "1.0 s of stillness must score"
    # the front arm bent past 30 degrees at any step ends credit for the trial, even if it shuts again
    task.reset_record()
    for k in range(40):
        task.observe({**good, "frame": arm(30.5) if k == 3 else arm()}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an over-bent arm (30.5 degrees) must end credit"
    task.reset_record()
    for k in range(40):
        task.observe({**good, "frame": arm(29.5) if k == 3 else arm()}, 0.05, k / 20.0)
    assert task.score(good) == 1.0, "29.5 degrees is within the arm's range"
    # movement during the hold restarts it
    task.reset_record()
    for k in range(25):
        task.observe({**good, "bottle": bottle(0.0, b=-0.0015 if k == 15 else 0.0)}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a 1.5 mm movement 0.45 s before the end must restart the hold"
    # unknown hands or bottle at any step: no credit
    task.reset_record()
    for k in range(30):
        task.observe(good, None if k == 3 else 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown hand state must give no credit"
    task.reset_record()
    bad = {**good, "bottle": BOTTLE.state(pos=(float("nan"), 0.0, 0.0))}
    for k in range(30):
        task.observe(bad if k == 2 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown bottle state must give no credit"
    # the hand gap: a point 0.5 mm off the bottle's side is caught; one on a flat hull face is sampled
    st = bottle()
    p_side = FLOOR + 0.12 * AX + (task.R_B + 0.0005) * WX
    g = task.hand_gap(good, np.array([p_side]))
    assert 0.0004 < g < 0.0006, g
    from yam_arena.analysis import Hull
    cube = np.array([[x, y, z] for x in (0, 0.03) for y in (0, 0.03) for z in (0, 0.03)], float)
    planes = np.array([[1, 0, 0, 0.03], [-1, 0, 0, 0], [0, 1, 0, 0.03], [0, -1, 0, 0], [0, 0, 1, 0.03], [0, 0, -1, 0]],
                      float)
    face = task._hull_face_points(Hull(vertices=cube, planes=planes))
    assert len(face) > 80, "hull faces must be sampled"
    mid_face = np.array([0.015, 0.015, 0.03])
    assert np.min(np.linalg.norm(face - mid_face, axis=1)) < 0.003, "the middle of a flat face must be sampled"
    # the hand gap counts the front arm too
    arm_c = task.arm_boxes(arm())[2][0]
    assert task.hand_gap(good, np.array([arm_c])) < 0.0, "a point inside the front arm must read as touching it"
    # lot B: same scoring
    lots.use("B")
    assert task.tier(good) == 1.0
    lots.use("A")
    task.reset_record()
    del st


_run()
