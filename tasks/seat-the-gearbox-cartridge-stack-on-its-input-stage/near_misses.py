"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so each case first
replays the step-side function observe() for 1.2 s of control steps with the given end state, the hands 50 mm away
(unless the case says otherwise), and then scores it. _run() afterwards drives observe() directly to check the 1 s
hold, the drop rule, the unknown-state rule, the face-sampled let-go distance and lot B, and raises if anything
scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import STACK, XM, YM, Z_IN

lots.use("A")
HP = math.degrees(task.HALF_PITCH)


def yawq(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def tiltq(deg, yaw=0.0):
    """Tilted ``deg`` about the stack's x axis, after turning ``yaw`` about z."""
    a, b = math.radians(yaw) / 2, math.radians(deg) / 2
    qz = np.array([0.0, 0.0, math.sin(a), math.cos(a)])
    qx = np.array([math.sin(b), 0.0, 0.0, math.cos(b)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def stack(gap=0.0, yaw=0.0, dx=0.0, dy=0.0, q=None, vz=0.0):
    return STACK.state(pos=(XM + dx, YM + dy, Z_IN + gap), quat_xyzw=q or yawq(yaw), lin_vel=(0.0, 0.0, vz))


def task_start():
    return {o.name: o.state() for o in task.OBJECTS}


def replay(objects, gap=0.05, seconds=1.2):
    """Fill the trial record as the simulator would after ``seconds`` at 20 Hz with these states."""
    task.reset_record()
    for k in range(int(round(seconds * 20)) + 1):
        task.observe(objects, gap, k / 20.0)
    return task.score(objects)


def case(label, st, want, gap=0.05):
    objs = {"stack": st}
    replay({**task_start(), **objs}, gap=gap)
    return (label, objs, want, want >= 1.0)


class _Lazy(list):
    """The checker scores each case right after building it, against the record its replay left."""

    def __init__(self, makers):
        super().__init__(range(len(makers)))
        self._makers = makers

    def __iter__(self):
        for m in self._makers:
            yield m()

    def __getitem__(self, i):
        return self._makers[i]()


START = STACK.state()
MAKERS = [
    lambda: case("seated square, released, 1 s still", stack(), 1.0),
    lambda: case("seated turned 180 degrees (ears over the other ears)", stack(yaw=180.0), 1.0),
    lambda: case("seated, 0.3 mm gap (exact limit)", stack(gap=0.0003), 1.0),
    lambda: case("0.4 mm gap all round", stack(gap=0.0004), 0.5),
    lambda: case("seated but turned 0.8 degree (ear holes 0.48 mm off)", stack(yaw=0.8), 1.0),
    lambda: case("seated but turned 0.85 degree (ear holes 0.51 mm off: screws do not start)", stack(yaw=0.85), 0.5),
    lambda: case("seated, axis 0.5 mm off (ear holes 0.5 mm off, exact limit)", stack(dx=0.0005), 1.0),
    lambda: case("seated, axis 0.52 mm off", stack(dx=0.00052), 0.5),
    lambda: case("tabs on the input stage's face: spline in, not turned back (1.1 mm gap, 3 degrees)",
                 stack(gap=0.0011, yaw=3.0), 0.5),
    lambda: case("turned 90 degrees: spline in, tabs on the face (1.1 mm gap)", stack(gap=0.0011, yaw=90.0), 0.5),
    lambda: case("spline in 1.0 mm (2.0 mm gap, exact limit)", stack(gap=0.0020, yaw=4.0), 0.5),
    lambda: case("spline only on its chamfers (2.1 mm gap)", stack(gap=0.0021, yaw=4.0), 0.25),
    lambda: case("pressed straight down, tooth on tooth (3.0 mm gap, square)", stack(gap=0.0030), 0.25),
    lambda: case("hanging on the shaft 18 mm up (jammed)", stack(gap=0.0180), 0.25),
    lambda: case("stack balanced on the shaft tip (19.5 mm up)", stack(gap=0.0195), 0.0),
    lambda: case("on the shaft but cocked 4 degrees, 3 mm up", stack(gap=0.0030, q=tiltq(4.0)), 0.25),
    lambda: case("cocked 6 degrees on the shaft", stack(gap=0.0030, q=tiltq(6.0)), 0.0),
    lambda: case("one side 0.5 mm high (cocked 0.7 degrees) at the seat", stack(gap=0.00025, q=tiltq(0.7)), 0.5),
    lambda: case("beside the motor, axis 2 mm off, on the input stage's face", stack(gap=0.0042, dx=0.002), 0.0),
    lambda: case("left in its cradle", START, 0.0),
    lambda: case("set down gently on the bench beside the motor", STACK.state(pos=(0.30, -0.12, task.TOP + 0.0042)), 0.0),
    lambda: case("seated but a fingertip 4 mm from it (still held)", stack(), 0.0, gap=0.004),
    lambda: case("seated, fingertips 5.5 mm away", stack(), 1.0, gap=0.0055),
]
CASES = _Lazy(MAKERS)


def _run():
    """Rules the case list cannot show: the 1 s hold, movement, the drop rule, unknown states, let-go faces, lot B."""
    start = task_start()
    good = {**start, "stack": stack()}
    assert replay(good, seconds=0.9) == 0.0, "0.9 s of stillness must not score"
    assert replay(good, seconds=1.0) == 1.0, "1.0 s of stillness must score"
    # movement during the hold restarts it
    task.reset_record()
    for k in range(25):
        task.observe({**good, "stack": stack(gap=0.0002 if k == 15 else 0.0)}, 0.05, k / 20.0)
    assert task.score(good) == 1.0, "0.2 mm of movement is within the stillness rule"
    task.reset_record()
    for k in range(25):
        task.observe({**good, "stack": stack(yaw=3.0 if k == 15 else 0.0)}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a 1.3 mm movement 0.45 s before the end must restart the hold"
    # let go of and falling, or let go of and tipped over, at any moment: 0 for the trial, even if then seated
    def run(st5, gap5):
        task.reset_record()
        for k in range(60):
            task.observe({**good, "stack": st5 if k == 5 else stack()}, gap5 if k == 5 else 0.05, k / 20.0)
        return task.score(good)

    falling = STACK.state(pos=(0.25, -0.10, task.TOP + 0.04), lin_vel=(0.0, 0.0, -0.55))
    assert run(falling, 0.05) == 0.0, "a stack let go of and falling at 0.55 m/s must score 0 for the trial"
    assert run(falling, 0.002) == 1.0, "moving it down fast while holding it is not a drop"
    slow = STACK.state(pos=(0.25, -0.10, task.TOP + 0.04), lin_vel=(0.0, 0.0, -0.45))
    assert run(slow, 0.05) == 1.0, "set down from under 13 mm (0.45 m/s) is not a drop"
    tipped = STACK.state(pos=(0.25, -0.10, task.TOP + 0.03), quat_xyzw=tiltq(50.0))
    assert run(tipped, 0.05) == 0.0, "let go of and tipped past 45 degrees must score 0 for the trial"
    assert run(tipped, 0.0) == 1.0, "tilting it 50 degrees in the hand is not a drop"
    # unknown hands or stack at any step ends credit
    task.reset_record()
    for k in range(30):
        task.observe(good, None if k == 3 else 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown hand state must give no credit"
    task.reset_record()
    bad = {**good, "stack": STACK.state(pos=(float("nan"), 0.0, Z_IN))}
    for k in range(30):
        task.observe(bad if k == 2 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown stack state must give no credit"
    # face sampling: a point 0.5 mm off the stack's flat side, far from any hull vertex, is caught
    boxes = task.stack_boxes(stack())
    pts = np.array([[XM + 0.010, YM + task.HALF + 0.0005, Z_IN + 0.030]])
    g = task.gap_to(pts, boxes)
    assert 0.0004 < g < 0.0006, g
    from yam_arena.analysis import Hull

    cube = np.array([[x, y, z] for x in (0, 0.03) for y in (0, 0.03) for z in (0, 0.03)], float)
    planes = np.array([[1, 0, 0, 0.03], [-1, 0, 0, 0], [0, 1, 0, 0.03], [0, -1, 0, 0], [0, 0, 1, 0.03],
                       [0, 0, -1, 0]], float)
    assert len(task._hull_face_points(Hull(vertices=cube, planes=planes))) > 80, "faces must be sampled"
    # lot B: same scoring; the stack starts elsewhere, turned -30 degrees
    lots.use("B")
    assert lots.current().turn < 0 and lots.current().cradle[1] > 0
    assert replay(good) == 1.0
    lots.use("A")
    task.reset_record()


_run()
