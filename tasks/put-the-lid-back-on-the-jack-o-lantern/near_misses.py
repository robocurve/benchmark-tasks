"""Near-successes and unintended strategies, scored offline.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so each case first
replays the step-side function observe() for 1.2 s of control steps with the given end state and the hands 50 mm away
(unless the case says otherwise), and then scores it. _run() afterwards drives observe() directly to check the 1 s
hold, the whole-trial rules (pumpkin shoved, lid fallen in), unknown states, face sampling and lots B and C, and
raises if anything scores wrongly.
"""

import math

import numpy as np

import lots
import parts
import task
from task import LID, PUMPKIN, R_LID, TOP, Z_RIM

lots.use("A")


def _pk(name="A"):
    lot = lots.current() if name is None else next(l for l in task.LOTS if l.name == name)
    return lot.yaw.get("pumpkin", 0.0)


def _rz(rad):
    return parts._rot((0, 0, 1), rad)


def lid(key_deg=0.0, dz=0.0, tilt_deg=0.0, off=0.0, p_yaw=0.0, flip=False):
    """The lid relative to the opening: notch ``key_deg`` from the tooth, top ``dz`` above the rim, tilted
    ``tilt_deg`` about the line across the notch, moved ``off`` sideways (toward the face)."""
    r = _rz(p_yaw + math.radians(key_deg)) @ parts._rot((1, 0, 0), math.radians(tilt_deg))
    if flip:
        r = r @ parts._rot((1, 0, 0), math.pi)
    p = np.array([task.P_XY[0], task.P_XY[1], TOP + Z_RIM - task.SEAT_DROP + dz]) + _rz(p_yaw) @ np.array([-off, 0, 0])
    return LID.state(pos=tuple(p), quat_xyzw=tuple(parts._matrix_to_quat(r)))


def pumpkin(p_yaw=0.0, dx=0.0):
    return PUMPKIN.state(pos=(task.P_XY[0] + dx, task.P_XY[1], TOP + Z_RIM),
                         quat_xyzw=tuple(parts._matrix_to_quat(_rz(p_yaw))))


def replay(objects, gap=0.05, seconds=1.2):
    """Fill the trial record as the simulator would after ``seconds`` at 20 Hz with these states."""
    task.reset_record()
    for k in range(int(round(seconds * 20)) + 1):
        task.observe(objects, gap, k / 20.0)
    return task.score(objects)


def case(label, lid_state, want, gap=0.05, pk=None):
    objs = {"lid": lid_state, "pumpkin": pk if pk is not None else pumpkin()}
    replay(objs, gap=gap)
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


MAKERS = [
    lambda: case("keyed and seated, hands clear, 1 s still", lid(), 1.0),
    lambda: case("as it starts: perched on the tooth 110 deg out, 5 mm proud, tilted 5 deg",
                 lid(110, 0.005, 5.0), 0.0),
    lambda: case("pressed down level but still on the tooth (4 mm proud)", lid(110, 0.004), 0.0),
    lambda: case("hanging on the notch's edge: 15 deg out, 1.2 mm proud, tilted 7 deg", lid(15, 0.0012, 7.0), 0.0),
    lambda: case("key 7.9 deg out, seated", lid(7.9), 1.0),
    lambda: case("key 8.1 deg out", lid(8.1), 0.0),
    lambda: case("notch toward the face (180 deg out), seated height", lid(180), 0.0),
    lambda: case("top 1.9 mm proud", lid(0, 0.0019), 1.0),
    lambda: case("top 2.1 mm proud", lid(0, 0.0021), 0.0),
    lambda: case("top 2.1 mm below the rim", lid(0, -0.0021), 0.0),
    lambda: case("tilted 2.9 deg", lid(0, 0, 2.9), 1.0),
    lambda: case("tilted 3.1 deg", lid(0, 0, 3.1), 0.0),
    lambda: case("3.1 mm off the opening's axis", lid(0, 0, 0, 0.0031), 0.0),
    lambda: case("upside down in the opening", lid(0, 0, 0, 0, flip=True), 0.0),
    lambda: case("set down on the bench beside the pumpkin",
                 LID.state(pos=(task.P_XY[0] - 0.12, task.P_XY[1], TOP + 0.012)), 0.0),
    lambda: case("fallen in: on the pumpkin's floor", lid(0, -0.090), 0.0),
    lambda: case("seated but a fingertip 4.9 mm from the stem (still held)", lid(), 0.0, gap=0.0049),
    lambda: case("seated, nearest hand point 5.1 mm away", lid(), 1.0, gap=0.0051),
]
CASES = _Lazy(MAKERS)


def _run():
    good = {"lid": lid(), "pumpkin": pumpkin()}
    assert replay(good, seconds=0.9) == 0.0, "0.9 s of stillness must not score"
    assert replay(good, seconds=1.0) == 1.0, "1.0 s of stillness must score"
    # the pumpkin shoved more than 10 mm or turned more than 5 degrees at any step ends credit, even if it goes back
    for d, want in ((0.0105, 0.0), (0.0095, 1.0)):
        task.reset_record()
        for k in range(40):
            task.observe({**good, "pumpkin": pumpkin(dx=d)} if k == 3 else good, 0.05, k / 20.0)
        assert task.score(good) == want, f"pumpkin shoved {d * 1000:.1f} mm must score {want}"
    task.reset_record()
    for k in range(40):
        task.observe({**good, "pumpkin": pumpkin(math.radians(5.5))} if k == 3 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a pumpkin turned 5.5 degrees must end credit"
    # the lid fallen in through the opening at any step ends credit, even if it is fished out and seated
    task.reset_record()
    for k in range(40):
        task.observe({**good, "lid": lid(0, -0.060)} if k == 3 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a lid that fell in must end credit"
    # lifted off and put on the bench, then seated: allowed
    task.reset_record()
    for k in range(40):
        task.observe({**good, "lid": LID.state(pos=(0.2, 0.1, TOP + 0.012))} if k == 3 else good, 0.05, k / 20.0)
    assert task.score(good) == 1.0, "setting the lid down and then seating it is allowed"
    # movement during the hold restarts it
    task.reset_record()
    for k in range(25):
        task.observe({**good, "lid": lid(0, 0.0015) if k == 15 else lid()}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a 1.5 mm movement 0.45 s before the end must restart the hold"
    # unknown hands or lid at any step: no credit
    task.reset_record()
    for k in range(30):
        task.observe(good, None if k == 3 else 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown hand state must give no credit"
    task.reset_record()
    bad = {**good, "lid": LID.state(pos=(float("nan"), 0.0, 0.0))}
    for k in range(30):
        task.observe(bad if k == 2 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown lid state must give no credit"
    task.reset_record()
    assert task.score(good) == 0.0, "no observed steps must give no credit"
    # the hand gap: a point 0.5 mm above the lid's top skin, and one inside the stem, are caught; faces are sampled
    lp = np.asarray(good["lid"].pos)
    g = task.hand_gap(good, np.array([lp + np.array([0.0, 0.020, 0.0005])]))
    assert 0.0004 < g < 0.0006, g
    assert task.hand_gap(good, np.array([lp + np.array([0.0, 0.0, 0.020])])) < 0.0, "inside the stem must touch"
    from yam_arena.analysis import Hull
    cube = np.array([[x, y, z] for x in (0, 0.03) for y in (0, 0.03) for z in (0, 0.03)], float)
    planes = np.array([[1, 0, 0, 0.03], [-1, 0, 0, 0], [0, 1, 0, 0.03], [0, -1, 0, 0], [0, 0, 1, 0.03], [0, 0, -1, 0]],
                      float)
    face = task._hull_face_points(Hull(vertices=cube, planes=planes))
    assert np.min(np.linalg.norm(face - np.array([0.015, 0.015, 0.03]), axis=1)) < 0.003, "faces must be sampled"
    # lots B and C: the pumpkin faces another way, so the notch must go to its turned tooth; lot A's lid angle fails
    for name in ("B", "C"):
        lots.use(name)
        py = lots.current().yaw["pumpkin"]
        at = {"lid": lid(p_yaw=py), "pumpkin": pumpkin(py)}
        assert replay(at) == 1.0, f"lot {name}: keyed to its tooth must score"
        habit = {"lid": lid(-math.degrees(py), p_yaw=py), "pumpkin": pumpkin(py)}
        assert replay(habit) == 0.0, f"lot {name}: the notch at the robot's far side (lot A's angle) must fail"
        start = {"lid": LID.state(pos=lots.current().place["lid"], quat_xyzw=tuple(parts._matrix_to_quat(
            _rz(lots.current().yaw["lid"])))), "pumpkin": pumpkin(py)}
        assert task.measures(start)["key"] != 0 and replay(start) == 0.0, f"lot {name}: the start must not score"
    lots.use("A")
    task.reset_record()


_run()
