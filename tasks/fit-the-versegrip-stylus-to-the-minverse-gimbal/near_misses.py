"""Near misses and shortcuts for fitting the VerseGrip stylus to the MinVerse gimbal.

check.py scores the static CASES below against a task module with no trial record: every one must score 0, because
score() only credits what the simulator's step hook has recorded (fail closed). Run this file directly
(``python near_misses.py``) to replay each scenario through the same record the simulator keeps, at 20 control
steps a second, with stand-in hand points. Each scenario prints its score and the score it must get.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _load():
    spec = importlib.util.spec_from_file_location("versegrip_task_copy", HERE / "task.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


_T = _load()
MV, ST = _T.MINVERSE, _T.STYLUS


def _stylus_at(m, rel_mm, tilt_deg=0.0, turn=0.0, jaw=0.0):
    """The device (gimbal turned ``turn`` rad, tine open ``jaw`` m) and the stylus with its ball at ``rel_mm``
    (du, dw, dn) in the gimbal frame, its body along +u (turned down by ``tilt_deg`` about w)."""
    dev = m.MINVERSE.state(joints={"turn": turn, "jaw": jaw})
    seat, g = m.gimbal_frame({"minverse": dev})
    ball = seat + g @ (np.asarray(rel_mm, float) * 1e-3)
    a = math.radians(tilt_deg)
    body = g[:, 0] * math.cos(a) - g[:, 2] * math.sin(a)  # from the ball toward the tail
    x = -body  # the stylus's local +x points from the tail to the ball
    y = np.cross([0.0, 0.0, 1.0], x)
    y = y / np.linalg.norm(y) if np.linalg.norm(y) > 1e-9 else np.array([0.0, 1.0, 0.0])
    z = np.cross(x, y)
    q = m.parts._matrix_to_quat(np.column_stack([x, y, z]))
    return {"minverse": dev, "versegrip": m.STYLUS.state(pos=tuple(ball), quat_xyzw=tuple(q))}


CASES = [
    ("start: stylus on the desk, nothing recorded", {}, 0.0, False),
    ("stylus seated, no trial record (fail closed)", _stylus_at(_T, (0, 0, 0)), 0.0, False),
    ("ball resting in the cutout's mouth, no trial record", _stylus_at(_T, (0, 0, 6.0)), 0.0, False),
]

FAR = np.array([[0.18, 0.28, 0.91], [0.18, -0.28, 0.91]])  # both hands parked well away


def _on_forearm(m, objects):
    """A hand point on the controller's forearm (holding the arm, not the stylus or gimbal)."""
    a, b, r = m.capsules(objects)[2]
    return np.vstack([FAR, ((a + b) / 2 + np.array([0.0, 0.0, r]))[None, :]])


def _near_stylus(m, objects, gap):
    """A hand point ``gap`` metres off the stylus's body, beside its middle."""
    a, b, r = m.capsules(objects)[0]
    mid = (a + b) / 2
    side = np.cross(b - a, [0.0, 0.0, 1.0])
    side /= np.linalg.norm(side)
    return np.vstack([FAR, (mid + side * (r + gap))[None, :]])


def replay(m, steps):
    """steps: (seconds, rel_mm, tilt_deg, jaw_m, hand) held that long; hand is 'far', 'holding' (on the body),
    'near' (20 mm off the body), 'close' (3 mm off) or None (unknown). Returns the final score."""
    m.REC.clear()
    m.REC.update(m._fresh())
    t, objects = 0.0, None
    for secs, rel, tilt_deg, jaw, hand in steps:
        for _ in range(max(1, round(secs * 20))):
            t += 0.05
            objects = rel if isinstance(rel, dict) else _stylus_at(m, rel, tilt_deg, 0.0, jaw)
            hands = {"far": FAR, "holding": None, "near": None, "close": None, "arm": None, None: None}[hand]
            if hand == "arm":
                hands = _on_forearm(m, objects)
            if hand in ("holding", "near", "close"):
                hands = _near_stylus(m, objects, {"holding": 0.0, "near": 0.020, "close": 0.003}[hand])
            m.observe(objects, hands, t)
    return m.score(objects)


SEAT0 = (0.0, 0.0, -0.2)  # the ball resting on the fork's floor
SCENARIOS = [
    ("stylus untouched on the desk", [(3.0, {"minverse": MV.state(), "versegrip": ST.state()}, 0, 0, "far")], 0.0),
    ("ball laid beside the gimbal, 12 mm off the cutout, let go", [(3.0, (0, 12.0, 0), 20, 0.0, "far")], 0.0),
    ("ball resting in the cutout's mouth (not snapped in), let go", [(3.0, (0, 0, 6.0), 20, 0.0, "far")], 0.25),
    ("ball held in the mouth by the gripper at the end", [(3.0, (0, 0, 6.0), 0, 0.0, "holding")], 0.25),
    ("ball 10.5 mm above the seat: outside the mouth", [(3.0, (0, 0, 10.5), 0, 0.0, "far")], 0.0),
    ("ball 3.6 mm off the cutout's axis: outside the mouth", [(3.0, (3.6, 0, 6.0), 0, 0.0, "far")], 0.0),
    ("ball 2.6 mm off on both sides (3.7 mm diagonally): outside the mouth", [(3.0, (2.6, 2.6, 6.0), 0, 0.0, "far")], 0.0),
    ("ball 2.4 mm off on both sides (3.4 mm diagonally): in the mouth", [(3.0, (2.4, 2.4, 6.0), 0, 0.0, "far")], 0.25),
    ("ball at the seat but the sprung tine held 0.6 mm open (not captured)", [(3.0, SEAT0, 20, 0.0006, "far")], 0.25),
    ("ball 1.6 mm above the seat, tine shut (cannot happen: the lip stops it 0.2 mm up)", [(3.0, (0, 0, 1.6), 20, 0.0, "far")],
     0.25),
    ("snapped in but still held at the end", [(3.0, SEAT0, 0, 0.0, "holding")], 0.5),
    ("snapped in, released, but the stylus still swinging", [(1.0, SEAT0, 0, 0.0, "far"), (0.05, SEAT0, 10, 0.0, "far"),
                                                             (0.5, SEAT0, 20, 0.0, "far")], 0.5),
    ("snapped in, a hand left 20 mm from the stylus", [(1.0, SEAT0, 0, 0.0, "holding"), (2.0, SEAT0, 20, 0.0, "near")],
     0.75),
    ("snapped in, a hand 3 mm from the stylus (not released)", [(1.0, SEAT0, 0, 0.0, "holding"),
                                                                 (2.0, SEAT0, 20, 0.0, "close")], 0.5),
    ("snapped in, hands clear for only 0.9 s at the end", [(1.0, SEAT0, 0, 0.0, "holding"),
                                                           (0.9, SEAT0, 20, 0.0, "far")], 0.5),
    ("snapped in, released, hands clear and still for 1 s", [(1.0, SEAT0, 0, 0.0, "holding"),
                                                             (1.2, SEAT0, 20, 0.0, "far")], 1.0),
    ("snapped in, resting 1.45 mm from the seat against the posts (limit 1.5 mm), released",
     [(1.0, (1.45, 0, 0), 0, 0.0, "holding"), (1.2, (1.45, 0, 0), 20, 0.0, "far")], 1.0),
    ("snapped in, released, but a gripper still holds the controller's forearm", [(1.0, SEAT0, 0, 0.0, "holding"),
                                                                                  (2.0, SEAT0, 20, 0.0, "arm")], 0.5),
    ("snapped in, then pulled back out and laid beside the gimbal", [(2.0, SEAT0, 0, 0.0, "holding"),
                                                                     (2.0, (0, 30.0, 0), 20, 0.0, "far")], 0.0),
    ("full fit, but the hand state was unknown at one step", [(1.0, SEAT0, 0, 0.0, "holding"), (0.1, SEAT0, 20, 0.0, None),
                                                              (1.5, SEAT0, 20, 0.0, "far")], 0.0),
]


def run():
    m = _load()
    bad = 0
    for label, steps, want in SCENARIOS:
        got = replay(m, steps)
        ok = abs(got - want) < 1e-9
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {got:.2f} (want {want:.2f})  {label}")
    # The stylus knocked off the desk: zero for the rest of the trial, even if it is then fitted.
    m.REC.clear()
    m.REC.update(m._fresh())
    t = 0.0
    off = {"minverse": MV.state(), "versegrip": ST.state(pos=(0.3, -0.5, _T.TOP - 0.3))}
    for _ in range(10):
        t += 0.05
        m.observe(off, FAR, t)
    for _ in range(40):
        t += 0.05
        objs = _stylus_at(m, SEAT0, 20)
        m.observe(objs, FAR, t)
    got = m.score(objs)
    ok = got == 0.0
    bad += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {got:.2f} (want 0.00)  stylus dropped off the desk, then fitted")
    # The gimbal pressed down onto the desk (braced on it) at some moment: zero for the rest of the trial.
    m.REC.clear()
    m.REC.update(m._fresh())
    t = 0.0
    low = _stylus_at(m, (0, 0, 6.0), 20)
    low["minverse"] = MV.state(joints={"turn": 0.0, "shoulder": 0.34, "elbow": 0.0})
    gl = m.gimbal_low(low)
    for _ in range(10):
        t += 0.05
        m.observe(low, FAR, t)
    for _ in range(40):
        t += 0.05
        objs = _stylus_at(m, SEAT0, 20)
        m.observe(objs, FAR, t)
    got = m.score(objs)
    ok = got == 0.0 and gl < m.DESK_GAP
    bad += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {got:.2f} (want 0.00)  gimbal pushed down to {gl * 1000:.1f} mm above the desk, then fitted")
    # A stale record: scored against a state the record was not written from.
    replay(m, [(1.0, SEAT0, 0, 0.0, "holding"), (1.2, SEAT0, 20, 0.0, "far")])
    got = m.score(_stylus_at(m, SEAT0, 25))
    ok = got == 0.0
    bad += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {got:.2f} (want 0.00)  stale record (state changed after the last step)")
    print("PASS" if not bad else f"{bad} FAILED")
    return bad


if __name__ == "__main__":
    sys.exit(1 if run() else 0)
