"""Offline checks of the end-face tracker in task.py: feed it synthetic connector trajectories (one pose per physics
substep) and check which ones count as a valid wipe / a dirty insertion. Run with the yam_arena environment:
    python test_scoring.py
"""

import math

import numpy as np

import task as T

DT = 1.0 / 240.0
X, Y, Z = np.eye(3)
POINT_RIGHT = (-Y, X, Z)  # connector axis -y (towards the pad / socket), width along x, height along z


def run(poses):
    """poses: list of (centre, axis, width_dir, height_dir). Returns the tracker state afterwards."""
    T._reset_tracker()
    it = iter(poses)
    T._plug_frame = lambda: next(it)
    for _ in range(len(poses)):
        T._track(DT)
    return dict(T._TR)


def at_tip(tip, frame=POINT_RIGHT):
    a, w, h = frame
    return (np.asarray(tip, float) - a * T.PL / 2, a, w, h)


def line(p0, p1, n):
    return [np.asarray(p0, float) + (np.asarray(p1, float) - np.asarray(p0, float)) * k / (n - 1) for k in range(n)]


Z0 = T.PORT_Z
ON_PAD = T.FACE_Y - 0.0005


def stroke(length, gap=-0.0005, frame=POINT_RIGHT):
    x0 = T.PAD_X0 + 0.03
    return [at_tip(p, frame) for p in line((x0, T.FACE_Y + gap, Z0), (x0 + length, T.FACE_Y + gap, Z0), 240)]


def insert():
    return [at_tip(p) for p in line((T.PORT_X, T.FACE_Y + 0.02, Z0), (T.PORT_X, T.FACE_Y - 0.022, Z0), 120)]


def travel():
    return [at_tip(p) for p in line((T.PAD_X0 + 0.05, T.FACE_Y + 0.02, Z0), (T.PORT_X, T.FACE_Y + 0.02, Z0), 120)]


def test_valid():
    tr = run(stroke(0.06) + travel() + insert())
    assert tr["clean"] and not tr["dirty_insert"], tr


def test_direct_insert():
    tr = run(travel() + insert())
    assert tr["dirty_insert"] and not tr["clean"], tr


def test_short_stroke():
    tr = run(stroke(0.035) + travel() + insert())
    assert tr["dirty_insert"], tr


def test_hover():
    tr = run(stroke(0.06, gap=0.004) + travel() + insert())
    assert tr["dirty_insert"], tr


def test_side_wipe():
    # connector pointing along +x, its side face (half-width 7 mm) pressed flat on the pad, slid 6 cm
    side = (X, Y, Z)
    x0 = T.PAD_X0 + 0.04
    poses = [(np.array([p, T.FACE_Y + T.PW / 2 - 0.0005, Z0]), *side) for p in np.linspace(x0, x0 + 0.06, 240)]
    tr = run(poses + travel() + insert())
    assert tr["best_stroke"] == 0.0 and tr["dirty_insert"], tr


def test_tilted_edge_wipe():
    # axis tilted 35 deg off the pad normal (wiping with the edge): not a wipe
    th = math.radians(35)
    a = np.array([0.0, -math.cos(th), -math.sin(th)])
    h = np.cross(X, a)
    tr = run(stroke(0.06, frame=(a, X, h)) + travel() + insert())
    assert tr["best_stroke"] == 0.0 and tr["dirty_insert"], tr


def test_table_drag_after_wipe():
    drag = [at_tip(p) for p in line((0.38, -0.15, T.TOP + T.PH / 2), (0.35, -0.15, T.TOP + T.PH / 2), 60)]
    tr = run(stroke(0.06) + drag + travel() + insert())
    assert tr["touches"] == 1 and tr["dirty_insert"], tr


def test_port_face_bump_after_wipe():
    bump = [at_tip((T.PORT_X + 0.012, T.FACE_Y + 0.0005, Z0))] * 10
    tr = run(stroke(0.06) + bump + travel() + insert())
    assert tr["dirty_insert"], tr


def test_rewipe_after_drag():
    drag = [at_tip(p) for p in line((0.38, -0.15, T.TOP + T.PH / 2), (0.35, -0.15, T.TOP + T.PH / 2), 60)]
    tr = run(stroke(0.06) + drag + travel() + stroke(0.06) + travel() + insert())
    assert tr["clean"] and not tr["dirty_insert"], tr


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
