"""Offline tests for price_label_gun_mark_boxes (pure Python: no simulator).

Run: python -m pytest -q test_price_label_gun_mark_boxes.py -p no:cacheprovider
They drive the label rule (_label_rule) and score() with hand-built labeller states. The simulated checks (scripted
demo, shortcuts, start jitter, friction) run in the MuJoCo gate; see README.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from yam_arena.task_file import load  # noqa: E402

T = load(HERE / "price_label_gun_mark_boxes.py")
M = T.module
import labkit  # noqa: E402

PARK = [(0.06, 0.40, M.TOP + 0.16), (0.06, -0.40, M.TOP + 0.16)]
PAD = (M.PAD_X, M.PAD_Y, M.TOP + M.PAD_Z)


def _q(yaw_deg=45.0, roll_deg=0.0):
    """XYZW quaternion: yaw about z, then roll about x (tilts the labeller off flat)."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    return np.array([cy * sr, sy * sr, sy * cr, cy * cr])


def _gun(pos=PAD, travel=0.0, yaw=45.0, roll=0.0):
    p = np.asarray(pos, float)
    return labkit.PartState(pos=p, quat_xyzw=_q(yaw, roll), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                            low=p - 0.1, high=p + 0.1, joints={"trigger": -travel * M.TRAVEL})


def _at_box(i, du=0.0, dz=0.0, gap=0.0, **kw):
    """Labeller pose with its roller front on box i's spot (+du along the face, +dz up), gap mm short of the face."""
    c, s = math.cos(M.YAW), math.sin(M.YAW)
    rx, ry = c * M.ROLLER_C[0] - s * M.ROLLER_C[1], s * M.ROLLER_C[0] + c * M.ROLLER_C[1]
    x = M.FACE_X - M.ROLLER_R - rx - gap
    y = M.BOX_Y[i] + du - ry
    z = M.TOP + M.SPOT_Z + dz - M.TH / 2
    return (x, y, z)


def _fresh():
    M._reset(None)
    M._set_test_trial(tcps=PARK)


def _cycle(pos, **kw):
    """One full trigger cycle at pos: released -> full -> released."""
    out = []
    for t in (0.0, 0.5, 0.95, 0.5, 0.1):
        out.append(M._label_rule(_gun(pos, travel=t, **kw)))
    return [r for r in out if r is not None]


def test_start_scores_nothing():
    _fresh()
    assert M.score({"gun": _gun()}) == 0.0
    assert M._trial["labels"] == [0, 0, 0] and M.distance({"gun": _gun()}) == 3.0
    assert not M.success({"gun": _gun()})


def test_full_cycle_on_spot_lays_one_label():
    _fresh()
    assert _cycle(_at_box(0)) == [0]
    assert M._trial["labels"] == [1, 0, 0]
    assert M.label_ok(0)


def test_all_three_then_pad_scores_one():
    _fresh()
    for i in range(3):
        _cycle(_at_box(i))
    assert abs(M.score({"gun": _gun()}) - 1.0) < 1e-9


def test_tiers():
    _fresh()
    _cycle(_at_box(0))
    assert abs(M.score({"gun": _gun(_at_box(0))}) - 0.25) < 1e-9  # one label, labeller not back
    assert abs(M.score({"gun": _gun()}) - 0.5) < 1e-9  # one label + labeller back on its pad


def test_squeeze_in_air_lays_nothing():
    _fresh()
    assert _cycle(_at_box(0, gap=0.030)) == []
    assert M.score({"gun": _gun()}) == 0.0


def test_press_without_full_squeeze_lays_nothing():
    _fresh()
    for t in (0.0, 0.6, 0.85, 0.3, 0.0):
        M._label_rule(_gun(_at_box(1), travel=t))
    assert M._trial["labels"] == [0, 0, 0]


def test_repeat_cycle_moves_the_one_label_score_follows_final_place():
    _fresh()
    _cycle(_at_box(2))
    assert M.label_ok(2)
    _cycle(_at_box(2, du=0.015))  # a second cycle moves the box's one label off the spot
    assert not M.label_ok(2)
    _cycle(_at_box(2, du=0.002))  # and a third moves it back on: only the final place counts
    assert M.label_ok(2)
    _cycle(_at_box(2))
    assert M.label_ok(2)


def test_label_off_spot_does_not_count():
    _fresh()
    _cycle(_at_box(1, du=0.015))
    assert not M.label_ok(1)
    _fresh()
    _cycle(_at_box(1, du=0.008))
    assert M.label_ok(1)


def test_tilted_or_skewed_labeller_lays_nothing():
    _fresh()
    assert _cycle(_at_box(0), roll=20.0) == []
    _fresh()
    assert _cycle(_at_box(0), yaw=65.0) == []


def test_pad_credit_needs_hands_clear_and_a_label():
    _fresh()
    _cycle(_at_box(0))
    M._set_test_trial(tcps=[np.array(PAD), PARK[1]])
    assert abs(M.score({"gun": _gun()}) - 0.25) < 1e-9
    _fresh()
    assert M.score({"gun": _gun()}) == 0.0  # back on the pad with no label scores nothing


def test_demo_uses_constants_only():
    d = M.demo()
    assert set(d) == {"left", "right"}
    tl, tr = (sum(w.duration for w in d[k]) for k in ("left", "right"))
    assert abs(tl - tr) < 1e-9 and tl * 20 < 2400


def test_occlusion_roller_on_a_hidden_face_lays_nothing():
    """Line-of-sight negative control: a squeeze with the roller past the end of the box row (no front face, nothing
    the camera sees as a spot) lays no label and scores 0."""
    _fresh()
    x, y, z = _at_box(0)
    assert _cycle((x, y + 0.20, z)) == []
    assert M.score({"gun": _gun()}) == 0.0


def test_hover_without_contact_lays_nothing():
    _fresh()
    assert _cycle(_at_box(0, gap=0.002)) == []
    assert _cycle(_at_box(1, gap=0.0015)) == []
    assert M._trial["labels"] == [0, 0, 0]
    assert M.score({"gun": _gun()}) == 0.0
    assert _cycle(_at_box(2, gap=0.0005)) == [2]
