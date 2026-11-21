"""Offline checks of the shapes, the needle's rule and the scoring rule. No simulator needed:

    python -m pytest -q test_scoring.py
"""
import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import veinkit as K  # noqa: E402
from yam_arena import task_file  # noqa: E402

T = task_file.load(str(HERE / "task.py")).module
SKIN = np.array([0.0, 0.0, K.SKIN_H])


def line(deg: float, roll: float = 0.0):
    """A needle pointing up the arm ``deg`` degrees below level, its bevel turned ``roll`` degrees from up."""
    t, r = math.radians(deg), math.radians(roll)
    n = np.array([0.0, math.cos(t), -math.sin(t)])
    up0 = np.array([0.0, math.sin(t), math.cos(t)])
    side = np.cross(n, up0)
    return n, math.cos(r) * up0 + math.sin(r) * side


def push(deg=25.0, to=0.0083, roll=0.0, at=(0.0, 0.0), held=True, start=-0.010, step=0.0004, stick=None):
    """Slide the tip along the needle's line from ``start`` to ``to`` (m past the skin), crossing the skin ``at``."""
    s = stick or K.Stick()
    n, up = line(deg, roll)
    e = np.array([at[0], at[1], K.skin_top(at[0])])
    for d in np.arange(start, to + 1e-9, step):
        s.step(e + n * d, n, up, held if not callable(held) else held(d))
    return s


# -- shapes ------------------------------------------------------------------------------------------------------------
def test_arm_shape():
    pts = np.vstack([p for p, _, solid in K.arm_pieces() if solid])
    assert abs(pts[:, 2].max() - 0.100) < 1e-9                      # the upper arm is the tallest part, 10 cm
    assert pts[:, 2].min() >= -1e-9                                 # nothing under the table
    assert abs((pts[:, 1].max() - pts[:, 1].min()) - 0.780) < 1e-9  # 78 cm with the shoulder cap
    assert abs(K.skin_top(0.0) - 0.084) < 1e-12
    assert K.skin_top(0.008) > K.SKIN_H - 0.001                     # nearly level across the site


def test_arm_distance():
    d = K.arm_distance([[0.0, 0.0, K.SKIN_H + 0.003], [0.0, 0.0, K.SKIN_H - 0.002], [0.0, -0.13, K.fore_top(-0.13) + 0.001]])
    assert abs(d[0] - 0.003) < 2e-4 and d[1] == 0.0 and abs(d[2] - 0.001) < 3e-4


def test_holder_and_needle():
    pts = np.vstack([p for p, _, solid in K.holder_pieces() if solid])
    assert abs(pts[:, 1].min() + 0.024) < 1e-9 and abs(pts[:, 1].max() - K.HUB_Y) < 1e-9
    assert abs(K.TIP_Y - K.HUB_Y - 0.0254) < 1e-12                  # a 1 in needle
    nd = K.needle((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    assert np.allclose(nd["tip"], (0.1, 0.2 + K.TIP_Y, 0.3)) and np.allclose(nd["dir"], (0, 1, 0)) and np.allclose(nd["up"], (0, 0, 1))
    nd = K.needle((0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0))            # turned over about its own axis
    assert np.allclose(nd["dir"], (0, 1, 0)) and np.allclose(nd["up"], (0, 0, -1))


def test_cradle_holds_the_barrel():
    # the V's walls are tangent to a 19 mm barrel whose axis is AXIS_H up, and the flange sits just over its shelf
    assert abs(K.V_APEX - (K.AXIS_H - K.BARREL_R * math.sqrt(2))) < 1e-12
    touch = K.AXIS_H - K.BARREL_R / math.sqrt(2)
    assert K.V_APEX < touch < K.V_TOP
    assert 0.0 < (K.AXIS_H + K.FLANGE[2][0]) - K.SHELF[2][1] < 0.001
    assert K.V_HALF < K.BARREL_R                                    # the pads close on the barrel outside the block


def test_layout():
    assert T.LOT == "A" and np.allclose(T.SITE, (0.250, 0.060)) and np.allclose(T.CRADLE_AT, (0.400, 0.170))
    h = T.holder_start()
    tip = K.needle(h, (0, 0, 0, 1))["tip"]
    assert tip[2] - T.TOP > 0.025                                   # the needle is clear of the table in the cradle
    arm = np.vstack([p for p, _, solid in K.arm_pieces() if solid]) + T.arm_pos()
    assert tip[0] - arm[:, 0].max() > 0.06                          # ... and well away from the training arm


# -- the needle's rule -----------------------------------------------------------------------------------------------
def test_attitude():
    for deg in (10, 25, 45):
        a, b = K.Stick.attitude(*line(deg), 0.0)
        assert abs(a - deg) < 1e-6 and b < 1e-4
    a, b = K.Stick.attitude(*line(25, roll=40), 0.0)
    assert abs(a - 25) < 1e-6 and abs(b - 40) < 1e-4
    assert abs(K.Stick.attitude(*line(25, roll=180), 0.0)[1] - 180) < 1e-4


def test_good_stick_reaches_the_vein():
    s = push(25.0, to=K.VEIN_DEPTH / math.sin(math.radians(25)))
    assert s.state == "in" and s.good and s.sticks == 1 and s.reached and s.held and s.kept and s.in_vein
    assert not s.through
    assert abs(s.entry_angle - 25) < 0.01 and np.linalg.norm(s.entry - SKIN) < 0.0005
    assert abs(s.depth - K.VEIN_DEPTH) < 0.0002


def test_not_in_the_skin_until_one_millimetre_under():
    s = push(25.0, to=0.0020)                                       # 0.85 mm under the skin
    assert s.state == "out" and s.sticks == 0
    s = push(25.0, to=0.0026)
    assert s.state == "in" and not s.reached and not s.in_vein      # in the skin, short of the vein


def test_window_of_angles():
    for deg, ok in ((9.0, False), (10.5, True), (20.0, True), (29.5, True), (31.0, False), (45.0, False)):
        s = push(deg, to=0.0015 / math.sin(math.radians(deg)))
        assert s.state == "in" and s.good is ok, deg
    assert "degrees" in push(45.0, to=0.004).why


def test_bevel_has_to_be_up():
    assert push(25.0, to=0.004, roll=25.0).good
    for roll in (35.0, 90.0, 180.0):
        s = push(25.0, to=0.004, roll=roll)
        assert not s.good and "bevel" in s.why, roll


def test_site_window():
    assert push(25.0, to=0.004, at=(0.007, 0.011)).good
    for at in ((0.010, 0.0), (0.0, 0.014), (0.0, -0.014), (-0.010, 0.0)):
        s = push(25.0, to=0.004, at=at)
        assert s.state == "in" and not s.good and s.why == "away from the site", at


def test_unheld_vein_rolls_aside():
    full = K.VEIN_DEPTH / math.sin(math.radians(25))
    s = push(25.0, to=full, held=False)
    assert s.state == "in" and s.good and s.reached and not s.held and not s.in_vein and abs(s.vein_x) == K.VEIN_ROLL
    s = push(25.0, to=0.016, held=False)                           # ... and going deeper does not find it, or go through
    assert not s.in_vein and not s.through
    # held only after the tip has reached the vein: too late
    reach = (K.VEIN_DEPTH - K.VEIN_R) / math.sin(math.radians(25))
    s = push(25.0, to=full, held=lambda d: d > reach + 0.001)
    assert not s.in_vein and not s.held


def test_letting_go_later_is_remembered():
    full = K.VEIN_DEPTH / math.sin(math.radians(25))
    s = push(25.0, to=full, held=True)
    n, up = line(25.0)
    s.step(SKIN + n * full, n, up, False)
    assert s.in_vein and s.held and not s.kept
    s.step(SKIN + n * full, n, up, True)
    assert not s.kept                                               # holding again does not undo it


def test_through_the_far_wall():
    far = (K.VEIN_DEPTH + K.VEIN_R) / math.sin(math.radians(25))
    assert not push(25.0, to=far - 0.0006).through
    s = push(25.0, to=far + 0.0006)
    assert s.through and not s.in_vein


def test_missing_the_vein_to_one_side():
    s = push(25.0, to=0.0083, at=(0.006, 0.0))                     # in at the site, 6 mm from the vein's line
    assert s.good and not s.in_vein and not s.through
    s = push(25.0, to=0.0083, at=(0.0015, 0.0))
    assert s.in_vein


def test_what_the_needle_meets():
    n, _ = line(25.0)
    along = lambda depth: SKIN + n * (depth / math.sin(math.radians(25.0)))          # noqa: E731
    assert K.resistance(along(-0.0002)) == K.F_SKIN == 1.0                  # on the skin
    assert K.resistance(along(0.0008)) == K.F_SKIN                          # in the skin, not through it
    assert K.resistance(along(0.0012)) == K.F_SHAFT == 0.10                 # through the skin, short of the vein
    assert K.resistance(along(0.0015)) == K.F_WALL == 0.40                  # the vein's near wall
    assert K.resistance(along(0.0035)) == K.F_SHAFT                         # in the lumen
    assert K.resistance(along(0.0055)) == K.F_WALL                          # the far wall
    assert K.resistance(along(0.0070)) == K.F_SHAFT                         # beyond it
    assert K.resistance(along(0.0015), vein_x=0.005) == K.F_SHAFT           # the vein has rolled aside: no wall here
    assert K.F_SKIN > K.F_WALL > K.F_SHAFT > 0


def test_where_the_line_meets_the_skin():
    n, _ = line(25.0)
    for d in (-0.004, -0.0003, 0.003):
        p = K.skin_cross(SKIN + n * d, n)
        assert np.allclose(p, SKIN, atol=1e-9), (d, p)
    off = K.skin_cross(SKIN + (0.006, 0.0, 0.0) + n * -0.002, n)
    assert abs(math.hypot(off[0], off[2] - K.ELBOW_R) - K.ELBOW_R) < 1e-9 and abs(off[0] - 0.006) < 1e-9
    assert K.skin_cross((0.2, 0.0, 0.3), (0.0, 1.0, 0.0)) is None           # a line that misses the arm


def test_skin_patch():
    d = K.patch_axis()
    assert abs(np.linalg.norm(d) - 1.0) < 1e-12 and d[0] == 0.0 and d[1] < -0.99 and d[2] < 0       # toward the wrist, a little down
    solid = [pts for pts, _, ok in K.patch_pieces() if ok]
    assert len(solid) == K.PATCH_SEG
    pts = np.vstack(solid)
    assert abs(pts[:, 1].min() - K.PATCH_Y[0]) < 1e-9 and abs(pts[:, 1].max() - K.PATCH_Y[1]) < 1e-9
    assert K.PATCH_Y[1] < -0.06                                            # all of it on the forearm, below the elbow piece
    # it stands 2.3 mm proud of the arm's core on the vein's line (its wedges reach down into the arm, unseen)
    top = max(p[2] for p in pts if abs(p[1] - K.PATCH_Y[1]) < 1e-9)
    assert abs(top - (K.fore_top(K.PATCH_Y[1]) + K.PATCH_GAP + K.PATCH_T)) < 2e-4
    # traction is read along the forearm, toward the wrist
    arm = np.array([0.25, 0.06, 0.10])
    assert abs(K.traction(arm + 0.007 * d, arm) - 0.007) < 1e-12
    assert K.traction(arm - 0.001 * d, arm) < 0 and K.traction(arm + (0.004, 0.0, 0.0), arm) == 0.0
    assert 0 < K.TRACTION < K.TRACTION_MAX and K.SKIN_K * K.TRACTION < 1.0   # 0.75 N holds the skin drawn 5 mm


def test_still_is_read_from_the_holders_back_end():
    n, _ = line(25.0)
    back = np.array([0.0, -0.07, 0.11])
    same = [back] * T.STILL_STEPS
    assert T.still_from(same, [n] * T.STILL_STEPS)
    assert not T.still_from(same[:-1], [n] * (T.STILL_STEPS - 1))                       # not yet a second
    assert not T.still_from([back + (0.0, 0.0012, 0.0)] + same[1:], [n] * T.STILL_STEPS)   # moved 1.2 mm
    assert T.still_from([back + (0.0, 0.0008, 0.0)] + same[1:], [n] * T.STILL_STEPS)       # 0.8 mm is still
    n3, _ = line(28.0)
    assert not T.still_from(same, [n3] + [n] * (T.STILL_STEPS - 1))                     # turned 3 degrees


def test_drawing_out_and_a_second_stick():
    s = push(25.0, to=0.0083)
    n, up = line(25.0)
    for d in np.arange(0.0083, -0.004, -0.0004):
        s.step(SKIN + n * d, n, up, True)
    assert s.state == "out" and not s.in_vein and s.sticks == 1
    push(25.0, to=0.0083, stick=s)
    assert s.state == "in" and s.sticks == 2 and s.in_vein


def test_holding_the_vein_is_drawing_the_skin():
    # the vein counts as held only while the skin below the site is drawn 5 mm or more toward the wrist
    n, up = line(25.0)
    arm = np.zeros(3)
    for drawn_mm, held in ((0.0, False), (4.0, False), (5.0, True), (9.0, True)):
        drawn = K.traction(arm + drawn_mm / 1000 * K.patch_axis(), arm) >= K.TRACTION
        assert drawn is held, drawn_mm
        s = push(25.0, to=0.0083, held=drawn)
        assert s.in_vein is held, drawn_mm


# -- the scoring rule ------------------------------------------------------------------------------------------------
def test_score_tiers():
    f = T.score_from
    assert f(None, False, False, True, True, True, True) == 0.0
    assert f(None, True, False, True, True, True, False) == 0.25
    assert f(None, True, True, True, False, True, True) == 0.5      # in the vein, still moving
    assert f(None, True, True, False, True, True, True) == 0.5      # in the vein, let go
    assert f(None, True, True, True, True, False, True) == 0.5      # in the vein, levered up
    assert f(None, True, True, True, True, True, False) == 0.75
    assert f(None, True, True, True, True, True, True) == 1.0
    assert f("needlestick", True, True, True, True, True, True) == 0.0


def test_segment_gap():
    a, b = np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0254, 0.0])
    assert abs(T._segment_gap([[0.01, 0.01, 0.0]], a, b) - 0.01) < 1e-12
    assert abs(T._segment_gap([[0.0, 0.04, 0.0], [0.5, 0.5, 0.5]], a, b) - 0.0146) < 1e-9


def test_text_matches_code():
    doc = json.loads((HERE / "task.json").read_text())
    assert doc["instruction"] == T.INSTRUCTION
    s = doc["scoring"]
    for phrase in ("8 mm", "12 mm", "10 to 30 degrees", "30 degrees", "4 mm", "3.5 mm", "5 mm", "1 mm", "2 degrees",
                   "5 to 30 degrees", "15 mm", "1.5 mm", "0.25", "0.5", "0.75", "1.0"):
        assert phrase in s, phrase
    assert (K.SITE_X, K.SITE_Y, K.ANGLE, K.BEVEL, K.VEIN_R, K.VEIN_DEPTH, K.TRACTION, K.IN_SKIN) == \
        (0.008, 0.012, (10.0, 30.0), 30.0, 0.002, 0.0035, 0.005, 0.001)
    assert (T.STILL, T.STILL_DEG, T.STILL_STEPS, T.ANGLE_END, T.STICK, T.TOUCH) == (0.001, 2.0, 20, (5.0, 30.0), 0.015, 0.0015)
    f = doc["fidelity"]
    assert (K.F_SKIN, K.F_SHAFT, K.F_WALL) == (1.0, 0.10, 0.40) and all(t in f for t in ("1 N", "0.1 N", "0.4 N"))
    assert K.SKIN_K == 80.0 and "0.8 N per cm" in doc["setup"] and abs(K.TRACTION_MAX - 0.015) < 1e-12 and "15 mm" in doc["setup"]
    assert "800 steps" in doc["run_limit"]
    assert len(doc["fidelity"]) <= 500 and len(doc["title"]) <= 120 and len(doc["context"]) <= 1000
    assert len(doc["scores"]) <= 300
