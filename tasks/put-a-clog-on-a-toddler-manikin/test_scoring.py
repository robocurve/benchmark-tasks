"""Checks of the clog task's shapes, fit measurements and scoring rule. No simulator needed:

    python -m pytest -q test_scoring.py
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from yam_arena import task_file  # noqa: E402

T = task_file.load(HERE / "task.py").module
K = T.K
I = (0.0, 0.0, 0.0, 1.0)
KNEE = T.knee_world()


def clog_on(dx: float = 0.0, dz: float = 0.0, dy: float = 0.0) -> np.ndarray:
    """A clog's origin when it is on the right foot, moved ``dx`` back, ``dy`` left and ``dz`` up from home."""
    return KNEE + K.worn_clog_origin() + (dx, dy, dz)


def fit(clog, leg_quat=I, strap_turn=K.STRAP_REST, clog_quat=I) -> dict:
    pivot = K.place(clog, clog_quat, (K.PIVOT[0], 0.0, K.PIVOT[1]))
    r = K._rotation(clog_quat) @ K._rotation(K.turn_y(strap_turn))
    return K.fit(clog, clog_quat, KNEE, leg_quat, pivot, K._quat(r))


# -- shapes ------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("pieces", [K.leg_pieces, K.clog_pieces, K.strap_pieces, K.manikin_pieces])
def test_every_piece_is_a_solid_convex_hull(pieces):
    for pts, color in pieces():
        assert 4 <= len(pts) <= 64 and len(color) == 3
        faces = K.hull_faces(pts)
        assert len(faces) >= 4
        for f in faces:
            a, b, c = pts[f[0]], pts[f[1]], pts[f[2]]
            n = np.cross(b - a, c - a)
            assert ((pts - a) @ n).max() < 1e-9          # every other point is behind the face: it faces outward
        assert np.ptp(pts, axis=0).min() > 0.0005         # not flat


def test_hull_faces_of_a_cube_are_six_squares():
    cube = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)], float)
    faces = K.hull_faces(np.vstack([cube, [[0.5, 0.5, 0.5]]]))
    assert sorted(len(f) for f in faces) == [4] * 6
    assert all(8 not in f for f in faces)                 # the inside point is on no face


def test_foot_is_the_stated_size():
    pts = np.vstack([p for p, _ in K.leg_pieces()])
    foot = pts[pts[:, 2] < K.SOLE + 0.03]
    assert np.ptp(foot[:, 0]) == pytest.approx(K.FOOT_L, abs=0.001)
    assert np.ptp(foot[:, 1]) == pytest.approx(0.054, abs=0.002)
    assert pts[:, 2].min() == pytest.approx(K.SOLE, abs=0.0005)


def test_clog_is_the_stated_size_and_fits_the_gripper():
    pts = np.vstack([p for p, _ in K.clog_pieces()])
    assert np.ptp(pts[:, 0]) == pytest.approx(K.LIN + 2 * K.WALL, abs=0.001)
    assert np.ptp(pts[:, 1]) == pytest.approx(0.072, abs=0.001)
    assert 2 * (K.STRAP_Y + K.ROD) < 0.0939              # with its strap it still passes between open fingers


def test_foot_has_room_inside_the_clog():
    for s, hw, _, top in K.FOOT_SECTIONS:
        x = 0.004 + K.FOOT_L - s                          # where that section lies with the toes 4 mm from the end
        assert K.width(x) - hw >= 0.0025                  # at least 2.5 mm each side
        if x <= K.VAMP:
            assert K.roof(x) - top >= 0.010               # at least 1 cm under the vamp along the midline


def test_strap_rests_on_the_heel_rim_behind_the_heel():
    bar = K.bar_in_clog((0, 0, 0), I, (K.PIVOT[0], 0, K.PIVOT[1]), K.turn_y(K.STRAP_REST))
    assert bar[2] == pytest.approx(K.RIM + K.ROD, abs=1e-6)
    assert K.LIN <= bar[0] <= K.LIN + K.WALL
    heel = 0.004 + K.FOOT_L
    assert bar[0] - K.ROD - heel > 0.010                  # over 1 cm between heel and strap with the toes forward


def test_strap_arms_clear_the_walls():
    for x in np.linspace(K.PIVOT[0], K.LIN, 7):
        assert K.STRAP_Y - K.ROD - (K.width(float(x)) + K.WALL) >= 0.001


def test_knee_flex_is_positive_when_the_foot_swings_away_from_the_robot():
    q = K.turn_y(math.radians(-30.0))
    assert math.degrees(K.knee_flex(q)) == pytest.approx(30.0)
    toe = K.place(KNEE, q, (K.TOE_X, 0.0, K.SOLE))
    assert toe[0] > K.place(KNEE, I, (K.TOE_X, 0.0, K.SOLE))[0]


# -- the fit measurements -------------------------------------------------------------------------------------------------

def test_fit_at_home():
    f = fit(clog_on())
    assert f["toe"] == pytest.approx([0.004, 0.0, 0.0005], abs=1e-6)
    assert f["heel"][0] == pytest.approx(0.134, abs=1e-6)
    assert f["cross"] == pytest.approx([0.004 - K.TOE_X, 0.0], abs=1e-6)
    assert f["ankle_up"] == pytest.approx(K.ANKLE_H + 0.0005, abs=1e-6)
    assert T.toes_under(f) and T.through_ring(f)


def test_start_is_neither():
    c = T.clog_start()
    f = K.fit(c, I, KNEE, I, c + (K.PIVOT[0], 0, K.PIVOT[1]), K.turn_y(K.STRAP_REST))
    assert not T.toes_under(f) and not T.through_ring(f)


def test_strap_forward_is_not_through_the_ring():
    f = fit(clog_on(), strap_turn=T.STRAP_ON_VAMP)
    assert T.toes_under(f) and not T.through_ring(f)
    assert f["bar"][0] < K.VAMP                           # the crossbar lies on the vamp


def test_strap_under_the_heel_is_not_through_the_ring():
    # the clog 3 cm further forward than home with the toes still under the vamp: the heel is over the crossbar
    f = fit(clog_on(dx=-0.030, dz=-0.025))
    assert T.toes_under(f)
    assert f["heel"][0] > f["bar"][0] - K.ROD
    assert not T.through_ring(f)


def test_clog_on_the_floor_under_the_hanging_foot_is_not_on():
    drop = T.TOP + K.SOLE_T - clog_on()[2]                # down to the floor
    f = fit(clog_on(dz=drop))
    assert f["ankle_up"] > T.ANKLE_UP and not T.through_ring(f)


@pytest.mark.parametrize("dx,ok", [(0.0, True), (0.040, True), (0.0415, False)])
def test_toes_limit_along_the_clog(dx, ok):
    assert T.toes_under(fit(clog_on(dx=-dx))) is ok       # the clog moved forward: toes further from the toe end


@pytest.mark.parametrize("dy,ok", [(0.019, True), (0.021, False)])
def test_toes_limit_across_the_clog(dy, ok):
    assert T.toes_under(fit(clog_on(dy=dy))) is ok


@pytest.mark.parametrize("dz,ok", [(0.009, True), (0.011, False), (-0.034, True), (-0.036, False)])
def test_toes_limit_in_height(dz, ok):
    assert T.toes_under(fit(clog_on(dz=dz))) is ok        # the clog raised: toes below the footbed's level


def test_hanging_clog_from_the_demo_counts():
    # as measured at the end of the demo: the clog hangs 9 degrees heel down, the leg 4 degrees back
    leg = K.turn_y(math.radians(-3.8))
    clog_q = K.turn_y(math.radians(-3.8 - 9.4))
    toe = K.place(KNEE, leg, (K.TOE_X, 0.0, K.SOLE))
    clog = toe - K._rotation(clog_q) @ np.array([0.0067, 0.0008, -0.0025])
    f = fit(clog, leg_quat=leg, clog_quat=clog_q)
    assert f["toe"] == pytest.approx([0.0067, 0.0008, -0.0025], abs=1e-6)
    assert math.degrees(f["lean"]) == pytest.approx(9.4, abs=0.01)
    assert T.toes_under(f) and T.through_ring(f)


# -- the scoring rule --------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("toes,through,off,released,still,want", [
    (False, False, False, False, False, 0.0), (False, True, True, True, True, 0.0),
    (True, False, False, True, True, 0.25), (True, False, True, False, True, 0.25), (True, False, True, True, False, 0.25),
    (True, False, True, True, True, 0.5),
    (True, True, False, True, True, 0.75), (True, True, True, False, True, 0.75), (True, True, True, True, False, 0.75),
    (True, True, True, True, True, 1.0)])
def test_score_rule(toes, through, off, released, still, want):
    assert T.score_from(toes, through, off, released, still) == want


def test_score_only_reads_the_trial_state():
    T._clear()
    assert T.score({}) == 0.0 and not T.success({})
    T._STATE.update(toes=True, through=True, off_floor=True, released=True, still=True)
    assert T.score({}) == 1.0 and T.success({})
    T._clear()


def test_layouts():
    assert T.LOT in ("A", "B")
    c = T.clog_start()
    assert c[2] - T.TOP == pytest.approx(K.SOLE_T + 0.0005)
    toe = K.place(KNEE, I, (K.TOE_X, 0.0, K.SOLE))
    assert toe[0] - (c[0] + K.LIN + K.WALL) > 0.02        # the loose clog stands clear in front of the hanging foot
    assert toe[2] - T.TOP == pytest.approx(0.075, abs=0.001)


# -- the demo's plan ---------------------------------------------------------------------------------------------------

def test_demo_keeps_the_foot_clear_on_the_way_in():
    import demo as D
    import sideview as S

    for deg in range(0, 31, 2):
        toes = D.toes_at(float(deg))
        assert S.clear(float(deg), toes, D.LIFT, margin=0.0015), deg
        assert S.strap_gap(float(deg), toes, D.LIFT) > 0.010, deg
    assert S.clear(D.BEND, D.OPEN_AT, D.LIFT, margin=0.002)       # where the toes first come down, in the opening
    assert not S.clear(0.0, 0.004, 0.030, margin=0.0)             # a level foot cannot be lifted straight out


def test_text_matches_code():
    doc = json.loads((HERE / "task.json").read_text())
    assert doc["instruction"] == T.INSTRUCTION
    s = doc["scoring"]
    for phrase in ("4.5 cm", "2 cm", "3.5 cm", "5 mm", "3 cm", "7.5 cm", "2.5 cm", "0.25", "0.5", "0.75", "1.0"):
        assert phrase in s, phrase
    assert (T.TOES_IN, T.TOES_SIDE, T.TOES_UP, T.OFF_FLOOR, T.CLEAR, T.ANKLE_UP, T.SIDE) == (0.045, 0.020, 0.035, 0.005, 0.030, 0.075, 0.025)
    assert "1,500 steps" in doc["run_limit"]
    assert len(doc["fidelity"]) <= 500 and len(doc["title"]) <= 120
