"""Offline checks of the shapes, the measurements and the scoring rule. No simulator needed:

    python -m pytest -q test_scoring.py
"""
import json
import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demo as D  # noqa: E402
import footkit as K  # noqa: E402
from yam_arena import task_file  # noqa: E402

T = task_file.load(str(HERE / "task.py")).module
IDENT = (0.0, 0.0, 0.0, 1.0)


def pose(side, lift_deg, plate_deg):
    """A leg lifted ``lift_deg`` and its plate lifted ``plate_deg``, as (leg pos, leg quat, plate pos, plate quat)."""
    return (T.knee_world(side), K.turn_y(math.radians(lift_deg)), T.hinge_world(side), K.plate_quat(side, plate_deg))


def gap(side, lift_deg, plate_deg):
    return K.leg_plate_gap(*pose(side, lift_deg, plate_deg), side)


# -- shapes ------------------------------------------------------------------------------------------------------------
def test_catalogue_sizes():
    assert K.SEAT_W == pytest.approx(0.4572) and K.SEAT_D == pytest.approx(0.4064) and K.SEAT_H == pytest.approx(0.4953)
    assert K.CASTER_R == pytest.approx(0.1016) and K.WHEEL_R == pytest.approx(0.3048)
    assert K.PLATE_W == pytest.approx(0.1524) and K.PLATE_D == pytest.approx(0.1524)


def test_leg_sizes():
    pts = np.vstack([p for p, _ in K.leg_pieces()])
    assert pts[:, 2].min() == pytest.approx(-K.SOLE_DOWN, abs=1e-3)
    assert pts[:, 0].max() - pts[:, 0].min() >= K.FOOT_L - 1e-6
    foot = pts[pts[:, 2] < -K.SHANK - 0.02]
    assert foot[:, 0].max() == pytest.approx(K.HEEL_BACK, abs=1e-6) and foot[:, 0].min() == pytest.approx(K.TOE_X, abs=1e-6)
    assert 2 * np.abs(foot[:, 1]).max() == pytest.approx(K.FOOT_W, abs=0.004)
    assert K.SHANK + K.ANKLE_H == pytest.approx(K.SOLE_DOWN)


def test_every_piece_is_a_solid():
    for pieces in (K.leg_pieces(), K.plate_pieces(1.0), K.plate_pieces(-1.0), K.chair_pieces(), K.manikin_pieces()):
        for piece in pieces:
            pts = piece[0]
            assert len(pts) >= 4 and np.linalg.matrix_rank(pts - pts.mean(0), tol=1e-6) == 3


def test_plate_is_mirrored():
    r, l = K.plate_pieces(1.0)[0][0], K.plate_pieces(-1.0)[0][0]
    assert r[:, 1].min() == pytest.approx(-K.PLATE_W) and r[:, 1].max() == pytest.approx(0.0)
    assert l[:, 1].max() == pytest.approx(K.PLATE_W) and l[:, 1].min() == pytest.approx(0.0)


def test_the_chair_stands_on_the_floor_and_the_manikin_on_the_seat():
    chair = np.vstack([p[0] for p in K.chair_pieces()])
    assert chair[:, 2].min() == pytest.approx(0.0, abs=1e-6)                       # casters and wheels on the floor
    casters = chair[(chair[:, 2] < 0.21) & (chair[:, 0] > -0.07) & (chair[:, 0] < 0.20)]
    assert chair[:, 0].max() - casters[:, 0].min() == pytest.approx(34 * K.IN, abs=0.01)   # 34 in deep without front riggings
    body = np.vstack([p[0] for p in K.manikin_pieces()]) + K.knee_in_chair(0.0)
    thighs = body[(body[:, 2] < K.SEAT_H + 0.02) & (body[:, 0] < 0.30)]
    assert thighs[:, 2].min() == pytest.approx(K.SEAT_H, abs=0.012)                # thighs lie on the seat
    assert body[:, 2].max() == pytest.approx(K.SEAT_H + 0.78, abs=0.03)            # sitting height for 58 in


def test_layout():
    assert T.SIDES == {"right": 1.0, "left": -1.0}
    assert len(T.OBJECTS) == 7
    for name, side in T.SIDES.items():
        knee, hinge = T.knee_world(side), T.hinge_world(side)
        assert knee[1] == pytest.approx(T.SEAT_FRONT[1] + side * K.KNEE_Y)
        assert hinge[2] - T.TOP == pytest.approx(K.PLATE_TOP)
        assert knee[2] - T.TOP - K.SOLE_DOWN == pytest.approx(0.155, abs=1e-3)     # the soles hang 15.5 cm up
    assert T.SEAT_FRONT[0] + K.KNEE_X == pytest.approx({"A": 0.660, "B": 0.680}[T.LOT])
    board = T.OBJECTS[0]
    assert board.pos[2] + board.size[2] / 2 == pytest.approx(T.TOP)                # level with the table
    assert board.pos[0] - board.size[0] / 2 == pytest.approx(T.TABLE_FAR)


# -- measurements --------------------------------------------------------------------------------------------------
def test_angles_round_trip():
    for deg in (-15.0, 0.0, 13.5, 50.0, 80.0):
        assert math.degrees(K.knee_lift(K.turn_y(math.radians(deg)))) == pytest.approx(deg)
    for side in (1.0, -1.0):
        for deg in (0.0, 5.0, 47.0, 95.0):
            assert math.degrees(K.plate_lift(K.plate_quat(side, deg), side)) == pytest.approx(deg)


def test_lifting_brings_the_foot_toward_the_robot():
    knee = T.knee_world(1.0)
    toe0 = K.place(knee, IDENT, (K.TOE_X, 0.0, K.SOLE))
    toe = K.place(knee, K.turn_y(math.radians(50.0)), (K.TOE_X, 0.0, K.SOLE))
    assert toe[0] < toe0[0] - 0.2 and toe[2] > toe0[2] + 0.2


def test_the_hanging_foot_is_in_the_plates_way():
    for side in (1.0, -1.0):
        assert gap(side, 0.0, K.PLATE_UP) > 0.015                     # folded up, the plate is clear of the leg
        assert gap(side, 0.0, 0.0) == 0.0                             # folded down, it would be inside the foot
        stops = next(d for d in np.arange(K.PLATE_UP, -0.1, -1.0) if gap(side, 0.0, d) < T.TOUCH)
        assert 35.0 <= stops <= 70.0                                  # a plate let fall comes to rest against the leg


def test_a_leg_lifted_50_degrees_lets_the_plate_past():
    for side in (1.0, -1.0):
        assert min(gap(side, D.LIFT, d) for d in np.arange(0.0, K.PLATE_UP + 0.1, 1.0)) > 0.030
        assert min(gap(side, 40.0, d) for d in np.arange(0.0, K.PLATE_UP + 0.1, 1.0)) == 0.0   # 40 degrees is not enough


def test_the_foot_rests_on_the_plate():
    rest = math.degrees(K.rest_lift())
    assert 12.0 <= rest <= 15.0
    for side in (1.0, -1.0):
        m = K.foot_marks(*pose(side, rest, 0.0), side)
        assert K.over_plate(m["heel"]) and K.over_plate(m["mid"])
        assert T.ON_TOP[0] <= m["low"] <= T.ON_TOP[1]
        assert min(m["heel"][1], m["mid"][1]) > 0.02 and max(m["heel"][1], m["mid"][1]) < K.PLATE_W - 0.02
        assert abs(m["heel"][0]) < K.PLATE_D / 2 - 0.02 and abs(m["mid"][0]) < K.PLATE_D / 2 - 0.02
        held = K.foot_marks(*pose(side, 30.0, 0.0), side)
        assert held["low"] > 0.02                                    # a foot held up is not on the plate


def test_leg_distance():
    knee = T.knee_world(1.0)
    assert K.leg_distance([knee + (0.0, 0.0, -0.2)], knee, IDENT) == 0.0
    assert 0.13 <= K.leg_distance([knee + (-0.20, 0.0, -0.2)], knee, IDENT) <= 0.16      # a lower bound on the true 15.7 cm
    assert K.leg_distance([knee + (0.0, 0.5, 0.0)], knee, IDENT) > 0.35


# -- the demo's geometry ---------------------------------------------------------------------------------------------
def test_supporting_hand_touches_the_sole_and_stays_below_it():
    prof = np.array([[x, 0.0, z] for x, z in D.PROFILE])
    for side in (1.0, -1.0):
        knee = T.knee_world(side)
        for deg in np.arange(0.0, D.LIFT + 0.1, 2.5):
            a = math.radians(deg)
            origin = knee + (-K.SOLE_DOWN * math.sin(a), 0.0, -K.SOLE_DOWN * math.cos(a))
            along, into = np.array([-math.cos(a), 0.0, math.sin(a)]), np.array([math.sin(a), 0.0, math.cos(a)])
            pts = D.hand_under(T, side, deg) + prof - origin
            under = (pts @ along >= -K.HEEL_BACK) & (pts @ along <= K.FOOT_L - K.HEEL_BACK)
            assert float((pts @ into)[under].max()) == pytest.approx(0.0, abs=1e-9)
            assert float((pts[0] @ along)) == pytest.approx(D.TIP_AT, abs=0.02)


def test_supporting_hand_ends_in_front_of_the_plate():
    rest = math.degrees(K.rest_lift())
    for side in (1.0, -1.0):
        tip = D.hand_under(T, side, rest - D.LOWER_TO) + (D.PROFILE[0][0], 0.0, 0.0)
        assert tip[0] < T.hinge_world(side)[0] - K.PLATE_D / 2 - 0.005         # fingertips in front of the plate's front edge
        assert tip[2] - 0.035 > T.hinge_world(side)[2] - 0.045                 # and the hand's underside above the floor


def test_push_goes_past_upright():
    for side in (1.0, -1.0):
        start, end = D.push_point(T, side, D.PUSH_FROM), D.push_point(T, side, D.PUSH_TO)
        face = K.PLATE_UP - 90.0                                               # the upright plate's lean, degrees
        assert side * (start[1] - T.hinge_world(side)[1]) > D.PUSH_UP * math.tan(math.radians(face)) + 0.02
        assert side * (end[1] - T.hinge_world(side)[1]) < -0.01                # inside the hinge line: past upright


# -- the scoring rule ------------------------------------------------------------------------------------------------
def side(down=False, on=False, released=False, still=False, struck=False):
    return dict(down=down, on=on, released=released, still=still, struck=struck)


DONE = side(True, True, True, True)


def test_side_rule():
    f = T.side_score
    assert f(False, False, True, True, False) == 0.0                  # nothing done
    assert f(False, False, True, True, True) == 0.0                   # plate let fall against the leg
    assert f(True, False, False, False, False) == 0.25                # plate down, leg still held up
    assert f(True, True, False, True, False) == 0.25                  # foot on the plate but not let go
    assert f(True, True, True, False, False) == 0.25                  # ... or still moving
    assert f(True, True, True, True, False) == 0.5
    assert f(True, True, True, True, True) == 0.0                     # done, but the plate had touched the leg


def test_every_outcome():
    for dr in (False, True):
        for onr in (False, True):
            for rel in (False, True):
                for st in (False, True):
                    for hit in (False, True):
                        s = T.score_from({"right": side(dr, onr, rel, st, hit), "left": side()}, None)
                        assert s in (0.0, 0.25, 0.5)
    assert T.score_from({"right": side(), "left": side()}, None) == 0.0
    assert T.score_from({"right": DONE, "left": side()}, None) == 0.5
    assert T.score_from({"right": DONE, "left": side(True)}, None) == 0.75
    assert T.score_from({"right": DONE, "left": DONE}, None) == 1.0
    assert T.score_from({"right": DONE, "left": DONE}, "right knee turned 200 deg/s at step 90") == 0.0
    assert T.score_from({"right": side(True), "left": side(True)}, None) == 0.5


def test_thresholds():
    assert math.degrees(T.DOWN) == pytest.approx(5.0) and T.CLEAR == 0.030 and T.TOUCH == 0.002
    assert math.degrees(T.JERK) == pytest.approx(90.0) and math.degrees(T.STILL) == pytest.approx(5.0)
    assert [round(math.degrees(v)) for v in T.RANGE] == [-15, 80]
    assert T.RANGE[0] > math.radians(K.KNEE_FREE[0]) and T.RANGE[1] < math.radians(K.KNEE_FREE[1])   # inside the hinge's stops
    assert T.ON_TOP == (-0.003, 0.005)
    assert D.LIFT < math.degrees(T.RANGE[1])


def test_start_state_scores_nothing():
    T._clear()
    assert T.score(None) == 0.0 and T.success(None) is False
    rep = T.report(None)
    assert rep["right"]["score"] == 0.0 and rep["left"]["score"] == 0.0 and rep["broken"] is None


def test_text_matches_code():
    path = HERE / "task.json"
    if not path.exists():
        pytest.skip("task.json not written yet")
    doc = json.loads(path.read_text())
    assert doc["instruction"] == T.INSTRUCTION
    s = doc["scoring"]
    for phrase in ("5 degrees", "2 mm", "3 cm", "5 mm", "90 degrees a second", "80 degrees", "15 degrees", "0.25", "0.5", "0.75", "1.0"):
        assert phrase in s, phrase
    assert len(doc["fidelity"]) <= 500 and len(doc["title"]) <= 120 and len(doc["context"]) <= 1000 and len(doc["scores"]) <= 300
