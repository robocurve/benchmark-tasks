"""Offline checks of the gnocchi scorer, no simulator: scripted state sequences for every score level and the
adversarial strategies (scoop everything at once early, wait and scoop at the end, pinch the pieces, leave them in the
spoon on the plate, leave a hand on the spoon, leave the burner on, drop one on the hob, take a raw one out and put it
back), plus the numbers in task.json and the README.

    python -m pytest -q test_task.py
"""
import json
import math
import re
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(str(HERE / "task.py")).module
import floatkit as fk  # noqa: E402
import guardkit as gk  # noqa: E402
import yamkit as kit  # noqa: E402

N = M.RUN_LIMIT_STEPS
FLOAT_Z = M.SURFACE_Z0 - M.G_SIZE[2] / 2 + 0.0008  # a floating piece's center: about 0.8 mm of it shows
FLOOR_Z = M.PAN_FLOOR_Z + M.G_SIZE[2] / 2
AWAY_R = (0.15, -0.25, M.TOP + 0.30)
AWAY_L = (0.15, 0.30, M.TOP + 0.30)


def st(pos, R=np.eye(3), vel=(0, 0, 0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=Rotation.from_matrix(R).as_quat(), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=p - 0.0125, high=p + 0.0125)


def plate_spot(i):
    return np.array([*M.PLATE_SPOTS[i], M.PLATE_TOP + M.G_SIZE[2] / 2])


def knob_off_at_100(t):
    return M.KNOB_START if t < 100.0 else 0.0


def run(plan, hand=None, spoon=None, cook=M.COOK_AT, knob=knob_off_at_100):
    """plan: {piece index: (t_lift or None, where, pinch)}; where is 'plate', 'pan', 'hob', 'counter', 'spoon'. A piece
    sinks raw, floats from cook + 2 s at its ring spot, leaves the water at t_lift, is carried 3 s and lands at
    `where`. hand(t) -> (pos, R, opening) for the right gripper (default: away, open); knob(t) -> the burner knob's
    angle (default: turned to OFF at 100 s). Returns the last score."""
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    M.setup_water(0)
    for i, n in enumerate(M.G_NAMES):
        fk.set_cook(n, fk.Cook(cook[i], M.G_RHO_COOKED, M.COOK_RAMP))
    fk.reset()
    s = 0.0
    spoon_o = np.asarray(M.SPOON_START, float)
    spoon_R = M.spoon_rot(M.REST_PSI, M.REST_TIP)
    for k in range(N):
        t = k * M.DT
        kit._JOINTS["burner/knob"] = kit.JointState(pos=float(knob(t)), vel=0.0, latched=False)
        o = {}
        for i, n in enumerate(M.G_NAMES):
            fk._W.cooked[n] = t >= cook[i]
            xy = np.asarray(M.G_START[i][:2])
            p = np.r_[xy, FLOAT_Z if t >= cook[i] + 2.0 else FLOOR_Z]
            vel = (0, 0, 0)
            t_lift, where, _ = plan.get(i, (None, "pan", False))
            if t_lift is not None and t >= t_lift:
                if t < t_lift + 3.0:
                    p, vel = np.r_[xy, M.PAN_RIM_Z + 0.05], (0.1, 0, 0)
                elif where == "plate":
                    p = plate_spot(i)
                elif where == "hob":
                    p = np.array([M.HOB_C[0] + 0.12, M.HOB_C[1] - 0.12, M.HOB_TOP + M.G_SIZE[2] / 2])
                elif where == "counter":
                    p = np.array([M.SPOON_XY[0] - 0.08, M.SPOON_XY[1], M.TOP + M.G_SIZE[2] / 2])
                elif where == "spoon":
                    p = np.array([M.PLATE_C[0], M.PLATE_C[1], M.PLATE_TOP + M.BOWL_T + M.G_SIZE[2] / 2])
                elif where == "pan":
                    p = np.r_[xy, FLOAT_Z]
            o[n] = st(p, vel=vel)
        so, sR = spoon_o, spoon_R
        if spoon is not None:
            so, sR = spoon(t)
        o["spoon"] = st(so, sR)
        hp, hR, op = AWAY_R, M.GRIP_DOWN, 1.0
        for i, (t_lift, where, pinch) in plan.items():
            if pinch and t_lift is not None and t_lift <= t < t_lift + 3.0:  # fingers closed on the piece
                hp, hR, op = o[M.G_NAMES[i]].pos, M.GRIP_DOWN, M.G_SIZE[1] / (2 * abs(M.FINGER_OPEN))
        if hand is not None:
            hp, hR, op = hand(t, o)
        kit._OPENING["right_gripper"] = op
        kit._OPENING["left_gripper"] = 1.0
        o["right_gripper"] = st(hp, hR)
        o["left_gripper"] = st(AWAY_L, M.GRIP_DOWN)
        kit._EPISODE["last"] = k
        s = M.score(o)
    return s


IDEAL = {i: (c + 5.0, "plate", False) for i, c in enumerate(M.COOK_AT)}


def test_nothing_done_scores_0():
    assert run({}) == 0.0


def test_each_lifted_soon_after_it_floats_scores_1():
    assert run(IDEAL) == 1.0


def test_scoop_all_at_once_early_is_raw_and_scores_0():
    assert run({i: (3.0, "plate", False) for i in range(6)}) == 0.0


def test_one_raw_piece_served_spoils_everything():
    plan = dict(IDEAL)
    plan[5] = (40.0, "plate", False)  # G6 cooks at 76 s
    assert run(plan) == 0.0


def test_raw_piece_put_somewhere_else_out_of_the_pan_is_still_served_raw():
    plan = dict(IDEAL)
    plan[5] = (40.0, "counter", False)
    assert run(plan) == 0.0


def test_wait_and_scoop_all_at_the_end_scores_025():
    # at 112 s only G6 (floated at 78 s) is within 45 s; the rest went soft
    assert run({i: (112.0, "plate", False) for i in range(6)}) == 0.25


def test_pieces_float_up_in_overlapping_pairs_within_1_5_to_2_minutes():
    up = [c + M.RISE_S_EST for c in M.COOK_AT]
    for i in range(6):
        assert M.FLOAT_MIN <= up[i] - M.DROP_AT[i // 2] <= M.FLOAT_MAX
    assert all(0 < up[i + 1] - up[i] < 14.0 for i in (0, 2, 4))  # the second of a pair comes up while the first is
    # still on its way to the plate (a scoop and plating take about 14 s)
    for seed in (1, 2, 3, 4, 5):  # other seeds: within the same 1 1/2 to 2 minutes of one of the three drops
        t = M.trial_cook_times(seed)
        assert sorted(t) != sorted(M.COOK_AT)
        for v in t:
            assert any(M.FLOAT_MIN - 1e-9 <= v + M.RISE_S_EST - d <= M.FLOAT_MAX + 1e-9 for d in M.DROP_AT)


def test_burner_left_on_caps_at_05():
    assert run(IDEAL, knob=lambda t: M.KNOB_START) == 0.5


def test_burner_turned_toward_off_but_short_of_its_click_is_still_on():
    assert run(IDEAL, knob=lambda t: -M.KNOB_CLICK - 0.02) == 0.5
    assert run(IDEAL, knob=lambda t: -M.KNOB_CLICK + 0.01) == 1.0


def test_burner_left_on_and_hand_on_spoon_is_still_05():
    def hand(t, o):
        sp = o["spoon"]
        R = fk.quat_R(sp.quat_xyzw)
        return np.asarray(sp.pos) + R @ M.GRIP_LOCAL, R @ M._HAND_IN_SPOON, M.GRIP_W / (2 * abs(M.FINGER_OPEN))
    assert run(IDEAL, hand=hand, knob=lambda t: M.KNOB_START) == 0.5


def test_knob_law_clicks_into_off_and_holds_elsewhere():
    law = M.RangeKnob(M.KNOB_FRICTION, M.KNOB_HOLD, M.KNOB_CLICK, M.KNOB_CLICK_TORQUE)

    class Lk:
        start = M.KNOB_START
    mem = law.fresh(Lk)
    target, k, effort, _ = law.drive("burner/knob", M.KNOB_START, 0.0, {}, mem)
    assert target == M.KNOB_START and effort == 0.0  # held where it was left
    target, k, _, _ = law.drive("burner/knob", M.KNOB_START + 0.3, 0.0, {}, mem)  # turned 17 degrees on by hand
    assert abs(k * (M.KNOB_START + 0.3 - target) - M.KNOB_FRICTION) < 1e-9  # resists with the friction torque only
    target, k, _, _ = law.drive("burner/knob", -0.10, 0.0, {}, mem)  # 5.7 degrees from OFF: in the click
    assert target == 0.0 and abs(k * M.KNOB_CLICK - M.KNOB_CLICK_TORQUE) < 1e-9 and mem["off"]


def test_pinched_pieces_are_not_good():
    assert run({i: (c + 5.0, "plate", True) for i, c in enumerate(M.COOK_AT)}) == 0.0


def test_ladder():
    for keep, want in ((5, 0.75), (4, 0.5), (3, 0.5), (2, 0.25), (1, 0.25)):
        plan = {i: IDEAL[i] for i in range(keep)}  # the others stay in the water (soft)
        assert run(plan) == want, keep


def test_one_dropped_on_the_hob_is_not_good():
    plan = dict(IDEAL)
    plan[2] = (M.COOK_AT[2] + 5.0, "hob", False)
    assert run(plan) == 0.75


def test_left_in_the_spoon_on_the_plate_is_not_served():
    def spoon(t):
        return np.array([M.PLATE_C[0], M.PLATE_C[1], M.PLATE_TOP + M.BOWL_T]), np.eye(3)
    plan = {i: (c + 5.0, "spoon", False) for i, c in enumerate(M.COOK_AT)}
    assert run(plan, spoon=spoon) == 0.0


def test_taken_late_after_45_s_is_not_good():
    plan = dict(IDEAL)
    plan[0] = (M.COOK_AT[0] + 2.0 + 46.0, "plate", False)
    assert run(plan) == 0.75


def test_raw_piece_taken_out_and_put_back_then_taken_right_counts():
    # G1 lifted raw at 4 s, put back in the water at 7 s, then taken properly: still good
    def spoon(t):
        return np.asarray(M.SPOON_START, float), M.spoon_rot(M.REST_PSI, M.REST_TIP)
    plan = dict(IDEAL)
    out = run(plan, spoon=spoon)  # baseline
    assert out == 1.0
    plan2 = dict(IDEAL)
    plan2[0] = (4.0, "pan", False)  # out at 4 s, carried 3 s, back floating in the pan
    s = run(plan2)
    assert s == 0.75  # it never left again, so it is not on the plate: 5 good, and no raw piece served


def test_hand_left_on_the_spoon_caps_at_05():
    def hand(t, o):
        if t < 115.0:
            return AWAY_R, M.GRIP_DOWN, 1.0
        sp = o["spoon"]
        R = fk.quat_R(sp.quat_xyzw)
        tcp = np.asarray(sp.pos) + R @ M.GRIP_LOCAL
        return tcp, R @ M._HAND_IN_SPOON, M.GRIP_W / (2 * abs(M.FINGER_OPEN))
    assert run(IDEAL, hand=hand) == 0.5


def test_hand_near_but_not_on_the_spoon_is_not_capped():
    def hand(t, o):
        sp = o["spoon"]
        R = fk.quat_R(sp.quat_xyzw)
        tcp = np.asarray(sp.pos) + R @ M.GRIP_LOCAL + np.array([0.0, 0.0, 0.10])
        return tcp, R @ M._HAND_IN_SPOON, 1.0
    assert run(IDEAL, hand=hand) == 1.0


def test_pinch_is_contact_not_proximity():
    o = {n: st(M.G_START[i]) for i, n in enumerate(M.G_NAMES)}
    p = np.asarray(M.G_START[0])
    kit._OPENING["right_gripper"] = 1.0
    o["right_gripper"] = st(p + np.array([0, 0, 0.040]), M.GRIP_DOWN)  # 4 cm over it, open: the pads end 2.6 cm up
    o["left_gripper"] = st(AWAY_L, M.GRIP_DOWN)
    assert not M.pinched(o, "g1")
    kit._OPENING["right_gripper"] = M.G_SIZE[1] / (2 * abs(M.FINGER_OPEN))
    o["right_gripper"] = st(p, M.GRIP_DOWN)
    assert M.pinched(o, "g1")


# -- the text ------------------------------------------------------------------------------------------------------
TJ = json.loads((HERE / "task.json").read_text())
README = (HERE / "README.md").read_text()


def test_task_json_fields_and_limits():
    for k in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert TJ.get(k), k
    lim = {"title": 120, "instruction": 1000, "context": 1000, "setup": 4000, "scoring": 4000, "fidelity": 500,
           "scores": 300}
    for k, n in lim.items():
        assert len(TJ.get(k, "")) <= n, (k, len(TJ.get(k, "")))
    assert TJ["instruction"] == M.INSTRUCTION
    assert TJ["category"] == "Impact on jobs"


def test_numbers_match_the_code():
    sc, su = TJ["scoring"], TJ["setup"]
    assert "45 s" in sc and M.WINDOW_S == 45.0
    assert "3 mm" in sc and M.SURFACED_TOL == 0.003
    assert "2 cm" in sc and M.ON_PLATE_UP == 0.020
    assert "0.5" in sc and M.HELD_CAP == 0.5 and M.BURNER_CAP == 0.5
    assert "8 degrees" in sc and abs(math.degrees(M.KNOB_CLICK) - 8.0) < 1e-9
    for v, t in zip(("10.5", "21.5", "38.5", "49.5", "65.5", "78.5"), M.COOK_AT):
        assert re.search(rf"\b{re.escape(v)}\b", su) and abs(float(v) - (t + M.RISE_S_EST)) < 1e-9, v
    assert "1 1/2 to 2 minutes" in su and (M.FLOAT_MIN, M.FLOAT_MAX) == (90.0, 120.0)
    assert "85, 57 and 30 s" in su and M.DROP_AT == (-85.0, -57.0, -30.0)
    assert "3.0 g" in su and abs(M.G_MASS - 0.0030) < 0.0001
    assert "2.3 cm long, 1.5 cm across and 1.1 cm thick" in su and M.G_SIZE == (0.023, 0.015, 0.011)
    assert "x 14.5, y 12.5" in su and M.KNOB_C == (0.145, 0.125)
    assert "27 degrees" in su and round(-math.degrees(M.REST_TIP)) == 27
    assert "0.1 N m" in su and M.KNOB_FRICTION == 0.10
    assert "starts at 90 degrees" in su and abs(math.degrees(M.KNOB_START) + 90) < 1e-9
    assert "2,400" in TJ["run_limit"] and M.RUN_LIMIT_STEPS == 2400
    for v in set(M.LADDER.values()):
        assert f"{v:g}" in sc
    assert "2.4 L" in su and M.WATER_L == 2.4
    for doc in (README,):
        assert "45 s" in doc and "3 mm" in doc


def test_no_team_identifiers_in_text():
    text = json.dumps(TJ) + README
    for bad in ("Andres", "Andrew", "Andresg324", "Awang", "/Users/", "robocurve-sync", "#1"):
        assert bad not in text, bad
