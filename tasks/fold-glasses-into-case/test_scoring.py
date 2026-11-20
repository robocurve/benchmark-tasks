"""Offline checks (no simulator) for the fold-the-glasses-into-their-case task.

    PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider test_scoring.py

They cover the parts' real sizes, the snug case (an open temple keeps the lid from shutting), every scoring outcome,
attempts to score without doing the task, success() only at 1.0, the over-centre lid spring, the demo's reach, and
that task.json states the numbers in the code.
"""

from __future__ import annotations

import json
import math
import pathlib
import re
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task as T  # noqa: E402
from mechkit import MechState  # noqa: E402

TASK_JSON = json.loads((HERE / "task.json").read_text())


def _quat(r) -> np.ndarray:
    w, x, y, z = T._wxyz(r)
    return np.array([x, y, z, w])


def glasses(origin=None, rot=None, left=0.0, right=0.0, vel=0.0) -> MechState:
    """A glasses state: origin (world), rotation, temple turns (degrees folded)."""
    origin = np.array(T.GLASSES.pos if origin is None else origin, float)
    rot = np.eye(3) if rot is None else np.asarray(rot, float)
    s = MechState(pos=origin, quat_xyzw=_quat(rot), lin_vel=np.array([vel, 0.0, 0.0]), ang_vel=np.zeros(3),
                  low=np.zeros(3), high=np.zeros(3), origin=origin,
                  joints={"left_temple": -math.radians(left), "right_temple": math.radians(right)})
    p = T.glasses_points(s)
    return MechState(pos=s.pos, quat_xyzw=s.quat_xyzw, lin_vel=s.lin_vel, ang_vel=s.ang_vel, low=p.min(0),
                     high=p.max(0), origin=origin, joints=s.joints)


def case(open_deg=105.0, origin=None, rot=None, vel=0.0) -> MechState:
    origin = np.array(T.CASE.pos if origin is None else origin, float)
    rot = np.eye(3) if rot is None else np.asarray(rot, float)
    q = math.radians(open_deg) - T.OPEN
    lo, hi = origin - np.array([0.03, 0.08, 0.0]), origin + np.array([0.03, 0.08, 0.038])
    return MechState(pos=(lo + hi) / 2, quat_xyzw=_quat(rot), lin_vel=np.array([vel, 0.0, 0.0]), ang_vel=np.zeros(3),
                     low=lo, high=hi, origin=origin, joints={"lid": q})


LENS_DOWN = T._ry(-math.pi / 2)
LENS_UP = T._rz(math.pi) @ T._ry(math.pi / 2)


def in_case_pose(rot=LENS_DOWN, left=81.0, right=90.0, shift=(0.0, 0.0, 0.0)) -> MechState:
    """Folded glasses lying flat on the case floor, centred in it."""
    g = glasses(origin=np.zeros(3), rot=rot, left=left, right=right)
    pts = T.glasses_points(g)
    c = np.array(T.CASE.pos)
    origin = np.array([c[0], c[1], c[2] + T.IN_Z[0]]) - np.array([(pts[:, 0].max() + pts[:, 0].min()) / 2,
                                                                     (pts[:, 1].max() + pts[:, 1].min()) / 2,
                                                                     pts[:, 2].min()]) + np.array(shift)
    return glasses(origin=origin, rot=rot, left=left, right=right)


@pytest.fixture(autouse=True)
def fresh():
    T._reset()
    T.OFFLINE_RELEASED = True
    yield
    T.OFFLINE_RELEASED = False


def run(states, steps):
    out = []
    for _ in range(steps):
        objs = {"glasses": states[0], "glasses_case": states[1]}   # a new dict each step, as the runtime gives
        out.append((T.score(objs), T.success(objs)))
    return out


# -- real sizes ---------------------------------------------------------------------------------------------------------

def test_glasses_real_size():
    g = glasses()
    assert abs((g.high[1] - g.low[1]) - 0.139) < 0.0005          # 139 mm across (52-18-140)
    assert abs(T.FRONT_H - 0.042) < 1e-9
    assert abs(T.TEMPLE_L + T.EAR_L - 0.140) < 1e-9              # temples 140 mm along their length
    assert abs(T.FRONT_MASS + 2 * T.TEMPLE_MASS - 0.025) < 1e-9  # 25 g
    assert T.TEMPLE_T == 0.004 and T.TEMPLE_H == 0.007
    assert abs(T.LENS_W - 0.052) < 1e-9 and abs(T.BRIDGE - 0.018) < 1e-9


def test_case_real_size():
    assert (T.CASE_L, T.CASE_W, T.BASE_H + T.LID_H) == (0.160, 0.060, 0.038)
    assert abs(T.CASE_MASS + T.LID_MASS - 0.090) < 1e-9
    assert abs(2 * T.IN_X - 0.056) < 1e-9 and abs(2 * T.IN_Y - 0.156) < 1e-9
    assert abs(T.IN_Z[1] - T.IN_Z[0] - 0.034) < 1e-9


def test_glasses_start_standing_on_rims_and_earpiece_tips():
    g = glasses()
    pts = T.glasses_points(g)
    assert abs(pts[:, 2].min() - (T.TOP + 0.0005)) < 1e-6
    ends = [T.mechkit.body_poses(T.GLASSES, g)[n] for n in ("left_temple", "right_temple")]
    for rb, ob in ends:
        tip = ob + rb @ (np.array([T.TEMPLE_L, 0, 0]) + T._ry(T.EAR_DROP) @ np.array([T.EAR_L, 0, 0]))
        assert tip[2] - T.TOP < 0.004                            # each earpiece tip within 4 mm of the table


# -- the snug case --------------------------------------------------------------------------------------------------

def _stack_height(rot, left, right) -> float:
    g = glasses(origin=np.zeros(3), rot=rot, left=left, right=right)
    p = T.glasses_points(g)
    return float(p[:, 2].max() - p[:, 2].min())


@pytest.mark.parametrize("rot", [LENS_DOWN, LENS_UP])
def test_folded_glasses_fit_under_the_shut_lid(rot):
    room = T.IN_Z[1] - T.IN_Z[0]
    assert _stack_height(rot, 81.0, 90.0) < room - 0.002     # as the demo leaves them
    g = in_case_pose(rot)
    assert T.in_case(g, case())


@pytest.mark.parametrize("rot", [LENS_DOWN, LENS_UP])
@pytest.mark.parametrize("open_side", ["left", "right"])
def test_a_temple_open_15_degrees_keeps_the_lid_from_shutting(rot, open_side):
    """Lying flat on the floor, glasses with a temple 15 degrees or more short of folded stand taller than the case's
    inside: the lid rests on them, so a shut lid means both temples are folded (and is a photo check)."""
    room = T.IN_Z[1] - T.IN_Z[0]
    for off in (15.0, 20.0, 30.0, 45.0, 60.0, 90.0):
        left, right = (90.0 - off, 90.0) if open_side == "left" else (83.0, 90.0 - off)
        assert _stack_height(rot, left, right) > room, off


def test_glasses_on_edge_do_not_fit():
    assert T.FRONT_H > T.IN_Z[1] - T.IN_Z[0]


def test_open_glasses_are_never_inside():
    g = in_case_pose(left=0.0, right=0.0)
    assert not T.in_case(g, case())


def test_half_outside_is_not_inside():
    assert not T.in_case(in_case_pose(shift=(0.0, 0.04, 0.0)), case())
    assert not T.in_case(in_case_pose(shift=(0.0, 0.0, 0.03)), case())


# -- scoring outcomes ---------------------------------------------------------------------------------------------------

def test_start_scores_zero():
    assert run((glasses(), case()), 30)[-1] == (0.0, False)


@pytest.mark.parametrize("left,right,want", [(80.0, 0.0, 0.25), (0.0, 76.0, 0.25), (74.0, 0.0, 0.0),
                                             (83.0, 90.0, 0.5), (60.0, 90.0, 0.25)])
def test_fold_tiers(left, right, want):
    assert run((glasses(left=left, right=right), case()), 25)[-1] == (want, False)


def test_folded_inside_open_case_is_075():
    assert run((in_case_pose(), case()), 25)[-1] == (0.75, False)


def test_full_score_needs_one_second():
    res = run((in_case_pose(), case(open_deg=0.0)), 25)
    assert all(s == 0.75 and not ok for s, ok in res[:19])
    assert all(s == 1.0 and ok for s, ok in res[19:])


@pytest.mark.parametrize("rot", [LENS_DOWN, LENS_UP])
def test_either_way_up_scores(rot):
    assert run((in_case_pose(rot), case(open_deg=0.0)), 21)[-1] == (1.0, True)


def test_lid_gap_limit():
    assert run((in_case_pose(), case(open_deg=math.degrees(math.asin(0.0009 / 0.060)))), 21)[-1] == (1.0, True)
    T._reset()
    assert run((in_case_pose(), case(open_deg=math.degrees(math.asin(0.0012 / 0.060)))), 21)[-1] == (0.75, False)


def test_still_held_is_not_full():
    T.OFFLINE_RELEASED = False
    assert run((in_case_pose(), case(open_deg=0.0)), 40)[-1] == (0.75, False)


def test_moving_is_not_full():
    assert run((in_case_pose(), case(open_deg=0.0, vel=0.01)), 40)[-1] == (0.75, False)
    T._reset()
    assert run((glasses(vel=0.01), case(open_deg=0.0)), 40)[-1] == (0.0, False)


def test_case_on_its_side_is_not_full():
    rot = T._rz(0.0) @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
    c = case(open_deg=0.0, rot=rot)
    g = in_case_pose()
    assert not T.complete(g, c)


def test_falling_off_the_table_is_zero_for_good():
    g = in_case_pose()
    fell = glasses(origin=np.array(T.GLASSES.pos) - np.array([0, 0, 0.3]))
    run((fell, case()), 1)
    assert run((g, case(open_deg=0.0)), 40)[-1] == (0.0, False)


def test_score_drops_when_conditions_break():
    run((in_case_pose(), case(open_deg=0.0)), 25)
    assert run((in_case_pose(), case(open_deg=30.0)), 1)[-1] == (0.75, False)


# -- attempts to score without doing the task ---------------------------------------------------------------------------

def test_shutting_the_empty_case_scores_nothing():
    assert run((glasses(), case(open_deg=0.0)), 40)[-1] == (0.0, False)


def test_case_shut_on_glasses_half_outside():
    g = in_case_pose(shift=(0.0, 0.05, 0.0))
    assert run((g, case(open_deg=0.0)), 40)[-1] == (0.5, False)


def test_unfolded_glasses_in_the_case_are_not_inside():
    g = in_case_pose(left=40.0, right=90.0)
    assert run((g, case(open_deg=20.0)), 40)[-1] == (0.25, False)


def test_success_only_at_full_score():
    for st in [(glasses(), case()), (glasses(left=83, right=90), case(open_deg=0.0)), (in_case_pose(), case())]:
        T._reset()
        for s, ok in run(st, 40):
            assert ok == (s >= 1.0)


# -- the lid's spring -------------------------------------------------------------------------------------------------

def test_over_centre_spring():
    assert T.lid_rest(0.0) == 0.0
    assert T.lid_rest(-math.radians(9.0)) == 0.0                  # within 10 degrees of open it holds it open
    assert T.lid_rest(-math.radians(11.0)) == -T.OPEN - T.PRELOAD   # past that it snaps it shut and keeps pressing
    assert T.lid_rest(-T.OPEN) == -T.OPEN - T.PRELOAD
    # the nudge it takes from open: under the force that would slide or tip the 90 g case (case on table 0.6)
    nudge = (T.LID_SPRING * T.FLIP) / 0.050                       # N at 50 mm up the lid
    assert nudge < 0.6 * (T.CASE_MASS + T.LID_MASS) * 9.81 / 2
    shut = T.LID_SPRING * T.PRELOAD                               # N m holding it shut, besides its weight
    assert shut > 0.01


def test_hinge_hold_beats_a_temples_weight():
    worst = T.TEMPLE_MASS * 9.81 * (T.TEMPLE_L / 2 + 0.01)        # temple horizontal, hinge axis horizontal
    assert T.HINGE_HOLD > 1.5 * worst


# -- the demo --------------------------------------------------------------------------------------------------------

def test_demo_reaches_and_fits_the_run_limit():
    import demo

    plan = demo.offline_plan(T, solve=True)
    limit = int(re.match(r"([\d,]+)", TASK_JSON["run_limit"]).group(1).replace(",", ""))
    assert plan["steps"] + 80 < limit
    assert plan["ctx"].left.worst_mm < 3.0 and plan["ctx"].right.worst_mm < 3.0


# -- task.json --------------------------------------------------------------------------------------------------------

LIMITS = {"title": 120, "context": 1000, "scores": 300, "fidelity": 500, "setup": 4000, "scoring": 4000,
          "instruction": 1000, "run_limit": 200}


def test_task_json_fields_and_limits():
    for k in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert TASK_JSON.get(k), k
    for k, n in LIMITS.items():
        assert len(TASK_JSON[k]) <= n, (k, len(TASK_JSON[k]))
    assert len(", ".join(TASK_JSON["keywords"])) <= 200
    assert TASK_JSON["instruction"] == T.INSTRUCTION
    assert TASK_JSON["category"] == "Societal relatability"


def test_task_json_states_the_code_numbers():
    text = json.dumps(TASK_JSON)
    for s in ("52-18-140", "139 mm", "140 mm", "25 g", "160 x 60 x 38 mm", "90 g", "105 degrees", "0.01 N m",
              "15 degrees", "1 mm", "3 cm", "1 s", "0.75", "0.5", "0.25"):
        assert s in text, s


def test_no_identifying_text():
    text = json.dumps(TASK_JSON) + (HERE / "README.md").read_text()
    for pat in ("/Users/", "/home/", "@"):
        assert pat not in text, pat
