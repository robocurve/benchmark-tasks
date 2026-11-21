"""Offline self-test for indoor_griddle_preparation: scoring cases on synthetic states,
geometry consistency, and kinematics limits.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import indoor_griddle_preparation as igp
import mechkit as mk
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load


def _fake_states(
    probe_x=igp.PROBE_START_X,
    probe_y=igp.SHROUD_Y,
    probe_z=igp.PROBE_Z,
    dial_deg=0.0,
    bread0_pos=igp.BREAD_START[0],
    bread1_pos=igp.BREAD_START[1],
    bread_flat=True,
    griddle_pos=(igp.PAN_XY[0], igp.PAN_XY[1], igp.PAN_Z),
    griddle_tilt=0.0,
    probe_latched=False,
    fell_off=False,
):
    q_dict = {
        "dial_turn": math.radians(dial_deg),
    }
    r_bread = np.eye(3) if bread_flat else mk.axis_angle((1, 0, 0), math.radians(45.0))
    q_bread = mk.mat_to_quat(r_bread)
    r_griddle = mk.axis_angle((0, 1, 0), math.radians(griddle_tilt))
    q_griddle = mk.mat_to_quat(r_griddle)

    moved = {
        "griddle": (np.array(griddle_pos), q_griddle),
        "probe": (np.array([probe_x, probe_y, probe_z]), np.array([0.0, 0.0, 0.0, 1.0])),
        "bread_0": (np.array(bread0_pos), q_bread),
        "bread_1": (np.array(bread1_pos), q_bread),
    }
    if fell_off:
        moved["griddle"] = (np.array([igp.PAN_XY[0], igp.PAN_XY[1], igp.TOP - 0.10]), q_griddle)

    st = igp.kit.fake_states(q=q_dict, moved=moved)
    if probe_latched:
        mk.MEMORY["probe_latched"] = True
    return st


def test_scoring_cases():
    dest0 = (igp.PAN_XY[0] + 0.035, igp.PAN_XY[1] - 0.065, igp.COOK_TOP + igp.BREAD_SIZE[2] / 2)
    dest1 = (igp.PAN_XY[0] - 0.060, igp.PAN_XY[1] + 0.100, igp.COOK_TOP + igp.BREAD_SIZE[2] / 2)
    seated_x = igp.PROBE_SEATED_X

    cases = [
        ("start layout (nothing done)", _fake_states(), [], 0.0, False),
        ("probe seated, dial off, food on plate", _fake_states(probe_x=seated_x, probe_latched=True), [], 0.25, False),
        ("probe seated, dial 90 deg ON, food on plate", _fake_states(probe_x=seated_x, dial_deg=90.0, probe_latched=True), [], 0.50, False),
        ("probe seated, dial 90 deg, 1 bread placed", _fake_states(probe_x=seated_x, dial_deg=90.0, bread0_pos=dest0, probe_latched=True), [], 0.75, False),
        ("full completion: probe seated, dial 90 deg, both bread flat, arms clear", _fake_states(probe_x=seated_x, dial_deg=90.0, bread0_pos=dest0, bread1_pos=dest1, probe_latched=True), [], 1.0, True),
        ("both bread placed but arm still touching griddle", _fake_states(probe_x=seated_x, dial_deg=90.0, bread0_pos=dest0, bread1_pos=dest1, probe_latched=True), [(igp.PAN_XY[0], igp.PAN_XY[1], igp.PAN_Z)], 0.75, False),
        ("both bread placed but probe not seated", _fake_states(bread0_pos=dest0, bread1_pos=dest1), [], 0.0, False),
        ("griddle knocked off counter", _fake_states(probe_x=seated_x, dial_deg=90.0, bread0_pos=dest0, bread1_pos=dest1, probe_latched=True, fell_off=True), [], 0.0, False),
        ("griddle tilted over 10 degrees", _fake_states(probe_x=seated_x, dial_deg=90.0, bread0_pos=dest0, bread1_pos=dest1, probe_latched=True, griddle_tilt=15.0), [], 0.0, False),
        ("bread placed tilted (not resting flat)", _fake_states(probe_x=seated_x, dial_deg=90.0, bread0_pos=dest0, bread1_pos=dest1, bread_flat=False, probe_latched=True), [], 0.50, False),
    ]

    for name, st, tcps, expected_score, expected_success in cases:
        mk.new_episode()
        igp._TEST_TCPS = tcps
        igp._TEST_ROBOT_BOXES = []
        try:
            igp._step_rule(st)
            got_score = igp.score(st)
            got_success = igp.success(st)
            assert abs(got_score - expected_score) < 1e-6, f"{name}: expected score {expected_score}, got {got_score}"
            assert got_success == expected_success, f"{name}: expected success {expected_success}, got {got_success}"
            print(f"PASS: {name} -> score={got_score:.2f}, success={got_success}")
        finally:
            igp._TEST_TCPS = None
            igp._TEST_ROBOT_BOXES = None
            mk.new_episode()


def test_kinematics_and_ik():
    task = load(Path(__file__).resolve().parent / "indoor_griddle_preparation.py")
    plans = task.demo()
    hz = 20.0
    for arm, wps in plans.items():
        plan = ArmPlan(list(wps), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, f"{arm} worst IK: {plan.worst_ik_mm} mm, {plan.worst_ik_deg} deg"
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), f"{arm} exceeded joint limits"
        print(f"PASS: {arm} plan duration={plan.duration:.1f}s, worst IK {plan.worst_ik_mm:.3f}mm / {plan.worst_ik_deg:.3f}deg")


if __name__ == "__main__":
    print("Running scoring test suite...")
    test_scoring_cases()
    print("\nRunning kinematics & IK test suite...")
    test_kinematics_and_ik()
    print("\nAll offline checks passed successfully!")
