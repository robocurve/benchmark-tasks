"""Offline self-test for clear_loose_hex_wrench_from_benchtop_miter_saw_turntable_before_lowering_blade."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import clear_loose_hex_wrench_from_benchtop_miter_saw_turntable_before_lowering_blade as miter
import mechkit as mk
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load


def _fake_states(
    wrench_pos=(miter.WRENCH_INIT_XY[0], miter.WRENCH_INIT_XY[1], miter.WRENCH_INIT_Z),
    pivot_deg=30.0,
):
    q_dict = {
        "saw_pivot": pivot_deg,
    }
    moved = {
        "hex_wrench": (np.array(wrench_pos), np.array([0.0, 0.0, 0.0, 1.0])),
    }
    return miter.kit.fake_states(q=q_dict, moved=moved, deg=True)


def test_scoring_cases():
    tray_pos = (miter.TRAY_XY[0], miter.TRAY_XY[1], miter.TOP + 0.015)
    lifted_pos = (miter.WRENCH_INIT_XY[0], miter.WRENCH_INIT_XY[1], miter.TURNTABLE_Z + 0.050)

    # 1. Start layout: wrench on turntable
    mk.new_episode()
    st = _fake_states()
    miter._on_step(st)
    assert miter.score(st) == 0.0

    # 2. Wrench lifted clear
    mk.new_episode()
    st = _fake_states(wrench_pos=lifted_pos)
    miter._on_step(st)
    assert miter.score(st) == 0.20

    # 3. Wrench placed in side tray
    mk.new_episode()
    st = _fake_states(wrench_pos=tray_pos)
    mk.MEMORY["wrench_cleared"] = True
    miter._on_step(st)
    assert miter.score(st) == 0.40

    # 4. Wrench in tray, cutting head bottomed out
    mk.new_episode()
    st = _fake_states(wrench_pos=tray_pos, pivot_deg=4.0)
    mk.MEMORY["wrench_cleared"] = True
    mk.MEMORY["wrench_in_tray"] = True
    miter._on_step(st)
    assert miter.score(st) == 0.75

    # 5. Full completion with stillness hold
    mk.new_episode()
    st = _fake_states(wrench_pos=tray_pos, pivot_deg=4.0)
    mk.MEMORY["wrench_cleared"] = True
    mk.MEMORY["wrench_in_tray"] = True
    mk.MEMORY["head_bottomed"] = True
    mk.MEMORY["hold"] = 20
    assert miter.score(st) == 1.00

    # 6. Safety hazard: head lowered while wrench still on turntable
    mk.new_episode()
    st = _fake_states(pivot_deg=10.0)
    miter._on_step(st)
    assert miter.score(st) == 0.0
    assert mk.MEMORY.get("defect") is True

    print("All scoring cases passed!")


def test_kinematics_load():
    task = load(Path(__file__).resolve().parent / "clear_loose_hex_wrench_from_benchtop_miter_saw_turntable_before_lowering_blade.py")
    assert len(task.objects) >= 4
    assert callable(task.demo)
    plans = task.demo()
    hz = 20.0
    for arm, wps in plans.items():
        plan = ArmPlan(list(wps), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 0.2, f"{arm} worst IK: {plan.worst_ik_mm} mm, {plan.worst_ik_deg} deg"
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), f"{arm} exceeded joint limits"
        print(f"PASS: {arm} plan duration={plan.duration:.1f}s, worst IK {plan.worst_ik_mm:.3f}mm / {plan.worst_ik_deg:.3f}deg")


if __name__ == "__main__":
    test_scoring_cases()
    test_kinematics_load()
