"""Offline self-test for clear_stray_wrench_from_bench_grinder_tool_rest_before_powering_on.

Verifies:
1. Imports and syntax
2. Kinematics reachability (offline IK via ik.solve with arm base offset)
3. Demo waypoint trajectory feasibility and variants
4. Scorer stage transitions and penalties
"""

import os
import sys
import numpy as np

# Add task dir and arena to path
task_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, task_dir)
arena_dir = "/home/freax/Documents/GitHub/yam-isaac-lab-arena"
if arena_dir not in sys.path:
    sys.path.insert(0, arena_dir)

from yam_arena import ik
from yam_arena.demo import grip_rotation, ArmPlan, LEFT_BASE_POS, RIGHT_BASE_POS
import mechkit as mk
import clear_stray_wrench_from_bench_grinder_tool_rest_before_powering_on as task

def test_ik_trajectories():
    print("Testing ArmPlan IK reachability across full trajectory...")
    plans = task.demo()
    lp = ArmPlan(plans["left"], LEFT_BASE_POS)
    rp = ArmPlan(plans["right"], RIGHT_BASE_POS)

    # Step through time at 0.1s intervals
    t = 0.0
    while t <= max(lp.duration, rp.duration):
        if t <= lp.duration:
            lp.action7(t)
        if t <= rp.duration:
            rp.action7(t)
        t += 0.1

    print(f"  Left arm worst IK: {lp.worst_ik_mm:.2f} mm, {lp.worst_ik_deg:.2f} deg")
    print(f"  Right arm worst IK: {rp.worst_ik_mm:.2f} mm, {rp.worst_ik_deg:.2f} deg")
    assert lp.worst_ik_mm < 5.0, f"Left arm IK error too large: {lp.worst_ik_mm} mm"
    assert rp.worst_ik_mm < 5.0, f"Right arm IK error too large: {rp.worst_ik_mm} mm"
    print("All trajectory IK solutions verified!")

def test_demo_waypoints():
    print("Testing scripted demo waypoints across variants...")
    for variant in ["", "no_tray", "no_switch", "no_release"]:
        os.environ["DEMO_VARIANT"] = variant
        demo_dict = task.demo()
        assert "left" in demo_dict and "right" in demo_dict
        t_left = sum(w.duration for w in demo_dict['left'])
        t_right = sum(w.duration for w in demo_dict['right'])
        print(f"  Variant '{variant or 'default'}': L={len(demo_dict['left'])}({t_left:.1f}s), R={len(demo_dict['right'])}({t_right:.1f}s)")
        assert abs(t_left - t_right) < 0.1, f"Trajectory duration mismatch: left {t_left}s vs right {t_right}s"

def test_scoring_logic():
    print("Testing score stages and anti-cheat defect latch...")
    mk.MEMORY.clear()
    assert task.score({}) == 0.0

    # Test defect latch
    mk.MEMORY["defect"] = True
    assert task.score({}) == 0.0
    mk.MEMORY.clear()

    # Stage 1: wrench_cleared
    mk.MEMORY["wrench_cleared"] = True
    assert abs(task.score({}) - 0.20) < 1e-4

    # Stage 2: wrench_in_tray
    mk.MEMORY["wrench_in_tray"] = True
    assert abs(task.score({}) - 0.40) < 1e-4

    # Stage 3: switch_on
    mk.MEMORY["switch_on"] = True
    assert abs(task.score({}) - 0.75) < 1e-4

    # Stage 4: hold steps
    mk.MEMORY["hold"] = task.HOLD_STEPS_REQUIRED
    assert abs(task.score({}) - 1.00) < 1e-4
    assert task.success({}) is True
    print("Scoring stages verified!")

if __name__ == "__main__":
    test_ik_trajectories()
    test_demo_waypoints()
    test_scoring_logic()
    print("\n>>> ALL OFFLINE SELF-TESTS PASSED! <<<")
