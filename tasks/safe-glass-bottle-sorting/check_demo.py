"""Check every horizontal demo target with the YAM URDF; no simulator required."""

from pathlib import Path

from yam_arena import station
from yam_arena.task_file import load


def check():
    task = load(Path(__file__).with_name("task.py"))
    results = {}
    for side, waypoints in task.demo().items():
        plan = task.module.RackArmPlan(waypoints, station.BASES[side])
        for step in range(round(plan.duration * 20) + 5):
            plan.action7((step + 1) / 20)
        results[side] = {
            "duration_s": plan.duration,
            "worst_position_error_mm": plan.worst_ik_mm,
            "worst_orientation_error_deg": plan.worst_ik_deg,
        }
        print(side, results[side], flush=True)
        assert plan.worst_ik_mm < 5 and plan.worst_ik_deg < 5
    return results


if __name__ == "__main__":
    check()
