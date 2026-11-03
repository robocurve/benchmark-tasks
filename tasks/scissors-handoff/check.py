"""Offline check of scissors_handoff.py (no simulator). Run from the repository root with yam_arena importable:

    python tasks/scissors-handoff/check.py

It tests the scoring rules on hand-built states of the scissors. Then it plays the left arm's demo through the IK
at 20 Hz and prints the worst IK error, the joint limits and the gripper's clearance from the hand.
"""

import importlib.util
import math
from pathlib import Path

import numpy as np

from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState

spec = importlib.util.spec_from_file_location("task", Path(__file__).with_name("scissors_handoff.py"))
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)

HALF = np.array([0.048, 0.1008, T.HALF_THICKNESS]) * T.SCALE  # half size of the model's bounding box


def state(centre_xy, bottom_z, yaw, moving=False) -> dict:
    """The scissors lying flat, bounding-box centre over centre_xy, lowest point at bottom_z."""
    q = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    pos = np.array([*centre_xy, bottom_z + HALF[2]])
    corners = np.array([[sx, sy] for sx in (-1, 1) for sy in (-1, 1)]) * HALF[:2] @ T._rz(yaw).T
    half = np.r_[np.abs(corners).max(0), HALF[2]]
    vel = np.array([0.1, 0.0, 0.0]) if moving else np.zeros(3)
    return {"scissors": ObjectState(pos, q, vel, np.zeros(3), pos - half, pos + half)}


def centre_for_handle(handle_xy, yaw):
    return np.asarray(handle_xy) - T._rz(yaw) @ ((T.HANDLE_POINT - T.MODEL_CENTRE) * T.SCALE)


def check_scores():
    end = centre_for_handle(T.END_HANDLE, T.END_YAW)
    above = T.PALM_TOP + 0.08
    cases = {
        "start, on the table": (state(T.START_XY, T.TOP, T.START_YAW), 0.0),
        "on the palm, blades out past the little finger": (state(end, T.PALM_TOP, T.END_YAW), 1.0),
        "on the palm, blades over the wrist": (state(centre_for_handle(T.END_HANDLE, T.START_YAW), T.PALM_TOP, T.START_YAW), 0.0),
        "on the palm, blades over the fingers": (
            state(centre_for_handle(T.END_HANDLE, math.pi - T.BLADE_ANGLE), T.PALM_TOP, math.pi - T.BLADE_ANGLE),
            0.0,
        ),
        "held 8 cm over the palm, blades out": (state(end, above, T.END_YAW), T.HELD_CREDIT),
        "on the palm, still moving": (state(end, T.PALM_TOP, T.END_YAW, moving=True), T.HELD_CREDIT),
        "on the table next to the hand": (state(end + (0.0, -0.10), T.TOP, T.END_YAW), 0.0),
        "held at the start, 5 cm up": (state(T.START_XY, T.TOP + 0.05, T.START_YAW), 0.0),
    }
    for name, (objects, want) in cases.items():
        got = T.score(objects)
        print(f"  {got:.3f}  {name}")
        assert abs(got - want) < 1e-9, f"{name}: score {got}, want {want}"
    print(f"  start distance d0 = {T.START_DISTANCE * 100:.1f} cm")


def check_demo():
    plan = ArmPlan(T.demo()["left"], station.BASES["left"])
    lo, hi = np.full(6, np.inf), np.full(6, -np.inf)
    for t in np.arange(0.0, plan.duration + 3.0, 1 / 20):
        a, _ = plan.action7(t)
        lo, hi = np.minimum(lo, a[:6]), np.maximum(hi, a[:6])
    margin = np.minimum(lo - ik.LOWER, ik.UPPER - hi)
    print(f"  worst IK {plan.worst_ik_mm:.2f} mm / {plan.worst_ik_deg:.3f} deg over {plan.duration:.1f} s")
    print(f"  smallest joint-limit margin per joint (deg): {np.round(np.degrees(margin), 1)}")
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1 and (margin > 0).all()


if __name__ == "__main__":
    print("scores:")
    check_scores()
    print("demo:")
    check_demo()
    print("ok")
