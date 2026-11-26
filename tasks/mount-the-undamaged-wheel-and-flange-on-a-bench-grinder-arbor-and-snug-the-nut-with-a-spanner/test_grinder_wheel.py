"""Pure CPU tests of the bench-grinder wheel task: maker/standard figures, right-hand M12 torque sign (r x F),
one-way relieved flange geometry, thread engagement, every score level (0.0, 0.5, 1.0), every latched zero rule,
and checklist / character / blind-review / no-pose-snap invariants."""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
FAR = {"left": np.array([[0.0, 0.35, 1.20]]), "right": np.array([[0.0, -0.35, 1.20]])}
EYE = np.eye(3)


@pytest.fixture
def m():
    before = {name: sys.modules.get(name) for name in ("mechkit", "parts", "scene", "checks", "demo")}
    module = task_file.load(TASK).module
    module.begin(0)
    module.mk.mend_all()
    yield module
    module.mk.MECH.clear()
    for name, old in before.items():
        sys.modules.pop(name, None)
        if old is not None:
            sys.modules[name] = old


def _seated_poses(m, *, backwards_flange: bool = False, wrench_on_nut: bool = False) -> dict:
    p = m.P
    flange_rot = m.mk.axis_angle((0.0, 0.0, 1.0), math.pi if backwards_flange else 0.0)
    poses = {
        "wheel_sound": (np.array([p.GRINDER_X, p.Y_WHEEL_SEATED, m.TOP + p.ARBOR_Z]), EYE),
        "outer_flange": (np.array([p.GRINDER_X, p.Y_FLANGE_SEATED, m.TOP + p.ARBOR_Z]), flange_rot),
    }
    if wrench_on_nut:  # left on the hex: not set down
        poses["wrench"] = (np.array([p.GRINDER_X - 0.002, p.Y_NUT_SNUG - 0.0015, m.TOP + p.ARBOR_Z]), EYE)
    return poses


def _put_nut_engaged(m, phi_deg: float, poses: dict | None = None, contacts: dict | None = None, hands=FAR) -> None:
    p = m.P
    rad = math.radians(phi_deg)
    nut_rot = m.mk.axis_angle((0.0, 1.0, 0.0), rad)
    all_poses = {
        **(poses or {}),
        "arbor_nut": (np.array([p.GRINDER_X, p.Y_NUT_SNUG, m.TOP + p.ARBOR_Z]), nut_rot),
    }
    m.mk.set_state(poses=all_poses, joints={"nut_turn": rad}, contacts=contacts or {})
    m.STATE["snug_ref"] = 0.0
    m.STATE["nut_held"] = rad
    m.mk.make("nut_catch")
    m.STATE["hands"] = hands


def _steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def test_maker_and_standard_figures_and_no_spindle_lock_button(m):
    p = m.P
    assert (p.WHEEL_D, p.WHEEL_T, p.ARBOR_D) == pytest.approx((0.150, 0.016, 0.0127))
    assert (p.WRENCH_SW, p.WRENCH_L, p.WRENCH_HEAD_W, p.WRENCH_T, p.WRENCH_MASS) == pytest.approx((0.019, 0.170, 0.040, 0.007, 0.081))
    assert p.FLANGE_D >= p.WHEEL_D / 3.0 and p.FLANGE_RELIEF >= p.FLANGE_MIN_RELIEF
    assert p.ARBOR_Z == pytest.approx(0.098)
    # No invented spindle-lock button; continuous spindle + nut_turn + nut_catch only
    assert "lock_btn" not in m.mk.MECH.bodies and "lock_btn" not in m.mk.MECH.joints
    assert sorted(m.mk.MECH.joints) == ["nut_catch", "nut_turn", "spindle"]
    # Kit rule: never weld two bodies that a hinge already joins
    hinge_pairs = {frozenset((j.child, j.parent)) for j in m.mk.MECH.joints.values() if j.kind != "weld"}
    weld_pairs = {frozenset((j.child, j.parent)) for j in m.mk.MECH.joints.values() if j.kind == "weld"}
    assert hinge_pairs.isdisjoint(weld_pairs)


def test_right_hand_thread_torque_sign_and_witness_window(m):
    p = m.P
    # Spanner handle pointing -x (r_x = -0.132 m) pulled UP (+z, F_z = +32 N) gives +y tightening torque > 0!
    tau_up = p.wrench_torque_y((-0.132, 0.0, 0.0), (0.0, 0.0, 32.0))
    tau_down = p.wrench_torque_y((-0.132, 0.0, 0.0), (0.0, 0.0, -32.0))
    assert tau_up == pytest.approx(+4.224) and tau_down < 0.0
    assert p.thread_torque_for_angle_deg(p.ANGLE_MIN_DEG) == pytest.approx(p.TAU_MIN, abs=0.12)
    assert p.thread_torque_for_angle_deg(p.ANGLE_MAX_DEG) == pytest.approx(p.TAU_MAX, abs=0.12)
    assert p.thread_torque_for_angle_deg(p.ANGLE_CRACK_DEG) >= p.TAU_CRACK


def test_idle_scores_zero_and_cracked_wheel_latches_zero(m):
    m.mk.set_state()
    m.STATE["hands"] = FAR
    assert _steps(m, 5) == 0.0 and m.STATE["zero"] is None

    # Moving cracked wheel > 10 mm latches 0.0 even if sound stack is then completed
    c_pos, c_rot = m.mk.built_pose("wheel_cracked")
    m.mk.set_state(poses={"wheel_cracked": (c_pos + np.array([0.0, -0.015, 0.0]), c_rot)})
    m.STATE["hands"] = FAR
    assert _steps(m, 1) == 0.0 and "cracked wheel" in m.STATE["zero"]
    _put_nut_engaged(m, 36.0, poses=_seated_poses(m))
    assert _steps(m, m.P.HOLD_STEPS + 3) == 0.0


def test_sound_wheel_and_recessed_inward_flange_earn_half_before_wrench(m):
    # Intact wheel + outer flange seated concave-recess-inward -> 0.5 immediately (score does not rest solely on turn!)
    m.mk.set_state(poses=_seated_poses(m, backwards_flange=False))
    m.STATE["hands"] = FAR
    assert m.wheel_seated() and m.flange_seated() and m.flange_rim_gap() < 0.001
    assert _steps(m, 3) == 0.5

    # Reversing outer flange (convex hub inward) leaves its Ø 52 mm outer bearing rim 8 mm clear -> 0.0!
    m.begin(0)
    m.mk.set_state(poses=_seated_poses(m, backwards_flange=True))
    m.STATE["hands"] = FAR
    assert m.wheel_seated() and not m.flange_seated() and m.flange_rim_gap() == pytest.approx(0.008, abs=5e-4)
    assert _steps(m, 3) == 0.0


def test_nut_thread_engages_only_on_contact_and_clockwise_turn(m):
    base = _seated_poses(m)
    p = m.P
    # Nearness alone without contact or turn never engages nut_catch
    m.mk.set_state(poses={**base, "arbor_nut": (np.array([p.GRINDER_X, p.Y_NUT_SNUG, m.TOP + p.ARBOR_Z]), EYE)})
    m.STATE["hands"] = FAR
    _steps(m, 3)
    assert not m.nut_engaged()

    # Turning 27 deg on the bare thread, 10 mm short of the flange (no flange contact), never engages nut_catch
    for deg in (0.0, 9.0, 18.0, 27.0):
        rot = m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(deg))
        m.mk.set_state(
            poses={**base, "arbor_nut": (np.array([p.GRINDER_X, p.Y_NUT_SNUG - 0.010, m.TOP + p.ARBOR_Z]), rot)},
            contacts={},
        )
        m.STATE["hands"] = FAR
        m.step()
    assert not m.nut_engaged()
    m.begin(0)

    # Contact + >= 16 deg clockwise (+y) rotation engages nut_catch in place at 0 deg witness angle
    for deg in (0.0, 9.0, 18.0):
        rot = m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(deg))
        m.mk.set_state(
            poses={**base, "arbor_nut": (np.array([p.GRINDER_X, p.Y_NUT_SNUG, m.TOP + p.ARBOR_Z]), rot)},
            contacts={("outer_flange", "arbor_nut"): 2.0},
        )
        m.STATE["hands"] = FAR
        m.step()
    assert m.nut_engaged() and m.witness_angle_deg() == pytest.approx(0.0, abs=1e-6)


def test_under_snug_window_full_hold_and_over_torque_zero(m):
    # Under 17 deg past snug (e.g. 12 deg = 1.8 N m): stays 0.5
    _put_nut_engaged(m, 12.0, poses=_seated_poses(m))
    assert _steps(m, m.P.HOLD_STEPS + 2) == 0.5

    # Inside [17, 195] deg (36 deg = 5.12 N m), spanner parked, hands clear: 0.5 until 20-step hold completes, then 1.0
    m.begin(0)
    m.mk.mend_all()
    _put_nut_engaged(m, 36.0, poses=_seated_poses(m))
    assert _steps(m, m.P.HOLD_STEPS - 1) == 0.5
    assert _steps(m, 1) == 1.0 and m.success(None)
    # Drive rest stays well within 45 deg of joint angle
    assert abs(m.STATE["nut_held"] - m.mk.joint("nut_turn")) < math.radians(10.0)

    # Spanner not parked (> 30 mm from cradle) caps at 0.5
    m.begin(0)
    m.mk.mend_all()
    _put_nut_engaged(m, 36.0, poses=_seated_poses(m, wrench_on_nut=True))
    assert _steps(m, m.P.HOLD_STEPS + 2) == 0.5

    # Over-tightening > 195 deg (> 27.1 N m, the maker limit) permanently latches 0.0
    m.begin(0)
    m.mk.mend_all()
    _put_nut_engaged(m, 200.0, poses=_seated_poses(m))
    assert _steps(m, 1) == 0.0 and "over-tightened" in m.STATE["zero"]
    _put_nut_engaged(m, 36.0, poses=_seated_poses(m))
    assert _steps(m, m.P.HOLD_STEPS + 2) == 0.0


def test_drop_off_bench_and_hand_clearance_rules(m):
    m.mk.set_state(poses={"wrench": (np.array([0.25, -0.30, m.TOP - 0.030]), EYE)})
    m.STATE["hands"] = FAR
    assert _steps(m, 1) == 0.0 and m.STATE["zero"] == "wrench dropped off the bench"

    m.begin(0)
    m.mk.mend_all()
    near_hand = {"left": FAR["left"], "right": np.array([[m.P.GRINDER_X, m.P.Y_WHEEL_SEATED, m.TOP + m.P.ARBOR_Z + 0.085]])}
    _put_nut_engaged(m, 36.0, poses=_seated_poses(m), hands=near_hand)
    assert m.hands_clear() < m.P.HANDS_CLEAR
    assert _steps(m, m.P.HOLD_STEPS + 2) == 0.5


def test_folder_and_plan_invariants():
    sys.path.insert(0, str(HERE))
    import checks as K
    import demo as D

    poses = K.poses()
    assert len(poses) >= 20 and all(p[1][2] - K.TOP <= 0.150 + 1e-6 for p in poses.values())
    plan = D.plan()
    assert set(plan) == {"left", "right"} and all(len(v) > 5 for v in plan.values())

    readme = (HERE / "README.md").read_text()
    task_code = (HERE / "task.py").read_text()
    spec = json.loads((HERE / "task.json").read_text())
    assert len(readme) <= 10_000 and len(task_code) <= 15_000
    assert len(spec["title"]) <= 120 and len(spec["context"]) <= 1_000 and len(spec["fidelity"]) <= 500
    assert spec["scores"] == "Dream-1 run: pending; demo: 1.0"
    low = readme.lower()
    assert "earlier review" not in low and "reviewer" not in low and "thank" not in low

    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json"):
            txt = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", txt) and not re.search(r"#\d{3,}", txt), path.name

    for name in ("task.py", "scene.py", "demo.py", "parts.py", "checks.py"):
        code = (HERE / name).read_text()
        assert "write_pose" not in code and "set_joint" not in code, name
