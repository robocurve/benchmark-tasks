"""Offline checks for ``press_button_through_hole.py``: no GPU and no Isaac Sim.

1. **The port blocks the gripper.** The YAM's own collision hulls are placed by URDF forward
   kinematics with the fingers closed, and the hand's cross-section perpendicular to the approach
   axis is measured as a function of depth behind the pad tips. That says how deep the closed hand
   can enter a square hole of a given size -- the construct-validity claim the task rests on.
2. **The demo is reachable.** Every waypoint is run through ``yam_arena.demo.ArmPlan``, which solves
   the same DLS IK the scripted policy uses, and the worst position and rotation error is reported.
3. **The hand clears the enclosure** along the insertion stroke.

Run:  python check_reach.py
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from yam_arena import YAM_URDF, analysis  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, LEFT_BASE_POS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import forward_kinematics  # noqa: E402

_spec = importlib.util.spec_from_file_location("pbth_task", HERE / "press_button_through_hole.py")
task = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(task)
cage = task.cage

PASS, FAIL = "PASS", "FAIL"
results: list[tuple[str, str, str]] = []


def record(name: str, ok: bool, detail: str) -> None:
    results.append((PASS if ok else FAIL, name, detail))


def hand_profile() -> np.ndarray:
    """The hand's points in the gripper frame, fingers closed (approach axis +z)."""
    q = {f"joint{i}": 0.0 for i in range(1, 7)}
    q["joint7"] = q["joint8"] = FINGER_CLOSED
    fk = forward_kinematics(YAM_URDF, q)
    inv = np.linalg.inv(fk["gripper"])
    hulls = analysis.link_hulls()
    pts = []
    for link in ("gripper", "tip_left", "tip_right"):
        t = inv @ fk[link]
        for hull in hulls.get(link, []):
            pts.append(hull.vertices @ t[:3, :3].T + t[:3, 3])
    return np.vstack(pts)


def check_port_blocks_gripper() -> None:
    v = hand_profile()
    tip_z = v[:, 2].max()
    port_mm = cage.PORT * 1000
    rows, deepest = [], 0.0
    for d_mm in range(0, 121, 5):
        slab = v[v[:, 2] >= tip_z - d_mm / 1000.0]
        if len(slab) < 3:
            continue
        need = max(np.ptp(slab[:, 0]), np.ptp(slab[:, 1])) * 1000
        rows.append((d_mm, need))
        if need <= port_mm:
            deepest = d_mm
    print(f"  square port {port_mm:.0f} mm; closed-hand clearance needed by depth behind the pad tips:")
    for d_mm, need in rows:
        if d_mm <= 60:
            mark = "fits" if need <= port_mm else "BLOCKED"
            print(f"    {d_mm:4d} mm -> {need:6.1f} mm  {mark}")
    margin = cage.BUTTON_DEPTH * 1000 - deepest
    print(f"  deepest the closed hand can enter: {deepest:.0f} mm")
    print(f"  button face is {cage.BUTTON_DEPTH * 1000:.0f} mm in -> short by {margin:.0f} mm")
    record("port blocks the gripper", margin > 30.0, f"{margin:.0f} mm of margin")


def check_demo_ik() -> None:
    plan = task.demo()["left"]
    arm = ArmPlan(list(plan), LEFT_BASE_POS)
    t, dt = 0.0, 1.0 / 20.0
    labels = {}
    while t <= arm.duration + 1e-9:
        _, label = arm.action7(t)
        labels[label] = max(labels.get(label, 0.0), arm.worst_ik_mm)
        t += dt
    print(f"  waypoints: {len(plan)}, duration {arm.duration:.1f} s ({round(arm.duration * 20)} steps at 20 Hz)")
    for label, mm in labels.items():
        print(f"    after '{label}': worst IK so far {mm:.2f} mm")
    print(f"  worst IK over the whole plan: {arm.worst_ik_mm:.2f} mm, {arm.worst_ik_deg:.2f} deg")
    record("demo IK", arm.worst_ik_mm < 5.0 and arm.worst_ik_deg < 5.0,
           f"{arm.worst_ik_mm:.2f} mm / {arm.worst_ik_deg:.2f} deg")


def check_hand_clears_enclosure() -> None:
    """Along the push, the hand must stay in front of the front panel at every height it shares with it.

    The hand widens fast behind the pad tips, so what matters is only the part of it that is low
    enough to meet the guard. Anything above the roof passes over it.
    """
    v = hand_profile()
    tip_z = v[:, 2].max()
    # World height of each hand point when the pad tips are on the port's axis.
    pad_tip_world_z = task.GRIP_Z - (tip_z - 0.134)
    world_z = pad_tip_world_z + (tip_z - v[:, 2])
    roof = cage.guard_roof_z(task.TOP)
    below = v[world_z <= roof + 1e-9]
    overhang = float(below[:, 0].max()) if len(below) else 0.0
    print(f"    pad tips at z={pad_tip_world_z:.4f}, guard roof at z={roof:.4f}")
    print(f"    hand points at or below the roof: {len(below)} of {len(v)}; widest reaches"
          f" {overhang * 1000:.1f} mm ahead of the gripper")
    worst = None
    for name, gx in (("stage", task.STAGE_X), ("press", task.PRESS_X + task.OVERSHOOT)):
        reach = gx + overhang
        gap = (cage.CAGE_X0 - reach) * 1000
        print(f"    {name}: gripper x={gx:.3f}, hand reaches x={reach:.3f}, panel at"
              f" {cage.CAGE_X0:.3f} -> {gap:.0f} mm")
        worst = gap if worst is None else min(worst, gap)
    record("hand clears the guard", worst > 5.0, f"{worst:.0f} mm at the tightest")


def check_tools() -> None:
    """The distractors must be unusable for the stated reasons."""
    rule_reach = cage.RULE_LEN / 2
    needed = cage.BUTTON_FACE_X - 0.10  # from the nearest the gripper can stand
    print(f"  short rule: {cage.RULE_LEN * 1000:.0f} mm long, reaches {rule_reach * 1000:.0f} mm past the grip;")
    print(f"              the button needs {needed * 1000:.0f} mm from the nearest gripper stand (x=0.10)")
    record("short rule is too short", rule_reach < needed, f"{(needed - rule_reach) * 1000:.0f} mm short")
    print(f"  thick baton: {cage.BATON_SIDE * 1000:.0f} mm square vs a {cage.PORT * 1000:.0f} mm port")
    record("thick baton will not fit", cage.BATON_SIDE > cage.PORT,
           f"{(cage.BATON_SIDE - cage.PORT) * 1000:.0f} mm too wide")
    record("rod fits the port", cage.ROD_SIDE < cage.PORT - 0.010,
           f"{(cage.PORT - cage.ROD_SIDE) * 1000:.0f} mm of clearance")


def check_layout() -> None:
    """The rod must not start inside or over the enclosure, and must clear it when drawn back."""
    far = cage.ROD_CX + cage.ROD_LEN / 2
    print(f"  rod at rest spans x {cage.ROD_CX - cage.ROD_LEN / 2:.3f}..{far:.3f} at y={cage.ROD_Y:.3f}")
    print(f"  enclosure spans x {cage.CAGE_X0:.3f}..{cage.CAGE_X1:.3f}, y {cage.CAGE_Y0:.3f}..{cage.CAGE_Y1:.3f}")
    record("rod starts clear of the guard", cage.ROD_Y > cage.CAGE_Y1, "rod is left of the guard in y")
    staged_far = task.STAGE_X + cage.ROD_AHEAD
    print(f"  staged far end x={staged_far:.3f} vs front panel {cage.CAGE_X0:.3f}")
    record("staging keeps the rod out of the port", staged_far < cage.CAGE_X0,
           f"{(cage.CAGE_X0 - staged_far) * 1000:.0f} mm clear")
    record("staging is within reach", task.STAGE_X >= 0.10, f"gripper x={task.STAGE_X:.3f}")
    record("press is within reach", task.PRESS_X + task.OVERSHOOT <= 0.40,
           f"gripper x={task.PRESS_X + task.OVERSHOOT:.3f}")


def main() -> int:
    for title, fn in (
        ("1. the port blocks the gripper", check_port_blocks_gripper),
        ("2. the demo's IK", check_demo_ik),
        ("3. the hand clears the guard", check_hand_clears_enclosure),
        ("4. the three bars", check_tools),
        ("5. the layout", check_layout),
    ):
        print(f"\n{title}")
        fn()
    print("\n--- summary ---")
    for status, name, detail in results:
        print(f"  {status}  {name}: {detail}")
    failed = [r for r in results if r[0] == FAIL]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
