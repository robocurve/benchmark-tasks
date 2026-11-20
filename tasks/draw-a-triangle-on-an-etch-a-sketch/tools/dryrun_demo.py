"""Dry run of the scripted demonstration without the simulator, scored by the real scorer.

The arms follow their joint commands with a first-order lag (no contact physics). A knob turns with its gripper
(a fraction ``follow`` of the gripper's roll, 1 = no slip) while the fingers are closed on it with the TCP within
3 mm of the knob's axis and at grip height; the stylus model in ``stylus.py`` (string slack, 35.5 mm per turn)
turns knob angles into the drawn line, and ``grading.py`` scores every tick. The demonstration gets the state it
gets in the simulator (stylus point, knob slack, knob positions). This checks reach, joint limits, the stroke
and regrip logic, the control flow and the time it takes; it says nothing about finger contact.

Run:  python tools/dryrun_demo.py [--clearance <folder with clearance.py>]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def dry_run(*, arm_lag_s=.12, follow=1., backlash_deg=20., knob_shift_mm=(0., 0.), clearance=None,
            max_steps=None, verbose=False, path_mm=None):
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN, READY

    from rdb_etch_triangle import fixed_scene as fixture
    from rdb_etch_triangle import layout
    from rdb_etch_triangle.grading import TriangleGrade
    from rdb_etch_triangle.scripted import GRIP_Z, EtchReference, SIDE
    from rdb_etch_triangle.stylus import Stylus

    reference = EtchReference()
    actions = reference.sequence()
    grade = TriangleGrade(start_xy=fixture.TOY_XY, start_yaw=fixture.TOY_YAW)
    stylus = Stylus(backlash=np.radians(backlash_deg))
    dt = 1 / fixture.CONTROL_HZ
    blend = 1 - np.exp(-dt / arm_lag_s)
    joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
    knobs = np.zeros(2)
    tops = {arm: fixture.knob_station(fixture.KNOB_OF[arm]) + (*np.asarray(knob_shift_mm) / 1000, 0.)
            for arm in fixture.ARMS}
    previous_j6 = [None, None]
    report = {"worst_limit_margin_deg": 180., "min_tcp_gap_mm": np.inf, "min_pad_tip_above_frame_mm": np.inf,
              "max_axis_error_mm": 0., "min_frame_gap_mm": np.inf}
    result = None
    steps = max_steps or fixture.MAX_STEPS
    knob_d = layout.KNOB_D
    for tick in range(1, steps + 1):
        engaged = [(stylus.engaged(i, +1), stylus.engaged(i, -1)) for i in range(2)]
        state = {"stylus": stylus.point.copy(), "knobs": knobs.copy(), "engaged": engaged,
                 "knob_tops": [tops["left"], tops["right"]]}
        measured = joints.copy()
        reference.observe(measured, state)
        try:
            command = np.asarray(next(actions).actions[0].data, dtype=float)
        except StopIteration:
            break
        for i in range(2):
            q = command[7 * i:7 * i + 6]
            if np.any(q < ik.LOWER - 1e-6) or np.any(q > ik.UPPER + 1e-6):
                raise RuntimeError(f"{fixture.ARMS[i]} arm commanded past a joint limit at tick {tick}: {q}")
            report["worst_limit_margin_deg"] = min(report["worst_limit_margin_deg"],
                                                   float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
        joints = joints + blend * (command - joints)
        touching = False
        tcps = []
        for i, arm in enumerate(fixture.ARMS):
            # the fingers stop on the knob when closed round it
            gap = joints[7 * i + 6] * 2 * abs(FINGER_OPEN)
            point, rotation = ik.tcp_pose(joints[7 * i:7 * i + 6], fixture.BASES[arm])
            tcps.append(point)
            axis_error = float(np.linalg.norm(point[:2] - tops[arm][:2]))
            at_height = abs(point[2] - GRIP_Z) < .003
            held = gap <= knob_d + .001 and axis_error < .003 and at_height
            if held:
                joints[7 * i + 6] = max(joints[7 * i + 6], (knob_d - .0005) / (2 * abs(FINGER_OPEN)))
                report["max_axis_error_mm"] = max(report["max_axis_error_mm"], 1000 * axis_error)
                touching = True
                j6 = joints[7 * i + 5]
                if previous_j6[i] is not None:
                    knobs[SIDE[arm]] -= follow * (j6 - previous_j6[i])
                previous_j6[i] = j6
            else:
                previous_j6[i] = None
                if gap < knob_d + .004 and axis_error < .02 and point[2] < layout.KNOB_TOP + .02:
                    touching = True
            report["min_pad_tip_above_frame_mm"] = min(report["min_pad_tip_above_frame_mm"],
                                                       1000 * (point[2] - .013 - layout.BODY_H))
            if clearance is not None and tick % 3 == 0:
                cloud = clearance.arm_cloud(joints[7 * i:7 * i + 6], joints[7 * i + 6], fixture.BASES[arm],
                                            links=("gripper", "tip_left", "tip_right", "link5", "link4", "link3"))
                local = (cloud - (*fixture.TOY_XY, 0.)) @ fixture.TOY_ROTATION
                inside = (np.abs(local[:, 0]) < layout.BODY[0] / 2) & (np.abs(local[:, 1]) < layout.BODY[1] / 2)
                # height above the frame face (the knob itself is excluded: within 21 mm of a knob axis)
                near_knob = np.zeros(len(local), dtype=bool)
                for kx, ky in fixture.KNOB_TOY.values():
                    near_knob |= np.hypot(local[:, 0] - kx, local[:, 1] - ky) < .021
                above = local[inside & ~near_knob, 2] - layout.RIDGE_H
                if len(above):
                    report["min_frame_gap_mm"] = min(report["min_frame_gap_mm"], 1000 * float(above.min()))
        report["min_tcp_gap_mm"] = min(report["min_tcp_gap_mm"], 1000 * float(np.linalg.norm(tcps[0] - tcps[1])))
        stylus.update(knobs)
        if path_mm is not None:
            path_mm.append(1000 * stylus.point.copy())
        result = grade.update(stylus.point, (*fixture.TOY_XY, 0.), fixture.TOY_ROTATION, touching)
        if verbose and tick % 100 == 0:
            print(tick, result["score"], result["tier"], result["covered_mm"], result["stylus_mm"],
                  result["longest_step_mm"], reference.phases[-1][1])
        if grade.finished:
            break
    report.update(score=result["score"], tier=result["tier"], steps=tick, covered=result["covered_mm"],
                  longest_step_mm=result["longest_step_mm"], stray=result["stray"], stylus_mm=result["stylus_mm"],
                  regrips=reference.regrips, worst_ik_mm=round(1000 * reference.worst_ik, 3),
                  phases=reference.phases, knob_turns=[round(float(k / (2 * np.pi)), 2) for k in knobs])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clearance", help="folder holding clearance.py (arm collision hulls), optional")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs"):
        sys.path.insert(0, str(path))
    clearance = None
    if args.clearance:
        sys.path.insert(0, args.clearance)
        import clearance
    for name, kw in (("nominal", {}),
                     ("knobs slip 8 %, slow arm, 25 deg slack", dict(follow=.92, arm_lag_s=.2, backlash_deg=25.)),
                     ("knobs 2 mm off their nominal place, 15 deg slack", dict(knob_shift_mm=(2., -2.), backlash_deg=15.))):
        report = dry_run(clearance=clearance if name == "nominal" else None, verbose=args.verbose, **kw)
        print(name, "->", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
