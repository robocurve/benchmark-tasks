"""Kinematic dry run of the scripted demonstration, scored by the real rules and scorer, without the simulator.

The arms follow their joint commands with a first-order lag. A spotter whose middle is between the left hand's pads
when they shut follows the hand; when they open it drops to its rack hole's floor (or onto the table). The plate
follows the right hand once its pads close on the plate's top end: it hangs from the pads (the swing is taken as
done once its bottom edge is above the bar), stands on the tank's floor, and on release falls back until its top
edge touches the back wall, its bottom edge staying put. The lid follows the hand and on release over the tank
rests on the rim. A spotter's tip pressing the plate pushes with 300 N/m of give (the arm's). The rules, the scorer
and the demonstration are the real ones (``tlc.py``, ``grading.py``, ``scripted.py``).

This checks reach (inverse kinematics on the real arm model), joint limits, the control flow and the step count. It
says nothing about contact, friction, the plate's swing or its fall: it does not replace a run in the simulator.

Run:  python tools/dryrun_demo.py [--variant full|plate_only|press_hard|no_dip] [--verbose]
Needs NumPy and yam_arena (its kinematics need no Isaac Sim).
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

GIVE = 300.                    # N/m: a spotter pressed on the plate (the arm gives about 3 mm/N)
HANG = (0., -math.sqrt(.5), 0., math.sqrt(.5))      # the plate hanging: its top edge up, its face toward -x


def quat_y(angle):
    return (0., math.sin(angle / 2), 0., math.cos(angle / 2))


def dry_run(*, variant="full", arm_lag_s=.12, max_steps=None, verbose=False):
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN, READY

    from rdb_tlc_spotting import fixed_scene as F
    from rdb_tlc_spotting import grading, tlc
    from rdb_tlc_spotting import layout as L
    from rdb_tlc_spotting.scripted import PAD_HALF_WIDTH, TIP, TLCReference

    reference = TLCReference(variant=variant)
    actions = reference.sequence()
    dt = 1. / F.CONTROL_HZ
    blend = 1. - math.exp(-dt / arm_lag_s)
    span = 2 * abs(float(FINGER_OPEN))
    joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
    bases = {arm: np.asarray(F.BASES[arm], dtype=float) for arm in F.ARMS}
    poses = {n: (np.asarray(p, dtype=float), np.asarray(q, dtype=float)) for n, (p, q) in F.START.items()}
    held = {}                                    # body name -> (arm, offset from the tool point)
    grade = grading.TLCGrade()
    result, forces = None, None
    report = {"worst_limit_margin_deg": 180., "events": []}
    tick = 0
    for tick in range(1, (max_steps or F.MAX_STEPS) + 1):
        state = {"poses": {n: (p.copy(), q.copy()) for n, (p, q) in poses.items()}, "forces": forces, "grade": result}
        reference.observe(joints.copy(), state)
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        for i in range(2):
            q = command[7 * i:7 * i + 6]
            if np.any(q < ik.LOWER - 1e-6) or np.any(q > ik.UPPER + 1e-6):
                raise RuntimeError(f"{F.ARMS[i]} arm commanded past a joint limit at step {tick}: {q}")
            margin = float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q).min()))
            if margin < report["worst_limit_margin_deg"]:
                report["worst_limit_margin_deg"] = margin
                report["worst_limit_at"] = (tick, F.ARMS[i], reference.phases[-1][1] if reference.phases else "",
                                            int(np.argmin(np.minimum(q - ik.LOWER, ik.UPPER - q))) + 1)
        joints = joints + blend * (command - joints)
        tools = {arm: ik.tcp_pose(joints[7 * i:7 * i + 6], bases[arm])[0] for i, arm in enumerate(F.ARMS)}
        width = {arm: float(joints[7 * i + 6]) * span for i, arm in enumerate(F.ARMS)}
        # pick up and let go
        for name in F.SPOTTERS:
            p, q = poses[name]
            if name in held:
                arm, offset = held[name]
                if width[arm] > .004:
                    del held[name]
                    tip = np.asarray(tlc.spotter_tip(poses[name]))
                    floor = next((L.hole_point(k)[2] for k in range(3)
                                  if math.hypot(tip[0] - L.RACK_X, tip[1] - L.HOLE_Y[k]) < L.HOLE / 2), 0.)
                    poses[name] = (p + (0., 0., floor - tip[2]), q)
                    report["events"].append((tick, f"let go of {name}, tip at {np.round(1000 * (tip[2] - floor + floor), 1)} mm"))
                else:
                    poses[name] = (tools[arm] + offset, q)
            elif width["left"] < .004 and np.hypot(*(tools["left"] - p)[:2]) < .003 and \
                    abs((tools["left"] - p)[2] - (TIP - L.CAP_LEN / 2 + .050 - .0)) < .03:
                held[name] = ("left", p - tools["left"])
                report["events"].append((tick, f"picked {name}"))
        p, q = poses["plate"]
        tank = np.asarray(poses["tank"][0])
        ix, iy = L.TANK_IN
        if "plate" in held:
            if width["right"] > L.PLATE_WIDE + .0003:
                del held["plate"]
                bottom = np.asarray(tlc.to_world((p, q), (-L.PLATE_LEN / 2, 0., 0.)))
                rel = bottom - tank
                if abs(rel[0]) < ix / 2 and abs(rel[1]) < iy / 2:
                    d = ix / 2 - rel[0]
                    lean = math.asin(min(1., d / L.PLATE_LEN))
                    q2 = np.asarray(tlc_quat_mul(quat_y(lean), HANG))
                    centre = bottom + np.asarray(tlc.to_world(((0., 0., 0.), q2), (L.PLATE_LEN / 2, 0., 0.)))
                    poses["plate"] = (centre, q2)
                    report["events"].append((tick, f"plate falls back to {math.degrees(lean):.1f} degrees"))
            else:
                pivot = tools["right"] + (-(PAD_HALF_WIDTH - .0035), 0., -(TIP - .0028))
                centre = pivot - np.asarray(tlc.to_world(((0., 0., 0.), HANG), (L.PLATE_LEN / 2 - .0035, 0., 0.)))
                rel = centre - tank
                floor = max(L.BAR[2] if abs(centre[0] - L.PLATE_X) < L.BAR[0] / 2 and abs(centre[1] - L.PLATE_Y) < .02 else 0.,
                            L.TANK_FLOOR if abs(rel[0]) < ix / 2 and abs(rel[1]) < iy / 2 else 0.)
                low = centre[2] - L.PLATE_LEN / 2
                if low < floor:
                    centre = centre + (0., 0., floor - low)
                poses["plate"] = (centre, np.asarray(HANG))
        elif width["right"] < L.PLATE_WIDE + .0005 and abs(tools["right"][1] - p[1]) < .003 and \
                abs(tools["right"][0] - (p[0] + L.PLATE_LEN / 2 + PAD_HALF_WIDTH - .007)) < .004 and tools["right"][2] < .04:
            held["plate"] = ("right", None)
            report["events"].append((tick, "plate pinched"))
        p, q = poses["lid"]
        if "lid" in held:
            arm, offset = held["lid"]
            if width[arm] > L.LID[0] + .002:
                del held["lid"]
                rel = p - tank
                if abs(rel[0]) < .01 and abs(rel[1]) < .01:
                    poses["lid"] = (np.array((tank[0], tank[1], tank[2] + L.TANK[2] + L.LID[2] / 2)), q)
                else:
                    poses["lid"] = (np.array((p[0], p[1], L.LID[2] / 2)), q)
                report["events"].append((tick, "let go of the lid"))
            else:
                poses["lid"] = (tools[arm] + offset, q)
        elif L.LID[0] - .01 < width["right"] < L.LID[0] and np.hypot(*(tools["right"] - p)[:2]) < .003 \
                and tools["right"][2] < .03:
            held["lid"] = ("right", p - tools["right"])
            report["events"].append((tick, "lid pinched"))
        # a spotter pressing the plate
        plate_force = [0., 0., 0.]
        for k, name in enumerate(F.SPOTTERS):
            tip = tlc.to_local(poses["plate"], tlc.spotter_tip(poses[name]))
            inside = abs(tip[0]) <= L.PLATE_LEN / 2 and abs(tip[1]) <= L.PLATE_WIDE / 2
            if inside and tip[2] < L.PLATE_T / 2 and name in held:
                pen = L.PLATE_T / 2 - tip[2]
                plate_force[k] = GIVE * pen
                p, q = poses[name]                     # the arm gives: the tip stays on the silica
                poses[name] = (np.asarray(p) + (0., 0., pen), q)
        forces = {"plate": plate_force, "other": [0., 0., 0.]}
        pads = [6. if name in held else 0. for name in F.SPOTTERS]      # a closed hand squeezes about 6 N
        result = grade.update(poses, plate_force, [0., 0., 0.], [tools["left"], tools["right"]], pads)
        if verbose and tick % 40 == 0:
            print(tick, result["score"], result["points"], reference.phases[-1][1] if reference.phases else "", flush=True)
        if grade.finished:
            break
        if reference.done and tick > reference.phases[-1][0] + 2 * F.CONTROL_HZ:
            break
    report.update(score=result["score"], points=result["points"], steps=tick, grade=result,
                  worst_ik_mm=round(1000 * reference.worst_ik, 3), worst_ik_at=reference.worst_ik_at, phases=reference.phases, log=reference.log)
    return report


def tlc_quat_mul(a, b):
    """Hamilton product of (x, y, z, w) quaternions: a after b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", default="full")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    report = dry_run(variant=args.variant, verbose=args.verbose)
    log, phases, events = report.pop("log"), report.pop("phases"), report.pop("events")
    grade = report.pop("grade")
    print(args.variant, "->", report)
    print("lanes:", grade["lanes"])
    print("plate:", grade["plate"], "lid:", grade["lid"], "racked:", grade["racked"], "void:", grade["void"])
    if args.verbose:
        print("phases:", phases)
        for event in events:
            print("  ", event)
        for event in log["events"]:
            print("  ", event)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
