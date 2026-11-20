#!/usr/bin/env python3
"""A check, not the task: the push the right arm gives on the garlic press's upper handle (scene ``push_gauge.py``).

    scripts/rta simpy <id> mech/tasks/garlic_press/push_check.py /workspace/robocurve/outputs/<id> [deg,deg]

For each angle the bar stands open at (default 25 and 3 degrees: where the plunger meets a clove, and near the end of
the stroke): first known torques put on the bar directly (the spring's rate as the scene gives it), then the right
hand, pointing down with its fingers across the bar, pinches it 25 mm from its end and presses straight down with
30, 45, 60 and 75 N asked for (each ramped over 1 s, held 1.5 s). The bar's turn times the rate read is the torque the
hand gives; over the bar's horizontal lever it is the downward push. Writes <out dir>/push_<deg>.json.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

TORQUES = (3.0, 6.0, 9.0)  # N m, closing: 24 to 71 N at the hand's 126 mm
ASKED = (30.0, 45.0, 60.0, 75.0)
DOWN = (60.0, 45.0, 30.0)  # then coming down
OVER = 0.06


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def known(embodiment, m, deg: float) -> dict:
    from inspect_robots import Scene

    mk = m.mk
    hz = float(embodiment.info.control_hz)
    m.TEST_DEG[0] = deg
    embodiment.reset(Scene(id=f"known {deg}", instruction=m.INSTRUCTION), seed=0)
    for _ in range(int(hz)):
        embodiment.step(_hold())
    base = mk.joint("lever")
    rows = []
    for torque in TORQUES:
        seen = []
        for _ in range(int(1.5 * hz)):
            mk.push("lever", torque=(torque, 0.0, 0.0))  # about +x: closes the bar (its hinge opens about -x)
            embodiment.step(_hold())
            seen.append(mk.joint("lever"))
        turn = base - float(np.median(seen[-10:]))
        rows.append({"torque (N m)": torque, "turn (deg)": round(math.degrees(turn), 3), "rate (N m/rad)": round(torque / turn, 2) if turn > 0 else None})
    mk.push("lever")
    out = {"bar at rest (deg)": round(math.degrees(base), 3), "known torques": rows}
    print(f"KNOWN {deg:g}: {json.dumps(out)}", flush=True)
    return out


def pressed(embodiment, m, deg: float, rate: float, out: Path) -> dict:
    import demo_kit as kit

    mk = m.mk
    m.TEST_DEG[0] = deg
    phi = math.radians(deg)
    at = m.grasp_point(phi)
    rot = kit.down(math.pi / 2)  # the fingers close along x: across the bar
    Move = kit.Move
    moves = [Move(3.0, pos=at + np.array([0.0, 0.0, OVER]), rot=rot, grip=0.6, label="over the bar"), Move(2.0, pos=at, label="down"),
             Move(1.0, grip=0.0, label="close"), Move(0.5, label="closed")]  # fmt: skip
    for force in ASKED:
        moves += [Move(1.0, push=(0.0, 0.0, -force), label=f"press {force:g} N"), Move(1.5, label=f"hold {force:g} N")]
    for force in DOWN:  # the same levels coming down: joints held by their friction read the other way round
        moves += [Move(1.0, push=(0.0, 0.0, -force), label=f"ease to {force:g} N"), Move(1.5, label=f"hold down {force:g} N")]
    moves += [Move(1.0, push=(0.0, 0.0, 0.0), label="ease"), Move(0.8, grip=0.6, label="open"), Move(1.2, pos=at + np.array([0.0, 0.0, OVER]), label="up")]

    def extra() -> dict:
        return {"bar (deg)": round(math.degrees(mk.joint("lever")), 4), "hand": [round(float(v), 4) for v in mk.gripper("right")[0]]}

    result = kit.run(embodiment, {"right": moves}, out / f"push_{deg:g}", seed=0, extra=extra, limit_s=60.0, tail_s=0.5)
    rows = result["rows"]
    rest = float(np.median([row["extra"]["bar (deg)"] for row in rows if row["right"] == "closed"]))
    found = {"bar with the hand closed on it (deg)": round(rest, 3), "rate used (N m/rad)": rate, "asked (N), torque (N m), push (N), arm's own estimate (N, x y z), bar (deg)": []}
    for label, force in [(f"hold {f:g} N", f) for f in ASKED] + [(f"hold down {f:g} N", f) for f in DOWN]:
        held = [row for row in rows if row["right"] == label][-10:]
        bar = float(np.median([row["extra"]["bar (deg)"] for row in held]))
        torque = rate * math.radians(rest - bar)
        u, _ = m.opened(*m.PUSH_UW, math.radians(bar))
        lever = u - m.HINGE_UW[0]
        estimate = np.median(np.array([row["force"]["right"] for row in held]), axis=0)
        found["asked (N), torque (N m), push (N), arm's own estimate (N, x y z), bar (deg)"].append([("down " if "down" in label else "") + f"{force:g}", round(torque, 3), round(torque / lever, 2), [round(float(v), 2) for v in estimate], round(bar, 3)])
    found["worst IK"] = result["worst_ik"]
    found["grasp point (m)"] = [round(float(v), 4) for v in at]
    print(f"PUSH {deg:g}: {json.dumps(found)}", flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    angles = [float(v) for v in sys.argv[2].split(",")] if len(sys.argv) > 2 else [25.0, 3.0]
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "push_gauge.py")
    m = embodiment.task.module
    failed = 0
    for deg in angles:
        try:
            k = known(embodiment, m, deg)
            rates = [r["rate (N m/rad)"] for r in k["known torques"] if r["rate (N m/rad)"]]
            rate = float(np.mean(rates)) if rates else m.K
            p = pressed(embodiment, m, deg, rate, out)
            (out / f"push_{deg:g}.json").write_text(json.dumps({"known": k, "pressed": p}, indent=1))
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"ANGLE {deg:g} FAILED: {error!r}", flush=True)
            failed += 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
