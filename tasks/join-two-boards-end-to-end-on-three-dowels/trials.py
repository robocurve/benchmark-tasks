#!/usr/bin/env python3
"""Checks of the dowel task in the simulator, no arm.

    scripts/rta simpy <id> mech/tasks/dowel_join/trials.py /workspace/robocurve/outputs/<id> [rest,checks] [bore=8.3]

``rest``    3 s with nobody touching anything: the first pictures (top camera and both wrist cameras, as rendered and
            at the size a policy gets), and how far anything moves.
``checks``  the three checks asked for before any script, each from the boards put 20.5 mm apart (the dowels' tips
            half a millimetre short of the walnut board's end: a steady 2 N on a free board of 125 g speeds it up by
            13 m/s^2, so from farther apart the boards would be thrown at each other) and pushed together with 2 N
            each, at their middles, along the walnut board, for 4 s, then let be for 1 s: (a) in line and square; (b) the cherry board 1.5 mm out of
            line; (c) the cherry board turned 2 degrees about the middle of its end. Then the same three once more with
            8 N each, to see what a harder push does.

``bore=<mm>`` builds the sockets that wide (the task file's own figure, 8.2 mm, otherwise). Every trial writes <out dir>/<trial>.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

START_GAP = 0.0205  # m between the boards' ends when a check begins: the dowels' tips half a millimetre short of the walnut end
CASES = {"a: in line and square": (0.0, 0.0), "b: 1.5 mm out of line": (0.0015, 0.0), "c: turned 2 degrees": (0.0, math.radians(2.0))}


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _save(out: Path, name: str, frame: np.ndarray) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    small = kit.policy_picture(frame)
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / f"{name}_policy_x3.png"), cv2.cvtColor(cv2.resize(small, (small.shape[1] * 3, small.shape[0] * 3), interpolation=cv2.INTER_NEAREST), cv2.COLOR_RGB2BGR))


def reading(m) -> dict:
    s = m.seam()
    return {"gaps (mm)": {k: round(1000 * v, 3) for k, v in s["gaps"].items()}, "across (mm)": round(1000 * s["across"], 3), "turned (deg)": round(math.degrees(s["turned"]), 3),
            "height (mm)": round(1000 * s["height"], 3), "tilts (deg)": {k: round(math.degrees(v), 2) for k, v in s["tilts"].items()}, "level": m.level()}  # fmt: skip


def rest(embodiment, out: Path) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=0)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    for _ in range(int(round(3.0 * hz))):
        embodiment.step(_hold())
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - mk.built_pose(name)[0])), 3) for name in m.NAMES}
    found = {"bore (mm)": round(1000 * m.BORE, 2), "after 3 s": reading(m), "boards from where they were built after 3 s (mm)": moved, "score": m.score(None)}
    print(f"REST {json.dumps(found)}", flush=True)
    return found


def checks(embodiment, out: Path) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    walnut_at = np.array([*m.WALNUT_XY, m.TOP + m.BOARD_T / 2])
    face = walnut_at + np.array([0.0, -m.BOARD_L / 2, 0.0])
    found: dict = {"bore (mm)": round(1000 * m.BORE, 2)}
    for newtons in (2.0, 8.0):
        for case, (across, turned) in CASES.items():
            embodiment.reset(Scene(id=f"{case} {newtons:g} N", instruction=m.INSTRUCTION), seed=0)
            embodiment.step(_hold())
            end = face + np.array([across, -START_GAP, 0.0])
            cherry_at = end - np.array([-math.sin(turned), math.cos(turned), 0.0]) * m.BOARD_L / 2
            mk.write_pose("cherry", cherry_at, mk.yaw_quat(turned))
            for _ in range(int(hz / 2)):
                embodiment.step(_hold())
            before = reading(m)
            rows, narrowest = [], None
            for k in range(int(round(4.0 * hz))):
                mk.push("cherry", force=(0.0, newtons, 0.0))
                mk.push("walnut", force=(0.0, -newtons, 0.0))
                embodiment.step(_hold())
                now = reading(m)
                widest = max(now["gaps (mm)"].values())
                narrowest = widest if narrowest is None else min(narrowest, widest)
                if k % int(hz / 2) == int(hz / 2) - 1:
                    rows.append([round((k + 1) / hz, 2), now["gaps (mm)"], now["across (mm)"], now["turned (deg)"]])
            pushed = reading(m)
            mk.push("cherry")
            mk.push("walnut")
            for _ in range(int(hz)):
                embodiment.step(_hold())
            after = reading(m)
            moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0][:2] - mk.built_pose(name)[0][:2])), 1) for name in m.NAMES}
            found[f"{case}, {newtons:g} N each"] = {"put": before, "after 4 s of pushing": pushed, "a second after": after, "narrowest seam while pushed (mm)": narrowest,
                                                    "closed (both gaps 0.5 mm or less)": max(pushed["gaps (mm)"].values()) <= 0.5,
                                                    "boards' middles from where they were built at the end (mm)": moved, "every 0.5 s: t, gaps, across, turned": rows}  # fmt: skip
            print(f"CHECK {case}, {newtons:g} N: {json.dumps(found[f'{case}, {newtons:g} N each'])}", flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    known = {"rest": rest, "checks": checks}
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    for arg in sys.argv[3:]:
        if arg.startswith("bore="):
            os.environ["DOWEL_BORE_MM"] = arg.split("=", 1)[1]  # read by the task file when it is loaded
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    failed = 0
    for name in trials:
        try:
            result = known[name](embodiment, out)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"TRIAL {name} FAILED: {error!r}", flush=True)
            failed += 1
            continue
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
