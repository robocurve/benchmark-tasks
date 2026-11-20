#!/usr/bin/env python3
"""Checks of the cupcake task in the simulator.

    scripts/rta simpy <id> mech/tasks/cupcake_candles/trials.py /workspace/robocurve/outputs/<id> [hold]

``hold``  check (a), on the trial scene ``hold_trial.py``: candles 8, 12 and 15 mm deep in cupcakes, held as the
          spec's table has it and with a hundredth of that, nudged at 1 s and tilted from 3 s on at 7.5 degrees a
          second to 60; two more candles pushed straight down with 0.9 and then 1.1 of their resistance along their
          length (12 mm deep in the cake, 1 mm deep in the icing). Writes ``hold.json`` and a run folder with videos.
``fall``  on the trial scene ``fall_trial.py``: candles 8, 9 and 15 mm deep in cupcakes that lean 1 degree at 0.5 s,
          the task's 10 g stand-in (its weight over 1 g carried) beside a real 1 g candle: when each leans 2, 10 and
          25 degrees, when it tears out. Writes ``fall.json``.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def hold(out: Path, embodiment) -> dict:
    import demo_kit as kit

    m = embodiment.task.module
    result = kit.run(embodiment, {}, out / "hold", seed=0, tail_s=12.0, limit_s=13.0, extra=m.readings)
    rows = result["rows"]
    found = {"events": m.report(), "by candle": {}}
    for name in m.cd.CANDLES:
        series = [(row["t"], row["extra"]["tilt_deg"], row["extra"][name]) for row in rows if "extra" in row]
        first_out = next(((t, tilt) for t, tilt, r in series if r["in"] is None), None)
        def at(time):
            return min(series, key=lambda s: abs(s[0] - time))[2]
        found["by candle"][name] = {
            "free at (s, tilt deg)": first_out,
            "at 0.9 s": at(0.9), "at 2.9 s": at(2.9), "at 4.9 s": at(4.9), "at 7 s": at(7.0), "end": series[-1][2],
            "most lean from vertical while in (deg)": max((r["lean_vertical_deg"] for _, _, r in series if r["in"] is not None), default=None),
        }
    found["worst IK (mm, deg)"] = result.get("worst_ik")
    print(f"HOLD {json.dumps(found)}", flush=True)
    return found


def fall(out: Path, embodiment) -> dict:
    import demo_kit as kit

    m = embodiment.task.module
    result = kit.run(embodiment, {}, out / "fall", seed=0, tail_s=6.0, limit_s=7.0, extra=m.readings)
    rows = [row for row in result["rows"] if "extra" in row]
    found = {"events": m.report(), "by candle": {}}
    for name, (row, depth) in m.CANDLE_OF.items():
        series = [(row_["t"], row_["extra"][name]) for row_ in rows]

        def first(test):
            return next((t for t, r in series if test(r)), None)

        found["by candle"][name] = {
            "candle": row, "depth_mm": round(1000 * depth, 1), "mass_g": round(1000 * m.cd.CANDLES[name].mass, 1),
            "lean at the first step (deg)": series[0][1]["lean_vertical_deg"],
            "lean at the tilt (deg)": next(r["lean_vertical_deg"] for t, r in series if t >= m.TILT_AT),
            **{f"leans {a} deg at (s)": first(lambda r, a=a: r["lean_vertical_deg"] >= a) for a in (2, 10, 25)},
            "torn out at (s)": first(lambda r: r["in"] is None),
            "most lean while in (deg)": max((r["lean_vertical_deg"] for _, r in series if r["in"] is not None), default=None),
            "end": series[-1][1],
        }
    found["worst IK (mm, deg)"] = result.get("worst_ik")
    print(f"FALL {json.dumps(found)}", flush=True)
    return found


def _hold_ready():
    import numpy as np
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _save(out: Path, name: str, frame) -> None:
    import cv2
    import numpy as np

    import demo_kit as kit

    frame = np.asarray(frame)
    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    small = kit.policy_picture(frame)
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
    big = cv2.resize(small, (small.shape[1] * 3, small.shape[0] * 3), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(str(out / f"{name}_policy_x3.png"), cv2.cvtColor(big, cv2.COLOR_RGB2BGR))


def rest(out: Path, embodiment) -> dict:
    """The task's first pictures (seed 0) and 3 s with nobody touching anything."""
    from inspect_robots import Scene

    m = embodiment.task.module
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=0)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, observation.images[camera])
    hz = float(embodiment.info.control_hz)
    rows = []
    for _ in range(int(round(3.0 * hz))):
        embodiment.step(_hold_ready())
        rows.append(m.readings())
    found = {"first": rows[0], "after 3 s": rows[-1], "report": m.report(), "score": m.score(None)}
    print(f"REST {json.dumps(found, default=str)}", flush=True)
    return found


def squeeze(out: Path, embodiment) -> dict:
    """Check (b) and the designer's added check: the left hand comes from the near side, 15 degrees under level, and
    holds the middle cupcake at its rim (grasp point 26.9 mm over the board); it closes to touch and 1.5 mm past it.
    Then the cupcake is pushed straight down from above (a load put on directly): 3, 6 and 10 N, 4 mm off its middle
    towards the far side, then 3 and 10 N 10 mm off. Then the hand closes on in steps of half a millimetre until the
    cupcake gives for good. Read every step: the pads' gap, the squeeze reading, the gripper's own force, the
    cupcake's place and how far it turned."""
    import math

    import numpy as np

    import demo_kit as kit
    import hands

    m = embodiment.task.module
    mk = m.mk
    cake = "cake1"
    rot = hands.side_grip(0.0, 15.0) @ np.diag([-1.0, -1.0, 1.0])  # the half turn the left arm reaches (hands.py)
    approach = rot[:, 2]
    grasp = np.array([m.ROW_X, m.CAKE_YS[1], m.BOARD_TOP + 0.0269])
    Move = kit.Move

    def grip(gap_mm: float) -> float:
        return gap_mm / 1000.0 / m.PAD_GAP_OPEN

    moves = [Move(3.0, pos=grasp - 0.08 * approach + np.array([0.0, 0.0, 0.04]), rot=rot, grip=grip(75), label="towards the cupcake"),
             Move(2.0, pos=grasp - 0.03 * approach, label="in front of it"), Move(2.0, pos=grasp, label="pads round the rim"),
             Move(1.0, label="settle"), Move(1.5, grip=grip(60.0), label="gap 60.0"), Move(1.5, grip=grip(58.5), label="gap 58.5"),
             Move(1.0, label="hold 58.5")]  # fmt: skip
    pushes = [(3.0, 0.004), (6.0, 0.004), (10.0, 0.004), (3.0, 0.010), (10.0, 0.010)]
    for force, off in pushes:
        moves.append(Move(2.0, label=f"push {force:g} N {1000 * off:g} mm off"))
    moves.append(Move(1.0, label="no push"))
    for gap in (58.0, 57.5, 57.0, 56.5, 56.0, 55.5, 55.0, 54.5, 54.0):
        moves.append(Move(1.5, grip=grip(gap), label=f"gap {gap:.1f}"))
    moves.append(Move(1.5, grip=grip(75), label="open"))
    moves.append(Move(2.0, pos=grasp - 0.08 * approach + np.array([0.0, 0.0, 0.04]), label="away"))
    plan = {"left": moves}
    STATE_PUSH = {}

    def director(t, arms, pictures):
        label = arms["left"].target(t)[3]
        load = next(((f, o) for f, o in pushes if label == f"push {f:g} N {1000 * o:g} mm off"), None)
        if mk.MECH.environment is not None and cake not in m.STATE.get("squashed", {}):
            if load is None:
                mk.push(cake)
            else:
                f, o = load
                mk.push(cake, force=(0.0, 0.0, -f), torque=tuple(np.cross((o, 0.0, 0.0), (0.0, 0.0, -f))))
        STATE_PUSH["now"] = load

    def extra() -> dict:
        r = m.readings()
        return {"push": STATE_PUSH.get("now"), "squeeze_mm": r["squeeze_mm"][cake], "pinch_N": r["pinch_N"].get("left"),
                "gap_mm": r["gap_mm"]["left"], cake: r[cake], "squashed": dict(m.STATE.get("squashed", {}))}

    result = kit.run(embodiment, plan, out / "squeeze", seed=0, director=director, extra=extra, limit_s=60.0, tail_s=0.5)
    rows = [row for row in result["rows"] if "extra" in row]
    by_label: dict = {}
    for row in rows:
        by_label.setdefault(row["left"], []).append(row)
    found = {"steps (label: last row's gap, squeeze, pinch force, cupcake tilt about y and x, place)": {}}
    for label, rs in by_label.items():
        e = rs[-1]["extra"]
        found["steps (label: last row's gap, squeeze, pinch force, cupcake tilt about y and x, place)"][label] = [
            e["gap_mm"], e["squeeze_mm"], e["pinch_N"], e[cake]["about_y_deg"], e[cake]["about_x_deg"], e[cake]["at"]]
    found["squashed"] = m.STATE.get("squashed")
    found["worst IK (mm, deg)"] = result.get("worst_ik")
    found["report"] = m.report()
    print(f"SQUEEZE {json.dumps(found, default=str)}", flush=True)
    return found


def view(out: Path, embodiment) -> dict:
    """Check (d): three candles standing 15 mm deep in the cupcakes (upright, 10 degrees sideways, 10 degrees away),
    looked at by the right wrist camera from 12 cm, the hand pointing down; and the start pictures of that scene.
    Saves each look at full size, at the policy's size and that enlarged three times."""
    import numpy as np

    import demo_kit as kit
    from yam_arena import station
    from yam_arena.urdf import quat_xyzw_to_matrix

    m = embodiment.task.module
    assert m.VIEW, "the view check needs CUPCAKE_VIEW=1"
    cam = station.wrist_camera("right")
    offset, turn = np.array(cam.pos) - np.array([0.0, 0.0, 0.134]), quat_xyzw_to_matrix(np.array(cam.rot_xyzw))
    look_g = -turn[:, 2]
    rot = np.column_stack([(1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)])  # pointing down, closing along x
    Move = kit.Move
    moves, looks = [], []
    for k, cake in enumerate(m.CAKES):
        x, y = m.ROW_X, m.CAKE_YS[k]
        target = np.array([x, y, m.BOARD_TOP + m.CAKE_H + 0.025])  # the middle of what shows of the candle
        tcp = target - 0.12 * (rot @ look_g) - rot @ offset
        moves += [Move(2.5, pos=tcp, rot=rot, grip=0.3, label=f"to {cake}"), Move(1.0, label=f"look {cake}")]
        looks.append(f"look {cake}")
    saved = {}

    def director(t, arms, pictures):
        label = arms["right"].target(t)[3]
        if pictures is not None and label in looks:
            saved[label] = pictures["right_cam"]

    result = kit.run(embodiment, {"right": moves}, out / "view", seed=0, director=director, limit_s=20.0, tail_s=0.5)
    for label, frame in saved.items():
        _save(out, label.replace(" ", "_"), frame)
    found = {"saved": sorted(saved), "worst IK (mm, deg)": result.get("worst_ik"),
             "candle width at 12 cm (px, policy size)": round(0.005 * cam.lens.fx * 320 / cam.lens.width / 0.12, 1)}
    print(f"VIEW {json.dumps(found, default=str)}", flush=True)
    return found


def calibrate(out: Path, embodiment) -> dict:
    """The give of the cupcake's sides, for the constant that puts the table's 4 N at 4 mm of closing past touch:
    run with CUPCAKE_GAUGE="k0,k1,k2" (each cupcake's contact stiffness, N/m; no squash). The left hand holds each
    cupcake in turn at its rim from the near side (15 degrees under level) and closes in steps of 1 mm from a 61 mm
    gap to 47 mm; read at the end of each step: the closing past touch (pads' gap against the 60 mm rim) and the
    gripper's own force."""
    import numpy as np

    import demo_kit as kit
    import hands

    m = embodiment.task.module
    assert m.GAUGE, "the calibration needs CUPCAKE_GAUGE"
    rot = hands.side_grip(0.0, 15.0) @ np.diag([-1.0, -1.0, 1.0])
    approach = rot[:, 2]
    Move = kit.Move

    def grip(gap_mm: float) -> float:
        return gap_mm / 1000.0 / m.PAD_GAP_OPEN

    moves = []
    for k, cake in enumerate(m.CAKES[: len(m.GAUGE)]):
        grasp = np.array([m.ROW_X, m.CAKE_YS[k], m.BOARD_TOP + 0.0269])
        back = grasp - 0.08 * approach + np.array([0.0, 0.0, 0.04])
        moves += [Move(3.0, pos=back, rot=rot, grip=grip(75), label=f"{cake} towards"), Move(2.0, pos=grasp - 0.03 * approach, label=f"{cake} front"),
                  Move(2.0, pos=grasp, label=f"{cake} round the rim"), Move(0.5, label=f"{cake} settle")]  # fmt: skip
        for gap in range(61, 46, -1):
            moves.append(Move(1.2, grip=grip(gap), label=f"{cake} gap {gap}"))
        moves += [Move(1.0, grip=grip(75), label=f"{cake} open"), Move(2.0, pos=back, label=f"{cake} away")]

    def extra() -> dict:
        r = m.readings()
        return {"squeeze_mm": r["squeeze_mm"], "pinch_N": r["pinch_N"].get("left"), "gap_mm": r["gap_mm"]["left"]}

    result = kit.run(embodiment, {"left": moves}, out / "calibrate", seed=0, extra=extra, limit_s=150.0, tail_s=0.5, record=False)
    rows = [row for row in result["rows"] if "extra" in row]
    found = {"stiffness (N/m)": dict(zip(m.CAKES, m.GAUGE)), "curves (commanded gap mm: closing mm, force N)": {}}
    for k, cake in enumerate(m.CAKES[: len(m.GAUGE)]):
        curve = {}
        for gap in range(61, 46, -1):
            rs = [row for row in rows if row["left"] == f"{cake} gap {gap}"]
            if rs:
                e = rs[-1]["extra"]
                curve[gap] = (e["squeeze_mm"][cake], e["pinch_N"])
        found["curves (commanded gap mm: closing mm, force N)"][cake] = curve
    found["worst IK (mm, deg)"] = result.get("worst_ik")
    print(f"CALIBRATE {json.dumps(found, default=str)}", flush=True)
    return found


def main() -> int:
    out = Path(sys.argv[1])
    known = {"hold": hold, "fall": fall, "rest": rest, "squeeze": squeeze, "view": view, "calibrate": calibrate}
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["hold"]
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    import demo_kit as kit

    own = {"hold": "hold_trial.py", "fall": "fall_trial.py"}  # trials on scenes of their own
    scene = next((own[name] for name in trials if name in own), "task.py")
    if "view" in trials:
        os.environ["CUPCAKE_VIEW"] = "1"  # read by the task file when it is loaded
    if "calibrate" in trials:
        os.environ.setdefault("CUPCAKE_GAUGE", "250,500,1000")
    if any(name in own for name in trials) and len(trials) > 1:
        raise SystemExit("hold and fall run on scenes of their own: run each alone")
    embodiment = kit.open_task(HERE / scene)
    failed = 0
    for name in trials:
        try:
            result = known[name](out, embodiment)
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
