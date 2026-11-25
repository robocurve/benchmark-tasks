#!/usr/bin/env python3
"""Simulator physics trials for ``hot_tube_clamp`` (one trial per score level and zero rule, all run in one container):

    scripts/rta simpy <id> mech/tasks/hot_tube_clamp/trials.py /workspace/robocurve/outputs/<id>

- ``bare_touch_zero``: left hand touches the 96 x 76 mm 95.0 C heating block -> permanently 0.0.
- ``midair_drop_zero``: tube released from open jaws over the bench -> hits bench -> permanently 0.0.
- ``tilted_bore_jam``: tube tilted in the 45.0 mm bore -> binds against opposite bore walls (0.0).
- ``quarter_clamped_lifted``: wooden holder clamped round the 16.0 mm tube neck under its real torsion spring and held
  clear over the dry bath -> 0.25.
- ``half_racked_holder_out``: tube standing upright in rack hole (3, 1) on the stainless floor while the holder sits off
  the mat -> 0.5.
- ``full_end_state``: tube upright in rack hole (3, 1), wooden holder flat on the blue mat, both arms at ready -> 1.0
  after the 1.0 s stillness hold.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _ready_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    q = np.r_[READY, 1.0]
    return Action(data=pack(q, q))


def _quat(m, rot: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in m.mk.matrix_to_quat(rot))


def _step_ready(emb, m, seconds: float) -> tuple[float, list[dict]]:
    hz = float(emb.info.control_hz)
    n = int(round(seconds * hz))
    act = _ready_action()
    samples = []
    res = None
    for i in range(n):
        res = emb.step(act)
        if (i + 1) % 5 == 0 or i == n - 1:
            fl, fs = m.jaw_forces_on_tube()
            samples.append(
                {
                    "t": round((i + 1) / hz, 2),
                    "score": float(res.info.get("score", 0.0)),
                    "zero": m.STATE.get("zero"),
                    "tube_bot_mm": round(1000.0 * m.tube_bottom_above_bench(), 2),
                    "tube_tilt_deg": round(m.tube_tilt_deg(), 2),
                    "jaw_deg": round(math.degrees(m.jaw_angle()), 2),
                    "jaw_forces_N": [round(fl, 2), round(fs, 2)],
                    "bath_force_N": round(float(m.mk.contact_force("tube", "bath")), 2),
                }
            )
    return float(res.info.get("score", 0.0)), samples


def bare_touch_zero(emb, m) -> dict:
    """Lower left gripper onto the 96 x 76 mm hot block -> physical contact triggers permanent 0.0."""
    import demo_kit as kit

    Move = kit.Move
    target = np.array([m.P.HOT_BLOCK_XY[0], m.P.HOT_BLOCK_XY[1], m.TOP + m.P.BATH_H + 0.008])
    moves = {"left": [Move(1.4, pos=target, rot=kit.down(0.0), grip=0.0, label="touch hot block")]}
    tmp = Path("/tmp/trial_bare_touch")
    kit.run(emb, moves, tmp, seed=0, tail_s=0.3, limit_s=3.0, record=False)
    rep = m.report()
    return {"score": rep["score"], "zero": rep["zero"], "report": rep, "pass": rep["score"] == 0.0 and rep["zero"] is not None}


def midair_drop_zero(emb, m) -> dict:
    """Place tube 80 mm above the open bench with jaws open -> gravity drops tube onto the bench -> permanent 0.0."""
    from inspect_robots import Scene

    emb.reset(Scene(id="drop", instruction=m.INSTRUCTION), seed=0)
    m.mk.write_pose("tube", (0.26, 0.02, m.TOP + 0.130), (0.0, 0.0, 0.0, 1.0))
    sc, samples = _step_ready(emb, m, 1.2)
    rep = m.report()
    return {"score": sc, "zero": rep["zero"], "report": rep, "samples": samples, "pass": sc == 0.0 and rep["zero"] is not None}


def tilted_bore_jam(emb, m) -> dict:
    """Tilt the 16.0 mm tube to 4.2 deg in the 45.0 mm bore -> contacts opposite bore walls and stays trapped (0.0)."""
    from inspect_robots import Scene

    emb.reset(Scene(id="jam", instruction=m.INSTRUCTION), seed=0)
    rot = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(4.2))
    z_c = m.TOP + m.P.WELL_FLOOR_Z + 0.008 + m.P.TUBE_L / 2.0
    m.mk.write_pose("tube", (m.P.WELL_XY[0], m.P.WELL_XY[1], z_c), _quat(m, rot), lin_vel=(0.0, 0.0, 0.15))
    sc, samples = _step_ready(emb, m, 1.0)
    peak_wall = max(s["bath_force_N"] for s in samples)
    rep = m.report()
    return {
        "score": sc,
        "peak_bore_wall_N": peak_wall,
        "report": rep,
        "samples": samples,
        "pass": sc == 0.0 and peak_wall > 0.05 and m.tube_bottom_above_bench() < m.P.BATH_H,
    }


def quarter_clamped_lifted(emb, m) -> dict:
    """Run the reference trajectory through plumb bore extraction (stopping at 16.5 s) -> verifies 0.25 partial level."""
    import demo
    import demo_kit as kit

    tmp = Path("/tmp/trial_q25")
    res = kit.run(emb, demo.plan(), tmp, seed=0, tail_s=0.0, limit_s=16.5, record=False)
    sc = float(res["rows"][-1]["info"].get("score", 0.0))
    rep = m.report()
    return {"score": sc, "report": rep, "pass": sc == 0.25 and rep["zero"] is None}


def half_racked_holder_out(emb, m) -> dict:
    """Tube standing upright in BOCHEM 8291 hole (3, 1), wooden holder resting off the blue mat -> 0.5."""
    from inspect_robots import Scene

    emb.reset(Scene(id="half", instruction=m.INSTRUCTION), seed=0)
    hx, hy = m.P.TARGET_HOLE_XY
    z_t = m.TOP + m.P.OVER + m.P.RACK_SHEET_T + m.P.TUBE_L / 2.0 + 0.0005
    m.mk.write_pose("tube", (hx, hy, z_t), (0.0, 0.0, 0.0, 1.0))
    off_mat = (0.20, 0.04, m.TOP + m.P.BAR_H / 2.0 + 0.001)
    m.mk.write_pose("long_arm", off_mat, (0.0, 0.0, 0.0, 1.0))
    m.mk.write_pose("short_lever", off_mat, (0.0, 0.0, 0.0, 1.0))
    sc, samples = _step_ready(emb, m, 1.5)
    rep = m.report()
    return {"score": sc, "report": rep, "samples": samples, "pass": sc == 0.5}


def full_end_state(emb, m) -> dict:
    """Tube upright in BOCHEM 8291 hole (3, 1), wooden holder flat on blue mat, arms ready for 1.5 s -> 1.0."""
    from inspect_robots import Scene

    emb.reset(Scene(id="full_state", instruction=m.INSTRUCTION), seed=0)
    hx, hy = m.P.TARGET_HOLE_XY
    z_t = m.TOP + m.P.OVER + m.P.RACK_SHEET_T + m.P.TUBE_L / 2.0 + 0.0005
    m.mk.write_pose("tube", (hx, hy, z_t), (0.0, 0.0, 0.0, 1.0))
    sc, samples = _step_ready(emb, m, 1.5)
    rep = m.report()
    return {"score": sc, "report": rep, "samples": samples, "pass": sc == 1.0}


TRIALS = {
    "bare_touch_zero": bare_touch_zero,
    "midair_drop_zero": midair_drop_zero,
    "tilted_bore_jam": tilted_bore_jam,
    "quarter_clamped_lifted": quarter_clamped_lifted,
    "half_racked_holder_out": half_racked_holder_out,
    "full_end_state": full_end_state,
}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    ok = True
    for name in names:
        result = TRIALS[name](emb, m)
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
        brief = {k: v for k, v in result.items() if k != "samples"}
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        ok = ok and bool(result["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
