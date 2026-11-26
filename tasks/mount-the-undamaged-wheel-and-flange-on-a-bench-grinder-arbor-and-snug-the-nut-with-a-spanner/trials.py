#!/usr/bin/env python3
"""PhysX trials verifying each score level (0.0, 0.5, 1.0), one-way flange fit, counter-hold mechanics, and zero rules.

    scripts/rta simpy <id> mech/tasks/grinder_wheel/trials.py /workspace/robocurve/outputs/<id> [names]
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import parts as P  # noqa: E402


def _hold_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _quat(m, rot: np.ndarray) -> tuple:
    return tuple(float(v) for v in m.mk.matrix_to_quat(rot))


def _seat_sound_wheel(m) -> None:
    m.mk.write_pose("wheel_sound", (P.GRINDER_X, P.Y_WHEEL_SEATED, m.TOP + P.ARBOR_Z), (0.0, 0.0, 0.0, 1.0))


def _seat_flange(m, *, backwards: bool = False) -> None:
    rot = K.rz(math.pi if backwards else 0.0)
    m.mk.write_pose("outer_flange", (P.GRINDER_X, P.Y_FLANGE_SEATED, m.TOP + P.ARBOR_Z), _quat(m, rot))


def _engage_nut_at_deg(m, phi_deg: float) -> None:
    travel = math.radians(min(phi_deg, 140.0))  # the nut_turn hinge stops at 150 deg: past it the snug mark shifts instead
    m.mk.set_joint("nut_turn", travel)
    m.STATE["snug_ref"] = travel - math.radians(phi_deg)
    m.STATE["nut_held"] = travel
    rot = K.ry(K.NUT_SNUG_ROLL + travel)
    m.mk.write_pose("arbor_nut", (P.GRINDER_X, P.Y_NUT_SNUG, m.TOP + P.ARBOR_Z), _quat(m, rot))
    m.mk.make("nut_catch")
    m._carry_threaded_nut()  # as the thread rule does when the nut catches
    tau = P.thread_torque_for_angle_deg(max(0.0, phi_deg), stack_clamped=True)
    m.mk.set_spring("nut_turn", rest=travel, most=tau)


def _step_ready(emb, seconds: float):
    hz = float(emb.info.control_hz)
    act = _hold_action()
    res = None
    for _ in range(int(round(seconds * hz))):
        res = emb.step(act)
    return res


def cracked_wheel_zero(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="cracked_zero", instruction=m.INSTRUCTION), seed=0)
    cx, cy, cz = P.CRACKED_START
    m.mk.write_pose("wheel_cracked", (cx, cy - 0.018, m.TOP + cz), _quat(m, K.rz(P.CRACKED_YAW)))
    _step_ready(emb, 0.2)
    # Even if the sound stack is subsequently completed into the safe window, latched zero stays 0.0
    _seat_sound_wheel(m)
    _seat_flange(m, backwards=False)
    _engage_nut_at_deg(m, 38.0)
    res = _step_ready(emb, 1.5)
    rep = m.report()
    return {
        "score": float(res.info.get("score", 0.0)),
        "report": rep,
        "pass": float(res.info.get("score", 0.0)) == 0.0 and rep["zero"] == "cracked wheel moved more than 10 mm from its rack slot",
    }


def flange_backwards_zero(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="flange_backwards", instruction=m.INSTRUCTION), seed=0)
    _seat_sound_wheel(m)
    _seat_flange(m, backwards=True)
    res = _step_ready(emb, 1.2)
    rep = m.report()
    return {
        "score": float(res.info.get("score", 0.0)),
        "report": rep,
        "pass": float(res.info.get("score", 0.0)) == 0.0 and not rep["flange seated (recess inward)"] and rep["flange rim gap (mm)"] >= 5.0,
    }


def no_counterhold_half(emb, m, out_dir: Path | None = None) -> dict:
    """PhysX arm trial: Right arm pulls the 19 mm spanner on the snugged nut with Left arm kept at READY (no counterhold).
    Spindle spins freely (~20-35 deg), witness angle stays well below 17 deg (2.5 N m), score is 0.5."""
    import demo_kit as kit
    import demo as D

    def setup_snug(_t):
        if not getattr(setup_snug, "done", False):
            _seat_sound_wheel(m)
            _seat_flange(m, backwards=False)
            _engage_nut_at_deg(m, 0.0)
            setup_snug.done = True

    setup_snug.done = False  # type: ignore[attr-defined]
    # The demo's own spanner moves and stroke director, with the left arm left at READY
    right = D.spanner_moves()
    tmp_dir = (out_dir / "nc") if out_dir is not None else Path("/tmp/gw_trial_nocounter")
    res = kit.run(emb, {"right": right}, tmp_dir, seed=0, director=D.Director(m, setup=setup_snug), tail_s=1.0, record=False, limit_s=60.0)
    rep = res["report"]
    score_end = float(res["info"].get("score", 0.0))
    return {
        "score": score_end,
        "report": rep,
        "pass": score_end == 0.5 and abs(rep["spindle turn (deg)"]) >= 8.0 and rep["witness angle past snug (deg)"] < P.ANGLE_MIN_DEG,
    }


def over_torque_zero(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="over_torque", instruction=m.INSTRUCTION), seed=0)
    _seat_sound_wheel(m)
    _seat_flange(m, backwards=False)
    _engage_nut_at_deg(m, 200.0)  # > 195 deg (27.1 N m)
    _step_ready(emb, 0.3)
    # Even if backed off afterward into [17, 195] deg, the crushed-wheel zero rule stays latched
    _engage_nut_at_deg(m, 36.0)
    res = _step_ready(emb, 1.5)
    rep = m.report()
    return {
        "score": float(res.info.get("score", 0.0)),
        "report": rep,
        "pass": float(res.info.get("score", 0.0)) == 0.0 and "over-tightened" in str(rep["zero"]),
    }


def dropped_off_bench_zero(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="drop_zero", instruction=m.INSTRUCTION), seed=0)
    # Beyond the y = -0.60 m right edge of the 0.80 x 1.20 m station bench so gravity drops it past TOP - 15 mm
    m.mk.write_pose("wrench", (0.25, -0.68, m.TOP - 0.030), (0.0, 0.0, 0.0, 1.0))
    res = _step_ready(emb, 0.4)
    rep = m.report()
    return {
        "score": float(res.info.get("score", 0.0)),
        "report": rep,
        "pass": float(res.info.get("score", 0.0)) == 0.0 and rep["zero"] == "wrench dropped off the bench",
    }


def snug_in_window_full(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="full_state", instruction=m.INSTRUCTION), seed=0)
    _seat_sound_wheel(m)
    _seat_flange(m, backwards=False)
    _engage_nut_at_deg(m, 36.0)  # 5.12 N m, inside [17, 195] deg = [2.5, 27.1] N m
    res = _step_ready(emb, 1.5)
    rep = m.report()
    return {
        "score": float(res.info.get("score", 0.0)),
        "report": rep,
        "pass": float(res.info.get("score", 0.0)) == 1.0 and P.TAU_MIN <= rep["thread torque (N m)"] <= P.TAU_MAX,
    }


TRIALS = {
    "cracked_wheel_zero": cracked_wheel_zero,
    "flange_backwards_zero": flange_backwards_zero,
    "no_counterhold_half": no_counterhold_half,
    "over_torque_zero": over_torque_zero,
    "dropped_off_bench_zero": dropped_off_bench_zero,
    "snug_in_window_full": snug_in_window_full,
}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    ok = True
    summary = {}
    for name in names:
        fn = TRIALS[name]
        result = fn(emb, m, out) if name == "no_counterhold_half" else fn(emb, m)
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
        print(f"TRIAL {name} " + json.dumps(result, default=str), flush=True)
        summary[name] = {"pass": bool(result["pass"]), "score": result["score"], "zero": result["report"]["zero"]}
        ok &= bool(result["pass"])
    (out / "summary.json").write_text(json.dumps({"all_pass": ok, "trials": summary}, indent=1))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
