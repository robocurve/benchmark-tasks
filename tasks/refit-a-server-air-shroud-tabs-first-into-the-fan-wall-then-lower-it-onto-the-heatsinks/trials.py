#!/usr/bin/env python3
"""Gate G2 240 Hz PhysX mechanics and rule trials for the 2U server air-shroud task:

    scripts/rta simpy <id> mech/tasks/server_shroud/trials.py /workspace/robocurve/outputs/<id>

Trials:
- ``flat_drop_blocked``       shroud released level over X_SEAT: front tabs strike the 83.0 mm bulkhead lip, roof stays
                              proud (> 86 mm), 0 tabs in slots -> score 0.0.
- ``rear_shift_drop_blocked`` shroud released level 12 mm rearward so tabs clear the bulkhead: rear floor strikes the
                              52.0 mm mid-bay divider, roof stays > 125 mm -> score 0.0.
- ``tucked_hung_half``        both front tabs tucked 7.5 mm into the 22.0 x 7.5 mm slots under the lip, shroud yawed
                              2.8 deg so its CPU duct skirt hangs on top of the 72.0 mm Dynatron S2 heatsink -> 0.5.
- ``tucked_and_seated_full``  both front tabs tucked 7.5 mm into the slots at 2.0 deg tilt inside the +/-2.5 mm heatsink
                              clearance: settles flat onto all 4 corners (78.0 +/- 1.5 mm) and holds 1.0 s -> 1.0.
- ``heatsink_touch_zero``     left gripper dips onto the exposed left heatsink (latches foul), then shroud is seated
                              flat for 1.5 s -> stays 0.0 permanently.
- ``dropped_outside_zero``    shroud placed beside the 430 mm bay falls onto the bench (latches foul), then shroud is
                              seated flat for 1.5 s -> stays 0.0 permanently.
"""

from __future__ import annotations

import os

for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

SAMPLE_S = 0.20


def _ready_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _quat(m, rot: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in m.mk.matrix_to_quat(rot))


def _rot_yp(m, pitch_deg: float = 0.0, yaw_deg: float = 0.0) -> np.ndarray:
    rz = m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(yaw_deg))
    ry = m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(pitch_deg))
    return rz @ ry


def _step_ready(embodiment, m, seconds: float) -> tuple[dict, list[dict]]:
    hz = float(embodiment.info.control_hz)
    every = max(1, int(round(SAMPLE_S * hz)))
    act = _ready_action()
    samples = []
    res = None
    for i in range(int(round(seconds * hz))):
        res = embodiment.step(act)
        if (i + 1) % every == 0:
            rep = m.report()
            samples.append({
                "t_s": round((i + 1) / hz, 2),
                "score": float(res.info.get("score", 0.0)),
                "zero": rep["zero"],
                "tabs_mm": rep["tab in slot (mm)"],
                "corners_mm": rep["corner roof heights (mm)"],
                "chassis_N": rep["chassis contact (N)"],
            })
    assert res is not None
    return {"end_score": float(res.info.get("score", 0.0)), "report": m.report()}, samples


def flat_drop_blocked(embodiment, m) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="flat_drop", instruction=m.INSTRUCTION), seed=0)
    # Release level 18 mm above the seated height directly over X_SEAT: tabs overlap the 83.0 mm bulkhead lip
    m.mk.write_pose("shroud", (m.P.X_SEAT, 0.0, m.FLOOR_Z + 0.018), (0.0, 0.0, 0.0, 1.0))
    out, samples = _step_ready(embodiment, m, 2.0)
    rep = out["report"]
    front_h = min(rep["corner roof heights (mm)"]["front_left"], rep["corner roof heights (mm)"]["front_right"])
    out["samples"] = samples
    out["pass"] = (
        out["end_score"] == 0.0
        and rep["zero"] is None
        and max(rep["tab in slot (mm)"].values()) == 0.0
        and front_h >= 84.0
        and rep["chassis contact (N)"] >= 4.0
    )
    return out


def rear_shift_drop_blocked(embodiment, m) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="rear_shift", instruction=m.INSTRUCTION), seed=0)
    # Shifted 12 mm back (-x) so tabs miss bulkhead: rear 43.0 mm stepped floor lands on the 52.0 mm mid-bay divider
    # (propping rear roof to ~87.5 mm > 79.5 mm limit, with 0.0 mm tab engagement)
    m.mk.write_pose("shroud", (m.P.X_SEAT - 0.012, 0.0, m.FLOOR_Z + 0.020), (0.0, 0.0, 0.0, 1.0))
    out, samples = _step_ready(embodiment, m, 2.0)
    rep = out["report"]
    max_corner = max(rep["corner roof heights (mm)"].values())
    out["samples"] = samples
    out["pass"] = (
        out["end_score"] == 0.0
        and max(rep["tab in slot (mm)"].values()) == 0.0
        and max_corner >= 85.0
    )
    return out


def tucked_hung_half(embodiment, m) -> dict:
    import checks as K
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station

    embodiment.reset(Scene(id="tucked_hung", instruction=m.INSTRUCTION), seed=0)
    # Bimanual arm execution of the first half only (pickup -> cross divider -> 17 deg nose-down -> tuck both front
    # tabs into the 22 x 7.5 mm fan-wall slots), stopping before rear hinge-down: rear stays > 110 mm proud -> 0.5!
    moves_all = K.plan_moves(m.TOP)
    moves_half = {s: moves_all[s][:7] for s in K.SIDES}  # up to and including tucked_17
    arms = {s: kit.Arm(s, station.BASES[s], moves_half[s]) for s in K.SIDES}
    hz = float(embodiment.info.control_hz)
    total_s = max(a.seconds for a in arms.values()) + 1.0
    every = max(1, int(round(SAMPLE_S * hz)))
    samples = []
    res = None
    for i in range(int(round(total_s * hz))):
        t = i / hz
        act_l, _ = arms["left"].action(t)
        act_r, _ = arms["right"].action(t)
        res = embodiment.step(Action(data=pack(act_l, act_r)))
        if (i + 1) % every == 0:
            rep = m.report()
            samples.append({
                "t_s": round((i + 1) / hz, 2),
                "score": float(res.info.get("score", 0.0)),
                "tabs_mm": rep["tab in slot (mm)"],
                "corners_mm": rep["corner roof heights (mm)"],
            })
    assert res is not None
    rep = m.report()
    rear_h = max(rep["corner roof heights (mm)"]["rear_left"], rep["corner roof heights (mm)"]["rear_right"])
    out = {"end_score": float(res.info.get("score", 0.0)), "report": rep, "samples": samples}
    out["pass"] = (
        out["end_score"] == 0.5
        and rep["zero"] is None
        and min(rep["tab in slot (mm)"].values()) >= 5.0
        and rear_h > 95.0
    )
    return out


def tucked_and_seated_full(embodiment, m) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="tucked_full", instruction=m.INSTRUCTION), seed=0)
    # Both front tabs tucked 7.5 mm in slots at 1.5 deg tilt inside the +/-2.5 mm heatsink clearance -> settles flat
    piv_loc = np.array([m.P.SHROUD_L / 2.0 + 0.0055, 0.0, m.P.TAB_Z_MID])
    piv_w = np.array([m.P.X_SEAT + m.P.SHROUD_L / 2.0 + 0.0055, 0.0, m.FLOOR_Z + m.P.TAB_Z_MID])
    r = _rot_yp(m, pitch_deg=1.5, yaw_deg=0.0)
    m.mk.write_pose("shroud", piv_w - r @ piv_loc, _quat(m, r))
    out, samples = _step_ready(embodiment, m, 2.0)
    rep = out["report"]
    corners = list(rep["corner roof heights (mm)"].values())
    out["samples"] = samples
    out["pass"] = (
        out["end_score"] == 1.0
        and rep["zero"] is None
        and min(rep["tab in slot (mm)"].values()) >= 5.0
        and all(76.5 <= c <= 79.5 for c in corners)
        and rep["chassis contact (N)"] >= 4.5
    )
    return out


def heatsink_touch_zero(embodiment, m) -> dict:
    import checks as K
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import constants as C, station

    embodiment.reset(Scene(id="hs_foul", instruction=m.INSTRUCTION), seed=0)
    # Drive left fingertip down onto the top of the left Dynatron S2 heatsink (x = 0.3915, y = +0.095, h = 72 mm)
    hs_top = np.array([m.P.X_SEAT, m.P.SLOT_Y, m.FLOOR_Z + m.P.SOCKET_H + m.P.HS_H - 0.002])
    _, rot = K.grasp_pose([m.P.X_SEAT, 0.0, m.FLOOR_Z], 0.0, "left")
    arm_l = kit.Arm("left", station.BASES["left"], [
        kit.Move(1.2, pos=hs_top + np.array([0.0, 0.0, 0.060]), rot=rot, grip=0.0, label="above_hs"),
        kit.Move(1.0, pos=hs_top + np.array([0.0, 0.0, 0.008]), rot=rot, grip=0.0, label="touch_hs"),
    ])
    hz = float(embodiment.info.control_hz)
    for i in range(int(round(2.3 * hz))):
        act_l, _ = arm_l.action(i / hz)
        embodiment.step(Action(data=pack(act_l, np.r_[C.READY, 1.0])))
    foul_seen = m.STATE.get("zero")
    # Retract arms to READY and place shroud in full seated pose for 1.5 s: score must stay 0.0 permanently
    m.mk.write_pose("shroud", (m.P.X_SEAT, 0.0, m.FLOOR_Z + 0.0005), (0.0, 0.0, 0.0, 1.0))
    out, samples = _step_ready(embodiment, m, 1.8)
    out["foul_seen"] = foul_seen
    out["samples"] = samples
    out["pass"] = foul_seen is not None and "heatsink left" in foul_seen and out["end_score"] == 0.0
    return out


def dropped_outside_zero(embodiment, m) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="drop_out", instruction=m.INSTRUCTION), seed=0)
    # Place shroud off the left side of the 430 mm bay so it drops onto the bench
    m.mk.write_pose("shroud", (0.28, 0.30, m.TOP + 0.025), (0.0, 0.0, 0.0, 1.0))
    _step_ready(embodiment, m, 0.8)
    foul_seen = m.STATE.get("zero")
    # Put shroud back into full seated pose for 1.5 s: score must stay 0.0 permanently
    m.mk.write_pose("shroud", (m.P.X_SEAT, 0.0, m.FLOOR_Z + 0.0005), (0.0, 0.0, 0.0, 1.0))
    out, samples = _step_ready(embodiment, m, 1.5)
    out["foul_seen"] = foul_seen
    out["samples"] = samples
    out["pass"] = foul_seen == "the shroud fell outside the 2U server bay" and out["end_score"] == 0.0
    return out


TRIALS = {
    "flat_drop_blocked": flat_drop_blocked,
    "rear_shift_drop_blocked": rear_shift_drop_blocked,
    "tucked_hung_half": tucked_hung_half,
    "tucked_and_seated_full": tucked_and_seated_full,
    "heatsink_touch_zero": heatsink_touch_zero,
    "dropped_outside_zero": dropped_outside_zero,
}


def main() -> int:
    import demo_kit as kit

    out_dir = Path(sys.argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    all_ok = True
    for name in names:
        res = TRIALS[name](embodiment, m)
        (out_dir / f"{name}.json").write_text(json.dumps(res, indent=2, default=str))
        brief = {k: v for k, v in res.items() if k != "samples"}
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        all_ok = all_ok and bool(res["pass"])
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
