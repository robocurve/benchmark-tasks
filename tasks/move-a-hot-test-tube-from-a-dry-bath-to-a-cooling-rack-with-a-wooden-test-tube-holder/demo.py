#!/usr/bin/env python3
"""The scripted full-score reference run of ``hot_tube_clamp``, and an offline dry-run check of its trajectory.

    scripts/rta demo <id> mech/tasks/hot_tube_clamp [--check|--no-video] [--quick]

Open-loop trajectory built from ``checks.py``'s verified poses:
1. Right hand picks the Eisco CH0726B wooden clothespin holder by its rear handle off the blue silicone mat and stages it
   in front of the FOUR E'S 95.0 C dry bath.
2. Left hand squeezes the short lever at 120.0 mm spacing from the right hand to open the 90-deg V-jaws against the
   torsion spring; both hands advance in lockstep to fit the open V-jaws round the exposed 16.0 mm PYREX tube neck well
   clear of the 96 x 76 mm heating block; left hand releases so the spring clamps the tube (~15.7 N per jaw), then moves
   clear.
3. Right hand lifts the clamped tube plumb out of the 45.0 mm bore (0.25), carries it over hole (1, 1) of the BOCHEM
   8291 stainless Z-rack, and lowers it plumb through both perforated guide tiers to 2 mm above the stainless floor.
4. Left hand squeezes the lever open to set the tube upright on the rack floor (0.5); both hands withdraw the open jaws
   straight back clear of the tube; left hand returns to ready while right hand lays the wooden holder flat back on the
   blue mat and returns to ready (1.0 after the 1.0 s stillness hold).
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

OPEN = 1.0
FINGER_STIFFNESS = 5000.0  # N/m per finger drive (yam_arena constants)
R_OPEN_MAT = 0.050 / K.MAX_JAW
R_HOLD_GRIP = 0.0  # ~29 N per finger pad (<= 40 N) on the 12 mm wooden handle so cantilever torque never slips
SQUEEZE_GRIP = 0.012 / K.MAX_JAW  # ~32 N per finger pad (<= 40 N) across the angled tail -> opens jaws to ~31.5 mm!
HOLDER_COM_AT = tuple(float(v) for v in (K.R_ROT.T @ np.array([0.125, 0.0, 0.0])))  # empty 45.4 g holder COM from R_TCP
LOADED_COM_AT = tuple(float(v) for v in (K.R_ROT.T @ np.array([0.165, 0.0, 0.0])))  # 45.4 g holder + 30 g tube COM from R_TCP
HOLDER_PUSH = (0.0, 0.0, 0.45)  # N: gravity + pitch torque balance for the 45.4 g cantilevered holder
CARRY_PUSH = (0.0, 0.0, 0.60)  # N: gravity + pitch torque balance for the 45.4 g holder + 13.5 g tube (58.9 g)
BATH_AIM_OFFSET = np.array([-0.0010, 0.0040, 0.0])  # m: places V-pocket dead-centre at WELL_XY (428.0, 115.0 mm)
MOUTH_CENTER_DY = np.array([0.0, -0.0065, 0.0])  # m: shifts clamp -6.5 mm in y while jaws are open so tube passes mid-throat!
RACK_HI_OFFSET = np.array([0.0003, 0.0004, 0.0])  # m: centres clamped tube on TARGET_HOLE_XY (405.0, -85.0 mm) to < 0.2 mm!
RACK_LO_OFFSET = np.array([0.0008, 0.0024, 0.0])  # m: keeps clamped tube dead-coaxial down through both Z-rack tiers!


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    pin_mat = np.array([P.CLAMP_START_PIN[0], P.CLAMP_START_PIN[1], K.TOP + K.CLAMP_START_Z], dtype=float)
    pin_mat_set = pin_mat + np.array([0.0, 0.0, 0.0015])
    pin_bath = K.clamp_pin_for_tube_at(P.WELL_XY, K.Z_CLAMP_AT_BATH) + BATH_AIM_OFFSET
    pin_bath_open = pin_bath + MOUTH_CENTER_DY
    pin_stage = pin_bath_open - np.array([K.STAGE_BACK_X, 0.0, 0.0])
    pin_lift = K.clamp_pin_for_tube_at(P.WELL_XY, K.Z_CLAMP_LIFT) + BATH_AIM_OFFSET
    pin_rack_hi = K.clamp_pin_for_tube_at(P.TARGET_HOLE_XY, K.Z_CLAMP_LIFT) + RACK_HI_OFFSET
    pin_rack_lo = K.clamp_pin_for_tube_at(P.TARGET_HOLE_XY, K.Z_CLAMP_AT_RACK) + RACK_LO_OFFSET
    pin_rack_open = pin_rack_lo + MOUTH_CENTER_DY
    pin_rack_back = pin_rack_open - np.array([0.050, 0.0, 0.0])

    span_0 = P.tail_span(0.0)
    span_tube = P.tail_span(K.THETA_TUBE)

    # Phase 1 (0.0 -> 7.2 s): right picks holder from mat & stages before bath; left moves to stage approach
    right = [
        Move(2.0, pos=K.right_grip_for_pin(pin_mat) + np.array([0.0, 0.0, 0.055]), rot=K.R_ROT, grip=R_OPEN_MAT, label="over holder"),
        Move(1.2, pos=K.right_grip_for_pin(pin_mat), label="on rear handle"),
        Move(0.8, grip=R_HOLD_GRIP, label="pinch handle"),
        Move(1.2, pos=K.right_grip_for_pin(pin_mat) + np.array([0.0, 0.0, 0.090]), push=HOLDER_PUSH, push_at=HOLDER_COM_AT, label="lift holder"),
        Move(2.0, pos=K.right_grip_for_pin(pin_stage), label="stage before bath"),
    ]
    left = [
        Move(5.2, pos=K.left_squeeze_for_pin(pin_stage, 0.0, out_y=0.045, up_z=0.020), rot=K.L_ROT, grip=OPEN, label="to stage tail"),
        Move(2.0, label="wait stage"),
    ]

    # Phase 2 (7.2 -> 14.6 s): squeeze open, advance mid-throat round tube, seat V-cradle, release onto tube, left clears
    right += [
        Move(1.0, label="hold stage for tail"),
        Move(1.2, label="hold stage while squeezed"),
        Move(1.6, pos=K.right_grip_for_pin(pin_bath_open), label="jaws round tube"),
        Move(0.7, pos=K.right_grip_for_pin(pin_bath), label="seat v-cradle"),
        Move(0.8, push=CARRY_PUSH, push_at=LOADED_COM_AT, label="hold while clamping"),
        Move(0.8, label="settle clamped"),
        Move(1.2, label="wait left clear"),
    ]
    left += [
        Move(1.0, pos=K.left_squeeze_for_pin(pin_stage, 0.0), grip=(span_0 + 0.020) / K.MAX_JAW, label="around tail"),
        Move(1.2, pos=K.left_squeeze_for_pin(pin_stage, K.THETA_OPEN), grip=SQUEEZE_GRIP, label="squeeze open"),
        Move(1.6, pos=K.left_squeeze_for_pin(pin_bath_open, K.THETA_OPEN), grip=SQUEEZE_GRIP, label="advance open"),
        Move(0.7, pos=K.left_squeeze_for_pin(pin_bath, K.THETA_OPEN), grip=SQUEEZE_GRIP, label="seat open"),
        # Open fingers in place first (zero y pull on long_arm while releasing!), then retract +y:
        Move(0.8, grip=OPEN, label="release to clamp"),
        Move(0.8, pos=K.left_squeeze_for_pin(pin_bath, K.THETA_TUBE, out_y=0.030), label="step off tail"),
        Move(1.2, pos=K.left_squeeze_for_pin(pin_bath, K.THETA_TUBE, out_y=0.090, up_z=0.035), grip=OPEN, label="clear of bath"),
    ]

    # Phase 3 (14.0 -> 22.0 s): right lifts plumb out of 45 mm bore, carries to rack hole (1, 1), lowers plumb
    right += [
        Move(2.2, pos=K.right_grip_for_pin(pin_lift), push=CARRY_PUSH, label="lift plumb"),
        Move(2.6, pos=K.right_grip_for_pin(pin_rack_hi), label="carry to rack"),
        Move(0.6, label="plumb settle"),
        Move(2.6, pos=K.right_grip_for_pin(pin_rack_lo), label="lower into rack"),
    ]
    left += [
        Move(2.2, label="wait lift"),
        Move(2.6, pos=np.array([0.230, 0.060, K.TOP + 0.130]), rot=K.L_ROT, grip=OPEN, label="arc front"),
        Move(3.2, pos=K.left_squeeze_for_pin(pin_rack_lo, K.THETA_TUBE, out_y=0.055, up_z=0.025), label="approach rack tail"),
    ]

    # Phase 4: squeeze open at rack, centre open mouth around standing tube, withdraw in lockstep, lay holder on blue mat
    right += [
        Move(1.0, label="hold at rack"),
        Move(1.2, push=HOLDER_PUSH, push_at=HOLDER_COM_AT, label="hold while opening"),
        Move(0.6, pos=K.right_grip_for_pin(pin_rack_open), label="centre open jaws"),
        Move(1.6, pos=K.right_grip_for_pin(pin_rack_back), label="withdraw open jaws"),
        Move(1.2, label="hold while left clears"),
        Move(2.2, pos=K.right_grip_for_pin(pin_mat) + np.array([0.0, 0.0, 0.060]), rot=K.R_ROT, label="over mat"),
        Move(1.4, pos=K.right_grip_for_pin(pin_mat_set), push=(0.0, 0.0, 0.0), label="lay on mat"),
        Move(0.8, grip=R_OPEN_MAT, label="release holder"),
        Move(1.0, pos=K.right_grip_for_pin(pin_mat) + np.array([0.0, 0.0, 0.065]), grip=OPEN, label="clear of holder"),
        Move(2.4, pos=ready("right")[0], rot=ready("right")[1], grip=OPEN, label="ready"),
    ]
    left += [
        Move(1.0, pos=K.left_squeeze_for_pin(pin_rack_lo, K.THETA_TUBE), grip=(span_tube + 0.020) / K.MAX_JAW, label="around rack tail"),
        Move(1.2, pos=K.left_squeeze_for_pin(pin_rack_lo, K.THETA_OPEN), grip=SQUEEZE_GRIP, label="squeeze to release"),
        Move(0.6, pos=K.left_squeeze_for_pin(pin_rack_open, K.THETA_OPEN), grip=SQUEEZE_GRIP, label="centre tail"),
        Move(1.6, pos=K.left_squeeze_for_pin(pin_rack_back, K.THETA_OPEN), grip=SQUEEZE_GRIP, label="withdraw tail"),
        Move(1.2, pos=K.left_squeeze_for_pin(pin_rack_back, 0.0, out_y=0.065, up_z=0.040), grip=OPEN, label="let go tail"),
        Move(2.5, pos=ready("left")[0], rot=ready("left")[1], grip=OPEN, label="ready"),
    ]
    return {"left": left, "right": right}


def save_end_picture(frame: np.ndarray, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    moves = plan()
    if "--quick" in sys.argv[2:]:
        from yam_arena import station

        dry = kit.dry_run(moves, station.BASES)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk

    def finger_squeeze_forces() -> dict[str, float]:
        """Normal forces (N) reported between each arm and the wooden holder's two parts."""
        totals: dict[str, float] = {}
        for part in ("long_arm", "short_lever"):
            for c in mk.contacts(part):
                if c.other.endswith("robot"):
                    key = f"{c.other} on {part}"
                    totals[key] = round(totals.get(key, 0.0) + float(c.force), 2)
        return totals

    def extra() -> dict:
        fl, fs = m.jaw_forces_on_tube()
        pt = mk.pose("tube")[0]
        pl, rl = mk.pose("long_arm")
        vp = pl + rl @ np.array([P.X_JAW, P.tube_y_in_clamp(P.TUBE_R), 0.0])
        return {
            "tube_xy_mm": [round(1000.0 * float(pt[0]), 1), round(1000.0 * float(pt[1]), 1)],
            "vpocket_xy_mm": [round(1000.0 * float(vp[0]), 1), round(1000.0 * float(vp[1]), 1)],
            "tube_bot_mm": round(1000.0 * m.tube_bottom_above_bench(), 2),
            "tube_tilt_deg": round(m.tube_tilt_deg(), 2),
            "jaw_deg": round(math.degrees(m.jaw_angle()), 2),
            "jaw_gap_mm": round(1000.0 * P.jaw_gap(m.jaw_angle()), 2),
            "jaw_on_tube_N": [round(fl, 2), round(fs, 2)],
            "bore_drag_N": round(float(mk.contact_force("tube", "bath")), 2),
            "rack_contact_N": round(float(mk.contact_force("tube", "rack")), 2),
            "finger_N": finger_squeeze_forces(),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
        }

    folder = out / "full"
    record = not any(flag in ("--no-video", "--check") for flag in sys.argv[2:])
    latest: dict = {}

    def director(t, arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)

    result = kit.run(
        embodiment,
        moves,
        folder,
        seed=0,
        director=director,
        extra=extra,
        limit_s=float(m.RUN_LIMIT_STEPS) / 20.0,
        tail_s=5.0,
        sheet_frames=16,
        record=record,
    )
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)

    first_q = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None)
    first_h = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.50), None)
    first_f = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)
    squeeze_log = [
        {
            "t": r["t"],
            "left": r["left"],
            "jaw_gap_mm": r["extra"]["jaw_gap_mm"],
            "finger_N": r["extra"]["finger_N"],
            "jaw_on_tube_N": r["extra"]["jaw_on_tube_N"],
        }
        for r in rows[::8]
        if r["extra"]["finger_N"]
    ]
    readings = {
        "score_end": rows[-1]["info"].get("score"),
        "steps": len(rows),
        "seconds": rows[-1]["t"],
        "first_0.25_s": first_q,
        "first_0.50_s": first_h,
        "first_1.00_s": first_f,
        "max_jaw_gap_mm": max(r["extra"]["jaw_gap_mm"] for r in rows),
        "peak_jaw_on_tube_N": max(min(r["extra"]["jaw_on_tube_N"]) for r in rows),
        "max_bore_drag_N": max(r["extra"]["bore_drag_N"] for r in rows),
        "both_arms_ready_end": {s: rows[-1][s] == "done" for s in ("left", "right")},
        "hands_clear_end_mm": rows[-1]["extra"]["hands_clear_mm"],
        "squeeze_vs_gap_log": squeeze_log,
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
