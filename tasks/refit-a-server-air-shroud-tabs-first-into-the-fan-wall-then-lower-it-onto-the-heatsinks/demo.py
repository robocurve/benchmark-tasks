#!/usr/bin/env python3
"""Scripted bimanual full-score reference run for the 2U server air-shroud task.

    scripts/rta demo <id> mech/tasks/server_shroud [--check|--no-video] [--quick]

Both hands descend into the shroud's recessed finger-lift slots on the front staging deck, pinch the 26.0 mm blue
touchpoint bridges (~20 N per pad), lift the shroud clear over the 52.0 mm mid-bay divider, pitch 17 deg nose-down,
slide forward (+x) to tuck both 10.0 mm front tabs into the 22.0 x 7.5 mm fan-wall slots beneath the 83.0 mm lip
(reaching 0.5), hinge the rear down from 17 deg to 0 deg between the two 78.0 mm Dynatron S2 heatsinks (2.50 mm
clearance per side) and four JEDEC MO-329 DDR5 RDIMM banks until all four corners seat at 78.0 mm, release, and return
both arms to READY so the 1.0 s hold completes (1.0) and the final frame shows the seated shroud unobstructed.

Open loop: reads nothing from the simulator to plan its actions. Results land in ``outputs/<id>/full/``.
"""

from __future__ import annotations

import os

for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import demo_kit as kit  # noqa: E402


def _write_clean_png(path: Path, rgb: np.ndarray) -> None:
    import cv2

    ok, buf = cv2.imencode(".png", cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR))
    if not ok:
        raise RuntimeError(f"could not encode {path}")
    path.write_bytes(bytes(buf))


def main() -> int:
    from yam_arena import station, task_file

    out = Path(sys.argv[1])
    flags = set(sys.argv[2:])
    top_z = task_file.TABLE_TOP["station"]
    moves = K.plan_moves(top_z)

    if "--quick" in flags:
        dry = kit.dry_run(moves, station.BASES, hz=4.0, limit_s=50.0)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk

    def finger_forces() -> dict[str, float]:
        totals = {"left": 0.0, "right": 0.0}
        for c in mk.contacts("shroud"):
            if c.other == "left robot":
                totals["left"] += abs(float(c.force))
            elif c.other == "right robot":
                totals["right"] += abs(float(c.force))
        return {k: round(v, 2) for k, v in totals.items()}

    def extra() -> dict:
        eng = m.tab_engagements()
        ch = m.corner_heights()
        return {
            "pitch_deg": round(m.pitch_deg(), 2),
            "tilt_deg": round(m.tilt_deg(), 2),
            "yaw_deg": round(m.yaw_deg(), 2),
            "tabs_mm": {k: round(1000.0 * v, 2) for k, v in eng.items()},
            "corners_mm": {k: round(1000.0 * v, 2) for k, v in ch.items()},
            "chassis_N": round(mk.contact_force("shroud", "chassis"), 2),
            "arm_contact_N": finger_forces(),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
            "zero": m.STATE.get("zero"),
        }

    folder = out / "full"
    record = ("--check" not in flags) and ("--no-video" not in flags)
    latest: dict = {}

    def director(_t, _arms, pictures) -> None:
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
        tail_s=4.5,
        sheet_frames=16,
        record=record,
    )
    rows = result["rows"]
    if "top_cam" in latest:
        _write_clean_png(folder / "end_top.png", latest["top_cam"])
        _write_clean_png(folder / "end_top_policy.png", kit.policy_picture(latest["top_cam"]))
    if record and (folder / "setup.png").is_file():
        import cv2

        bgr = cv2.imread(str(folder / "setup.png"))
        if bgr is not None:
            ok_enc, buf = cv2.imencode(".png", bgr)
            if ok_enc:
                (folder / "setup.png").write_bytes(bytes(buf))

    end_score = float(rows[-1]["info"].get("score", 0.0))
    first_half = next((r["t"] for r in rows if float(r["info"].get("score", 0.0)) >= 0.5), None)
    first_full = next((r["t"] for r in rows if float(r["info"].get("score", 0.0)) == 1.0), None)
    peak_arm_n = {
        s: round(max((r["extra"]["arm_contact_N"][s] for r in rows), default=0.0), 2)
        for s in ("left", "right")
    }
    readings = {
        "end_score": end_score,
        "steps": len(rows),
        "seconds": rows[-1]["t"],
        "first_0.5_s": first_half,
        "first_1.0_s": first_full,
        "peak_pitch_deg": round(max(r["extra"]["pitch_deg"] for r in rows), 2),
        "peak_arm_total_contact_N": peak_arm_n,
        "peak_per_pad_est_N": {s: round(peak_arm_n[s] / 2.0, 2) for s in ("left", "right")},
        "end_tabs_in_slot_mm": rows[-1]["extra"]["tabs_mm"],
        "end_corner_heights_mm": rows[-1]["extra"]["corners_mm"],
        "end_chassis_contact_N": rows[-1]["extra"]["chassis_N"],
        "end_hands_clear_mm": rows[-1]["extra"]["hands_clear_mm"],
        "both_arms_ready_at_end": {s: rows[-1][s] == "done" for s in ("left", "right")},
        "zero_foul": rows[-1]["extra"]["zero"],
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=2, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if end_score == 1.0 and readings["zero_foul"] is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
