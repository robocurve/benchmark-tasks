#!/usr/bin/env python3
"""Gate G2 simulator mechanics trials for ``mech/tasks/game_box_lid`` (frameless, ``record=False``).

    scripts/rta simpy <id> mech/tasks/game_box_lid/trials.py /workspace/robocurve/outputs/<id> [cases]

Default cases (sharing one simulator instance):
- ``off_centre_drop_seats``: carry ``box_lid`` level with a 1.4 mm diagonal offset and release it 30 mm above full
  seat: the free drop keeps the offset, so it lands ~1.5 mm off-centre -- and still scores 1.0, because the seated
  test is roof height only (every corner within 1.0 mm of CLOSED_H, tilt <= 2 deg) and the walls bound centring.
- ``tilted_wedge``: lower ``box_lid`` tilted 6.0 deg in roll (with -3.5 mm lateral offset so one skirt lip catches the
  1.8 mm base rim), release, and park at READY -> rests cocked ON the box, tilted ~9 deg, roof ~80 mm, pressing on
  the base: the half-credit state (0.5).
- ``tilted_hold``: lower ``box_lid`` tilted 6.0 deg in roll at 0.0 mm offset and hold at bottom of stroke with the
  hands still at the walls -> not hands-clear, not seated (0.0; skirt jams physically well above CLOSED_H).
- ``level_seat``: lower ``box_lid`` level (0.0 deg) -> seats cleanly onto ``box_base`` to CLOSED_H (score 1.0).
- ``side_shove``: shove ``box_lid`` horizontally along +x into ``box_base`` without lifting -> slides ``box_base`` > 25 mm
  off its corner marks and latches permanent 0.0 fault.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import traceback

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demo as D  # noqa: E402
import demo_kit as kit  # noqa: E402

ALL_CASES = ("off_centre_drop_seats", "tilted_wedge", "tilted_hold", "level_seat", "side_shove")


def side_shove_moves(m) -> dict[str, list[kit.Move]]:
    """Shortcut trial: left arm pushes ``box_lid`` horizontally along +x across the bench into ``box_base``."""
    from yam_arena import constants as YC, ik, station

    P = m.P
    top_z = m.TOP
    rot = kit.down(0.0)
    r_pos, r_rot = ik.tcp_pose(np.array(YC.READY, dtype=float), station.BASES["left"])
    x_start = P.LID_AT[0] - 0.5 * P.LID_OUT_W - 0.025
    x_push = P.BASE_AT[0] - 0.5 * P.BASE_W + 0.045
    z_push = top_z + 0.032
    return {
        "left": [
            kit.Move(1.6, (x_start, 0.0, top_z + 0.110), rot, 0.0, "over_front"),
            kit.Move(1.0, (x_start, 0.0, z_push), rot, 0.0, "down_behind_lid"),
            kit.Move(2.2, (x_push, 0.0, z_push), rot, 0.0, "shove_into_base"),
            kit.Move(1.2, (x_start, +0.18, top_z + 0.140), rot, 1.0, "retract"),
            kit.Move(1.6, r_pos, r_rot, 1.0, "ready"),
        ]
    }


def run_case(emb, case: str, out: Path) -> dict:
    m = emb.task.module
    spec = m.P.ACTIVE_BOX
    folder = out / case

    if case == "off_centre_drop_seats":
        # The seated test is roof height only: carry the lid level with a 1.4 mm diagonal offset, release it
        # 30 mm above full seat -- the free drop keeps the offset, it lands ~1.5 mm off-centre, and every roof
        # corner still sits within 1.0 mm of CLOSED_H at tilt ~0: full score, walls bound the centring.
        initial, director, trim = D.make_director(
            m, spec, roll_deg=0.0, target_offset_xy=(0.0014, 0.0014), seat_z_above_top=0.030, align_over_base=True, park_at_end=True
        )
        tail_s = 3.0
    elif case == "tilted_wedge":
        initial, director, trim = D.make_director(
            m, spec, roll_deg=6.0, target_offset_xy=(0.0, -0.0035), align_over_base=True, park_at_end=True
        )
        tail_s = 2.0
    elif case == "tilted_hold":
        initial, director, trim = D.make_director(
            m, spec, roll_deg=6.0, target_offset_xy=(0.0, 0.0), align_over_base=True, park_at_end=False
        )
        tail_s = 1.2
    elif case == "level_seat":
        initial, director, trim = D.make_director(
            m, spec, roll_deg=0.0, target_offset_xy=(0.0, 0.0), align_over_base=True, park_at_end=True
        )
        tail_s = 5.5
    elif case == "side_shove":
        initial, director, trim = side_shove_moves(m), None, None
        tail_s = 1.2
    else:
        raise ValueError(f"unknown case {case!r}; expected from {ALL_CASES}")

    res = kit.run(
        emb,
        initial,
        folder,
        seed=0,
        director=director,
        tail_s=tail_s,
        limit_s=m.P.RUN_LIMIT_S,
        sheet_frames=12,
        extra=lambda: D.step_telemetry(m),
        record=False,
    )
    rows = res["rows"]
    end_score = float(rows[-1]["info"].get("score", 0.0)) if rows else 0.0
    final_score = 1.0 if res["terminated"] and res["termination_reason"] == "success" else end_score
    seat_rows = [r for r in rows if r["left"] in ("seat", "hold_seat")]
    min_seat_roof_mm = min((r["extra"]["roof_h_mm"] for r in seat_rows), default=None)
    end_roof_mm = rows[-1]["extra"]["roof_h_mm"] if rows else None
    max_contact_n = max((r["extra"]["lid_base_force_N"] for r in rows), default=0.0)

    summary = {
        "case": case,
        "final_score": final_score,
        "end_info_score": end_score,
        "terminated": res["terminated"],
        "termination_reason": res["termination_reason"],
        "seconds": res["seconds"],
        "steps": res["steps"],
        "min_seat_roof_h_mm": min_seat_roof_mm,
        "end_roof_h_mm": end_roof_mm,
        "closed_h_mm": round(1000.0 * m.P.CLOSED_H, 2),
        "max_lid_base_force_N": round(max_contact_n, 3),
        "trim": None if trim is None else {k: v for k, v in trim.items() if k != "last_top"},
        "report": res["report"],
    }

    # Gate G2 invariants
    if case == "off_centre_drop_seats":
        # ~1.5 mm off-centre at full height is still the full score: seated = roof height only
        assert final_score == 1.0, summary
        rp = res["report"]
        assert rp["lid_offset_mm"] > 1.3 and rp["roof_corner_dev_mm"] <= 1.0, summary
    elif case == "tilted_wedge":
        # Released cocked lid rests ON the box: at rest, hands clear, pressing on the base, not seated -> 0.5
        assert final_score == 0.5, summary
        rp = res["report"]
        assert rp["all_at_rest"] is True and rp["grippers_clear"] is True, summary
        assert rp["lid_on_base_contact_N"] >= m.P.ON_BOX_MIN_CONTACT_N and rp["lid_tilt_deg"] <= 15.0, summary
        assert rp["eng_held_s"] is not None and rp["eng_held_s"] >= 1.0, summary
    elif case == "tilted_hold":
        assert final_score < 1.0, summary
        assert end_roof_mm is not None and end_roof_mm > 1000.0 * m.P.CLOSED_H + 5.0, summary
    elif case == "level_seat":
        assert final_score == 1.0, summary
        assert end_roof_mm is not None and abs(end_roof_mm - 1000.0 * m.P.CLOSED_H) <= 3.0, summary
    elif case == "side_shove":
        assert final_score == 0.0 and res["report"]["fault"] is not None, summary

    return summary


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args:
        raise SystemExit(f"usage: trials.py <out_dir> [case1,case2,...]  (cases: {ALL_CASES})")
    out = Path(args[0])
    cases = tuple(args[1].split(",")) if len(args) > 1 else ALL_CASES
    out.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    results = {}
    for case in cases:
        print(f"=== TRIAL {case} ===", flush=True)
        results[case] = run_case(emb, case, out)
        print("TRIAL_RESULT " + json.dumps(results[case]), flush=True)
    (out / "trials_summary.json").write_text(json.dumps(results, indent=2))
    print("ALL_TRIALS " + json.dumps(results), flush=True)
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    except BaseException:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    raise SystemExit(rc)
