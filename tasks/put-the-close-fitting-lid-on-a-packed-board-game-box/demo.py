#!/usr/bin/env python3
"""Scripted reference demonstration for ``mech/tasks/game_box_lid``.

    scripts/rta demo <id> mech/tasks/game_box_lid [--check] [--quick]

Both YAM arms close their fingers to 20 mm flat paddles, approach opposite outer short walls of ``box_lid`` at 32 deg
below horizontal (hand TCP separation scales with the preset lid, zero finger parts below z_lid_bottom + 16 mm or
inside the skirt), press inward with 10 N per arm via ``demo_kit``'s Jacobian spring offset, lift the lid to 72 mm
skirt clearance, carry it over ``box_base``, align the four corners from the observed pose above the rim, lower
straight down to seat over all four walls, relax the inward squeeze to 0 N in place before retracting outward along
+-y, and return both arms to ``READY`` while ``SUCCESS_HOLD_S = 3.5 s`` completes so the final frame shows the closed
box with both arms parked out of the way.

``--check``: run in the simulator without MP4 video encoding.
``--quick``: run the trajectory offline through ``demo_kit.dry_run`` and ``checks.run_offline`` (no simulator).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as Ck  # noqa: E402
import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402

SIDES = ("left", "right")
HOVER_Z = P.CARRY_CLEAR_Z  # 72.0 mm above bench (> 52.0 mm base rim + 20 mm clearance)
SEAT_CLEAR_Z = 0.0015  # m: lower to 1.5 mm above full seat, unclamp to 0 N in place, let gravity settle cleanly


def _lid_yaw_rel_base(bp: np.ndarray, br: np.ndarray, lp: np.ndarray, lr: np.ndarray) -> float:
    """Signed small yaw angle (rad) of ``box_lid`` relative to ``box_base`` about world +z."""
    rel = br.T @ lr
    # Choose the nearer of 0 and pi (rectangular lid symmetry)
    y_axis = rel[:2, 1]
    if y_axis[1] < 0.0:
        y_axis = -y_axis
    return math.atan2(float(y_axis[0]), float(y_axis[1]))


def make_director(
    m,
    spec: P.BoxSpec,
    *,
    squeeze_n: float = P.SQUEEZE_FORCE_N,
    roll_deg: float = 0.0,
    target_offset_xy: tuple[float, float] = (0.0, 0.0),
    seat_z_above_top: float | None = None,
    twist_yaw_at_seat_deg: float = 0.0,
    align_over_base: bool = True,
    park_at_end: bool = True,
):
    """Build initial pickup-lift-carry moves and a step director that appends precision alignment, seating, unclamp,
    outward release, and return to READY once the lid arrives above ``box_base``."""
    from yam_arena import constants as YC, ik, station

    top_z = m.TOP
    lx, ly = spec.lid_at
    bx, by = spec.base_at
    half_y = 0.5 * spec.lid_out_l
    z_touch = min(0.028, spec.lid_out_h - 0.008)
    roll = math.radians(roll_deg)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(roll), -math.sin(roll)], [0.0, math.sin(roll), math.cos(roll)]])

    geom = {s: Ck.grasp_geometry(spec, s) for s in SIDES}
    ready_pose = {s: ik.tcp_pose(np.array(YC.READY, dtype=float), station.BASES[s]) for s in SIDES}

    def wall_tcp(side: str, center_xyz: np.ndarray, lid_rot: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        g = geom[side]
        sgn = g["sgn"]
        hand_rot = lid_rot @ g["rot"]
        inward_world = lid_rot @ g["inward_world_from_tcp"]
        wall_pt = center_xyz + lid_rot @ np.array([0.0, sgn * half_y, z_touch])
        push_world = lid_rot @ np.array([0.0, -sgn * squeeze_n, 0.0])
        return wall_pt - inward_world, hand_rot, push_world

    hover_center = np.array([bx + target_offset_xy[0], by + target_offset_xy[1], top_z + HOVER_Z], dtype=float)

    initial: dict[str, list[kit.Move]] = {}
    for side in SIDES:
        g = geom[side]
        sgn, rot, ip, pat, grip = g["sgn"], g["rot"], g["inward_world_from_tcp"], g["push_at_local"], g["grip"]
        tcp_approach = np.array([lx, ly + sgn * (half_y + 0.038), top_z + z_touch + 0.055]) - ip
        tcp_align = np.array([lx, ly + sgn * (half_y + 0.038), top_z + z_touch]) - ip
        tcp_touch = np.array([lx, ly + sgn * half_y, top_z + z_touch]) - ip
        lift_tcp, lift_rot, lift_push = wall_tcp(side, np.array([lx, ly, top_z + HOVER_Z]), rx)
        carry_tcp, carry_rot, carry_push = wall_tcp(side, hover_center, rx)
        initial[side] = [
            kit.Move(1.8, tcp_approach, rot, grip, "approach"),
            kit.Move(1.1, tcp_align, rot, grip, "align"),
            kit.Move(1.0, tcp_touch, rot, grip, "touch"),
            kit.Move(0.8, None, None, grip, "squeeze", push=(0.0, -sgn * squeeze_n, 0.0), push_at=pat),
            kit.Move(1.9, lift_tcp, lift_rot, grip, "lift", push=tuple(lift_push)),
            kit.Move(2.4, carry_tcp, carry_rot, grip, "carry", push=tuple(carry_push)),
            kit.Move(0.5, None, None, grip, "settle_above", push=tuple(carry_push)),
        ]

    planned_stage = {"done": False, "trim_mm": [0.0, 0.0], "trim_yaw_deg": 0.0, "last_top": None}

    def director(t: float, arms: dict[str, kit.Arm], pictures) -> None:
        if pictures is not None and "top_cam" in pictures:
            planned_stage["last_top"] = pictures["top_cam"]
        if planned_stage["done"] or t < arms["left"].seconds - 0.06:
            return
        planned_stage["done"] = True

        # Read observed base & lid poses above the base (in dry_run when poses are empty, corrections are zero)
        dx, dy, dyaw = 0.0, 0.0, 0.0
        if align_over_base and m.mk.MECH.poses:
            bp, br = m.mk.pose("box_base")
            lp, lr = m.mk.pose("box_lid")
            want_xy = bp[:2] + (br[:2, :2] @ np.asarray(target_offset_xy, dtype=float))
            err_xy = want_xy - lp[:2]
            dx = float(np.clip(err_xy[0], -0.025, 0.025))
            dy = float(np.clip(err_xy[1], -0.025, 0.025))
            dyaw = float(np.clip(-_lid_yaw_rel_base(bp, br, lp, lr), -math.radians(8.0), math.radians(8.0)))
            planned_stage["trim_mm"] = [round(1000 * dx, 2), round(1000 * dy, 2)]
            planned_stage["trim_yaw_deg"] = round(math.degrees(dyaw), 2)

        cz = math.cos(dyaw)
        sz = math.sin(dyaw)
        rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        lid_rot = rz @ rx
        trimmed_hover = hover_center + np.array([dx, dy, 0.0])
        seat_dz = (SEAT_CLEAR_Z if roll_deg == 0.0 else 0.002) if seat_z_above_top is None else seat_z_above_top
        seat_z = top_z + seat_dz
        seat_center = np.array([trimmed_hover[0], trimmed_hover[1], seat_z])

        twist_rad = dyaw + math.radians(twist_yaw_at_seat_deg)
        ct, st = math.cos(twist_rad), math.sin(twist_rad)
        lid_rot_twisted = np.array([[ct, -st, 0.0], [st, ct, 0.0], [0.0, 0.0, 1.0]]) @ rx

        for side in SIDES:
            g = geom[side]
            sgn, grip = g["sgn"], g["grip"]
            trim_tcp, trim_rot, trim_push = wall_tcp(side, trimmed_hover, lid_rot)
            seat_tcp, seat_rot, seat_push = wall_tcp(side, seat_center, lid_rot)
            twist_tcp, twist_rot, twist_push = wall_tcp(side, seat_center, lid_rot_twisted)
            # Release along +-y at current height after relaxing push in place
            rel_center = np.array([trimmed_hover[0], trimmed_hover[1], top_z + max(0.006, seat_dz)])
            rel_wall = rel_center + np.array([0.0, sgn * (half_y + 0.052), z_touch])
            rel_tcp = rel_wall - g["inward_world_from_tcp"]
            up_tcp = np.array([trimmed_hover[0], by + sgn * (half_y + 0.058), top_z + 0.115]) - g["inward_world_from_tcp"]
            r_pos, r_rot = ready_pose[side]

            tail = [
                kit.Move(0.8, trim_tcp, trim_rot, grip, "trim", push=tuple(trim_push)),
                kit.Move(3.0, seat_tcp, seat_rot, grip, "seat", push=tuple(seat_push)),
            ]
            if abs(twist_yaw_at_seat_deg) > 1e-6:
                tail.append(kit.Move(0.9, twist_tcp, twist_rot, grip, "twist_bind", push=tuple(twist_push)))
            tail.append(kit.Move(0.5, None, None, grip, "hold_seat", push=tuple(twist_push if abs(twist_yaw_at_seat_deg) > 1e-6 else seat_push)))
            if park_at_end:
                tail.extend(
                    [
                        kit.Move(0.5, None, None, grip, "unclamp", push=(0.0, 0.0, 0.0)),
                        kit.Move(0.8, rel_tcp, g["rot"], grip, "release", push=(0.0, 0.0, 0.0)),
                        kit.Move(0.9, up_tcp, g["rot"], 1.0, "up", push=(0.0, 0.0, 0.0)),
                        kit.Move(1.6, r_pos, r_rot, 1.0, "ready", push=(0.0, 0.0, 0.0)),
                    ]
                )
            arms[side].moves.extend(tail)

    return initial, director, planned_stage


def step_telemetry(m) -> dict:
    """Per-step physical telemetry logged into ``result.json`` rows['extra']."""
    lm = m.lid_metrics()
    return {
        "roof_h_mm": round(1000.0 * lm["roof_h"], 2),
        "lid_tilt_deg": round(lm["tilt_deg"], 2),
        "lid_yaw_deg": round(lm["yaw_deg"], 2),
        "lid_offset_mm": round(1000.0 * lm["offset_m"], 2),
        "lid_base_force_N": round(m.mk.contact_force("box_lid", "box_base"), 3),
        "base_shift_mm": round(1000.0 * m.base_corner_shift_m(), 2),
        "base_tilt_deg": round(m.base_tilt_deg(), 2),
    }


def summarize_force_vs_travel(rows: list[dict]) -> list[dict]:
    """Compact force-vs-travel trace across the seating stroke for Check 51 / README reporting."""
    seating = [r for r in rows if r["left"] in ("trim", "seat", "hold_seat", "unclamp", "release", "ready")]
    if not seating:
        return []
    idxs = sorted({int(round(k * (len(seating) - 1) / min(9, max(1, len(seating) - 1)))) for k in range(min(10, len(seating)))})
    out = []
    for idx in idxs:
        r = seating[idx]
        ex = r.get("extra", {})
        fy_l = r["force"]["left"][1]
        fy_r = r["force"]["right"][1]
        out.append(
            {
                "t_s": r["t"],
                "stage": r["left"],
                "roof_h_mm": ex.get("roof_h_mm"),
                "squeeze_fy_N": [round(fy_l, 2), round(fy_r, 2)],
                "lid_base_force_N": ex.get("lid_base_force_N"),
                "score": r["info"].get("score"),
            }
        )
    return out


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    quick = "--quick" in args
    check = "--check" in args
    pos_args = [a for a in args if a not in ("--quick", "--check")]

    if quick:
        from yam_arena import station, task_file

        mod = task_file.load(HERE / "task.py").module
        bases = {s: np.array(station.BASES[s], dtype=float) for s in SIDES}
        initial, director, _ = make_director(mod, mod.P.ACTIVE_BOX)
        dry = kit.dry_run(initial, bases, director=director, tail_s=4.0, limit_s=mod.P.RUN_LIMIT_S)
        g1 = Ck.run_offline()
        mod.mk.MECH.clear()
        print(json.dumps({"dry_demo": dry, "presets": {k: v["ik_and_hulls"]["dry_run"] for k, v in g1.items()}}, indent=2))
        return 0

    if not pos_args:
        raise SystemExit("usage: demo.py <out_dir> [--check] [--quick]")
    out = Path(pos_args[0])
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    initial, director, trim_info = make_director(m, m.P.ACTIVE_BOX)
    folder = out / ("check" if check else "demo")
    res = kit.run(
        emb,
        initial,
        folder,
        seed=0,
        director=director,
        tail_s=5.5,
        limit_s=m.P.RUN_LIMIT_S,
        sheet_frames=12,
        extra=lambda: step_telemetry(m),
        record=not check,
    )
    rows = res["rows"]
    last_top = trim_info.pop("last_top", None)
    if last_top is not None:
        import cv2

        cv2.imwrite(str(out / "seated_top.png"), cv2.cvtColor(last_top, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out / "seated_top_policy.png"), cv2.cvtColor(kit.policy_picture(last_top), cv2.COLOR_RGB2BGR))
    end_score = float(rows[-1]["info"].get("score", 0.0)) if rows else 0.0
    final_score = 1.0 if res["terminated"] and res["termination_reason"] == "success" else end_score
    trace = summarize_force_vs_travel(rows)
    digest = {
        "final_score": final_score,
        "end_info_score": end_score,
        "terminated": res["terminated"],
        "termination_reason": res["termination_reason"],
        "steps": res["steps"],
        "seconds": res["seconds"],
        "final_arm_stage": [rows[-1]["left"], rows[-1]["right"]] if rows else None,
        "trim": trim_info,
        "worst_ik": res["worst_ik"],
        "report": res["report"],
        "force_vs_travel": trace,
    }
    (out / "demo_summary.json").write_text(json.dumps(digest, indent=2))
    print("DEMO_SUMMARY " + json.dumps(digest), flush=True)
    if final_score != 1.0:
        raise SystemExit(f"demo did not reach full score 1.0 (got {final_score})")
    if rows and (rows[-1]["left"] != "done" or rows[-1]["right"] != "done"):
        raise SystemExit(f"arms did not finish parking at READY before episode end: {rows[-1]['left']}, {rows[-1]['right']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
