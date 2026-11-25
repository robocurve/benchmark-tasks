#!/usr/bin/env python3
"""Scripted bimanual full-score reference demo for sink_stopper.

    scripts/rta demo <id> mech/tasks/sink_stopper [--check | --no-video] [--quick]

The right hand takes the rod by its clevis block, lifts it onto the port's axis and pushes it 16 mm in: the tip goes
through the stopper's hole and the ball seats. The left hand takes the pivot nut where it hangs on the rod, carries it
forward with the ball and sets it on the port's thread start; it aims from where the nut really is (twice, while it
hangs free) so that the nut comes down coaxial and square, and the thread catches there. The left hand lets go and
turns the nut on with its closed fingertips' upper edge, stroke by stroke down the nut's left side, until the thread
stops (snug). The right hand then presses the rod's rear end down about 20 deg, so the stopper lifts about 9 mm,
lets go (the nut's washer holds the rod), and both hands go back to ready. Nothing writes a pose.
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

OPEN_NUT = K.NUT_GRIP_OPEN / K.MAX_JAW
PINCH_NUT = K.NUT_PINCH / K.MAX_JAW
OPEN_CLEVIS = K.CLEVIS_GRIP_OPEN / K.MAX_JAW
PINCH_CLEVIS = K.CLEVIS_PINCH / K.MAX_JAW
NO_PUSH = (0.0, 0.0, 0.0)
FREE_GAP = 0.0035  # m: the nut held short of the thread start, clear of the ball, while the hand aims it
NEAR_GAP = 0.0020  # m: an aim 2 mm short of the thread start
TOUCH_GAP = 0.0006  # m: the last aim, the washer just short of the ball
SET_IN = 0.0020  # m: the set aims this far past the thread start; the washer meeting the ball stops the nut at it
SEAT_PUSH = (-5.0, 0.0, 0.0)  # N: the ball pushed onto its seat
PRESS_PUSH = (0.0, 0.0, -4.0)  # N: the rod's rear pressed down
STROKE_PUSH = 20.0  # N: the closed fingertips pressed onto the nut's side while stroking (one fingertip's edge bears it)
STROKES = 20
PRESS_LIFT, STROKE_LIFT, OUT_LIFT = 0.001, -0.001, 0.008  # m the fingertips' upper edge stands off the nut (negative: in)
TIP_AT = (0.0, 0.0, K.TIP_REACH)  # the closed fingertips' end, from the grasp point in the gripper's frame


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    w = K.poses()
    l0, ls, r0 = K.left_rot(K.LEFT_NUT), K.left_rot(K.LEFT_STROKE), K.right_rot()
    stroke_push = tuple(float(v) for v in STROKE_PUSH * ls[:, 2])
    pl_rdy, rl_rdy = ready("left")
    rl_roll, rl_mid = K.frame((0.0, 0.0, 1.0), rl_rdy[:, 2]), K.frame((0.0, 0.0, 1.0), (0.7, -0.5, -0.4))
    pl_mid = np.array([0.32, 0.20, K.TOP + 0.19])
    left = [
        Move(0.6, pos=pl_rdy, rot=rl_roll, grip=OPEN_NUT, label="roll wrist"),
        Move(0.7, pos=pl_mid, rot=rl_mid, label="under basin"),
        Move(0.7, pos=w["left: over nut"][1], rot=l0, label="over nut"),
        Move(1.0, pos=w["left: round nut"][1], label="round nut"),
        Move(0.6, grip=PINCH_NUT, label="pinch nut"),
        Move(0.8, pos=w["left: nut held"][1], label="nut to axis"),
        Move(1.8, label="hold nut"),
        Move(2.0, pos=K.nut_grasp(P.U_START + FREE_GAP), label="nut forward"),
        Move(0.6, label="hover nut"),
        Move(0.4, pos=K.nut_grasp(P.U_START + FREE_GAP), label="recentre nut"),
        Move(0.6, label="hover nut 2"),
        Move(1.0, pos=K.nut_grasp(P.U_START + NEAR_GAP), label="nut near start"),
        Move(0.5, label="hover near"),
        Move(0.6, pos=K.nut_grasp(P.U_START + TOUCH_GAP), label="nut to washer"),
        Move(0.4, label="hover touch"),
        Move(1.5, pos=K.nut_grasp(P.U_START - SET_IN), label="set nut on start"),
        Move(0.6, label="caught"),
        Move(0.5, grip=OPEN_NUT, label="let go of nut"),
        Move(0.8, pos=K.nut_grasp(P.U_START) - 0.035 * l0[:, 2], label="back off nut"),
        Move(0.4, grip=0.0, label="close fingers"),
        Move(0.6, pos=K.stroke_tcp(P.U_START, K.STROKE_PHI[0], lift=0.012), rot=ls, label="to strokes"),
    ]
    for k in range(STROKES):
        left += [
            Move(0.35, pos=K.stroke_tcp(P.U_START, K.STROKE_PHI[0], lift=PRESS_LIFT), push=stroke_push, push_at=TIP_AT, label=f"press {k}"),
            Move(0.8, pos=K.stroke_tcp(P.U_START, K.STROKE_PHI[1], lift=STROKE_LIFT), label=f"stroke {k}"),
            Move(0.25, pos=K.stroke_tcp(P.U_START, K.STROKE_PHI[1], lift=OUT_LIFT), push=NO_PUSH, label=f"lift {k}"),
            Move(0.45, pos=K.stroke_tcp(P.U_START, K.STROKE_PHI[0], lift=OUT_LIFT), label=f"return {k}"),
        ]
    left += [
        Move(1.0, pos=w["left: clear"][1], grip=OPEN_NUT, push=NO_PUSH, label="clear"),
        Move(1.0, pos=pl_mid, rot=rl_mid, label="out under basin"),
        Move(0.8, pos=pl_rdy, rot=rl_roll, label="out roll"),
        Move(0.8, rot=rl_rdy, grip=1.0, label="ready"),
    ]
    t_clear = sum(m.seconds for m in left[:-3])

    p_rdy, r_rdy = ready("right")
    r_roll, r_mid = K.frame((0.0, 0.0, 1.0), r_rdy[:, 2]), K.frame((0.0, 0.0, 1.0), (0.7, 0.5, -0.4))
    p_mid = np.array([0.32, -0.20, K.TOP + 0.19])
    right = [
        Move(0.8, pos=p_rdy, rot=r_roll, grip=OPEN_CLEVIS, label="roll wrist"),
        Move(1.0, pos=p_mid, rot=r_mid, label="under basin"),
        Move(1.2, pos=w["right: over clevis"][1], rot=r0, label="over clevis"),
        Move(1.0, pos=w["right: round clevis"][1], label="round clevis"),
        Move(0.6, grip=PINCH_CLEVIS, label="pinch clevis"),
        Move(0.8, pos=w["right: rod on axis"][1], label="rod to axis"),
        Move(0.8, label="hold rod up"),
        Move(2.0, pos=K.clevis_at(P.SEAT_OFFSET_X - 0.0005), push=SEAT_PUSH, label="push rod in"),
        Move(0.5, push=(-2.0, 0.0, 0.0), label="seated"),
    ]
    right.append(Move(round(t_clear - sum(m.seconds for m in right), 3), label="hold rod"))
    right += [
        Move(1.6, pos=w["right: rod pressed down"][1], rot=K.right_rot(K.PRESS_DEG), push=PRESS_PUSH, label="press rod down"),
        Move(0.5, label="settle open"),
        Move(0.4, push=NO_PUSH, label="ease off"),
        Move(0.5, grip=OPEN_CLEVIS, label="let go of rod"),
        Move(0.8, pos=w["right: back off"][1], label="back off"),
        Move(1.0, pos=p_mid, rot=r_mid, label="out under basin"),
        Move(0.8, pos=p_rdy, rot=r_roll, label="out roll"),
        Move(0.8, rot=r_rdy, grip=1.0, label="ready"),
    ]
    return {"left": left, "right": right}


def rot_to_x(axis) -> np.ndarray:
    """The smallest rotation that turns ``axis`` onto +x."""
    a = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    v = np.cross(a, [1.0, 0.0, 0.0])
    s, c = float(np.linalg.norm(v)), float(a[0])
    if s < 1e-12:
        return np.eye(3)
    k = v / s
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    th = math.atan2(s, c)
    return np.eye(3) + math.sin(th) * kx + (1.0 - math.cos(th)) * kx @ kx


def rot_y(rad: float) -> np.ndarray:
    return np.array([[math.cos(rad), 0.0, math.sin(rad)], [0.0, 1.0, 0.0], [-math.sin(rad), 0.0, math.cos(rad)]])


def save_end_picture(frame, folder: Path) -> None:
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
        ok = all(dry[s]["worst_ik_mm"] <= 0.5 and dry[s]["smallest_margin_deg"] >= 5.0 for s in ("left", "right"))
        return 0 if ok else 1

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk

    def finger_forces(body_name: str) -> dict[str, float]:
        """Total normal force (N) from each arm's fingers on ``body_name`` (a pinch: two pads; a stroke: one fingertip)."""
        totals: dict[str, float] = {}
        for c in mk.contacts(body_name):
            if c.other.endswith("robot"):
                totals[c.other] = round(totals.get(c.other, 0.0) + abs(float(c.force)), 2)
        return totals

    def extra() -> dict:
        return {
            "gap_mm": round(1000.0 * m.stopper_gap(), 2),
            "ball_seat_mm": round(1000.0 * m.ball_seat_offset(), 2),
            "rod_in": m.rod_in_port(),
            "pitch_deg": round(m.rod_pitch_deg(), 2),
            "engaged": m.thread_engaged(),
            "cw_deg": round(m.thread_cw_deg(), 1),
            "land_gap_mm": round(1000.0 * m.land_gap(), 3),
            "nut_off_mm": round(1000.0 * m.nut_xy_off(), 3),
            "nut_tilt_deg": round(m.nut_tilt_deg(), 3),
            "f_rod_drain_N": round(mk.contact_force("pivot_rod", "drain"), 2),
            "f_rod_stopper_N": round(mk.contact_force("pivot_rod", "stopper"), 2),
            "f_nut_rod_N": round(mk.contact_force("pivot_nut", "pivot_rod"), 2),
            "f_nut_drain_N": round(mk.contact_force("pivot_nut", "drain"), 2),
            "f_stopper_drain_N": round(mk.contact_force("stopper", "drain"), 2),
            "fingers_nut_N": finger_forces("pivot_nut"),
            "fingers_rod_N": finger_forces("pivot_rod"),
            "fingers_stopper_N": finger_forces("stopper"),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
            "speed_mm_s": {n: round(1000.0 * float(np.linalg.norm(mk.velocity(n)[0])), 2) for n in ("stopper", "pivot_rod", "pivot_nut")},
            "full": {"rod_in": m.rod_in_port(), "snug": m.nut_snug(), "open": m.stopper_open(), "rest": m.at_rest()},
        }

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:] and "--check" not in sys.argv[2:]
    latest: dict = {}
    left_by = {mv.label: mv for mv in moves["left"]}
    right_by = {mv.label: mv for mv in moves["right"]}
    done: set = set()
    notes: dict = {"aims": {}}
    shots: dict = {}
    l1 = K.left_rot(K.LEFT_STROKE)

    def start_of(moves_list, label: str) -> float:
        t0 = 0.0
        for mv in moves_list:
            if mv.label == label:
                return t0
            t0 += mv.seconds
        raise KeyError(label)

    def put(by: dict, label: str, pos, rot=None) -> None:
        by[label].pos = np.asarray(pos, dtype=float)
        if rot is not None:
            by[label].rot = np.asarray(rot, dtype=float)

    def aim_nut(t: float, hand, label: str, front_u: float, dz: float = 0.0) -> dict:
        """Aim from where the nut is: the world motion that takes the nut's front face onto the port's axis at
        ``front_u`` (``dz`` up), its axis along +x, moves the hand's present target too."""
        p_cmd, r_cmd = hand.state(t)[:2]
        p_nut, r_nut = mk.pose("pivot_nut")
        turn = rot_to_x(r_nut[:, 0])
        put(left_by, label, K.port(front_u, dz) + turn @ (p_cmd - p_nut), turn @ r_cmd)
        return {"t": round(t, 2), "off_mm": round(1000.0 * m.nut_xy_off(), 3), "tilt_deg": round(m.nut_tilt_deg(), 3),
                "land_gap_mm": round(1000.0 * m.land_gap(), 3)}

    def aim_rod(t: float, hand, label: str, ball_u: float, dz: float) -> dict:
        p_cmd, r_cmd = hand.state(t)[:2]
        p_rod, r_rod = mk.pose("pivot_rod")
        turn = rot_to_x(r_rod[:, 0])
        put(right_by, label, K.port(ball_u, dz) + turn @ (p_cmd - p_rod), turn @ r_cmd)
        return {"t": round(t, 2), "ball_mm": [round(1000.0 * float(v), 2) for v in (p_rod - K.port(ball_u, dz))]}

    def stroke_targets():
        """Press, stroke, lift and return points of one turning stroke, from where the nut is now."""
        p_nut, r_nut = mk.pose("pivot_nut")
        centre = p_nut + r_nut @ np.array([K.STROKE_AT, 0.0, 0.0])

        def at(phi_deg: float, lift: float) -> np.ndarray:
            phi = math.radians(phi_deg)
            return K.edge_to_tcp(centre + (P.NUT_OD / 2.0 + lift) * np.array([0.0, math.cos(phi), math.sin(phi)]), l1)

        top, bottom = K.STROKE_PHI
        return at(top, PRESS_LIFT), at(bottom, STROKE_LIFT), at(bottom, OUT_LIFT), at(top, OUT_LIFT)

    def director(t, arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)
        lh, rh = arms["left"], arms["right"]
        soon = lambda moves_list, label: t >= start_of(moves_list, label) - 1.5 / 20.0  # noqa: E731
        end_l = lambda label: start_of(lh.moves, label) + left_by[label].seconds  # noqa: E731
        end_r = lambda label: start_of(rh.moves, label) + right_by[label].seconds  # noqa: E731
        # Take the nut and the clevis where they really hang
        if "grasp" not in done and soon(lh.moves, "round nut"):
            done.add("grasp")
            p_nut, r_nut = mk.pose("pivot_nut")
            put(left_by, "round nut", p_nut + r_nut @ np.array([K.NUT_C, 0.0, 0.0]))
            p_rod, r_rod = mk.pose("pivot_rod")
            put(right_by, "round clevis", p_rod + r_rod @ np.array([P.CLEVIS_AT_X, 0.0, 0.0]) + np.array([0.0, K.CLEVIS_DY, 0.0]))
            notes["nut_start"] = {"front_u_mm": round(1000.0 * m.nut_front_u(), 2), "tilt_deg": round(m.nut_tilt_deg(), 2)}
        # The nut squared onto the axis where it is; the rod aimed onto the port's axis 1 mm low (the hole's middle)
        if "a1" not in done and t >= end_l("pinch nut") - 0.05:
            done.add("a1")
            notes["aims"]["a1"] = aim_nut(t, lh, "nut to axis", m.nut_front_u())
        if "a2" not in done and t >= end_l("nut to axis") + 0.3:
            done.add("a2")
            notes["aims"]["a2"] = aim_nut(t, lh, "hold nut", m.nut_front_u())
            ref = left_by["hold nut"]
            put(left_by, "nut forward", ref.pos + (P.U_START + FREE_GAP - m.nut_front_u()) * np.array([1.0, 0.0, 0.0]), ref.rot)
        if "r1" not in done and t >= end_r("rod to axis") + 0.3:
            done.add("r1")
            u_now = float(mk.pose("pivot_rod")[0][0] - P.X_DRAIN)
            notes["aims"]["r1"] = aim_rod(t, rh, "hold rod up", u_now, -0.001)
            ref = right_by["hold rod up"]
            put(right_by, "push rod in", ref.pos + (P.SEAT_OFFSET_X - 0.0005 - u_now) * np.array([1.0, 0.0, 0.0]), ref.rot)
        for key, hover, nxt, front in (("a3", "hover nut", "recentre nut", P.U_START + FREE_GAP),
                                       ("a4", "hover nut 2", "nut near start", P.U_START + NEAR_GAP),
                                       ("a5", "hover near", "nut to washer", P.U_START + TOUCH_GAP),
                                       ("a6", "hover touch", "set nut on start", P.U_START - SET_IN)):
            if key not in done and t >= end_l(hover) - 1.5 / 20.0:
                done.add(key)
                notes["aims"][key] = aim_nut(t, lh, nxt, front)
        if "catch" not in done and t >= end_l("caught") - 1.5 / 20.0:
            seen_set = m.land_gap() <= 0.0001 and m.nut_xy_off() <= P.CATCH_XY_MAX and m.nut_tilt_deg() <= P.CATCH_TILT_DEG
            if not seen_set and notes.get("waits", 0) < 6:
                notes["waits"] = notes.get("waits", 0) + 1
                left_by["caught"].seconds += 0.5
                right_by["hold rod"].seconds += 0.5
                notes["aims"][f"w{notes['waits']}"] = aim_nut(t, lh, "caught", P.U_START - SET_IN)
                return
            done.add("catch")
            notes["catch"] = m.STATE.get("catch")
            p_cmd, r_cmd = lh.state(t)[:2]
            put(left_by, "back off nut", p_cmd - 0.035 * r_cmd[:, 2])
        # The left wrist's view under the basin: backed off 35 mm from the nut it has just set on the thread start
        if "shot" not in done and t >= end_l("back off nut") and pictures is not None and "left_cam" in pictures:
            done.add("shot")
            shots["under_basin_left_wrist"] = np.asarray(pictures["left_cam"]).copy()
        # Turning strokes from where the nut is now
        for k in range(STROKES):
            key = f"s{k}"
            if key in done or not soon(lh.moves, f"press {k}"):
                continue
            done.add(key)
            a, b, c, d = stroke_targets()
            put(left_by, f"press {k}", a)
            put(left_by, f"stroke {k}", b)
            put(left_by, f"lift {k}", c)
            put(left_by, f"return {k}", d)
            notes.setdefault("strokes", []).append({"k": k, "t": round(t, 2), "nut_front_mm": round(1000.0 * m.nut_front_u(), 3),
                                                    "cw_deg": round(m.thread_cw_deg(), 1)})
        # The rod's rear pressed down about the ball's centre from where the hand is; one trim from the stopper's lift
        if "p1" not in done and soon(rh.moves, "press rod down"):
            done.add("p1")
            p_cmd, r_cmd = rh.state(t)[:2]
            ball = mk.pose("pivot_rod")[0]
            turn = rot_y(math.radians(K.PRESS_DEG))
            put(right_by, "press rod down", ball + turn @ (p_cmd - ball), turn @ r_cmd)
            notes["press_from"] = {"cw_deg": round(m.thread_cw_deg(), 1), "snug": m.nut_snug()}
        if "p2" not in done and t >= end_r("press rod down") - 0.05:
            done.add("p2")
            gap, pitch = m.stopper_gap(), m.rod_pitch_deg()
            extra_deg = min(max(K.PRESS_DEG + 0.5 - pitch, 0.0), 2.5)
            p_cmd, r_cmd = rh.state(t)[:2]
            ball = mk.pose("pivot_rod")[0]
            turn = rot_y(math.radians(extra_deg))
            put(right_by, "settle open", ball + turn @ (p_cmd - ball), turn @ r_cmd)
            notes["press_trim"] = {"gap_mm": round(1000.0 * gap, 2), "pitch_deg": round(pitch, 2), "extra_deg": round(extra_deg, 2)}

    result = kit.run(embodiment, moves, folder, seed=0, director=director, extra=extra,
                     limit_s=float(m.RUN_LIMIT_STEPS) / 20.0, tail_s=5.0, sheet_frames=16, record=record)
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)
    if "under_basin_left_wrist" in shots:
        import cv2

        bgr = cv2.cvtColor(shots["under_basin_left_wrist"], cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(folder / "under_basin_left_wrist.png"), bgr)
        cv2.imwrite(str(folder / "under_basin_left_wrist_policy.png"), cv2.resize(bgr, kit.POLICY_PICTURE, interpolation=cv2.INTER_AREA))

    def peak(key: str) -> float:
        return max((r["extra"][key] for r in rows), default=0.0)

    def peak_fingers(key: str) -> float:
        return max((max(r["extra"][key].values(), default=0.0) for r in rows), default=0.0)

    first = {lv: next((r["t"] for r in rows if r["info"].get("score", 0.0) >= lv), None) for lv in (0.25, 0.5, 1.0)}
    readings = {
        "score_at_end": rows[-1]["info"].get("score"),
        "seconds": rows[-1]["t"],
        "steps": len(rows),
        "first_at_s": first,
        "peaks_N": {k: peak(k) for k in ("f_rod_drain_N", "f_rod_stopper_N", "f_nut_rod_N", "f_nut_drain_N", "f_stopper_drain_N")},
        "peak_hand_total_N": {k: peak_fingers(k) for k in ("fingers_nut_N", "fingers_rod_N", "fingers_stopper_N")},
        "end": {k: rows[-1]["extra"][k] for k in ("gap_mm", "ball_seat_mm", "pitch_deg", "cw_deg", "hands_clear_mm", "hold")},
        "notes": notes,
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "trace": [{"t": r["t"], "L": r["left"], "R": r["right"], **{k: r["extra"][k] for k in ("gap_mm", "ball_seat_mm", "cw_deg", "land_gap_mm",
                   "nut_off_mm", "nut_tilt_deg", "f_nut_rod_N", "fingers_nut_N", "fingers_rod_N")}} for r in rows[::10]],
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps({k: v for k, v in readings.items() if k != "trace"}, default=str), flush=True)
    return 0 if readings["score_at_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
