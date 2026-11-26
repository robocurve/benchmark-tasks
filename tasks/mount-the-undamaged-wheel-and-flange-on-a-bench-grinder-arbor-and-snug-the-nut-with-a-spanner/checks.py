#!/usr/bin/env python3
"""Reach, joint-margin, grasp-height, collision-hull and frame-0 start checks for the bench-grinder wheel task.

1. Offline (no simulator): checks every waypoint pose of the reference plan against ``yam_arena.ik`` (position error
   <= 0.5 mm, orientation error <= 0.6 deg, every joint >= 5 deg inside its limits, every grasp height <= 150 mm over
   the bench), checks 3D hand-hull clearances against the bench, grinder, spindle, rack and cracked wheel, and checks
   that where both hands act together their grasp points are >= 200 mm apart and their collision hulls >= 20 mm apart.
2. ``--frame0 <dir>`` (simulator): renders ``start_top.png``, ``start_left_wrist.png``, ``start_right_wrist.png`` and
   their 320 x 176 policy-size counterparts, steps 2.0 s idle at READY, and verifies < 2 mm settle, score 0.0, zero None.

    scripts/rta simpy <id> mech/tasks/grinder_wheel/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.constants import READY
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION, HULL_SEP_MM, MAX_GRASP_Z = 0.5, 0.6, 5.0, 0.200, 20.0, 0.150


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation matrix: columns are (closing axis, pad width axis, approach axis)."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def ry(angle_rad: float) -> np.ndarray:
    return mk.axis_angle((0.0, 1.0, 0.0), angle_rad)


def rz(angle_rad: float) -> np.ndarray:
    return mk.axis_angle((0.0, 0.0, 1.0), angle_rad)


# --- Canonical grasps & poses shared with demo.py (all waypoints & grasps <= 150 mm) ---------------------------------
# 1. Sound wheel: pinched across its 16 mm faces 70 mm out from the bore, 48 deg up from the front, the approach
# pointing at the bore. Only the thin 10 mm fingertips then come inside the right guard plate's 80 mm radius: the
# fingers' 19-24 mm thick rear parts stay outside it, so the wheel can reach the inner flange (3.5 mm plate clearance).
WHEEL_GRASP_R, WHEEL_GRASP_UP = 0.070, math.radians(48.0)
WHEEL_APPROACH = np.array([math.sin(WHEEL_GRASP_UP), 0.0, -math.cos(WHEEL_GRASP_UP)])
WHEEL_ROT = frame((0.0, -1.0, 0.0), WHEEL_APPROACH)
WHEEL_GRIP_REL = -WHEEL_GRASP_R * WHEEL_APPROACH  # from the wheel's centre to the grasp point (z 46.8 mm over the bore)


def wheel_grip(wheel_centre, yaw_rad: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    R_y = rz(yaw_rad)
    return np.asarray(wheel_centre, dtype=float) + R_y @ WHEEL_GRIP_REL, R_y @ WHEEL_ROT


# 2. Outer flange & arbor nut: P_contact placed 9 mm deep inside the +12.8 mm finger pads for 100% solid pick & carry!
COAX_PLACE_ROT = frame((1.0, 0.0, 0.0), (0.38, 0.78, -0.497))
COAX_PICK_ROT = rz(-math.pi / 2.0) @ COAX_PLACE_ROT

FLANGE_CONTACT_PLACE = np.array([0.0, -0.0020, 0.0])  # centre of Ø 52 mm rim!
FLANGE_TCP_PLACE_REL = FLANGE_CONTACT_PLACE - 0.0035 * COAX_PLACE_ROT[:, 2]
FLANGE_TCP_PICK_REL = rz(-math.pi / 2.0) @ FLANGE_TCP_PLACE_REL

NUT_START_ROLL = math.radians(8.0)   # gripper roll (about +y) when the nut reaches the thread
NUT_SNUG_ROLL = math.radians(28.0)   # +20 deg finger turn: the catch takes 16 deg, then the thread spring stops the fingers,
                                    # so the nut ends near 27 deg with its flats parallel to the spanner slot at entry
NUT_PLACE_PRE_ROT = ry(NUT_START_ROLL) @ COAX_PLACE_ROT
NUT_PLACE_END_ROT = ry(NUT_SNUG_ROLL) @ COAX_PLACE_ROT
NUT_CONTACT_PLACE = np.array([0.0, -0.0015, 0.0])  # middle of 19 mm hex flats!
NUT_TCP_PRE_REL = ry(NUT_START_ROLL) @ NUT_CONTACT_PLACE - 0.0025 * NUT_PLACE_PRE_ROT[:, 2]
NUT_TCP_END_REL = ry(NUT_SNUG_ROLL) @ NUT_CONTACT_PLACE - 0.0025 * NUT_PLACE_END_ROT[:, 2]
NUT_TCP_PICK_REL = rz(-math.pi / 2.0) @ NUT_CONTACT_PLACE - 0.0025 * COAX_PICK_ROT[:, 2]


# 2b. End-on regrasp of the nut on the arbor: the hand points along the arbor (+y), the pads close on the Ø 28 mm round
# collar, the fingertips end 1.5 mm short of the collar face (their upper corners 0.6 mm), so nothing of the hand reaches past the nut toward the
# flange (the tilted carry grasp's finger corners stand ~8 mm past it). The round collar keeps the finger turn a pure
# friction couple (~1.4 N m at most: the spanner does the snug).
ENDON_APPROACH = np.array([0.0, math.cos(math.radians(4.0)), -math.sin(math.radians(4.0))])  # 4 deg down: joint margins
ENDON_ROT = frame((1.0, 0.0, 0.0), ENDON_APPROACH)
ENDON_TCP_REL = np.array([0.0, P.NUT_LEN / 2.0 - 0.0015, 0.0]) - C.PAD_TIP_PAST_TCP * ENDON_APPROACH  # from the nut's centre


def nut_endon_pose(nut_centre, roll_rad: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Hand pose holding the nut end-on at ``nut_centre``, turned ``roll_rad`` about the arbor axis (+y)."""
    R = ry(roll_rad)
    axis_pt = np.array([P.GRINDER_X, float(nut_centre[1]), TOP + P.ARBOR_Z])
    tcp = np.asarray(nut_centre, dtype=float) + ENDON_TCP_REL
    return R @ (tcp - axis_pt) + axis_pt, R @ ENDON_ROT


# 2c. End-on push that seats the flange: the hand points along the arbor (4 deg down), fingers 20 mm apart above and
# below it, and the fingertip ends press the flange's outer web face (r 10-20 mm, square to the arbor) home on the wheel.
PUSH_APPROACH = np.array([0.0, math.cos(math.radians(4.0)), -math.sin(math.radians(4.0))])
PUSH_ROT = frame((0.0, 0.0, 1.0), PUSH_APPROACH)
PUSH_OPEN = 0.020  # m between the pads: the Ø 12.7 mm arbor passes between the fingers


def flange_push_pose(face_y: float) -> tuple[np.ndarray, np.ndarray]:
    """Hand pose whose fingertip ends touch the plane y = ``face_y`` around the arbor axis."""
    tip = np.array([P.GRINDER_X, face_y, TOP + P.ARBOR_Z])
    return tip - C.PAD_TIP_PAST_TCP * PUSH_APPROACH, PUSH_ROT


# The spanner's 7 mm jaw slides on over the middle of the 11 mm hex (2 mm short of the Ø 28 mm collar), then the stroke
# presses it lightly against the collar
WRENCH_ON_NUT_DY = (P.NUT_HEX_LEN - P.NUT_LEN) / 2.0  # jaw centre from the nut centre along +y (the hex's middle)


def flange_pick_pose(back: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    c = np.array([P.FLANGE_START[0], P.FLANGE_START[1], TOP + P.FLANGE_START[2]])
    return c + FLANGE_TCP_PICK_REL - back * COAX_PICK_ROT[:, 2], COAX_PICK_ROT


def flange_place_pose(y_centre: float) -> tuple[np.ndarray, np.ndarray]:
    c = np.array([P.GRINDER_X, y_centre, TOP + P.ARBOR_Z])
    return c + FLANGE_TCP_PLACE_REL, COAX_PLACE_ROT


def nut_pick_pose(back: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    c = np.array([P.NUT_START[0], P.NUT_START[1], TOP + P.NUT_START[2]])
    return c + NUT_TCP_PICK_REL - back * COAX_PICK_ROT[:, 2], COAX_PICK_ROT


def nut_place_pose(y_centre: float, turned: bool = False) -> tuple[np.ndarray, np.ndarray]:
    rot = NUT_PLACE_END_ROT if turned else NUT_PLACE_PRE_ROT
    rel = NUT_TCP_END_REL if turned else NUT_TCP_PRE_REL
    c = np.array([P.GRINDER_X, y_centre, TOP + P.ARBOR_Z])
    return c + rel, rot


# 3. Left arm counter-hold on the installed +y K60 wheel (pinching 16 mm wheel at front-upper rim above tool rest)
# 62 mm in front of the axis: the 68 mm pads then end 15 mm short of the wheel guard's side plates (they start 8 mm behind
# the axis and reach +-18.5 mm from the wheel's faces, right where the finger bodies are)
LEFT_HOLD_ROT = frame((0.0, -1.0, 0.0), (0.35, -0.22, -0.910))
LEFT_HOLD_POS = np.array([P.GRINDER_X - 0.062, P.LEFT_WHEEL_Y, TOP + P.ARBOR_Z + 0.010])


# 4. Bare 7 mm-thick GEDORE 894 19 spanner: vertical x-z approach, closing along -y squarely across the 7 mm flat sides!
WRENCH_GRIP_R = 0.110  # m from the jaw centre along the handle: 46 mm behind the spanner's centre of mass (64 mm), so
                       # its weight twists the pinch by only 0.037 N m; 3.7 N m then takes a 34 N pull (pinch friction ~47 N)
WRENCH_BASE_ROT = frame((0.0, -1.0, 0.0), (0.58, 0.0, -0.8146))  # from above and behind, pads closing across the 7 mm faces
WRENCH_NUT_Y = P.Y_NUT_SNUG - 0.0048  # -0.0888 m: middle of the 11 mm hex, 2.2 mm clear of the Ø 28 mm collar
WRENCH_LIFT = 0.052     # m straight up out of the cradle: the 40 mm head clears the 16 mm fork lips
WRENCH_LANE_UP = 0.014  # m the level carry passes above the nut's centre line
WRENCH_BACK = 0.040     # m the jaw waits back along its slot axis before it slides onto the hex
WRENCH_OUT = 0.030      # m the jaw is drawn back out of the slot before the carry
ARC_SEGMENTS = 6
WRENCH_CARRY_PITCH = math.radians(-4.0)  # jaw 4 deg up while carried: what the hand droops under the spanner then
                                        # leaves it about level, and its jaw end clear of the cradle forks on the lift

# Handle pitch (0: horizontal): -18 deg at entry keeps the handle end 50 mm over the bench; +14 deg ends the 32 deg
# stroke with the grasp point 132 mm over the bench (every pose stays <= 150 mm).
WRENCH_ENTRY_PITCH = math.radians(-18.0)
WRENCH_PULL_END_PITCH = math.radians(14.0)

def wrench_pose_for_jaw(jaw_centre, handle_pitch_rad: float) -> tuple[np.ndarray, np.ndarray]:
    """(TCP world pos, gripper rot) when the spanner's jaw centre is at ``jaw_centre`` and its handle is pitched
    ``handle_pitch_rad`` (negative: handle end below the jaw)."""
    R_p = ry(handle_pitch_rad)
    rot = ry(handle_pitch_rad - P.WRENCH_REST_PITCH) @ WRENCH_BASE_ROT
    tcp = np.asarray(jaw_centre, dtype=float) + R_p @ np.array([-WRENCH_GRIP_R, 0.0, 0.0])
    return tcp, rot


def nut_jaw_centre() -> np.ndarray:
    return np.array([P.GRINDER_X, WRENCH_NUT_Y, TOP + P.ARBOR_Z])


def wrench_jaw_dir(handle_pitch_rad: float) -> np.ndarray:
    """Unit direction of the slot axis at ``handle_pitch_rad``: from the throat out through the jaw's mouth."""
    return ry(handle_pitch_rad + math.radians(P.WRENCH_JAW_DEG)) @ np.array([1.0, 0.0, 0.0])


WHEEL_PRE_Y = -0.125  # m: wheel centre waiting on the arbor line, its inner face 12 mm short of the domed tip
FLANGE_RELEASE_Y = -0.091  # m: where the carry grasp leaves the flange: its finger corners stop on the wheel 18 mm out
NUT_RELEASE_Y = -0.092  # m: where the carry grasp leaves the nut: its fingers stop on the seated flange 8 mm past the collar
NUT_FINGER_TURN = math.radians(20.0)  # end-on finger turn: the thread catches after 16 deg


def poses() -> dict:
    """Named waypoints: (side, tcp_pos, rot, jaw_opening_m)."""
    out = {}
    w0 = np.array([P.SOUND_START[0], P.SOUND_START[1], TOP + P.SOUND_START[2]])
    w_lift = np.array([P.SOUND_START[0], P.SOUND_START[1], TOP + P.ARBOR_Z])
    w_pre = np.array([P.GRINDER_X, WHEEL_PRE_Y, TOP + P.ARBOR_Z])
    w_seat = np.array([P.GRINDER_X, P.Y_WHEEL_SEATED, TOP + P.ARBOR_Z])

    wp_pick, wr_pick = wheel_grip(w0, P.SOUND_YAW)
    # Down onto the wheel along the approach axis, lift straight up, carry along the arbor's line, slide on along +y
    out["wheel: approach"] = ("right", wp_pick - 0.030 * WHEEL_APPROACH, wr_pick, 0.064)
    out["wheel: grasp"] = ("right", wp_pick, wr_pick, P.WHEEL_T)
    out["wheel: lift"] = ("right", *wheel_grip(w_lift, P.SOUND_YAW), P.WHEEL_T)
    out["wheel: before arbor"] = ("right", *wheel_grip(w_pre, 0.0), P.WHEEL_T)
    out["wheel: seated on arbor"] = ("right", *wheel_grip(w_seat, 0.0), P.WHEEL_T)
    out["wheel: release back"] = ("right", wheel_grip(w_seat, 0.0)[0] + np.array([-0.050, -0.010, 0.0]), WHEEL_ROT, 0.064)

    f_grasp, _ = flange_pick_pose(back=0.0)
    out["flange: approach"] = ("right", f_grasp - 0.042 * COAX_PICK_ROT[:, 2], COAX_PICK_ROT, 0.082)
    out["flange: grasp"] = ("right", *flange_pick_pose(back=0.0), P.FLANGE_D)
    out["flange: lift"] = ("right", f_grasp + np.array([0.0, 0.0, 0.055]), COAX_PICK_ROT, P.FLANGE_D)
    out["flange: oriented before arbor"] = ("right", *flange_place_pose(P.Y_ARBOR_TIP - 0.022), P.FLANGE_D)
    # Seat target at Y_FLANGE_SEATED - 0.002 so outer_flange (+4 mm) contacts wheel_sound while upper pad corner clears it!
    out["flange: seated recessed-inward"] = ("right", *flange_place_pose(P.Y_FLANGE_SEATED - 0.002), P.FLANGE_D)
    fp_seat, fr_seat = flange_place_pose(P.Y_FLANGE_SEATED - 0.002)
    out["flange: release back"] = ("right", fp_seat - 0.050 * fr_seat[:, 2], fr_seat, 0.082)

    f_out = FLANGE_RELEASE_Y - P.FLANGE_DEPTH / 2.0  # outer web face where the carry grasp leaves the flange
    out["flange: push start"] = ("right", flange_push_pose(f_out - 0.006)[0], PUSH_ROT, PUSH_OPEN)
    out["flange: pushed home"] = ("right", flange_push_pose(P.Y_FLANGE_SEATED - P.FLANGE_DEPTH / 2.0 + 0.0015)[0], PUSH_ROT, PUSH_OPEN)
    out["flange: push off"] = ("right", flange_push_pose(P.Y_FLANGE_SEATED - P.FLANGE_DEPTH / 2.0 - 0.012)[0], PUSH_ROT, PUSH_OPEN)
    # Out of the end-on poses: back off and up with the hand still level, then on to READY (a straight swing loses the IK)
    out["right: end-on away"] = ("right", np.array([P.GRINDER_X - 0.060, -0.150, TOP + 0.140]), frame((0.0, 0.0, 1.0), (0.45, 0.80, -0.40)), 0.050)

    out["nut: approach"] = ("right", *nut_pick_pose(back=0.045), 0.050)
    out["nut: grasp"] = ("right", *nut_pick_pose(back=0.0), P.NUT_AF)
    n_lift, _ = nut_pick_pose(back=0.0)
    out["nut: lift"] = ("right", n_lift + np.array([0.0, 0.0, 0.060]), COAX_PICK_ROT, P.NUT_AF)
    out["nut: before arbor"] = ("right", *nut_place_pose(P.Y_ARBOR_TIP - 0.026, turned=False), P.NUT_AF)
    out["nut: on thread start"] = ("right", *nut_place_pose(P.Y_NUT_SNUG, turned=False), P.NUT_AF)
    out["nut: finger-snugged"] = ("right", *nut_place_pose(P.Y_NUT_SNUG, turned=True), P.NUT_AF)
    np_end, nr_end = nut_place_pose(P.Y_NUT_SNUG, turned=True)
    out["nut: release back"] = ("right", np_end - 0.050 * nr_end[:, 2], nr_end, 0.050)

    n_rel = np.array([P.GRINDER_X, NUT_RELEASE_Y, TOP + P.ARBOR_Z])
    out["nut: end-on far"] = ("right", nut_endon_pose(n_rel)[0] - np.array([0.0, 0.014, 0.0]), ENDON_ROT, 0.060)
    out["nut: end-on back"] = ("right", nut_endon_pose(n_rel)[0] - np.array([0.0, 0.0065, 0.0]), ENDON_ROT, 0.050)
    out["nut: end-on grasp"] = ("right", *nut_endon_pose(n_rel), P.NUT_COLLAR_D)
    n_snug = np.array([P.GRINDER_X, P.Y_NUT_SNUG, TOP + P.ARBOR_Z])
    out["nut: end-on pressed"] = ("right", *nut_endon_pose(n_snug), P.NUT_COLLAR_D)
    out["nut: end-on turned"] = ("right", *nut_endon_pose(n_snug, NUT_FINGER_TURN), P.NUT_COLLAR_D)
    out["nut: end-on off"] = ("right", nut_endon_pose(n_snug, NUT_FINGER_TURN)[0] - np.array([0.0, 0.014, 0.0]), nut_endon_pose(n_snug, NUT_FINGER_TURN)[1], 0.050)

    out["left: approach +y wheel"] = ("left", LEFT_HOLD_POS + np.array([-0.042, 0.0, 0.022]), LEFT_HOLD_ROT, 0.060)
    out["left: counter-hold +y wheel"] = ("left", LEFT_HOLD_POS, LEFT_HOLD_ROT, P.WHEEL_T)

    w_cradle_jaw = np.array([P.WRENCH_START[0], P.WRENCH_START[1], TOP + P.WRENCH_START[2]])
    wp0, wr0 = wrench_pose_for_jaw(w_cradle_jaw, P.WRENCH_REST_PITCH)
    out["wrench: approach cradle"] = ("right", wp0 - 0.048 * wr0[:, 2], wr0, 0.070)
    out["wrench: grasp shank"] = ("right", wp0, wr0, P.WRENCH_T)
    out["wrench: eased up"] = ("right", *wrench_pose_for_jaw(w_cradle_jaw + np.array([0.0, 0.0, 0.012]), P.WRENCH_REST_PITCH), P.WRENCH_T)
    out["wrench: lift level"] = ("right", *wrench_pose_for_jaw(w_cradle_jaw + np.array([0.0, 0.0, WRENCH_LIFT]), WRENCH_CARRY_PITCH), P.WRENCH_T)
    nj = nut_jaw_centre()
    # Level carry to the nut's lane: the jaw waits 50 mm in front of (-x) the hex, handle horizontal
    w_lane = np.array([w_cradle_jaw[0], WRENCH_NUT_Y, TOP + P.ARBOR_Z + WRENCH_LANE_UP])
    out["wrench: over nut lane"] = ("right", *wrench_pose_for_jaw(w_lane, WRENCH_CARRY_PITCH), P.WRENCH_T)
    jdir_in = wrench_jaw_dir(WRENCH_ENTRY_PITCH)
    out["wrench: before hex flats"] = ("right", *wrench_pose_for_jaw(nj - WRENCH_BACK * jdir_in, WRENCH_ENTRY_PITCH), P.WRENCH_T)
    out["wrench: engaged"] = ("right", *wrench_pose_for_jaw(nj, WRENCH_ENTRY_PITCH), P.WRENCH_T)
    p_mid = 0.5 * (WRENCH_ENTRY_PITCH + WRENCH_PULL_END_PITCH)
    out["wrench: mid-pull"] = ("right", *wrench_pose_for_jaw(nj, p_mid), P.WRENCH_T)
    out["wrench: snugged"] = ("right", *wrench_pose_for_jaw(nj, WRENCH_PULL_END_PITCH), P.WRENCH_T)
    jdir_out = wrench_jaw_dir(WRENCH_PULL_END_PITCH)
    out["wrench: withdrawn off nut"] = ("right", *wrench_pose_for_jaw(nj - WRENCH_OUT * jdir_out, WRENCH_PULL_END_PITCH), P.WRENCH_T)
    out["wrench: back over lane"] = ("right", *wrench_pose_for_jaw(w_lane, WRENCH_CARRY_PITCH), P.WRENCH_T)
    out["wrench: place in cradle"] = ("right", *wrench_pose_for_jaw(w_cradle_jaw + np.array([0.0, 0.0, 0.002]), P.WRENCH_REST_PITCH), P.WRENCH_T)
    return out


TOGETHER = tuple(
    ("left: counter-hold +y wheel", r_step)
    for r_step in (
        "wheel: before arbor",
        "wheel: seated on arbor",
        "flange: oriented before arbor",
        "flange: seated recessed-inward",
        "nut: before arbor",
        "nut: on thread start",
        "nut: finger-snugged",
        "wrench: lift level",
        "wrench: over nut lane",
        "wrench: before hex flats",
        "wrench: engaged",
        "wrench: mid-pull",
        "wrench: snugged",
        "wrench: withdrawn off nut",
    )
)

def solve(side: str, pos, rot, start=None):
    best = None
    seeds = ([np.asarray(start, dtype=float)] if start is not None else []) + [
        np.array(s, dtype=float)
        for s in (
            READY,
            [0.0, 1.2, 1.2, -0.5, 0.0, 0.0],
            [0.0, 1.5, 1.5, -0.4, 0.0, 0.0],
            [0.6, 1.4, 1.2, -0.8, 0.5, 0.0],
            [-0.6, 1.4, 1.2, -0.8, -0.5, 0.0],
            [0.3, 1.0, 1.4, 0.4, 1.2, 0.0],
            [-0.3, 1.0, 1.4, 0.4, -1.2, 0.0],
            [0.2, 1.7, 1.1, -0.7, -0.8, -1.0],
        )
    ]
    for flip in (False, True):
        r = np.asarray(rot, dtype=float) @ (np.diag([-1.0, -1.0, 1.0]) if flip else np.eye(3))
        quat = ik.matrix_to_quat_xyzw(r)
        for q0 in seeds:
            q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=300)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG
            key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
            if best is None or key > best[0]:
                best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin, flip)
    assert best is not None
    return best[1], best[2], best[3], best[4], best[5]


def hand_hulls(side: str, q: np.ndarray, opening: float) -> list:
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    poses_w = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses_w[link][:3, :3], poses_w[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def box_hull(centre, rotation, size) -> analysis.Hull:
    centre, rotation, half = np.asarray(centre, float), np.asarray(rotation, float), np.asarray(size, float) / 2.0
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    vertices = corners @ rotation.T + centre
    planes = []
    for i in range(3):
        for s in (1.0, -1.0):
            n = s * rotation[:, i]
            planes.append([*n, float(n @ centre + half[i])])
    return analysis.Hull(vertices=vertices, planes=np.array(planes))


def gap(hulls_a, hulls_b) -> float:
    best = math.inf
    for _, a in hulls_a:
        for _, b in hulls_b:
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def static_obstacles(include_seated_wheel: bool = False) -> dict:
    gx, gy, az = P.GRINDER_X, P.GRINDER_Y, TOP + P.ARBOR_Z
    bench = [("bench", box_hull((0.35, 0.0, TOP - 0.020), np.eye(3), (0.80, 1.20, 0.040)))]
    grinder = [
        ("motor", box_hull((gx, gy, az), np.eye(3), (2 * P.MOTOR_R, P.MOTOR_LEN, 2 * P.MOTOR_R))),
        ("base", box_hull((gx, gy, TOP + P.BASE_SIZE[2] / 2.0), np.eye(3), P.BASE_SIZE)),
        ("left_guard_rear", box_hull((gx + 0.042, P.LEFT_WHEEL_Y, az + 0.080), np.eye(3), (0.085, 0.036, 0.010))),
        ("left_rest", box_hull((gx - P.WHEEL_R - P.REST_GAP - 0.013, P.LEFT_WHEEL_Y, az - 0.016), np.eye(3), (0.026, 0.030, 0.006))),
        ("right_guard_plate", box_hull((gx, P.Y_INNER_FACE + 0.016, az), np.eye(3), (0.160, 0.005, 0.160))),
    ]
    cracked = [
        (
            "wheel_cracked",
            box_hull(
                (P.CRACKED_START[0], P.CRACKED_START[1], TOP + P.CRACKED_START[2]),
                rz(P.CRACKED_YAW),
                (P.WHEEL_D, P.WHEEL_T, P.WHEEL_D),
            ),
        )
    ]
    out = {"bench": bench, "grinder": grinder, "cracked_wheel": cracked}
    if include_seated_wheel:
        # Inscribed octagon/cross of the round Ø 150 mm wheel (avoids 31 mm fake square corners outside r = 75 mm!)
        s45 = P.WHEEL_D * math.cos(math.pi / 8.0)
        out["seated_wheel"] = [
            ("wheel_sound_seated", box_hull((gx, P.Y_WHEEL_SEATED, az), np.eye(3), (s45, P.WHEEL_T, s45)))
        ]
    return out


def run_offline() -> dict:
    found, rows, ok = {}, {}, True
    for name, (side, pos, rot, opening) in poses().items():
        q, mm, deg, margin, flip = solve(side, pos, rot)
        found[name] = (side, q, opening)
        hulls = hand_hulls(side, q, opening)
        z_rel = float(pos[2] - TOP)
        mounted_wheel = name.startswith(("nut:", "wrench: before", "wrench: engaged", "wrench: mid", "wrench: snugged", "wrench: withdrawn"))
        obs = static_obstacles(include_seated_wheel=mounted_wheel and not name.startswith(("nut: approach", "nut: grasp", "nut: lift")))
        clear = {k: round(1000.0 * gap(hulls, v), 1) for k, v in obs.items()}
        good = (
            mm <= IK_MM
            and deg <= IK_DEG
            and margin >= MARGIN_DEG
            and z_rel <= MAX_GRASP_Z + 1e-6
            and min(clear.values()) > 0.0
        )
        ok &= good
        rows[name] = {
            "side": side,
            "at (m)": [round(float(v), 4) for v in pos],
            "z over bench (mm)": round(1000.0 * z_rel, 1),
            "IK (mm, deg)": [round(mm, 3), round(deg, 3)],
            "least joint margin (deg)": round(margin, 1),
            "hulls clear of (mm)": clear,
            "ok": good,
        }
    pairs = {}
    for a, b in TOGETHER:
        (sa, qa, oa), (sb, qb, ob) = found[a], found[b]
        tcp = float(np.linalg.norm(poses()[a][1] - poses()[b][1]))
        hull_gap = 1000.0 * gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob))
        good = tcp >= SEPARATION and hull_gap >= HULL_SEP_MM
        ok &= good
        pairs[f"{a} + {b}"] = {
            "grasp points apart (mm)": round(1000.0 * tcp, 1),
            "hulls apart (mm)": round(hull_gap, 1),
            "ok": good,
        }
    return {
        "pass": bool(ok),
        "worst_ik_mm": max(r["IK (mm, deg)"][0] for r in rows.values()),
        "smallest_margin_deg": min(r["least joint margin (deg)"] for r in rows.values()),
        "max_grasp_z_mm": max(r["z over bench (mm)"] for r in rows.values()),
        "min_two_hand_hull_mm": min(p["hulls apart (mm)"] for p in pairs.values()),
        "poses": rows,
        "both_hands_at_once": pairs,
    }


def run_frame0(out_dir: Path) -> dict:
    import cv2
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    obs = emb.reset(Scene(id="g1", instruction=m.INSTRUCTION), seed=0)
    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(obs.images[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out_dir / f"{stem}.png"), bgr)
        cv2.imwrite(str(out_dir / f"{stem}_policy.png"), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))
    ready = np.array([*READY, 1.0], dtype=float)
    action = Action(data=pack(ready, ready))
    res = None
    for _ in range(40):
        res = emb.step(action)
    assert res is not None
    moved = {
        name: round(1000.0 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
        for name in ("spindle", *m.FREE_PARTS)
    }
    out = {
        "moved from built after 2 s (mm)": moved,
        "score after 2 s": float(res.info.get("score", 0.0)),
        "report": m.report(),
        "pass": float(res.info.get("score", 0.0)) == 0.0 and m.STATE.get("zero") is None and max(moved.values()) < 3.0,
    }
    (out_dir / "g1_frame0.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main() -> int:
    if "--parts" in sys.argv:
        print(json.dumps(P.summary(), indent=1))
        return 0
    offline = run_offline()
    print("OFFLINE " + json.dumps({k: v for k, v in offline.items() if k not in ("poses", "both_hands_at_once")}), flush=True)
    if not offline["pass"]:
        for k, r in offline["poses"].items():
            if not r["ok"]:
                print("FAIL POSE:", k, json.dumps(r), flush=True)
        for k, r in offline["both_hands_at_once"].items():
            if not r["ok"]:
                print("FAIL PAIR:", k, json.dumps(r), flush=True)
        return 1
    if "--frame0" in sys.argv:
        idx = sys.argv.index("--frame0")
        out_dir = Path(sys.argv[idx + 1])
        (out_dir).mkdir(parents=True, exist_ok=True)
        (out_dir / "g1_offline.json").write_text(json.dumps(offline, indent=1))
        f0 = run_frame0(out_dir)
        print("FRAME0 " + json.dumps(f0, default=str), flush=True)
        return 0 if f0["pass"] else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
