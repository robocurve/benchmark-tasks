"""Scripted two-arm demo for the rail-plate-mount task, closed loop on the simulator's object poses.

    python demo.py --variant full          # both screws tight through the rail: 1.0
    python demo.py --variant one_tight     # both started, only the first tightened: 0.75
    python demo.py --variant no_tighten    # both started, neither tightened: 0.5
    python demo.py --variant one_started   # only the first started: 0.25
    python demo.py --variant all --no-video

It drives the ``yam_arena`` embodiment directly (20 Hz control, object_jitter 0, as a trial runs) and reads the
task's own scoring from each step's ``info``. Each arm follows Cartesian targets turned into joint targets by
yam_arena's IK, corrected from the measured rail, screw and screwdriver poses (the YAM wrist is soft: correct, settle,
measure).

1. right arm: grips the rail across its lips near its near end, lifts it a few mm, carries and turns it until a slot
   is over each hole, sets it down gently and lines it up again where it lies; then lets go and holds it down from
   above with its closed fingers in the channel (taking it again if a hole is more than 0.45 mm off its slot);
2. left arm, for each screw (the far hole first): takes it from the holder by its head, carries it upright over the
   hole with its fingers across the rail, centres the measured tip over the hole, sets it in and turns the hand 30
   degrees clockwise about the screw's axis, keeping it upright: the thread catches; lets go with the fingers opened
   just clear of the head. After the first screw the right arm moves back to its start pose;
3. left arm: takes the hex screwdriver from the holder from above, coaxially, and drives each screw home in wrist
   strokes, pressing down: between strokes it lifts the key out, turns back in the air and re-engages; it stops when
   the screw is tight (variants: once a screw is 1.5 turns in);
4. left arm lays the screwdriver down on the table and moves away; the run continues 3 s, hands off.

Writes ``result.json``, ``steps.jsonl`` and (unless ``--no-video``) one H.264 video per camera to ``--out``.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
INSTRUCTION = (
    "Fix the DIN rail to the mounting plate with the two screws: line the rail up so a slot is over each hole, hold "
    "it, put a screw through a slot into each hole and tighten both with the hex screwdriver."
)
VARIANTS = ("full", "one_tight", "no_tighten", "one_started")
CAMERAS = ("top_cam", "left_cam", "right_cam")
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
MAX_STEPS = 13000  # the task's run limit
MM = 1e-3

# -- gripper ----------------------------------------------------------------------------------------------------
PAD_TIP = 0.013  # pad tips beyond the TCP
OPEN_RAIL = 0.45  # ~42 mm, for the rail's 35 mm lips
GRIP_RAIL = 0.34  # ~32 mm: a gentle clamp on the 35 mm lips (closed fully, the two independently driven fingers store
# so much force that opening them kicked the rail 0.7 mm sideways and 0.2 degrees, run 10)
RAIL_TIPS_UP = 4.0 * MM  # gripping the rail: pad tips this far above the plate (the lips are 5.5-7.5 mm up)
RELEASE_RAIL = 0.8  # ~75 mm: opened wide before lifting off (at 42 mm a finger still dragged a lip and turned the
# rail 0.37 degrees, run 8)
HOLD_PRESS = 4.0 * MM  # held from above: the closed fingertips commanded this far into the rail's web
RAIL_LIFT = 6.0 * MM  # lined up in the air, lifted by this command (the soft wrist sags ~2.3 mm under the rail): on
# the plate, its friction made it stick and jump instead of following small corrections (demo run 4)
OPEN_HEAD = 0.22  # ~21 mm, for a screw's 8.5 mm head
RELEASE_HEAD = 0.13  # ~12.5 mm: letting go of a screw in the rail's channel, opened just clear of the 8.5 mm head (opened
# to 21 mm, the fingers pressed on the channel's sides and lifted the rail 1.8 mm and turned it 1.8 degrees, run 34)
HEAD_TIPS_UP = -2.0 * MM  # the pad tips this far above a screw's head's underside (below: they cover the whole head)
OPEN_DRIVER = 0.45  # ~42 mm, for the 26 mm handle
DRIVER_TOP_GRIP = 0.057  # the pad tips reach down to this far from the blade tip (the handle's upper part)

# -- lining up the rail --------------------------------------------------------------------------------------
LINE_UP_TOL = 0.25 * MM  # done when each hole is within this of its slot's centre line (the shank has 0.65 mm) ...
LINE_UP_ALONG = 3.0 * MM  # ... and of its slot's middle along the rail (the slot is 15 mm long)
LINE_UP_TRIES, LINE_UP_SETTLE = 8, 0.6
SET_DOWN_TOL, SET_DOWN_TRIES = 0.4 * MM, 4  # set down, each hole within this of its slot's centre line, else again
RAIL_TARGET_ALONG = -12.5 * MM
SET_DOWN_PRESS = 1.0 * MM  # set down: lowered by the rail's measured height above the plate plus this
HELD_TOL, HOLD_TRIES = 0.45 * MM, 3  # held down, each hole within this of its slot's centre line, else taken again
HOLD_Y = -0.105  # where along the rail the right hand holds it: near its -y end, beyond the far hole (held in the
# middle, the two wrists met above the rail while the left hand worked at the near hole, demo run 5)

# -- placing a screw -----------------------------------------------------------------------------------------
ABOVE_HOLE = 0.03  # carried with its tip this far above the hole ...
ABOVE_SLOT = 3.0 * MM  # ... then down to this far above it, centred again
CENTRE_TOL = 0.15 * MM
CENTRE_SETTLE = 0.5
UPRIGHT_TOL = math.radians(2.5)  # ... and the screw upright within this (the thread catches within 3)
JUST_ABOVE = 0.8 * MM  # then centred again with its tip this far above the hole, and set in
SEAT_TRIES = 3
START_TURN = math.radians(30.0)
START_SERVO, START_SERVO_MAX = 0.2, 1.5 * MM  # while turning: the hand follows the tip's offset from the hole  # turned this far clockwise once seated: the thread catches after ~20 degrees
START_PRESS = 0.5 * MM
START_TRIES, START_TILT_GAIN = 3, 0.15
CARRY_YAW_TOL_DEG = 0
LOST_MM, LOST_TILT = 40.0 * MM, math.radians(20.0)  # a screw this far off, or tilted this far, has been lost

# -- the screwdriver -----------------------------------------------------------------------------------------
HOVER = 8 * MM  # the key is centred on the socket this far above it, then lowered straight in
NEAR = 2 * MM
CENTRE_TRIES = 5
BLADE_CENTRE_TOL = 0.25 * MM
ENGAGE_TOL = 1.0 * MM  # engaged: the key's tip within this of the socket's floor (then 1.5 mm or more in it; the soft
# wrist stops it ~0.7 mm short of the floor, demo run 23)
INTO_SOCKET = 1.5 * MM  # lowered in: commanded this far below the socket's floor
SPIRAL_R, SPIRAL_TURNS, SPIRAL_S = 0.6 * MM, 2.0, 4.0
WIGGLE, WIGGLE_TURNS = math.radians(25.0), 3.0  # ... while turning the key to and fro this far
PRESS = 6.0 * MM  # while turning, the TCP is commanded this far below where the blade sits
PRESS_RAMP_S = 0.3
TIGHT_MARGIN_TURNS = 0.1  # the demo turns this far past the scorer's TIGHT_PAST
MAX_SLIPS = 20
ENGAGE_TRIES = 5  # re-engaging the key between strokes
MAX_STROKES = 30
SLIP_DROP = 0.6 * MM
CAM_OUT = 1.8 * MM  # slipped: the key's tip this far above the socket's floor (0.7 mm still in it; turning against the
# seated head, the key rides up ~0.7 mm and at 1.2 mm each stroke stopped after a few hundredths of a turn, run 36)
TURN_RATE = math.radians(120.0)
RATCHET_RATE = math.radians(120.0)
TURN_LEAD = math.radians(10.0)  # the hand turns at most this far ahead of the key (measured) ...
TURN_LEAD_SEATED = math.radians(35.0)  # ... and this far once the head is down (more torque from the soft wrist: at
# 10 degrees, 20 strokes turned it only 0.07 turns past contact, run 37)
STALL_S = 3.0  # a stroke ends after this long without turning the screw
WRIST_LIMIT = math.radians(95.0)
WRIST_JOINT_LIMIT = math.radians(120.0)
WRIST_MARGIN = math.radians(25.0)
DRIVER_PUT_DOWN = (0.18, 0.19)  # on the table between the plate and the holder (beside the holder it was out of reach,
# run 44)

SETTLE_TOL, SETTLE_MAX_S = 0.5 * MM, 2.0  # before gripping: the measured TCP within this of its command
STARTED_DRIVE_TURNS = 1.5  # variants: a "started" screw is driven this far into its thread with the key
PICK_TILT_MAX, PICK_TRIES = math.radians(10.0), 1  # (re-gripping made it worse: off, run 44)  # a screw picked up tilted more than this goes back for a new grip
AFTER_SUCCESS_S = 3.0
ABORT_EARLY = True
IK_ABORT_MM, IK_ABORT_S = 20.0, 2.0


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_pi_half(a: float) -> float:
    """An angle modulo pi, in [-pi/2, pi/2): a direction either way round."""
    return (a + math.pi / 2) % math.pi - math.pi / 2


def wrap_sixth(a: float) -> float:
    """An angle modulo 60 degrees, in [-30, 30): a hexagon key fits its socket six ways round."""
    return (a + math.pi / 6) % (math.pi / 3) - math.pi / 6


class Arm:
    """One arm's commanded TCP pose and gripper, turned into joint targets by IK (warm-started)."""

    def __init__(self, side: str, base, q0, ik, grip_rotation):
        self.side, self.base, self.ik = side, np.asarray(base, dtype=float), ik
        self.q = np.array(q0, dtype=float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.grip = 1.0
        self.down = grip_rotation(0.0)  # pointing down, fingers closing along world y
        self.ik_err = (0.0, 0.0)

    def action(self) -> np.ndarray:
        q, pe, re = self.ik.solve(self.pos, self.ik.matrix_to_quat_xyzw(self.rot), self.q, base_pos=self.base, iters=60)
        self.q, self.ik_err = q, (pe, re)
        return np.r_[q, self.grip]


def make_embodiment(video: bool, device: str = "cpu"):
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    size = (360, 640) if video else (64, 64)
    return YamArenaEmbodiment(task=str(TASK), control_hz=CONTROL_HZ, object_jitter=0.0, cam_height=size[0],
                              cam_width=size[1], device=device)


def set_rendering(emb, on: bool) -> None:
    emb._env.unwrapped.cfg.sim.render_interval = emb._env.unwrapped.cfg.decimation if on else 10**9


def run(variant: str, out: Path, video: bool, max_steps: int, emb=None) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack, split

    from yam_arena import ik, station
    from yam_arena.demo import grip_rotation

    emb = emb or make_embodiment(video)
    obs = emb.reset(Scene(id=f"rail-plate-{variant}", instruction=INSTRUCTION), seed=0)
    set_rendering(emb, video)
    task = emb.task.module
    rp = task.rp
    from yam_arena.scene import object_states  # patched by the task: runs the thread swap

    left_q, right_q = split(np.asarray(obs.state["joint_pos"]))
    arms = {
        "left": Arm("left", station.BASES["left"], left_q[:6], ik, grip_rotation),
        "right": Arm("right", station.BASES["right"], right_q[:6], ik, grip_rotation),
    }
    world = {"states": object_states(emb._env, emb.task.names), "obs": obs, "step": 0}
    flags: dict = {}
    report: dict = {"screws": {}}
    phase = {"left": "", "right": ""}

    # -- measurements -------------------------------------------------------------------------------------------
    def rail():
        return task.rail_pose(world["states"])

    def screw(name):
        """(tip, rotation) of a screw (whichever body is live)."""
        st = world["states"][name]
        r = task.quat_xyzw_to_matrix(np.asarray(st.quat_xyzw, dtype=float))
        return np.asarray(st.pos, dtype=float) - r @ np.array(rp.local_center("screw")), r

    def driver():
        """(blade tip, rotation) of the screwdriver."""
        st = world["states"]["driver"]
        r = task.quat_xyzw_to_matrix(np.asarray(st.quat_xyzw, dtype=float))
        return np.asarray(st.pos, dtype=float) - r @ np.array(rp.local_center("driver")), r

    def wrist_angle(side: str) -> float:
        left, right = split(np.asarray(world["obs"].state["joint_pos"]))
        return float((left if side == "left" else right)[5])

    # -- motions -----------------------------------------------------------------------------------------------
    def move(arm: Arm, pos, duration: float, grip=None, rot=None, label=""):
        phase[arm.side] = label
        p0, r0, g0 = arm.pos.copy(), arm.rot.copy(), arm.grip
        p1 = np.asarray(pos, dtype=float)
        r1 = r0 if rot is None else np.asarray(rot, dtype=float)
        g1 = g0 if grip is None else grip
        n = max(1, round(duration / DT))
        for i in range(1, n + 1):
            s = smooth(i / n)
            arm.pos = p0 + s * (p1 - p0)
            arm.rot = ik.slerp_matrix(r0, r1, s)
            arm.grip = g0 + s * (g1 - g0)
            yield

    def wait(arm: Arm, duration: float, label=""):
        phase[arm.side] = label
        for _ in range(max(1, round(duration / DT))):
            yield

    def tcp_measured(arm: Arm):
        left, right = split(np.asarray(world["obs"].state["joint_pos"]))
        return ik.tcp_pose((left if arm.side == "left" else right)[:6], arm.base)[0]

    def settle_at(arm: Arm, label="", tol=SETTLE_TOL, max_s=SETTLE_MAX_S):
        """Wait until the soft arm has caught up with its command (it lags 3-5 mm after a quick move: closing on a
        screw head then, the fingers knocked it 20 degrees over, run 39)."""
        phase[arm.side] = label
        for _ in range(round(max_s / DT)):
            if np.linalg.norm(tcp_measured(arm) - arm.pos) < tol:
                break
            yield

    def wait_for(arm: Arm, flag: str, label=""):
        phase[arm.side] = label
        while not flags.get(flag):
            yield

    def hand_for(r_target, p_target, offset):
        r_off, p_off = offset
        r_hand = r_target @ r_off.T
        return r_hand, p_target - r_hand @ p_off

    def offset_of(arm: Arm, pose):
        p_part, r_part = pose
        return arm.rot.T @ r_part, arm.rot.T @ (p_part - arm.pos)

    def best_wrist_yaw(arm: Arm, tcp_at, align: float, grasp_at=None) -> float:
        """The grasp yaw (of a straight-down hand) that, after a further turn by ``align``, leaves joint 6 nearest
        the end of its range that tightening (clockwise from above) turns away from."""
        best, best_score = 0.0, None
        q6_tight = None
        for k in range(24):
            yaw = -math.pi + k * math.pi / 12
            r = rot_z(yaw + align) @ arm.down
            q, pe, _ = ik.solve(np.asarray(tcp_at), ik.matrix_to_quat_xyzw(r), arm.q, base_pos=arm.base, iters=80)
            if pe > 2 * MM:
                continue
            if grasp_at is not None:
                _, pe_g, _ = ik.solve(np.asarray(grasp_at), ik.matrix_to_quat_xyzw(rot_z(yaw) @ arm.down), arm.q,
                                      base_pos=arm.base, iters=80)
                if pe_g > 2 * MM:
                    continue
            if q6_tight is None:
                q2, _, _ = ik.solve(np.asarray(tcp_at), ik.matrix_to_quat_xyzw(rot_z(-0.2) @ r), q, base_pos=arm.base,
                                    iters=80)
                q6_tight = 1.0 if q2[5] > q[5] else -1.0
            score = -abs(q[5] - (-q6_tight * (WRIST_JOINT_LIMIT - WRIST_MARGIN)))
            if best_score is None or score > best_score:
                best, best_score = yaw, score
        if q6_tight is None:
            report.setdefault("wrist_yaw_search", []).append({"tcp_at": [round(float(v), 4) for v in tcp_at],
                                                             "grasp_at": None if grasp_at is None else
                                                             [round(float(v), 4) for v in grasp_at], "found": False})
            if grasp_at is not None:
                return best_wrist_yaw(arm, tcp_at, align)
            q6_tight = 1.0
        report["wrist_q6_sign"] = q6_tight
        return best

    # -- the rail's target: a slot over each hole, square to the plate -------------------------------------------
    def rail_target():
        # with its origin 12.5 mm towards the arms from the plate's centre: the slots 12.5 mm in from the rail's far end
        # and 37.5 mm from its near end lie over the holes, and the right hand holds it near its near end, 17.5 mm
        # beyond the near hole
        return np.array(task.plate_point(RAIL_TARGET_ALONG)), rot_z(task.PLATE_YAW)

    # -- right arm: line the rail up and hold it -----------------------------------------------------------------
    def right_script():
        arm = arms["right"]
        home_pos, home_rot = arm.pos.copy(), arm.rot.copy()
        yield from wait(arm, 0.5, "start")

        def hole_offsets():
            p_m, r_m = rail()
            across, along = [], []
            for h in task.HOLES:
                local = r_m.T @ (np.array(h) - p_m)
                across.append(float(local[0]))
                along.append(min((float(local[1]) - y for y in rp.SLOTS), key=abs))
            return across, along, r_m

        def rail_height():
            p_m, _ = rail()
            return float(p_m[2] - task.PLATE_ORIGIN[2])

        def grip_rail(label):
            p_r, r_r = rail()
            yaw = math.atan2(r_r[1, 0], r_r[0, 0])
            grip_rot = rot_z(wrap_pi_half(yaw + math.pi / 2)) @ arm.down  # closing across the rail (the smaller turn)
            at = p_r + r_r @ np.array([0.0, HOLD_Y, 0.0])
            tcp = np.array([at[0], at[1], task.PLATE_ORIGIN[2] + RAIL_TIPS_UP + PAD_TIP])
            yield from move(arm, tcp + (0, 0, 0.05), 2.5, grip=OPEN_RAIL, rot=grip_rot, label=f"{label}: over the rail")
            yield from move(arm, tcp, 1.5, label=f"{label}: down round the rail")
            yield from move(arm, tcp, 0.8, grip=GRIP_RAIL, label=f"{label}: grip the rail")

        def line_up():
            for k in range(LINE_UP_TRIES):
                yield from wait(arm, LINE_UP_SETTLE, "let it settle")
                across, along, r_m = hole_offsets()
                report.setdefault("line_up", []).append([round(a / MM, 3) for a in across] + [round(a / MM, 2) for a in along])
                if max(abs(a) for a in across) < LINE_UP_TOL and max(abs(a) for a in along) < LINE_UP_ALONG:
                    break
                # move the hand so both holes come onto the slots' centre line (and the slots' middles over them)
                x_dir, y_dir = r_m[:, 0], r_m[:, 1]
                d_hole = np.array(task.HOLES[1]) - np.array(task.HOLES[0])
                turn = math.atan2(across[1] - across[0], float(np.linalg.norm(d_hole[:2])))
                shift = 0.5 * (across[0] + across[1]) * x_dir + 0.5 * (along[0] + along[1]) * y_dir
                mid = 0.5 * (np.array(task.HOLES[0]) + np.array(task.HOLES[1]))
                rz = rot_z(-turn)
                new_pos = mid + rz @ (arm.pos - mid) + shift * np.array([1, 1, 0])
                yield from move(arm, (new_pos[0], new_pos[1], arm.pos[2]), 0.6, rot=rz @ arm.rot, label="line the rail up")

        for hold_try in range(HOLD_TRIES):
            yield from grip_rail("rail" if hold_try == 0 else "grip the rail again")
            off = offset_of(arm, rail())
            p_t, r_t = rail_target()
            report["rail_target_along_mm"] = round(float((rot_z(task.PLATE_YAW).T @ (p_t - np.array(task.PLATE_ORIGIN)))[1]) / MM, 2)
            # lift it a little and carry it into line, then correct-settle-measure on the holes: each hole's offset from
            # its slot's centre line, across the rail (the slots are 15 mm long, so along the rail it only has to be
            # roughly right)
            r_h, p_h = hand_for(r_t, p_t, off)
            yield from move(arm, arm.pos + (0, 0, RAIL_LIFT), 0.8, label="lift the rail")
            yield from move(arm, (p_h[0], p_h[1], arm.pos[2]), 3.0, rot=r_h, label="carry the rail into line")
            for attempt in range(SET_DOWN_TRIES):
                yield from line_up()
                across, along, r_m = hole_offsets()
                report.setdefault("rail_lined_up", []).append({"across_mm": [round(a / MM, 3) for a in across],
                                                              "along_mm": [round(a / MM, 2) for a in along]})
                # set it down gently: only as far as it hangs above the plate, plus a little (lowered the whole lift,
                # the wrist pressed the gripped rail onto the plate and it jumped ~1 mm sideways in the fingers, runs
                # 27-29)
                drop = max(0.0, rail_height()) + SET_DOWN_PRESS
                yield from move(arm, arm.pos - (0, 0, drop), 1.0, label="set it down")
                yield from wait(arm, 1.0, "hold it, settle")
                # it jumps ~1 mm across as it touches down (runs 27-30): line it up again where it lies, gripped (it
                # weighs 95 g and is barely pressed: it slides)
                yield from line_up()
                across, along, r_m = hole_offsets()
                report.setdefault("rail_set_down", []).append({"across_mm": [round(a / MM, 3) for a in across],
                                                              "along_mm": [round(a / MM, 2) for a in along],
                                                              "drop_mm": round(drop / MM, 2)})
                if max(abs(a) for a in across) < SET_DOWN_TOL:
                    break
                yield from move(arm, arm.pos + (0, 0, drop), 0.8, label="lift it again")
            # let go and hold it from above instead: closed fingers pressed onto the web, in the channel (clamped across
            # its lips, the rail floats between the YAM's two independently driven fingers and crept 1 mm sideways, run 7)
            yield from move(arm, arm.pos, 0.6, grip=RELEASE_RAIL, label="let go of the rail")
            yield from move(arm, arm.pos + (0, 0, 0.012), 0.6, label="up a little")
            yield from move(arm, arm.pos, 0.5, grip=0.0, label="close the fingers")
            p_r, r_r = rail()
            web = p_r + r_r @ np.array([0.0, HOLD_Y, rp.WEB])
            tcp = web + np.array([0.0, 0.0, PAD_TIP])
            yield from move(arm, tcp + (0, 0, 0.004), 0.8, label="down into the channel")
            yield from move(arm, tcp - (0, 0, HOLD_PRESS), 0.8, label="press the rail down")
            yield from wait(arm, 0.5, "hold the rail, settle")
            across, along, r_m = hole_offsets()
            report.setdefault("rail_pressed", []).append([round(a / MM, 3) for a in across])
            if max(abs(a) for a in across) < HELD_TOL:
                break
            if hold_try < HOLD_TRIES - 1:  # too far off for a screw to pass: take it again
                yield from move(arm, arm.pos + (0, 0, HOLD_PRESS + 0.03), 0.8, label="up, the rail is off line")
        across, along, r_m = hole_offsets()
        report["rail_held_at"] = {"across_mm": [round(a / MM, 3) for a in across], "along_mm": [round(a / MM, 2) for a in along]}
        flags["rail_held"] = True
        # until the first screw has caught: from then on it pins the rail (through its slot), and the hand would be
        # in the way at the other hole
        yield from wait_for(arm, "first_started", "hold the rail")
        yield from move(arm, arm.pos + (0, 0, HOLD_PRESS + 0.06), 1.2, label="up off the rail")
        # back to where it started, out of the left arm's way (parked 10 cm beside the near hole, the arms met while
        # the left one carried the second screw there, demo run 22)
        yield from move(arm, home_pos, 2.5, rot=home_rot, label="away")
        flags["right_clear"] = True
        phase["right"] = "done"
        while True:
            yield

    # -- left arm: screws, then the screwdriver --------------------------------------------------------------------
    def place_screw(name, hole_idx):
        arm = arms["left"]
        hole = np.array(task.HOLES[hole_idx])
        tip, r = screw(name)
        hand = rot_z(math.pi / 2) @ arm.down  # closing along the holder (x): the pads' outer faces (11 mm from the
        # screw) clear the next screw's head (15 mm) and the screwdriver's handle (16 mm)
        tcp = tip + rp.SCREW_LEN * r[:, 2] + (0, 0, HEAD_TIPS_UP + PAD_TIP)
        yield from move(arm, tcp + (0, 0, 0.05), 2.0, grip=OPEN_HEAD, rot=hand, label=f"{name}: over it")
        for pick_try in range(PICK_TRIES):
            yield from move(arm, tcp, 1.2, label=f"{name}: down round the head")
            yield from settle_at(arm, f"{name}: settle round the head")
            yield from move(arm, tcp, 0.6, grip=0.0, label=f"{name}: grip the head")
            yield from move(arm, tcp + (0, 0, 0.04), 1.0, label=f"{name}: lift it out")
            yield from wait(arm, 0.3, f"{name}: settle")
            tip, r = screw(name)
            if math.acos(min(1.0, r[2, 2])) < PICK_TILT_MAX or pick_try == PICK_TRIES - 1:
                break
            # held badly askew (20 degrees, run 39): put it back in the holder and take it again (up to ~6 degrees
            # is usual and the upright correction takes it out; re-gripping those knocked them over, run 43)
            yield from move(arm, tcp + (0, 0, 0.5 * MM), 1.0, label=f"{name}: back into the holder")
            yield from move(arm, arm.pos, 0.5, grip=OPEN_HEAD, label=f"{name}: let go, take it again")
            yield from move(arm, tcp + (0, 0, 0.02), 0.6, label=f"{name}: up")
            yield from wait(arm, 0.5, f"{name}: let it settle")
            tip, r = screw(name)
            tcp = tip + rp.SCREW_LEN * r[:, 2] + (0, 0, HEAD_TIPS_UP + PAD_TIP)
        report["screws"].setdefault(name, {})["pick_tries"] = pick_try + 1
        if tip[2] < task.HOLDER_ORIGIN[2] + 0.02:
            report["abort"] = f"{name}: not picked up"
            while True:
                phase["left"] = f"{name}: not picked up"
                yield
        off = offset_of(arm, (tip, r))
        # an upright screw looks the same at any turn about its axis: carry it at the turn the arm reaches best at the
        # hole (lowest IK error there)
        # (lowest IK error there), checked along the whole carry with chained IK as the arm will follow it (checked at
        # the end only, the wrist flipped half way and threw the screw, demo runs 19 and 20)
        # Only turns that keep the fingers closing across the rail (within CARRY_YAW_TOL, either way round): turned
        # 30 degrees, the 27 mm wide fingers sat on the rail's edges at hole A and the hand couldn't go down, run 23.
        yaw0 = math.atan2(r[1, 0], r[0, 0])
        close = arm.rot @ arm.down.T @ np.array([0.0, 1.0, 0.0])
        _, r_rail = rail()
        d0 = wrap_pi_half(math.atan2(r_rail[1, 0], r_rail[0, 0]) - math.atan2(close[1], close[0]))
        best = None
        for k, d in enumerate(d0 + math.radians(v) + w for w in (0.0, math.pi)
                              for v in range(-CARRY_YAW_TOL_DEG, CARRY_YAW_TOL_DEG + 1, 5)):
            d = wrap(d)
            cand = rot_z(yaw0 + d)
            r_c, p_c = hand_for(cand, hole + (0, 0, ABOVE_HOLE), off)
            q, worst = arm.q.copy(), 0.0
            for j in range(1, 11):
                sj = j / 10
                p_j = arm.pos + sj * (p_c - arm.pos)
                p_j[2] = max(p_c[2], arm.pos[2]) if j < 10 else p_c[2]
                q, pe, re = ik.solve(p_j, ik.matrix_to_quat_xyzw(ik.slerp_matrix(arm.rot, r_c, sj)), q,
                                     base_pos=arm.base, iters=60)
                worst = max(worst, pe + 0.01 * re)
            r_s, p_s = hand_for(cand, hole + (0, 0, ABOVE_SLOT), off)
            _, pe, re = ik.solve(p_s, ik.matrix_to_quat_xyzw(r_s), q, base_pos=arm.base, iters=80)
            err = max(worst, pe + 0.01 * re) + 1e-4 * abs(d)  # (the least turn among equals)
            if best is None or err < best[0]:
                best = (err, cand, d)
        upright = best[1]
        report["screws"].setdefault(name, {})["carry_turn_deg"] = round(math.degrees(best[2]), 1)
        rec0 = report["screws"].setdefault(name, {})
        rec0["carry_ik_mm"] = round(best[0] / MM, 3)
        r_h, p_h = hand_for(upright, hole + (0, 0, ABOVE_HOLE), off)
        yield from move(arm, (p_h[0], p_h[1], max(p_h[2], arm.pos[2])), 2.0, rot=r_h, label=f"{name}: carry")
        yield from move(arm, p_h, 0.8, label=f"{name}: over the hole")

        def centre(height, tries, tag, upright=True):
            """Correct-settle-measure: the tip over the hole, and (``upright``) the screw upright (it pivots a little in
            the fingers, which hold only its 3 mm head): the hand turns about the tip. Ends settled after a correction
            (a correction still under way, the soft wrist drifted the tip 1.8 mm onto the web on the way down, run 11)."""
            err = np.zeros(2)
            for i in range(tries + 1):
                yield from wait(arm, CENTRE_SETTLE, f"{name}: {tag}")
                tip, r = screw(name)
                err = (hole - tip)[:2]
                axis = r[:, 2]
                tilt = math.acos(min(1.0, axis[2]))
                if np.linalg.norm(err) > LOST_MM or tilt > LOST_TILT:  # dropped or turned over in the fingers: stop
                    report["abort"] = f"{name}: lost from the fingers ({tag})"  # (chasing it, the hand knocked the
                    while True:  # screwdriver off the holder, run 20)
                        phase["left"] = f"{name}: lost"
                        yield
                if i == tries or (np.linalg.norm(err) < CENTRE_TOL and (tilt < UPRIGHT_TOL or not upright)):
                    break
                if not upright:
                    tilt = 0.0
                k = np.cross(axis, (0.0, 0.0, 1.0))
                kn = np.linalg.norm(k)
                r_c = np.eye(3)
                if kn > 1e-9:
                    k = k / kn
                    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                    r_c = np.eye(3) + math.sin(tilt) * kx + (1 - math.cos(tilt)) * kx @ kx
                new_pos = tip + r_c @ (arm.pos - tip) + (err[0], err[1], 0.0)
                yield from move(arm, new_pos, 0.4, rot=r_c @ arm.rot, label=f"{name}: {tag}")
            return float(np.linalg.norm(err))

        e1 = yield from centre(ABOVE_HOLE, 3, "centre high")
        yield from move(arm, arm.pos - (0, 0, ABOVE_HOLE - ABOVE_SLOT), 1.5, label=f"{name}: down through the slot")
        e2 = yield from centre(ABOVE_SLOT, 6, "centre over the hole")
        e3 = 0.0
        for attempt in range(SEAT_TRIES):
            # the last few mm in two steps, centring again just above the hole (the soft arm drifts ~1 mm on the way)
            tip, _ = screw(name)
            yield from move(arm, arm.pos - (0, 0, tip[2] - hole[2] - JUST_ABOVE), 0.8, label=f"{name}: just above the hole")
            e3 = yield from centre(JUST_ABOVE, 4, "centre just above the hole", upright=False)
            tip, _ = screw(name)
            yield from move(arm, arm.pos - (0, 0, tip[2] - hole[2] + rp.SEAT_DEPTH + START_PRESS), 0.8, label=f"{name}: set it in")
            yield from wait(arm, 0.4, f"{name}: settle in the hole")
            tip, r = screw(name)
            if tip[2] < hole[2] - rp.ENGAGE_MIN_DEPTH:  # its tip in the countersink: it will catch when turned
                break
            # on the edge, not in the hole: back up and centre again
            yield from move(arm, arm.pos + (0, 0, JUST_ABOVE + 1.5 * MM), 0.6, label=f"{name}: up, not in the hole")
        tip, r = screw(name)
        rec = report["screws"].setdefault(name, {})
        rec.update(centre_err_mm=[round(e1 / MM, 3), round(e2 / MM, 3), round(e3 / MM, 3)], seat_tries=attempt + 1,
                   seated_mm=[round(float(v) / MM, 2) for v in tip - hole],
                   tilt_deg=round(math.degrees(math.acos(min(1.0, r[2, 2]))), 2))
        p0, r0 = arm.pos.copy(), arm.rot.copy()
        about = np.array([tip[0], tip[1], p0[2]])
        n = round(2.0 / DT)
        for start_try in range(START_TRIES):
            if start_try:  # not caught: turn the hand back and try again
                yield from move(arm, p0, 1.0, rot=r0, label=f"{name}: turn back, try again")
            servo = np.zeros(3)
            r_tilt = np.eye(3)
            for i in range(1, n + 1):
                rz = rot_z(-START_TURN * smooth(i / n))
                tip, r_s = screw(name)
                # keep the tip over the hole while turning (the soft wrist doesn't turn exactly about the screw's axis)
                servo[:2] = np.clip(servo[:2] + START_SERVO * (hole - tip)[:2], -START_SERVO_MAX, START_SERVO_MAX)
                # ... and the screw upright: tip the hand back about the screw's tip by part of its tilt (turning, the
                # wrist tilted it from 1.8 to 4.8 degrees and the thread didn't catch, run 40)
                k = np.cross(r_s[:, 2], (0.0, 0.0, 1.0))
                kn = float(np.linalg.norm(k))
                if kn > 1e-6:
                    k = k / kn
                    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                    ang = START_TILT_GAIN * math.asin(min(1.0, kn))
                    r_tilt = (np.eye(3) + math.sin(ang) * kx + (1 - math.cos(ang)) * kx @ kx) @ r_tilt
                pos = about + rz @ (p0 - about) + servo
                arm.rot, arm.pos = r_tilt @ rz @ r0, tip + r_tilt @ (pos - tip)
                phase["left"] = f"{name}: start the thread"
                yield
                if world["states"][name].engaged:
                    break
            if world["states"][name].engaged:
                break
        rec_start = report["screws"].setdefault(name, {})
        rec_start["start_tries"] = start_try + 1
        rec["caught"] = bool(world["states"][name].engaged)
        yield from move(arm, arm.pos, 0.5, grip=RELEASE_HEAD, label=f"{name}: let go")
        yield from move(arm, arm.pos + (0, 0, 0.04), 0.8, label=f"{name}: up")

    def socket(name):
        """(the socket's floor at its centre, the socket's angle) of a screw."""
        tip, r = screw(name)
        bottom = tip + (rp.SCREW_LEN + rp.HEAD_H - rp.SOCKET_DEPTH) * r[:, 2]
        d = r[:, 0]
        return bottom, math.atan2(d[1], d[0])

    def blade_angle() -> float:
        _, r = driver()
        return math.atan2(r[1, 0], r[0, 0])

    def tcp_for_tip(target_tip):
        tip, _ = driver()
        return arms["left"].pos + (np.asarray(target_tip) - tip)

    def engage(name, label):
        """Turn the key to the socket's angle, centre it and lower it in (if it rests on the socket's rim: a small
        spiral while turning it to and fro, until it drops in)."""
        arm = arms["left"]
        rec = report["screws"][name]
        bottom, ang = socket(name)
        sign = report.get("wrist_q6_sign", 1.0)
        room = sign * wrist_angle("left") + (WRIST_JOINT_LIMIT - math.radians(10.0))
        base_turn = wrap_sixth(ang - blade_angle())
        want = sign * wrist_angle("left") + (WRIST_JOINT_LIMIT - WRIST_MARGIN)
        fits = [base_turn + k * math.pi / 3 for k in range(-6, 7) if base_turn + k * math.pi / 3 <= room]
        phi = min(fits, key=lambda p: abs(p - want)) if fits else base_turn
        if driver()[0][2] > bottom[2] + 0.015:
            yield from move(arm, tcp_for_tip(bottom + (0, 0, 0.03)), 1.5, label=f"{label}: over the screw")
            for _ in range(round(0.5 / DT)):
                arm.pos = arm.pos + np.clip(0.5 * ((socket(name)[0] + (0, 0, 0.03)) - driver()[0]), -0.002, 0.002)
                yield
        r_from, p_from = arm.rot.copy(), arm.pos.copy()
        p_to = tcp_for_tip(socket(name)[0] + (0, 0, HOVER))
        n = max(1, round(max(1.0, abs(phi) / RATCHET_RATE) / DT))
        for i in range(1, n + 1):
            s = smooth(i / n)
            arm.rot = rot_z(phi * s) @ r_from
            arm.pos = p_from + s * (p_to - p_from)
            phase["left"] = f"{label}: over the socket"
            yield

        def centre(height, tries, tag):
            phase["left"] = f"{label}: {tag}"
            err = np.zeros(3)
            for _ in range(tries):
                for _ in range(round(CENTRE_SETTLE / DT)):
                    yield
                errs = []
                for _ in range(4):
                    bottom, ang = socket(name)
                    errs.append((bottom + (0, 0, height)) - driver()[0])
                    yield
                err = np.mean(errs, axis=0)
                if np.linalg.norm(err[:2]) < BLADE_CENTRE_TOL:
                    break
                arm.pos = arm.pos + np.clip(err, -0.003, 0.003)
                arm.rot = rot_z(wrap_sixth(ang - blade_angle())) @ arm.rot
            rec.setdefault("blade_centre_err_mm", []).append(round(float(np.linalg.norm(err[:2])) / MM, 3))

        yield from centre(HOVER, CENTRE_TRIES, "centre on the socket")
        yield from move(arm, arm.pos - np.array([0.0, 0.0, HOVER - NEAR]), 1.0, label=f"{label}: down to the head")
        yield from centre(NEAR, 2, "centre again near the head")
        yield from move(arm, arm.pos - np.array([0.0, 0.0, NEAR + INTO_SOCKET]), 0.8, label=f"{label}: into the socket")
        ok = abs(driver()[0][2] - socket(name)[0][2]) < ENGAGE_TOL
        if not ok:
            centre_xy, z_cmd = arm.pos[:2].copy(), arm.pos[2]
            r_spiral = arm.rot.copy()
            n = round(SPIRAL_S / DT)
            for i in range(1, n + 1):
                r_ = SPIRAL_R * i / n
                a_ = 2 * math.pi * SPIRAL_TURNS * i / n
                arm.pos = np.array([centre_xy[0] + r_ * math.cos(a_), centre_xy[1] + r_ * math.sin(a_), z_cmd])
                arm.rot = rot_z(WIGGLE * math.sin(2 * math.pi * WIGGLE_TURNS * i / n)) @ r_spiral
                phase["left"] = f"{label}: find the socket"
                yield
                if abs(driver()[0][2] - socket(name)[0][2]) < ENGAGE_TOL:
                    ok = True
                    break
            rec.setdefault("spiral_hits", []).append(ok)
        rec["engaged"] = ok

    def tighten(name, until_turns=None):
        """Drive a started screw home in wrist strokes; stop when it is tight (or, with ``until_turns``, once it has
        turned that far into its thread: the variants' "started" screws)."""
        arm = arms["left"]
        rec = report["screws"][name]
        rec.update(strokes=0, slips=0)
        for attempt in range(3):
            if attempt:
                yield from move(arm, arm.pos + (0, 0, 0.004), 0.3, label="lift for another try")
            yield from engage(name, f"{name}: engage")
            if rec.get("engaged"):
                break
        if not rec.get("engaged"):
            rec["gave_up"] = "the key doesn't go into the socket"
            return
        while rec["strokes"] < MAX_STROKES:
            rec["strokes"] += 1
            r0 = arm.rot.copy()
            base = arm.pos.copy()
            n_ramp = round(PRESS_RAMP_S / DT)
            for i in range(1, n_ramp + 1):
                arm.pos = base - (0, 0, smooth(i / n_ramp) * PRESS)
                phase["left"] = f"{name}: stroke {rec['strokes']} (press)"
                yield
            turned, last_meas, meas_total = 0.0, blade_angle(), 0.0
            best_meas, stall_steps = 0.0, 0
            phase["left"] = f"{name}: stroke {rec['strokes']}"
            slipped = False
            while True:
                st = world["states"][name]
                if until_turns is None and st.tight() and st.past_contact >= rp.TIGHT_PAST + TIGHT_MARGIN_TURNS:
                    rec["tight_step"] = world["step"]
                    break
                if until_turns is not None and st.turns >= until_turns:
                    rec["tight_step"] = world["step"]  # (here: driven as far as asked)
                    rec["driven_to_turns"] = round(float(st.turns), 3)
                    break
                a_ = blade_angle()
                meas_total += wrap(a_ - last_meas)
                last_meas = a_
                lead = TURN_LEAD_SEATED if st.past_contact > 0.0 or st.clamp_force >= rp.CONTACT_CLAMP else TURN_LEAD
                turned = max(turned - TURN_RATE * DT, meas_total - lead)
                # a stroke that has stopped turning the screw ends (the wrist can't push it further)
                if meas_total < best_meas - math.radians(1.0):
                    best_meas, stall_steps = meas_total, 0
                else:
                    stall_steps += 1
                    if stall_steps > round(STALL_S / DT):
                        break
                arm.rot = rot_z(turned) @ r0
                arm.pos = base - (0, 0, PRESS)
                bottom, _ = socket(name)
                tip, _ = driver()
                if tip[2] < bottom[2] - SLIP_DROP or tip[2] > bottom[2] + CAM_OUT:
                    slipped = True
                    break
                if report.get("wrist_q6_sign", 1.0) * wrist_angle("left") > WRIST_LIMIT or -turned > math.radians(200):
                    break
                yield
            if "tight_step" in rec:
                break
            if slipped:
                rec["slips"] += 1
                if rec["slips"] >= MAX_SLIPS:
                    rec["gave_up"] = f"slipped out of the socket {MAX_SLIPS} times"
                    break
            yield from move(arm, base + (0, 0, 0.006), 0.4, label=f"{name}: lift out of the socket")
            rec["engaged"] = False
            for attempt in range(ENGAGE_TRIES):
                yield from engage(name, f"{name}: next stroke")
                if rec.get("engaged"):
                    break
                yield from move(arm, arm.pos + (0, 0, 0.004), 0.3, label="lift for another try")
            if not rec.get("engaged"):
                rec["gave_up"] = "the key doesn't go back into the socket"
                break
        yield from wait(arm, 0.3, f"{name}: hold")
        yield from move(arm, arm.pos + (0, 0, PRESS + 0.03), 0.8, label=f"{name}: lift the screwdriver out")

    def left_script():
        arm = arms["left"]
        yield from wait_for(arm, "rail_held", "wait for the rail")
        # the hole nearer the holder first; screw_2 (nearer the plate) into it
        order = [("screw_2", 1), ("screw_1", 0)]
        if variant == "one_started":
            order = order[:1]
        for i, (name, hole_idx) in enumerate(order):
            if i:
                yield from wait_for(arm, "right_clear", "wait for the right hand to move away")
            yield from place_screw(name, hole_idx)
            report["screws"][name]["started_step"] = world["step"]
            flags["first_started"] = True
        # what the screwdriver does to each placed screw: None = tighten, a number = drive it this many turns in
        # (a screw counts as started from 1 turn)
        names = [n for n, _ in order]
        if variant == "full":
            plan = [(n, None) for n in names]
        elif variant == "one_tight":
            plan = [(names[0], None)] + [(n, STARTED_DRIVE_TURNS) for n in names[1:]]
        else:  # no_tighten, one_started
            plan = [(n, STARTED_DRIVE_TURNS) for n in names]
        to_tighten = plan
        if to_tighten:
            # the screwdriver, from above, coaxially; the yaw that leaves the wrist room for long strokes at the first
            name0 = to_tighten[0][0]
            tip, rd = driver()
            axis = rd[:, 2]
            bottom, ang = socket(name0)
            align = wrap_sixth(ang - math.atan2(rd[1, 0], rd[0, 0]))
            screw_at = bottom + np.array([0.0, 0.0, DRIVER_TOP_GRIP + PAD_TIP])
            tips_at = tip + DRIVER_TOP_GRIP * axis
            yaw = best_wrist_yaw(arm, screw_at, align, grasp_at=tips_at + (0, 0, PAD_TIP))
            top_down = rot_z(yaw) @ arm.down
            tcp = tips_at - PAD_TIP * top_down[:, 2]
            yield from move(arm, tcp + (0, 0, 0.05), 2.5, grip=OPEN_DRIVER, rot=top_down, label="over the screwdriver")
            tip, rd = driver()
            tcp = tip + DRIVER_TOP_GRIP * rd[:, 2] - PAD_TIP * top_down[:, 2]
            yield from move(arm, tcp, 1.2, label="down round the handle")
            yield from move(arm, tcp, 0.8, grip=0.0, label="grip the handle")
            yield from move(arm, tcp + (0, 0, 0.045), 1.2, label="lift it out of the holder")
            for name, until in to_tighten:
                yield from tighten(name, until)
                if "gave_up" in report["screws"][name]:
                    report["abort"] = f"{name}: {report['screws'][name]['gave_up']}"
                    break
            yield from move(arm, np.array([DRIVER_PUT_DOWN[0], DRIVER_PUT_DOWN[1], arm.pos[2]]), 2.0,
                            label="carry the screwdriver away")
            yield from move(arm, arm.pos - np.array([0.0, 0.0, arm.pos[2] - task.TOP - 0.11]), 1.0,
                            label="down to the table")
            yield from move(arm, arm.pos, 0.5, grip=OPEN_DRIVER, label="let go of the screwdriver")
        yield from move(arm, arm.pos + (0, 0, 0.06), 1.0, label="up")
        yield from move(arm, arm.pos + (-0.05, 0.08, 0.0), 1.5, label="away")
        flags["screws_done"] = True
        report["script_done_step"] = world["step"]
        phase["left"] = "done"
        while True:
            yield

    scripts = {"left": left_script(), "right": right_script()}
    frames = {c: [] for c in CAMERAS}
    log = []
    result = None
    success_step = None
    t0 = time.time()
    for step in range(1, max_steps + 1):
        world["step"] = step
        try:
            for side in ("left", "right"):
                next(scripts[side])
        except Exception as exc:  # keep what the run produced
            import traceback

            report["crash"] = f"step {step}: {type(exc).__name__}: {exc}"
            report["crash_trace"] = traceback.format_exc()[-1500:]
            break
        res = emb.step(Action(data=pack(arms["left"].action(), arms["right"].action())))
        world["obs"] = res.observation
        world["states"] = object_states(emb._env, emb.task.names)
        st = world["states"]
        p_r, r_r = rail()
        row = {"step": step, "left": phase["left"], "right": phase["right"], "score": res.info.get("score"),
               "success": res.info.get("success"),
               "rail_mm": [round(float(v) / MM, 2) for v in (p_r - np.array(task.PLATE_ORIGIN))],
               "rail_yaw_deg": round(math.degrees(math.atan2(r_r[1, 0], r_r[0, 0])), 3),
               "l_ik_mm": round(arms["left"].ik_err[0] / MM, 2), "r_ik_mm": round(arms["right"].ik_err[0] / MM, 2),
               "driver_tip": [round(float(v), 4) for v in driver()[0]],
               "l_cmd": [round(float(v), 4) for v in arms["left"].pos],
               "l_close_vs_rail_deg": round(math.degrees(wrap_pi_half(
                   math.atan2(*(lambda c: (c[1], c[0]))(ik.tcp_pose(split(np.asarray(world["obs"].state["joint_pos"]))[0][:6],
                                                                    arms["left"].base)[1] @ arms["left"].down.T @ np.array([0.0, 1.0, 0.0])))
                   - math.atan2(r_r[1, 0], r_r[0, 0]))), 1),
               "l_meas": [round(float(v), 4) for v in ik.tcp_pose(split(np.asarray(world["obs"].state["joint_pos"]))[0][:6],
                                                                 arms["left"].base)[0]],
               "r_cmd": [round(float(v), 4) for v in arms["right"].pos],
               "r_meas": [round(float(v), 4) for v in ik.tcp_pose(split(np.asarray(world["obs"].state["joint_pos"]))[1][:6],
                                                                 arms["right"].base)[0]]}
        for n in task.SCREWS:
            s = st[n]
            row[n] = {"engaged": bool(s.engaged), "turns": round(float(s.turns), 3), "clamp_N": round(float(s.clamp_force), 1),
                      "past": round(float(s.past_contact), 3), "slot": task.through_slot(st, n),
                      "yaw_deg": round(math.degrees(math.atan2(screw(n)[1][1, 0], screw(n)[1][0, 0])), 2),
                      "tilt_deg": round(math.degrees(math.acos(min(1.0, screw(n)[1][2, 2]))), 2),
                      "turned_deg": round(math.degrees(rp._TRACK[n].turned), 1) if n in rp._TRACK else None,
                      "tip": [round(float(v), 4) for v in screw(n)[0]]}
        log.append(row)
        if video:
            for c in CAMERAS:
                frames[c].append(res.observation.images[c])
        if "abort" not in report:
            worst_ik = max(row["l_ik_mm"], row["r_ik_mm"])
            report["_ik_bad"] = report.get("_ik_bad", 0) + 1 if worst_ik > IK_ABORT_MM else 0
            if report["_ik_bad"] >= round(IK_ABORT_S / DT):
                report["abort"] = f"step {step}: an arm can't reach its target ({worst_ik:.0f} mm) for {IK_ABORT_S} s"
        result = res
        if "abort" in report and ABORT_EARLY:
            break
        if res.terminated and success_step is None:
            success_step = step
            report["success_step"] = step
        if success_step is not None and step - success_step >= round(AFTER_SUCCESS_S / DT):
            break
        if success_step is None and "script_done_step" in report and step - report["script_done_step"] >= round(
                AFTER_SUCCESS_S / DT):
            break
    st = world["states"]
    success = bool(success_step is not None)
    summary = {
        "variant": variant,
        "task_score": 1.0 if success else float(result.info.get("score", 0.0)),
        "success_step": success_step,
        "steps": len(log),
        "run_limit_steps": MAX_STEPS,
        "screws": {n: {"engaged": bool(st[n].engaged), "turns": float(st[n].turns), "clamp_N": float(st[n].clamp_force),
                       "past_contact": float(st[n].past_contact), "tight": bool(st[n].tight()),
                       "through_slot": task.through_slot(st, n), "credit": task.screw_credit(st, n)} for n in task.SCREWS},
        "rail_flat": task.rail_flat(st),
        "report": report,
        "wall_s": time.time() - t0,
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "result.json").write_text(json.dumps(summary, indent=2, default=float))
    with open(out / "steps.jsonl", "w") as f:
        for row in log:
            f.write(json.dumps(row, default=float) + "\n")
    if video:
        write_videos(frames, out, variant)
        import cv2

        cv2.imwrite(str(out / "setup.png"), np.ascontiguousarray(frames["top_cam"][0][..., ::-1]))
    return summary


def write_videos(frames: dict, out: Path, variant: str) -> None:
    import cv2

    for cam, imgs in frames.items():
        if not imgs:
            continue
        h, w = imgs[0].shape[:2]
        raw = out / f"_{cam}.avi"
        writer = cv2.VideoWriter(str(raw), cv2.VideoWriter_fourcc(*"MJPG"), CONTROL_HZ, (w, h))
        for im in imgs:
            writer.write(np.ascontiguousarray(im[..., ::-1]))
        writer.release()
        name = out / (f"demo_{cam}.mp4" if variant == "full" else f"demo_{variant}_{cam}.mp4")
        if shutil.which("ffmpeg"):
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(raw), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                            "-movflags", "+faststart", str(name)], check=True)
            raw.unlink()
        else:
            raw.rename(name.with_suffix(".avi"))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", choices=VARIANTS + ("all",), default="full")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--no-abort", action="store_true")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a constant: --set PRESS=0.003, or a part's with --set rp.CLAMP_SPRING=1e5")
    args = ap.parse_args(argv)
    for item in args.set:
        name, value = item.split("=", 1)
        if name.startswith("rp."):
            sys.path.insert(0, str(HERE))
            import rp_parts

            target = vars(rp_parts)
        else:
            target = globals()
        name = name.removeprefix("rp.")
        if name not in target:
            ap.error(f"--set: no constant {name!r}")
        target[name] = ast.literal_eval(value)
    video = not args.no_video
    global ABORT_EARLY
    ABORT_EARLY = not args.no_abort
    emb = make_embodiment(video, args.device)
    for variant in VARIANTS if args.variant == "all" else (args.variant,):
        out = args.out / variant if args.out and args.variant == "all" else args.out or Path("demo_runs") / variant
        summary = run(variant, out, video, args.max_steps, emb)
        print("RESULT " + json.dumps({k: summary[k] for k in ("variant", "task_score", "success_step", "steps",
                                                               "wall_s")} | {"screws": summary["screws"]}, default=float))
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
