"""Scripted two-arm demo for the cord-clamp task, closed loop on the simulator's object poses.

    python demo.py --variant full          # cable held, both screws snug in turns, bar parallel: 1.0
    python demo.py --variant uneven        # cable held, both snug, one screw much further (not in turns): 0.75
    python demo.py --variant one_screw     # cable held, one screw tightened, the other barely touching: slips, 0.25
    python demo.py --variant let_go        # pushed in and let go, then tightened: it sprang back, cores clamped: 0
    python demo.py --variant no_tighten    # pushed in and let go, the screws not touched: it springs back, 0
    python demo.py --variant cores         # the cable not pushed in, the screws tightened: cores clamped, 0
    python demo.py --variant all --no-video

It drives the ``yam_arena`` embodiment directly (20 Hz control, object_jitter 0, as a trial runs) and reads the
task's own scoring from each step's ``info``. Each arm follows Cartesian targets turned into joint targets by
yam_arena's IK, corrected from the measured cable, screws and screwdriver (the YAM wrist is soft).

1. right arm: grips the cable just outside the box's entry (tilted 40 degrees, its wrist leaning out away from the
   box, clear of the other hand), pushes it in in one stroke toward 18 mm past the bar's middle (closed loop on the
   measured end; the goal is 9 mm) and holds it there, pressing it in a little (the connected cores' bow pushes it
   back), until the other hand has tightened the screws; then lets go and moves away;
2. left arm: takes the hex screwdriver from its holder from above, and tightens the two screws in turns: a go on one
   screw (the key centred and lowered into the socket, wrist strokes, the key lifted out between strokes), then the
   other: a full turn per go until the heads press the bar, then 0.4 turn per go, until each is 0.8 turn past its
   head's first contact, or both are 0.3 or more past it and a whole round gains nothing (as tight as the grip gets
   them);
3. the screwdriver is laid down beside the holder and the hand moves away; after 1 s hands off the task pulls the
   cable.

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
    "Secure the cable in the terminal box's cord clamp: slide the cable in until its sheath reaches at least 5 mm past "
    "the clamp bar and hold it there while you tighten the bar's two screws in turns with the hex screwdriver, so that "
    "the bar presses on the sheath and stays parallel. Then let go of the cable and the screwdriver."
)
# "uneven": the screws tightened alternately but unevenly: s0 to 0.5 turn past contact, s1 on to 0.9 (as far as the
# grip turns it), then a last go on s0, which the see-sawing bar pushed back: both snug, ~0.4 turn apart: not in
# turns, 0.75 (demo run 70). At (0.28, 0.7), s0 too loose to hold the springy cable: pull failed, 0.25 (demo run 68).
TILTED_FIRST = 0
TILTED_TARGETS = (0.5, 0.9)
TILTED_ALTERNATE = True
TILTED_TOUCHUP = 0.52  # then one more go on the first screw, to this many turns past contact (None: none)
VARIANTS = ("full", "uneven", "one_screw", "let_go", "no_tighten", "cores")
# per variant: push the cable in?, the screws' targets (turns past contact), tighten in alternation?, the cable hand
# holds the cable until the screws are tight? (let go, the connected cores' bow pushes the cable back out)
FULL_TARGETS = (0.8, 0.8)  # the full demo's screws, turns past contact (at 0.35 the bar pressed only 5-10 N and the
# cable, its cores' bow pushing outward too, slipped in the pull test: 1.7 mm, demo run 64; at 0.6 it held 0.66 mm, at
# 0.8 0.004 mm, success at step 6,595: demo runs 66, 67)
PLANS = {"full": (True, None, True, True), "uneven": (True, None, False, True),
         "one_screw": (True, (0.1, 0.7), True, True), "let_go": (True, (0.35, 0.35), True, False),
         "no_tighten": (True, None, True, False), "cores": (False, (0.5, 0.5), True, False)}
CAMERAS = ("top_cam", "left_cam", "right_cam")
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
MAX_STEPS = 9000  # the task's run limit
MM = 1e-3

# -- the cable hand ---------------------------------------------------------------------------------------------------
TCP_UP = 6.0 * MM
OPEN_CABLE = 0.30
GRIP_CABLE = 0.0
GRIP_LINK = "link4"  # the right hand takes the cable here (level, just outside the entry; it holds it there while the
# other hand tightens the screws)
GRIP_TILT = math.radians(40.0)  # the cable hand tilted, its wrist leaning out away from the box (top-down, the two
# hands' wrists met over the box, demo run 61)
SHEATH_TARGET = 18 * MM  # pushed until the sheath's end is this far past the bar's middle (the goal is 9 mm;
# let go, the cable creeps back a few mm: pushed to 12 it ended at 6.5, demo run 3)
PUSH_SPEED = 8 * MM
PUSH_LEAD = 40 * MM  # the push command runs at most this far ahead of the measured hand (the cores' bow pushes back:
# at 25 mm the hand stalled with the sheath at 12.5, demo run 59)
HOLD_LEAD = 6 * MM  # holding, the command stays this far ahead of the hand (at 0 the soft arm gave 5 mm, demo run 59)
STALL_STEPS = 20  # the command PUSH_LEAD ahead and the hand advanced < 0.5 mm in this many steps: blocked (its fingers at the box's wall)
PUSH_STROKES = 1  # (re-gripping lets the cable spring back, demo run 56)

# -- the screwdriver (as in rail-plate-mount, a 3 mm key) -------------------------------------------------------------
PAD_TIP = 0.013
OPEN_DRIVER = 0.45
DRIVER_TOP_GRIP = 0.060  # the pad tips reach down to this far from the blade tip (the handle)
HOVER = 8 * MM
NEAR = 2 * MM
CENTRE_TRIES = 5
CENTRE_SETTLE = 0.5
BLADE_CENTRE_TOL = 0.25 * MM
ENGAGE_TOL = 0.8 * MM  # engaged: the key's tip within this of the socket's floor (the socket is 2 mm deep)
INTO_SOCKET = 1.2 * MM
SPIRAL_R, SPIRAL_TURNS, SPIRAL_S = 0.5 * MM, 2.0, 4.0
WIGGLE, WIGGLE_TURNS = math.radians(25.0), 3.0
PRESS = 5.0 * MM
PRESS_SEATED = 9.0 * MM  # pressing harder once the head presses the bar (at 5 mm the key cammed out of the
# loaded socket, riding up 0.5 mm and clicking round, demo runs 9-13)
PRESS_RAMP_S = 0.3
TURN_RATE = math.radians(120.0)
RATCHET_RATE = math.radians(120.0)
TURN_LEAD = math.radians(10.0)
TURN_LEAD_SEATED = math.radians(35.0)
STALL_S = 3.0
SLIP_DROP = 0.6 * MM
CAM_OUT = 1.4 * MM
MAX_SLIPS = 30
ENGAGE_TRIES = 5
WRIST_LIMIT = math.radians(95.0)
WRIST_JOINT_LIMIT = math.radians(120.0)
WRIST_MARGIN = math.radians(25.0)
INC_FREE, INC_PRESSED = 1.0, 0.4  # turns per go: before both heads press, and after
GO_MAX_STROKES = 5  # strokes per go at most (a screw that no longer turns: go to the other one)
SNUG_ENOUGH = 0.3  # both screws this far past contact and a whole round gained nothing: done (as tight as
# the hand gets them; well inside the snug window 0.25-1.75)
DRIVER_PUT_DOWN = (0.17, 0.11)  # laid down on the table here

SETTLE_TOL, SETTLE_MAX_S = 0.6 * MM, 2.0
AFTER_SUCCESS_S = 3.0
IK_ABORT_MM, IK_ABORT_S = 20.0, 2.0
ABORT_EARLY = True


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def rot_axis(n, a: float) -> np.ndarray:
    """Rotation by ``a`` about the unit axis ``n`` (Rodrigues)."""
    n = np.asarray(n, dtype=float) / np.linalg.norm(n)
    k = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_pi_half(a: float) -> float:
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
        self.down = grip_rotation(0.0)
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
    obs = emb.reset(Scene(id=f"cord-clamp-{variant}", instruction=INSTRUCTION), seed=0)
    set_rendering(emb, video)
    task = emb.task.module
    cc = task.cc
    base_env = emb._env.unwrapped
    from yam_arena.scene import object_states

    push, targets, alternate, hold = PLANS[variant]
    if variant == "full":
        targets = FULL_TARGETS
    if variant == "uneven":
        targets, alternate = TILTED_TARGETS, TILTED_ALTERNATE
    left_q, right_q = split(np.asarray(obs.state["joint_pos"]))
    arms = {"left": Arm("left", station.BASES["left"], left_q[:6], ik, grip_rotation),
            "right": Arm("right", station.BASES["right"], right_q[:6], ik, grip_rotation)}
    world = {"states": object_states(emb._env, emb.task.names), "obs": obs, "step": 0}
    flags: dict = {}
    report: dict = {"screws": {s: {} for s in cc.SCREWS}}
    phase = {"left": "", "right": ""}
    O, R = cc.clamp_frame()

    # -- measurements -------------------------------------------------------------------------------------------
    def geo():
        return cc.cable_geometry(base_env)

    def sheath_y():
        return float(geo()["sheath_end"][1])

    def link_point(name):
        """(world point at the link's middle, unit direction toward the terminals) of a cable link."""
        g = geo()
        i = [n for n, *_ in cc.cable_links()].index(name)
        p = g["starts"][i] + 0.5 * cc.LINK_LEN * g["axes"][i]
        return R @ p + O, -(R @ g["axes"][i])

    def screw(s):
        """(head underside on the axis, rotation) of a screw."""
        p, r = cc.body_poses(base_env, cc._NAMES["clamp"])[s]
        return p, r

    def socket(s):
        p, r = screw(s)
        return p + (cc.HEAD_H - cc.SOCKET_DEPTH) * r[:, 2], math.atan2(r[1, 0], r[0, 0])

    def driver():
        return cc.rigid_pose(base_env, cc._NAMES["driver"])

    def blade_angle() -> float:
        _, r = driver()
        return math.atan2(r[1, 0], r[0, 0])

    def wrist_angle(side: str) -> float:
        left, right = split(np.asarray(world["obs"].state["joint_pos"]))
        return float((left if side == "left" else right)[5])

    def tcp_measured(arm: Arm):
        left, right = split(np.asarray(world["obs"].state["joint_pos"]))
        return ik.tcp_pose((left if arm.side == "left" else right)[:6], arm.base)[0]

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

    def settle_at(arm: Arm, label="", tol=SETTLE_TOL, max_s=SETTLE_MAX_S):
        phase[arm.side] = label
        for _ in range(round(max_s / DT)):
            if np.linalg.norm(tcp_measured(arm) - arm.pos) < tol:
                break
            yield

    def wait_for(arm: Arm, flag: str, label=""):
        phase[arm.side] = label
        while not flags.get(flag):
            yield

    # -- right arm: the cable -----------------------------------------------------------------------------------
    def right_script():
        arm = arms["right"]
        home, home_rot = arm.pos.copy(), arm.rot.copy()
        yield from wait(arm, 0.5, "start")
        if push:
            p, d = link_point(GRIP_LINK)
            r = rot_z(wrap_pi_half(math.atan2(d[1], d[0]))) @ arm.down
            d_in = np.array([d[0], d[1], 0.0]) / math.hypot(d[0], d[1])
            r = rot_axis(np.cross(d_in, (0.0, 0.0, 1.0)), GRIP_TILT) @ r  # the wrist leaning out (away from the box)
            yield from move(arm, p + (0, 0, TCP_UP + 0.04), 2.0, grip=OPEN_CABLE, rot=r, label="above the cable")
            p, d = link_point(GRIP_LINK)
            yield from move(arm, p + (0, 0, TCP_UP), 1.0, label="down to the cable")
            yield from settle_at(arm, "settle")
            # push it in level (toward the terminals) until the sheath's end is past the bar; when the hand stalls
            # (its fingers at the box's wall), let go, take the cable further out and push again (hand over hand;
            # pushing along the drooping link lifted the cable and the hand stalled at the wall, demo run 4)
            d = np.array([d[0], d[1], 0.0])
            d /= np.linalg.norm(d)
            grip_at = arm.pos.copy()
            for stroke in range(PUSH_STROKES):
                if stroke:
                    yield from move(arm, arm.pos, 0.5, grip=OPEN_CABLE, label="let go to re-grip")
                    yield from move(arm, arm.pos + (0, 0, 0.02), 0.6, label="up")
                    yield from move(arm, grip_at + (0, 0, 0.02), 1.2, label="back along the cable")
                    yield from move(arm, grip_at, 0.6, label="down to the cable")
                    yield from settle_at(arm, "settle")
                yield from move(arm, arm.pos, 0.6, grip=GRIP_CABLE, label="grip the cable")
                yield from wait(arm, 0.3, "gripped")
                phase["right"] = f"push the cable in ({stroke + 1})"
                track = []
                for _ in range(round(10.0 / DT)):
                    err = SHEATH_TARGET - sheath_y()
                    meas = tcp_measured(arm)
                    track.append(float(meas @ d))
                    if err <= 0.0:
                        break
                    lead = float((arm.pos - meas) @ d)
                    if (lead > PUSH_LEAD - 1 * MM and len(track) > STALL_STEPS
                            and track[-1] - track[-1 - STALL_STEPS] < 0.5 * MM):
                        break  # pressing as hard as it may and the hand doesn't advance any more
                    if lead < PUSH_LEAD:  # the command runs at most PUSH_LEAD ahead of the hand
                        arm.pos = arm.pos + d * min(PUSH_SPEED * DT, err + 0.5 * MM)
                    yield
                arm.pos = tcp_measured(arm) + (0.0, 0.0, 0.0)  # stop pressing on the stalled hand
                yield from wait(arm, 0.3, "hold")
                if sheath_y() >= SHEATH_TARGET - 2 * MM:
                    break
            report["pushed"] = {"sheath_y_mm": round(sheath_y() / MM, 2), "step": world["step"], "strokes": stroke + 1}
        if push and hold:  # hold the cable in (its cores' bow pushes it back) until the other hand has tightened the
            # screws; then let go
            arm.pos = tcp_measured(arm) + d * HOLD_LEAD  # keep pressing it in a little
            flags["cable_in"] = True
            yield from wait_for(arm, "clamped", "hold the cable")
        # (variants let_go, no_tighten: let go at once and move away; the cable springs back)
        if push:
            yield from move(arm, arm.pos, 0.5, grip=OPEN_CABLE, label="let go")
            yield from move(arm, arm.pos + (0, 0, 0.06), 1.0, label="up")
        yield from move(arm, home, 2.0, rot=home_rot, label="home")
        flags["cable_in"] = True
        flags["right_done"] = True
        phase["right"] = "done"
        while True:
            yield

    # -- left arm: the screwdriver (after rail-plate-mount's demo) ----------------------------------------------------
    def best_wrist_yaw(arm: Arm, tcp_at, align: float, grasp_at=None) -> float:
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
            if grasp_at is not None:
                return best_wrist_yaw(arm, tcp_at, align)
            q6_tight = 1.0
        report["wrist_q6_sign"] = q6_tight
        return best

    def tcp_for_tip(target_tip):
        tip, _ = driver()
        return arms["left"].pos + (np.asarray(target_tip) - tip)

    def engage(s, label):
        """Turn the key to the socket's angle, centre it and lower it in (a small spiral if it rests on the rim)."""
        arm = arms["left"]
        rec = report["screws"][s]
        bottom, ang = socket(s)
        sign = report.get("wrist_q6_sign", 1.0)
        room = sign * wrist_angle("left") + (WRIST_JOINT_LIMIT - math.radians(10.0))
        base_turn = wrap_sixth(ang - blade_angle())
        want = sign * wrist_angle("left") + (WRIST_JOINT_LIMIT - WRIST_MARGIN)
        fits = [base_turn + k * math.pi / 3 for k in range(-6, 7) if base_turn + k * math.pi / 3 <= room]
        phi = min(fits, key=lambda p: abs(p - want)) if fits else base_turn
        if driver()[0][2] > bottom[2] + 0.015 or np.linalg.norm((driver()[0] - bottom)[:2]) > 0.004:
            yield from move(arm, tcp_for_tip(bottom + (0, 0, 0.03)), 1.2, label=f"{label}: over the screw")
            for _ in range(round(0.5 / DT)):
                arm.pos = arm.pos + np.clip(0.5 * ((socket(s)[0] + (0, 0, 0.03)) - driver()[0]), -0.002, 0.002)
                yield
        r_from, p_from = arm.rot.copy(), arm.pos.copy()
        p_to = tcp_for_tip(socket(s)[0] + (0, 0, HOVER))
        n = max(1, round(max(1.0, abs(phi) / RATCHET_RATE) / DT))
        for i in range(1, n + 1):
            sm = smooth(i / n)
            arm.rot = rot_z(phi * sm) @ r_from
            arm.pos = p_from + sm * (p_to - p_from)
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
                    b_, a_ = socket(s)
                    errs.append((b_ + (0, 0, height)) - driver()[0])
                    yield
                err = np.mean(errs, axis=0)
                if np.linalg.norm(err[:2]) < BLADE_CENTRE_TOL:
                    break
                arm.pos = arm.pos + np.clip(err, -0.003, 0.003)
                arm.rot = rot_z(wrap_sixth(a_ - blade_angle())) @ arm.rot
            rec.setdefault("blade_centre_err_mm", []).append(round(float(np.linalg.norm(err[:2])) / MM, 3))

        yield from centre(HOVER, CENTRE_TRIES, "centre on the socket")
        yield from move(arm, arm.pos - np.array([0.0, 0.0, HOVER - NEAR]), 0.8, label=f"{label}: down to the head")
        yield from centre(NEAR, 2, "centre again near the head")
        yield from move(arm, arm.pos - np.array([0.0, 0.0, NEAR + INTO_SOCKET]), 0.6, label=f"{label}: into the socket")
        ok = abs(driver()[0][2] - socket(s)[0][2]) < ENGAGE_TOL
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
                if abs(driver()[0][2] - socket(s)[0][2]) < ENGAGE_TOL:
                    ok = True
                    break
            rec.setdefault("spiral_hits", []).append(ok)
        rec["engaged"] = ok

    def screw_go(s, target, inc):
        """One go on a screw: engage, wrist strokes until it has turned ``inc`` more or reached ``target`` turns past
        its head's first contact; lift the key out."""
        arm = arms["left"]
        rec = report["screws"][s]
        rec.setdefault("strokes", 0)
        rec.setdefault("slips", 0)
        i = cc.SCREWS.index(s)
        start_turns = world["states"]["clamp"].turns[i]
        go_strokes = 0

        def done():
            k = world["states"]["clamp"]
            reached = k.contact_turns[i] is not None and k.past_contact[i] >= target
            return reached or k.turns[i] - start_turns >= inc

        for attempt in range(ENGAGE_TRIES):
            if attempt:
                yield from move(arm, arm.pos + (0, 0, 0.004), 0.3, label="lift for another try")
            yield from engage(s, f"{s}: engage")
            if rec.get("engaged"):
                break
        if not rec.get("engaged"):
            rec["gave_up"] = "the key doesn't go into the socket"
            return
        while not done():
            rec["strokes"] += 1
            r0 = arm.rot.copy()
            base = arm.pos.copy()
            n_ramp = round(PRESS_RAMP_S / DT)
            press = PRESS_SEATED if world["states"]["clamp"].clamp_force[i] >= cc.CONTACT_CLAMP else PRESS
            for j in range(1, n_ramp + 1):
                arm.pos = base - (0, 0, smooth(j / n_ramp) * press)
                phase["left"] = f"{s}: stroke {rec['strokes']} (press)"
                yield
            turned, last_meas, meas_total = 0.0, blade_angle(), 0.0
            best_meas, stall_steps = 0.0, 0
            phase["left"] = f"{s}: stroke {rec['strokes']}"
            slipped = False
            while not done():
                k = world["states"]["clamp"]
                a_ = blade_angle()
                meas_total += wrap(a_ - last_meas)
                last_meas = a_
                lead = TURN_LEAD_SEATED if k.clamp_force[i] >= cc.CONTACT_CLAMP else TURN_LEAD
                turned = max(turned - TURN_RATE * DT, meas_total - lead)
                if meas_total < best_meas - math.radians(1.0):
                    best_meas, stall_steps = meas_total, 0
                else:
                    stall_steps += 1
                    if stall_steps > round(STALL_S / DT):
                        break
                arm.rot = rot_z(turned) @ r0
                arm.pos = base - (0, 0, press)
                bottom, _ = socket(s)
                tip, _ = driver()
                if tip[2] < bottom[2] - SLIP_DROP or tip[2] > bottom[2] + CAM_OUT:
                    slipped = True
                    break
                if report.get("wrist_q6_sign", 1.0) * wrist_angle("left") > WRIST_LIMIT or -turned > math.radians(200):
                    break
                yield
            if slipped:
                rec["slips"] += 1
                if rec["slips"] >= MAX_SLIPS:
                    rec["gave_up"] = f"slipped out of the socket {MAX_SLIPS} times"
                    return
            go_strokes += 1
            if done() or go_strokes >= GO_MAX_STROKES:
                break
            yield from move(arm, base + (0, 0, 0.004), 0.4, label=f"{s}: lift out of the socket")
            rec["engaged"] = False
            for attempt in range(ENGAGE_TRIES):
                yield from engage(s, f"{s}: next stroke")
                if rec.get("engaged"):
                    break
                yield from move(arm, arm.pos + (0, 0, 0.004), 0.3, label="lift for another try")
            if not rec.get("engaged"):
                rec["gave_up"] = "the key doesn't go back into the socket"
                return
        yield from move(arm, arm.pos + (0, 0, PRESS + 0.012), 0.6, label=f"{s}: lift the key out")

    def left_script():
        arm = arms["left"]
        home, home_rot = arm.pos.copy(), arm.rot.copy()
        yield from wait(arm, 0.5, "start")
        if targets is not None:
            # the screwdriver, from above, coaxially; the yaw leaving the wrist room for long strokes
            tip, rd = driver()
            axis = rd[:, 2]
            bottom, ang = socket(cc.SCREWS[0])
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
            yield from wait_for(arm, "cable_in", "wait for the cable")
            order = list(cc.SCREWS)
            if alternate:
                cur = 0
                last_round = None
                for go in range(60):
                    k = world["states"]["clamp"]
                    reached = [k.contact_turns[i] is not None and k.past_contact[i] >= targets[i] for i in range(2)]
                    if all(reached):
                        break
                    if go % 2 == 0:  # a round = a go on each screw
                        now = list(k.past_contact)
                        stuck = last_round is not None and max(a - b for a, b in zip(now, last_round)) < 0.02
                        if (all(c is not None for c in k.contact_turns) and stuck
                                and (min(now) >= SNUG_ENOUGH or min(targets) < SNUG_ENOUGH)):
                            # a whole round gained nothing: as tight as the hand gets them (well inside the snug
                            # window once both are past SNUG_ENOUGH)
                            report["stopped"] = "a round gained nothing"
                            break
                        last_round = now
                    if reached[cur]:
                        cur = 1 - cur
                    pressed = all(c is not None for c in k.contact_turns)
                    yield from screw_go(order[cur], targets[cur], INC_PRESSED if pressed else INC_FREE)
                    if any("gave_up" in report["screws"][s] for s in order):
                        break
                    cur = 1 - cur
            else:  # one screw all the way first, then the other
                for i in (TILTED_FIRST, 1 - TILTED_FIRST):
                    s = order[i]
                    before = None
                    for _ in range(30):
                        k = world["states"]["clamp"]
                        if k.contact_turns[i] is not None and k.past_contact[i] >= targets[i]:
                            break
                        if before is not None and k.contact_turns[i] is not None and k.turns[i] - before < 0.02:
                            break  # it doesn't turn any more: as tight as the hand gets it
                        before = k.turns[i]
                        yield from screw_go(s, targets[i], 10.0)
                        if "gave_up" in report["screws"][s]:
                            break
            if variant == "uneven" and TILTED_TOUCHUP is not None:  # a last go on the first screw (the bar pushed it
                # back while the other was tightened)
                i = TILTED_FIRST
                k = world["states"]["clamp"]
                if k.contact_turns[i] is not None and k.past_contact[i] < TILTED_TOUCHUP:
                    yield from screw_go(order[i], TILTED_TOUCHUP, INC_PRESSED)
            k = world["states"]["clamp"]
            report["clamp"] = {"turns": [round(v, 3) for v in k.turns], "past": [round(v, 3) for v in k.past_contact],
                               "force_N": [round(v, 1) for v in k.clamp_force],
                               "tilt_deg": round(world["states"]["bar"].tilt_deg, 2)}
            gave = [report["screws"][s].get("gave_up") for s in order if report["screws"][s].get("gave_up")]
            if gave:
                report["abort"] = gave[0]
            yield from move(arm, arm.pos + (0, 0, 0.04), 1.0, label="up")
            flags["clamped"] = True  # the other hand may let go of the cable
            yield from move(arm, np.array([DRIVER_PUT_DOWN[0], DRIVER_PUT_DOWN[1], arm.pos[2]]), 2.0,
                            label="carry the screwdriver away")
            yield from move(arm, arm.pos - np.array([0.0, 0.0, arm.pos[2] - task.TOP - 0.10]), 1.0,
                            label="down to the table")
            yield from move(arm, arm.pos, 0.5, grip=OPEN_DRIVER, label="let go of the screwdriver")
            yield from move(arm, arm.pos + (0, 0, 0.06), 1.0, label="up")
        else:
            yield from wait_for(arm, "cable_in", "wait for the cable")
            flags["clamped"] = True
        flags["screws_done"] = True
        yield from move(arm, home, 2.0, rot=home_rot, label="home")
        flags["left_done"] = True
        phase["left"] = "done"
        while True:
            yield

    scripts = {"left": left_script(), "right": right_script()}
    frames = {c: [] for c in CAMERAS}
    log = []
    result = None
    success_step = None
    t0 = time.time()
    out.mkdir(parents=True, exist_ok=True)
    live = open(out / "steps_live.jsonl", "w")  # written as the run goes (kept if the simulator crashes)
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
        k, c, b = st["clamp"], st["cable"], st["bar"]
        tip, _ = driver()
        row = {"step": step, "left": phase["left"], "right": phase["right"], "score": res.info.get("score"),
               "success": res.info.get("success"), "turns": [round(v, 3) for v in k.turns],
               "past": [round(v, 3) for v in k.past_contact], "force_N": [round(v, 1) for v in k.clamp_force],
               "tilt_deg": round(float(b.tilt_deg), 2), "sheath_mm": round(c.sheath_y / MM, 2),
               "crushed": k.crushed, "cores": k.cores_clamped, "pull": k.pull,
               "pull_moved_mm": round(k.pull_moved / MM, 3), "hands_off": int(k.hands_off_steps),
               "driver_tip": [round(float(v), 4) for v in tip],
               "l_ik_mm": round(arms["left"].ik_err[0] / MM, 2), "r_ik_mm": round(arms["right"].ik_err[0] / MM, 2),
               "l_cmd": [round(float(v), 4) for v in arms["left"].pos],
               "l_meas": [round(float(v), 4) for v in tcp_measured(arms["left"])],
               "r_cmd": [round(float(v), 4) for v in arms["right"].pos],
               "r_meas": [round(float(v), 4) for v in tcp_measured(arms["right"])],
               "still": {n: [bool(o.resting()), round(float(np.linalg.norm(o.lin_vel)), 4),
                             round(float(np.linalg.norm(o.ang_vel)), 3)] for n, o in (("cable", c), ("clamp", k),
                                                                                    ("bar", b))}}
        log.append(row)
        if step % 20 == 0:
            live.write(json.dumps(row, default=float) + "\n")
            live.flush()
        if video:
            for cam in CAMERAS:
                frames[cam].append(res.observation.images[cam])
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
        if (success_step is None and flags.get("left_done") and flags.get("right_done")
                and k.hands_off_steps >= round(6.0 / DT)):
            break
    live.close()
    (out / "steps_live.jsonl").unlink()
    st = world["states"]
    k, c, b = st["clamp"], st["cable"], st["bar"]
    summary = {
        "variant": variant,
        "task_score": 1.0 if success_step is not None else float(result.info.get("score", 0.0)) if result else 0.0,
        "success_step": success_step,
        "steps": len(log),
        "run_limit_steps": MAX_STEPS,
        "final": {"sheath_past": task.sheath_past(st), "holds": task.holds(st), "both_snug": task.both_snug(st),
                  "in_turns": task.in_turns(st), "parallel": task.parallel(st), "tilt_deg": float(b.tilt_deg), "crushed": bool(k.crushed),
                  "cores_clamped": bool(k.cores_clamped), "past_contact": list(k.past_contact), "pull": k.pull,
                  "pull_moved_mm": float(k.pull_moved) / MM, "sheath_y_mm": float(c.sheath_y) / MM},
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
                    help="override a constant: --set PUSH_SPEED=0.005, or a part's with --set cc.CLAMP_K=1e5")
    args = ap.parse_args(argv)
    for item in args.set:
        name, value = item.split("=", 1)
        if name.startswith("cc."):
            sys.path.insert(0, str(HERE))
            import cc_parts

            target = vars(cc_parts)
        else:
            target = globals()
        name = name.removeprefix("cc.")
        if name not in target:
            ap.error(f"--set: no constant {name!r}")
        try:
            target[name] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            target[name] = value  # a plain word: a string (GRIP_LINK=link4)
    video = not args.no_video
    global ABORT_EARLY
    ABORT_EARLY = not args.no_abort
    emb = make_embodiment(video, args.device)
    for variant in VARIANTS if args.variant == "all" else (args.variant,):
        out = args.out / variant if args.out and args.variant == "all" else args.out or Path("demo_runs") / variant
        summary = run(variant, out, video, args.max_steps, emb)
        print("RESULT " + json.dumps({k: summary[k] for k in ("variant", "task_score", "success_step", "steps",
                                                               "wall_s")} | {"final": summary["final"]}, default=float))
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
