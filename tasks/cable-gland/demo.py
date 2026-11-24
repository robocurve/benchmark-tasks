"""Scripted two-arm demo for the cable-gland task, closed loop on the simulator's object poses.

    python demo.py --variant full          # inserted, snug, cable held while tightening: 1.0
    python demo.py --variant overtight     # tightened 1.12 turns past contact: past the upper mark, 0.75
    python demo.py --variant loose         # tightened 0.36 turn past contact: holds, short of the marks, 0.5
    python demo.py --variant no_tighten    # inserted, nut not touched: 0.25
    python demo.py --variant crush         # tightened 1.45 turns past contact: crushed, 0
    python demo.py --variant all --no-video

It drives the ``yam_arena`` embodiment directly (20 Hz control, object_jitter 0, as a trial runs) and reads the
task's own scoring from each step's ``info``. Each arm follows Cartesian targets turned into joint targets by
yam_arena's IK, corrected from the measured cable and nut (the YAM wrist is soft: correct, settle, measure).

1. left arm: takes the cable 15 mm behind the sheath's cut end, lifts it onto the gland's axis in front of the cap
   nut (closed loop on the measured cores' tip), pushes the cores in, lets go, takes it again further back and
   pushes until the sheath's end is ~15 mm past the gland's inner end; then holds it 100 mm in front of the cap;
2. right arm: grips the cap nut across two flats and turns it clockwise (seen from the arms) in wrist strokes about
   the gland's axis (re-gripping between strokes), until it has turned the target amount past the seal's first
   contact (full: 0.65 turn, inside the 0.4-1.0 marks); lets go;
3. both hands move away; after 1 s hands off the task pulls the cable (30 N); the run continues 3 s past success.

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
    "Fit the cable through the cable gland into the box until its sheath reaches at least 5 mm past the gland's inner "
    "end, then tighten the cap nut snug, between the marks, neither short of them nor past them. Then let go of both."
)
VARIANTS = ("full", "overtight", "loose", "no_tighten", "crush")
TARGET_PAST = {"full": 1.25, "overtight": 1.62, "loose": 0.5, "no_tighten": None, "crush": 1.9}  # between the
# measured marks (H 1.0, T 1.5, D 1.75 turns past first contact)
CAMERAS = ("top_cam", "left_cam", "right_cam")
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
MAX_STEPS = 6000  # the task's run limit (to set from the demo)
MM = 1e-3

# -- the cable hand -----------------------------------------------------------------------------------------------
CABLE_LEAN = math.radians(0.0)  # the cable hand leaned forward (its upper part toward the box): upright, the cable
# behind it, bent up by the lift and set so, jammed against the gripper on every push (demo runs 21-26)
TCP_UP = 6.0 * MM  # gripping the cable: the TCP this far above its centre (pad tips ~7 mm below it)
OPEN_CABLE = 0.30  # ~28 mm
GRIP_CABLE = 0.0  # closed on the 8 mm cable (the gripper's force limit holds it)
GRIP_BACK_1 = 15 * MM  # first grip: this far behind the sheath's cut end (the cores 75 mm ahead)
FRONT_GAP = 30 * MM  # lined up with the cores' tip this far in front of the cap's mouth
TIP_TOL = 0.8 * MM  # the tip within this of the axis before pushing in (the mouth's lead-in catches ~3 mm)
LINE_UP_TRIES = 6
HAND_TO_MOUTH = 24 * MM  # the gripping hand stops this far in front of the mouth (pads 27 mm wide)
REGRIPS = 3
STRAIGHT_PULL = 2.0 * MM  # after a regrip the hand goes this much further back than the free length (pulls it straight)
HOLD_TILT = math.radians(40.0)  # the holding hand tilted about the cable's axis, its upper part outward (-y), away from
# the left arm at the nut
HOLD_ALONG = 100 * MM  # the right hand holds the cable this far (along it) in front of the cap's mouth (at 90 mm
# its gripper ran into the left one, still on the cable by the gland, and the wrists met during the strokes, demo run 5)
SHEATH_GOAL = 25 * MM  # pushed until the sheath's end is here (gland frame; the inner end is at +9, scored from +14;
# at +20 an unheld cable ended at +13.4 after the strokes, demo run 13)
PUSH_SPEED = 20 * MM  # m/s (at 20 the bowed cable behind the hand, its set lagging, slid through the grip, demo run 28)

# -- the nut hand -------------------------------------------------------------------------------------------------
NUT_TCP_UP = 2.0 * MM  # on the nut: the TCP this far out from its axis (pad tips ~11 mm past it; the soft wrist rides
# ~6 mm high: at 8 mm the pads only just reached the flats, demo run 3)
NUT_LEAN = math.radians(-25.0)  # the nut hand leaned back toward the arms: upright, its upper part hit the box wall's
# top edge (40 mm above the axis) when it rolled, and the nut slipped out (demo run 3)
OPEN_NUT = 0.40  # ~38 mm (the 24 mm hexagon's corners: 27.7 mm)
GRIP_NUT = 0.0
ROLL_MIN, ROLL_MAX = math.radians(0.0), math.radians(70.0)  # (rolled to -41 degrees its wrist camera met the box
# wall's top edge, demo run 7)  # the left hand (leaned) reaches -60..+30 degrees
# (reach scan); a stroke starts at the flats' angle in [-50, +10) and turns to +20
STROKE_RATE = math.radians(45.0)  # rad/s
APPROACH = 25 * MM  # the hand comes in along its own axis from this far out
MAX_STROKES = 90

SETTLE_TOL, SETTLE_MAX_S = 0.6 * MM, 2.0
AFTER_SUCCESS_S = 3.0
IK_ABORT_MM, IK_ABORT_S = 20.0, 2.0
ABORT_EARLY = True
STOP_AFTER_STROKES = 0  # debug: stop once the nut hand starts this stroke (0: never)


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_axis(axis, a: float) -> np.ndarray:
    """Rotation by ``a`` about a unit ``axis`` (Rodrigues)."""
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def wrap_pi_half(a: float) -> float:
    return (a + math.pi / 2) % math.pi - math.pi / 2


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
    obs = emb.reset(Scene(id=f"cable-gland-{variant}", instruction=INSTRUCTION), seed=0)
    set_rendering(emb, video)
    task = emb.task.module
    cg = task.cg
    base_env = emb._env.unwrapped
    from yam_arena.scene import object_states  # patched by the task: runs the gland update

    left_q, right_q = split(np.asarray(obs.state["joint_pos"]))
    arms = {
        "left": Arm("left", station.BASES["left"], left_q[:6], ik, grip_rotation),
        "right": Arm("right", station.BASES["right"], right_q[:6], ik, grip_rotation),
    }
    world = {"states": object_states(emb._env, emb.task.names), "obs": obs, "step": 0}
    flags: dict = {}
    report: dict = {}
    phase = {"left": "", "right": ""}
    G = np.array(task.GLAND_ORIGIN)  # the gland frame (yaw 0: its axes are the world's)
    mouth_x = G[0] + cg.NUT_X0 - cg.NUT_DOME_LEN

    # -- measurements -------------------------------------------------------------------------------------------
    def geo():
        return cg.cable_geometry(base_env, "gland")

    def cable_point(back: float):
        """(world point, unit direction toward the free end) on the cable's centreline ``back`` m behind the sheath's
        cut end."""
        g = geo()
        links = cg.cable_links()
        i = cg.SHEATH_LINKS - 1
        remaining = back
        while i > 0 and remaining > links[i][2]:
            remaining -= links[i][2]
            i -= 1
        p = g["starts"][i] + (links[i][2] - remaining) * g["axes"][i]
        return p + G, g["axes"][i]

    def tip():
        return geo()["tip"] + G

    def nut():
        """(the nut's travel m, its spin angle rad, the hexagon's centre in the world)."""
        q = cg.joints(base_env, "gland")
        s = q["thread"]
        return s, q["spin"], G + np.array([cg.NUT_X0 + cg.NUT_HEX_LEN / 2 + s, 0.0, 0.0])

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

    # -- left arm: the cable -------------------------------------------------------------------------------------
    def grip_cable_at(arm: Arm, back: float, label: str, tilt: float = 0.0, tilt_toward_y: float = 0.0,
                      lean: bool = True):
        """Take the cable ``back`` m behind the sheath's end from above, fingers across it; the hand tilted ``tilt``
        about the cable's axis, its upper part toward the side ``tilt_toward_y`` (sign of world y)."""
        p, d = cable_point(back)
        yaw = wrap_pi_half(math.atan2(d[1], d[0]))
        r = rot_z(yaw) @ rot_y(CABLE_LEAN if lean else 0.0) @ arm.down
        if tilt:
            axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            best = None
            for t in (tilt, -tilt):
                rt = rot_axis(axis, t) @ r
                up_t = rt @ arm.down.T @ np.array([0.0, 0.0, 1.0])
                if best is None or up_t[1] * tilt_toward_y > best[0]:
                    best = (up_t[1] * tilt_toward_y, rt)
            r = best[1]
        up = r @ arm.down.T @ np.array([0.0, 0.0, 1.0])
        yield from move(arm, p + (TCP_UP + 0.04) * up, 1.5, grip=OPEN_CABLE, rot=r, label=f"{label}: above")
        p, _ = cable_point(back)
        yield from move(arm, p + TCP_UP * up, 1.0, label=f"{label}: down")
        yield from settle_at(arm, f"{label}: settle")
        yield from move(arm, arm.pos, 0.6, grip=GRIP_CABLE, label=f"{label}: close")
        yield from wait(arm, 0.3, f"{label}: closed")

    def left_script():
        arm = arms["left"]
        home, home_rot = arm.pos.copy(), arm.rot.copy()
        yield from wait(arm, 0.5, "start")
        yield from grip_cable_at(arm, GRIP_BACK_1, "take the cable")
        yield from move(arm, arm.pos + np.array([0.0, 0.0, 0.05]), 1.0, label="lift")
        # in front of the mouth, the cable along the axis: the hand GRIP_BACK_1 + 75 mm behind the tip
        reach = GRIP_BACK_1 + cg.CORE_LINKS * cg.CORE_LEN
        carry_rot = rot_y(CABLE_LEAN) @ arm.down
        up = carry_rot @ arm.down.T @ np.array([0.0, 0.0, 1.0])
        target = np.array([mouth_x - FRONT_GAP - reach, G[1], G[2]]) + TCP_UP * up
        yield from move(arm, target + np.array([0.0, 0.0, 0.02]), 2.0, rot=carry_rot, label="carry to the gland")
        yield from move(arm, target, 1.0, label="onto the axis")
        # line the cores' tip up on the axis (the cable droops and keeps some of its curve)
        for k in range(LINE_UP_TRIES):
            yield from wait(arm, 0.6, f"line up {k}")
            t = tip()
            err = np.array([0.0, t[1] - G[1], t[2] - G[2]])
            report.setdefault("line_up", []).append([round(float(v) / MM, 2) for v in err[1:]])
            if np.linalg.norm(err) < TIP_TOL:
                break
            arm.pos = arm.pos - err
        # push: the hand along the axis until it is HAND_TO_MOUTH in front of the mouth (correcting the tip until
        # it is in the mouth)
        def push_to(x_hand_end, label):
            x0 = arm.pos[0]
            n = max(1, round(abs(x_hand_end - x0) / PUSH_SPEED / DT))
            phase[arm.side] = label
            for i in range(1, n + 1):
                arm.pos = np.array([x0 + (x_hand_end - x0) * i / n, arm.pos[1], arm.pos[2]])
                t = tip()
                if t[0] < mouth_x - 2 * MM:
                    arm.pos[1:] -= 0.15 * (t[1:] - G[1:])
                if cg.cable_geometry(base_env, "gland")["sheath_end"][0] >= SHEATH_GOAL:
                    break
                yield

        yield from push_to(mouth_x - HAND_TO_MOUTH, "push the cores in")
        report["after_push_1"] = {"sheath_x_mm": round(float(geo()["sheath_end"][0]) / MM, 1),
                                  "tip_x_mm": round(float(geo()["tip"][0]) / MM, 1)}
        # regrip further back and push the sheath through (again if the cable slipped in the fingers)
        for k in range(REGRIPS):
            if float(geo()["sheath_end"][0]) >= SHEATH_GOAL - 1.0 * MM:
                break
            yield from move(arm, arm.pos, 0.5, grip=OPEN_CABLE, label=f"let go {k}")
            yield from move(arm, arm.pos + np.array([0.0, 0.0, 0.015]), 0.5, label=f"up {k}")
            # where to take it so that the hand, stopped HAND_TO_MOUTH in front of the mouth, has pushed the sheath's
            # end to SHEATH_GOAL (+ a little: the cable lags the hand)
            back = (G[0] + SHEATH_GOAL + 4.0 * MM) - (mouth_x - HAND_TO_MOUTH)
            yield from grip_cable_at(arm, back, f"take it further back {k}")
            report.setdefault("regrips", []).append({"sheath_x_mm": round(float(geo()["sheath_end"][0]) / MM, 1),
                                                     "grip_meas": round(float(split(np.asarray(
                                                         world["obs"].state["joint_pos"]))[0][6]), 4)})
            up = arm.rot @ arm.down.T @ np.array([0.0, 0.0, 1.0])
            arm.pos = np.array([arm.pos[0], G[1], G[2]]) + TCP_UP * up
            yield from settle_at(arm, f"on the axis {k}: settle")
            yield from push_to(mouth_x - HAND_TO_MOUTH, f"push the sheath through {k}")
            g_ = geo()
            report.setdefault("push_end_cable_mm", []).append(
                [[round(float(v) / MM, 1) for v in p_] for p_ in g_["starts"]])
            report.setdefault("push_end_hand", []).append([round(float(v), 4) for v in tcp_measured(arm)])
        # hold it further back, the hand tilted toward its own side (+y) about the cable: the nut hand needs room
        # (the grippers and forearms met at the gland, demo runs 2-6)
        yield from move(arm, arm.pos, 0.5, grip=OPEN_CABLE, label="let go to hold")
        yield from move(arm, arm.pos + np.array([0.0, 0.0, 0.03]), 0.6, label="up to hold")
        report["inserted"] = {"sheath_x_mm": round(float(geo()["sheath_end"][0]) / MM, 1), "step": world["step"]}
        if TARGET_PAST[variant] is None:
            flags["inserted"] = True
            yield from move(arm, arm.pos + np.array([0.0, 0.0, 0.05]), 1.0, label="up")
            yield from move(arm, home, 2.0, rot=home_rot, label="home")
        else:
            back = float(geo()["sheath_end"][0]) - (cg.NUT_X0 - cg.NUT_DOME_LEN) + HOLD_ALONG
            yield from grip_cable_at(arm, back, "hold the cable", tilt=HOLD_TILT, tilt_toward_y=1.0, lean=False)
            flags["inserted"] = True
            yield from wait_for(arm, "nut_done", "hold the cable")
            yield from move(arm, arm.pos, 0.5, grip=OPEN_CABLE, label="let go")
            yield from move(arm, arm.pos + np.array([0.0, 0.0, 0.08]), 1.0, label="up")
            yield from move(arm, home, 2.0, rot=home_rot, label="home")
        flags["left_done"] = True
        phase["left"] = "done"
        while True:
            yield

    # -- the cap nut -------------------------------------------------------------------------------------------------
    def nut_hand(arm: Arm, roll: float, out_by: float = 0.0):
        """TCP and rotation for the hand on the nut: leaned back NUT_LEAN toward the arms (its upper part clear of the
        box wall), turned ``roll`` about the axis, its TCP on its own axis NUT_TCP_UP (+ ``out_by``) from the nut's."""
        _, _, c = nut()
        r = rot_x(roll) @ rot_y(NUT_LEAN) @ arm.down
        up = r @ arm.down.T @ np.array([0.0, 0.0, 1.0])
        return c + (NUT_TCP_UP + out_by) * up, r

    def flats_roll(lo: float) -> float:
        """The hand roll in [lo, lo + 60 deg) that lines its fingers up with two flats."""
        _, spin, _ = nut()
        a = spin % (math.pi / 3)
        while a < lo:
            a += math.pi / 3
        while a - math.pi / 3 >= lo:
            a -= math.pi / 3
        return a

    def turn_nut(arm: Arm, target: float):
        """Wrist strokes about the gland's axis (clockwise seen from the arms), re-gripping between them, until the nut
        has turned ``target`` turns past the seal's first contact."""
        roll = flats_roll(ROLL_MIN)
        p, r = nut_hand(arm, roll, APPROACH)
        yield from move(arm, p + np.array([0.0, 0.0, 0.03]), 2.0, grip=OPEN_NUT, rot=r, label="above the nut")
        strokes = 0
        while strokes < MAX_STROKES:
            g = world["states"]["gland"]
            if g.contact_turns is not None and g.past_contact >= target:
                break
            roll = flats_roll(ROLL_MIN)
            p, r = nut_hand(arm, roll, APPROACH)
            yield from move(arm, p, 0.6, grip=OPEN_NUT, rot=r, label=f"stroke {strokes}: out")
            p, r = nut_hand(arm, roll)
            yield from move(arm, p, 0.5, label=f"stroke {strokes}: in")
            yield from settle_at(arm, f"stroke {strokes}: settle", tol=1.0 * MM, max_s=1.0)
            yield from move(arm, arm.pos, 0.4, grip=GRIP_NUT, label=f"stroke {strokes}: grip")
            a = roll
            phase[arm.side] = f"stroke {strokes}: turn"
            while a < ROLL_MAX:
                a = min(ROLL_MAX, a + STROKE_RATE * DT)
                arm.pos, arm.rot = nut_hand(arm, a)
                yield
                g = world["states"]["gland"]
                if g.contact_turns is not None and g.past_contact >= target:
                    break
            yield from wait(arm, 0.3, f"stroke {strokes}: hold")
            yield from move(arm, arm.pos, 0.4, grip=OPEN_NUT, label=f"stroke {strokes}: open")
            p, r = nut_hand(arm, a, APPROACH)
            yield from move(arm, p, 0.5, label=f"stroke {strokes}: back out")
            strokes += 1
        g = world["states"]["gland"]
        report["nut"] = {"strokes": strokes, "turns": round(float(g.turns), 3),
                         "past_contact": round(float(g.past_contact), 3), "contact_turns": g.contact_turns,
                         "crushed": bool(g.crushed)}

    # -- right arm: turns the cap nut -------------------------------------------------------------------------------
    def right_script():
        arm = arms["right"]
        home, home_rot = arm.pos.copy(), arm.rot.copy()
        yield from wait(arm, 0.5, "start")
        yield from wait_for(arm, "inserted", "wait for the cable")
        if TARGET_PAST[variant] is not None:
            yield from turn_nut(arm, TARGET_PAST[variant])
        flags["nut_done"] = True
        yield from move(arm, arm.pos + np.array([0.0, 0.0, 0.06]), 1.0, rot=arm.down, label="up")
        yield from move(arm, home, 2.0, rot=home_rot, label="home")
        flags["right_done"] = True
        phase["right"] = "done"
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
        g, c = st["gland"], st["cable"]
        row = {"step": step, "left": phase["left"], "right": phase["right"], "score": res.info.get("score"),
               "success": res.info.get("success"), "turns": round(float(g.turns), 4),
               "past": round(float(g.past_contact), 4), "contact": g.contact_turns, "crushed": bool(g.crushed),
               "defl_mm": round(float(g.pad_deflection) / MM, 3), "pull": g.pull,
               "pull_moved_mm": round(float(g.pull_moved) / MM, 2), "hands_off": int(g.hands_off_steps),
               "c_v": round(float(np.linalg.norm(c.lin_vel)), 4), "c_w": round(float(np.linalg.norm(c.ang_vel)), 3),
               "g_w": round(float(np.linalg.norm(g.ang_vel)), 3),
               "through": bool(c.through), "sheath_mm": round(float(c.sheath_x) / MM, 2),
               "twist_deg": round(float(c.twist_deg), 1), "roll_deg": round(float(c.roll_deg), 1), "tip_mm": round(float(c.tip_x) / MM, 1),
               "l_ik_mm": round(arms["left"].ik_err[0] / MM, 2), "r_ik_mm": round(arms["right"].ik_err[0] / MM, 2),
               "grip_meas": [round(float(v), 4) for v in (split(np.asarray(world["obs"].state["joint_pos"]))[0][6],
                                                          split(np.asarray(world["obs"].state["joint_pos"]))[1][6])],
               "hand_mm": round(cg._TRACKS["gland"].hand_dist / MM, 1) if "gland" in cg._TRACKS else None,
               "r_meas": [round(float(v), 4) for v in tcp_measured(arms["right"])],
               "l_meas": [round(float(v), 4) for v in tcp_measured(arms["left"])],
               "l_qerr_deg": [round(math.degrees(float(v)), 1) for v in (
                   arms["left"].q - split(np.asarray(world["obs"].state["joint_pos"]))[0][:6])],
               "r_qerr_deg": [round(math.degrees(float(v)), 1) for v in (
                   arms["right"].q - split(np.asarray(world["obs"].state["joint_pos"]))[1][:6])],
               "nut_c": [round(float(v), 4) for v in nut()[2]],
               "l_cmd": [round(float(v), 4) for v in arms["left"].pos],
               "r_cmd": [round(float(v), 4) for v in arms["right"].pos]}
        log.append(row)
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
        if STOP_AFTER_STROKES and phase["right"].startswith(f"stroke {STOP_AFTER_STROKES}:"):
            report["stopped"] = f"after {STOP_AFTER_STROKES} strokes (debug)"
            break
        if res.terminated and success_step is None:
            success_step = step
            report["success_step"] = step
        if success_step is not None and step - success_step >= round(AFTER_SUCCESS_S / DT):
            break
        if (success_step is None and flags.get("left_done") and flags.get("right_done")
                and g.pull in ("passed", "failed", "none") and g.hands_off_steps >= round(6.0 / DT)):
            break  # both hands away for 6 s: the pull test has had its chance
    st = world["states"]
    success = bool(success_step is not None)
    g, c = st["gland"], st["cable"]
    summary = {
        "variant": variant,
        "task_score": 1.0 if success else float(result.info.get("score", 0.0)) if result is not None else 0.0,
        "success_step": success_step,
        "steps": len(log),
        "run_limit_steps": MAX_STEPS,
        "final": {"inserted": task.inserted(st), "snug": task.snug(st), "holds": task.holds(st),
                  "crushed": bool(g.crushed), "past_contact": float(g.past_contact), "pull": g.pull,
                  "pull_moved_mm": float(g.pull_moved) / MM, "twist_deg": float(c.twist_deg),
                  "roll_deg": float(c.roll_deg),
                  "sheath_x_mm": float(c.sheath_x) / MM, "through": bool(c.through)},
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
    ap.add_argument("--past", type=float, default=None, help="override the variant's turns past contact (experiments)")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a constant: --set PUSH_SPEED=0.01, or a part's with --set cg.PAD_K=3e4")
    args = ap.parse_args(argv)
    for item in args.set:
        name, value = item.split("=", 1)
        if name.startswith("cg."):
            sys.path.insert(0, str(HERE))
            import cg_parts

            target = vars(cg_parts)
        else:
            target = globals()
        name = name.removeprefix("cg.")
        if name not in target:
            ap.error(f"--set: no constant {name!r}")
        target[name] = ast.literal_eval(value)
    if args.past is not None and args.variant != "all":
        TARGET_PAST[args.variant] = args.past
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
