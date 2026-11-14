"""Scripted two-arm demo for the end-bracket task, closed loop on the simulator's object poses.

    python demo.py --variant full       # bracket slid on, pushed flush, screw tightened: 1.0
    python demo.py --variant no_hold    # the same, the rail never held: the rail moves, score capped at 0.25
    python demo.py --variant no_screw   # slid on and pushed flush, never tightened: 0.75
    python demo.py --variant no_push    # slid on, stopped 20 mm short of the block and tightened there: 0.5
    python demo.py --variant all --no-video

It drives the ``yam_arena`` embodiment directly (20 Hz control, object_jitter 0, as a trial runs) and reads the
task's own scoring from each step's ``info``. Each arm follows Cartesian targets turned into joint targets by
yam_arena's IK; the targets are corrected every step from the measured bracket, screwdriver and rail poses.

The choreography (a two-finger gripper can't turn a screwdriver it holds across the handle, and the bracket's open
screw recess doesn't hold a screwdriver upright, so the arms hand the screwdriver over and swap jobs):

1. left arm: holds the rail down (closed fingers pressed into its channel, left of the beige block);
2. right arm: picks the standing bracket up from above, pinched across its thickness, lines it up with the rail's
   free right end (0.4 mm above the lips, its hook under the near lip), slides it on, lowers it onto the lips, slides
   it along the rail to 15 mm short of the beige block, lets go, and pushes it flush with its closed fingers (squaring
   it on the two ends' measured gaps);
3. right arm: picks the screwdriver up across its handle, turns it upright and holds it over the bracket's screw;
4. left arm: lets go of the rail and takes the screwdriver from above, coaxially (its handle's top in the palm, the
   grasp yaw chosen so the wrist has the longest stroke); right arm: lets go, moves clear, and holds the rail down
   right behind the bracket, leaned 30 degrees, its pads a stop that keeps the bracket square against the block
   (the screw's friction twists a loose bracket within its guides' play otherwise);
5. left arm: centres the blade over the slot on the measured blade tip, settling between corrections (the wrist is
   soft), lowers it in, and tightens in wrist strokes, pressing down; between strokes it lifts the blade out, turns
   back in the air and re-engages (choosing the blade's way round that gives the longest next stroke); it stops when
   the bracket is tight, lifts the screwdriver out and lays it down on the table;
6. both arms move away; the run continues 3 s, hands off, to check the bracket stays flush and tight.

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
    "Hold the DIN rail down with one arm. With the other, put the end bracket on the rail against the right end of "
    "the row of beige terminal blocks, push it flush, and tighten its screw with the screwdriver to 0.5 Nm."
)
VARIANTS = ("full", "no_hold", "no_screw", "no_push")
CAMERAS = ("top_cam", "left_cam", "right_cam")
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
MAX_STEPS = 3600
MM = 1e-3

# -- grasps and gripper ------------------------------------------------------------------------------------------
PAD_TIP = 0.013  # pad tips beyond the TCP
OPEN_BRACKET = 0.30  # gripper opening (0..1 of 94 mm) for the bracket: ~28 mm between the pads (it is 8 mm thick)
PAD_THICK = 6.4 * MM  # each YAM finger pad: 6.4 mm thick (closing direction), 27 mm wide, tips 12.8 mm past the TCP
OPEN_DRIVER = 0.45  # ~42 mm, for the 26 mm handle
LOOSE_DRIVER = 0.30  # ~28 mm: loose round the 26 mm handle between strokes (it stays upright, its blade in the slot)
BRACKET_TIPS_Z = 12 * MM  # picking the standing bracket: the pad tips this far above its seat (19 mm above the table);
# the pads then cover its faces from 12 to 32 mm, under the screw recess
DRIVER_GRIP_Z = 0.040  # the right hand's pads are centred this far from the blade tip (the handle's lower part) ...
DRIVER_TOP_GRIP = 0.057  # ... and the left hand's pad tips reach down to here (its upper part; the right pads end at 53.5)
DRIVER_TIP_CLEAR = 8.0 * MM  # pad tips this far below the handle's axis when picking the lying screwdriver (5 mm
# above the table)

# -- the bracket: slide-on from the rail's free right end ---------------------------------------------------------
# Tilting it on about its hook doesn't fit this geometry in rigid-body physics (at a small tilt the far guide catches
# the lip, at a larger one its skirts, 0.5 mm above the table, dig into it); a real one flexes. Sliding on from
# the free end is the other way an electrician fits one. Level, it fits with 0.4 mm above and under the lip
# (ENTRY_LIFT, the hook's play is 0.8) and 0.2 mm beside each guide.
ENTRY_OUT = 6 * MM  # lined up this far beyond the rail's end ...
ENTRY_LIFT = 0.4 * MM  # ... with its seat this far above the lips
ENTRY_IN = 15 * MM  # slid this far on before it is lowered onto the lips
SLIDE_SPEED = 10 * MM  # per second, onto the rail and along it
SERVO_GAIN, SERVO_LIMIT = 0.4, 4 * MM  # correcting the hand on the measured bracket pose
SERVO_TURN = math.radians(4.0)  # ... and its yaw and roll, at most this
SLIDE_LIFT = 0.25 * MM  # sliding along the rail held, its seat this far above the lips (the hook doesn't drag)
SERVO_DOWN = 0.5 * MM  # ... but at most this far downward (on the lips, a lower target only presses it into them)
NO_PUSH_SHORT = 20.0 * MM  # no_push: it stays this far short of the beige block
HELD_SHORT = 15 * MM  # held, it is slid to this far short of the beige block (the block-side finger is in the way)
PUSH_FAST, PUSH_SLOW, SLOW_ZONE = 8 * MM, 3 * MM, 3 * MM  # pushing it on: per second, slow for the last 3 mm
PUSH_HEIGHT = 4 * MM  # the closed pads' tips this far above the lips while pushing
PUSH_LEAN = math.radians(25.0)  # pushing, the hand leans back so the pad tips lead
STOP_GAP = -3.0 * MM  # the right hand's pads, by estimate, this far behind the bracket's face (negative: into it; the
# leaned pad reaches a few mm short of the estimate, as when pushing; the hand presses the rail, so it can't slide)
PAD_HALF_WIDTH = 13.2 * MM  # half the YAM pad's width
PUSH_LEAD = 6 * MM  # the push command may lead the bracket's face by this much (the leaned pads reach a few mm short of the
# estimate; the bracket slides at 0.1 N, so once touched it moves and the lead stays small)
SQUARE_GAIN, SQUARE_REACH = 4.0, 8 * MM  # push this far off-centre per mm the two ends' gaps differ, at most this
BACK_OFF = 8 * MM
DONE_GAP = 0.3 * MM
LEAD = 4 * MM  # sliding it held: the command may lead the bracket's measured position by this much

# -- the screwdriver -----------------------------------------------------------------------------------------
PRESENT_ABOVE = 10 * MM  # the right hand presents the blade this far above the screw head (lower: easier to reach)
ENGAGE_TOL = 0.4 * MM  # engaged: the blade tip within this of the slot's bottom
SPIRAL_R, SPIRAL_TURNS, SPIRAL_S = 1.6 * MM, 3.0, 4.0  # slot search on the head: radius, turns, seconds
HOVER = 8 * MM  # the blade is centred on the slot this far above it (clear of the screw recess, whose top is 5.8 mm
# above the slot's bottom), then lowered straight in
CENTRE_TOL = 0.3 * MM  # ... to within this (the chamfered slot catches a blade within ~0.5 mm)
NEAR = 2 * MM  # ... then from just above the head (the head's top is 1.1 mm above the slot's bottom)
CENTRE_TRIES, CENTRE_SETTLE = 5, 0.5  # correct-and-settle rounds, and the settling time (s) before measuring
PRESS = 4.0 * MM  # while turning, the TCP is commanded this far below where the blade sits (presses it into the slot)
TOWARD_BLOCK = 1.0 * MM  # while turning, the hand is commanded this far toward the beige block
TIGHT_MARGIN_TURNS = 0.15  # the demo turns this far past the scorer's TIGHT_TURNS
PRESS_RAMP_S = 0.3  # the press is ramped in over this before a stroke turns
MAX_SLIPS = 8
SLIP_DROP = 0.6 * MM  # the blade tip this far below the slot's bottom: it has slipped off the head
CAM_OUT = 0.6 * MM  # ... this far above it: it has climbed out of the slot (cam-out)
TURN_RATE = math.radians(120.0)  # wrist turning speed while tightening
RATCHET_RATE = math.radians(120.0)  # ... and turning back in the air between strokes
TURN_LEAD = math.radians(10.0)  # the command may lead the screwdriver's measured angle by this much
WRIST_LIMIT = math.radians(95.0)  # joint 6 is +-120 degrees: a stroke ends here (the wrist overruns ~10 degrees)
WRIST_JOINT_LIMIT = math.radians(120.0)
WRIST_MARGIN = math.radians(25.0)  # a stroke starts this far inside the limit (room to align and wiggle the blade)

# -- rail holds ----------------------------------------------------------------------------------------------
LEFT_HOLD_Y = 0.100  # along the rail: the left hand holds left of the beige block, near the rail's end (the right
# hand's fingers reach ~5 cm ahead of the bracket along the rail) ...
RIGHT_HOLD_Y = -0.100  # ... the right hand right of the bracket, far enough from the screwing hand's sweep
HOLD_PRESS = 8 * MM  # the fingertips are commanded this far into the rail's web
HOLD_LEAN = math.radians(30.0)  # the holding hand leans along the rail, its wrist away from the middle

AFTER_SUCCESS_S = 3.0
DRIVER_PUT_DOWN = (0.22, 0.17)  # where the left hand lays the screwdriver down afterwards (on the table, left)
START_AT = "start"  # --start-at
ABORT_EARLY = True  # stop a run as soon as it can no longer succeed
IK_ABORT_MM, IK_ABORT_S = 20.0, 2.0  # an arm this far from its target for this long


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_half(a: float) -> float:
    """An angle modulo pi, in [-pi/2, pi/2): a blade fits its slot either way round."""
    return (a + math.pi / 2) % math.pi - math.pi / 2


class Arm:
    """One arm's commanded TCP pose and gripper, turned into joint targets by IK (warm-started)."""

    def __init__(self, side: str, base, q0, ik, grip_rotation):
        self.side, self.base, self.ik = side, np.asarray(base, dtype=float), ik
        self.q = np.array(q0, dtype=float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.grip = 1.0
        self.down = grip_rotation(0.0)  # pointing down, fingers closing along world y

    def action(self) -> np.ndarray:
        q, pe, re = self.ik.solve(self.pos, self.ik.matrix_to_quat_xyzw(self.rot), self.q, base_pos=self.base, iters=60)
        self.q, self.ik_err = q, (pe, re)
        return np.r_[q, self.grip]


def screwdriver_start_task() -> Path:
    """A development task file for ``--start-at screwdriver``: task.py unchanged except that the bracket starts
    seated on the rail, flush against the beige block (where the validated bracket half of the demo leaves it), so the
    screwdriver half can be iterated without re-running the first ~750 steps. Never used for scoring a submission."""
    import tempfile

    path = Path(tempfile.mkdtemp(prefix="end_bracket_dev_")) / "task_screwdriver_start.py"
    path.write_text(f'''"""Development start state (see demo.screwdriver_start_task); the real task is task.py."""
import sys
sys.path.insert(0, {str(HERE)!r})
import task as _base

globals().update({{k: v for k, v in vars(_base).items() if not k.startswith("__")}})
__doc__ = _base.__doc__
_ep = _base.ep
_slot = _ep.BEIGE_FACE_Y - _ep.BRACKET_WIDTH / 2 - 0.2 * _ep.MM
_pos = (_base.RAIL_ORIGIN[0], _base.RAIL_ORIGIN[1] + _slot, _base.RAIL_ORIGIN[2] + _ep.RAIL_HEIGHT + 0.05 * _ep.MM)
OBJECTS = [o if o.name != "bracket" else _ep.part("bracket", "bracket", _pos) for o in _base.OBJECTS]
''')
    return path


def make_embodiment(video: bool, device: str = "cpu"):
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    size = (360, 640) if video else (64, 64)
    task_path = screwdriver_start_task() if START_AT == "screwdriver" else TASK
    return YamArenaEmbodiment(task=str(task_path), control_hz=CONTROL_HZ, object_jitter=0.0, cam_height=size[0],
                              cam_width=size[1], device=device)


def set_rendering(emb, on: bool) -> None:
    """Turn the RTX render that follows every control step on or off (the physics and scoring are unaffected)."""
    emb._env.unwrapped.cfg.sim.render_interval = emb._env.unwrapped.cfg.decimation if on else 10**9


def run(variant: str, out: Path, video: bool, max_steps: int, emb=None) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack, split

    from yam_arena import ik, station
    from yam_arena.demo import grip_rotation

    emb = emb or make_embodiment(video)
    obs = emb.reset(Scene(id=f"end-bracket-{variant}", instruction=INSTRUCTION), seed=0)
    set_rendering(emb, video)
    task = emb.task.module
    ep = task.ep
    from yam_arena.scene import object_states  # patched by the task to cover the jointed bracket

    left_q, right_q = split(np.asarray(obs.state["joint_pos"]))
    arms = {
        "left": Arm("left", station.BASES["left"], left_q[:6], ik, grip_rotation),
        "right": Arm("right", station.BASES["right"], right_q[:6], ik, grip_rotation),
    }
    world = {"states": object_states(emb._env, emb.task.names), "obs": obs, "step": 0}
    flags: dict = {}  # hand-over signals between the two scripts
    report: dict = {}
    phase = {"left": "", "right": ""}

    # -- measurements -------------------------------------------------------------------------------------------
    def rail():
        return task.rail_pose(world["states"])

    def rail_point(local) -> np.ndarray:
        p, r = rail()
        return p + r @ np.asarray(local, dtype=float)

    def bracket():
        """(origin, rotation) of the bracket, measured."""
        return task._origin(world["states"]["bracket"], task.C_BRACKET)

    def driver():
        """(origin = blade tip, rotation) of the screwdriver, measured."""
        st = world["states"]["driver"]
        r = task.quat_xyzw_to_matrix(np.asarray(st.quat_xyzw, dtype=float))
        return np.asarray(st.pos, dtype=float) - r @ np.array(ep.local_center("driver")), r

    def measured_tcp(side: str):
        left, right = split(np.asarray(world["obs"].state["joint_pos"]))
        q = (left if side == "left" else right)[:6]
        return ik.tcp_pose(q, arms[side].base)

    def finger_tips(side: str) -> list:
        """World positions (m) of the two finger links' origins, for diagnostics."""
        art = emb._env.unwrapped.scene.articulations.get(f"{side}_robot")
        if art is None:
            return []
        names = list(art.body_names)
        pos = art.data.body_pos_w.torch[0].detach().cpu().numpy()
        org = emb._env.unwrapped.scene.env_origins[0].detach().cpu().numpy()
        return [[round(float(v), 4) for v in pos[names.index(n)] - org] for n in ("tip_left", "tip_right") if n in names]

    def wrist_angle(side: str) -> float:
        left, right = split(np.asarray(world["obs"].state["joint_pos"]))
        return float((left if side == "left" else right)[5])

    def best_wrist_yaw(arm: Arm, tcp_at, align: float, grasp_at=None) -> float:
        """The grasp yaw (about the vertical, of a straight-down hand) that, after a further turn by ``align``,
        leaves joint 6 nearest the end of its range that tightening (clockwise from above) turns away from; with
        ``grasp_at``, only yaws at which the hand also reaches the grasp there."""
        best, best_q6 = 0.0, None
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
            if q6_tight is None:  # which way does joint 6 move when the hand turns clockwise?
                q2, _, _ = ik.solve(np.asarray(tcp_at), ik.matrix_to_quat_xyzw(rot_z(-0.2) @ r), q, base_pos=arm.base,
                                    iters=80)
                q6_tight = 1.0 if q2[5] > q[5] else -1.0
            # nearest to WRIST_MARGIN inside the end of the range that tightening moves away from: room for the
            # longest stroke, and still room to turn the blade onto the slot and to wiggle it in
            score = -abs(q[5] - (-q6_tight * (WRIST_JOINT_LIMIT - WRIST_MARGIN)))
            if best_q6 is None or score > best_q6:
                best, best_q6 = yaw, score
        report["wrist_q6_sign"] = q6_tight
        return best

    # -- motions -----------------------------------------------------------------------------------------------
    def move(arm: Arm, pos, duration: float, grip=None, rot=None, label=""):
        """Smoothly to ``pos`` (and ``rot``, ``grip``) over ``duration`` seconds."""
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

    def wait_for(arm: Arm, flag: str, label=""):
        phase[arm.side] = label
        while not flags.get(flag):
            yield

    def hold_rail(arm: Arm, along: float, label: str, outward: float, press_down: bool = False):
        """Closed fingers pressed into the rail's channel at ``along``, closing across the rail, the wrist camera
        (mounted off one side of the hand) turned toward ``outward`` (+1: +y, -1: -y) so it stays clear of the work
        in the middle of the rail."""
        _, r = rail()
        # leaned along the rail, the wrist away from the middle: upright, the two arms' wrist housings meet above the
        # work while the other hand is in the middle of the rail
        down = rot_x(-outward * HOLD_LEAN) @ rot_z(math.atan2(r[1, 0], r[0, 0]) + outward * math.pi / 2) @ arm.down
        top = rail_point((0.0, along, ep.SHEET))
        tcp = top - PAD_TIP * down[:, 2]
        approach = down[:, 2]
        if press_down:  # beside the work: come in low from behind along the rail, not down past the screwdriver
            behind = tcp + outward * 0.06 * r[:, 1] + (0, 0, 0.02)
            yield from move(arm, behind + (0, 0, 0.03), 1.5, grip=0.0, rot=down, label=f"{label}: behind, along the rail")
            yield from move(arm, behind, 0.8, label=f"{label}: down behind")
            yield from move(arm, tcp + (0, 0, 0.012), 1.5, label=f"{label}: in along the rail")
        else:
            yield from move(arm, tcp + (0, 0, 0.06), 2.0, grip=0.0, rot=down, label=f"{label}: over the rail")
        yield from move(arm, tcp - 0.010 * approach, 1.2, label=f"{label}: down to the rail")
        press = np.array([0.0, 0.0, -HOLD_PRESS]) if press_down else HOLD_PRESS * approach
        yield from move(arm, tcp + press, 0.8, label=f"{label}: press the rail down")
        phase[arm.side] = f"{label}: holding the rail"

    def release_rail(arm: Arm, label: str):
        yield from move(arm, arm.pos + (0, 0, 0.06), 1.0, label=f"{label}: off the rail")

    # -- the hand pose that puts a held part at a pose (from the grip's measured offset) -------------------------
    def hand_for(r_target, p_target, offset):
        """Hand (rotation, TCP) that puts ``part`` at (``r_target``, ``p_target``), with the grip ``offset``."""
        r_off, p_off = offset
        r_hand = r_target @ r_off.T
        return r_hand, p_target - r_hand @ p_off

    def measure_offset(part, side: str):
        p_part, r_part = part()
        arm = arms[side]
        return arm.rot.T @ r_part, arm.rot.T @ (p_part - arm.pos)

    # -- left arm ----------------------------------------------------------------------------------------------
    def left_script():
        arm = arms["left"]
        if variant != "no_hold":
            yield from hold_rail(arm, LEFT_HOLD_Y, "left", outward=1.0)
        else:
            phase["left"] = "ready (no rail hold)"
        if variant == "no_screw":
            while True:
                yield
        yield from wait_for(arm, "driver_presented", "waiting for the screwdriver")
        if variant != "no_hold":
            yield from release_rail(arm, "left")

        # take the screwdriver from above, coaxially: the handle's top in the palm, fingers closing along world y
        tip, rd = driver()
        axis = rd[:, 2]
        # the handle is round, so any yaw grips it; choose the one that leaves the wrist (joint 6) at the end of its
        # range opposite to the tightening direction once the blade is turned onto the slot, for the longest strokes
        pb, rb = bracket()
        turns0 = float(world["states"]["bracket"].screw_turns)
        slot_dir = rb @ (rot_z(-2 * math.pi * turns0) @ np.array([1.0, 0.0, 0.0]))
        align = wrap_half(math.atan2(slot_dir[1], slot_dir[0]) - math.atan2(rd[1, 0], rd[0, 0]))
        screw_at = pb + rb @ np.array([0.0, 0.0, ep.RECESS_FLOOR + DRIVER_TOP_GRIP + PAD_TIP])
        tips_at = tip + DRIVER_TOP_GRIP * axis
        grasp_tcp = tips_at + np.array([0.0, 0.0, PAD_TIP])  # pointing down: the TCP is PAD_TIP above the tips
        yaw = best_wrist_yaw(arm, screw_at, align, grasp_at=grasp_tcp)
        top_down = rot_z(yaw) @ arm.down
        tcp = tips_at - PAD_TIP * top_down[:, 2]
        yield from move(arm, tcp + (0, 0, 0.05), 2.5, grip=OPEN_DRIVER, rot=top_down, label="over the screwdriver")
        tip, rd = driver()
        tcp = tip + DRIVER_TOP_GRIP * rd[:, 2] - PAD_TIP * top_down[:, 2]
        yield from move(arm, tcp, 1.2, label="down round the handle")
        yield from move(arm, tcp, 0.8, grip=0.0, label="grip the handle")
        flags["left_has_driver"] = True
        yield from wait_for(arm, "right_holding", "waiting for the right hand to hold the rail")

        # into the slot: align the blade on the measured slot, lower it to 3 mm above, then engage
        def slot():
            """(slot centre at its bottom, slot direction angle in the world) of the bracket's screw."""
            pb, rb = bracket()
            turns = float(world["states"]["bracket"].screw_turns)
            z = ep.RECESS_FLOOR + 0.2 * MM + ep.SCREW_HEAD_H - ep.SLOT_DEPTH
            direction = rb @ (rot_z(-2 * math.pi * turns) @ np.array([1.0, 0.0, 0.0]))
            return pb + rb @ np.array([0.0, 0.0, z]), math.atan2(direction[1], direction[0])

        def blade_angle() -> float:
            _, r = driver()
            return math.atan2(r[1, 0], r[0, 0])

        def tcp_for_tip(target_tip):
            tip, _ = driver()
            return arm.pos + (np.asarray(target_tip) - tip)

        # tighten in strokes: turn clockwise (seen from above), pressing down; between strokes (or after a slip) lift the
        # blade out, turn back in the air and re-engage; stop when the bracket is tight
        phase["left"] = "tighten"
        report["strokes"] = 0
        report["slips"] = 0

        def engage(label):
            """Align the blade on the slot, centre it on the screw and lower it in (wiggling if it sits on the head).
            Leaves the hand where the blade just sits in the slot; returns whether it engaged (in report)."""
            bottom, ang = slot()
            # the blade fits the slot either way round: of the turns that align it, take the one that leaves the wrist
            # (joint 6) nearest the start of its range for tightening, so the next stroke is as long as possible.
            # Turning the hand by phi about the vertical moves joint 6 by about -sign * phi
            sign = report.get("wrist_q6_sign", 1.0)
            # the largest turn back that keeps joint 6 at least 10 degrees inside its limit (the choices are 180 apart)
            room = sign * wrist_angle("left") + (WRIST_JOINT_LIMIT - math.radians(10.0))
            base_turn = wrap_half(ang - blade_angle())
            want = sign * wrist_angle("left") + (WRIST_JOINT_LIMIT - WRIST_MARGIN)  # joint 6 at the start of its range
            fits = [base_turn + k * math.pi for k in (-2, -1, 0, 1, 2) if base_turn + k * math.pi <= room]
            phi = min(fits, key=lambda p: abs(p - want)) if fits else base_turn
            if driver()[0][2] > bottom[2] + 0.015:  # coming from above: line up 30 mm over it first, then go down
                yield from move(arm, tcp_for_tip(bottom + (0, 0, 0.03)), 1.5, label=f"{label}: over the screw")
                for _ in range(round(0.5 / DT)):
                    arm.pos = arm.pos + np.clip(0.5 * ((slot()[0] + (0, 0, 0.03)) - driver()[0]), -0.002, 0.002)
                    yield
            # turn by phi explicitly about the vertical (an orientation slerp takes the short way round, which for a
            # turn past 180 degrees drives the wrist into its limit), moving to the hover meanwhile
            r_from, p_from = arm.rot.copy(), arm.pos.copy()
            p_to = tcp_for_tip(slot()[0] + (0, 0, HOVER))
            n = max(1, round(max(1.0, abs(phi) / RATCHET_RATE) / DT))
            for i in range(1, n + 1):
                s = smooth(i / n)
                arm.rot = rot_z(phi * s) @ r_from
                arm.pos = p_from + s * (p_to - p_from)
                phase["left"] = f"{label}: over the slot"
                yield
            # centre the blade tip on the slot while it hovers (the screwdriver leans ~1 degree in the grip, which
            # puts its tip ~1.5 mm off; once it touches the head, friction pins it)
            # correct, then let the arm settle before measuring again (the wrist is soft, 10 Nm/rad, and lags its
            # command by 1-2 mm: correcting every step makes it oscillate)
            def centre(height, tries, tag):
                """Correct-and-settle rounds that bring the blade tip over the slot at ``height`` above its bottom (the
                wrist is soft, 10 Nm/rad, and lags its command by 1-2 mm: correcting every step oscillates)."""
                phase["left"] = f"{label}: {tag}"
                err = np.zeros(3)
                for _ in range(tries):
                    for _ in range(round(CENTRE_SETTLE / DT)):
                        yield
                    errs = []
                    for _ in range(4):  # average over 0.2 s
                        bottom, ang = slot()
                        errs.append((bottom + (0, 0, height)) - driver()[0])
                        yield
                    err = np.mean(errs, axis=0)
                    if np.linalg.norm(err[:2]) < CENTRE_TOL:
                        break
                    arm.pos = arm.pos + np.clip(err, -0.003, 0.003)
                    arm.rot = rot_z(wrap_half(ang - blade_angle())) @ arm.rot
                report.setdefault("centre_err_mm", []).append(round(float(np.linalg.norm(err[:2])) / MM, 3))

            # centre high, clear of the recess; go down slowly to just above the head; centre again; then straight in
            yield from centre(HOVER, CENTRE_TRIES, "centre on the slot")
            yield from move(arm, arm.pos - np.array([0.0, 0.0, HOVER - NEAR]), 1.0, label=f"{label}: down to the head")
            yield from centre(NEAR, 2, "centre again near the head")
            _, ang = slot()
            report.setdefault("insert_errors", []).append({
                "xy_mm": [round(float(v) / MM, 3) for v in (slot()[0] - driver()[0])[:2]],
                "angle_deg": round(math.degrees(wrap_half(ang - blade_angle())), 2)})
            yield from move(arm, arm.pos - np.array([0.0, 0.0, NEAR + 0.0005]), 0.8, label=f"{label}: into the slot")
            ok = abs(driver()[0][2] - slot()[0][2]) < ENGAGE_TOL
            if not ok:
                # on the head beside the slot: press lightly and move the blade in a widening spiral until it drops in
                # (the slot runs the head's full width, so the search is across it; a person does the same)
                centre_xy, z_cmd = arm.pos[:2].copy(), arm.pos[2]
                n = round(SPIRAL_S / DT)
                for i in range(1, n + 1):
                    r_ = SPIRAL_R * i / n
                    a_ = 2 * math.pi * SPIRAL_TURNS * i / n
                    arm.pos = np.array([centre_xy[0] + r_ * math.cos(a_), centre_xy[1] + r_ * math.sin(a_), z_cmd])
                    phase["left"] = f"{label}: find the slot"
                    yield
                    if abs(driver()[0][2] - slot()[0][2]) < ENGAGE_TOL:
                        ok = True
                        break
                report.setdefault("spiral_hits", []).append(ok)
            report["engaged"] = ok

        for attempt in range(3):
            if attempt:
                yield from move(arm, arm.pos + (0, 0, 0.004), 0.3, label="lift for another try")
            yield from engage("engage")
            if report.get("engaged"):
                report["engaged_step"] = world["step"]
                break
        if not report.get("engaged"):
            report["gave_up"] = "the blade doesn't go into the slot"
            report["abort"] = report["gave_up"]
        while report["strokes"] < 12 and report.get("engaged"):
            report["strokes"] += 1
            r0 = arm.rot.copy()
            base = arm.pos.copy()  # the hand with the blade just sitting in the slot
            # press and lean in gradually before turning (a step change jolts the soft wrist and lifts the blade out)
            n_ramp = round(PRESS_RAMP_S / DT)
            for i in range(1, n_ramp + 1):
                s = smooth(i / n_ramp)
                arm.pos = np.array([base[0], base[1], base[2] - s * PRESS]) + s * TOWARD_BLOCK * rail()[1][:, 1]
                phase["left"] = f"tighten: stroke {report['strokes']} (press)"
                yield
            turned, last_meas, meas_total = 0.0, blade_angle(), 0.0
            phase["left"] = f"tighten: stroke {report['strokes']}"
            slipped = False
            while True:
                b = task.bracket_on_rail(world["states"])
                if task.tight(b) and b["screw_turns"] >= task.TIGHT_TURNS + TIGHT_MARGIN_TURNS:
                    report["tight_step"] = world["step"]
                    break
                a_ = blade_angle()
                meas_total += wrap(a_ - last_meas)
                last_meas = a_
                # clockwise from above is negative about z; don't let the command lead the blade by much
                turned = max(turned - TURN_RATE * DT, meas_total - TURN_LEAD)
                arm.rot = rot_z(turned) @ r0
                # the hand stays where the blade engaged (re-centring on the measured slot would follow a bracket
                # that tilts and pull it further); press PRESS below that
                bottom, _ = slot()
                tip, _ = driver()
                # press down, and lean a little toward the beige block: the bracket's face stays against the block
                # (else the screw's friction twists the loose bracket within its guides' play)
                arm.pos = np.array([base[0], base[1], base[2] - PRESS]) + TOWARD_BLOCK * rail()[1][:, 1]
                if tip[2] < bottom[2] - SLIP_DROP or tip[2] > bottom[2] + CAM_OUT:  # off the head, or out of the slot
                    slipped = True
                    break
                # a stroke ends near the wrist's limit that tightening turns toward (joint 6, +-120 degrees), or after
                # 200 degrees
                if report.get("wrist_q6_sign", 1.0) * wrist_angle("left") > WRIST_LIMIT or -turned > math.radians(200):
                    break
                yield
            if "tight_step" in report:
                break
            if slipped:
                report["slips"] += 1
                if report["slips"] >= MAX_SLIPS:
                    report["gave_up"] = f"slipped out of the slot {MAX_SLIPS} times"
                    break
            # next stroke: lift the blade out of the slot (still gripped firmly), turn back in the air, re-engage
            # (a loosened grip lets the screwdriver, balanced on its blade, fall over)
            yield from move(arm, base + (0, 0, 0.006), 0.4, label="lift out of the slot")
            report["engaged"] = False
            for attempt in range(3):
                yield from engage("next stroke")
                if report.get("engaged"):
                    break
                yield from move(arm, arm.pos + (0, 0, 0.004), 0.3, label="lift for another try")
            if not report.get("engaged"):
                report["gave_up"] = "the blade doesn't go back into the slot"
                break
        if "gave_up" in report:
            report["abort"] = report["gave_up"]
        yield from wait(arm, 0.5, "hold the torque")
        # lift the screwdriver out of the slot and lay it down on the table, clear of the work (left of the rail)
        yield from move(arm, arm.pos + (0, 0, PRESS + 0.03), 0.8, label="lift the screwdriver out")
        report["left_withdrawn_step"] = world["step"]
        yield from move(arm, np.array([DRIVER_PUT_DOWN[0], DRIVER_PUT_DOWN[1], arm.pos[2]]), 2.0,
                        label="carry the screwdriver away")
        yield from move(arm, arm.pos - np.array([0.0, 0.0, arm.pos[2] - task.TOP - 0.11]), 1.0,
                        label="down to the table")
        flags["screwed"] = True
        report["driver_released_step"] = world["step"]  # it falls over onto the table: fine
        yield from move(arm, arm.pos, 0.5, grip=OPEN_DRIVER, label="let go of the screwdriver")
        yield from move(arm, arm.pos + (0, 0, 0.06), 1.0, label="up")
        phase["left"] = "done"
        while True:
            yield

    # -- right arm ---------------------------------------------------------------------------------------------
    def right_script():
        arm = arms["right"]
        yield from wait(arm, 0.5, "start")
        if START_AT != "screwdriver":  # (--start-at screwdriver: the bracket is already flush on the rail)

            # 1. pick the standing bracket up from above, pinched across its 8 mm thickness on its two big faces (the YAM
            # pads are 27 mm wide: no other pair of the bracket's faces is flat and parallel over that width)
            pb, rb = bracket()
            b_yaw = math.atan2(rb[1, 0], rb[0, 0])
            pick = rot_z(b_yaw) @ arm.down  # fingers closing along the bracket's thickness (world y)
            tips_at = pb + rb @ np.array([0.0, 0.0, BRACKET_TIPS_Z])
            tcp = tips_at - PAD_TIP * pick[:, 2]
            yield from move(arm, tcp + (0, 0, 0.06), 2.5, grip=OPEN_BRACKET, rot=pick, label="over the bracket")
            yield from move(arm, tcp, 1.5, label="down round the bracket")
            yield from move(arm, tcp, 0.8, grip=0.0, label="grip the bracket")
            yield from move(arm, tcp + (0, 0, 0.05), 1.2, label="lift the bracket")
            report["lifted_step"] = world["step"]
            yield from wait(arm, 0.3, "settle")
            offset = measure_offset(bracket, "right")

            def hand_at(r_local, p_local):
                """The hand pose that puts the bracket at (r_local, p_local) in the rail's frame."""
                p_r, r_r = rail()
                return hand_for(r_r @ r_local, p_r + r_r @ p_local, offset)

            # 2. slide it on from the rail's free right end and along the rail, correcting the hand on the measured
            # bracket pose all the way, to HELD_SHORT short of the beige block (a finger is on its block side) or, for
            # no_push, NO_PUSH_SHORT short
            slot_y = ep.BEIGE_FACE_Y - ep.BRACKET_WIDTH / 2
            stop_y = slot_y - (NO_PUSH_SHORT if variant == "no_push" else HELD_SHORT)
            entry_y = -ep.RAIL_LENGTH / 2 - ep.BRACKET_WIDTH / 2 - ENTRY_OUT
            corr = np.zeros(3)
            turn = np.zeros(2)  # yaw and roll corrections (rad), in the rail's frame

            def follow(along, lift, label):
                """One control step toward the bracket at (along, lift) in the rail's frame, square and level, servoing
                on its measured position and on its yaw and roll (a twist wedges it between the guides like a drawer)."""
                nonlocal corr, turn
                p_r, r_r = rail()
                pbm, rbm = bracket()
                rel = r_r.T @ rbm
                yaw_err = (math.atan2(rel[1, 0], rel[0, 0]) + math.pi / 2) % math.pi - math.pi / 2
                roll_err = math.atan2(rel[0, 2], rel[2, 2])
                turn = np.clip(turn - SERVO_GAIN * np.array([yaw_err, roll_err]), -SERVO_TURN, SERVO_TURN)
                want = p_r + r_r @ np.array([0.0, along, ep.RAIL_HEIGHT + lift])
                r_h, p_h = hand_at(rot_z(turn[0]) @ rot_y(turn[1]), np.array([0.0, along, ep.RAIL_HEIGHT + lift]))
                corr = np.clip(corr + SERVO_GAIN * (want - pbm), -SERVO_LIMIT, SERVO_LIMIT)
                # the lips stop it going lower: don't let the height correction wind up into a hard press on the rail
                corr[2] = max(corr[2], -SERVO_DOWN)
                arm.pos, arm.rot = p_h + corr, r_h
                phase["right"] = label

            r_h, p_h = hand_at(np.eye(3), np.array([0.0, entry_y, ep.RAIL_HEIGHT + 0.02]))
            yield from move(arm, (p_h[0], p_h[1], max(arm.pos[2], p_h[2])), 2.5, rot=r_h, label="carry to the rail's end")
            yield from move(arm, p_h, 1.0, label="over the rail's end")
            r_h, p_h = hand_at(np.eye(3), np.array([0.0, entry_y, ep.RAIL_HEIGHT + ENTRY_LIFT]))
            yield from move(arm, p_h, 1.2, label="down to the rail's end")
            for _ in range(round(1.5 / DT)):  # line it up on its measured pose
                follow(entry_y, ENTRY_LIFT, "line up with the rail")
                yield
            report["lined_up"] = {k: round(float(v) / MM, 3) for k, v in zip(
                ("across_mm", "along_mm", "lift_mm"), rail()[1].T @ (bracket()[0] - rail_point((0, entry_y, ep.RAIL_HEIGHT + ENTRY_LIFT))))}
            along = entry_y
            while along < -ep.RAIL_LENGTH / 2 + ENTRY_IN:  # onto the rail, still lifted
                along += SLIDE_SPEED * DT
                follow(along, ENTRY_LIFT, "slide onto the rail")
                yield
            for i in range(1, round(0.8 / DT) + 1):  # down onto the lips
                follow(along, ENTRY_LIFT * (1 - smooth(i * DT / 0.8)), "down onto the lips")
                yield
            report["on_rail_step"] = world["step"]
            b = task.bracket_on_rail(world["states"])
            report["on_rail"] = {"stage": task.stage(world["states"]), "seat_mm": [s / MM for s in b["seat"]],
                                 "across_mm": b["across"] / MM, "along_mm": b["along"] / MM}
            t_start = world["step"]
            while world["step"] - t_start < round(20.0 / DT):  # along the rail
                b = task.bracket_on_rail(world["states"])
                if b["along"] >= stop_y - 0.5 * MM:
                    break
                along = min(along + SLIDE_SPEED * DT, b["along"] + LEAD, stop_y + 0.5 * MM)
                follow(along, SLIDE_LIFT, "slide along the rail")  # carried just clear of the lips
                yield
            for i in range(1, round(0.6 / DT) + 1):  # down onto the lips
                follow(along, SLIDE_LIFT * (1 - smooth(i * DT / 0.6)), "set it down")
                yield
            report["slid_step"] = world["step"]
            b = task.bracket_on_rail(world["states"])
            report["slid"] = {"stage": task.stage(world["states"]), "seat_mm": [s / MM for s in b["seat"]],
                              "across_mm": b["across"] / MM, "along_mm": b["along"] / MM, "yaw_deg": b["yaw_deg"]}
            # let go and lift clear
            yield from move(arm, arm.pos, 0.6, grip=OPEN_BRACKET, label="let go of the bracket")
            yield from move(arm, arm.pos + (0, 0, 0.05), 1.0, label="up")

            # 3. push it the rest of the way with the closed fingers, low on its rail-end face, squaring it as it goes
            if variant != "no_push":
                _, r_r = rail()
                ydir = r_r[:, 1]
                # closing along the rail (pads stacked along y), leaned back PUSH_LEAN so only the pad tips touch: higher up
                # the YAM finger widens (3.6 mm past the pad 16 mm up, 12.6 mm 33 mm up) and, upright, it pushes the
                # bracket high and tips it until it jams
                push_rot = rot_x(PUSH_LEAN) @ rot_z(math.atan2(r_r[1, 0], r_r[0, 0])) @ arm.down
                b = task.bracket_on_rail(world["states"])
                face_y = b["along"] - ep.BRACKET_WIDTH / 2

                def tcp_for(across, along_face, height):
                    """TCP that puts the closed pads' block-side face at (across, along_face) in the rail's frame and
                    their tips ``height`` above the lips."""
                    tips = rail_point((across, along_face - PAD_THICK, ep.RAIL_HEIGHT + height))
                    return tips - PAD_TIP * push_rot[:, 2]

                yield from move(arm, tcp_for(0.0, face_y - 4 * MM, PUSH_HEIGHT + 0.03), 1.5, grip=0.0, rot=push_rot,
                                label="behind the bracket")
                yield from move(arm, tcp_for(0.0, face_y - 4 * MM, PUSH_HEIGHT), 1.0, label="down behind the bracket")
                phase["right"] = "push flush"
                push_y, push_x = face_y - 4 * MM, 0.0
                t_start = world["step"]
                while world["step"] - t_start < round(15.0 / DT):
                    b = task.bracket_on_rail(world["states"])
                    gap = max(b["gaps"])
                    if all(g <= DONE_GAP for g in b["gaps"]):
                        break
                    speed = PUSH_SLOW if gap < SLOW_ZONE else PUSH_FAST
                    face_now = b["along"] - ep.BRACKET_WIDTH / 2
                    push_y = min(push_y + speed * DT, face_now + PUSH_LEAD)
                    g_p, g_n = max(b["gaps"][:2]), max(b["gaps"][2:])  # +x end, -x end
                    push_x = b["across"] + float(np.clip((g_p - g_n) * SQUARE_GAIN, -SQUARE_REACH, SQUARE_REACH))
                    arm.pos = tcp_for(push_x, push_y, PUSH_HEIGHT)
                    yield
                report["flush_step"] = world["step"]
                yield from move(arm, arm.pos - BACK_OFF * ydir, 0.3, label="back off")
                yield from move(arm, arm.pos + (0, 0, 0.05), 0.8, label="lift clear")
            if variant == "no_screw":
                yield from move(arm, arm.pos + (-0.06, -0.08, 0.0), 1.5, label="away")
                phase["right"] = "done"
                report["script_done_step"] = world["step"]
                while True:
                    yield

        # 4. pick the screwdriver up across the lower part of its handle
        tip, rd = driver()
        axis = rd[:, 2]
        grip_pt = tip + DRIVER_GRIP_Z * axis
        across = rot_z(math.atan2(axis[1], axis[0])) @ arm.down  # fingers closing across the handle
        across = rot_z(-math.pi / 2) @ across if abs(across[:, 0] @ axis) > 0.7 else across
        tips_z = max(grip_pt[2] - DRIVER_TIP_CLEAR, task.TOP + 3 * MM)  # the lying stubby's axis is low here
        tcp = np.array([grip_pt[0], grip_pt[1], tips_z]) - PAD_TIP * across[:, 2]
        yield from move(arm, tcp + (0, 0, 0.07), 3.0, grip=OPEN_DRIVER, rot=across, label="over the screwdriver")
        yield from move(arm, tcp, 1.5, label="down round the handle")
        yield from move(arm, tcp, 0.8, grip=0.0, label="grip the handle")
        yield from move(arm, tcp + (0, 0, 0.10), 1.5, label="lift the screwdriver")
        # turn it upright, blade down (the hand ends pointing forward, +x): the blade swings to 125 mm below the
        # hand, so go up as it turns
        upright = rot_y(-math.pi / 2) @ arm.rot
        yield from move(arm, arm.pos + (0.0, 0.05, 0.10), 2.0, rot=upright, label="turn the screwdriver upright")
        yield from wait(arm, 0.3, "settle")
        d_off = measure_offset(driver, "right")
        pb, rb = bracket()
        head = pb + rb @ np.array([0.0, 0.0, ep.RECESS_FLOOR + 0.2 * MM + ep.SCREW_HEAD_H])
        r_h, p_h = hand_for(driver()[1], head + (0, 0, PRESENT_ABOVE), d_off)
        yield from move(arm, p_h, 2.5, label="hold it over the screw")
        flags["driver_presented"] = True
        report["presented_step"] = world["step"]
        yield from wait_for(arm, "left_has_driver", "hand-over: waiting for the left hand")
        yield from move(arm, arm.pos, 0.6, grip=OPEN_DRIVER, label="hand-over: let go")
        yield from move(arm, arm.pos - 0.05 * arm.rot[:, 2], 1.0, label="back off")
        flags["right_let_go"] = True
        # clear of the screwdriver before turning the hand down for the hold
        yield from move(arm, arm.pos + (-0.05, -0.10, 0.04), 1.2, label="clear of the screwdriver")
        # 5. while the left hand tightens, hold the rail down right behind the bracket: closed fingers pressed
        # straight down into the rail's channel, the hand leaned 30 degrees with the fingertips toward the bracket, so
        # the pads' front lower corner stands 0.5 mm behind the bracket's face. It holds the rail and stops the bracket
        # twisting away from the block (the screw's friction twists a loose bracket within its guides' play, ~3
        # degrees, as an electrician's other hand would prevent)
        if variant != "no_hold":
            face = task.bracket_on_rail(world["states"])["along"] - ep.BRACKET_WIDTH / 2
            # the pads' front corner is 13.2 mm (half the pad width) along the leaned pad from the tips' centre
            along = face - STOP_GAP - PAD_HALF_WIDTH * math.cos(HOLD_LEAN)
            yield from hold_rail(arm, along, "right", outward=-1.0, press_down=True)
            phase["right"] = "holding the rail behind the bracket"
        flags["right_holding"] = True
        yield from wait_for(arm, "screwed", phase["right"] if variant != "no_hold" else "waiting")
        if variant != "no_hold":
            yield from release_rail(arm, "right")
        yield from move(arm, arm.pos + (-0.06, -0.08, 0.0), 1.5, label="away")
        phase["right"] = "done"
        report["script_done_step"] = world["step"]
        while True:
            yield

    # -- run -----------------------------------------------------------------------------------------------------
    scripts = {"left": left_script(), "right": right_script()}
    frames = {c: [] for c in CAMERAS} if video else None
    if video:
        for c in CAMERAS:
            frames[c].append(obs.images[c])
    log = []
    result = None
    success_step = None
    t0 = time.time()
    for step in range(1, max_steps + 1):
        world["step"] = step
        for side in ("left", "right"):
            next(scripts[side])
        a = pack(arms["left"].action(), arms["right"].action())
        res = emb.step(Action(data=a))
        world["obs"] = res.observation
        world["states"] = object_states(emb._env, emb.task.names)
        st = world["states"]
        b = task.bracket_on_rail(st)
        shift, turn = task.rail_motion(st)
        tip, rd = driver()
        log.append({
            "step": step, "left": phase["left"], "right": phase["right"], "score": res.info.get("score"),
            "success": res.info.get("success"), "stage": task.stage(st), "rail_shift_mm": shift / MM,
            "rail_turn_deg": turn, "across_mm": b["across"] / MM, "along_mm": b["along"] / MM,
            "seat_mm": [s / MM for s in b["seat"]], "gaps_mm": [g / MM for g in b["gaps"]],
            "yaw_deg": b["yaw_deg"], "tilt_deg": b["tilt_deg"], "screw_turns": b["screw_turns"],
            "clamp_N": b["clamp_force"],
            "driver_tip": [round(float(v), 4) for v in tip], "driver_tilt_deg": math.degrees(math.acos(min(1.0, abs(rd[2, 2])))),
            "speed_mm_s": float(np.linalg.norm(st["bracket"].lin_vel)) / MM,
            "r_cmd": [round(float(v), 4) for v in arms["right"].pos], "r_ik_mm": round(arms["right"].ik_err[0] / MM, 2),
            "r_meas": [round(float(v), 4) for v in measured_tcp("right")[0]],
            "l_meas": [round(float(v), 4) for v in measured_tcp("left")[0]],
            "l_cmd": [round(float(v), 4) for v in arms["left"].pos], "l_ik_mm": round(arms["left"].ik_err[0] / MM, 2),
            "l_wrist_deg": math.degrees(wrist_angle("left")),
            "r_tips": finger_tips("right"), "bracket_origin": [round(float(v), 4) for v in bracket()[0]],
        })
        if video:
            for c in CAMERAS:
                frames[c].append(res.observation.images[c])
        done_script = "script_done_step" in report and step - report["script_done_step"] >= round(AFTER_SUCCESS_S / DT)
        if "rail_moved_step" not in report and not task.rail_ok(st):
            report["rail_moved_step"] = step
        done_script = done_script or ("rail_moved_step" in report and step - report["rail_moved_step"] >= round(1.0 / DT))
        # stop early when the run can no longer succeed (saves compute while iterating)
        if "abort" not in report:
            tip_z = log[-1]["driver_tip"][2]
            if "presented_step" in report and "driver_released_step" not in report and (
                    tip_z < task.TOP + 0.02 or log[-1]["driver_tilt_deg"] > 45.0):
                report["abort"] = f"step {step}: the screwdriver fell"
            worst_ik = max(log[-1]["r_ik_mm"], log[-1]["l_ik_mm"])
            report["_ik_bad"] = report.get("_ik_bad", 0) + 1 if worst_ik > IK_ABORT_MM else 0
            if report["_ik_bad"] >= round(IK_ABORT_S / DT):
                report["abort"] = f"step {step}: an arm can't reach its target ({worst_ik:.0f} mm) for {IK_ABORT_S} s"
            if "flush_step" in report and task.stage(st) is None and step - report["flush_step"] > round(1.0 / DT):
                report["abort"] = f"step {step}: the bracket came off the rail"
        if "abort" in report and ABORT_EARLY:
            result = res
            break
        if res.terminated and success_step is None:
            success_step, result = step, res
            report["success_step"] = step
        if success_step is not None:
            if step - success_step >= round(AFTER_SUCCESS_S / DT):
                break
            continue
        if res.terminated or done_script:
            result = res
            break
        result = res
    st = world["states"]
    shift, turn = task.rail_motion(st)
    success = bool(result.terminated and result.termination_reason == "success")
    b = task.bracket_on_rail(st)
    summary = {
        "variant": variant,
        "task_score": 1.0 if success else float(result.info.get("score", 0.0)),
        "terminated": bool(result.terminated),
        "termination_reason": result.termination_reason,
        "steps": len(log),
        "run_limit_steps": MAX_STEPS,
        "final_stage": task.stage(st),
        "rail_shift_mm": shift / MM,
        "rail_turn_deg": turn,
        "bracket": {"across_mm": b["across"] / MM, "along_mm": b["along"] / MM, "seat_mm": [s / MM for s in b["seat"]],
                    "gaps_mm": [g / MM for g in b["gaps"]], "yaw_deg": b["yaw_deg"], "tilt_deg": b["tilt_deg"],
                    "screw_turns": b["screw_turns"], "clamp_N": b["clamp_force"]},
        "max_rail_shift_mm": max(r["rail_shift_mm"] for r in log),
        "max_rail_turn_deg": max(r["rail_turn_deg"] for r in log),
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

        first = np.ascontiguousarray(frames["top_cam"][0][..., ::-1])
        cv2.imwrite(str(out / "setup.png"), first)
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
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg:
            subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-i", str(raw), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                            "-movflags", "+faststart", str(name)], check=True)
            raw.unlink()
        else:
            raw.rename(name.with_suffix(".avi"))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", choices=VARIANTS + ("all",), default="full",
                    help="'all' runs every variant in one simulator session (reset between them)")
    ap.add_argument("--out", type=Path, default=None, help="output folder (default: ./demo_runs/<variant>)")
    ap.add_argument("--no-video", action="store_true", help="no videos (and no rendering)")
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    ap.add_argument("--device", default="cpu",
                    help="physics device: trials of this task run CPU PhysX (-E device=cpu; see the README: on cuda:0 the "
                         "sub-millimetre blade-in-slot contact doesn't pass torque to the screw)")
    ap.add_argument("--start-at", choices=("start", "screwdriver"), default="start",
                    help="'screwdriver': development only, the bracket starts flush on the rail and the right arm goes "
                         "straight to the screwdriver")
    ap.add_argument("--no-abort", action="store_true", help="run to the step limit even when the run has failed")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a constant (a Python literal): --set PRESS=0.003, or a part's with --set ep.JAW_SPRING=1e5")
    args = ap.parse_args(argv)
    for item in args.set:
        name, value = item.split("=", 1)
        if name.startswith("ep."):
            sys.path.insert(0, str(HERE))
            import eb_parts

            target = vars(eb_parts)
        else:
            target = globals()
        name = name.removeprefix("ep.")
        if name not in target:
            ap.error(f"--set: no constant {name!r}")
        target[name] = ast.literal_eval(value)
    video = not args.no_video
    global START_AT, ABORT_EARLY
    START_AT, ABORT_EARLY = args.start_at, not args.no_abort
    emb = make_embodiment(video, args.device)
    for variant in VARIANTS if args.variant == "all" else (args.variant,):
        out = args.out / variant if args.out and args.variant == "all" else args.out or Path("demo_runs") / variant
        summary = run(variant, out, video, args.max_steps, emb)
        print("RESULT " + json.dumps({k: summary[k] for k in ("variant", "task_score", "steps", "termination_reason",
                                                               "final_stage", "rail_shift_mm", "rail_turn_deg",
                                                               "wall_s")}, default=float))
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
