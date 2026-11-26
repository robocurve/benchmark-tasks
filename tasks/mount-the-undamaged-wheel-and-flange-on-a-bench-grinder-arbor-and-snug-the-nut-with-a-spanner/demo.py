#!/usr/bin/env python3
"""Scripted reference run for the bench-grinder wheel task.

    scripts/rta demo <id> mech/tasks/grinder_wheel [--no-video] [--check] [--quick]

Leaves the visibly cracked wheel untouched in its left slot; picks the intact 150 x 16 mm wheel and slides it onto the
bare 12.7 mm right arbor against the keyed inner flange; picks the Ø 52 mm stamped steel outer flange, turns it +90 deg
so its concave relieved bearing ring faces inward against the wheel blotter, and seats it (reaching 0.5); counter-holds
the installed +y fine wheel with the left hand while the right hand threads the 19 mm flanged hex nut hand-snug, picks
the bare GEDORE 894 19 open-end spanner from its rack, slides its jaw onto the hex flats, pulls the handle upward until
the nut's witness stripe stands 17-195 deg (2.5-27.1 N m) past snug, sets the spanner back down by its rack, and returns
both arms to READY (1.0).

The run acts on what it sees, as a person at the bench would, through a ``director`` (demo_kit.run): before each part
goes onto the arbor it hovers and lines the part's bore up with the arbor from the part's observed pose; it reads the
nut's flats to set the spanner's entry angle; during the stroke it reads the witness stripe against the arbor's tick
(and stops the stroke inside the window) and the stripe on the left wheel (and presses with the left hand so the
spindle stays put). It never writes a pose or a joint.
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
WHEEL_GRIP = 0.002 / K.MAX_JAW   # ~35 N per finger across the 16 mm wheel
FLANGE_GRIP = 0.040 / K.MAX_JAW  # ~30 N per finger across the Ø 52 mm steel flange rim
NUT_GRIP = 0.013 / K.MAX_JAW     # pads meet the Ø 28 mm collar first: ~35 N per finger
WRENCH_GRIP = 0.0                # full close on the 7 mm steel shank
LEFT_HOLD_GRIP = 0.001 / K.MAX_JAW  # ~37.5 N per finger on the +y K60 wheel (<= 40 N)
GX, GZ = P.GRINDER_X, K.TOP + P.ARBOR_Z

# Stroke: the witness angle the nut is to rest at (inside 17-195 deg; under load the stripe stands the thread's elastic
# windup, torque / NUT_DRIVE_K, further on, given back when the spanner lets go), how fast the spanner's angle target
# advances, the highest handle pitch (grasp 146 mm over the bench), the most drive effort the left hand's push may ask
# for, and how far the handle backs off before the jaw is drawn (windup ~1.9 deg + the jaw's 3.3 deg play on the flats)
WITNESS_STOP_DEG = 18.0  # ~2.8 N m under load: stopped at 3.1 N m the hex stayed wedged in the jaw (gw_d17-d21)
BACKOFF_MAX_DEG, BACKOFF_MAX_S = 6.0, 1.5  # the handle turns back 1 deg a step until the jaw bears under 2 N (at least 3 deg)
SQUARE_GAIN, SQUARE_MAX = 0.2, math.radians(6.0)  # share of the spanner's observed tilt on the nut the hand takes up
# each control step, and the most it may turn the hand in all: a spanner cocked 1-1.7 deg loaded one end of each flank
# and the hex rolled through at 3 N m (runs gw_p19, gw_p20)
STROKE_RATE = math.radians(7.0)  # rad/s
PITCH_MAX = math.radians(20.0)
LEFT_EFFORT_MAX = 0.92
SPINDLE_KI = 0.03  # N m of left-hand hold added per degree of spindle turn, per control step
SPINDLE_KP = 0.10  # N m of left-hand hold per degree the spindle stands turned
PULL_MAX_S, HOLD_MAX_S = 9.0, 400.0
COLLAR_PRESS = 0.0  # N toward the grinder during the stroke (2 N pressed the jaw's face onto the collar: runs gw_d10-d13)
JAW_NEAR = 0.026     # m back along the slot for the last aim: the jaw's mouth tip then stands 1 mm clear of the hex corners
SEAT_S, SEAT_MAX_S = 2.0, 3.5  # s the jaw takes to slide on (servoed on the spanner's observed pose), most it may take
SEAT_GAIN = 0.25     # share of the observed jaw error the hand's target takes up each control step
ENDON_PREROLL = math.radians(-13.0)  # the end-on hand takes the collar turned back 13 deg and turns the still-free nut on
# while pressing it home: after its 17 deg catch turn the flats then ask for a ~-17 deg spanner entry, where the jaw
# seats square (a -30 deg entry seated it 4-6 mm off along the arbor, runs gw_d5-d8); rolls -20..+25 deg reach (gw_r2)
DRAW_PUSH_MAX, DRAW_MAX_S = 40.0, 1.5  # N the hand pushes the jaw off the hex with at most (ramped), s it may take: after
# a stroke the hex stays wedged in the jaw at 14-30 N, which a position move of the soft arm (~6 N) never shifted (gw_d19)
JAW_SHORT = 0.002    # m the jaw centre stays short of the nut's centre: the hex's back corner clears the throat and
                     # both corners that lock sit on the gripping flanks, not on the steel tips


def ready_pose(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def entry_pitch_for_flats(nut_rot: np.ndarray, nominal: float = K.WRENCH_ENTRY_PITCH) -> float:
    """Handle pitch (rad) nearest ``nominal`` at which the spanner's slot is parallel to a pair of the nut's flats.
    The flats' normals lie at the nut's local x angle + k 60 deg (angle from world +x toward +z); the slot axis lies at
    -(pitch + 15 deg)."""
    alpha0 = math.atan2(float(nut_rot[2, 0]), float(nut_rot[0, 0]))
    base = -math.radians(105.0) - alpha0  # slot parallel to the flat whose normal is alpha0 (+ k 60 deg)
    step = math.radians(60.0)
    k = round((base - nominal) / step)
    return base - k * step


def spanner_moves(p_entry: float = K.WRENCH_ENTRY_PITCH) -> list:
    """Right arm, last stage: the GEDORE 894 19 spanner out of its edge cradle, its jaw slid onto the 19 mm hex, the
    stroke (a placeholder the director plays and ends), the jaw drawn off, the spanner parked, the arm back at READY."""
    import demo_kit as kit

    Move = kit.Move
    poses = K.poses()
    r_ready_pos, r_ready_rot = ready_pose("right")
    nj = K.nut_jaw_centre()
    before = K.wrench_pose_for_jaw(nj - K.WRENCH_BACK * K.wrench_jaw_dir(p_entry), p_entry)
    near = K.wrench_pose_for_jaw(nj - JAW_NEAR * K.wrench_jaw_dir(p_entry), p_entry)
    engaged = K.wrench_pose_for_jaw(nj, p_entry)
    out = K.wrench_pose_for_jaw(nj - K.WRENCH_OUT * K.wrench_jaw_dir(K.WRENCH_PULL_END_PITCH), K.WRENCH_PULL_END_PITCH)
    return [
        Move(1.1, pos=poses["wrench: approach cradle"][1], rot=poses["wrench: approach cradle"][2], grip=0.45, label="to spanner"),
        Move(0.7, pos=poses["wrench: grasp shank"][1], label="on spanner"),
        Move(0.5, grip=WRENCH_GRIP, label="pinch spanner"),
        Move(0.5, pos=poses["wrench: eased up"][1], label="ease spanner up"),
        Move(0.8, pos=poses["wrench: lift level"][1], rot=poses["wrench: lift level"][2], label="lift spanner"),
        Move(1.2, pos=poses["wrench: over nut lane"][1], label="to nut lane"),
        Move(1.0, pos=before[0], rot=before[1], label="before hex nut"),
        Move(0.6, pos=before[0], rot=before[1], label="jaw hover 1"),
        Move(0.5, pos=before[0], rot=before[1], label="jaw hover 2"),
        Move(0.6, pos=near[0], rot=near[1], label="jaw near"),
        Move(0.5, pos=near[0], rot=near[1], label="jaw hover 3"),
        Move(SEAT_MAX_S, pos=near[0], rot=near[1], push=(0.0, 0.0, 0.0), label="seat 19mm jaw"),
        Move(PULL_MAX_S, pos=engaged[0], rot=engaged[1], label="pull"),
        Move(BACKOFF_MAX_S, pos=engaged[0], rot=engaged[1], push=(0.0, 0.0, 0.0), label="back off"),
        Move(DRAW_MAX_S, pos=out[0], rot=out[1], push=(0.0, 0.0, 0.0), label="draw jaw off"),
        Move(1.0, pos=poses["wrench: back over lane"][1], rot=poses["wrench: back over lane"][2], label="clear nut"),
        Move(1.2, pos=poses["wrench: lift level"][1], label="over cradle"),
        Move(0.5, pos=poses["wrench: lift level"][1], rot=poses["wrench: lift level"][2], label="cradle hover"),
        Move(1.0, pos=poses["wrench: place in cradle"][1], rot=poses["wrench: place in cradle"][2], label="place spanner"),
        Move(0.3, label="settle spanner"),
        Move(0.5, grip=OPEN, label="release spanner"),
        Move(0.6, pos=poses["wrench: approach cradle"][1], label="clear spanner"),
        Move(1.5, pos=r_ready_pos, rot=r_ready_rot, grip=OPEN, label="ready"),
    ]


def left_moves() -> list:
    """Left arm: holds the +y wheel from the start (the nut's finger turn and the stroke both need the spindle held);
    the hold is a placeholder the director ends once the right hand's stroke has eased off."""
    import demo_kit as kit

    Move = kit.Move
    poses = K.poses()
    l_ready_pos, l_ready_rot = ready_pose("left")
    hold = poses["left: counter-hold +y wheel"]
    return [
        Move(0.5, grip=OPEN, label="wait"),
        Move(1.5, pos=poses["left: approach +y wheel"][1], rot=poses["left: approach +y wheel"][2], grip=0.65, label="to +y wheel"),
        Move(0.9, pos=hold[1], label="on +y wheel"),
        Move(0.7, grip=LEFT_HOLD_GRIP, label="counter-hold"),
        Move(HOLD_MAX_S, pos=hold[1], rot=hold[2], label="hold spindle"),
        Move(0.6, grip=0.65, push=(0.0, 0.0, 0.0), label="release +y wheel"),
        Move(0.9, pos=poses["left: approach +y wheel"][1], label="clear +y wheel"),
        Move(2.0, pos=l_ready_pos, rot=l_ready_rot, grip=OPEN, label="ready"),
    ]


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    poses = K.poses()
    r_ready_pos, r_ready_rot = ready_pose("right")

    def at(name, dy=0.0, dz=0.0):
        return poses[name][1] + np.array([0.0, dy, dz])

    # 1. Sound wheel: down onto it along the approach, lift, carry to the arbor's line, hover and aim, slide on
    w_lift = (0.0, 0.0, P.WHEEL_MASS * P.GRAVITY)  # N: carries the wheel's weight so the soft arm does not sag
    w_rot = poses["wheel: before arbor"][2]
    right = [
        Move(1.1, pos=poses["wheel: approach"][1], rot=poses["wheel: approach"][2], grip=0.68, label="to wheel"),
        Move(0.7, pos=poses["wheel: grasp"][1], label="on sound wheel"),
        Move(0.5, grip=WHEEL_GRIP, label="pinch wheel"),
        Move(0.9, pos=poses["wheel: lift"][1], rot=poses["wheel: lift"][2], push=w_lift, label="lift wheel"),
        Move(1.4, pos=at("wheel: before arbor"), rot=w_rot, push=w_lift, label="wheel to arbor"),
        Move(0.7, pos=at("wheel: before arbor"), rot=w_rot, label="wheel hover 1"),
        Move(0.5, pos=at("wheel: before arbor"), rot=w_rot, label="wheel hover 2"),
        Move(1.0, pos=at("wheel: before arbor", dy=0.030), rot=w_rot, label="wheel onto dome"),
        Move(1.4, pos=at("wheel: seated on arbor", dy=0.004), rot=w_rot, push=(0.0, 8.0, w_lift[2]), label="slide wheel on"),
        Move(0.3, label="wheel home"),
        Move(0.4, grip=0.68, push=(0.0, 0.0, 0.0), label="let go wheel"),
        Move(0.7, pos=poses["wheel: release back"][1], label="clear wheel"),
        Move(0.8, pos=r_ready_pos, rot=r_ready_rot, grip=0.84, label="through ready 1"),
    ]

    # 2. Outer flange, turned +90 deg so its relieved bearing ring faces the wheel: hover, aim, slide on slowly, seat
    f_rot = poses["flange: oriented before arbor"][2]
    f_up = np.array([0.0, 0.0, 0.001])
    right += [
        Move(1.1, pos=poses["flange: approach"][1], rot=poses["flange: approach"][2], grip=0.84, label="to flange"),
        Move(0.7, pos=poses["flange: grasp"][1], label="on flange"),
        Move(0.5, grip=FLANGE_GRIP, label="pinch flange"),
        Move(0.8, pos=poses["flange: lift"][1], push=(0.0, 0.0, 1.0), label="lift flange"),
        Move(1.4, pos=at("flange: oriented before arbor") + f_up, rot=f_rot, push=(0.0, 0.0, 1.0), label="orient recess +y"),
        Move(0.8, pos=at("flange: oriented before arbor") + f_up, rot=f_rot, label="flange hover 1"),
        Move(0.6, pos=at("flange: oriented before arbor") + f_up, rot=f_rot, label="flange hover 2"),
        Move(0.4, pos=at("flange: oriented before arbor") + f_up, rot=f_rot, label="flange hover 3"),
        Move(1.2, pos=at("flange: oriented before arbor", dy=0.032) + f_up, rot=f_rot, label="flange onto dome"),
        Move(1.4, pos=at("flange: seated recessed-inward", dy=0.006) + f_up, rot=f_rot, push=(0.0, 6.5, 1.0), label="slide flange on"),
        Move(0.3, label="flange home"),
        Move(0.4, grip=0.84, push=(0.0, 0.0, 0.0), label="let go flange"),
        Move(0.6, pos=poses["flange: release back"][1], label="clear flange"),
        # The carry grasp's finger corners stop on the wheel 18 mm short: the fingertips then push it home end-on
        Move(0.9, pos=poses["right: end-on away"][1], rot=poses["right: end-on away"][2], grip=K.PUSH_OPEN / K.MAX_JAW, label="away from arbor 0"),
        Move(0.9, pos=poses["flange: push start"][1], rot=poses["flange: push start"][2], label="to flange push"),
        Move(0.4, pos=poses["flange: push start"][1], rot=poses["flange: push start"][2], label="flange push hover"),
        Move(0.5, pos=poses["flange: push start"][1], label="flange touch"),
        Move(1.4, pos=poses["flange: pushed home"][1], push=(0.0, 6.0, 0.0), label="push flange on"),
        Move(0.3, label="flange pressed"),
        Move(0.6, pos=poses["flange: push off"][1], push=(0.0, 0.0, 0.0), label="clear flange push"),
        Move(0.9, pos=poses["right: end-on away"][1], rot=poses["right: end-on away"][2], label="away from arbor 1"),
        Move(0.8, pos=r_ready_pos, rot=r_ready_rot, grip=0.52, label="through ready 2"),
    ]

    # 3. Nut: carried on with the tilted grasp (its finger corners, 8 mm past the collar, push the flange home on the
    # way), let go on the arbor, taken again end-on by its round collar, pressed on the flange and turned 20 deg
    n_rot = poses["nut: before arbor"][2]
    pre_rot = K.nut_endon_pose(np.zeros(3), ENDON_PREROLL)[1]  # the hand takes the collar turned back, turns it on the way in
    n_up = np.array([0.0, 0.0, 0.002])
    right += [
        Move(1.1, pos=poses["nut: approach"][1], rot=poses["nut: approach"][2], grip=0.52, label="to nut"),
        Move(0.7, pos=poses["nut: grasp"][1], label="on nut"),
        Move(0.5, grip=NUT_GRIP, label="pinch nut"),
        Move(0.7, pos=poses["nut: lift"][1], label="lift nut"),
        Move(1.4, pos=at("nut: before arbor") + n_up, rot=n_rot, push=(0.0, 0.0, 0.8), label="nut to arbor"),
        Move(0.8, pos=at("nut: before arbor") + n_up, rot=n_rot, label="nut hover 1"),
        Move(0.6, pos=at("nut: before arbor") + n_up, rot=n_rot, label="nut hover 2"),
        Move(0.4, pos=at("nut: before arbor") + n_up, rot=n_rot, label="nut hover 3"),
        Move(1.35, pos=at("nut: before arbor", dy=0.034) + n_up, rot=n_rot, label="nut onto dome"),
        Move(1.65, pos=at("nut: on thread start", dy=0.006) + n_up, rot=n_rot, push=(0.0, 6.5, 0.8), label="push flange home"),
        Move(0.4, label="flange pushed"),
        Move(0.5, grip=0.52, push=(0.0, 0.0, 0.0), label="let go nut"),
        Move(0.8, pos=poses["nut: release back"][1], label="clear nut"),
        Move(1.0, pos=poses["right: end-on away"][1], rot=poses["right: end-on away"][2], grip=0.62, label="away from arbor 1b"),
        Move(1.0, pos=poses["nut: end-on far"][1], rot=pre_rot, label="to nut end-on"),
        Move(0.55, pos=poses["nut: end-on far"][1], rot=pre_rot, label="nut end-on hover"),
        Move(0.8, pos=poses["nut: end-on back"][1], label="end-on back"),
        Move(0.6, pos=poses["nut: end-on grasp"][1], label="on nut end-on"),
        Move(0.6, grip=NUT_GRIP, label="pinch collar"),
        Move(1.25, pos=at("nut: end-on pressed", dy=0.002), rot=poses["nut: end-on pressed"][2], push=(0.0, 5.5, 0.0), label="press nut on"),
        Move(1.6, pos=at("nut: end-on turned", dy=0.002), rot=poses["nut: end-on turned"][2], push=(0.0, 2.0, 0.0), label="finger-snug nut"),  # 6 N froze an 11 N stack preload at the catch (gw_d16)
        Move(0.4, grip=0.62, push=(0.0, 0.0, 0.0), label="let go collar"),
        Move(0.6, pos=poses["nut: end-on off"][1], label="clear nut end-on"),
        Move(0.9, pos=poses["right: end-on away"][1], rot=poses["right: end-on away"][2], label="away from arbor 2"),
        Move(0.8, pos=r_ready_pos, rot=r_ready_rot, grip=0.45, label="through ready 3"),
    ]

    # 4. Spanner onto the nut, the stroke, parked again
    right += spanner_moves()
    return {"left": left_moves(), "right": right}


def _rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        return np.eye(3)
    k = v / s
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    ang = math.atan2(s, c)
    return np.eye(3) + math.sin(ang) * kx + (1.0 - math.cos(ang)) * (kx @ kx)


def _slerp_rot(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    """Rotation a turned the share ``s`` of the way to b."""
    rel = b @ a.T
    ang = math.acos(max(-1.0, min(1.0, 0.5 * (float(np.trace(rel)) - 1.0))))
    if ang < 1e-9:
        return a
    axis = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]]) / (2.0 * math.sin(ang))
    kx = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    th = s * ang
    return (np.eye(3) + math.sin(th) * kx + (1.0 - math.cos(th)) * (kx @ kx)) @ a


def _active(arm, t: float) -> tuple[int, float]:
    acc = 0.0
    for i, mv in enumerate(arm.moves):
        if t < acc + mv.seconds:
            return i, t - acc
        acc += mv.seconds
    return len(arm.moves), t - acc


class Director:
    """What the demo reads and does each control step (see the module docstring). ``setup(t)``, if given, runs first
    (trials and development probes only: they may set a start state; the demo passes none)."""

    AIMS = {  # hover move -> (part, last move the correction applies to)
        "wheel hover 1": ("wheel_sound", "wheel home"),
        "wheel hover 2": ("wheel_sound", "wheel home"),
        "flange hover 1": ("outer_flange", "flange home"),
        "flange hover 2": ("outer_flange", "flange home"),
        "flange hover 3": ("outer_flange", "flange home"),
        "nut hover 1": ("arbor_nut", "flange pushed"),
        "nut hover 2": ("arbor_nut", "flange pushed"),
        "nut hover 3": ("arbor_nut", "flange pushed"),
    }

    def __init__(self, m, setup=None) -> None:
        self.m, self.mk, self.setup = m, m.mk, setup
        self.done: set = set()
        self.log: dict = {"aims": [], "stroke": []}
        self.over: dict = {}
        self.patched = False
        self.stroke: dict | None = None
        self.seat: dict | None = None
        self.back: dict | None = None
        self.draw: dict | None = None
        self.p_entry = K.WRENCH_ENTRY_PITCH
        self.nj = K.nut_jaw_centre()
        hold = K.poses()["left: counter-hold +y wheel"][1]
        self.left_r = hold - np.array([GX, P.LEFT_WHEEL_Y, GZ])

    def _patch(self, arms) -> None:
        for side in ("left", "right"):
            arm = arms[side]
            orig = arm.state

            def state(t, arm=arm, orig=orig, side=side):
                out = orig(t)
                ov = self.over.get(side)
                if ov is not None and out[3] == ov["label"]:
                    return ov["pos"], ov["rot"], out[2], out[3], ov["push"], out[5]
                return out

            arm.state = state
        self.patched = True

    @staticmethod
    def _index(arm, label: str) -> int | None:
        return next((i for i, mv in enumerate(arm.moves) if mv.label == label), None)

    def _fire(self, arm, t: float, label: str) -> bool:
        """True once, at the first step after the move ``label`` has ended."""
        if label in self.done:
            return False
        idx = self._index(arm, label)
        if idx is None or _active(arm, t)[0] <= idx:
            return False
        self.done.add(label)
        return True

    def _correct(self, arm, hover: str, last: str, R: np.ndarray, pivot: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Move the held part from ``pivot`` to ``target`` and turn it by ``R``: every later target up to ``last`` is
        shifted by what that does to the hand at the hover pose (slides keep their world direction) and turned by R."""
        i_hover, i_last = self._index(arm, hover), self._index(arm, last)
        goal = np.asarray(arm.moves[i_hover].pos, dtype=float)
        delta = R @ (goal - pivot) + target - goal
        for mv in arm.moves[i_hover + 1 : i_last + 1]:
            if mv.pos is not None:
                mv.pos = np.asarray(mv.pos, dtype=float) + delta
            if mv.rot is not None:
                mv.rot = R @ np.asarray(mv.rot, dtype=float)
        return delta

    def _aim(self, arm, t: float, hover: str, part: str, last: str) -> None:
        pos, rot = self.mk.pose(part)
        R = _rotation_between(rot[:, 1], np.array([0.0, 1.0, 0.0]))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[1, 1])))))
        if tilt > 12.0:
            R = np.eye(3)  # not a small misalignment: correct the position only
        if math.hypot(float(pos[0]) - GX, float(pos[2]) - GZ) > 0.025:
            self.log["aims"].append({"t": round(t, 2), "hover": hover, "skipped": "part not in the hand"})
            return
        delta = self._correct(arm, hover, last, R, pos, np.array([GX, pos[1], GZ]))
        self.log["aims"].append({"t": round(t, 2), "hover": hover, "off_mm": [round(1000 * float(v), 2) for v in (pos[0] - GX, pos[2] - GZ)], "tilt_deg": round(tilt, 2), "delta_mm": [round(1000 * float(v), 2) for v in delta]})

    def _aim_wrench(self, arm, t: float, hover: str, last: str, jaw: np.ndarray, pitch: float) -> None:
        """Line the spanner up from its observed pose: jaw centre to ``jaw``, its face square to the arbor, pitch."""
        pos, rot = self.mk.pose("wrench")
        R = K.ry(pitch) @ rot.T
        ang = math.degrees(math.acos(max(-1.0, min(1.0, 0.5 * (float(np.trace(R)) - 1.0)))))
        # at the cradle the spanner may hang turned and slid in the grasp (torn off a wedged hex: runs gw_d17-d22)
        lim_deg, lim_m = (45.0, 0.120) if hover == "cradle hover" else (15.0, 0.040)
        if ang > lim_deg or float(np.linalg.norm(pos - jaw)) > lim_m:
            self.log["aims"].append({"t": round(t, 2), "hover": hover, "skipped": "spanner not where the hand holds it", "jaw_off_mm": [round(1000 * float(v), 1) for v in (pos - jaw)], "turn_deg": round(ang, 2)})
            return
        delta = self._correct(arm, hover, last, R, pos, jaw)
        self.log["aims"].append({"t": round(t, 2), "hover": hover, "jaw_off_mm": [round(1000 * float(v), 2) for v in (pos - jaw)], "turn_deg": round(ang, 2), "delta_mm": [round(1000 * float(v), 2) for v in delta]})

    def _flange_touch(self, arm, t: float) -> None:
        """Bring the fingertips to 1 mm short of the flange's outer web face where it hangs (read from its pose)."""
        pos, rot = self.mk.pose("outer_flange")
        face = float((pos + rot @ np.array([0.0, -P.FLANGE_DEPTH / 2.0, 0.0]))[1])
        arm.moves[self._index(arm, "flange touch")].pos = K.flange_push_pose(face - 0.001)[0]
        self.log["flange_touch"] = {"t": round(t, 2), "face_mm": round(1000 * face, 1), "tilt_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, float(rot[1, 1]))))), 2)}

    def _endon(self, arm, t: float) -> None:
        """Take the nut end-on where it was let go (read from its pose)."""
        pos, _ = self.mk.pose("arbor_nut")
        grasp = K.nut_endon_pose(pos, ENDON_PREROLL)[0]
        for label, back in (("end-on back", 0.0065), ("on nut end-on", 0.0)):
            i = self._index(arm, label)
            arm.moves[i].pos = grasp - np.array([0.0, back, 0.0])
        self.log["endon"] = {"t": round(t, 2), "nut_mm": [round(1000 * float(v), 1) for v in pos]}

    def _set_entry(self, arm, t: float) -> None:
        npos, nrot = self.mk.pose("arbor_nut")
        self.nj = np.array([float(npos[0]), float(npos[1]) + K.WRENCH_ON_NUT_DY, float(npos[2])])
        p = entry_pitch_for_flats(nrot)
        # reachable entries (offline reach run gw_r1: IK <= 0.08 mm, margins >= 24 deg, hands >= 12 mm clear down to
        # -32 deg); the caught nut's flats asked for -30.2 deg in run gw_d4, where a -24 deg limit left the jaw 6 deg off
        lo, hi = math.radians(-33.0), math.radians(-10.0)
        self.log["entry"] = {"t": round(t, 2), "flats_pitch_deg": round(math.degrees(p), 2)}
        p = min(hi, max(lo, p))
        self.p_entry = p
        before = K.wrench_pose_for_jaw(self.nj - K.WRENCH_BACK * K.wrench_jaw_dir(p), p)
        engaged = K.wrench_pose_for_jaw(self.nj, p)
        near = K.wrench_pose_for_jaw(self.nj - JAW_NEAR * K.wrench_jaw_dir(p), p)
        for mv in arm.moves:
            if mv.label in ("before hex nut", "jaw hover 1", "jaw hover 2"):
                mv.pos, mv.rot = before
            elif mv.label in ("jaw near", "jaw hover 3"):
                mv.pos, mv.rot = near
            elif mv.label == "seat 19mm jaw":
                mv.pos, mv.rot = near
                mv.push = (0.0, 0.0, 0.0)
            elif mv.label == "pull":
                mv.pos, mv.rot = engaged
        self.log["entry"]["used_deg"] = round(math.degrees(p), 2)

    def _seat(self, arms, t: float, i_seat: int, t_in: float) -> None:
        """Slide the jaw onto the hex along the slot line, correcting the hand every step from the spanner's observed
        pose (the soft wrist turns the spanner ~0.6 deg per newton of jaw contact)."""
        right = arms["right"]
        seat = right.moves[i_seat]
        if self.seat is None:
            self.seat = {"pos": np.asarray(seat.pos, float), "rot": np.asarray(seat.rot, float), "t0": t, "start_pos": np.asarray(seat.pos, float), "start_rot": np.asarray(seat.rot, float)}
        sv = self.seat
        prog = min(1.0, (t - sv["t0"]) / SEAT_S)
        jdir = K.wrench_jaw_dir(self.p_entry)
        j_des = self.nj - (JAW_SHORT + (JAW_NEAR - JAW_SHORT) * (1.0 - prog)) * jdir
        j_obs, r_obs = self.mk.pose("wrench")
        R = K.ry(self.p_entry) @ r_obs.T
        full = R @ (sv["pos"] - j_obs) + j_des - sv["pos"]
        step = SEAT_GAIN * full
        # in the slot's plane at most 2 mm a step, along the arbor (y) apart at most 1 mm: one cap on the whole step
        # left a 4.7 mm y error standing while the jaw lagged along the slot (run gw_d6)
        step_xz = np.array([step[0], 0.0, step[2]])
        n = float(np.linalg.norm(step_xz))
        if n > 0.002:
            step_xz *= 0.002 / n
        step = step_xz + np.array([0.0, max(-0.001, min(0.001, float(step[1]))), 0.0])
        ang = math.acos(max(-1.0, min(1.0, 0.5 * (float(np.trace(R)) - 1.0))))
        # Bounded: a jaw that is stuck must not wind the hand's target off (at most 6 mm and 5 deg from the planned slide)
        planned = sv["start_pos"] + (j_des - (self.nj - JAW_NEAR * jdir))  # the slide as planned, from the near pose
        cand = sv["pos"] + step
        off = cand - planned
        # in the slot's plane at most 6 mm; across it (y, along the arbor) apart, at most 8 mm: one bound on the whole
        # offset let a 15 mm lag along the slot starve the y correction, the jaw slid 6 mm off the hex's end and its
        # throat stopped on the arbor's dome (run gw_d5)
        off_xz = np.array([off[0], 0.0, off[2]])
        n_xz = float(np.linalg.norm(off_xz))
        if n_xz > 0.006:
            off_xz *= 0.006 / n_xz
        cand = planned + off_xz + np.array([0.0, max(-0.008, min(0.008, float(off[1]))), 0.0])
        sv["pos"] = cand
        if ang > 1e-6:
            new_rot = _slerp_rot(sv["rot"], R @ sv["rot"], min(SEAT_GAIN, math.radians(0.5) / ang))  # at most 0.5 deg a step
            dev = new_rot @ sv["start_rot"].T
            if math.degrees(math.acos(max(-1.0, min(1.0, 0.5 * (float(np.trace(dev)) - 1.0))))) <= 5.0:
                sv["rot"] = new_rot
        # the hand also advances with the jaw's wanted point along the slot
        sv["pos"] = sv["pos"] + (j_des - sv.get("j_prev", j_des))
        sv["j_prev"] = j_des
        self.over["right"] = {"label": "seat 19mm jaw", "pos": sv["pos"], "rot": sv["rot"], "push": np.zeros(3)}
        err = j_obs - self.nj
        if int(round(t * 20)) % 2 == 0:
            self.log.setdefault("seat", []).append({"t": round(t, 2), "prog": round(prog, 2), "jaw_mm": [round(1000 * float(v), 2) for v in err], "turn_deg": round(math.degrees(ang), 2), "contact": round(self.mk.contact_force("wrench", "arbor_nut"), 1)})
        along = float(np.dot(err, jdir)) + JAW_SHORT
        if (prog >= 1.0 and abs(along) < 0.0015 and ang < math.radians(2.0)) or t_in >= SEAT_MAX_S - 0.05:
            seat.seconds = t_in
            seat.pos, seat.rot = sv["pos"], sv["rot"]
            pull = right.moves[i_seat + 1]
            pull.pos, pull.rot = sv["pos"], sv["rot"]
            self.over.pop("right", None)
            self.seat["done"] = True
            self.log["seat_end"] = {"t": round(t, 2), "jaw_mm": [round(1000 * float(v), 2) for v in err], "turn_deg": round(math.degrees(ang), 2)}

    def _wrench_pitch(self) -> float:
        _, r = self.mk.pose("wrench")
        return math.atan2(-float(r[2, 0]), float(r[0, 0]))

    def _left_push(self, arms, torque: float) -> np.ndarray:
        """Straight down at the left grasp (the cheapest direction for the drives): -torque about +y on the spindle."""
        import demo_kit as kit

        f = np.array([0.0, 0.0, -max(0.0, torque) / -float(self.left_r[0])])
        jac = kit.point_jacobian(arms["left"].q, arms["left"].base)
        effort = float(np.abs(jac.T @ f / kit.JOINT_EFFORT).max())
        if effort > LEFT_EFFORT_MAX:
            f *= LEFT_EFFORT_MAX / effort
        return f

    def _stroke(self, arms, t: float, i_pull: int, t_in: float) -> None:
        m, mk = self.m, self.mk
        right, left = arms["right"], arms["left"]
        pull = right.moves[i_pull]
        w = m.witness_angle_deg()
        spin = math.degrees(mk.joint("spindle"))
        if self.stroke is None:
            seat = right.moves[self._index(right, "seat 19mm jaw")]
            self.stroke = {"phase": "stroke", "t0": t, "p": self.p_entry, "s0": spin, "ti": 0.0, "pos": np.asarray(seat.pos, float), "rot": np.asarray(seat.rot, float)}
        st = self.stroke
        tau = P.thread_torque_for_angle_deg(max(0.0, w))
        if st["phase"] == "stroke":
            w_rest = w - math.degrees(tau / P.NUT_DRIVE_K)
            if w_rest >= WITNESS_STOP_DEG or st["p"] >= PITCH_MAX:
                st["phase"], st["tp"] = "hold", t
                st["p"] = max(self.p_entry, st["p"] - math.radians(2.0))
            else:
                st["p"] = min(PITCH_MAX, st["p"] + STROKE_RATE / 20.0)
            scale, extra = 1.0, 0.25
        elif st["phase"] == "hold":
            scale, extra = 1.0, -0.3
            if t - st["tp"] >= 0.4:
                st["phase"], st["tp"] = "ease", t
        else:
            scale, extra = max(0.0, 1.0 - (t - st["tp"]) / 0.6), -0.3
        p = st["p"]
        Rp = K.ry(p - self.p_entry)
        # Square: the spanner's own y axis back onto the arbor's, by turning the hand about the jaw
        _, wrot_now = mk.pose("wrench")
        tilt = _rotation_between(wrot_now[:, 1], np.array([0.0, 1.0, 0.0]))
        sq = _slerp_rot(st.get("square", np.eye(3)), tilt @ st.get("square", np.eye(3)), SQUARE_GAIN)
        if math.acos(max(-1.0, min(1.0, 0.5 * (float(np.trace(sq)) - 1.0)))) <= SQUARE_MAX:
            st["square"] = sq
        sq = st.get("square", np.eye(3))
        pos = sq @ (Rp @ (st["pos"] - self.nj)) + self.nj
        rot = sq @ Rp @ st["rot"]
        r = pos - self.nj
        t_dir = np.array([r[2], 0.0, -r[0]]) / max(1e-6, math.hypot(float(r[0]), float(r[2])))
        f_r = scale * max(0.0, tau + extra) / max(0.05, math.hypot(float(r[0]), float(r[2]))) * t_dir + np.array([0.0, COLLAR_PRESS * scale, 0.0])
        self.over["right"] = {"label": "pull", "pos": pos, "rot": rot, "push": f_r}
        # Left hand: the thread torque the stroke puts on the spindle, plus a correction for the turn the spindle has made
        i_hold = self._index(left, "hold spindle")
        f_l = np.zeros(3)
        if i_hold is not None:
            err = spin - st["s0"]
            st["ti"] = max(0.0, min(4.5, st["ti"] + SPINDLE_KI * err * scale))
            f_l = self._left_push(arms, scale * (st["ti"] + SPINDLE_KP * max(0.0, err)))
            self.over["left"] = {"label": "hold spindle", "pos": np.asarray(left.moves[i_hold].pos, float), "rot": np.asarray(left.moves[i_hold].rot, float), "push": f_l}
        if int(round(t * 20)) % 2 == 0:
            self.log["stroke"].append({"t": round(t, 2), "phase": st["phase"], "p_cmd": round(math.degrees(p), 2), "p_seen": round(math.degrees(self._wrench_pitch()), 2), "witness": round(w, 2), "spindle": round(spin - st["s0"], 2), "f_r": round(float(np.linalg.norm(f_r)), 1), "f_l": round(float(-f_l[2]), 1), "contact": round(mk.contact_force("wrench", "arbor_nut"), 1), "jaw_mm": [round(1000 * float(v), 1) for v in (mk.pose("wrench")[0] - self.nj)]})
        if st["phase"] == "ease" and scale <= 0.0:
            # End both placeholders here: the right arm goes on from this pose, the left hand lets go
            pull.seconds = t_in
            pull.pos, pull.rot, pull.push = pos, rot, (0.0, 0.0, 0.0)
            # the left hand keeps the wheel (no push) until the jaw is drawn off: let go, the shaft turned back with the
            # handle and the jaw stayed loaded on the hex, dragged off at 40 N (run gw_d17)
            if i_hold is not None:
                left.moves[i_hold].push = (0.0, 0.0, 0.0)
            self.over.pop("right", None)
            self.over.pop("left", None)
            st["phase"] = "done"
            self.log["stroke_end"] = {"t": round(t, 2), "witness": round(w, 2), "pitch_deg": round(math.degrees(p), 2), "spindle": round(spin - st["s0"], 2)}

    def __call__(self, t, arms, pictures) -> None:
        if self.setup is not None:
            self.setup(t)
        j, _ = _active(arms["right"], t)
        if j < len(arms["right"].moves) and arms["right"].moves[j].label in ("press nut on", "finger-snug nut", "let go collar", "before hex nut") and int(round(t * 20)) % 4 == 0:
            pn, rn = self.mk.pose("arbor_nut")
            self.log.setdefault("nut_square", []).append({"t": round(t, 2), "move": arms["right"].moves[j].label, "tilt_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, float(rn[1, 1]))))), 2),
                                                          "radial_mm": round(1000 * self.m.radial_error(pn), 2), "turned_deg": round(float(self.m.STATE.get("nut_turned_deg", 0.0)), 1), "caught": self.m.nut_engaged()})
        if not self.patched:
            self._patch(arms)
        right = arms["right"]
        labels = {mv.label for mv in right.moves}
        for hover, (part, last) in self.AIMS.items():
            if hover in labels and self._fire(right, t, hover):
                self._aim(right, t, hover, part, last)
        if "flange push hover" in labels and self._fire(right, t, "flange push hover"):
            self._flange_touch(right, t)
        if "nut end-on hover" in labels and self._fire(right, t, "nut end-on hover"):
            self._endon(right, t)
        if "lift spanner" in labels and self._fire(right, t, "lift spanner"):
            self._set_entry(right, t)
        for hover in ("jaw hover 1", "jaw hover 2", "jaw hover 3"):
            if hover in labels and self._fire(right, t, hover):
                back = JAW_NEAR if hover == "jaw hover 3" else K.WRENCH_BACK
                jaw = self.nj - back * K.wrench_jaw_dir(self.p_entry)
                self._aim_wrench(right, t, hover, "pull", jaw, self.p_entry)
        if "cradle hover" in labels and self._fire(right, t, "cradle hover"):
            cradle = np.array([P.WRENCH_START[0], P.WRENCH_START[1], K.TOP + P.WRENCH_START[2] + K.WRENCH_LIFT])
            self._aim_wrench(right, t, "cradle hover", "place spanner", cradle, P.WRENCH_REST_PITCH)
        i, t_in = _active(right, t)
        if i < len(right.moves) and right.moves[i].label == "seat 19mm jaw" and not (self.seat or {}).get("done"):
            self._seat(arms, t, i, t_in)
            i, t_in = _active(right, t)
        if i < len(right.moves) and right.moves[i].label == "pull" and (self.stroke is None or self.stroke["phase"] != "done"):
            self._stroke(arms, t, i, t_in)
            i, t_in = _active(right, t)
        if i < len(right.moves) and right.moves[i].label == "back off" and not (self.back or {}).get("done"):
            self._back_off(arms, t, i, t_in)
            i, t_in = _active(right, t)
        if i < len(right.moves) and right.moves[i].label == "draw jaw off" and not (self.draw or {}).get("done"):
            self._draw_off(arms, t, i, t_in)
            i, t_in = _active(right, t)
        i_draw = self._index(right, "draw jaw off")
        if i_draw is not None and i > i_draw and "left" in arms and not self.log.get("left_let_go"):
            left = arms["left"]
            i_hold = self._index(left, "hold spindle")
            if i_hold is not None:
                j, tj = _active(left, t)
                if j == i_hold:
                    left.moves[i_hold].seconds = max(0.05, tj)  # the jaw is off the hex: the left hand lets go now
                self.log["left_let_go"] = round(t, 2)

    def _draw_off(self, arms, t: float, i_draw: int, t_in: float) -> None:
        """Pull the jaw off the hex out along its slot, the pull ramped up (at most 0.9 of any drive) until the spanner has
        come 15 mm, then the planned moves (zero push) take over."""
        mv = arms["right"].moves[i_draw]
        if self.draw is None:
            self.draw = {"p0": np.asarray(self.mk.pose("wrench")[0], float), "t0": t}
        dr = self.draw
        moved = float(np.dot(self.mk.pose("wrench")[0] - dr["p0"], self.slot_out))
        if moved >= 0.015 or t_in >= DRAW_MAX_S - 0.05:
            mv.seconds = max(0.05, t_in)
            self.over.pop("right", None)
            dr["done"] = True
            self.log["draw_off"] = {"t": round(t, 2), "moved_mm": round(1000 * moved, 1), "push_n": round(dr.get("push", 0.0), 1)}
            return
        import demo_kit as kit

        f = min(DRAW_PUSH_MAX, 8.0 + 60.0 * (t - dr["t0"])) * self.slot_out
        jac = kit.point_jacobian(arms["right"].q, arms["right"].base)
        effort = float(np.abs(jac.T @ f / kit.JOINT_EFFORT).max())
        if effort > 0.9:
            f *= 0.9 / effort
        dr["push"] = float(np.linalg.norm(f))
        self.over["right"] = {"label": "draw jaw off", "pos": np.asarray(mv.pos, float), "rot": np.asarray(mv.rot, float), "push": f}

    def _back_off(self, arms, t: float, i_back: int, t_in: float) -> None:
        """Turn the handle back about the nut a degree a step until the jaw bears on the hex with under 2 N (at least
        3 deg, at most BACKOFF_MAX_DEG), then aim the draw along the spanner's slot from there."""
        right = arms["right"]
        if self.back is None:
            pull = right.moves[i_back - 1]
            self.back = {"a": 0.0, "pos": np.asarray(pull.pos, float), "rot": np.asarray(pull.rot, float)}
        bo = self.back
        c = self.mk.contact_force("wrench", "arbor_nut")
        done = (bo["a"] >= math.radians(3.0) and c < 2.0) or bo["a"] >= math.radians(BACKOFF_MAX_DEG) or t_in >= BACKOFF_MAX_S - 0.05
        if not done:
            bo["a"] += math.radians(1.0)
        rb = K.ry(-bo["a"])
        pos, rot = rb @ (bo["pos"] - self.nj) + self.nj, rb @ bo["rot"]
        self.over["right"] = {"label": "back off", "pos": pos, "rot": rot, "push": np.zeros(3)}
        if done:
            back, draw = right.moves[i_back], right.moves[i_back + 1]
            back.seconds, back.pos, back.rot = max(0.05, t_in), pos, rot
            # out along its slot, pulled with a force (a position move left the jaw wedged at 19-40 N and tore it off,
            # gw_d17-d18; a 38 N push along the arbor did not shift it, gw_d20)
            _, wrot = self.mk.pose("wrench")
            slot = wrot @ np.array([math.cos(math.radians(P.WRENCH_JAW_DEG)), 0.0, -math.sin(math.radians(P.WRENCH_JAW_DEG))])
            draw.pos, draw.rot = pos - K.WRENCH_OUT * slot, rot
            self.slot_out = -slot
            self.over.pop("right", None)
            bo["done"] = True
            self.log["back_off"] = {"t": round(t, 2), "deg": round(math.degrees(bo["a"]), 1), "contact": round(c, 1)}


def save_end_picture(frame, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def extra_reader(m):
    mk = m.mk

    def extra() -> dict:
        return {
            "witness_deg": round(m.witness_angle_deg(), 2),
            "torque_nm": round(m.current_torque_nm(), 2),
            "wrench_nut_n": round(mk.contact_force("wrench", "arbor_nut"), 1),
            "flange_nut_n": round(mk.contact_force("outer_flange", "arbor_nut"), 2),
            "spindle_deg": round(math.degrees(mk.joint("spindle")), 2),
            "wheel_seated": m.wheel_seated(),
            "flange_seated": m.flange_seated(),
            "wheel_gap_mm": round(1000.0 * m.wheel_inner_gap(), 2),
            "flange_gap_mm": round(1000.0 * m.flange_rim_gap(), 2),
            "flange_tilt_deg": round(m.flange_tilt_deg(), 2),
            "nut_turned_deg": round(float(m.STATE.get("nut_turned_deg", 0.0)), 2),
            "wrench_off_mm": round(1000.0 * m.wrench_offset_m(), 1),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "parts_mm": {n: [round(1000.0 * float(v), 1) for v in mk.pose(n)[0]] for n in m.FREE_PARTS},
        }

    return extra


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

    folder = out / "full"
    record = ("--no-video" not in sys.argv[2:]) and ("--check" not in sys.argv[2:])
    latest: dict = {}
    director = Director(m)

    def direct(t, arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)
        director(t, arms, pictures)

    result = kit.run(
        embodiment,
        moves,
        folder,
        seed=0,
        director=direct,
        extra=extra_reader(m),
        limit_s=float(m.RUN_LIMIT_STEPS) / 20.0,
        tail_s=4.0,
        sheet_frames=16,
        record=record,
    )
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)
    first_half = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_full = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)
    readings = {
        "score at the end": rows[-1]["info"].get("score") if rows else 0.0,
        "steps": len(rows),
        "seconds": rows[-1]["t"] if rows else 0.0,
        "first 0.5 at (s)": first_half,
        "first 1.0 at (s)": first_full,
        "director": director.log,
        "report": result.get("report"),
        "worst IK": result.get("worst_ik"),
        "median step (ms)": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps({k: v for k, v in readings.items() if k != "director"}, default=str), flush=True)
    print("DIRECTOR " + json.dumps({k: v for k, v in director.log.items() if k != "stroke"}, default=str), flush=True)
    return 0 if readings["score at the end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
