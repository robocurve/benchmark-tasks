"""Scripted two-arm demo for the contact-insert task, closed loop on the simulator's object poses.

    python demo.py --variant full --out ../../results/ct_demo_N      # contact in cavity 3, clicked, lock in: 1.0
    python demo.py --variant all --no-video

1. left arm: grips the housing's front half from above (tilted, its wrist leaning away from the other hand) and
   holds it on the table;
2. right arm: takes the wire right behind the crimp from above (over the rest's gap, tilted, its wrist leaning out),
   lifts the contact, lines it up with cavity 3 (closed loop on the measured nose) and pushes it in along the cavity
   until its fingers reach the housing's rear face; lets go, takes the wire again 30 mm behind the crimp and pushes on
   until the contact clicks (seated); still gripping the wire, tugs it (draws it back along the cavity until the
   contact comes back onto its locking fingers, holds 1 s, eases off); lets go and moves away;
3. the left hand lets go of the housing and moves away (hands off: the task's independent retention test tugs the wire);
4. right arm: grips the housing's rear half from above and holds it on the table;
5. left arm: takes the secondary lock (a wedgelock) out of its holder by the edges of its face plate, turns it so its
   blades point back along the cavities, lines them up with the channels in the mating face (closed loop on the
   measured lock), pushes it in until the plate is on the mating face, lets go and moves home;
6. the right hand lets go of the housing and moves home.

Writes ``result.json``, ``steps.jsonl`` and (unless ``--no-video``) one H.264 video per camera to ``--out``.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
INSTRUCTION = (
    "Push the crimped contact into cavity 3 of the connector, as on the pinout card, from the wire side, until it "
    "clicks; hold the housing with one hand and push the wire close behind the crimp with the other. Tug the wire "
    "lightly to check it is locked, then push the secondary lock fully into the housing's mating face. Then let go of "
    "everything."
)
VARIANTS = ("full", "hold_on", "no_lock", "no_tug", "shallow", "forced", "wrong_cavity")
# per variant: the cavity, push until it clicks (else stop with the shoulder under the fingers)?, press the secondary
# lock in?, let go of the lock?, the robot's own tug after the click?
PLANS = {"full": (3, True, True, True, True),  # 1.0
         "hold_on": (3, True, True, False, True),  # 0.75: the hand stays on the lock
         "no_lock": (3, True, False, True, True),  # 0.5: clicked and tugged, no secondary lock
         "no_tug": (3, True, True, True, False),  # 0.25: clicked, the lock in, but the robot never tugs
         "shallow": (3, False, False, True, False),  # 0.25: pushed in short of the click
         "forced": (3, False, True, True, False),  # 0: the lock pressed over a contact short of the click
         "wrong_cavity": (2, True, True, True, True)}  # 0: cavity 2
CAMERAS = ("top_cam", "left_cam", "right_cam")
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
MAX_STEPS = 3000  # the task's run limit
MM = 1e-3

PAD_TIP = 0.0128  # the pads' tips below the TCP
TILT = math.radians(30.0)  # both hands tilted, wrists leaning away from each other (cord-clamp lesson: wrists collide)
# -- the housing hand
H_GRIP_Y = 31 * MM  # the grip's centre along the housing (from its rear face; front half)
H_TIPS_Z = 3 * MM  # the pads' tips above the table
OPEN_H, GRIP_H = 0.45, 0.0
# -- the contact hand
W_GRIP_1, W_GRIP_2 = 15 * MM, 30 * MM  # the grips along the wire, behind the crimp (first stroke, second stroke)
OPEN_W, GRIP_W = 0.25, 0.0
W_TIPS_BELOW = 4 * MM  # the pads' tips this far below the wire's axis
LIFT = 15 * MM
PRE_NOSE = 4 * MM  # lined up with the nose this far before the rear face
PUSH_SPEED = 6 * MM  # m/s
PUSH_LEAD = 25 * MM  # the push command runs at most this far ahead of the measured hand
PAD_CLEAR = 3 * MM  # the first stroke ends with the pads' front edge this far from the rear face
PAD_HALF = 13.5 * MM
SEAT_EXTRA = 1.5 * MM  # pushed until the crimp end is this past its seated depth (then let go: the fingers hold it)
ALIGN_GAIN, ALIGN_STEPS = 0.6, 30
# -- the robot's own tug (still gripping the wire after the click, the other hand holding the housing)
TUG_SPEED = 3 * MM  # m/s: the command drawn back along the cavity ...
TUG_LEAD = 12 * MM  # ... at most this far behind where the hand was
TUG_PULL_TO = 0.2 * MM  # until the crimp end is this far short of its seated depth (the shoulder on the fingers)
TUG_TOO_FAR = 0.8 * MM  # (a light tug: the fingers hold the shoulder 0.26 mm back; beyond this, stop pulling)
TUG_HOLD_S, TUG_MAX_S = 1.0, 6.0
# -- the second housing grip (the right hand, while the lock goes in)
H_GRIP_REAR_Y = 15 * MM  # the grip's centre along the housing (from its rear face; rear half)
TUG_WAIT_MAX_S = 6.0  # wait at most this long for the task's retention test before gripping the housing again
# -- the secondary lock (a wedgelock: gripped by its plate's two side edges, the pads running along its blades' width)
OPEN_L, GRIP_L = 0.45, 0.0
L_TIPS_ABOVE_BAR = 1.5 * MM  # the pads' tips this far above the plate's underside (off the mating face when it is in)
L_PAD_OFFSET = 1 * MM  # the grip point this far along the plate's y from its centre. Tilted, the fingers reach back
# from it along -y (toward the wrist): at -8 to -13 mm they overlapped the plate's edge by 3 mm or less and slipped
# (demos p2, p5-p7, fingertips logged in the lock's frame)
L_TILT = math.radians(35.0)  # (a level push into the face put the left wrist on its joint-4 limit: IK 10 mm short;
# at 40 degrees the IK lost the arm on the way to the holder, demo p3)
L_LIFT = 45 * MM  # lifted this far out of the holder before it is turned
L_STAGE = 12 * MM  # the blades' tips this far out from the mating face while lining up
L_PRESS = 6 * MM  # pushed this far past flush (gripped)
PUSH_RETRIES = 3  # a push that stalls before the click: back off and push again
SETTLE_TOL, SETTLE_MAX_S = 0.6 * MM, 2.0
AFTER_SUCCESS_S = 3.0
IK_ABORT_MM, IK_ABORT_S = 20.0, 2.0


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def rot_axis(n, a: float) -> np.ndarray:
    n = np.asarray(n, dtype=float) / np.linalg.norm(n)
    k = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
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


def hand_rot(arm, along, lean, tilt=TILT) -> np.ndarray:
    """A top-down grip whose fingers close across ``along`` (horizontal), tilted by ``tilt`` so that the wrist leans
    toward ``lean`` (horizontal unit vector; along ``along`` keeps the pads parallel to what they grip)."""
    a = np.array([along[0], along[1], 0.0])
    a /= np.linalg.norm(a)
    r = rot_z(wrap_pi_half(math.atan2(a[1], a[0]))) @ arm.down
    lean = np.array([lean[0], lean[1], 0.0]) / np.linalg.norm(lean[:2])
    axis = np.cross(-lean, (0.0, 0.0, 1.0))  # tilting about this leans the approach toward -lean: wrist toward lean
    return rot_axis(axis, tilt) @ r


def run(variant: str, out: Path, video: bool, max_steps: int, emb=None) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack, split

    from yam_arena import ik, station
    from yam_arena.demo import grip_rotation

    emb = emb or make_embodiment(video)
    obs = emb.reset(Scene(id=f"contact-insert-{variant}", instruction=INSTRUCTION), seed=0)
    set_rendering(emb, video)
    task = emb.task.module
    ct = task.ct
    base_env = emb._env.unwrapped
    from yam_arena.scene import object_states

    cavity, seat, press_lock, let_go_lock, robot_tug = PLANS[variant]
    left_q, right_q = split(np.asarray(obs.state["joint_pos"]))
    arms = {"left": Arm("left", station.BASES["left"], left_q[:6], ik, grip_rotation),
            "right": Arm("right", station.BASES["right"], right_q[:6], ik, grip_rotation)}
    world = {"states": object_states(emb._env, emb.task.names), "obs": obs, "step": 0}
    flags: dict = {}
    report: dict = {}
    phase = {"left": "", "right": ""}

    # -- measurements -------------------------------------------------------------------------------------------
    def housing():
        return ct.body_poses(base_env, ct._NAMES["housing"])["base"]

    def cavity_axis(c):
        """(world point at cavity c's mouth on the rear face, unit direction into the cavity)."""
        h_p, h_r = housing()
        xc, zc = ct.CAVITIES[c]
        return h_p + h_r @ np.array([xc, 0.0, zc]), h_r @ np.array([0.0, 1.0, 0.0])

    def contact():
        """(crimp end, unit direction from the crimp along the wire, nose point) in the world."""
        p, r = ct.body_poses(base_env, ct._NAMES["contact"])["contact"]
        return p, r[:, 0], p + r @ np.array([ct.NOSE_X, 0.0, 0.0])

    def wire_point(dist):
        """The wire's axis ``dist`` behind the crimp (along the links as they lie)."""
        poses = ct.body_poses(base_env, ct._NAMES["contact"])
        i = min(int(dist // ct.WIRE_LINK), ct.WIRE_LINKS - 1)
        p, r = poses[f"w{i}"]
        return p + r @ np.array([dist - i * ct.WIRE_LINK, 0.0, 0.0]), r[:, 0]

    def lock():
        return ct.body_poses(base_env, ct._NAMES["lock"])["bar"]

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

    def tcp_for_tips(arm: Arm, tips_at, rot):
        """The TCP that puts the pads' tips at ``tips_at`` with the hand turned ``rot``."""
        return np.asarray(tips_at) - PAD_TIP * rot[:, 2]

    # -- left arm: the housing ------------------------------------------------------------------------------------
    def left_script():
        arm = arms["left"]
        home, home_rot = arm.pos.copy(), arm.rot.copy()
        h_p, h_r = housing()
        along = h_r @ np.array([0.0, 1.0, 0.0])
        rot = hand_rot(arm, along, along)  # the wrist leaning toward the housing's front (away from the other hand)
        grip_at = h_p + h_r @ np.array([0.0, H_GRIP_Y, 0.0])
        grip_at[2] = task.TOP + H_TIPS_Z  # the pads' tips
        tcp = tcp_for_tips(arm, grip_at, rot)
        yield from move(arm, tcp + (0, 0, 0.06), 2.5, grip=OPEN_H, rot=rot, label="over the housing")
        yield from move(arm, tcp, 1.5, label="down round the housing")
        yield from settle_at(arm, "settle")
        yield from move(arm, arm.pos - (0, 0, 2 * MM), 0.8, grip=GRIP_H, label="grip the housing")
        flags["housing_held"] = True
        yield from wait_for(arm, "contact_in", "hold the housing")
        yield from move(arm, arm.pos, 0.6, grip=OPEN_H, label="let go of the housing")
        yield from move(arm, arm.pos + (0, 0, 0.06), 1.0, label="up")
        flags["left_away"] = True
        if press_lock:
            yield from move(arm, home, 2.0, rot=home_rot, label="home first")  # (straight from the housing to the tilted
            # grip over the holder the IK lost the arm, demos p3-p4)
            yield from fit_lock(arm)
        yield from move(arm, home, 2.0, rot=home_rot, label="home")
        flags["left_done"] = True
        phase["left"] = "done"
        while True:
            yield

    # -- right arm: the contact, then the lock ------------------------------------------------------------------
    def grip_wire(arm, dist, label):
        """Take the wire ``dist`` behind the crimp from above (fingers across it, tilted)."""
        p, d = wire_point(dist)
        rot = hand_rot(arm, d, d)  # the wrist leaning back along the wire (away from the housing)
        tips = p - np.array([0.0, 0.0, W_TIPS_BELOW])
        tcp = tcp_for_tips(arm, tips, rot)
        yield from move(arm, tcp + (0, 0, 0.04), 1.8, grip=OPEN_W, rot=rot, label=f"{label}: over the wire")
        p, d = wire_point(dist)
        tcp = tcp_for_tips(arm, p - np.array([0.0, 0.0, W_TIPS_BELOW]), rot)
        yield from move(arm, tcp, 1.0, label=f"{label}: down to the wire")
        yield from settle_at(arm, f"{label}: settle")
        yield from move(arm, arm.pos, 0.6, grip=GRIP_W, label=f"{label}: grip the wire")
        yield from wait(arm, 0.3, f"{label}: gripped")

    def line_up(arm, c, nose_y, label):
        """Closed loop: move the hand until the contact's nose is on cavity c's axis, ``nose_y`` from the rear face
        (negative: outside), the contact pointing along the cavity."""
        for k in range(ALIGN_STEPS):
            mouth, d = cavity_axis(c)
            _, wdir, nose = contact()
            target = mouth + d * nose_y
            err = target - nose
            phase[arm.side] = f"line up ({np.linalg.norm(err) / MM:.1f} mm)"
            if np.linalg.norm(err) < 0.3 * MM and k > 5:
                break
            arm.pos = arm.pos + ALIGN_GAIN * err
            for _ in range(4):
                yield

    def push(arm, c, until, label):
        """Push along cavity c until ``until()`` or the hand stalls; returns the reason."""
        _, d = cavity_axis(c)
        d = np.array([d[0], d[1], 0.0]) / np.linalg.norm(d[:2])
        track = []
        phase[arm.side] = label
        for _ in range(round(12.0 / DT)):
            if until():
                return "done"
            meas = tcp_measured(arm)
            track.append(float(meas @ d))
            lead = float((arm.pos - meas) @ d)
            if lead > PUSH_LEAD - 1 * MM and len(track) > 30 and track[-1] - track[-31] < 0.3 * MM:
                return "stalled"
            if lead < PUSH_LEAD:
                arm.pos = arm.pos + d * PUSH_SPEED * DT
            yield
        return "timeout"

    def tug(arm, c):
        """The robot's own tug: draw the gripped wire back along cavity c until the contact is pulled back onto its
        locking fingers, hold it there TUG_HOLD_S, then ease off (the command back to the measured hand)."""
        _, d = cavity_axis(c)
        d = np.array([d[0], d[1], 0.0]) / np.linalg.norm(d[:2])
        start = tcp_measured(arm)
        arm.pos = start.copy()
        phase[arm.side] = "tug the wire"
        held, deepest_back = 0, 0.0
        for _ in range(round(TUG_MAX_S / DT)):
            back = ct.Y_SEATED_CRIMP - float(ct.geometry(base_env)["crimp"][1])
            deepest_back = max(deepest_back, back)
            if back > TUG_TOO_FAR:  # pulled harder than a light tug: stop pulling
                arm.pos = tcp_measured(arm)
                break
            if back >= TUG_PULL_TO:
                held += 1
                if held >= round(TUG_HOLD_S / DT):
                    break
            else:
                held = 0
                if float((start - arm.pos) @ d) < TUG_LEAD:
                    arm.pos = arm.pos - d * TUG_SPEED * DT
            yield
        report["tug"] = {"held_s": round(held * DT, 2), "max_back_mm": round(deepest_back / MM, 3),
                         "robot_tug": bool(world["states"]["housing"].robot_tug), "step": world["step"]}
        arm.pos = tcp_measured(arm)
        yield from wait(arm, 0.3, "ease off the tug")

    def right_script():
        arm = arms["right"]
        home, home_rot = arm.pos.copy(), arm.rot.copy()
        yield from wait_for(arm, "housing_held", "wait for the housing")
        yield from grip_wire(arm, W_GRIP_1, "stroke 1")
        yield from move(arm, arm.pos + (0, 0, LIFT), 1.0, label="lift the contact")
        mouth, d = cavity_axis(cavity)
        _, _, nose = contact()
        yield from move(arm, arm.pos + (mouth - d * PRE_NOSE - nose), 2.0, label="to the cavity")
        yield from line_up(arm, cavity, -PRE_NOSE, "line up")
        mouth, d = cavity_axis(cavity)

        def pads_at_face():
            p, _ = wire_point(W_GRIP_1)
            return float((p - mouth) @ d) > -(PAD_HALF + PAD_CLEAR)

        g = push(arm, cavity, pads_at_face, "push (1)")
        report["stroke1"] = yield from g
        report["after_stroke1_mm"] = round(float(ct.geometry(base_env)["crimp"][1]) / MM, 2)
        arm.pos = tcp_measured(arm)
        yield from move(arm, arm.pos, 0.5, grip=OPEN_W, label="let go to re-grip")
        yield from move(arm, arm.pos + (0, 0, 0.02), 0.6, label="up")
        yield from grip_wire(arm, W_GRIP_2, "stroke 2")
        target = ct.Y_SEATED_CRIMP + SEAT_EXTRA if seat else ct.Y_LAT[0] + ct.SHOULDER_FRONT_X + 1.0 * MM

        def deep_enough():
            return float(ct.geometry(base_env)["crimp"][1]) >= target

        for k in range(PUSH_RETRIES + 1):
            g = push(arm, cavity, deep_enough, f"push (2.{k})")
            report["stroke2"] = yield from g
            if report["stroke2"] == "done" or not seat:
                break
            _, d = cavity_axis(cavity)
            arm.pos = tcp_measured(arm) - np.array([d[0], d[1], 0.0]) * 1.5 * MM  # back off, push again
            yield from wait(arm, 0.4, f"back off ({k + 1})")
        st = world["states"]["housing"]
        report["pushed"] = {"crimp_mm": round(st.crimp_depth / MM, 2), "cavity": st.cavity, "seated": st.seated,
                            "step": world["step"]}
        if robot_tug:
            yield from tug(arm, cavity)
        arm.pos = tcp_measured(arm)
        yield from move(arm, arm.pos, 0.5, grip=OPEN_W, label="let go of the wire")
        yield from move(arm, arm.pos + (0, 0, 0.05), 1.0, label="up")
        flags["contact_in"] = True
        yield from move(arm, home, 2.0, rot=home_rot, label="home (the tug)")
        yield from wait_for(arm, "left_away", "wait for the other hand")
        phase[arm.side] = "wait for the tug"
        for _ in range(round(TUG_WAIT_MAX_S / DT)):  # the task's retention test runs with both hands off the wire
            if world["states"]["housing"].tug in ("passed", "failed"):
                break
            yield
        if press_lock:
            yield from wait_for(arm, "lock_lifted", "wait for the lock")  # (moving in while the other hand took the
            # lock out of its holder, the arms met and knocked it out of the grip, demo v_forced)
            yield from hold_rear(arm)
            yield from wait_for(arm, "lock_done", "hold the housing")
            yield from move(arm, arm.pos, 0.6, grip=OPEN_H, label="let go of the housing")
            yield from move(arm, arm.pos + (0, 0, 0.06), 1.0, label="up")
        yield from move(arm, home, 2.5, rot=home_rot, label="home")
        flags["right_done"] = True
        phase["right"] = "done"
        while True:
            yield

    def hold_rear(arm):
        """Grip the housing's rear half from above (the wrist leaning back toward the wire's side) and hold it."""
        h_p, h_r = housing()
        along = h_r @ np.array([0.0, 1.0, 0.0])
        rot = hand_rot(arm, along, -along)
        grip_at = h_p + h_r @ np.array([0.0, H_GRIP_REAR_Y, 0.0])
        grip_at[2] = task.TOP + H_TIPS_Z
        tcp = tcp_for_tips(arm, grip_at, rot)
        yield from move(arm, tcp + (0, 0, 0.06), 2.0, grip=OPEN_H, rot=rot, label="over the housing's rear")
        yield from move(arm, tcp, 1.5, label="down round the housing")
        yield from settle_at(arm, "settle")
        yield from move(arm, arm.pos - (0, 0, 2 * MM), 0.8, grip=GRIP_H, label="grip the housing")
        flags["rear_held"] = True

    def fit_lock(arm):
        l_p, l_r = lock()
        along = l_r @ np.array([0.0, 1.0, 0.0])  # the pads run along the plate's y; the fingers close on its x edges
        _, h_r = housing()
        rot = hand_rot(arm, along, -(h_r @ np.array([0.0, 1.0, 0.0])), tilt=L_TILT)  # tilted about the closing axis,
        # the wrist leaning toward the housing's rear: turned, the hand comes down onto the mating face at L_TILT
        tips = l_p + l_r @ np.array([0.0, L_PAD_OFFSET, L_TIPS_ABOVE_BAR])
        tcp = tcp_for_tips(arm, tips, rot)
        yield from move(arm, tcp + (0, 0, 0.05), 2.5, grip=OPEN_L, rot=rot, label="over the lock")
        yield from move(arm, tcp, 1.2, label="down to the lock")
        yield from settle_at(arm, "settle")
        yield from move(arm, arm.pos, 0.6, grip=GRIP_L, label="grip the lock")
        l_p, l_r = lock()  # (where the fingertips are, in the lock's frame: for tuning the grasp)
        dbg = {}
        for art_name, art in base_env.scene.articulations.items():
            if art_name in ct._NAMES.values():
                continue
            poses_r = ct.body_poses(base_env, art_name)
            for b in (n for n in art.body_names if n.startswith("tip")):
                p, r = poses_r[b]
                q = (ct.finger_points(b) @ r.T + p - l_p) @ l_r
                if np.linalg.norm(q.mean(axis=0)) < 0.08:
                    dbg[f"{art_name}/{b}"] = [[round(float(v) / MM, 1) for v in q.min(axis=0)],
                                              [round(float(v) / MM, 1) for v in q.max(axis=0)]]
        report["grip_fingers_in_lock_mm"] = dbg
        z0 = float(lock()[0][2])
        yield from move(arm, arm.pos + (0, 0, L_LIFT), 1.2, label="lift it out")
        yield from wait(arm, 0.3, "lifted")
        report["lock_lifted_mm"] = round((float(lock()[0][2]) - z0) / MM, 1)
        if report["lock_lifted_mm"] < 0.5 * L_LIFT / MM:  # not in the hand: stop rather than wave the arm about
            phase[arm.side] = "the lock is not in the hand"
            while True:
                yield
        flags["lock_lifted"] = True
        yield from wait_for(arm, "rear_held", "wait for the housing to be held")
        h_p, h_r = housing()
        out = h_r @ np.array([0.0, 1.0, 0.0])  # the mating face's outward normal
        turn = rot_axis(np.cross((0.0, 0.0, 1.0), out), math.pi / 2)  # the lock's +z (up) turned onto it
        stage = h_p + h_r @ np.array([0.0, ct.Y_LOCK + ct.LOCK_PRONG[2] + L_STAGE, 0.0])
        l_p, _ = lock()
        tcp = stage - turn @ (l_p - arm.pos)  # (the lock turns with the hand about the TCP)
        yield from move(arm, tcp + (0, 0, 0.04), 3.0, rot=turn @ arm.rot, label="turn it to the face")
        yield from move(arm, tcp, 1.2, label="down in front of the face")
        yield from settle_at(arm, "settle")
        for k in range(ALIGN_STEPS * 2):  # closed loop: the plate square to the face, its centre on the housing's axis
            h_p, h_r = housing()
            stage = h_p + h_r @ np.array([0.0, ct.Y_LOCK + ct.LOCK_PRONG[2] + L_STAGE, 0.0])
            l_p, l_r = lock()
            err = stage - l_p
            ang = np.cross(l_r[:, 2], h_r @ np.array([0.0, 1.0, 0.0]))  # small rotation turning the lock's z onto +y
            phase[arm.side] = f"line up the lock ({np.linalg.norm(err) / MM:.1f} mm, {math.degrees(np.linalg.norm(ang)):.1f} deg)"
            if np.linalg.norm(err) < 0.25 * MM and np.linalg.norm(ang) < math.radians(0.5) and k > 5:
                break
            step_ = ALIGN_GAIN * err
            arm.pos = arm.pos + step_ * min(1.0, 3 * MM / max(np.linalg.norm(step_), 1e-9))
            meas = tcp_measured(arm)  # (never wind the command up more than 4 mm past the measured hand)
            lag = arm.pos - meas
            if np.linalg.norm(lag) > 4 * MM:
                arm.pos = meas + lag * (4 * MM / np.linalg.norm(lag))
            if 1e-6 < np.linalg.norm(ang) < math.sin(math.radians(15.0)):  # (small corrections only)
                arm.rot = rot_axis(ang, ALIGN_GAIN * math.asin(min(1.0, np.linalg.norm(ang)))) @ arm.rot
            for _ in range(4):
                yield
        h_p, h_r = housing()
        d = h_r @ np.array([0.0, 1.0, 0.0])
        yield from move(arm, arm.pos - d * (ct.LOCK_PRONG[2] + L_STAGE - 2 * MM), 2.5, label="blades into the channels")
        yield from move(arm, arm.pos - d * (2 * MM + L_PRESS), 1.5, label="push the lock in")
        yield from wait(arm, 1.0, "pushed")
        st = world["states"]["housing"]
        report["lock"] = {"depth_mm": round(st.lock_depth / MM, 2), "in": st.lock_in, "forced": st.forced,
                          "step": world["step"]}
        if not let_go_lock:
            phase[arm.side] = "keep holding the lock"
            while True:
                yield
        yield from move(arm, arm.pos + d * L_PRESS, 0.6, label="ease off")
        yield from move(arm, arm.pos, 0.5, grip=OPEN_L, label="let go of the lock")
        yield from move(arm, arm.pos + d * 0.05, 1.2, label="back off")
        flags["lock_done"] = True
        yield from move(arm, arm.pos + (0, 0, 0.05), 1.0, label="up")

    scripts = {"left": left_script(), "right": right_script()}
    frames = {c: [] for c in CAMERAS}
    log = []
    result = None
    success_step = None
    t0 = time.time()
    out.mkdir(parents=True, exist_ok=True)
    live = open(out / "steps_live.jsonl", "w")
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
        h = world["states"]["housing"]
        geo = ct.geometry(base_env)
        row = {"step": step, "left": phase["left"], "right": phase["right"], "score": res.info.get("score"),
               "success": res.info.get("success"), "cavity": h.cavity, "crimp_mm": round(h.crimp_depth / MM, 2),
               "nose_mm": round(h.nose_depth / MM, 2), "seated": h.seated, "locked": h.locked,
               "fingers_mm": [round(v / MM, 3) for v in h.latch_open], "lock_in": h.lock_in,
               "lock_depth_mm": round(h.lock_depth / MM, 2), "forced": h.forced, "wrong": h.wrong_locked,
               "tug": h.tug, "tug_moved_mm": round(h.tug_moved / MM, 3), "hands_off": int(h.hands_off_steps),
               "mark_ok": h.mark_ok, "robot_tug": h.robot_tug,
               "crimp_xz_mm": [round(float(geo["crimp"][i]) / MM, 2) for i in (0, 2)],
               "l_ik_mm": round(arms["left"].ik_err[0] / MM, 2), "r_ik_mm": round(arms["right"].ik_err[0] / MM, 2),
               "l_cmd": [round(float(v), 4) for v in arms["left"].pos],
               "l_meas": [round(float(v), 4) for v in tcp_measured(arms["left"])],
               "r_cmd": [round(float(v), 4) for v in arms["right"].pos],
               "r_meas": [round(float(v), 4) for v in tcp_measured(arms["right"])],
               "housing_w": [round(float(v), 4) for v in housing()[0]]}
        log.append(row)
        if step % 20 == 0:
            live.write(json.dumps(row, default=float) + "\n")
            live.flush()
        if video:
            for cam in CAMERAS:
                frames[cam].append(res.observation.images[cam])
        worst_ik = max(row["l_ik_mm"], row["r_ik_mm"])
        report["_ik_bad"] = report.get("_ik_bad", 0) + 1 if worst_ik > IK_ABORT_MM else 0
        if report["_ik_bad"] >= round(IK_ABORT_S / DT):
            report["abort"] = f"step {step}: an arm can't reach its target ({worst_ik:.0f} mm) for {IK_ABORT_S} s"
            break
        result = res
        if res.terminated and success_step is None:
            success_step = step
            report["success_step"] = step
        if success_step is not None and step - success_step >= round(AFTER_SUCCESS_S / DT):
            break
        if success_step is None and flags.get("left_done") and flags.get("right_done") and h.hands_off_steps >= 120:
            break
    live.close()
    if "lock" in ct._NAMES:  # where the lock ended, in the housing's frame (for tuning)
        h_p, h_r = housing()
        l_p, l_r = lock()
        report["lock_end_in_housing_mm"] = [round(float(v) / MM, 2) for v in h_r.T @ (l_p - h_p)]
        report["lock_end_axis"] = [round(float(v), 4) for v in h_r.T @ l_r[:, 2]]
    (out / "steps_live.jsonl").unlink()
    h = world["states"]["housing"]
    summary = {
        "variant": variant,
        "task_score": 1.0 if success_step is not None else float(result.info.get("score", 0.0)) if result else 0.0,
        "success_step": success_step,
        "steps": len(log),
        "run_limit_steps": MAX_STEPS,
        "final": {k: getattr(h, k) for k in ("cavity", "crimp_depth", "seated", "locked", "latch_open", "lock_in",
                                              "lock_depth", "forced", "wrong_locked", "tug", "tug_moved",
                                              "mark_ok", "robot_tug")},
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
    """One H.264 MP4 per camera, written through imageio-ffmpeg (OpenCV's MJPG writer failed on the L40S box)."""
    import imageio_ffmpeg

    for cam, imgs in frames.items():
        if not imgs:
            continue
        h, w = imgs[0].shape[:2]
        name = out / (f"demo_{cam}.mp4" if variant == "full" else f"demo_{variant}_{cam}.mp4")
        writer = imageio_ffmpeg.write_frames(str(name), (w, h), fps=CONTROL_HZ, codec="libx264", macro_block_size=8,
                                             output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
        writer.send(None)
        for im in imgs:
            writer.send(np.ascontiguousarray(im))
        writer.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", choices=VARIANTS + ("all",), default="full")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a constant: --set PUSH_SPEED=0.005, or a part's with --set ct.LAT_K=5e3")
    args = ap.parse_args(argv)
    for item in args.set:
        name, value = item.split("=", 1)
        if name.startswith("ct."):
            sys.path.insert(0, str(HERE))
            import ct_parts

            target = vars(ct_parts)
        else:
            target = globals()
        name = name.removeprefix("ct.")
        if name not in target:
            ap.error(f"--set: no constant {name!r}")
        try:
            target[name] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            target[name] = value
    video = not args.no_video
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
