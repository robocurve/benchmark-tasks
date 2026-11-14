"""Set a twist-lock extension pole to its mark and lock it: the lock has no detent, so only a push proves it.

A 2 to 4 ft aluminium twist-lock extension pole lies loose on the table: outer tube 25 mm across, inner tube 22 mm, 61 cm collapsed. An eccentric cam inside locks the length
when the inner tube is turned a quarter turn against the outer. Both arms counter-twist the tubes to release the cam (one arm alone only rolls the pole: a round tube resists
rolling by about 0.006 N m, the cam by 0.3 N m), draw the inner tube out until the red band reaches the outer tube's end, and counter-twist again to lock it. Then the scorer
pushes the pole's end with 50 N along its length: a locked pole holds.

Sim: one free mechanism. The outer tube is its frame (0.25 kg); the inner tube slides (inner_slide, along y) and turns (inner_rot, about y) inside it. The cam is a stated
coupling in code: within 20 degrees of inner_rot 0 (as found) the slide is held with up to 200 N; turned 70 degrees or more either way it slides against 2 N; between, the
hold falls linearly. The cam's
turning torque is a drive on inner_rot that holds its angle with at most 0.3 N m (a typical value, to be measured).

    python demokit.py task.py OUT --mode full                                     the full-score reference, filmed
    python demokit.py task.py OUT --no-record --mode one_arm,nolock,short --hold 1  failure checks: 0, 0.5, 0
    python demokit.py task.py OUT --no-record --no-cameras --mode g_roll,g_pull --hold 0   code-force checks of the cam
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Twist the pole's tubes against each other to free its lock, set it to the red mark, and lock it again."
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.5

R_OUT, R_IN = 0.0125, 0.011
L_OUT, L_IN = 0.55, 0.55
COLLAPSED = 0.61  # overall length, collapsed
TARGET = float(os.environ.get("POLE_TARGET", "0.85"))  # overall length at the mark
EXTEND = TARGET - COLLAPSED  # inner_slide's travel to the mark
CAM_T = 0.3  # N m to turn the cam (a typical value): below what a 45 N pinch turns on a 22 mm tube, about 2 x 0.5 x 45 N x 0.011 m = 0.5 N m
UNLOCKED, LOCKED = math.radians(70.0), math.radians(20.0)  # turned this far either way: free; within this: held
HOLD_N, SLIDE_N = 200.0, 2.0
PUSH_N = 50.0
X0 = 0.40
Y_END = 0.40  # the outer tube's closed end (cap), towards the left arm
Y_MOUTH = Y_END - L_OUT  # the outer tube's open end, where the inner tube comes out
LOOKS = {"alu": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=0.9, roughness=0.35), "inner": m.Look(color=m.rgb(0.70, 0.71, 0.73), metallic=0.9, roughness=0.3),
         "grip": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.85), "cam": m.Look(color=m.rgb(0.95, 0.75, 0.10), roughness=0.5),
         "red": m.Look(color=m.rgb(0.85, 0.08, 0.06), roughness=0.5), "tip": m.Look(color=m.rgb(0.12, 0.12, 0.13), roughness=0.6)}


def _tube(r, y0, y1, n=32):
    return m.cylinder(r, y1 - y0, (0.0, (y0 + y1) / 2, 0.0), axis="y", segments=n)


def _pole() -> m.Part:
    # in the part's frame: the pole's axis along y, the outer tube from Y_MOUTH to Y_END (relative to the part's origin at x X0, y 0)
    outer = _tube(R_OUT, Y_MOUTH, Y_END)
    cap = _tube(R_OUT + 0.002, Y_END - 0.03, Y_END + 0.005)
    collar = _tube(R_OUT + 0.0015, Y_MOUTH, Y_MOUTH + 0.025)
    # the inner tube, at joint zero: from inside the outer up to 6 cm past its mouth; the red band where it meets the mouth at the mark
    in0, in1 = Y_MOUTH - (COLLAPSED - L_OUT), Y_MOUTH - (COLLAPSED - L_OUT) + L_IN
    inner = _tube(R_IN, in0, in1)
    band = _tube(R_IN + 0.0003, Y_MOUTH + EXTEND - 0.010, Y_MOUTH + EXTEND, 32)
    tip = _tube(R_IN + 0.003, in0 - 0.03, in0, 24)  # the threaded tip's grey sleeve
    cam = m.box((0.004, 0.006, 0.004), (0.0, in1 - 0.01, R_IN))  # a stripe to read the inner tube's turn by
    stripe = m.box((0.002, 0.04, 0.0015), (0.0, Y_MOUTH - 0.02, R_IN + 0.0003))
    slide = m.slider("inner_slide", "Y", -0.45, 0.0, [], [], mass=0.02, spring=0.0, damping=2.0, friction=0.0, effort=HOLD_N)
    rot = m.hinge("inner", "Y", (0.0, 0.0, 0.0), -100.0, 100.0, [("inner", inner, "inner"), ("band", band, "red"), ("tip", tip, "tip"), ("stripe", stripe, "cam")],
                  [m.col_hull("inner_c", _tube(R_IN, in0 - 0.03, Y_MOUTH, 32))], mass=0.20, spring=20.0, effort=CAM_T, friction=0.0, damping=0.002, parent="inner_slide",
                  inertia=(0.2 * L_IN ** 2 / 12, 0.2 * R_IN ** 2, 0.2 * L_IN ** 2 / 12))  # a thin tube: little about its own axis, so a twist turns it rather than the kit's round guess
    frame = [("outer", outer, "alu"), ("cap", cap, "grip"), ("collar", collar, "grip"), ("cam", cam, "cam")]
    return m.mechanism("pole", (X0, 0.0, TOP + R_OUT + 0.0005), [slide, rot], LOOKS, frame_parts=frame,
                       frame_colliders=[m.col_hull("outer_c", _tube(R_OUT, Y_MOUTH, Y_END + 0.005, 32))], free_mass=0.25, friction=(0.6, 0.5),
                       frame_inertia=(0.25 * L_OUT ** 2 / 12, 0.25 * R_OUT ** 2, 0.25 * L_OUT ** 2 / 12))


OBJECTS = [_pole()]


class Cam:
    """The cam's coupling, and the scorer's trial-long record: where the band is, whether locked, and the push test once the grippers are clear."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.cap, self.hold_at = 0, None, None
        self.clear_for, self.push_t, self.push_start, self.short_mm, self.passed = 0, 0.0, None, None, False
        self.off_table = False
        self.last = {}

    def step(self, env):
        import torch

        self.steps += 1
        dt = float(env.unwrapped.step_dt)
        s = m.state(env, "pole")
        rot, ext = s.joint("inner"), -s.joint("inner_slide")
        # the cam's turning resistance: its drive holds the angle where it is now, with at most CAM_T, so a twist above CAM_T turns it (joint friction would scale with the axial load)
        asset = env.unwrapped.scene["pole"]
        asset.set_joint_position_target(torch.tensor([[rot]], device=asset.device), joint_ids=[asset.find_joints("inner_joint")[0][0]])
        f = (UNLOCKED - abs(rot)) / (UNLOCKED - LOCKED)
        cap = SLIDE_N + (HOLD_N - SLIDE_N) * float(np.clip(f, 0.0, 1.0))
        if self.cap is None or abs(cap - self.cap) > 1.0:
            asset = env.unwrapped.scene["pole"]
            j = asset.find_joints("inner_slide_joint")[0][0]
            if cap > SLIDE_N + 1.0:
                if self.hold_at is None:
                    self.hold_at = s.joint("inner_slide")
                m._TOUCHED.add(("pole", "inner_slide"))
                asset.write_joint_stiffness_to_sim(torch.tensor([[50000.0]], device=asset.device), joint_ids=[j])
                asset.set_joint_position_target(torch.tensor([[self.hold_at]], device=asset.device), joint_ids=[j])
            else:
                self.hold_at = None
                asset.write_joint_stiffness_to_sim(torch.tensor([[0.0]], device=asset.device), joint_ids=[j])
            asset.write_joint_effort_limit_to_sim(torch.tensor([[cap]], device=asset.device), joint_ids=[j])
            self.cap = cap
        # the scorer's push, once the pole is at its mark, locked, and both grippers 3 cm clear for 0.5 s
        tips = m.fingertips(env)
        axis_pts = s.to_world(np.array([[0.0, y, 0.0] for y in np.linspace(Y_MOUTH - EXTEND - 0.06, Y_END, 25)]))
        gap = float(np.min(np.linalg.norm(tips[:, None, :] - axis_pts[None, :, :], axis=2))) - R_OUT
        at_mark = abs(ext - EXTEND) < 0.010
        locked = abs(rot) <= LOCKED
        self.clear_for = self.clear_for + 1 if (gap > 0.03 and at_mark and locked) else 0
        asset = env.unwrapped.scene["pole"]
        i = int(asset.find_bodies("inner")[0][0])
        if self.clear_for * dt >= 0.5 and self.short_mm is None:
            if self.push_start is None:
                self.push_start = ext
            self.push_t += dt
            fw = s.rotation() @ np.array([0.0, PUSH_N, 0.0])  # along +y in the pole's frame: pushes the inner tube in, the outer's end held back as a floor would
            fr = int(asset.find_bodies("frame")[0][0])
            asset.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.tensor([[fw.tolist(), (-fw).tolist()]], device=asset.device),
                                                                         torques=torch.zeros((1, 2, 3), device=asset.device), body_ids=[i, fr], is_global=True)
            m._PUSHED.add(("pole", i))
            m._PUSHED.add(("pole", fr))
            if self.push_t >= 2.0:
                self.short_mm = (self.push_start - ext) * 1000
                self.passed = self.short_mm < 2.0
                asset.permanent_wrench_composer.reset()
        if s.pos[2] < TOP - 0.02:
            self.off_table = True
        self.last = {"t": round(self.steps * dt, 2), "rot_deg": round(math.degrees(rot), 1), "ext_mm": round(ext * 1000, 1), "to_mark_mm": round((ext - EXTEND) * 1000, 1),
                     "cap_N": round(cap, 1), "roll_deg": round(math.degrees(2 * math.atan2(s.quat_xyzw[1], s.quat_xyzw[3])), 1), "gap_mm": round(gap * 1000, 1),
                     "push_short_mm": None if self.short_mm is None else round(self.short_mm, 2), "passed": self.passed}
        step_file = os.environ.get("TASK_STEP_FILE")
        if step_file:  # the simulated time, for the Dream-1 runner's session watch
            with open(step_file, "w") as f:
                f.write(f"{self.steps * dt:.2f}")
        every = max(1, round(0.25 / dt))
        if self.steps % every == 0:
            print("[pole] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


CAM = Cam()
m.on_step(CAM.step)
m.on_reset(CAM.reset)


def score(objects) -> float:
    c = CAM.last
    if not c or CAM.off_table or c["ext_mm"] < 20.0:
        return 0.0
    at_mark = abs(c["to_mark_mm"]) <= 10.0
    locked = abs(c["rot_deg"]) <= math.degrees(LOCKED)
    if at_mark and locked and CAM.passed:
        return 1.0
    if at_mark and CAM.short_mm is not None and not CAM.passed:
        return 0.5
    if at_mark and not locked:
        return 0.5
    if locked and abs(c["to_mark_mm"]) <= 30.0:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- gates: code torques, no arms --------------------------------------------------------------

def _gate(env):
    """g_roll: one hand twists the inner tube to unlock, rising to 0.8 N m over 2 s, the pole lying free. g_pull: the inner tube pulled out with 50 N, its cam as found (locked)."""
    mode = os.environ.get("TASK_DEMO", "")
    if mode not in ("g_roll", "g_pull"):
        return
    import torch

    asset = env.unwrapped.scene["pole"]
    t = CAM.steps * float(env.unwrapped.step_dt)
    i = int(asset.find_bodies("inner")[0][0])
    s = m.state(env, "pole")
    on = 0.5 < t < 2.5
    ramp = float(np.clip((t - 0.5) / 2.0, 0.0, 1.0)) if on else 0.0  # a hand's twist builds over 2 s, it is not a blow
    tq = s.rotation() @ np.array([0.0, 0.8 * ramp if mode == "g_roll" else 0.0, 0.0])
    fr = int(asset.find_bodies("frame")[0][0])
    fw = s.rotation() @ np.array([0.0, -PUSH_N if (on and mode == "g_pull") else 0.0, 0.0])
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.tensor([[fw.tolist(), (-fw).tolist()]], device=asset.device),
                                                                 torques=torch.tensor([[tq.tolist(), [0.0, 0.0, 0.0]]], device=asset.device), body_ids=[i, fr], is_global=True)
    m._PUSHED.add(("pole", i))
    m._PUSHED.add(("pole", fr))


m.on_step(_gate)


# -- the scripted demo ----------------------------------------------------------------------------

def demo_script(ctx):
    if ctx.mode.startswith("g_"):
        yield from ctx.left.wait(3.0)
        print("[pole] END " + json.dumps({"mode": ctx.mode, **CAM.last}), flush=True)
        return
    pole = ctx.pos("pole")  # a scripted reference: reads the pole's pose, which every camera sees
    z = pole[2]
    y_out = pole[1] + 0.05  # the left hand on the outer tube, near its middle
    y_in = pole[1] + Y_MOUTH - 0.035  # the right hand on the inner tube, just out of the mouth
    zg = z + 0.0133 - 0.002
    arms = [("left", y_out, -90.0), ("right", y_in, 90.0)] if ctx.mode != "one_arm" else [("right", y_in, 90.0)]
    yield from ctx.together(*[getattr(ctx, a).move((pole[0], y, zg + 0.08), grip=d.down(yaw), finger=d.FINGER_OPEN, t=2.0, label="over the tube") for a, y, yaw in arms])
    yield from ctx.together(*[getattr(ctx, a).move((pole[0], y, zg), t=1.0, label="down round the tube") for a, y, _ in arms])
    yield from ctx.together(*[getattr(ctx, a).close(2 * R_OUT if a == "left" else 2 * R_IN, force=45.0, label="grip") for a, _, _ in arms])
    up = np.array([0.0, 0.0, 0.012])
    yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() + up, t=0.6, label="lift the pole off the table") for a, _, _ in arms])
    axis = np.array([0.0, 1.0, 0.0])
    reach = 0.0133  # the TCP sits this far back from the tube's axis along the fingers

    def centre():
        """A point on the pole's axis now: the pole rolls a little as it is handled."""
        p = ctx.pos("pole")
        return np.array([p[0], 0.0, p[2]])

    def turn(deg, label):
        yield from ctx.right.arc(centre(), axis, deg, t=1.6, label=label)

    def regrip(deg, label):
        """Let go, turn back open by deg, come onto the tube's axis again, and grip."""
        yield from ctx.right.open(t=0.4, label=label + ": let go")
        yield from ctx.right.arc(centre(), axis, deg, t=0.8, label=label + ": back")
        c = centre()
        at = ctx.right.at()
        onto = np.array([c[0], at[1], c[2]]) - reach * ctx.right.approach_axis()
        yield from ctx.right.move(onto, t=0.5, label=label + ": onto the tube")
        yield from ctx.right.close(2 * R_IN, force=45.0, t=0.4, label=label + ": grip again")

    yield from turn(-45.0, "unlock, first eighth")
    yield from regrip(45.0, "unlock")
    yield from turn(-45.0, "unlock, second eighth")
    if ctx.mode == "one_arm":
        yield from ctx.right.move(ctx.right.at() + np.array([0.0, -EXTEND, 0.0]), t=3.0, label="draw")
        yield from ctx.right.wait(1.0)
        print("[pole] END " + json.dumps({"mode": ctx.mode, **CAM.last}), flush=True)
        return
    draw = EXTEND - 0.040 if ctx.mode == "short" else EXTEND  # short: a failure check, locked 40 mm short of the mark
    yield from ctx.right.move(ctx.right.at() + np.array([0.0, -draw, 0.0]), t=3.0, label="draw to the mark")
    if ctx.mode == "nolock":  # a failure check: set to the mark but let go without locking it
        yield from ctx.together(*[getattr(ctx, a).open(label="let go") for a, _, _ in arms])
        yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear") for a, _, _ in arms])
        yield from ctx.left.wait(3.0)
        print("[pole] END " + json.dumps({"mode": ctx.mode, **CAM.last}), flush=True)
        return
    yield from turn(45.0, "lock, first eighth")
    # the second eighth: the right hand holds the inner tube now, and the left hand turns the outer tube the other way
    yield from ctx.left.arc(centre(), axis, -45.0, t=1.6, label="lock, second eighth (outer tube)")
    yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() - up, t=0.6, label="set it down") for a, _, _ in arms])
    yield from ctx.together(*[getattr(ctx, a).open(label="let go") for a, _, _ in arms])
    yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear") for a, _, _ in arms])
    yield from ctx.left.wait(3.0)
    print("[pole] END " + json.dumps({"mode": ctx.mode, **CAM.last}), flush=True)
