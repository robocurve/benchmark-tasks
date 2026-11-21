"""Scripted full-score demo for microtome-block-out, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]
    python demo.py task.py --ik          (no simulator: IK error of every target and of each path, step by step)

Open loop: every target comes from the task file's fixed layout and the clock. The only feedback is each arm's own
joint readings (a trim that keeps the measured tool point on the plan while it pushes). No object pose or joint state
of the microtome steers it.

1. The right hand takes the handwheel's knob (pointing at the machine, rolling with the knob) and turns the wheel from
   10 to 12 o'clock, which brings the specimen to its upper end, then pushes the handle in so its lock pin drops into
   the wall's hole. Meanwhile the left hand lifts the knife guard from below and pushes it over, so it falls onto its
   stop above the edge.
2. The right hand moves round to the cassette clamp and pushes its lever forward; the left hand takes the cassette by
   its two ends (fingers beside the clamp jaws, never below the block) and draws it straight out.
3. The right hand lets the lever go and parks; the left hand carries the cassette to the tray and lets it go.

Negative variants (DEMO_VARIANT=...):
  no_lock    the wheel is turned to 12 o'clock but the handle is not pushed in; the rest as usual (0)
  no_guard   the guard is left down; the rest as usual (0.25: locked only)
  reach_in   before locking, the left hand takes hold of the cassette in the clamp and lets go (0 for good)
Writes one MP4 per camera, setup.png and result.json; DEMO_DEBUG=1 prints a trace every 10 steps.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
VARIANTS = ("", "no_lock", "no_guard", "reach_in")
KI, CORR_MAX = 0.2, 0.03        # joint-feedback trim: gain per step and limit (m)


# -- grip rotations: columns are the TCP's closing axis, its y axis and its approach axis (yam_arena convention) -----------
DOWN_Y = DOWN_X = None


def _rotations():
    """yam_arena's straight-down grip (fingers closing along y) and the same turned a quarter turn (closing along x)."""
    global DOWN_Y, DOWN_X
    if DOWN_Y is None:
        from yam_arena.demo import grip_rotation

        DOWN_Y, DOWN_X = grip_rotation(0.0), grip_rotation(math.pi / 2)


FLIP = np.diag([-1.0, -1.0, 1.0])       # the same parallel grip turned half a turn about its approach axis


def best_grip(arm, positions, rot):
    """The grip (rot or rot turned half a turn about its approach) whose worst IK error over ``positions``, solved one
    after another from the arm's last command, is smaller, with the wrist roll kept off its +-120 degree limit."""
    from yam_arena import ik

    positions = [positions] if np.ndim(positions) == 1 else positions
    out = []
    for R in (rot, rot @ FLIP):
        q, worst = arm.q.copy(), 0.0
        for pos in positions:
            q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(R), q, base_pos=arm.base, iters=200)
            worst = max(worst, pe + 0.05 * re + 0.01 * max(0.0, abs(float(q[5])) - 1.9))
        out.append((worst, R))
    return min(out, key=lambda t: t[0])[1]


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each from where the last command left it. With ``servo`` the
    command is trimmed by the arm's own joint readings so the measured tool point follows the plan under load."""

    def __init__(self, name: str, base_pos, read=None):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.pos = np.asarray(self.pos, float)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.read = read
        self.corr = np.zeros(3)
        self.corr_max = CORR_MAX
        self.max_corr = 0.0

    def _solve(self, pos, rot, servo=None):
        from yam_arena import ik

        if servo is not None and self.read is not None:
            meas = self.read()
            if meas is not None:
                self.corr = np.clip(self.corr + KI * np.asarray(servo) * (self.pos - meas[0]), -self.corr_max,
                                    self.corr_max)
                self.max_corr = max(self.max_corr, float(np.abs(self.corr).max()))
        q, pe, _ = ik.solve(np.asarray(pos, float) + self.corr, ik.matrix_to_quat_xyzw(rot), self.q,
                            base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, servo=None):
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, rot, s), servo)
            yield

    def hold(self, duration: float, grip: float | None = None, servo=None):
        yield from self.move(self.pos, self.rot, duration, grip, servo)

    def relax(self, duration: float = 0.5):
        c0 = self.corr.copy()
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.corr = c0 * (1 - k / n)
            self._solve(self.pos, self.rot)
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


def chain(*gens):
    for g in gens:
        yield from g


def shift(arm, d, rot, duration: float, grip: float | None = None, servo=None):
    """Generator: move by ``d`` from wherever the arm's command is when this step starts (not when it was built)."""
    yield from arm.move(arm.pos + np.asarray(d, float), rot, duration, grip, servo)


def stay(arm, rot, duration: float, grip: float | None = None):
    """Generator: stay where the arm is when this step starts, turning to ``rot`` and setting the grip."""
    yield from arm.move(arm.pos, rot, duration, grip)



# -- the plan ------------------------------------------------------------------------------------------------------------
START_LIFT = 0.075        # first, straight up from the ready pose: off the housing's top corner
START_BACK_X = 0.235      # then back over the wheel (plus task.XS: 145 mm in front of the housing's handwheel side)
START_HIGH_Z = 0.335      # at this height (the housing's top is at 265 mm, the wheel's rim at 240 mm)
KNOB_TCP_X = 0.235       # the tool point on the knob's axis, fingertips 14 mm short of the wheel's disc
PUSH_IN = 0.013          # the handle is pushed 13 mm (3 mm past its 10 mm travel)
WIGGLE = math.radians(3.0)
LEVER_TCP_X = 0.380      # the right hand's pads across the clamp lever's 8 mm push bar (plus task.XS)
LEVER_Z = 0.140          # where the closed fingers push the bar, 25 mm above its foot (carriage at its upper end)
LEVER_Y0, LEVER_Y1 = 0.070, 0.095    # the fingertips 5 mm behind the bar, then pushed 20 mm past touching it (the
#                                      hand's widest part is at the wrist, 80 mm back, where nothing is near)
LEVER_VIA_DY = 0.015     # the way in to the bar runs this much further from the housing than the bar itself
LEVER_ROLL = math.pi / 2  # the knob grip turned a quarter turn about its axis: the fingers close up and down, so the
#                           hand is 65 mm deep front to back and stays 20 mm or more clear of the housing's front
GUARD_TCP_Y = 0.178      # the left hand's closed fingers under the guard's free end
GUARD_UNDER = 0.012      # tool point this far below the guard's underside
CASS_TCP_Y = 0.0865      # pads over the cassette's ends, from its back face to its front
DRAW = 0.045
TRAY_DROP = 0.090        # the cassette is let go with the tool point this high over the tray (above the table)


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def cols(closing, approach):
    x, z = np.asarray(closing, float), np.asarray(approach, float)
    return np.column_stack([x, np.cross(z, x), z])


KNOB_ROT0 = cols((0, 1, 0), (1, 0, 0))     # pointing at the machine, fingers closing across y (at 12 o'clock)
GUARD_ROT = cols((0, 0, 1), (0, -1, 0))    # pointing -y, fingers closing up-down: a flat paddle
CASS_ROT = cols((1, 0, 0), (0, -1, 0))     # pointing -y, fingers closing across x on the cassette's ends


def knob_pos(T, a, x=KNOB_TCP_X):
    x = x + T.XS
    return np.array([x, T.WHEEL_Y + T.YS - T.KNOB_R * math.sin(a), T.TOP + T.WHEEL_Z + T.KNOB_R * math.cos(a)])


def plan(T) -> dict:
    """Tool-point targets (world, m) from the task file's fixed layout."""
    p = {}
    top, ys = T.TOP, T.YS             # the layout constants are 4 cm off the world frame by YS
    p["knob_pre"] = knob_pos(T, T.A0, KNOB_TCP_X - 0.06)
    p["knob"] = knob_pos(T, T.A0)
    p["knob_12"] = knob_pos(T, 0.0)
    p["knob_in"] = knob_pos(T, 0.0, KNOB_TCP_X + PUSH_IN)
    p["knob_off"] = knob_pos(T, 0.0, KNOB_TCP_X - 0.07)
    p["r_via1"] = np.array([0.17, T.WHEEL_Y + ys, top + 0.24])
    p["r_via2"] = np.array([0.17, LEVER_Y0 + LEVER_VIA_DY + ys, top + LEVER_Z + 0.02])
    p["lever_pre"] = np.array([LEVER_TCP_X, LEVER_Y0 + ys, top + LEVER_Z])
    p["lever_push"] = np.array([LEVER_TCP_X, LEVER_Y1 + ys, top + LEVER_Z])
    p["park_right"] = np.array([0.17, -0.10, top + 0.24])
    gy, gz = T.GUARD_PIVOT
    gy += ys
    p["guard_pre"] = np.array([T.CLAMP_X, GUARD_TCP_Y + ys, top + gz - 0.002 - GUARD_UNDER - 0.025])   # below it
    p["guard_under"] = np.array([T.CLAMP_X, GUARD_TCP_Y + ys, top + gz - 0.002 - GUARD_UNDER])
    p["guard_lift"] = np.array([T.CLAMP_X, GUARD_TCP_Y + ys, top + gz + 0.020])
    p["guard_over1"] = np.array([T.CLAMP_X, gy + 0.025, top + gz + 0.036])
    p["guard_over2"] = np.array([T.CLAMP_X, gy + 0.003, top + gz + 0.036])
    p["guard_off"] = np.array([T.CLAMP_X, gy + 0.03, top + gz + 0.05])
    cz = top + T.CASS_Z_TOP
    p["cass_pre"] = np.array([T.CLAMP_X, CASS_TCP_Y + ys + 0.06, cz])
    p["cass"] = np.array([T.CLAMP_X, CASS_TCP_Y + ys, cz])
    p["cass_out"] = np.array([T.CLAMP_X, CASS_TCP_Y + ys + DRAW, cz])
    p["cass_up"] = np.array([T.CLAMP_X, CASS_TCP_Y + ys + DRAW + 0.01, cz + 0.035])
    p["tray_over"] = np.array([T.TRAY[0], T.TRAY[1] + ys, top + TRAY_DROP + 0.05])
    p["tray_drop"] = np.array([T.TRAY[0], T.TRAY[1] + ys, top + TRAY_DROP])
    p["park_left"] = np.array([0.25, 0.12, top + 0.20])
    for k, v in p.items():                     # the layout stands XS further out than its numbers (task.XS)
        if not k.startswith("knob"):           # knob_pos() adds it already
            p[k] = v + np.array([T.XS, 0.0, 0.0])
    return p


def arc(arm, T, a0, a1, rot0, duration, x=KNOB_TCP_X, servo=None):
    """Generator: carry the knob round the wheel from angle a0 to a1, the hand rolling with it."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    for k in range(1, n + 1):
        a = a0 + ik.smoothstep(k / n) * (a1 - a0)
        arm._solve(knob_pos(T, a, x), rx(a) @ rot0, servo)
        yield


def knob_grip(arm, T):
    """The knob grip (or the same turned half a turn about its approach) with the smaller worst IK error along the
    whole turn from 10 to 12 o'clock, solved one after another as the demo moves."""
    from yam_arena import ik

    out = []
    for R0 in (KNOB_ROT0, KNOB_ROT0 @ FLIP):
        q, worst = arm.q.copy(), 0.0
        for a in np.linspace(T.A0, 0.0, 13):
            q, pe, re = ik.solve(knob_pos(T, a), ik.matrix_to_quat_xyzw(rx(a) @ R0), q, base_pos=arm.base, iters=200)
            worst = max(worst, pe + 0.05 * re + 0.01 * max(0.0, abs(float(q[5])) - 1.9))
        out.append((worst, R0))
    return min(out, key=lambda t: t[0])[1]


def script(task, ctx):
    _rotations()
    T, P = task, plan(task)
    variant = os.environ.get("DEMO_VARIANT", "")
    L, R = ctx.left, ctx.right
    up = np.array([0.0, 0.0, 1.0])
    knob0 = knob_grip(R, T)
    lever_rot = knob0 @ rz(LEVER_ROLL)      # the knob grip turned a quarter turn about its axis
    guard_rot = best_grip(L, [P[n] for n in ("guard_pre", "guard_under", "guard_lift", "guard_over1", "guard_over2")],
                          GUARD_ROT)
    cass_rot = best_grip(L, [P[n] for n in ("cass_pre", "cass", "cass_out", "cass_up", "tray_over", "tray_drop")],
                         CASS_ROT)
    servo_x = np.array([1.0, 0.0, 0.0])
    servo_yz = np.array([0.0, 1.0, 1.0])
    servo_xyz = np.ones(3)

    # 1. right: wheel to 12 o'clock, handle in; left: guard over
    # The right arm's ready tool point is at x 323 mm, z 261 mm, close above the housing's handwheel side (x 380 mm
    # since the layout moved by task.XS; before, at x 300 mm, the housing held the fingertips at reset). It first lifts
    # straight up, then moves back over the wheel and only then down to the knob, so it never passes low near the
    # housing's top corner (DEMO_OLD_START=1 replays the old direct move, for the record).
    if os.environ.get("DEMO_OLD_START") == "1":
        start = [R.move(P["knob_pre"], rx(T.A0) @ knob0, 2.5, 1.0)]
    else:
        start = [R.move(R.pos + START_LIFT * up, R.rot, 1.0, 1.0),
                 R.move(np.array([START_BACK_X + T.XS, R.pos[1] + 0.05, T.TOP + START_HIGH_Z]), rx(T.A0) @ knob0, 1.4),
                 R.move(P["knob_pre"], rx(T.A0) @ knob0, 1.6)]
    lock = start + [R.move(P["knob"], rx(T.A0) @ knob0, 1.0),
            R.move(P["knob"], rx(T.A0) @ knob0, 0.6, 0.0),
            arc(R, T, T.A0, 0.0, knob0, 3.0, servo=servo_yz),     # on the arm's own reading: run 1 lagged 13 mm
            R.hold(0.6, servo=servo_yz)]                           # settle at 12 o'clock before pushing in
    if variant != "no_lock":
        lock += [R.move(P["knob_in"], knob0, 1.0, servo=servo_x),
                 arc(R, T, 0.0, WIGGLE, knob0, 0.4, KNOB_TCP_X + PUSH_IN, servo_xyz),
                 arc(R, T, WIGGLE, -WIGGLE, knob0, 0.6, KNOB_TCP_X + PUSH_IN, servo_xyz),
                 arc(R, T, -WIGGLE, 0.0, knob0, 0.4, KNOB_TCP_X + PUSH_IN, servo_xyz), R.hold(0.3)]
    lock += [R.relax(0.3), stay(R, knob0, 0.5, 1.0), shift(R, np.array([-0.07, 0.0, 0.0]), knob0, 0.8),
             stay(R, knob0, 0.5, 0.0)]          # fingers closed before moving on: open ones swept the wheel's rim (run 1)
    guard = [wait(0.5)]
    if variant == "reach_in":                # takes hold of the cassette before anything is safe, then lets go
        guard += [L.move(P["cass_pre"], cass_rot, 2.5, 1.0), L.move(P["cass"], cass_rot, 1.2), stay(L, cass_rot, 0.4),
                  L.move(P["cass_pre"], cass_rot, 1.2)]
    if variant != "no_guard":
        guard += [L.move(P["guard_pre"], guard_rot, 2.5, 0.0), L.move(P["guard_under"], guard_rot, 1.0),
                  L.move(P["guard_lift"], guard_rot, 1.2), L.move(P["guard_over1"], guard_rot, 1.0),
                  L.move(P["guard_over2"], guard_rot, 0.8), L.move(P["guard_off"], guard_rot, 1.0)]
    yield from both(chain(*lock), chain(*guard))

    # 2. right: round to the clamp lever and push it forward; left: take the cassette by its ends and draw it out
    yield from both(chain(R.move(P["r_via1"], lever_rot, 1.2), R.move(P["r_via2"], lever_rot, 2.0),
                          R.move(P["lever_pre"], lever_rot, 1.5), R.move(P["lever_push"], lever_rot, 1.2,
                                                                         servo=np.array([0.0, 1.0, 0.0]))),
                    chain(L.move(P["cass_pre"], cass_rot, 2.5, 1.0), wait(1.0)))
    yield from chain(L.move(P["cass"], cass_rot, 1.2), stay(L, cass_rot, 0.6, 0.0),
                     L.move(P["cass_out"], cass_rot, 1.2), L.move(P["cass_up"], cass_rot, 0.8))

    # 3. right lets the lever go and parks; left carries the cassette to the tray
    yield from both(chain(R.relax(0.3), R.move(P["lever_pre"], lever_rot, 0.8), R.move(P["r_via2"], lever_rot, 1.2),
                          R.move(P["park_right"], lever_rot, 2.0)),
                    chain(L.move(P["tray_over"], cass_rot, 2.5), L.move(P["tray_drop"], cass_rot, 1.0),
                          stay(L, cass_rot, 0.6, 1.0), shift(L, 0.06 * up, cass_rot, 0.8),
                          L.move(P["park_left"], cass_rot, 1.5)))


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"], read=lambda: self.joint_tcp("left"))
        self.right = Arm("right", station.BASES["right"], read=lambda: self.joint_tcp("right"))

    def joint_tcp(self, name: str):
        """The arm's tool point and gripper rotation from its own joint positions (forward kinematics)."""
        from partkit import _np
        from yam_arena import ik, station
        from yam_arena.constants import ARM_JOINTS

        art = self.emb._env.unwrapped.scene[f"{name}_robot"]
        ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        pos, rot = ik.tcp_pose(_np(art.data.joint_pos)[0][ids], station.BASES[name])
        return np.asarray(pos, float), np.asarray(rot, float)

    def measured_q(self, name: str):
        """The arm's own joint readings and finger opening (1 open .. 0 closed): for logging only."""
        from partkit import _np
        from yam_arena.constants import ARM_JOINTS, FINGER_OPEN

        art = self.emb._env.unwrapped.scene[f"{name}_robot"]
        jp = _np(art.data.joint_pos)[0]
        ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        f = art.find_joints(["joint7"], preserve_order=True)[0]
        return jp[ids], float(abs(jp[f[0]]) / FINGER_OPEN) if f else 1.0

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


HOUSING_GROUPS = {"housing": range(0, 15), "knife holder": range(15, 19)}   # the microtome body's boxes


def housing_boxes(T) -> dict:
    """World centre and half-size of the microtome body's boxes (axis-aligned: the housing is not turned), by group."""
    from partkit import _box

    o = np.asarray(T.HOUSING.pos, float)
    out = {}
    for g, idx in HOUSING_GROUPS.items():
        bs = [_box(T.HOUSING.boxes[i]) for i in idx if i < len(T.HOUSING.boxes)]
        out[g] = (np.array([o + np.asarray(c, float) for c, _, _, _ in bs]), np.array([np.asarray(sz, float) / 2 for _, sz, _, _ in bs]))
    return out


def signed_box_distance(pts: np.ndarray, centres: np.ndarray, halves: np.ndarray) -> float:
    """Smallest signed distance (m) from any point to any box: negative inside a box."""
    q = np.abs(pts[:, None, :] - centres[None, :, :]) - halves[None, :, :]
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=2)
    inside = np.minimum(q.max(axis=2), 0.0)
    return float((outside + inside).min())


def fixture_clearance(T, ctx, boxes, joints=None) -> dict:
    """Logging only (reads the arms' joint readings, or ``joints`` {"left": (q, grip), ...} when given): for each arm,
    the closest approach of any of its collision meshes (every link, fingers included) to the housing and to the knife
    holder, with the link and the box ("microtome__body:<k>", as in the task file's box order)."""
    import clearance

    res = {}
    for name in ("left", "right"):
        arm = ctx.left if name == "left" else ctx.right
        q, grip = joints[name] if joints else ctx.measured_q(name)
        links = clearance.arm_points(q, grip, arm.base)
        for g, (c, h) in boxes.items():
            best = (np.inf, "", -1)
            for link, p in links.items():
                for k in range(len(c)):
                    d = signed_box_distance(p, c[k:k + 1], h[k:k + 1])
                    if d < best[0]:
                        best = (d, link, list(HOUSING_GROUPS[g])[k])
            res[f"{name}/{g}"] = (round(1000 * best[0], 1), best[1], f"microtome__body:{best[2]}")
    return res


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 3.0) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}
    debug = os.environ.get("DEMO_DEBUG", "1") != "0"
    min_gap = float("inf")
    import clearance

    boxes = housing_boxes(task)
    fix_min: dict = {}            # "arm/group" -> (mm, link, step): closest approach to the instrument's body

    print("PLAN " + json.dumps({k: np.round(v, 4).tolist() for k, v in plan(task).items()}), flush=True)

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    last_gap = {"mm": None}

    def row(steps, info):
        """Logging only: reads simulator state for the trace."""
        from yam_arena.scene import object_states

        st = object_states(emb._env, emb.task.names)
        S = task._STATE
        w, h = st["handwheel"], st["head"]
        return {"step": steps, "score": info.get("score"), "levels": list(task.levels(st)), "released": S["released"],
                "violation": S["violation"], "gap_mm": last_gap["mm"],
                "wheel_deg": round(math.degrees(task.wheel_angle(st)), 1),
                "handle_mm": round(1000 * w.sliders.get("handle", 0.0), 1),
                "carriage_mm": round(1000 * h.sliders.get("carriage", 0.0), 1),
                "clamp_deg": round(math.degrees(h.hinges.get("clamp", 0.0)), 1),
                "guard_deg": round(math.degrees(st["microtome"].hinges.get("guard", 0.0)), 1),
                "cassette": np.round(st["cassette"].pos - [0, 0, task.TOP], 3).tolist(),
                "tray": np.round(st["tray"].pos - [0, 0, task.TOP], 3).tolist(),
                "corr_mm": {"L": np.round(1000 * ctx.left.corr, 1).tolist(),
                            "R": np.round(1000 * ctx.right.corr, 1).tolist()},
                "instrument_mm": {k: list(v) for k, v in fixture_clearance(task, ctx, boxes).items()},
                "tcp": {"L_cmd": np.round(ctx.left.pos - [0, 0, task.TOP], 3).tolist(),
                        "L_meas": np.round(ctx.joint_tcp("left")[0] - [0, 0, task.TOP], 3).tolist(),
                        "R_cmd": np.round(ctx.right.pos - [0, 0, task.TOP], 3).tolist(),
                        "R_meas": np.round(ctx.joint_tcp("right")[0] - [0, 0, task.TOP], 3).tolist(),
                        "grip": [round(float(ctx.left.grip), 2), round(float(ctx.right.grip), 2)]}}

    snap(obs)
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % 5 == 0:
            try:
                gap, _ = clearance.min_clearance(ctx.left.q, ctx.left.grip, ctx.left.base, ctx.right.q,
                                                 ctx.right.grip, ctx.right.base)
                min_gap = min(min_gap, gap)
                last_gap["mm"] = round(1000 * gap, 1)
            except Exception as exc:  # noqa: BLE001 - logging only
                if steps == 5:
                    print(f"clearance unavailable: {exc!r}", flush=True)
        if steps <= 150 or steps % 2 == 0:
            try:
                for k, (mm, link, box) in fixture_clearance(task, ctx, boxes).items():
                    if k not in fix_min or mm < fix_min[k][0]:
                        fix_min[k] = (mm, link, box, steps)
            except Exception as exc:  # noqa: BLE001 - logging only
                if steps == 1:
                    print(f"fixture clearance unavailable: {exc!r}", flush=True)
        if debug and steps % 10 == 0:
            try:
                print("TRACE " + json.dumps(row(steps, info), default=str), flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"trace unavailable: {exc!r}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    S = task._STATE
    result = {"task_score": score, "success": success, "steps": steps, "variant": os.environ.get("DEMO_VARIANT", ""),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)},
              "max_trim_mm": {"left": round(1000 * ctx.left.max_corr, 1), "right": round(1000 * ctx.right.max_corr, 1)},
              "min_arm_link_gap_mm": round(1000 * min_gap, 1) if min_gap < float("inf") else None,
              "min_clearance_to_instrument_mm": {k: {"mm": v[0], "link": v[1], "box": v[2], "step": v[3]}
                                                 for k, v in sorted(fix_min.items())},
              "violation": S["violation"]}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, default=str), flush=True)
    return result


def sweep(task_path: str, every: int = 1, variant: str = "") -> dict:
    """No simulator: step the demo's script on its commanded joints (no trim) and measure every collision mesh of both
    arms against the instrument's body at every step; prints the closest approaches with link, box and step."""
    import importlib

    from yam_arena import station

    if variant:
        os.environ["DEMO_VARIANT"] = variant
    sys.path.insert(0, str(Path(task_path).resolve().parent))
    task = importlib.import_module(Path(task_path).stem)

    class Fake:
        def __init__(self):
            self.left, self.right = Arm("left", station.BASES["left"]), Arm("right", station.BASES["right"])

    ctx = Fake()
    boxes = housing_boxes(task)
    gen = script(task, ctx)
    worst, low, step = {}, [], 0
    while step < 1400:
        try:
            next(gen)
        except StopIteration:
            break
        step += 1
        if step % every:
            continue
        res = fixture_clearance(task, ctx, boxes, joints={"left": (ctx.left.q, ctx.left.grip),
                                                          "right": (ctx.right.q, ctx.right.grip)})
        for k, v in res.items():
            if k not in worst or v[0] < worst[k][0]:
                worst[k] = (*v, step)
            if v[0] < 15.0:
                low.append((step, k, *v))
    print(f"SWEEP {step} steps (commanded joints, variant {variant or 'demo'})")
    for k, v in sorted(worst.items()):
        print(f"  MIN {k:<20} {v[0]:7.1f} mm  {v[1]:<12} {v[2]:<20} step {v[3]}")
    for row in low[:200:5]:
        print(f"  LOW step {row[0]:4d} {row[1]:<20} {row[2]:7.1f} mm {row[3]} {row[4]}")
    return worst


def ik_check(task_path: str) -> None:
    """No simulator: yam_arena IK error of every planned target (both grips), then each arm's paths step by step."""
    import importlib

    from yam_arena import ik, station

    sys.path.insert(0, str(Path(task_path).resolve().parent))
    task = importlib.import_module(Path(task_path).stem)
    _rotations()
    P = plan(task)
    left, right = Arm("left", station.BASES["left"]), Arm("right", station.BASES["right"])
    knob0 = knob_grip(right, task)
    lever_rot = knob0 @ rz(LEVER_ROLL)
    rots = {"knob": knob0, "r_": lever_rot, "lever": lever_rot, "park_right": lever_rot, "guard": GUARD_ROT, "cass": CASS_ROT,
            "tray": CASS_ROT, "park_left": CASS_ROT}
    for name, p in P.items():
        arm = left if name.startswith(("guard", "cass", "tray", "park_left")) else right
        R = next(v for k, v in rots.items() if name.startswith(k))
        if name in ("knob_pre", "knob"):
            R = rx(task.A0) @ R
        errs = []
        for RR in (R, R @ FLIP):
            _, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(RR), arm.q, base_pos=arm.base, iters=300)
            errs.append(f"{1000 * pe:.1f}mm/{re:.3f}")
        print(f"IK {name:12s} {arm.name:5s} {np.round(p - [0, 0, task.TOP], 3).tolist()} grip0 {errs[0]} grip180 {errs[1]}",
              flush=True)
    # the knob turn, solved step by step as the demo does
    q, worst = right.q.copy(), 0.0
    for a in np.linspace(task.A0, 0.0, 41):
        q, pe, _ = ik.solve(knob_pos(task, a), ik.matrix_to_quat_xyzw(rx(a) @ knob0), q, base_pos=right.base, iters=60)
        worst = max(worst, pe)
    print(f"SEQ right knob turn 10->12: worst {1000 * worst:.1f} mm  q={np.round(q, 2).tolist()}", flush=True)
    for seq_name, arm, names, R in (
            ("right lever", right, ["knob_12", "knob_off", "r_via1", "r_via2", "lever_pre", "lever_push", "park_right"],
             lever_rot),
            ("left guard", left, ["guard_pre", "guard_under", "guard_lift", "guard_over1", "guard_over2", "guard_off"],
             best_grip(left, [P[n] for n in ("guard_pre", "guard_under", "guard_lift", "guard_over1", "guard_over2")],
                       GUARD_ROT)),
            ("left cassette", left, ["cass_pre", "cass", "cass_out", "cass_up", "tray_over", "tray_drop", "park_left"],
             best_grip(left, [P[n] for n in ("cass_pre", "cass", "cass_out", "cass_up", "tray_over", "tray_drop")],
                       CASS_ROT))):
        q = arm.q.copy()
        prev, worst = None, 0.0
        for n in names:
            target = P[n]
            start = prev if prev is not None else target
            for k in range(1, 41):
                q, pe, _ = ik.solve(start + (target - start) * k / 40, ik.matrix_to_quat_xyzw(R), q, base_pos=arm.base,
                                    iters=60)
                worst = max(worst, pe)
            prev = target
            print(f"SEQ {seq_name}: {n} worst so far {1000 * worst:.1f} mm  q={np.round(q, 2).tolist()}", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out", nargs="?", default="demo_out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--ik", action="store_true", help="only print the IK error of every planned target")
    ap.add_argument("--sweep", action="store_true", help="only measure both arms' clearance to the instrument along "
                    "the script, on commanded joints (no simulator)")
    a = ap.parse_args()
    if a.sweep:
        sweep(a.task)
    elif a.ik:
        ik_check(a.task)
    else:
        run(a.task, a.out, a.max_steps, record=not a.no_record)
