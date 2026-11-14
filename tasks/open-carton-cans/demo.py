"""Scripted full-score demo for the open-carton task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]
    python demo.py task.py --dry          # offline: IK error of every planned pose (no simulator)

Open loop from the fixed published layout (the task's constants; the run uses object_jitter=0). At run time it reads
only what a policy could get: the robot's own joint positions, and three looks at the carton's pose (a large box in
plain view of the cameras). It never reads the flap angles, contact forces or any other internal state. The joint
positions feed a small integral term that cancels the joint controllers' sag while an arm carries load.

1. The left arm pinches the front rim (wall plus folded-down top flap) left of centre and tips the carton about 20
   degrees back about its far bottom edge. That edge stays on the table, so the untaped bottom flaps stay shut.
2. Look at the carton. The right arm, fingers closed into a flat "palm" pitched along the tipped bottom, slides 5 cm in
   under the near edge, across the seam where the two side flaps meet, and takes the weight.
3. Together they lift the carton, easing the tip to 8 degrees, carry it over the shelf and set its far edge on the
   board with the near edge 6 cm out over the board's front edge, the palm beneath that overhang.
4. Look. The front is lowered onto the board (the palm keeps its angle), the palm drops and slides out, the left hand
   lets go and backs off.
5. Look. The right hand, fingers still closed, pushes the front wall until the carton's front is inside the board's
   edge, and moves clear.
DEMO_VARIANT selects a cheap strategy instead (see VARIANTS).
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
PINCH_Y = 0.045      # pinch point on the front rim, left of centre (carton frame)
PINCH_LEAN = 55      # degrees from vertical, toward the right, in the front wall's plane
PINCH_DEPTH = 0.018  # TCP this far past the rim along the approach
TIP_DEG = 24         # commanded tip; the carton settles a few degrees lower
PAD_IN = 0.05        # paddle finger tips this far in under the front edge
OVERHANG_SET = 0.06  # front overhang when first set on the board (paddle beneath it)
CARRY_DEG = 8        # front-up tip kept while carrying and setting down
PUSH_IN = 0.005      # pushed until the front is this far inside the board's edge

VARIANTS = {
    "": "the full demo",
    "nothing": "the arms stay where they are",
    "rims": "naive: pinch the left and right wall rims, lift and carry to the shelf (no support underneath)",
    "pinch_only": "naive: tip as in the demo, then carry the carton by the front rim alone (no hand under it)",
    "tip_only": "tip the carton and slide the paddle under, then stop (nothing carried)",
    "rims_low": "naive: pinch the left and right wall rims, lift 4 cm and hold it there",
    "rims_high": "naive: pinch the left and right wall rims, lift 9 cm and hold it there",
    "push": "push the carton along the table toward the shelf without lifting it",
    "side": "tip the carton over onto its back wall (bottom facing the robot), then carry it by the front rim to the shelf",
}


def _rot_axis(axis, angle) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def frame(approach, closing) -> np.ndarray:
    """A TCP rotation: approach axis (gripper z) and closing axis (gripper x)."""
    z = np.asarray(approach, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


class Pose:
    """A rigid carton pose: rotation ``rot`` and the position ``pos`` of the centre of its bottom face."""

    def __init__(self, pos, rot):
        self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)

    def world(self, local):
        return self.pos + self.rot @ np.asarray(local, float)

    def tilted(self, theta, pivot_local):
        """This pose turned ``theta`` about the carton's local y axis through ``pivot_local`` (front up for > 0)."""
        r = self.rot @ _rot_axis((0, 1, 0), theta)
        p = self.world(pivot_local)
        return Pose(p - r @ np.asarray(pivot_local, float), r)

    def moved(self, d):
        return Pose(self.pos + np.asarray(d, float), self.rot)


def yaw_pose(xy, z, yaw=0.0):
    return Pose((xy[0], xy[1], z), _rot_axis((0, 0, 1), yaw))


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm, self.worst_deg, self.worst_label = 0.0, 0.0, ""
        self.label = ""
        # load compensation from the arm's own joint readings: the joint PD sags under a payload, so an integral
        # term adds the remaining joint error to the command while the arm carries something
        self.comp = False
        self.offset = np.zeros(6)
        self.measured = self.q.copy()

    def observe(self, q_measured) -> None:
        self.measured = np.asarray(q_measured[:6], float)
        self.measured_grip = float(q_measured[6])
        if self.comp:
            self.offset[1:5] += 0.35 * (self.q - self.measured)[1:5]
            np.clip(self.offset, -0.3, 0.3, out=self.offset)
        else:
            self.offset *= 0.85
        self.cmd = np.r_[self.q + self.offset, self.grip]

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        if pe > 0.001 or re > 0.01:   # tracking fell behind: solve this pose fully from the last command
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=400)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        if pe * 1000 > self.worst_mm:
            self.worst_mm, self.worst_label = pe * 1000, self.label
        self.worst_deg = max(self.worst_deg, math.degrees(re))
        self.cmd = np.r_[q + self.offset, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, label: str = ""):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def follow(self, poses, duration: float, label: str = ""):
        """Generator: track ``poses(s)`` -> (pos, rot) for s from 0 to 1 (smoothstep timing)."""
        from yam_arena import ik

        self.label = label or self.label
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self._solve(*poses(ik.smoothstep(k / n)))
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)


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


def interp(a: Pose, b: Pose, s: float) -> Pose:
    from yam_arena import ik

    return Pose(a.pos + s * (b.pos - a.pos), ik.slerp_matrix(a.rot, b.rot, s))


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    T = task
    L, R = ctx.left, ctx.right
    variant = os.environ.get("DEMO_VARIANT", "")
    H, S = T.HALF, T.SIZE
    WALL_Y = T.HALF - T.WALL / 2                  # mid-plane of the left wall, carton frame
    BACK_EDGE = (H, 0.0, 0.0)                     # the back bottom edge, pivot of every tip
    c0 = yaw_pose(T.CARTON_XY, T.TOP)             # published start pose

    if variant == "nothing":
        yield from wait(5.0)
        return

    # rim grasp on the left wall: approach pitched 45 degrees toward the back in the carton frame, closing across
    # the wall; TCP 14 mm below the rim
    phi = math.radians(45)
    rim_app = np.array([math.sin(phi), 0.0, -math.cos(phi)])
    rim_rot = frame(rim_app, (0.0, -1.0, 0.0))

    def rim_tcp(x):
        return np.array([x, WALL_Y, S]) + 0.014 * rim_app

    front_rim, back_rim = rim_tcp(-0.050), rim_tcp(0.030)

    if variant in ("rims", "rims_low", "rims_high"):
        # naive: pinch the middle of the left and right wall rims, lift, carry, set down on the shelf
        right_rot = frame(rim_app, (0.0, 1.0, 0.0))
        l_loc = np.array([-0.01, WALL_Y, S]) + 0.014 * rim_app
        r_loc = np.array([-0.01, -WALL_Y, S]) + 0.014 * rim_app
        yield from both(L.move(c0.world(l_loc) + [0, 0, 0.08], rim_rot, 2.5, grip=1.0, label="rims pre"),
                        R.move(c0.world(r_loc) + [0, 0, 0.08], right_rot, 2.5, grip=1.0, label="rims pre"))
        yield from both(L.move(c0.world(l_loc), rim_rot, 1.5), R.move(c0.world(r_loc), right_rot, 1.5))
        yield from both(L.set_grip(0.0, 1.0), R.set_grip(0.0, 1.0))
        yield from wait(0.3)
        L.comp = R.comp = True
        hold = ((L, l_loc, rim_rot), (R, r_loc, right_rot))
        if variant in ("rims_low", "rims_high"):
            h = 0.04 if variant == "rims_low" else 0.09
            yield from _rigid(hold, c0, c0.moved((0, 0, h)), 2.0, "lift")
            yield from wait(5.0)
            return
        yield from _carry_naive(T, hold, c0, ctx)
        return

    if variant == "push":
        # push the left wall toward the shelf with the closed left gripper, fingers down, 6 cm above the table
        down = frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
        z = T.TOP + 0.06 + 0.013
        y0 = T.CARTON_XY[1] + H + 0.03
        yield from L.move((T.CARTON_XY[0], y0, z + 0.08), down, 2.5, grip=0.0, label="push pre")
        yield from L.move((T.CARTON_XY[0], y0, z), down, 1.0)
        yield from L.move((T.CARTON_XY[0], T.SHELF_XY[1] + 0.02, z), down, 5.0, label="push")
        yield from wait(2.0)
        return

    # 1. left arm pinches the doubled front rim (wall plus folded-down top flap) left of centre, approaching in the
    #    wall's plane and leaning 55 degrees to the right, and tips the carton about its back bottom edge; the back
    #    edge stays on the table, so the untaped bottom flaps stay shut
    pinch_app = np.array([0.0, -math.sin(math.radians(PINCH_LEAN)), -math.cos(math.radians(PINCH_LEAN))])
    pinch_rot = frame(pinch_app, (-1.0, 0.0, 0.0))
    pinch = np.array([T.RIM_X, PINCH_Y, S]) + PINCH_DEPTH * pinch_app
    yield from L.move(c0.world(pinch - 0.08 * pinch_app), c0.rot @ pinch_rot, 2.5, grip=1.0, label="L to front rim")
    yield from L.move(c0.world(pinch), c0.rot @ pinch_rot, 1.2)
    yield from L.set_grip(0.0, 1.0)
    yield from wait(0.3)
    L.comp = True
    tilt = math.radians(90 if variant == "side" else TIP_DEG)
    yield from L.follow(lambda s: (c0.tilted(s * tilt, BACK_EDGE).world(pinch),
                                   c0.tilted(s * tilt, BACK_EDGE).rot @ pinch_rot), 4.0 if variant == "side" else 2.5,
                        "tip")
    yield from wait(1.0)
    seen = ctx.states()["carton"]                        # one look: where the tipped carton really is
    c1 = Pose(seen.pos, seen.rotation())
    if variant == "side":
        # carry it lying on its back wall by the front rim (now on top), set it on the board the same way, let go
        up = c1.moved((0, 0, 0.15))
        mid = c1.world((0.0, 0.0, H))
        over = up.moved((T.SHELF_XY[0] - mid[0], T.SHELF_XY[1] - mid[1], 0.0))
        hold = ((L, pinch, pinch_rot),)
        yield from _rigid(hold, c1, up, 3.0, "side lift")
        yield from _rigid(hold, up, over, 3.0, "side carry")
        yield from _rigid(hold, over, over.moved((0, 0, -(0.15 - T.SHELF_TOP) + 0.002)), 2.0, "side set")
        L.comp = False
        yield from L.set_grip(1.0, 0.8)
        yield from L.move(L.pos + [0, 0, 0.06], L.rot, 1.0, label="L clear")
        yield from wait(2.0)
        return

    # 2. right arm: fingers closed into a flat paddle, pitched down along the tipped bottom (closing axis normal to
    #    it), slides 5 cm in under the front edge across the seam where the two side flaps meet, takes the weight
    pad_rot = frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))      # carton frame
    pad_in = np.array([-H + PAD_IN - 0.013, 0.0, -0.011])  # finger tips PAD_IN from the front edge
    pad_out = pad_in - [0.08, 0.0, 0.0]
    if variant == "pinch_only":
        # naive: carry it by the front rim alone, with nothing under the bottom
        yield from _carry(T, ((L, pinch, pinch_rot),), c1, ctx)
        L.comp = False
        yield from L.set_grip(1.0, 0.8)
        yield from L.move(L.pos - 0.06 * (L.rot @ np.array([0, 0, 1.0])) + [0, 0, 0.03], L.rot, 1.0, label="L clear")
        return
    yield from R.move(c1.world(pad_out) + [0, 0, 0.04], c1.rot @ pad_rot, 3.0, grip=0.0, label="R paddle pre")
    yield from R.move(c1.world(pad_out), c1.rot @ pad_rot, 1.0)
    yield from R.move(c1.world(pad_in), c1.rot @ pad_rot, 2.0, label="R paddle in")
    pad_in = pad_in + [0.0, 0.0, 0.004]               # take the weight
    R.comp = True
    yield from R.move(c1.world(pad_in), c1.rot @ pad_rot, 0.8)
    yield from wait(0.5)
    if variant == "tip_only":
        yield from wait(3.0)
        return

    # 3. carry, levelling it: lift, across to the shelf, set it down level on the board with its front overhanging the
    #    board's front edge by OVERHANG_SET, so the paddle under the front is clear of the board
    hold = ((L, pinch, pinch_rot), (R, pad_in, pad_rot))
    yield from _carry(T, hold, c1, ctx)
    seen = ctx.states()["carton"]                        # look: where it was set down
    c3 = Pose(seen.pos, seen.rotation())
    # lower the front onto the board about the back edge; the paddle keeps its angle (only its tips stay under)
    tilt_now = math.atan2(-c3.rot[2, 0], c3.rot[2, 2])
    c4 = c3.tilted(-tilt_now, BACK_EDGE)
    r_world = c3.rot @ pad_rot

    def lower(s):
        c = c3.tilted(-s * tilt_now, BACK_EDGE)
        L._solve(c.world(pinch), c.rot @ pinch_rot)
        R._solve(c.world(pad_in), r_world)

    for k in range(1, 41):
        L.label = R.label = "lower front"
        from yam_arena import ik as _ik
        lower(_ik.smoothstep(k / 40))
        yield
    yield from wait(0.5)

    # 4. paddle down a few millimetres and out; left hand lets go and backs off
    R.comp = False
    L.comp = False
    yield from R.move(c4.world(pad_in - [0.0, 0.0, 0.010]), r_world, 0.8, label="paddle out")
    yield from R.move(c4.world(pad_in - [0.10, 0.0, 0.010]), r_world, 1.5)
    yield from L.set_grip(1.0, 0.8)
    yield from L.move(L.pos - 0.06 * (L.rot @ np.array([0, 0, 1.0])) + [0, 0, 0.03], L.rot, 1.0, label="L clear")

    # 5. right hand, fingers still closed, pushes the carton's front wall until the front is inside the board's edge
    seen = ctx.states()["carton"]
    c5 = Pose(seen.pos, seen.rotation())
    board_front = T.SHELF_XY[0] - T.SHELF_BOARD[0] / 2
    push_rot = frame((math.cos(math.radians(15)), 0.0, -math.sin(math.radians(15))), (0.0, 0.0, 1.0))
    z_push = T.TOP + T.SHELF_TOP + 0.05
    front_x = c5.world((-H, 0.0, 0.0))[0]
    tcp = lambda x_tip: np.array([x_tip - 0.013, c5.pos[1], z_push])
    yield from R.move(tcp(front_x - 0.04), push_rot, 1.5, label="to push")
    yield from R.move(tcp(front_x - 0.005), push_rot, 0.8)
    yield from R.move(tcp(board_front + PUSH_IN), push_rot, 2.5, label="push")
    yield from R.move(tcp(board_front - 0.06) + [0.0, 0.0, 0.03], push_rot, 1.2, label="R clear")


def _shelf_goal(T, overhang):
    """Level carton pose on the shelf board: front edge ``overhang`` in front of the board's front edge, centred
    2 cm in from the board's middle (toward the carton's start)."""
    x_front = T.SHELF_XY[0] - T.SHELF_BOARD[0] / 2 - overhang
    y = T.SHELF_XY[1] - math.copysign(0.02, T.SHELF_XY[1])
    return yaw_pose((x_front + T.HALF, y), T.TOP + T.SHELF_TOP)


def _carry(T, hold, c1, ctx):
    """3. Lift (easing the tip to CARRY_DEG), carry over the shelf, set its back edge on the board."""
    arms = hold
    goal = _shelf_goal(T, OVERHANG_SET).tilted(math.radians(CARRY_DEG), (T.HALF, 0.0, 0.0))
    up = goal.moved((0, 0, 0.03))
    raised = Pose(np.array([c1.pos[0], c1.pos[1], up.pos[2]]), up.rot)
    over = Pose(np.array([raised.pos[0], up.pos[1], up.pos[2]]), up.rot)
    yield from _rigid(arms, c1, raised, 3.0, "lift")
    yield from _rigid(arms, raised, over, 3.0, "carry")
    yield from _rigid(arms, over, up, 1.5, "over shelf")
    yield from _rigid(arms, up, goal.moved((0, 0, -0.002)), 2.0, "set down")
    ctx.last_pose = goal
    yield from wait(0.5)


def _carry_naive(T, hold, c0, ctx):
    goal = _shelf_goal(T, 0.0)
    up = goal.moved((0, 0, 0.035))
    raised = Pose(np.array([c0.pos[0], c0.pos[1], up.pos[2]]), up.rot)
    yield from _rigid(hold, c0, raised, 3.0, "lift")
    yield from _rigid(hold, raised, up, 3.5, "carry")
    yield from _rigid(hold, up, goal.moved((0, 0, 0.002)), 2.0, "set down")
    yield from wait(0.5)
    (L, _, _), (R, _, _) = hold
    L.comp = R.comp = False
    yield from both(L.set_grip(1.0, 0.8), R.set_grip(1.0, 0.8))
    yield from both(L.move(L.pos + [0, 0, 0.08], L.rot, 1.5), R.move(R.pos + [0, 0, 0.08], R.rot, 1.5))
    ctx.last_pose = goal


def _rigid(arms, a: Pose, b: Pose, duration, label):
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    for k in range(1, n + 1):
        c = interp(a, b, ik.smoothstep(k / n))
        for arm, loc, rot in arms:
            arm.label = label
            arm._solve(c.world(loc), c.rot @ rot)
        yield


class Ctx:
    def __init__(self, emb=None):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.last_pose = None
        self.carry_tilt = 0.0
        self.planned = None

    def states(self):
        if self.emb is None:   # dry run: the carton where the plan put it
            if self.last_pose is not None:
                p = self.last_pose
                return {"carton": type("Seen", (), {"pos": p.pos, "rotation": lambda _s: p.rot})()}
            return {"carton": self.planned}
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def dry(task_path: str) -> None:
    """Offline: run the plan without a simulator and report the worst IK error per arm."""
    import importlib.util

    sys.path.insert(0, str(Path(task_path).resolve().parent))
    spec = importlib.util.spec_from_file_location("task", task_path)
    task = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(task)
    ctx = Ctx()

    class Seen:
        def __init__(self, p):
            self.pos, self._r = p.pos, p.rot

        def rotation(self):
            return self._r

    tilt = math.radians(90 if os.environ.get("DEMO_VARIANT") == "side" else TIP_DEG - 4)
    ctx.planned = Seen(yaw_pose(task.CARTON_XY, task.TOP).tilted(tilt, (task.HALF, 0, 0)))
    n = sum(1 for _ in script(task, ctx))
    for arm in (ctx.left, ctx.right):
        print(f"{arm.name}: {n} steps, worst IK {arm.worst_mm:.1f} mm ({arm.worst_label}), {arm.worst_deg:.1f} deg")


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 2.0) -> dict:
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

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        from inspect_robots_yam.packing import STATE_KEY

        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()   # [left 6 + grip, right 6 + grip]
        ctx.left.observe(jp[:7])
        ctx.right.observe(jp[7:14])
        steps += 1
        info = res.info
        snap(res.observation)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"variant": os.environ.get("DEMO_VARIANT", ""), "task_score": score, "success": success, "steps": steps,
              "spilled": task._STATE["spilled"],
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out", nargs="?", default="demo_out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()
    if a.dry:
        dry(a.task)
    else:
        run(a.task, a.out, a.max_steps, record=not a.no_record)
