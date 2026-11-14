"""Scripted full-score demo for the gladhand coupling task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--cam-width 1280]
    python demo.py task.py - --dry          # check the plan's IK without starting the simulator

The left arm grips the hose gladhand's handle from behind, turns it back off its dummy coupler, lifts it, carries it
to the red trailer gladhand, lines it up, presses the two seals together with the bodies at 90 degrees, turns it a
quarter turn to the stop in two grips, lets go and moves clear. The right arm stays at its ready pose.

The plan comes from the task's fixed layout (the run uses object_jitter=0). The demo looks at where the gladhand
actually is before each grasp and while it lines up and turns (it is in plain view of the cameras); it never moves
objects or edits the simulation. DEMO_VARIANT selects wrong strategies used to test the scorer (see the README).
Writes one MP4 per camera, setup.png (the first top-camera frame), final.png and result.json to the output folder.
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


class Arm:
    """One arm: Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())  # the ready pose
        self.hold = (np.zeros(3), np.eye(3))  # the tool point's pose in a held gladhand's frame
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.worst_deg = 0.0
        self.min_margin = math.inf
        self.phase = "start"
        self.by_phase: dict[str, list[float]] = {}  # phase -> [worst mm, worst deg, smallest joint margin, last j6]

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=80)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)
        self._record(pe, re, q)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot, float), s))
            yield

    def _record(self, pe: float, re: float, q) -> None:
        from yam_arena import ik

        margin = float(np.min(np.minimum(q - ik.LOWER, ik.UPPER - q)))
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, math.degrees(re))
        self.min_margin = min(self.min_margin, margin)
        rec = self.by_phase.setdefault(self.phase, [0.0, 0.0, math.inf, 0.0])
        rec[:] = [max(rec[0], pe * 1000), max(rec[1], math.degrees(re)), min(rec[2], margin), float(q[5])]

    def goto(self, pos, rot, duration: float, j6: float | None = None, grip: float | None = None):
        """Generator: go to ``pos``/``rot`` by interpolating the joints, for big turns of the wrist where a
        straight Cartesian path would run a joint into its limit. ``j6`` picks the wrist-roll branch to end in."""
        from yam_arena import ik

        seeds = [self.q.copy()]
        if j6 is not None:
            for roll in (j6, j6 + 0.4, j6 - 0.4):
                seed = self.q.copy()
                seed[5] = roll
                seeds.append(seed)
        best, best_cost, fallback = None, math.inf, None
        for seed in seeds:
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(np.asarray(rot, float)), seed, base_pos=self.base, iters=200)
            if fallback is None or pe + 0.05 * re < fallback[1] + 0.05 * fallback[2]:
                fallback = (q, pe, re)
            if pe < 1e-3 and re < 0.02:
                cost = abs(q[5] - j6) if j6 is not None else float(np.abs(q - self.q).sum())
                if cost < best_cost:
                    best, best_cost = (q, pe, re), cost
        target, pe, re = best if best is not None else fallback
        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q, self.grip = q0 + s * (target - q0), g0 + s * (g1 - g0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            self.cmd = np.r_[self.q, self.grip]
            self._record(pe, re, self.q)
            yield

    def go_home(self, duration: float = 2.5):
        """Generator: back to the ready pose, gripper open."""
        yield from self.goto(*self.home, duration, j6=0.0, grip=1.0)

    def calibrate(self, g_pos, g_rot) -> None:
        """Note how the gladhand actually sits in the grip (it can shift a little between the pads): the tool
        point's pose in the gladhand's frame, from where the gladhand is seen to be."""
        g_rot = np.asarray(g_rot, float)
        self.hold = (g_rot.T @ (self.pos - np.asarray(g_pos, float)), g_rot.T @ self.rot)

    def holding(self, g_pos, g_rot):
        """The tool-point pose that puts the held gladhand at (``g_pos``, ``g_rot``), given the last calibrate()."""
        g_rot = np.asarray(g_rot, float)
        return np.asarray(g_pos, float) + g_rot @ self.hold[0], g_rot @ self.hold[1]

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)

    def follow(self, pose_at, duration: float):
        """Generator: track ``pose_at(s)`` -> (pos, rot) for s from 0 to 1."""
        from yam_arena import ik

        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self._solve(*pose_at(ik.smoothstep(k / n)))
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def grip_pose(g_pos, g_rot, side: float, back: float = 0.0):
    """TCP pose holding a gladhand (position ``g_pos``, rotation ``g_rot``) by its handle from behind: the gripper
    points along the gladhand's axis toward the mating coupler and its pads close across the handle's width.
    ``side`` (+1 or -1) picks which pad is on which side; ``back`` pulls the gripper back along its approach."""
    import gladkit

    g_rot = np.asarray(g_rot, float)
    z = g_rot[:, 2]
    x = side * g_rot[:, 1]
    rot = np.column_stack([x, np.cross(z, x), z])
    return np.asarray(g_pos, float) + g_rot @ (np.array(gladkit.GRIP) * gladkit.MM) - back * z, rot


def turn_gap(angle_deg: float, open_gap: float) -> float:
    """The face gap (mm) to hold while the lug is at ``angle_deg`` under the lip: just inside the lip's underside,
    and no wider than ``open_gap``."""
    import gladkit

    lo, hi = angle_deg - gladkit.LUG_HALF_DEG, angle_deg + gladkit.LUG_HALF_DEG
    if hi <= gladkit.PLATE_DEG[0]:
        return open_gap
    ts = np.linspace(max(lo, gladkit.PLATE_DEG[0]), min(hi, gladkit.PLATE_DEG[1]), 9)
    lip = min(gladkit.lip_height(t) for t in ts)
    return float(min(open_gap, lip + gladkit.LUG_Z[0] - 0.15))


def regrasp_check(arm: Arm, ctx, name: str, planned, settle: float = 0.4):
    """Generator: wait, then look at how the gladhand sits in the grip and remember it."""
    yield from wait(settle)
    arm.calibrate(*ctx.look(name, planned))


def turn(arm: Arm, ctx, name: str, c_pos, c_rot, planned_from: float, goal: float, side: float, open_gap: float,
         lead: float = 28.0, ahead: float = 5.0, rate: float = 45.0, timeout: float = 12.0, tol: float = 1.5):
    """Generator: turn a held gladhand about a coupler's axis to ``goal`` (deg), watching where it actually is.

    The wrist is commanded ahead of the gladhand in rotation (at most ``lead`` degrees), which winds up torque about
    the gladhand's axis, while the tool point is kept only ``ahead`` degrees along the circle from where the handle
    is, so the arm does not shove the lug sideways into the plate. With ``ahead`` equal to ``lead`` the command is a
    plain rigid rotation about the axis. It ends holding the gladhand where it is. ``planned_from`` is the angle a
    dry run (no simulator) starts from."""
    from gladkit import mated_pose, measure

    cmd = None
    for _ in range(round(timeout * CONTROL_HZ)):
        plan = planned_from if cmd is None else cmd
        g_pos, g_rot = ctx.look(name, mated_pose(c_pos, c_rot, plan, turn_gap(plan, open_gap)))
        now = measure(c_pos, c_rot, g_pos, g_rot)["angle_deg"]
        if cmd is None:
            cmd = now
        if abs(goal - now) < tol:
            break
        way = 1.0 if goal > now else -1.0
        cmd = float(np.clip(cmd + way * rate / CONTROL_HZ, now - lead, now + lead))
        gap = turn_gap(now, open_gap)
        _, rot = arm.holding(*mated_pose(c_pos, c_rot, cmd, gap))
        pos, _ = arm.holding(*mated_pose(c_pos, c_rot, now + way * min(ahead, abs(cmd - now)), gap))
        arm._solve(pos, rot)
        yield
    g_pos, g_rot = ctx.look(name, mated_pose(c_pos, c_rot, goal, turn_gap(goal, open_gap)))
    yield from arm.move(*arm.holding(g_pos, g_rot), 0.4)


def line_script(task, ctx, colour: str, arm: Arm, variant: str):
    """The line: off its dummy, onto the trailer gladhand, a quarter turn in two grips, let go."""
    import gladkit
    from gladkit import mated_pose, measure

    T = task
    name = T.LINES[colour]
    dummy, d_rot = T.dummy_pos(colour), T.dummy_rot(colour)
    trailer, t_rot = T.trailer_pos(variant_target(colour, variant)), T.TRAILER_ROT
    start = -100.0  # where a turn starts and an unlocking turn ends: the lug 4 mm clear of the plate's mouth
    regrip = -66.0  # the wrist's roll runs out here on the way in; the lug is under the lip, so it can let go
    first, second = -1.0, 1.0  # which way round the gripper holds the handle: before and after the re-grip
    dummy_open = gladkit.SEAL_PROUD + gladkit.DUMMY_BOSS - 0.2  # the seal barely pressed on the dummy's plain face

    arm.phase = "1 grip"
    # 1. Grip the handle. One look at the gladhand first: settling on the dummy shifts it a little.
    g_pos, g_rot = ctx.look(name, mated_pose(dummy, d_rot, 0.0))
    yield from arm.move(*grip_pose(g_pos, g_rot, first, back=0.06), 2.5, grip=1.0)
    yield from arm.move(*grip_pose(g_pos, g_rot, first), 1.5)
    yield from arm.set_grip(0.0, 1.0)
    yield from regrasp_check(arm, ctx, name, (g_pos, g_rot))

    arm.phase = "2 unlock"
    # 2. Turn it back off the dummy coupler: past the detent, down the ramp, out of the plate's mouth.
    yield from turn(arm, ctx, name, dummy, d_rot, 0.0, start, first, dummy_open)
    yield from wait(0.3)
    if variant == "off_only":
        # Lift it clear, set it down on the table beside the dummy and let go.
        aside = np.array([0.03, -0.10, 0.0])
        p, r = grip_pose(*mated_pose(dummy, d_rot, start, 50.0), first)
        yield from arm.move(p, r, 1.5)
        yield from arm.move(p + aside, r, 2.0)
        p, r = grip_pose(*mated_pose(dummy, d_rot, start, -60.0), first)
        yield from arm.move(p + aside, r, 2.5)
        yield from arm.set_grip(1.0, 0.8)
        yield from arm.move(arm.pos + [0, 0, 0.08], arm.rot, 1.5)
        yield from arm.go_home()
        return

    arm.phase = "3 carry"
    # 3. Lift it and swing its handle round, away from the arm (through straight ahead), so the hose trails in front
    #    and never wraps the wrist; then tip it up to face the trailer gladhand from well back, bodies at 90 degrees.
    yield from arm.move(*grip_pose(*mated_pose(dummy, d_rot, start, 50.0), first), 1.5)
    seal_down = np.eye(3)
    low = dummy[2] + 0.05
    mid = np.array([0.27, (dummy[1] + trailer[1]) / 2, low])
    swing = 60.0  # how far round the handle swings before tipping up (deg from straight ahead)
    # Keep the grip point 16 cm in from the arm's own side while the gladhand is seal-down: its reach is short there.
    near = np.array([0.25, 0.16 * T.SIDE[colour] - 0.07 * math.sin(math.radians(swing)), low])
    arm.phase = "3 carry: swing to straight ahead"
    yield from arm.move(*grip_pose(*mated_pose(mid, seal_down, 0.0, 0.0), first), 2.5)
    arm.phase = "3 carry: swing on round"
    yield from arm.move(*grip_pose(*mated_pose(near, seal_down, swing, 0.0), first), 2.5)
    arm.phase = "3 carry: tip up to face"
    yield from arm.move(*grip_pose(*mated_pose(trailer, t_rot, start, 140.0), first), 3.0)
    # The gladhand may have shifted in the grip on the way: look, and line it up from where it really is.
    for gap_mm, seconds in ((140.0, 1.0), (40.0, 2.0), (40.0, 0.8)):
        yield from regrasp_check(arm, ctx, name, mated_pose(trailer, t_rot, start, gap_mm if seconds != 2.0 else 140.0))
        yield from arm.move(*arm.holding(*mated_pose(trailer, t_rot, start, gap_mm)), seconds)

    arm.phase = "4 press and first turn"
    # 4. Press the seals together, then start the turn: the lug slides into the plate's mouth, under the lip.
    press = 2.6 if variant != "seals_apart" else 30.0  # mm between the faces: both seals pressed in ~0.45 mm
    target = mated_pose(trailer, t_rot, start, press)
    hover = mated_pose(trailer, t_rot, start, max(20.0, press))  # still clear of the other gladhand's plate
    planned = mated_pose(trailer, t_rot, start, 40.0)
    for attempt in range(6):
        # Line it up while nothing touches: look at where the gladhand really is (the arm gives a little and the
        # grip can shift) and correct, until the two faces are square and on one axis.
        yield from regrasp_check(arm, ctx, name, planned, settle=0.3)
        yield from arm.move(*arm.holding(*hover), 1.0 if attempt == 0 else 0.5)
        yield from wait(0.3)
        planned = hover
        seen = measure(trailer, t_rot, *ctx.look(name, hover))
        if seen["lateral_mm"] < 0.7 and seen["tilt_deg"] < 1.0:
            break
    # Straight in until the seals meet, then start the turn slowly, keeping the gladhand on the coupler's axis: the
    # lug slides into the plate's mouth, under the lip.
    yield from regrasp_check(arm, ctx, name, hover, settle=0.2)
    yield from arm.move(*arm.holding(*target), 1.5)
    yield from wait(0.3)
    yield from turn(arm, ctx, name, trailer, t_rot, start, regrip, first, press, tol=2.0, timeout=8.0, lead=10.0,
                    ahead=10.0, rate=20.0)
    yield from wait(0.3)

    arm.phase = "5 re-grip"
    # 5. Let go (the lip holds the lug), back off, roll the wrist half a turn, look, and grip again.
    yield from arm.set_grip(1.0, 0.8)
    yield from arm.move(arm.pos - 0.06 * arm.rot[:, 2], arm.rot, 1.2)
    if variant == "seals_apart":  # nothing held it: it has dropped
        yield from arm.go_home()
        return
    yield from wait(0.8)
    g_pos, g_rot = ctx.look(name, mated_pose(trailer, t_rot, regrip, turn_gap(regrip, press)))
    yield from arm.goto(*grip_pose(g_pos, g_rot, second, back=0.06), 2.5, j6=1.2)
    yield from arm.move(*grip_pose(g_pos, g_rot, second), 1.2)
    yield from arm.set_grip(0.0, 1.0)
    yield from regrasp_check(arm, ctx, name, (g_pos, g_rot))

    arm.phase = "6 finish turn"
    # 6. Finish the turn down the ramp, over the detent, to the stop.
    stop = {"short": -25.0}.get(variant, gladkit.LOCK_ANGLE_DEG)
    yield from turn(arm, ctx, name, trailer, t_rot, regrip, stop, second, press)
    yield from wait(0.5)

    arm.phase = "7 let go"
    # 7. Let go, back away along the gladhand's axis, and return to the ready pose.
    if variant == "hold":  # keep hold of the locked gladhand
        return
    yield from arm.set_grip(1.0, 1.0)
    yield from arm.move(arm.pos - 0.08 * arm.rot[:, 2], arm.rot, 1.5)
    yield from arm.go_home()


def variant_target(colour: str, variant: str) -> str:
    """Which trailer gladhand the line goes to: its own colour, or the other one for the ``crossed`` variant."""
    if variant == "crossed":
        return {"red": "blue", "blue": "red"}[colour]
    return colour


def script(task, ctx, variant: str = ""):
    """The demo as a generator: one yield per control step."""
    if variant == "none":
        yield from wait(5.0)
        return
    yield from line_script(task, ctx, task.LINE, ctx.left, variant)
    yield from wait(1.5)


class Ctx:
    def __init__(self, emb=None, task=None):
        from yam_arena import station

        self.emb, self.task = emb, task
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def look(self, name: str, planned):
        """Where a hose's gladhand is now: (position, rotation). A dry run returns ``planned``."""
        if self.emb is None:
            return np.asarray(planned[0], float), np.asarray(planned[1], float)
        state = self.states()[name]
        return state.pos, state.rotation()

    def states(self):
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


def describe(task, states) -> str:
    """The hose gladhand: where it is against its dummy and each trailer gladhand."""
    import gladkit

    out = []
    line = states[task.LINES[task.LINE]]
    d = gladkit.measure(task.dummy_pos(), task.dummy_rot(), line.pos, line.rotation())
    seen = f"{task.LINE}: seal {1000 * line.seal:+.2f} mm | dummy gap {d['gap_mm']:.2f} ang {d['angle_deg']:.1f} " \
           f"lat {d['lateral_mm']:.1f} under {d['lug_under_lip']}"
    for colour in task.COLOURS:
        t = gladkit.measure(task.trailer_pos(colour), task.TRAILER_ROT, line.pos, line.rotation())
        seen += (f" | {colour} trailer gap {t['gap_mm']:.2f} ang {t['angle_deg']:.1f} lat {t['lateral_mm']:.1f} "
                 f"tilt {t['tilt_deg']:.1f} under {t['lug_under_lip']}")
    out.append(seen + f" | hose z {1000 * (line.points[:, 2].min() - task.TOP):.0f}.."
               f"{1000 * (line.points[:, 2].max() - task.TOP):.0f} mm | stretch {1000 * line.stretch:.1f} mm")
    flags = getattr(task, "_STATE", {})
    out.append(f"flags: off table {flags.get('floor')}, hose off fitting {flags.get('detached')}, "
               f"clear for {flags.get('clear_steps')} steps")
    return "\n  ".join(out)


def arm_report(ctx) -> str:
    """Each arm's phase, how far its tool point is from where it was told to be (mm) and its joint torques (N·m)."""
    from yam_arena.constants import TCP_OFFSET

    import gladkit

    base = ctx.emb._env.unwrapped
    origin = gladkit._np(base.scene.env_origins[0])
    out = []
    for arm, key in ((ctx.left, "left_robot"), (ctx.right, "right_robot")):
        robot = base.scene[key]
        pose = gladkit._np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        tcp = pose[:3] + gladkit.rot_from_xyzw(pose[3:7]) @ np.array(TCP_OFFSET) - origin
        names = list(robot.joint_names)
        torque = gladkit._np(robot.data.applied_torque)[0]
        q = gladkit._np(robot.data.joint_pos)[0]
        idx = [names.index(f"joint{i}") for i in range(1, 7)]
        line = ctx.states()[ctx.task.LINES[ctx.task.LINE]]
        rot = gladkit.rot_from_xyzw(pose[3:7])
        in_hand = 1000 * (line.rotation().T @ (tcp - line.pos) - np.array(gladkit.GRIP) * gladkit.MM)
        lean = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[:, 2] @ line.rotation()[:, 2])))))
        reach = float(np.linalg.norm(line.points[0] - line.to_world(np.array(gladkit.PORT) * gladkit.MM)))
        out.append(f"{arm.name} [{arm.phase}] tcp off {np.round(1000 * (arm.pos - tcp), 1)} mm, joint err "
                   f"{np.round(np.degrees(arm.q - q[idx]), 1)} deg, torque {np.round(torque[idx], 1)}; grip point off "
                   f"{np.round(in_hand, 0)} mm, lean {lean:.0f} deg, hose span {reach:.2f} m, fingers "
                   f"{np.round(1000 * q[[names.index('joint7'), names.index('joint8')]], 1)}")
    return "\n  ".join(out)


def dry_run(task_path: str, variant: str) -> dict:
    """Play the plan through IK alone and report how well each arm can follow it."""
    from yam_arena import task_file

    task = task_file.load(task_path).module
    ctx = Ctx(None, task)
    steps = sum(1 for _ in script(task, ctx, variant))
    result = {"steps": steps}
    for arm in (ctx.left, ctx.right):
        result[arm.name] = {"worst_ik_mm": round(arm.worst_mm, 2), "worst_ik_deg": round(arm.worst_deg, 2),
                            "min_joint_margin_rad": round(arm.min_margin, 3)}
        for phase, (mm, deg, margin, j6) in arm.by_phase.items():
            print(f"{arm.name:5s} {phase:24s} worst {mm:7.2f} mm {deg:6.2f} deg  joint margin {margin:5.2f} rad"
                  f"  ends with joint6 {j6:+.2f}")
    print(json.dumps(result))
    return result


def _render_at(width: int, height: int) -> None:
    """yam_arena renders every camera at 640x360 and resizes; for a sharper video, raise the render size itself
    (the embodiment's default camera size) once Isaac Sim has started and the embodiment module loads."""
    import importlib.abc
    import importlib.util

    class Hook(importlib.abc.MetaPathFinder, importlib.abc.Loader):
        NAME = "yam_arena.embodiment"

        def find_spec(self, name, path, target=None):
            if name != self.NAME:
                return None
            sys.meta_path.remove(self)
            spec = importlib.util.find_spec(name)
            self._inner = spec.loader
            spec.loader = self
            return spec

        def create_module(self, spec):
            return self._inner.create_module(spec)

        def exec_module(self, module):
            self._inner.exec_module(module)
            cls = module.BimanualYamEmbodiment
            init = cls.__init__

            def patched(obj, *args, **kwargs):
                kwargs.setdefault("cam_height", height)
                kwargs.setdefault("cam_width", width)
                init(obj, *args, **kwargs)

            cls.__init__ = patched

    sys.meta_path.insert(0, Hook())


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 2.0, variant: str = "", log_every: int = 100) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if cam_width > 640:
        _render_at(cam_width, cam_height)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb, task)
    print("start\n ", describe(task, ctx.states()), flush=True)
    gen = script(task, ctx, variant)
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
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % log_every == 0:
            print(f"step {steps} score {info.get('score')}\n ", describe(task, ctx.states()), flush=True)
            if os.environ.get("DEMO_DEBUG"):
                print(" ", arm_report(ctx), flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    print(f"end at step {steps}\n ", describe(task, ctx.states()), flush=True)
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "variant": variant or "demo",
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
            imageio.imwrite(out / "final.png", frames[top][-1])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--log-every", type=int, default=100)
    ap.add_argument("--cam-width", type=int, default=320)
    a = ap.parse_args()
    v = os.environ.get("DEMO_VARIANT", "")
    if a.dry:
        dry_run(a.task, v)
    else:
        run(a.task, a.out, a.max_steps, record=not a.no_record, variant=v, log_every=a.log_every,
            cam_width=a.cam_width, cam_height=a.cam_width * 9 // 16 if a.cam_width > 320 else 176)
        os._exit(0)
