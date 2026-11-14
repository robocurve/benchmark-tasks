"""Scripted full-score demo for the U-lock task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The demo plans from the fixed start layout (the task's constants; the run uses object_jitter=0), from looks at the
two lock parts' poses (objects in plain view of the cameras) and from its own arm commands. It never reads the latch
state, contact forces or any other internal state of the simulation.

The left arm picks the shackle up by one leg, turns it so its legs point away from the robot and pushes it level, at
hub height, through the back wheel (the inner leg between two spokes, the outer leg past the tyre) and round the
stand's leg. The right arm picks the crossbar up round its middle, carries it up past the stand's leg to behind the
wheel, lowers it in line with the leg ends and pushes it onto both of them at once. Then both arms let go and back
away.

DEMO_VARIANT picks a variant for the negative checks: "nothing" (both arms stay still), "air" (the lock is closed in
the air in front of the robot, round nothing), "no_crossbar" (the shackle is threaded, the crossbar never brought),
"short" (the crossbar is pushed onto the leg ends but stopped 10 mm short of home), "held" (locked correctly, but
neither hand lets go).
Writes one MP4 per camera, a side-by-side MP4 of all three, setup.png (the first top-camera frame) and result.json.
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
VARIANT = os.environ.get("DEMO_VARIANT", "")
CB_F = 1.0
SH_F = -1.0


def rot(approach, finger) -> np.ndarray:
    """TCP rotation whose approach axis (+z) is ``approach`` and whose fingers close along ``finger``."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(finger, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def rz(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


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
        self.worst_mm = 0.0

    def _solve(self, pos, r):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(r), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), r
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, r, duration: float, grip: float | None = None):
        """Generator: go to ``pos``/``r`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, r, s))
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


class Holder:
    """An object held in one arm's grip: the TCP pose that puts the object at a wanted pose, from the object's pose
    in the grip (measured by a look at the object, or the planned grasp before the first look)."""

    def __init__(self, arm: Arm, grip_local, rot_local):
        self.arm = arm
        self.p_in_tcp = -np.asarray(rot_local).T @ np.asarray(grip_local, float)  # object origin in the TCP frame
        self.r_in_tcp = np.asarray(rot_local).T                                      # object rotation in the TCP frame

    def look(self, obj_pos, obj_rot):
        """Re-measure the object's pose in the grip from a look at it (and the TCP's commanded pose)."""
        self.p_in_tcp = self.arm.rot.T @ (np.asarray(obj_pos) - self.arm.pos)
        self.r_in_tcp = self.arm.rot.T @ np.asarray(obj_rot)

    def tcp_for(self, obj_pos, obj_rot):
        r = np.asarray(obj_rot) @ self.r_in_tcp.T
        return np.asarray(obj_pos) - r @ self.p_in_tcp, r

    def move(self, obj_pos, obj_rot, duration):
        p, r = self.tcp_for(obj_pos, obj_rot)
        yield from self.arm.move(p, r, duration)

    def servo(self, ctx, name, obj_pos, obj_rot, n=3, duration=0.6):
        """Put the object at a pose by looking at it and correcting, ``n`` times (the arm sags under the load)."""
        for _ in range(n):
            st = ctx.states()[name]
            p_tcp = self.arm.rot.T @ (np.asarray(st.pos) - self.arm.pos)
            if np.linalg.norm(p_tcp - self.p_in_tcp) > 0.05:   # not where the grip should hold it: do not chase it
                print(f"servo: {name} is {1000 * np.linalg.norm(p_tcp - self.p_in_tcp):.0f} mm off the grip", flush=True)
                return
            self.look(st.pos, st.rotation())
            yield from self.move(obj_pos, obj_rot, duration)
            yield from wait(0.3)


def script(T, ctx):
    """The demo as a generator: one yield per control step."""
    L, R = ctx.left, ctx.right
    if VARIANT == "nothing":
        yield from wait(5.0)
        return

    # Grasps, in each part's own frame. The shackle is held by its inner leg, 5 cm from the bend (level with its centre of mass), fingers across the
    # leg, the gripper leaning back toward the robot and to the left. The crossbar is held round its middle, at its
    # centre of mass, fingers flat on its front and back faces, the gripper leaning out to the right.
    a, side = math.radians(70), math.radians(-25)
    sh_grip = np.array([0.05, T.S, 0.0])
    sh_rot = rot((math.cos(a) * math.cos(side), math.cos(a) * math.sin(side), -math.sin(a)), (0, SH_F, 0))
    b = math.radians(40)
    cb_grip = np.array([0.0, -0.022, 0.0])
    cb_rot = rot((0, math.sin(b), -math.cos(b)), (CB_F, 0, 0))

    def tcp(pos0, yaw, grip, r_local):
        return np.asarray(pos0) + rz(yaw) @ grip, rz(yaw) @ r_local

    hub_z = T.TOP + T.H
    y_lock = T.HY - 0.070 - T.S            # shackle origin y: inner leg 70 mm from the hub, between two spokes
    x_fin = T.XB + 0.06 - 0.198            # leg ends 6 cm behind the wheel plane
    x_pre = T.XS - T.STAND_R - 0.015 - 0.198   # leg ends just in front of the stand's leg

    # 1. Both arms pick their part up. The shackle's leg lies on the table: the tilted fingertips stay above it.
    sp, sr = tcp(T.SHACKLE0, T.SHACKLE0_YAW, sh_grip, sh_rot)
    sp = sp + [0, 0, 0.0039]
    cp, cr = tcp(T.CROSSBAR0, T.CROSSBAR0_YAW, cb_grip, cb_rot)
    up = np.array([0, 0, 0.08])
    yield from both(L.move(sp + up, sr, 2.5, grip=1.0), R.move(cp + up, cr, 2.5, grip=1.0))
    yield from both(L.move(sp, sr, 1.5), R.move(cp + [0, 0, 0.002], cr, 1.5))
    yield from both(L.set_grip(0.0, 1.0), R.set_grip(0.0, 1.0))
    yield from wait(0.3)
    yield from both(L.move(sp + [0, 0, 0.10], sr, 1.5), R.move(cp + [0, 0, 0.10], cr, 1.5))

    hs = Holder(L, sh_grip, sh_rot)
    hc = Holder(R, cb_grip, cb_rot)
    yield from wait(0.3)
    st = ctx.states()
    hs.look(st["shackle"].pos, st["shackle"].rotation())
    hc.look(st["crossbar"].pos, st["crossbar"].rotation())
    # Lock poses are planned in the park frame of the published layout (x across the bike, y along it).
    P = T.park_to_world
    level = T.PARK_ROT
    c30, s30 = math.cos(math.radians(-30)), math.sin(math.radians(-30))
    rolled = level @ np.array([[c30, 0, s30], [0, 1, 0], [-s30, 0, c30]])   # turned 30 deg about its long axis

    if VARIANT == "air":
        # Close the lock in front of the robot, round nothing.
        air = np.array([0.06, -0.10, T.TOP + 0.24])
        yield from both(hs.move(air, np.eye(3), 3.0), hc.move((0.25, -0.30, T.TOP + 0.20), rolled, 2.0))
        yield from hc.move(air + [T.SEAT_X + 0.03, 0, 0.0], np.eye(3), 2.5)
    else:
        # 2. Shackle level at hub height, legs pointing away from the robot, leg ends just in front of the stand's
        #    leg, then pushed home: through the wheel and round the stand's leg. Meanwhile the crossbar, turned a
        #    little about its long axis, goes up, past the stand's leg on its right, over to behind the wheel.
        def shackle_in():
            yield from hs.move(P((x_pre, y_lock, hub_z + 0.03)), level, 4.0)
            yield from hs.move(P((x_pre, y_lock, hub_z)), level, 1.0)
            yield from hs.servo(ctx, "shackle", P((x_pre, y_lock, hub_z)), level)
            mid = T.XB - 0.0285 - 0.012 - 0.198       # leg ends 12 mm in front of the tyre
            yield from hs.move(P((mid, y_lock, hub_z)), level, 2.0)
            yield from hs.servo(ctx, "shackle", P((mid, y_lock, hub_z)), level, n=3)
            yield from hs.move(P((x_fin, y_lock, hub_z)), level, 3.0)
            yield from hs.servo(ctx, "shackle", P((x_fin, y_lock, hub_z)), level, n=2)

        def crossbar_over():
            yield from hc.move(P((-0.13, -0.29, T.TOP + 0.22)), rolled, 2.0)
            yield from hc.move(P((T.XB + 0.09, -0.29, T.TOP + 0.25)), rolled, 2.5)
            yield from hc.move(P((T.XB + 0.09, y_lock, T.TOP + 0.25)), rolled, 2.0)
            yield from hc.servo(ctx, "crossbar", P((T.XB + 0.09, y_lock, T.TOP + 0.25)), rolled, n=2)

        yield from both(shackle_in(), crossbar_over())
        if VARIANT == "no_crossbar":
            yield from both(L.set_grip(1.0, 0.8), R.move(R.pos + [0.0, -0.08, 0.05], R.rot, 2.0))
            yield from L.move(L.pos + [-0.08, 0.0, 0.06], L.rot, 1.5)
            return
        # 3. Crossbar down behind the leg ends and level.
        yield from hc.move(P((T.XB + 0.09, y_lock, hub_z + 0.02)), level, 2.5)
        yield from hc.servo(ctx, "crossbar", P((T.XB + 0.09, y_lock, hub_z + 0.02)), level, n=2)

    # 4. Look at where the shackle actually is; bring the crossbar in line with its leg ends, 3 cm beyond them
    #    (correcting from looks at both parts), then push it onto both at once, a few millimetres past the socket
    #    bottoms (the arm gives a little).
    st = ctx.states()
    sh_p, sh_r = st["shackle"].pos, st["shackle"].rotation()
    seat = sh_p + sh_r @ np.array([T.SEAT_X, 0.0, 0.0])
    yield from hc.move(seat + sh_r @ [0.03, 0, 0], sh_r, 2.0)
    yield from hc.servo(ctx, "crossbar", seat + sh_r @ [0.03, 0, 0], sh_r, n=3)
    st = ctx.states()
    sh_p, sh_r = st["shackle"].pos, st["shackle"].rotation()
    seat = sh_p + sh_r @ np.array([T.SEAT_X, 0.0, 0.0])
    yield from hc.servo(ctx, "crossbar", seat + sh_r @ [0.03, 0, 0], sh_r, n=1)
    yield from hc.move(seat + sh_r @ [0.010 if VARIANT == "short" else -0.006, 0, 0], sh_r, 2.5)
    yield from wait(0.5)
    if VARIANT == "held":                  # locked, but both hands stay on the lock
        yield from wait(10.0)
        return

    # 5. Let go, back away.
    yield from both(L.set_grip(1.0, 0.8), R.set_grip(1.0, 0.8))
    yield from both(L.move(L.pos + [-0.08, 0.0, 0.03], L.rot, 1.5), R.move(R.pos - 0.06 * R.rot[:, 2], R.rot, 1.5))
    yield from both(L.move(L.pos + [-0.02, 0.0, 0.08], L.rot, 1.5), R.move(R.pos - 0.04 * R.rot[:, 2], R.rot, 1.5))


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _arm_report(emb, ctx) -> str:
    """Debug: each arm's TCP from its measured joints vs the commanded one (mm), and its finger opening."""
    if not os.environ.get("DEMO_DEBUG"):
        return ""
    from yam_arena import ik

    out = ""
    for side, arm in (("left", ctx.left), ("right", ctx.right)):
        robot = emb._env.unwrapped.scene[f"{side}_robot"]
        q = robot.data.joint_pos[0].detach().cpu().numpy()
        p, _ = ik.tcp_pose(q[:6], arm.base)
        out += f" {side}_err_mm {1000 * np.linalg.norm(p - arm.pos):.0f} {side}_fing_mm {1000 * abs(q[6]):.1f}"
    return out


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
    task = emb.task.module
    obs = emb.reset(Scene(id="demo", instruction=task.INSTRUCTION))
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
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % 20 == 0:
            st = ctx.states()
            sh, cb = st["shackle"], st["crossbar"]
            loop = task.lock_loop(sh)
            shp = task.world_to_park(sh.pos)
            shyaw = math.degrees(math.atan2(sh.rotation()[1, 0], sh.rotation()[0, 0]) - task.PARK_YAW)
            print(f"step {steps} shackle_park_mm {np.round(1000 * shp, 1)} yaw_deg {shyaw:.1f} "
                  f"tilt_deg {math.degrees(math.acos(min(1.0, sh.rotation()[2, 2]))):.1f} score {info.get('score')} latched {task._S['latched']} rim {task.round_rim(loop)} "
                  f"stand {task.round_stand(loop)} released {task._S['released']} shackle_mm "
                  f"{np.round(1000 * sh.pos, 1)} crossbar_mm {np.round(1000 * cb.pos, 1)} wheel_deg "
                  f"{math.degrees(st['bike_and_stand'].joints.get('back_wheel', 0.0)):.1f}{_arm_report(emb, ctx)}",
                  flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "variant": VARIANT,
              "latched": task._S["latched"],
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record and frames:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        cams = sorted(frames)
        n = min(len(frames[c]) for c in cams)
        _write_mp4([np.concatenate([frames[c][i] for c in cams], axis=1) for i in range(n)], out / "demo.mp4")
        top = next((c for c in cams if "top" in c), cams[0])
        imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--cam-width", type=int, default=320)
    ap.add_argument("--cam-height", type=int, default=176)
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=a.cam_width, cam_height=a.cam_height)
