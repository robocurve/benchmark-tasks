"""Scripted full-score demo for the butter-bread task, run in the yam_arena embodiment.

    python demo.py butter_bread.py demo_out [--max-steps 2400] [--no-record]

The left arm holds the bread still the way a hand steadies a slice against its fingertips: its fingers, opened 7 cm
apart along the bread's left edge, stand on the plate just past it, and every knife stroke runs toward them, so the
bread can neither slide away nor turn.
The right arm picks up the knife by its handle (well behind the neck) and turns it to point away from the robot, so the
two hands stay apart. It scrapes the top of the butter block with the blade flat and pressed, then draws the loaded
blade slowly across the slice from its right edge to its left, toward the stop, pressing gently. It goes back for more
butter and spreads again, four loads in all. The left hand then lifts off and goes back to its ready pose, so the
finished slice is in full view; the right arm lays the knife back exactly where it lay and goes back too.

The demo is open loop: it plans every move from the starting layout (the poses the setup describes, read once after the
objects settle) and the knife's offset in the hand from the commanded grasp. It reads no simulator state while it runs
(no live object poses, slider positions or butter amounts); it only prints them for the log.

DEMO_NO_HOLD=1 leaves the left arm at its ready pose: the knife then drags the bread along and little butter goes on.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
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

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
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
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def line(self, pos, duration: float):
        """Generator: a straight line at constant speed (no easing), keeping the rotation."""
        p0 = self.pos.copy()
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self._solve(p0 + (k / n) * (np.asarray(pos) - p0), self.rot)
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)


def grip_for(width: float, squeeze: float) -> float:
    """Normalized grip (1 open .. 0 closed) that commands the pads ``squeeze`` narrower than ``width``."""
    from yam_arena.constants import FINGER_OPEN

    return float(np.clip((width - squeeze) / 2 / -FINGER_OPEN, 0.0, 1.0))


def down_grip(closing_dir) -> np.ndarray:
    """A straight-down grip whose fingers close along the horizontal ``closing_dir`` (TCP x is the closing axis)."""
    from yam_arena.demo import grip_rotation

    x0 = grip_rotation(0.0)[:, 0]
    yaw = math.atan2(closing_dir[1], closing_dir[0]) - math.atan2(x0[1], x0[0])
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2  # either way round closes the same: take the smaller turn
    return grip_rotation(yaw)


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def pose(rot, pos) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = rot, pos
    return m


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


# -- the plan ----------------------------------------------------------------------------------------------------------
FENCE_GAP = 0.010            # the fingers' centre line this far past the bread's left edge (bread y)
FENCE_TIPS = 0.003           # their tips this far above the plate
FENCE_X = 0.010              # the hand's centre this far along the edge, away from the robot
FENCE_SPAN = 0.070           # the fingers open this wide along the edge: two stops, so the slice cannot pivot
GRIP_BACK = 0.040            # the right hand grips the handle this far behind its middle, to keep the wrists apart
HANDLE_SQUEEZE = 0.006       # pads commanded 6 mm narrower than the 18 mm handle
TIPS_UP = 0.004              # pad tips 4 mm above the handle's underside
PRESS = 0.003                # the blade's underside commanded this far into the butter or the crumb
SPREAD_SPEED = 0.02          # m/s across the bread
SCRAPE = 0.040               # m of scraping stroke over the block
SPREAD_Y = 0.031             # the blade's middle runs from -31 to +31 mm across the bread (bread y), toward the stop
HOVER = 0.05
LOADS = 4                    # scrape-and-spread passes: a fixed number, not stopped on the simulator's butter count


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    T = task
    L, R = ctx.left, ctx.right
    s0 = ctx.states()
    bread, knife, dish = s0["bread"], s0["knife"], s0["butter_dish"]
    no_hold = bool(os.environ.get("DEMO_NO_HOLD"))
    home = {arm: (arm.pos.copy(), arm.rot.copy()) for arm in (L, R)}  # the ready poses, to go back to at the end
    print(f"knife settled tilt {math.degrees(T.tilt(knife)):.1f} deg", flush=True)

    # 1. Left arm: closed fingers standing on the plate just past the bread's left edge (a stop for the strokes).
    hold_rot = down_grip(bread.rotation()[:, 0])  # the fingers open along the edge
    hold = bread.to_world((FENCE_X, T.BREAD_SIDE / 2 + FENCE_GAP, 0.0)) + [0, 0, FENCE_TIPS + PAD_TIP_PAST_TCP]
    # 2. Right arm: over the knife's handle, fingers closing across it.
    handle_dir = knife.rotation()[:, 0]
    knife_rot = down_grip(np.cross([0, 0, 1.0], handle_dir))
    grasp = knife.to_world((-GRIP_BACK, 0.0, T.HANDLE_Z0 + TIPS_UP)) + [0, 0, PAD_TIP_PAST_TCP]
    moves = [R.move(grasp + [0, 0, HOVER], knife_rot, 2.5, grip=1.0)]
    if not no_hold:
        moves.append(L.move(hold + [0, 0, HOVER], hold_rot, 2.5, grip=grip_for(FENCE_SPAN, 0.0)))
    yield from both(*moves)
    if not no_hold:
        yield from L.move(hold, hold_rot, 2.0)
    yield from R.move(grasp, knife_rot, 2.0)
    yield from wait(0.3)
    yield from R.set_grip(grip_for(T.HANDLE_SIZE[1], HANDLE_SQUEEZE), 1.2)
    yield from wait(0.3)
    # The knife in the TCP frame, as gripped: from the commanded grasp and the knife's starting pose, not read back.
    X = np.linalg.inv(pose(knife_rot, grasp)) @ pose(knife.rotation(), knife.pos)
    yield from R.move(R.pos + [0, 0, HOVER], R.rot, 1.5)
    yield from wait(0.5)

    def tcp_for(blade_mid_world, yaw):
        """TCP pose that puts the blade flat at ``yaw`` with its middle (underside) at ``blade_mid_world``."""
        rot = rz(yaw)
        mid_local = np.array([T.BLADE_X0 + T.BLADE_LEN / 2, 0.0, 0.0])
        origin = np.asarray(blade_mid_world) - rot @ mid_local
        m = pose(rot, origin) @ np.linalg.inv(X)
        return m[:3, 3], m[:3, :3]

    yaw = math.atan2(bread.rotation()[1, 0], bread.rotation()[0, 0])  # the blade along the bread's x, pointing away

    def load():
        block_top = dish.pos + [0, 0, T.PAT_TOP - PRESS]
        p, r = tcp_for(block_top - [0, SCRAPE / 2, 0], yaw)
        yield from R.move(p + [0, 0, HOVER], r, 2.5)
        yield from R.move(p, r, 1.5)
        yield from wait(0.3)
        p2, _ = tcp_for(block_top + [0, SCRAPE / 2, 0], yaw)
        yield from R.line(p2, SCRAPE / SPREAD_SPEED)
        yield from R.move(R.pos + [0, 0, HOVER], r, 1.2)
        print(f"loaded {T._STATE['volume']:.0f} mm3", flush=True)

    def spread():
        crumb_top = bread.to_world((0.0, 0.0, T.BREAD_H - PRESS))  # where the slice starts: the left hand keeps it there
        by = bread.rotation()[:, 1]
        p, r = tcp_for(crumb_top - by * SPREAD_Y, yaw)
        yield from R.move(p + [0, 0, HOVER], r, 2.5)
        yield from R.move(p, r, 1.5)
        yield from wait(0.3)
        p2, _ = tcp_for(crumb_top + by * SPREAD_Y, yaw)
        yield from R.line(p2, 2 * SPREAD_Y / SPREAD_SPEED)
        yield from R.move(R.pos + [0, 0, HOVER], r, 1.2)
        print(f"buttered {T.covered()} squares, lumps {T.lumps()}, left on the blade {T._STATE['volume']:.0f} mm3, "
              f"thickness {T._STATE['h'].min():.2f}..{T._STATE['h'].max():.2f} mm, max shear "
              f"{T._STATE['max_shear'] * 1000:.2f} mm, max press {T._STATE['max_press'] * 1000:.2f} mm", flush=True)

    for _ in range(LOADS):
        yield from load()
        yield from spread()

    # 3. The left hand lifts off the plate and goes back to its ready pose, leaving the buttered slice in full view
    # (the slice is not being spread any more, so nothing pushes it). Then the knife is laid back on the table where
    # it was, let go, and the right arm goes back to its ready pose too.
    if not no_hold:
        yield from L.move(L.pos + [0, 0, 0.06], L.rot, 1.2)
        yield from L.move(*home[L], 2.5)
    m = pose(knife.rotation(), knife.pos + [0, 0, 0.002]) @ np.linalg.inv(X)  # exactly as it lay, 2 mm up
    p, r = m[:3, 3], m[:3, :3]
    yield from R.move(p + [0, 0, HOVER], r, 2.5)
    yield from R.move(p, r, 2.0)
    yield from wait(0.3)
    yield from R.set_grip(1.0, 0.8)
    yield from R.move(R.pos + [0, 0, 0.08], R.rot, 1.2)
    yield from R.move(*home[R], 2.5)


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


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 320,
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
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    from inspect_robots.types import Action  # noqa: F401
    for _ in range(10):  # let the knife settle onto its blade and handle end before planning
        res = emb.step(ctx.action())
        snap(res.observation)
    gen = script(task, ctx)
    steps, ended, success, info = 10, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        snap(res.observation)
        if os.environ.get("DEMO_DEBUG") and steps % 10 == 0:
            from yam_arena import ik

            arm = emb._env.unwrapped.scene["right_robot"]
            qa = arm.data.joint_pos[0].detach().cpu().numpy().astype(float)[:6]
            pa, _ = ik.tcp_pose(qa, ctx.right.base)
            kn = ctx.states()["knife"]
            print(f"dbg {steps}: tcp cmd {np.round(ctx.right.pos, 4)} act {np.round(pa, 4)} "
                  f"err {np.linalg.norm(pa - ctx.right.pos) * 1000:.1f} mm, grip {ctx.right.grip:.2f}, knife "
                  f"{np.round(kn.pos, 4)} tilt {math.degrees(task.tilt(kn)):.1f}", flush=True)
        if steps % 100 == 0:
            st = ctx.states()
            print(f"step {steps} score {info.get('score')} squares {task.covered()} blade {task._STATE['volume']:.0f} "
                  f"crumb {st['bread'].sliders['crumb'] * 1000:.2f} mm bread {np.round(st['bread'].pos, 3)} "
                  f"knife {np.round(st['knife'].pos, 3)}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "squares": task.covered(),
              "lumps": task.lumps(), "thickness_mm": [round(float(task._STATE["h"].min()), 2),
                                                      round(float(task._STATE["h"].max()), 2)],
              "torn": task._STATE["torn"],
              "h_mm": np.round(task._STATE["h"], 2).tolist(),
              "max_shear_mm": round(task._STATE["max_shear"] * 1000, 2),
              "max_press_mm": round(task._STATE["max_press"] * 1000, 2),
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
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    cam = int(os.environ.get("DEMO_CAM_WIDTH", "320"))
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=cam, cam_height=cam * 11 // 20)
