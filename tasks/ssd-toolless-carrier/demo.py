"""Scripted full-score demo for the SSD-into-carrier task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1800] [--no-record]

The left arm pinches the carrier's pull tab and draws the sprung rail about 6 mm open, and holds it. The right arm
picks the SSD up across its width, carries it over the carrier and lowers it to just above the ribs, 3 mm toward the
open rail (clear of the fixed pins); it slides it across onto the fixed pins, sets it down and lets go. The left arm
then lets the rail back, so its pins spring into the drive's other two holes, and lets go.

The demo is open loop: every target comes from the published starting layout (task.py constants), and it reads no
object poses while it runs. The arms' lag and the way the drive settles in the fingers were measured once and are set
as calibration constants below (the simulation is deterministic with the fixed layout).
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20


def _rot_axis(axis, angle) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


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

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)


def reachable_yaw(arm, pos, yaw):
    """The grip yaw, or the same parallel grip turned 180 degrees, whichever the arm reaches more exactly. A wrist at
    its joint limit otherwise leaves the fingers several degrees off and they twist light parts as they close."""
    from yam_arena import ik
    from yam_arena.demo import grip_rotation

    best = None
    for y in (yaw, yaw + math.pi, yaw - math.pi):
        _, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(grip_rotation(y)), arm.q, base_pos=arm.base, iters=200)
        if best is None or pe + 0.05 * re < best[0]:
            best = (pe + 0.05 * re, y)
    return best[1]


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



# -- calibration ------------------------------------------------------------------------------------------------------
# The demo runs open loop from the published starting layout (task.py constants); it reads no object poses while it
# runs. The simulation is deterministic (fixed layout, no jitter), so where the compliant arms lag their commands or
# the drive settles in the fingers off the commanded spot, the correction was measured once and is set here.
RAIL_PULL = 0.0110       # commanded pull on the tab: the rail's spring makes the arm lag; this reaches its 7 mm stop
CARRY_DXYZ = (0.0008, 0.0008, 0.0)   # TCP correction over the seat: where the drive sits in the fingers, measured
HOVER_DZ = 0.0008        # the drive held this far above the ribs (it sags 0.4 mm below the command)
SLIDE_DY = -0.0038       # the slide onto the fixed pins (across the carrier): 3 mm, commanded past the rail
PRESS_DZ = -0.0012       # then set down on the ribs
TAB_TILT = -30.0         # degrees: the left hand's lean (about the world x axis)
RAIL_REST = 0.0005       # the left hand stops 0.5 mm short of the rail's closed position, so it never pushes it
PLACE_YAW = -2.35        # degrees: the drive turns this much in the fingers as they close; the carry turns it back


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    L, R = ctx.left, ctx.right
    T = task
    tips = 0.013   # PAD_TIP_PAST_TCP: the TCP sits this far above the pad tips
    up = np.array([0, 0, 0.05])
    c, s = math.cos(T.CARRIER_YAW), math.sin(T.CARRIER_YAW)
    turn = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])        # the carrier's frame in the world (turned about z)
    car = np.array([*T.CARRIER_XY, T.TOP])        # the carrier's origin, from the published layout

    def on_car(local):
        return car + turn @ np.asarray(local, float)

    across = turn @ np.array([0.0, 1.0, 0.0])     # across the carrier, toward the sprung rail
    ssd = np.array([*T.SSD_XY, T.TOP])            # the drive's origin on the table, turned like the carrier
    tab = on_car((sum(T.TAB_X) / 2000, sum(T.TAB_Y) / 2000, 0.017 + tips))   # pad tips 17 mm up
    pinch = np.array([0.0, 0.0, 0.002 + tips])    # across the drive's width at its centre, pad tips 2 mm up
    SIDE = 0.003                                  # the drive comes down this far toward the open rail
    over = on_car(T.SEAT + np.array([0.0, SIDE, 0.0])) + pinch + np.array(CARRY_DXYZ)   # TCP, drive set down
    # Fingers close across the carrier (and across the drive): the grip turned like the carrier.
    # The left hand is tilted 30 degrees about the fingers' closing line, its wrist leaning away to the left, so that
    # the two wrists stay clear of each other with the hands only 8 cm apart.
    rot_l = _rot_axis((1, 0, 0), math.radians(TAB_TILT)) @ grip_rotation(reachable_yaw(L, tab, T.CARRIER_YAW))
    rot_r = grip_rotation(reachable_yaw(R, ssd + pinch, T.SSD_YAW))

    def phase(name):
        print(f"PHASE {name} at step {ctx.steps}", flush=True)

    # 1. Both arms at once: the left over the rail's pull tab, the right over the drive.
    phase("approach")
    yield from both(L.move(tab + up, rot_l, 2.0, grip=0.4), R.move(ssd + pinch + up, rot_r, 2.0, grip=1.0))
    yield from both(L.move(tab, rot_l, 1.2), R.move(ssd + pinch, rot_r, 1.2))
    yield from wait(0.5)                          # let the arms settle before closing
    yield from both(L.set_grip(0.0, 0.8), R.set_grip(0.0, 0.8))
    yield from wait(0.3)
    # 2. Left arm: draw the rail out to its stop by the tab and hold it. Right arm: lift the drive 6 cm.
    phase("open rail, lift drive")
    yield from both(L.move(L.pos + RAIL_PULL * across, rot_l, 1.5), R.move(R.pos + np.array([0, 0, 0.06]), rot_r, 1.5))
    yield from wait(0.3)
    # 3. Right arm: carry the drive over its place, 3 mm toward the open rail, and lower it to just above the ribs.
    phase("carry")
    rot_r = _rot_axis((0, 0, 1), math.radians(PLACE_YAW)) @ rot_r
    yield from R.move(over + np.array([0, 0, 0.06]), rot_r, 2.5)
    yield from R.move(over + np.array([0, 0, 0.010]), rot_r, 1.5)
    yield from R.move(over + np.array([0, 0, HOVER_DZ]), rot_r, 1.5)
    yield from wait(1.0)
    # 4. Right arm: slide the drive across onto the fixed pins, just clear of the ribs, then set it down on them.
    phase("slide onto fixed pins")
    yield from R.move(R.pos + SLIDE_DY * across, rot_r, 1.5)
    yield from wait(0.5)
    phase("set down")
    yield from R.move(R.pos + np.array([0, 0, PRESS_DZ]), rot_r, 1.0)
    yield from wait(0.5)
    # 5. Right arm: let go of the drive, which now sits on the ribs on the fixed pins, and lift away to the right.
    phase("let go of the drive")
    yield from R.set_grip(1.0, 0.8)
    yield from R.move(R.pos + np.array([0, -0.03, 0.05]), rot_r, 1.2)
    # 6. Left arm: let the rail back slowly, so its pins spring into the drive's other two holes; open and lift away
    #    to the left.
    phase("release rail")
    yield from L.move(tab + RAIL_REST * across, rot_l, 1.5)
    yield from wait(0.5)
    yield from L.set_grip(0.5, 0.6)
    yield from L.move(L.pos + np.array([0, 0.06, 0.04]), rot_l, 1.2)
    yield from wait(0.5)
    phase("done")


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.steps = 0
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 1800, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 4.0, on_step=None) -> dict:
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
        steps += 1
        info = res.info
        snap(res.observation)
        ctx.steps = steps
        if on_step is not None:
            on_step(emb, task, ctx, steps, info)      # development only (e.g. printing poses); never steers the demo
        elif steps % 100 == 0:
            print(f"step {steps} score {info.get('score')}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps,
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
    ap.add_argument("--max-steps", type=int, default=1800)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
