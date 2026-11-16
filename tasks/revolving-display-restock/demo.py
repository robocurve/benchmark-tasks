"""Scripted full-score demo for the revolving-display restocking task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record]

The arms turn the display half a turn by its corner uprights (three strokes, pinching an upright and carrying it round
the axis), so the empty pockets face them; then each arm lowers the boxed charger on its side into the empty pocket on
its side. Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
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

    def aim(self, pos, rot):
        """Track a pose this step (for closed-loop moves)."""
        self._solve(pos, rot)


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


# The demo runs open loop: it reads no simulator state. The arms are compliant and lag the turntable's drag, so how
# far each stroke actually turns the display (and where its base sits after the strokes) was measured once with
# DEMO_CALIBRATE=1 (which runs the same moves, reads the display's pose and turntable angle from the simulator and
# prints these numbers) and is fixed here. The layout is fixed (object_jitter=0) and the simulation deterministic.
CAL = {
    # (x, y, base yaw, turntable angle) of the display after each of the three strokes
    "after_1": (0.35, 0.0, 0.0, 0.789),
    "after_2": (0.35, 0.0, 0.0, 2.35867),
    "turned": (0.35, 0.0, 0.0, 3.16232),
    # (dx, dy) command corrections holding each box over its pocket, then at three heights on the way down
    "place_charger_left": [(0.00011, 0.01137), (0.00011, 0.01137), (0.00011, 0.01137), (0.00022, 0.0117)],
    "place_charger_right": [(-3e-05, 0.01135), (0.00052, 0.01195), (0.00052, 0.01195), (0.00052, 0.01195)],
}
CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
_MEASURED: dict = {}


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    L, R = ctx.left, ctx.right
    T = task
    top = T.TOP
    tips = 0.013                       # PAD_TIP_PAST_TCP: the TCP sits this far above the pad tips
    up = np.array([0, 0, 0.05])
    r_post = math.sqrt(2) * T.POST_XY  # the uprights' distance from the axis
    z_post = top + T.BASE_H + T.POST_TOP - 0.018 + tips    # pad tips 18 mm below an upright's top
    z_safe = top + 0.170                                   # pad tips 1.7 cm above the stocked boxes' tops

    def display_pose(key):
        """(x, y, base yaw, turntable angle): the published start, or as measured (calibration) / calibrated."""
        if key is None:
            return (*T.DISPLAY_XY, 0.0, 0.0)
        if CALIBRATE:
            s = ctx.states()["display"]
            r = s.rotation()
            _MEASURED[key] = [round(float(s.pos[0]), 5), round(float(s.pos[1]), 5),
                              round(math.atan2(r[1, 0], r[0, 0]), 5), round(float(s.sliders["tray"]), 5)]
            print("CAL so far = " + json.dumps(_MEASURED), flush=True)
            return tuple(_MEASURED[key])
        return tuple(CAL[key])

    def grip_yaw(arm, pos, yaw):
        y = reachable_yaw(arm, pos, yaw)
        return yaw + round((y - yaw) / math.pi) * math.pi

    def arc_offset(arm, pts, ths):
        """The finger direction (relative to the circle's tangent) that the arm reaches best all along an arc."""
        from yam_arena import ik
        from yam_arena.constants import READY

        def worst(off):
            return max(ik.solve(p, ik.matrix_to_quat_xyzw(grip_rotation(th + off)), np.array(READY), base_pos=arm.base,
                                iters=200)[1] for p, th in zip(pts, ths))

        return min((0.0, math.pi / 2, math.pi, -math.pi / 2), key=worst)

    def stroke(arm, pose, corner, to_deg, overshoot_deg=0.0, release=True):
        """Pinch the upright at ``corner`` (degrees round the turntable) and carry it round the axis to ``to_deg``
        (degrees, world), fingers closing along the circle's tangent so they push the upright as it goes. Without
        ``release``, keep holding it there (holding the display still)."""
        cx, cy, byaw, phi = pose
        th0 = byaw + phi + math.radians(corner)
        th0 = math.atan2(math.sin(th0), math.cos(th0))
        th1 = th0 + _wrap(math.radians(to_deg + overshoot_deg) - th0)

        def at(th):
            return np.array([cx + r_post * math.cos(th), cy + r_post * math.sin(th), z_post])

        off = arc_offset(arm, [at(th0 + (th1 - th0) * k / 6) for k in range(7)], [th0 + (th1 - th0) * k / 6 for k in range(7)])
        y0 = th0 + off
        above = np.array([*at(th0)[:2], z_safe])
        yield from arm.move(np.array([*arm.pos[:2], z_safe]), arm.rot, 0.6)
        yield from arm.move(above, grip_rotation(y0), 1.8, grip=0.5)
        yield from arm.move(at(th0), grip_rotation(y0), 1.0)
        yield from arm.set_grip(0.0, 0.6)
        yield from wait(0.2)
        n = int(abs(th1 - th0) / math.radians(3))
        for k in range(1, n + 1):
            th = th0 + (th1 - th0) * k / n
            yield from arm.move(at(th), grip_rotation(th + off), 0.12)
        yield from wait(0.3)
        if release:
            yield from let_go(arm)

    def let_go(arm):
        yield from arm.set_grip(0.5, 0.5)
        yield from arm.move(np.array([*arm.pos[:2], z_safe]), arm.rot, 0.8)

    def hold(arm, pose, world_deg):
        """Pinch the upright nearest ``world_deg`` (degrees round the axis, world) and hold the display still."""
        cx, cy, byaw, phi = pose
        corner = min((45, 135, 225, 315), key=lambda c: abs(_wrap(byaw + phi + math.radians(c - world_deg))))
        yield from stroke(arm, pose, corner, math.degrees(byaw + phi) + corner, release=False)

    def park(arm, side):
        yield from arm.move(np.array([0.16, 0.20 * side, top + 0.17]), grip_rotation(0.0), 1.2)

    # 1. Turn the display half a turn (counterclockwise from above), in three strokes on its corner uprights: the left
    #    arm carries the front-left upright round to the front, the right arm carries it on round to the right side,
    #    then carries the next upright from the front to the front-right. The empty side now faces the arms.
    yield from stroke(L, display_pose(None), 135, 180)
    yield from park(L, 1)
    yield from stroke(R, display_pose("after_1"), 135, 270)
    yield from stroke(R, display_pose("after_2"), 45, 225, release=False)   # and the right arm holds it there
    yield from wait(0.7)

    # 2. Each arm takes the box on its side by its top, lifts it clear of the uprights and the pocket walls, lowers it
    #    straight into the empty pocket on its side, square to the pocket, and lets go 2 mm above the deck, while the
    #    other arm holds the display still by its front upright on the other side.
    pose = display_pose("turned")
    cx, cy, byaw, phi = pose
    rot = np.array([[math.cos(byaw + phi), -math.sin(byaw + phi), 0], [math.sin(byaw + phi), math.cos(byaw + phi), 0],
                    [0, 0, 1]])
    pockets = [np.array([cx, cy, 0.0]) + rot @ np.array([px, py, 0.0]) for px, py in T.EMPTY_POCKETS]
    z_pick = top + T.BOX_H - 0.022 + tips                       # pads on the box's top 22 mm
    z_lift = top + 0.020 + T.BOX_H - 0.022 + tips               # the box's bottom 2 cm off the counter
    z_carry = top + 0.100 + T.BOX_H - 0.022 + tips              # its bottom 1 cm above the pocket walls and uprights
    z_seat = top + T.BASE_H + T.DECK_TOP + 0.002 + T.BOX_H - 0.022 + tips

    def place_box(arm, side, box_xy, name):
        pk = max(pockets, key=lambda p: side * p[1])
        pick = np.array([box_xy[0], box_xy[1], z_pick])
        rot_pick = grip_rotation(grip_yaw(arm, pick, 0.0))
        yield from arm.move(np.array([*arm.pos[:2], z_safe]), arm.rot, 0.5)
        yield from arm.move(pick + up, rot_pick, 2.0, grip=1.0)
        yield from arm.move(pick, rot_pick, 1.0)
        yield from arm.set_grip(0.0, 0.8)
        yield from wait(0.3)
        yield from arm.move(np.array([pick[0], pick[1], z_lift]), rot_pick, 0.8)
        # out to the pocket's distance beside the display (clear of it), up over the walls, then across to the pocket
        # (the box passes outside the front upright and over the pocket row's end wall)
        place = np.array([pk[0], pk[1], z_carry])
        rot_place = grip_rotation(grip_yaw(arm, place, byaw + phi - math.pi))
        yield from arm.move(np.array([pk[0], pick[1], z_lift]), rot_place, 1.5)
        yield from arm.move(np.array([pk[0], pick[1], z_carry]), rot_place, 0.8)
        yield from arm.move(place, rot_place, 2.0)
        yield from wait(0.5)
        # The arm sags and lags a little under the box, by an amount that changes as it reaches down: where it
        # actually holds the box was measured over the pocket and at three heights on the way down (calibration), and
        # the command corrected by that much at each.
        key = f"place_{name}"
        levels = [z_carry, z_seat + 0.045, z_seat + 0.020, z_seat]
        if CALIBRATE:
            corr, done = np.zeros(2), []
            for z in levels:
                yield from arm.move(np.array([*(pk[:2] + corr), z]), rot_place, 1.0)
                yield from wait(0.4)
                for _ in range(3):
                    err = pk[:2] - np.asarray(ctx.states()[name].pos[:2], float)
                    if np.abs(err).max() < 0.0005:
                        break
                    corr += err
                    yield from arm.move(np.array([*(pk[:2] + corr), z]), rot_place, 0.6)
                    yield from wait(0.4)
                done.append([round(float(corr[0]), 5), round(float(corr[1]), 5)])
            _MEASURED[key] = done
            print("CAL so far = " + json.dumps(_MEASURED), flush=True)
        else:
            for z, c in zip(levels, CAL[key]):
                yield from arm.move(np.array([*(pk[:2] + np.asarray(c)), z]), rot_place, 1.0)
                yield from wait(0.4)
        yield from wait(0.3)
        yield from arm.set_grip(1.0, 0.6)
        yield from arm.move(arm.pos + up, rot_place, 1.0)
        yield from park(arm, side)

    yield from place_box(L, 1, T.BOX_L_XY, "charger_left")
    yield from hold(L, pose, 135)            # the left arm takes over holding the display
    yield from let_go(R)
    yield from place_box(R, -1, T.BOX_R_XY, "charger_right")
    yield from let_go(L)
    yield from park(L, 1)


def _yaw_of(p):
    r = p.rotation()
    return math.atan2(r[1, 0], r[0, 0])


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
        cam_height: int = 176, hold_s: float = 4.0) -> dict:
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
        if steps % 100 == 0 or (os.environ.get("DEMO_DEBUG") and steps % 20 == 0 and steps > int(os.environ.get("DEMO_DEBUG_FROM", 0))):
            msg = f"step {steps} score {info.get('score')}"
            if os.environ.get("DEMO_DEBUG"):
                st = ctx.states()
                d = st["display"]
                msg += (f" | display {np.round(1000 * d.pos[:2], 1)}mm yaw {math.degrees(_yaw_of(d)):.2f}"
                        f" tray {math.degrees(d.sliders['tray']):.2f} tilt {task.tilt_deg(d):.1f}"
                        f" facing {task.facing_deg(d):.1f}")
                for n in task.BOXES:
                    msg += f" | {n} {np.round(1000 * task.in_tray(d, st[n]), 1)}mm tilt {task.tilt_deg(st[n]):.1f}"
            print(msg, flush=True)
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
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
