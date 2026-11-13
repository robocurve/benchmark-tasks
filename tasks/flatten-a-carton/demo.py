"""Demo for the carton-flattening task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1200] [--no-record] [--mode sideways]

The left arm stays in its ready pose. The right hand, tilted 40 degrees forward with its fingers closed, comes onto
the carton's near top edge (where the near wall meets the top panel) from outside and above, and presses on it 70
degrees below level, toward the far side, while following the arc that edge travels as the carton shears. The
carton folds over: the near wall comes down onto the bottom panel, the top panel comes down and across, the far wall
folds outward onto the table. A push that steep never slides the carton (its sideways part is 0.36 of its downward
part, under the board's friction on the table, 0.45). The hand keeps pressing until the carton is flat and its
creases set, holds it 0.8 s, and lifts clear.

``--mode sideways`` is the control: the same hand pushes the near wall square to its face, 9.5 cm up, as a level
push would. The carton shears about 11 degrees, then slides across the table and springs back square.

Open loop: every pose comes from the layout constants in the task file. Object states are read only to report
where things ended up. Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
FULL_OPEN = 0.0939                 # m between the pads at grip command 1.0
TIP = 0.013                        # the pads' tips reach this far past the tool point
FINGER_HALF = 0.008                # the closed fingers' leading edge is about this far ahead of the tip line

PUSH_R = 0.095                     # sideways control: where on the near wall the push lands, up from the crease
PUSH_Y = -0.030                    # and this far right of the carton's middle
PRESS = 0.008                      # how far the push is aimed past the wall (about 2.5 N on the soft arm)
END_RACK = 89.0                    # the press's last aim, in degrees of shear (the carton stops at 88)


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


_t = math.radians(40.0)
PUSH_APPROACH = np.array([math.sin(_t), 0.0, -math.cos(_t)])   # tips 40 degrees forward of straight down
DOWN_R = frame(PUSH_APPROACH, (0.0, -1.0, 0.0))   # right hand tilted forward, fingers closing across; its body trails back


def gap(width: float) -> float:
    """Grip command (1 open .. 0 closed) for a gap between the pads in metres."""
    return min(1.0, max(0.0, width / FULL_OPEN))


class Hand:
    """One arm: smooth moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = self.worst_deg = 0.0

    def _solve(self, pos, rot) -> None:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, duration: float, rot=None, grip: float | None = None):
        """Generator: go in a straight line to ``pos`` (world, tool point) and ``rot``; ``grip`` 1 open .. 0 closed."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        r1 = r0 if rot is None else rot
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, r1, s))
            yield

    def follow(self, path, duration: float, rot=None):
        """Generator: follow ``path(s)`` (world tool point for s from 0 to 1) at a smooth pace."""
        from yam_arena import ik

        r = self.rot if rot is None else rot
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self._solve(path(ik.smoothstep(k / n)), r)
            yield

    def joints_for(self, pos, rot, seed=None) -> np.ndarray:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.array(self.q if seed is None else seed, float),
                             base_pos=self.base, iters=400)
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        return q

    def move_joints(self, q_goal, duration: float, grip: float | None = None):
        """Generator: swing to joint angles ``q_goal``, every joint moving together."""
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q, self.grip = q0 + s * (np.asarray(q_goal, float) - q0), g0 + s * (g1 - g0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            self.cmd = np.r_[self.q, self.grip]
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def chain(*gens):
    for g in gens:
        yield from g


def together(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


def push_point(T, rack_deg: float, press: float = PRESS) -> np.ndarray:
    """Right tool point pushing the near wall's outside at PUSH_R up from its crease, at this shear: the fingertips'
    front face on the wall, aimed ``press`` past it, square to it."""
    r = math.radians(rack_deg)
    c, s = math.cos(r), math.sin(r)
    along, normal = np.array([s, 0.0, c]), np.array([c, 0.0, -s])      # up the wall, and out of it toward the far side
    crease = np.array([T.NEAR_X, T.MID_Y + PUSH_Y, T.TOP + T.BOARD / 2])
    face = crease + PUSH_R * along - (T.BOARD / 2) * normal              # the wall's outside face
    tips = face + (press - FINGER_HALF) * normal                         # where the fingers' front face is aimed
    return tips - TIP * PUSH_APPROACH                                    # tool point 13 mm behind the pad tips


CORNER_DIR = np.array([math.cos(math.radians(70.0)), 0.0, -math.sin(math.radians(70.0))])   # 70 deg below level


def corner_point(T, rack_deg: float, press: float) -> np.ndarray:
    """Right tool point pressing on the carton's near top edge at this shear, aimed ``press`` past it along
    CORNER_DIR."""
    r = math.radians(rack_deg)
    edge = np.array([T.NEAR_X + T.TALL * math.sin(r), T.MID_Y + PUSH_Y, T.TOP + T.BOARD / 2 + T.TALL * math.cos(r)])
    tips = edge + np.array([-0.006, 0.0, 0.004]) + press * CORNER_DIR    # on the edge from outside and above
    return tips - TIP * PUSH_APPROACH


def flatten(T, right):
    start = corner_point(T, 0.0, press=0.0) + [-0.05, 0.0, 0.05]
    yield from right.move_joints(right.joints_for(start, DOWN_R), 2.5, grip=0.0)
    yield from right.move(corner_point(T, 0.0, press=-0.01), 0.8)
    yield from chain(right.move(corner_point(T, 0.0, press=0.008), 0.5), wait(0.2))
    yield from right.follow(lambda s: corner_point(T, END_RACK * s, press=0.008 + 0.014 * s), 4.0)


def sideways(T, right):
    high = push_point(T, 0.0, press=-0.080) + [0.0, 0.0, 0.08]          # well in front of the carton and above it
    yield from right.move_joints(right.joints_for(high, DOWN_R), 2.5, grip=0.0)
    yield from right.move(high - [0.0, 0.0, 0.08], 0.8)                 # down in front of the near wall
    yield from right.move(push_point(T, 0.0, press=-0.030), 0.6)
    yield from chain(right.move(push_point(T, 0.0), 0.6), wait(0.2))    # against the near wall, pressing
    yield from right.follow(lambda s: push_point(T, END_RACK * s, press=PRESS + 0.010 * s), 4.5)


def script(T, left, right, mode: str = "flatten"):
    yield from wait(0.3)
    yield from (flatten(T, right) if mode == "flatten" else sideways(T, right))
    yield from wait(0.8)                                                  # hold it flat while the creases set
    end = right.pos.copy()
    yield from right.move(end + [-0.02, 0.0, 0.10], 1.0)                  # lift clear
    yield from right.move(end + [-0.06, 0.0, 0.14], 0.8)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "flatten", every: int = 5) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=320, cam_height=176,
                               headless=not window)
    T = emb.task.module
    obs = emb.reset(Scene(id="carton-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode)
    frames: dict[str, list] = {}
    trace = []

    def snap(o) -> None:
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, T.NAMES)

    def hands() -> dict:
        from cartonkit import finger_points

        return {n: [round(100 * float(v), 2) for v in p] for n, p in zip(("left", "right"), finger_points(emb._env))}

    snap(obs)
    start = T.report(states())
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(left.cmd, right.cmd)))
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % every == 0:
            r = T.report(states())
            trace.append({"step": steps, "score": info.get("score"), "rack_deg": r["rack_deg"],
                          "height_cm": r["height_cm"], "origin_cm": r["origin_cm"], "set": r["creases_set"],
                          "touched": r["gripper_on_it"], "tool_cm": hands(),
                          "want_cm": {h.name: [round(100 * float(v), 2) for v in h.pos] for h in (left, right)}})
        if steps % 40 == 0 and trace:
            t = trace[-1]
            print(f"step {steps} score {t['score']} rack {t['rack_deg']} h {t['height_cm']} origin {t['origin_cm']}",
                  flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": T.report(states()), "trace": trace}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "trace"}), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="flatten", choices=("flatten", "sideways"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every)
