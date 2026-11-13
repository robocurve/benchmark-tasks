"""Two-handed demo for the straw task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 2400] [--no-record] [--let-go]

1. The left hand pinches the lifter's red handle. It leans 40 degrees out to its own side, which keeps its wrist
   clear of the right arm: the handle is 9 cm from where the right hand works.
2. For each glass: the left hand raises the handle 3 cm and holds it. The right hand comes straight down on the
   lifted straw, 2.9 cm from its left end, where the lifter's groove is cut away for two fingers, pinches it and
   lifts it out. The left hand lowers the handle and the next straw rolls onto the lifter.
3. The right hand carries the straw 4.5 cm out in front of the dispenser and there, over open table, turns its
   wrist so the straw slants 45 degrees with its free end down. It carries the straw to the glass with that end just
   above the rim and slides it down along its own length until 8 cm of it is inside. It opens; the straw tips onto
   the rim and comes to rest leaning in the glass. The glasses are filled left column first, so the hand never
   works over a straw that is already standing in a glass.

One control that must not score:
  --let-go   the left hand raises the handle and lets it go before the right hand arrives, once.

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

HANDLE_PINCH = 0.027               # the handle is pinched this far above its foot
OVER_LIFT = 0.002                  # the handle is sent this far past the lifter's upper stop, to hold it there
STRAW_OPEN, STRAW_GRIP = 0.030, 0.004   # finger gap around / on the 8 mm straw
TRAVEL = 0.125                     # tool height above the table for lifting a straw out, level
TURN = 0.190                       # tool height at which the straw is turned to its slant
TURN_FRONT = 0.045                 # the turn is made this far in front of the dispenser's straw line, over open table
SLANT = 45.0                       # degrees from level at which a straw goes into a glass
FULCRUM_IN = 0.0275                # the straw's line crosses the rim's height this far left of the glass's middle
TIP_OVER_RIM = 0.008               # the straw's tip starts this far above the rim
INSIDE = 0.079                     # length of straw inside the glass when it is let go (its middle is at 7.3 cm)


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _lean(deg: float) -> tuple:
    return math.sin(math.radians(deg)), math.cos(math.radians(deg))


# Each arm's wrist reaches only one of the two ways of closing its fingers front to back.
RIGHT_DOWN = frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))                 # right hand pointing straight down
RIGHT_SLANT = frame((0.0, _lean(SLANT)[0], -_lean(SLANT)[1]), (1.0, 0.0, 0.0))   # leaning out to the right: a straw it
                                                                      # holds slants down to the right
LEFT_HOLD = frame((0.0, -_lean(40.0)[0], -_lean(40.0)[1]), (-1.0, 0.0, 0.0))     # left hand leaning out to the left
STRAW_DOWN = np.array([0.0, -_lean(SLANT)[1], -_lean(SLANT)[0]])      # along a slanted straw, toward its low end


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

    def set_grip(self, grip: float, duration: float = 0.5):
        yield from self.move(self.pos, duration, grip=grip)


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


def at(p, z) -> np.ndarray:
    return np.array([p[0], p[1], z])


def poses(T) -> dict:
    """Every pose the demo uses, in world coordinates, from the task file's layout."""
    handle = np.array(T.dispenser_point(sum(T.LIFTER_X) / 2, T.ARM_Y1 - T.HANDLE_W / 2, T.HANDLE_Z0 + HANDLE_PINCH))
    straw = np.array(T.dispenser_point(T.STRAW_X, T.STRAW_LEN / 2 - T.GRIP_FROM_END, T.GROOVE_Z + T.STRAW_R + T.LIFT))
    held = T.STRAW_LEN - T.GRIP_FROM_END                    # from the pinch to the straw's free end
    travel = T.TOP + TRAVEL
    glasses = []
    for gx, gy in T.GLASS_XY:
        fulcrum = np.array([gx, gy + FULCRUM_IN, T.TOP + T.GLASS_H])
        final = fulcrum - STRAW_DOWN * (held - INSIDE)                      # the pinch when the straw is let go
        start = TIP_OVER_RIM / math.sin(math.radians(SLANT))               # the tip starts this far back up the line
        entry = final - STRAW_DOWN * (INSIDE + start)
        tip0 = fulcrum - STRAW_DOWN * start
        glasses.append(dict(entry=entry, final=final, tip0=tip0, out=final + [0.0, 0.03, 0.04]))
    order = sorted(range(len(glasses)), key=lambda i: (-round(T.GLASS_XY[i][1], 4), T.GLASS_XY[i][0]))   # left column first
    return dict(handle=handle, handle_up=handle + [0.0, 0.0, T.LIFT + OVER_LIFT], straw=straw, travel=travel,
                clear=straw + [-TURN_FRONT, 0.0, TRAVEL - (straw[2] - T.TOP)],
                turn=np.array([straw[0] - TURN_FRONT, straw[1], T.TOP + TURN]),
                glasses=glasses, order=order,
                left_park=np.array([0.26, 0.24, T.TOP + 0.15]), right_park=np.array([0.20, -0.20, T.TOP + 0.15]))


def take_handle(P, left):
    yield from left.move(P["handle"] + [0.0, 0.0, 0.05], 2.0, rot=LEFT_HOLD, grip=gap(0.030))
    yield from left.move(P["handle"], 1.0)
    yield from left.set_grip(0.0, 0.5)


def serve(T, P, left, right, i: int, hold: bool = True):
    g, z = P["glasses"][i], P["travel"]
    if hold:
        yield from chain(left.move(P["handle_up"], 0.8), wait(0.2))
    yield from right.move(at(P["straw"], z), 1.4, rot=RIGHT_DOWN, grip=gap(STRAW_OPEN))
    yield from right.move(P["straw"], 0.8)
    yield from chain(right.set_grip(gap(STRAW_GRIP), 0.4), wait(0.2))
    yield from right.move(at(P["straw"], z), 0.8)
    moves = [chain(right.move(P["clear"], 0.7), right.move(P["turn"], 2.0, rot=RIGHT_SLANT), wait(0.2),
                   right.move(g["entry"], 1.6), wait(0.2), right.move(g["final"], 1.4))]
    if hold:
        moves.append(left.move(P["handle"], 0.8))          # lifter down: the next straw rolls onto it
    yield from together(*moves)
    yield from chain(right.set_grip(gap(STRAW_OPEN), 0.4), wait(0.4))
    yield from right.move(g["out"], 0.6)


def script(T, left, right, mode: str = "serve"):
    P = poses(T)
    yield from wait(0.5)
    yield from take_handle(P, left)
    if mode == "let-go":
        yield from chain(left.move(P["handle_up"], 0.8), wait(0.3), left.set_grip(gap(0.030), 0.4),
                         left.move(P["handle_up"] + [0.0, 0.05, 0.05], 0.8), left.move(P["left_park"], 1.2))
        yield from serve(T, P, left, right, P["order"][0], hold=False)
        yield from right.move(P["right_park"], 1.5, rot=RIGHT_DOWN)
        return
    for i in P["order"]:
        yield from serve(T, P, left, right, i)
    yield from together(chain(left.set_grip(gap(0.030), 0.4), left.move(P["handle"] + [0.0, 0.0, 0.05], 0.8),
                              left.move(P["left_park"], 1.2)),
                        right.move(P["right_park"], 1.5, rot=RIGHT_DOWN))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "serve", every: int = 20) -> dict:
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
    obs = emb.reset(Scene(id="straw-demo", instruction=T.INSTRUCTION))
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
        """Where each gripper really is and how far its fingers really are apart (for the result file only)."""
        from strawkit import _np, finger_points

        res = {}
        for name, point in zip(("left", "right"), finger_points(emb._env)):
            robot = emb._env.unwrapped.scene[f"{name}_robot"]
            q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            res[name] = {"tool_cm": [round(100 * float(v), 2) for v in point],
                         "gap_mm": round(1000 * sum(abs(float(q[names.index(j)])) for j in ("joint7", "joint8")), 1)}
        return res

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
            r, h = T.report(states()), hands()
            trace.append({"step": steps, "lifter_up_mm": r["lifter_up_mm"], "score": info.get("score"),
                          "straws": {n: {"in": s["in"], "low_cm": s["low_cm"], "slant_deg": s["slant_deg"], "held": s["held"]}
                                     for n, s in r["straws"].items()},
                          "glass_tilt": {g: v["tilt_deg"] for g, v in r["glasses"].items()},
                          "left": h["left"], "right": h["right"],
                          "want": {x.name: {"tool_cm": [round(100 * float(v), 2) for v in x.pos],
                                            "gap_mm": round(1000 * x.grip * FULL_OPEN, 1)} for x in (left, right)}})
        if steps % 100 == 0 and trace:
            t = trace[-1]
            print(f"step {steps} score {info.get('score')} lifter {t['lifter_up_mm']} in glasses "
                  f"{sum(1 for s in t['straws'].values() if s['in'])}", flush=True)
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
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--let-go", action="store_true", help="control: raise the handle and let it go before taking a straw")
    ap.add_argument("--trace-every", type=int, default=20, help="steps between entries of the trace in result.json")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, every=a.trace_every,
        mode="let-go" if a.let_go else "serve")
