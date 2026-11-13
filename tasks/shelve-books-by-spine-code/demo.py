"""Demo for the book-shelving task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 2400] [--no-record]

The left arm swings out of the way. The right hand points level and forward and, for each book from the top of the
pile down:

1. reaches in with its fingers one above the other and pinches the book by the 11 mm of spine edge that sticks out
   past the book beneath, lifts it 4 cm (the arm sags about 1 cm under the book, and the book must not drag on the
   one beneath) and draws it out toward the arms;
2. carries it across in front of the shelf and turns its wrist a quarter turn, which stands the book upright, head
   up, with its spine still toward the arms;
3. carries it to its place on the shelf (the place its code has in alphabetical order, from the left end), slides it
   in until 1.8 cm of spine still sticks out, sets it down and lets go;
4. closes its fingers and pushes the spine home, flush with the shelf's front.

Open loop: every pose comes from the layout constants in the task file, and each book's place from the codes the
task file gives for the layout being run. Object states are read only to report where things ended up. Writes one
MP4 per camera, setup.png (the first top-camera frame) and result.json.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
FULL_OPEN = 0.0939                 # m between the pads at grip command 1.0
TIP = 0.013                        # the pads' tips reach this far past the tool point

PINCH_IN = 0.011                   # how far onto the book the finger pads reach
OPEN, GRIP, RELEASE = 0.045, 0.012, 0.050   # finger gap around the 26 mm book / commanded on it (about 35 N) / to let go
CARRY_X = 0.325                    # the tool's x while a book is carried in front of the pile and the shelf
CARRY_Z = 0.125                    # and its height above the table
TURN_Y = 0.060                     # where the book is stood upright: in front of the shelf, clear of the pile
PROTRUDE = 0.018                   # how far the spine still sticks out when the book is let go
PUSH_Z = 0.045                     # height above the shelf's board at which the spine is pushed home
LIFT = 0.040                       # how far a book is lifted off the pile before it is drawn out
SAG = 0.012                        # how far the level arm sinks below its commanded height with a book in hand
CLEAR = 0.013                      # room kept under the book while it is carried over the shelf's board


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def about_x(deg: float) -> np.ndarray:
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


SAG_ROLL = 6.0                                         # degrees the loaded hand rolls off level; the turn allows for it
FLAT = frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))        # pointing forward, fingers one above the other
UPRIGHT = about_x(SAG_ROLL) @ frame((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))   # a quarter turn later: fingers side by side
RIGHT_LEVEL_SEED = (1.24, 2.68, 2.13, 0.56, 1.24, -1.57)   # joint angles near the first level pose, to start IK from
LEFT_AWAY = (1.2, 1.0, 1.0, -0.6, 0.0, 0.0)           # the left arm swung out to its own side


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

    def joints_for(self, pos, rot, seed) -> np.ndarray:
        """Joint angles that put the tool at ``pos``/``rot``, solved from ``seed``."""
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.array(seed, float), base_pos=self.base, iters=400)
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


def poses(T) -> dict:
    """Every pose the demo uses, in world coordinates, from the task file's layout."""
    carry = np.array([CARRY_X, T.PILE_Y, T.TOP + CARRY_Z])
    stand_z = T.TOP + T.SHELF_T + T.BOOK_H / 2                  # tool height with the book standing on the board
    books = []
    for i, name in enumerate(T.BOOKS):                          # the pile from the top down
        layer = T.N - 1 - i
        pinch = np.array([T.pile_spine_x(i) + PINCH_IN - TIP, T.PILE_Y, T.TOP + T.STAND_H + (layer + 0.5) * T.BOOK_T])
        y = T.slot_y(T.WANTED.index(name))
        seat = np.array([T.SHELF_X - PROTRUDE + PINCH_IN - TIP, y, stand_z + SAG - 0.004])   # set down, lightly pressed
        over = stand_z + SAG + CLEAR
        high = max(carry[2], pinch[2] + LIFT)
        books.append(dict(pre=pinch - [0.06, 0.0, 0.0], pinch=pinch, lift=pinch + [0.0, 0.0, LIFT],
                          out=np.array([CARRY_X, T.PILE_Y, high]),
                          turn=np.array([CARRY_X, TURN_Y, T.TOP + CARRY_Z]),
                          front=np.array([CARRY_X, y, over]), above=np.array([seat[0], y, over]), seat=seat,
                          back=np.array([seat[0] - 0.04, y, T.TOP + T.SHELF_T + PUSH_Z]),
                          push=np.array([T.SHELF_X - TIP + 0.001, y, T.TOP + T.SHELF_T + PUSH_Z])))
    return dict(carry=carry, books=books, ready=np.array([CARRY_X - 0.03, TURN_Y, T.TOP + 0.17]))


def shelve(P, right, i: int):
    b = P["books"][i]
    yield from right.move(b["pre"], 1.2, rot=FLAT, grip=gap(OPEN))
    yield from right.move(b["pinch"], 0.8)
    yield from chain(right.set_grip(gap(GRIP), 0.5), wait(0.2))
    yield from right.move(b["lift"], 0.8)
    yield from right.move(b["out"], 1.2)                        # out in front of the pile
    yield from right.move(b["turn"], 1.4)                       # across in front of the shelf
    yield from chain(right.move(b["turn"], 1.6, rot=UPRIGHT), wait(0.2))    # the quarter turn: the book stands up
    yield from right.move(b["front"], 0.8)
    yield from right.move(b["above"], 1.0)
    yield from right.move(b["seat"], 0.5)
    yield from chain(right.set_grip(gap(RELEASE), 0.5), wait(0.3))
    yield from right.move(b["back"], 0.6)
    yield from right.set_grip(0.0, 0.4)
    yield from right.move(b["push"], 0.8)
    yield from right.move(b["back"], 0.5)


def script(T, left, right, mode: str = "shelve"):
    P = poses(T)
    yield from wait(0.3)
    # The first swing ends in front of the shelf, well short of the pile and above it.
    level = right.joints_for(P["ready"], FLAT, RIGHT_LEVEL_SEED)
    yield from together(left.move_joints(LEFT_AWAY, 2.0), right.move_joints(level, 3.0, grip=gap(OPEN)))
    yield from right.move(P["carry"] + [0.0, 0.0, 0.03], 1.2)
    for i in range(T.N):
        yield from shelve(P, right, i)
    yield from right.move(P["ready"], 1.5, rot=FLAT, grip=gap(OPEN))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "shelve", every: int = 20) -> dict:
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
    obs = emb.reset(Scene(id="book-demo", instruction=T.INSTRUCTION))
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
        from bookkit import _np, finger_points

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
            trace.append({"step": steps, "score": info.get("score"), "row": r["row"],
                          "books": {n: {"pos_cm": b["pos_cm"], "lean_deg": b["lean_deg"], "facing_deg": b["facing_deg"],
                                        "head": b["head"],
                                        "shelved": b["shelved"], "held": b["held"]} for n, b in r["books"].items()},
                          "right": h["right"],
                          "want": {"tool_cm": [round(100 * float(v), 2) for v in right.pos],
                                   "gap_mm": round(1000 * right.grip * FULL_OPEN, 1)}})
        if steps % 100 == 0 and trace:
            print(f"step {steps} score {info.get('score')} row {trace[-1]['row']}", flush=True)
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
    ap.add_argument("--trace-every", type=int, default=20, help="steps between entries of the trace in result.json")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, every=a.trace_every)
