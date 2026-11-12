"""Two-handed demo for the page-filing task, run in the yam_arena embodiment.

    python pick_demo.py task.py out_dir [--window] [--max-steps 3000] [--no-record] [--sweep]

The pages have to end in the tray in order, page 4 at the bottom. Only the top page of the pile can be taken, so a
page that comes up too early is laid on the shelf and fetched later. For each move:

1. If the page comes from the pile and the pile's top page does not stick out yet, the left hand, closed and pointing
   down, presses on the pile and slides all of it 1.8 cm to the right over the slick shelf.
2. The right hand, level and pointing left, pinches the page by the edge that sticks out past the shelf (its
   fingertips reach 1.1 cm onto the page, less than the pile's 1.8 cm step) and lifts it 3 cm.
3. To lay the page aside, it carries the page along the shelf and sets it down there with the same edge sticking
   out. To file it, it draws the page out, carries it to the tray and lowers it until the page's far edge is just
   above the shelf's edge, then opens fully and backs away: the page tips off the edge and slides down flat into the
   tray.

With ``--sweep`` it plays a shortcut instead, which must not score: the left hand pushes the whole pile over the
shelf's edge.

Open loop: every pose comes from the layout constants in the task file, and the sequence of moves from the task
file's pile order for the lot being run. Object states are read only to report where each page ended up. Writes
one MP4 per camera, setup.png (the first top-camera frame) and result.json.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


LEVEL_LEFTWARD = frame((0.0, 1.0, 0.0), (0.0, 0.0, 1.0))   # right hand: pointing toward +y, fingers closing vertically
RIGHT_LEVEL_SEED = (0.32, 2.03, 0.65, 1.38, -1.25, -1.57)  # joint angles near that pose beside the shelf, to start IK from


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

    def set_grip(self, grip: float, duration: float = 0.8):
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


def moves(T) -> list:
    """The demo's moves, from the pile order: (source, target, page number). The source is "pile" or the index of a
    place on the shelf; the target is "tray" or the index of a place on the shelf."""
    pile = list(T.ORDER)                       # top page first
    spots = [None] * len(T.SPOTS)
    plan = []
    for want in T.WANTED:
        while True:
            if want in spots:
                s = spots.index(want)
                spots[s] = None
                plan.append((s, "tray", want))
                break
            top = pile.pop(0)
            if top == want:
                plan.append(("pile", "tray", want))
                break
            s = spots.index(None)              # a free place; the two lots never need more than two
            spots[s] = top
            plan.append(("pile", s, top))
    return plan


def geometry(T) -> dict:
    """Heights and offsets every move shares (world frame, m)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    pinch_y = T.SHELF_EDGE - T.PILE_STEP + T.PICK_GRIP_DEPTH - PAD_TIP_PAST_TCP   # fingertips PICK_GRIP_DEPTH onto the page
    return dict(
        pinch_y=pinch_y,
        pre_y=pinch_y - 0.07,
        drawn_y=pinch_y - (T.SHEET_Y - T.PILE_STEP - T.PICK_OVERLAP),   # far edge then PICK_OVERLAP over the shelf's edge
        carry_z=T.SHELF_TOP + T.N * T.SHEET_T + T.PICK_LIFT,
        tip=PAD_TIP_PAST_TCP,
    )


def script(T, left: Hand, right: Hand):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    G = geometry(T)
    down = grip_rotation(0.0)
    open_grip = T.PICK_OPEN / T.GRIP_FULL_OPEN
    on_pile = T.N                     # pages on the pile
    sticks_out = True                 # whether the pile's top page sticks out past the shelf
    shelf_z = T.SHELF_TOP + T.SHEET_T / 2
    left_park = (T.PILE_X, T.PUSH_Y + 0.04, T.SHELF_TOP + T.N * T.SHEET_T + G["tip"] + 0.012)   # beside the pile
    first = True
    for source, target, _number in moves(T):
        from_pile = source == "pile"
        x_src = T.PILE_X if from_pile else T.SPOTS[source]
        z_src = T.SHELF_TOP + (on_pile - 1) * T.SHEET_T + T.SHEET_T / 2 if from_pile else shelf_z
        pre = (x_src, G["pre_y"], z_src)
        # 1. The right hand comes level beside the page. The first time it swings there joint by joint: a
        #    straight-line move with the hand turning on the way passes through poses its wrist cannot reach.
        #    If the pile's top page does not stick out yet, the left hand slides the pile one step meanwhile.
        if first:
            q_pre = right.joints_for(pre, LEVEL_LEFTWARD, RIGHT_LEVEL_SEED)
            come = right.move_joints(q_pre, 3.2, grip=open_grip)
            left_part = left.move(left_park, 2.5, rot=down, grip=0.0)
            first = False
        else:
            home = (x_src, G["pre_y"], G["carry_z"])
            come = chain(right.move(home, 0.8 + 4.0 * abs(x_src - right.pos[0]), grip=open_grip), right.move(pre, 0.7))
            left_part = wait(0.05)
        if from_pile and not sticks_out:
            tip_z = T.SHELF_TOP + on_pile * T.SHEET_T + G["tip"]
            left_part = chain(left_part, left.move((T.PILE_X, T.PUSH_Y, tip_z + 0.012), 0.6),
                              left.move((T.PILE_X, T.PUSH_Y, tip_z - T.PUSH_PRESS), 0.7),
                              left.move((T.PILE_X, T.PUSH_Y - T.PILE_STEP, tip_z - T.PUSH_PRESS), 1.2),
                              left.move((T.PILE_X, T.PUSH_Y - T.PILE_STEP, tip_z + 0.012), 0.5),
                              left.move(left_park, 0.6))
            sticks_out = True
        yield from together(left_part, come)
        # 2. Pinch the edge that sticks out and lift the page.
        yield from right.move((x_src, G["pinch_y"], z_src), 1.2)
        yield from right.set_grip(0.0, 0.7)
        yield from wait(0.2)
        yield from right.move((x_src, G["pinch_y"], G["carry_z"]), 0.8)
        if from_pile:
            on_pile -= 1
            sticks_out = False
        if target == "tray":
            # 3a. Draw the page out, carry it to the tray, lower it until its far edge is just above the shelf's
            #     edge (or the pile's, while pages are left there), open fully and back away.
            rest = T.SHELF_TOP + on_pile * T.SHEET_T
            yield from right.move((x_src, G["drawn_y"], G["carry_z"]), 1.2)
            if abs(T.TRAY_X - x_src) > 1e-6:
                yield from right.move((T.TRAY_X, G["drawn_y"], G["carry_z"]), 0.9 + 4.0 * abs(T.TRAY_X - x_src))
            yield from right.move((T.TRAY_X, G["drawn_y"], rest + T.PICK_SET_GAP), 0.8)
            yield from right.set_grip(1.0, 0.4)
            yield from right.move((T.TRAY_X, G["drawn_y"] - T.PICK_CLEAR, rest + T.PICK_SET_GAP), 0.5)
            yield from wait(0.6)
        else:
            # 3b. Carry the page along the shelf and set it down there, its edge still sticking out.
            x_spot = T.SPOTS[target]
            yield from right.move((x_spot, G["pinch_y"], G["carry_z"]), 0.9 + 4.0 * abs(x_spot - x_src))
            yield from right.move((x_spot, G["pinch_y"], shelf_z + 0.0005), 0.8)
            yield from right.set_grip(1.0, 0.4)
            yield from right.move((x_spot, G["pre_y"], shelf_z + 0.0005), 0.6)
            yield from wait(0.2)
        yield from right.move((right.pos[0], right.pos[1], G["carry_z"]), 0.6)


def sweep_script(T, left: Hand, right: Hand):
    """A shortcut that must not score: the left hand presses on the pile and slides all of it over the shelf's edge,
    so every page falls into the tray below in the pile's order. The right hand does nothing."""
    from yam_arena.demo import grip_rotation

    G = geometry(T)
    tip_z = T.SHELF_TOP + T.N * T.SHEET_T + G["tip"]
    yield from left.move((T.PILE_X, T.PUSH_Y, tip_z + 0.012), 2.5, rot=grip_rotation(0.0), grip=0.0)
    yield from left.move((T.PILE_X, T.PUSH_Y, tip_z - T.PUSH_PRESS), 0.8)
    yield from left.move((T.PILE_X, T.SHELF_EDGE + 0.012, tip_z - T.PUSH_PRESS), 6.0)
    yield from left.move((T.PILE_X, T.SHELF_EDGE + 0.012, tip_z + 0.03), 0.6)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 3000, record: bool = True, window: bool = False,
        hold_s: float = 2.5, sweep: bool = False) -> dict:
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
    obs = emb.reset(Scene(id="pick-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = sweep_script(T, left, right) if sweep else script(T, left, right)
    frames: dict[str, list] = {}

    def snap(o) -> None:
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    def report() -> dict:
        from yam_arena.scene import object_states   # importable only once the simulator has started

        states = object_states(emb._env, emb.task.names)
        pages = {n: {"number": T.NUMBER_OF[n], "where": T.where(states[n]), "tilt_deg": round(T.tilt_deg(states[n]), 1),
                     "pos_cm": [round(100 * float(v), 2) for v in states[n].pos]} for n in T.PAGES}
        return {"pages": pages, "tray_pile_bottom_first": T.tray_pile(states), "in_order": T.in_order(states)}

    snap(obs)
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
        if steps % 100 == 0:
            print(f"step {steps} score {info.get('score')}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    end = report()
    result = {"lot": T.LOT, "mode": "sweep" if sweep else "file",
              "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              **end}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"pick_{cam}.mp4")
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
    ap.add_argument("--max-steps", type=int, default=3000)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--sweep", action="store_true", help="instead of sorting, push the whole pile off the shelf")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, sweep=a.sweep)
