"""Two-handed sorting demo for the paper-sorting task, run in the yam_arena embodiment.

    python pick_demo.py task.py out_dir [--sheets 4] [--window] [--max-steps 2400] [--no-record] [--sweep]

For each sheet, top first:

1. The right hand, level and pointing left, pinches the top sheet by the edge that sticks out past the shelf (the
   fingertips reach 1.1 cm onto it, less than the pile's 1.8 cm step) and lifts it 3 cm straight up.
2. It draws the sheet out, carries it along the shelf to the tray of the sheet's color, and lowers it until the
   sheet's far edge is just above the shelf's edge. It opens fully and backs away, and the sheet tips off the edge
   and slides down flat into the tray.
3. The left hand, closed and pointing down, presses on the pile and slides all of it 1.8 cm to the right over the
   slick shelf, so the next sheet sticks out.

With ``--sweep`` it plays a shortcut instead, which must not score: the left hand pushes the whole pile over the
shelf's edge.

Open loop: every pose comes from the layout constants in the task file, and the order of colors from the task
file's stack order for the lot being run. Object states are read only to report where each sheet ended up. Writes
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


def pick_poses(T, i: int) -> dict:
    """Every tool-point position for sheet ``i`` (0 = top), from the task file's layout (world frame, m)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    name = T.SHEETS[i]
    color = T.COLOR_OF[name]
    xp, xt = T.PILE_X, T.TRAY_X[color]
    on_pile = T.N - i                                             # sheets on the pile before this pick
    below = T.SHELF_TOP + (on_pile - 1) * T.SHEET_T               # top of the sheets under this one
    mid = below + T.SHEET_T / 2                                   # this sheet, lying level on them
    sheet_edge = T.SHELF_EDGE - T.PILE_STEP                       # its right edge, past the shelf's edge
    pinch_y = sheet_edge + T.PICK_GRIP_DEPTH - PAD_TIP_PAST_TCP   # fingertips reach PICK_GRIP_DEPTH onto the sheet
    drawn_y = pinch_y - (T.SHEET_Y - T.PILE_STEP - T.PICK_OVERLAP)   # far edge then PICK_OVERLAP over the shelf's edge
    carry_z = mid + T.PICK_LIFT
    # What the sheet's far edge comes to rest on: the rest of the pile above the nearest tray, else the bare shelf.
    set_z = (below if xt == xp else T.SHELF_TOP) + T.PICK_SET_GAP
    tip = below + PAD_TIP_PAST_TCP                                # tool height at which closed fingertips touch the pile
    return dict(
        color=color, carry=abs(xt - xp), last=on_pile == 1,
        r_home=(xp, pinch_y - 0.07, carry_z),
        r_pre=(xp, pinch_y - 0.07, mid),
        r_pinch=(xp, pinch_y, mid),
        r_lift=(xp, pinch_y, carry_z),
        r_drawn=(xp, drawn_y, carry_z),
        r_over=(xt, drawn_y, carry_z),
        r_set=(xt, drawn_y, set_z),
        r_back=(xt, drawn_y - T.PICK_CLEAR, set_z),
        r_up=(xt, drawn_y - T.PICK_CLEAR, carry_z),
        l_park=(xp, T.PUSH_Y + 0.04, tip + 0.012),                # beside the pile, clear of a sheet being lifted
        l_over=(xp, T.PUSH_Y, tip + 0.012),
        l_press=(xp, T.PUSH_Y, tip - T.PUSH_PRESS),
        l_pushed=(xp, T.PUSH_Y - T.PILE_STEP, tip - T.PUSH_PRESS),
        l_clear=(xp, T.PUSH_Y - T.PILE_STEP, tip + 0.012),
    )


def script(T, left: Hand, right: Hand, sheets: int | None = None):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    down = grip_rotation(0.0)
    open_grip = T.PICK_OPEN / T.GRIP_FULL_OPEN
    count = T.N if sheets is None else min(sheets, T.N)
    for i in range(count):
        P = pick_poses(T, i)
        if i == 0:
            # The right hand comes level beside the pile. It swings there joint by joint: a straight-line move with
            # the hand turning on the way passes through poses its wrist cannot reach. The left hand waits beside
            # the pile.
            q_pre = right.joints_for(P["r_pre"], LEVEL_LEFTWARD, RIGHT_LEVEL_SEED)
            yield from together(left.move(P["l_park"], 2.5, rot=down, grip=0.0),
                                right.move_joints(q_pre, 3.2, grip=open_grip))
        # 1. Right hand pinches the edge that sticks out and lifts the sheet straight up.
        yield from right.move(P["r_pinch"], 1.2)
        yield from right.set_grip(0.0, 0.7)
        yield from wait(0.2)
        yield from right.move(P["r_lift"], 0.8)
        # 2. It draws the sheet out, carries it along the shelf to its tray, and lowers it until its far edge is just
        #    above the shelf's edge. It opens fully and backs away: the sheet tips off the edge into the tray.
        yield from right.move(P["r_drawn"], 1.2)
        if P["carry"] > 1e-6:
            yield from right.move(P["r_over"], 0.9 + 4.0 * P["carry"])
        yield from right.move(P["r_set"], 0.8)
        yield from right.set_grip(1.0, 0.4)
        yield from right.move(P["r_back"], 0.5)
        yield from wait(0.6)
        yield from right.move(P["r_up"], 0.6)
        # 3. Left hand presses on the pile and slides all of it one step to the right, so the next sheet sticks out,
        #    while the right hand comes back beside the pile.
        if not P["last"] and i + 1 < count:
            nxt = pick_poses(T, i + 1)
            push = chain(left.move(P["l_over"], 0.6), left.move(P["l_press"], 0.7), left.move(P["l_pushed"], 1.2),
                         left.move(P["l_clear"], 0.5), left.move(nxt["l_park"], 0.6))
            back = chain(right.move(nxt["r_home"], 0.8 + 4.0 * P["carry"], grip=open_grip),
                         right.move(nxt["r_pre"], 0.7))
            yield from together(push, back)


def sweep_script(T, left: Hand, right: Hand):
    """A shortcut that must not score: the left hand presses on the pile and slides all of it over the shelf's edge,
    so every sheet falls into the tray below the pile. The right hand does nothing."""
    from yam_arena.demo import grip_rotation

    P = pick_poses(T, 0)
    far = (P["l_press"][0], T.SHELF_EDGE + 0.012, P["l_press"][2])
    yield from left.move(P["l_over"], 2.5, rot=grip_rotation(0.0), grip=0.0)
    yield from left.move(P["l_press"], 0.8)
    yield from left.move(far, 6.0)
    yield from left.move((far[0], far[1], far[2] + 0.03), 0.6)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, window: bool = False,
        sheets: int | None = None, hold_s: float = 2.5, sweep: bool = False) -> dict:
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
    gen = sweep_script(T, left, right) if sweep else script(T, left, right, sheets)
    frames: dict[str, list] = {}

    def snap(o) -> None:
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    def report() -> dict:
        from yam_arena.scene import object_states   # importable only once the simulator has started

        states = object_states(emb._env, emb.task.names)
        return {n: {"color": T.COLOR_OF[n], "where": T.where(states[n]), "tilt_deg": round(T.tilt_deg(states[n]), 1),
                    "pos_cm": [round(100 * float(v), 1) for v in states[n].pos]} for n in T.SHEETS}

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
    result = {"lot": T.LOT, "mode": "sweep" if sweep else "sort",
              "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "sheets": end}
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
    ap.add_argument("--sheets", type=int, default=None, help="how many sheets to sort, from the top (default: all)")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--sweep", action="store_true", help="instead of sorting, push the whole pile off the shelf")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, sheets=a.sheets, sweep=a.sweep)
