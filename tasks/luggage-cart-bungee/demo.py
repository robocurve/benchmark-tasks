"""Scripted full-score demo for the luggage-cart bungee task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2000] [--no-record] [--cam-width 640 --cam-height 352]

It plans from the fixed, published start layout (including where the two side tubes are) and, while it runs, reads
only what a camera or the robot itself could give: the poses of the two hooks and the carton, and the arms' own
commanded and measured joint angles. It never reads the cord's tension or any flag of the scorer.

1. The left arm picks up the left hook by its crimp sleeve (hand pitched 45 degrees back along the shank) and sets it
   on the left side tube from behind: bend reached past the tube and outward, slid inward behind the tube until the
   shank meets it, then drawn forward so the tube sits in the bend. It keeps hold for now.
2. The right arm picks up the right hook and, stretching the cord round the carton's front, sets it on the right
   side tube the same way; it lets go once the tube is between shank and tip, and the cord draws the hook home.
   Under the cord's pull the arm gives several centimetres; the demo adds the gap between commanded and measured
   joint angles back into its command and checks the hook's pose against the tube before letting go.
3. The left arm lets go; both arms move away and wait while the scorer tilts, rocks, lifts and tips the cart.

Variants (DEMO_VARIANT=...): nothing (both arms stay still); letgo_first (the left hook is let go as soon as it is
home, before the cord is stretched); drop (the right hook is let go while stretched, beside its tube but not round
it); hold (strapped correctly but the left gripper keeps hold of its hook); low (both hooks aimed 5.5 cm above the
table, near the bottom cross tube). Measurements, not strategies (they edit the scene): unstrapped (the scorer's test
started with no cord on) and seated_probe (both hooks put home directly, then the test).
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

    REACH_X = 0.55      # never command the hand further out than this (beyond it the arm cannot follow)

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.bias = np.zeros(6)        # joint offsets that make up for the arm's give under load (from its own
                                       # joint readings: commanded minus measured)

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q + self.bias, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None):
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        pos = np.asarray(pos, float).copy()
        pos[0] = min(pos[0], self.REACH_X)
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)

    def stiffen(self, measured, rounds: int = 4, gain: float = 0.8):
        """Hold the commanded pose under load: add the gap between commanded and measured joint angles to the
        command, a few times (the robot's own joint readings only)."""
        for _ in range(rounds):
            yield from wait(0.3)
            err = self.q - measured(self.name)
            if np.max(np.abs(err)) < 0.004:
                break
            self.bias = np.clip(self.bias + gain * err, -0.6, 0.6)
            self.cmd = np.r_[self.q + self.bias, self.grip]

    def relax(self):
        self.bias = np.zeros(6)


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def wait_for(flags, key):
    while not flags.get(key):
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


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


PITCH = math.radians(45)   # the hand leans back along the hook's shank: keeps the wrist camera clear of the tube


def hand_rot(a) -> np.ndarray:
    """Hand (TCP) rotation for holding a hook whose shank points along ``a``: fingers across the crimp sleeve,
    approach pitched PITCH from straight down, leaning toward the hook's bend."""
    a = np.array([a[0], a[1], 0.0])
    a /= np.linalg.norm(a)
    f = np.cross([0.0, 0.0, 1.0], a)
    z = -math.cos(PITCH) * np.array([0.0, 0.0, 1.0]) - math.sin(PITCH) * a
    return np.column_stack([f, np.cross(z, f), z])


ENTRY_BACK = 0.045     # the hook's bend this far behind its home on the tube while it is slid in past the tube
ENTRY_OUT = 0.040      # and this far outward before it is slid in
ENTRY_IN = 0.015       # slid in this far past home: the shank stops against the tube


def script(T, ctx):
    from yam_arena.demo import grip_rotation

    variant = os.environ.get("DEMO_VARIANT", "")
    flags: dict = {}
    zh = T.TOP + 0.13                                   # hook height on the tubes: middle of the carton
    if variant == "low":
        zh = T.TOP + 0.055                              # just above the lowest cross tube

    def hook(name):
        s = ctx.states()[name]
        return np.asarray(s.pos, float), _rot(s.quat_xyzw)

    def home(side: str):
        """Where the hook sits home on its tube (origin, rotation), pointing at the carton's front corner."""
        s = 1 if side == "left" else -1
        tube = np.array([T.RAIL_X, s * T.RAIL_Y, zh])
        box = ctx.states()["carton"].pos
        corner = np.array([box[0] - T.BOX_D / 2, box[1] + s * T.BOX_W / 2, zh])
        u = corner - tube
        u[2] = 0.0
        R = T.hook_rotation(u, (0.0, 0.0, -s))
        return tube - R @ np.array([-0.002, 0.0, 0.0]), R, s

    class Held:
        """A hook in a gripper: its pose in the hand frame, and a running correction for the arm's give."""

        def __init__(self, A, name):
            p, R = hook(name)
            self.A, self.name = A, name
            self.p_in = A.rot.T @ (p - A.pos)
            self.R_in = A.rot.T @ R
            self.off = np.zeros(3)

        def hand_for(self, O, R):
            Rh = hand_rot(R[:, 0])
            return O - Rh @ self.p_in, Rh

        def go(self, O, R, duration, settle=0, tol=0.003):
            pos, rot = self.hand_for(O, R)
            yield from self.A.move(pos + self.off, rot, duration)
            for _ in range(settle):                     # look at the hook and correct what is left
                yield from self.A.stiffen(ctx.joints)
                e = O - hook(self.name)[0]
                if np.linalg.norm(e) < tol:
                    break
                self.off += 0.9 * e
                n = np.linalg.norm(self.off)
                if n > 0.10:                            # never chase more than 10 cm of give
                    self.off *= 0.10 / n
                yield from self.A.move(pos + self.off, rot, 0.5)

    def pick(A, name):
        p, R = hook(name)
        c = p + R @ T.CRIMP
        rot = hand_rot(R[:, 0])
        yield from A.move(c + [0, 0, 0.008] - 0.08 * rot[:, 2], rot, 2.5, grip=0.6)
        yield from A.move(c + [0, 0, 0.008], rot, 1.5)
        yield from A.set_grip(0.0, 1.0)
        yield from wait(0.3)
        yield from A.move(A.pos + [0, 0, 0.05], rot, 1.0)
        return Held(A, name)

    def tube_in_hook(name, side):
        """The side tube's centre at the hook's height, in the hook's frame (from the hook's seen pose and the
        tube's fixed published position)."""
        p, R = hook(name)
        s = 1 if side == "left" else -1
        return R.T @ (np.array([T.RAIL_X, s * T.RAIL_Y, p[2]]) - p)

    def align(h, side, want, R_goal, iters=8, tol=0.0025, gain=0.9):
        """Move the hook until the tube sits at ``want`` (x', y') in the hook's frame, looking after each move."""
        t = tube_in_hook(h.name, side)
        for _ in range(iters):
            if np.linalg.norm(t[:2]) > 0.12:            # the hook is not near its tube any more: stop correcting
                break
            e = np.array([want[0] - t[0], want[1] - t[1], 0.0])
            if abs(e[0]) < tol and abs(e[1]) < tol:
                break
            p, R = hook(h.name)
            dp = -(R @ e)
            dp[2] = 0.0
            h.off += gain * dp
            n = np.linalg.norm(h.off)
            if n > 0.10:
                h.off *= 0.10 / n
            yield from h.A.move(h.A.pos + gain * dp, h.A.rot, 0.6)
            yield from h.A.stiffen(ctx.joints)
            t = tube_in_hook(h.name, side)
        return t

    def hook_on(h: "Held", side: str, let_cord_draw=False):
        O, R, s = home(side)
        out = np.array([0.0, s * ENTRY_OUT, 0.0])
        back = np.array([ENTRY_BACK, 0.0, 0.0])
        yield from h.go(O + back + out + [0, 0, 0.025], R, 3.0, settle=2, tol=0.005)
        yield from h.go(O + back + out, R, 1.5, settle=3, tol=0.004)
        # bend and tip behind the tube, beside it: the tube 3.5 cm past the tip's end, clear of the tip
        t = yield from align(h, side, (0.035, -0.028), R)
        ctx.note(f"{side} hook beside its tube, tube at x' {100 * t[0]:.1f} y' {100 * t[1]:.1f} cm")
        # slide it in behind the tube until the shank stops against the tube
        t = yield from align(h, side, (0.035, 0.002), R, iters=10)
        ctx.note(f"{side} hook round the tube, tube at x' {100 * t[0]:.1f} y' {100 * t[1]:.1f} cm")
        if let_cord_draw:                               # stretched: let go and the cord draws the hook home
            yield from h.A.set_grip(0.6, 0.5)
            h.A.relax()
            yield from wait(0.5)
            ctx.note(f"{side} hook home")
            return
        # draw it forward: the tube passes the tip and goes into the bend
        t = yield from align(h, side, (-0.002, 0.002), R, iters=8, tol=0.003)
        ctx.note(f"{side} hook home, tube at x' {100 * t[0]:.1f} y' {100 * t[1]:.1f} cm")

    def seated_probe():
        """Diagnostic only (not a strategy, edits the scene): put both hooks home on their tubes and watch them."""
        from partkit import set_pose
        for side in ("left", "right"):
            O, R, s = home(side)
            set_pose(ctx.emb._env, f"hook_{side}", O, T._quat_from_matrix(R))
        for k in range(440):
            if k % 10 == 0 or (T._S['phase'] == 'test' and k % 3 == 0):
                out = []
                for side in ("left", "right"):
                    p, R = hook(f"hook_{side}")
                    t = tube_in_hook(f"hook_{side}", side)
                    v = np.linalg.norm(ctx.states()[f"hook_{side}"].lin_vel)
                    out.append(f"{side} v {1000 * v:.0f}mm/s z {1000 * (p[2] - T.TOP):.1f} tilt {math.degrees(math.acos(min(1, abs(R[2, 2])))):.1f} "
                               f"tube x' {1000 * t[0]:.1f} y' {1000 * t[1]:.1f}")
                bv = np.linalg.norm(ctx.states()["carton"].lin_vel)
                print(f"PROBE {k} {T._S['phase']} tilt {math.degrees(T._S['tilt']):.0f} rock {math.degrees(T._S['rock']):.0f} fwd {math.degrees(T._S['fwd']):.0f} shift {100 * float(np.linalg.norm(T._S['last']['bottom'] - T._S['box_ref'])) if T._S['box_ref'] is not None else 0:.1f} "
                      f"T {T._S['tension']:.2f} mode {T._S['mode']} box v {1000 * bv:.0f} " + " | ".join(out), flush=True)
            yield

    def left():
        A = ctx.left
        if variant == "seated_probe":
            yield from seated_probe()
            return
        if variant in ("nothing", "unstrapped"):
            yield from wait(80)
            return
        h = yield from pick(A, "hook_left")
        yield from hook_on(h, "left")
        if variant == "letgo_first":
            yield from A.set_grip(0.6, 0.6)
            A.relax()
            yield from A.move(A.pos + [0, 0.06, 0.02], A.rot, 1.5)
            flags["left_on"] = True
            yield from wait_for(flags, "right_done")
            yield from park(A)
            return
        flags["left_on"] = True
        yield from wait_for(flags, "right_done")
        if variant == "hold":
            yield from wait(60)
            return
        yield from wait(0.5)
        yield from A.set_grip(0.6, 0.8)
        A.relax()
        yield from A.move(A.pos + [0, 0.05, 0.0], A.rot, 1.0)
        yield from A.move(A.pos + [0, 0.0, 0.03], A.rot, 1.0)
        yield from park(A)

    def park(A):
        s = 1 if A.name == "left" else -1
        yield from A.move(np.array([0.25, s * 0.30, T.TOP + 0.20]), grip_rotation(0.0), 2.5, grip=1.0)

    def right():
        A = ctx.right
        if variant == "seated_probe":
            return
        if variant in ("nothing", "unstrapped"):
            if variant == "unstrapped":
                yield from wait(1.0)
                T._S.update(phase="test", test_t=0.0, box_ref=T._S["last"]["bottom"].copy(), test_ok=True,
                            max_shift=0.0)
            yield from wait(80)
            return
        h = yield from pick(A, "hook_right")
        yield from wait_for(flags, "left_on")
        O, R, s = home("right")
        # round the front of the carton, then back along its right side
        if variant == "drop":
            yield from h.go(O + [0.0, -ENTRY_OUT, 0.0] + [-0.06, 0, 0], R, 2.0)
            ctx.note("right hook let go while stretched")
            yield from A.set_grip(1.0, 0.2)
            yield from A.move(A.pos + [0, -0.05, 0.02], A.rot, 1.0)
            flags["right_done"] = True
            yield from park(A)
            return
        yield from hook_on(h, "right", let_cord_draw=True)
        yield from A.move(A.pos + [0, -0.05, 0.0], A.rot, 1.0)
        yield from A.move(A.pos + [0, 0.0, 0.03], A.rot, 1.0)
        flags["right_done"] = True
        yield from park(A)

    yield from both(left(), right())


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.steps = 0

    def joints(self, arm: str) -> np.ndarray:
        """The arm's six measured joint angles (what the policy observes)."""
        return np.asarray(self.emb._env.unwrapped.scene[f"{arm}_robot"].data.joint_pos[0].cpu(), float)[:6]

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, ["hook_left", "hook_right", "carton"])

    def note(self, what: str) -> None:
        """Measurement only (not used by the demo): what the scorer sees now."""
        T = self.emb.task.module
        ev = T._S.get("last", {})
        print(f"NOTE {what} step {self.steps}: tension {T._S['tension']:.1f} N, mode {T._S['mode']}, seats "
              f"{ev.get('seats')}, crossings {[round(100 * c, 1) for c in ev.get('cross', [])]} cm, score "
              f"{T._S['score']}", flush=True)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 2000, record: bool = True, cam_width: int = 320,
        cam_height: int = 176) -> dict:
    from types import SimpleNamespace

    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.scoring import task_score

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb)
    ctx.note("start")
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, res = 0, None
    with (out / "actions.jsonl").open("w") as log:
        while steps < max_steps:
            try:
                next(gen)
            except StopIteration:
                pass
            act = ctx.action()
            res = emb.step(act)
            steps = ctx.steps = steps + 1
            log.write(json.dumps({"step": steps, "action": np.asarray(act.data, float).round(6).tolist()}) + "\n")
            snap(res.observation)
            if steps == 1:
                print("camera brightness", {c: round(float(np.asarray(i).mean()), 1)
                                            for c, i in res.observation.images.items()}, flush=True)
            if steps % 50 == 0:
                s = ctx.states()
                S = task._S
                print(f"step {steps} score {res.info.get('score')} phase {S['phase']} T {S['tension']:.1f} "
                      f"mode {S['mode']} seats {S.get('last', {}).get('seats')} hl {np.round(s['hook_left'].pos, 3).tolist()} "
                      f"hr {np.round(s['hook_right'].pos, 3).tolist()} box {np.round(s['carton'].pos, 3).tolist()} "
                      f"tcp_l {np.round(ctx.left.pos, 3).tolist()} tcp_r {np.round(ctx.right.pos, 3).tolist()}",
                      flush=True)
            if res.terminated:
                break
    scored = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    print(f"task_score: {scored.value:g} ({scored.explanation}) after {steps} steps", flush=True)
    result = {"task_score": scored.value, "explanation": scored.explanation, "steps": steps,
              "variant": os.environ.get("DEMO_VARIANT", ""), "max_test_shift_cm": round(100 * task._S["max_shift"], 2),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        cams = sorted(frames, key=lambda c: ("left" not in c, "top" not in c))   # left, top, right
        _write_mp4([np.hstack([frames[c][i] for c in cams]) for i in range(len(frames[cams[0]]))], out / "demo.mp4")
        imageio.imwrite(out / "setup.png", np.hstack([frames[c][0] for c in cams]))
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--cam-width", type=int, default=320)
    ap.add_argument("--cam-height", type=int, default=176)
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=a.cam_width, cam_height=a.cam_height)
