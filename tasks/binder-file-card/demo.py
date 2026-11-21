"""Scripted full-score demo for the file-a-card-in-a-ring-binder task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1800] [--no-record] [--cam 640x352]

Both arms open the rings: they pinch the two halves of the far ring, the left hand the left half and the right hand the
right half, and pull them apart past the mechanism's snap (the plates are shared, so all three rings open). The right
arm then takes the card out of the step file by the middle of its top edge, holds it upright over the rings with its
punched edge down between the open tips, lowers it into the rings and swings it over toward the right page, so that
the three right ring halves go through the three holes. It lets go and the card falls onto the right page, hanging on
the rings. The left arm then squeezes the middle ring's two halves together until the mechanism snaps the rings shut,
and both arms draw back.
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


def _rot_axis(axis, angle) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def _cols(x, z) -> np.ndarray:
    """A TCP rotation with the fingers closing along ``x`` and pointing along ``z``."""
    x, z = np.asarray(x, float), np.asarray(z, float)
    return np.column_stack([x, np.cross(z, x), z])


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it. With
    ``solve=False`` it only records where each move ends (for the offline reach check)."""

    def __init__(self, name: str, base_pos, solve: bool = True):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base, self.solve = name, np.asarray(base_pos, float), solve
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.ends: list[tuple[np.ndarray, np.ndarray, str]] = []

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, label: str = ""):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        pos = np.asarray(pos, float)
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            p, r = p0 + s * (pos - p0), ik.slerp_matrix(r0, rot, s)
            if self.solve:
                self._solve(p, r)
            else:
                self.pos, self.rot = p, r
            yield
        self.ends.append((pos.copy(), np.asarray(rot, float).copy(), label))

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip, "grip")


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


# The demo runs open loop: it reads no simulator state. Where the card sits in the right hand after it is taken out of
# the step file (and how far it has turned in the fingers) is measured once with DEMO_CALIBRATE=1 (which reads the
# simulator's poses at that moment and prints them) and fixed here. The layout is fixed (object_jitter=0) and the
# simulation deterministic.
CAL = {
    "card_in_hand": (0.0002, -0.00143, 0.00026),   # the TCP further toward the card's far edge, its far end and its back (m)
    "card_turn_deg": -0.07374,          # the card turned about x in the fingers from where it was planned (degrees)
}
CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
_MEASURED: dict = {}

TIPS = 0.013               # the TCP sits this far short of the pad tips
PINCH_DEG = 50.0           # a ring half is pinched this far up from its outermost point, from outside along its radius,
PINCH_IN = 0.003           # ... the pad tips this far inside the wire's centre line, the fingers closing along x
PINCH_RING = (2, 2)        # the ring each hand pinches (left, right): 0 the near ring, 2 the far one
PINCH_SIGN = (-1.0, -1.0)    # which way the fingers' x axis points (+-world x), for each arm's wrist
PULL = 0.055   # each pinched half's hand is moved this far outward (the arms are
                           # compliant: they pull only as hard as they are commanded past where the half stops)
GRAB_DOWN = 0.012          # holding the card, the TCP this far below its top edge
BEYOND = 0.006            # the card's back this far beyond the right tips' ends before it slides onto them
SWING_PAST_TIP = 30.0      # the card is swung this far (degrees, about the ring's centre) past the right tips
SQUEEZE_TIP_Z = 0.004      # squeezing a ring shut: the pad tips this far above the rings' centre


def _ex(angle):
    return _rot_axis((1.0, 0.0, 0.0), angle)


def card_frame(psi):
    """The card's rotation when it lies along polar angle ``psi`` (rad, in the y-z plane, from +y toward +z): its y
    axis (from the punched edge up the card) along (0, cos psi, sin psi)."""
    up = np.array([0.0, math.cos(psi), math.sin(psi)])
    return np.column_stack([[1.0, 0.0, 0.0], up, np.cross([1.0, 0.0, 0.0], up)])


def hold_rot(card_r):
    """The TCP rotation holding a card of rotation ``card_r`` by its top edge: pointing down the card, the fingers
    closing through it."""
    return _cols(card_r[:, 2], -card_r[:, 1])


def open_geometry(task):
    """The right ring halves' centre (world y, z) and their tips' polar angle about it, rings fully open."""
    T = task
    pl, pr = T.plate_poses(-T.OPEN_DEG)
    c = pr[0] + pr[1] @ (np.array([T.XC, T.YM, T.ZC]) - T.H0)
    w = T.wire_points(pl, pr)
    tip_r, tip_l = w[(1, -1)][-1], w[(1, 1)][-1]
    psi_tip = math.atan2(tip_r[2] - c[2], tip_r[1] - c[1])
    mid = (tip_r + tip_l) / 2
    psi_mid = math.atan2(mid[2] - c[2], mid[1] - c[1])
    return c, psi_tip, psi_mid


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    L, R = ctx.left, ctx.right
    T = task
    ez = np.array([0.0, 0.0, 1.0])
    ey = np.array([0.0, 1.0, 0.0])
    rot_squeeze = grip_rotation(0.0)             # pointing down, the fingers closing along y

    def measure(key, value):
        if CALIBRATE:
            _MEASURED[key] = [round(float(v), 5) for v in np.atleast_1d(value)]
            print("CAL so far = " + json.dumps(_MEASURED), flush=True)

    # 1. Open the rings: the left hand pinches the left half of the far ring, the right hand its right half, each
    #    from outside and above at 50 degrees up its half (the pads reach past the wire into the ring
    #    and cover only that half), and both pull outward past the snap.
    def pinch(side, ring, sign):
        t = math.radians(PINCH_DEG)
        u = np.array([0.0, side * math.cos(t), math.sin(t)])          # out from the ring's centre
        pos = np.array([T.RING_X[ring], T.YM, T.ZC]) + (T.RC - PINCH_IN + TIPS) * u
        return pos, _cols((sign, 0.0, 0.0), -u)

    pl, rot_pinch_l = pinch(1, PINCH_RING[0], PINCH_SIGN[0])
    pr, rot_pinch_r = pinch(-1, PINCH_RING[1], PINCH_SIGN[1])
    yield from both(L.move(pl + 0.05 * rot_pinch_l[:, 2] * -1, rot_pinch_l, 3.0, grip=0.3,
                           label="over the ring's left half"),
                    R.move(pr + 0.05 * rot_pinch_r[:, 2] * -1, rot_pinch_r, 3.0, grip=0.3,
                           label="over the ring's right half"))
    yield from both(L.move(pl, rot_pinch_l, 1.5, label="pads round the wire"),
                    R.move(pr, rot_pinch_r, 1.5, label="pads round the wire"))
    yield from both(L.set_grip(0.0, 0.8), R.set_grip(0.0, 0.8))
    yield from both(L.move(pl + PULL * ey, rot_pinch_l, 1.5, label="pull the half out"),
                    R.move(pr - PULL * ey, rot_pinch_r, 1.5, label="pull the half out"))
    yield from wait(0.5)
    if CALIBRATE:
        st = ctx.states()
        w = T.wire_points(*((st[k].pos, st[k].rotation()) for k in ("plate_l", "plate_r")))
        print("pulled: tip gaps (mm)", [round(1000 * g, 2) for g in T.tip_gaps(w)],
              "fingers", np.round(ctx.fingers(), 4).tolist(), "tcp err (mm)", ctx.tcp_errors(), flush=True)
    yield from both(L.set_grip(0.3, 0.6), R.set_grip(0.3, 0.6))
    yield from both(L.move(L.pos + 0.07 * ez, rot_pinch_l, 1.2, label="up"),
                    R.move(R.pos + 0.07 * ez, rot_pinch_r, 1.2, label="up"))
    yield from L.move(*L.home, 2.0, grip=1.0, label="out of the way")

    # 2. The right hand takes the card out of the step file by the middle of its top edge.
    r0 = T.CARD_START_R
    up0 = r0[:, 1]
    top = T.CARD_BOTTOM + T.CARD_W * up0
    grab = top - GRAB_DOWN * up0
    rot_h = hold_rot(r0)
    yield from R.move(grab + 0.025 * up0, rot_h, 3.0, grip=0.35, label="over the card's top edge")
    yield from R.move(grab, rot_h, 1.5, label="fingers round its top edge")
    yield from R.set_grip(0.0, 0.8)
    yield from wait(0.3)
    yield from R.move(grab + 0.012 * up0, rot_h, 1.0, label="lift it over the lip")
    if CALIBRATE:
        s = ctx.states()["card"]
        cr = s.rotation()
        d = cr.T @ (R.pos - s.pos)            # the TCP in the card's frame
        measure("card_in_hand", (d[1] - (T.CARD_W / 2 - GRAB_DOWN), d[0], d[2]))
        measure("card_turn_deg", math.degrees(math.atan2(cr[2, 1], cr[1, 1]) - math.atan2(r0[2, 1], r0[1, 1])))
    da, dx, db = CAL["card_in_hand"]
    turn = math.radians(CAL["card_turn_deg"])

    c, psi_tip, _ = open_geometry(T)
    hole_to_tcp = (T.CARD_W / 2 - T.HOLE_Y) - GRAB_DOWN + da   # along the card, from a hole's centre to the TCP

    def tcp_for(psi, radius, beyond=0.0):
        """TCP pose holding the card along polar angle psi about the right halves' centre, the holes' centres at
        ``radius`` from it and moved ``beyond`` (m) out of the card's plane, off its back (toward the left halves)."""
        cr = card_frame(psi)
        hole = c + radius * cr[:, 1] - beyond * cr[:, 2]
        pos = hole + hole_to_tcp * cr[:, 1] + dx * cr[:, 0] + db * cr[:, 2]
        return pos, hold_rot(card_frame(psi - turn))

    # 3. Over the rings, leaning toward the right page square to the right halves' tips and just beyond them (its
    #    back 4 mm clear of their ends), lower the punched edge down between the open tips.
    yield from R.move(*tcp_for(psi_tip, T.RC + 0.025, BEYOND), 3.5, label="card over the open rings")
    yield from R.move(*tcp_for(psi_tip, T.RC, BEYOND), 2.0, label="punched edge down between the tips")
    # 4. Slide it onto the right halves' tips, then swing it about the rings' centre toward the right page, so that
    #    the tips go through the holes and on round the wire.
    yield from R.move(*tcp_for(psi_tip, T.RC, 0.0), 1.2, label="holes onto the tips")
    psi_end = psi_tip + math.radians(SWING_PAST_TIP)
    n = 6
    for k in range(1, n + 1):
        yield from R.move(*tcp_for(psi_tip + (psi_end - psi_tip) * k / n, T.RC), 0.6, label="swing onto the rings")
    yield from wait(0.3)
    if CALIBRATE:
        s, sl, sr = (ctx.states()[k] for k in ("card", "plate_l", "plate_r"))
        w = T.wire_points((sl.pos, sl.rotation()), (sr.pos, sr.rotation()))
        print("on rings after swing:", T.rings_through_holes(s.pos, s.rotation(), w), flush=True)
    # 5. Let go: open the fingers, draw the hand out along the card and up; the card falls onto the right page.
    yield from R.set_grip(0.35, 0.5)
    yield from R.move(R.pos + 0.04 * card_frame(psi_end)[:, 1], R.rot, 1.0, label="out along the card")
    yield from R.move(R.pos + 0.03 * ez, R.rot, 0.8, label="up")
    yield from R.move(*R.home, 2.5, grip=1.0, label="back")
    yield from wait(1.0)

    # 6. The left hand squeezes the middle ring's halves together until the rings snap shut.
    sq = np.array([T.XC, T.YM, T.ZC + SQUEEZE_TIP_Z + TIPS])
    yield from L.move(sq + 0.06 * ez, rot_squeeze, 3.0, grip=1.0, label="over the middle ring")
    yield from L.move(sq, rot_squeeze, 1.5, label="fingers either side of it")
    yield from L.set_grip(0.0, 1.5)
    yield from wait(0.5)
    yield from L.set_grip(1.0, 0.8)
    yield from L.move(sq + 0.08 * ez, rot_squeeze, 1.2, label="up")
    yield from L.move(*L.home, 2.5, label="back")


def offline_plan(task):
    """The demo's moves without the simulator: each arm's list of (pos, rot, label) at the end of every move."""
    from yam_arena import station

    class _Ctx:
        left = Arm("left", station.BASES["left"], solve=False)
        right = Arm("right", station.BASES["right"], solve=False)

    ctx = _Ctx()
    steps = sum(1 for _ in script(task, ctx))
    return {"left": ctx.left.ends, "right": ctx.right.ends, "steps": steps}


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def fingers(self):
        """The four finger joint positions (m), as measured."""
        q = self.emb._env.unwrapped.scene
        out = []
        for arm in ("left_robot", "right_robot"):
            out += [float(v) for v in q[arm].data.joint_pos[0, -2:].cpu().numpy()]
        return out

    def tcp_errors(self):
        """How far (mm) each arm's TCP is from where it was last commanded."""
        from yam_arena import ik

        out = []
        for a, name in ((self.left, "left_robot"), (self.right, "right_robot")):
            q = self.emb._env.unwrapped.scene[name].data.joint_pos[0, :6].cpu().numpy()
            p, _ = ik.tcp_pose(q, a.base)
            out.append(round(float(np.linalg.norm(p - a.pos)) * 1000, 1))
        return out

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


def run(task_path: str, out_dir: str, max_steps: int = 1800, record: bool = True, cam_width: int = 640,
        cam_height: int = 352, hold_s: float = 6.0) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction=emb.task.module.INSTRUCTION))
    task = emb.task.module
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info, first_full = 0, None, False, {}, None
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        if first_full is None and float(info.get("score", 0.0) or 0.0) >= 1.0:
            first_full = steps
        snap(res.observation)
        if steps % 100 == 0:
            print(f"step {steps} score {info.get('score')}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    # The trial's score is what the task file's score() reported on the last step (task_score gives 1.0 on success,
    # which score() also is then); it is never set to 1.0 here just because the episode ended.
    final = float(info.get("score", 0.0) or 0.0)
    result = {"task_score": final, "success": success, "steps": steps, "first_full_score_step": first_full,
              "script_ended_step": ended,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if CALIBRATE:
        result["CAL"] = _MEASURED
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
    ap.add_argument("--cam", default="640x352", help="camera width x height")
    a = ap.parse_args()
    w, h = (int(v) for v in a.cam.split("x"))
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=w, cam_height=h)
