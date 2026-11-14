"""Scripted full-score demo for the hand can opener task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record]

Closed loop. It plans from the fixed start layout and, while it runs, reads the poses of the can, the lid and the
opener and the opener's two joint angles (how far its handles are open, where its key's wings point), all of which a
camera sees, and the arms' own commands. It never reads the clamp, the cut or any force.

Left arm: grips both handles from above where the opener lies, squeezes them shut to carry it, turns it upright
in the air, opens the handles part way, sets the head on the front of the can's rim, squeezes the handles shut
(this clamps the rim between the wheels and pierces the lid) and keeps them squeezed. Right arm: grips the wing key
end-on, turns it clockwise (seen from its end) by rolling its wrist half a turn, opens, rolls back, grips again, and
so on, until the lid has dropped free (seen from the lid's height) or the can has turned one full turn plus a margin
(seen from the stripe on its label). Then the left arm lets the handles open, lifts the opener off, lays it flat on
the table where it started and lets go. Both arms move clear.

Writes demo.mp4 (the three camera views side by side), setup.png, actions.jsonl and result.json.
Variants (DEMO_VARIANT=...): nothing; no_squeeze (head set on the rim, handles held open, key turned); let_go
(squeezed, then the handles released before the key is turned); no_turn (squeezed and held, key never turned);
backwards (key turned anticlockwise); half (stop after half a turn of the can); knock (shove the can over).
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
PAD_GAP = 2 * 0.04695           # finger pads apart when fully open (m)
ROLL_MIN, ROLL_MAX = math.radians(-20), math.radians(200)   # wrist roll range used on the key (joint 6 within limits)
ROLL_SPEED = 2.5                # rad/s while turning the key
STOP_TURN = math.radians(360 + 12)
MAX_GRASPS = 20                 # half-turns of the key at most (a full cut takes about 14)


def rot_from(closing, approach) -> np.ndarray:
    x = np.asarray(closing, float) / np.linalg.norm(closing)
    z = np.asarray(approach, float)
    z = z - (z @ x) * x
    z /= np.linalg.norm(z)
    return np.column_stack([x, np.cross(z, x), z])


def rx(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


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

    def _solve(self, pos, rot, iters=150):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=iters)
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

    def set_grip(self, grip: float, duration: float = 0.6):
        yield from self.move(self.pos, self.rot, duration, grip)

    def aim(self, pos, rot):
        self._solve(pos, rot)

    def set_q(self, q):
        """Command joint angles directly (the TCP pose follows by forward kinematics)."""
        from yam_arena import ik

        self.q = np.asarray(q, float).copy()
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]

    def move_q(self, q, duration: float, grip: float | None = None):
        """Generator: joint-space move to ``q``."""
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_q(q0 + s * (np.asarray(q) - q0))
            yield

    def better(self, pos, rots):
        """The rotation among ``rots`` the arm reaches most exactly from where it is."""
        from yam_arena import ik

        best = None
        for r in rots:
            _, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(r), self.q, base_pos=self.base, iters=300)
            if best is None or pe + 0.05 * re < best[0]:
                best = (pe + 0.05 * re, r)
        return best[1]


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


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def script(T, ctx):
    variant = os.environ.get("DEMO_VARIANT", "")
    flags: dict = {}

    def handle_gap(angle) -> float:
        """Pad gap that just touches both handles' outer faces at the grip, handles at ``angle``."""
        rel = np.array([T.GRIP_T - T.PIVOT[0], 0.004 - T.PIVOT[2]])
        z = -rel[0] * math.sin(angle) + rel[1] * math.cos(angle) + T.PIVOT[2]
        return z - (-0.016)

    def grip_for(angle, squeeze=0.004) -> float:
        return float(np.clip((handle_gap(angle) - squeeze) / PAD_GAP, 0.0, 1.0))

    def lower_face_mid(op, angle):
        """World point between the handles' outer faces at the grip (the TCP target), in the opener's frame first."""
        top = -(T.GRIP_T - T.PIVOT[0]) * math.sin(angle) + (0.004 - T.PIVOT[2]) * math.cos(angle) + T.PIVOT[2]
        return op.to_world((T.GRIP_T, 0.014, (top - 0.016) / 2))

    def left():
        A = ctx.left
        yield from wait(0.5)                             # let the scene's state refresh after the reset
        op = ctx.states()["opener"]
        a0 = -T.handle_opening(op)                       # negative: open (as in handle_gap)
        R = op.rotation()
        closing, approach = R[:, 2], -R[:, 1]           # across the handles; down onto the key-up face
        g = lower_face_mid(op, a0)
        print(f"opener start pos {np.round(op.pos, 4).tolist()} quat {np.round(op.quat_xyzw, 4).tolist()} upper "
              f"{math.degrees(a0):.1f} grip target {np.round(g, 4).tolist()}", flush=True)
        rot = A.better(g, [rot_from(closing, approach), rot_from(-closing, approach)])
        yield from A.move(g + [0, 0, 0.10], rot, 2.5, grip=1.0)
        yield from A.move(g + [0, 0, 0.001], rot, 1.5)
        yield from A.set_grip(0.0, 1.2)                  # squeeze the handles shut to carry it
        yield from wait(0.4)
        # the opener's pose in the gripper's frame (from the poses), to place it from here on
        op = ctx.states()["opener"]
        Tg = np.eye(4); Tg[:3, :3], Tg[:3, 3] = A.rot, A.pos
        To = np.eye(4); To[:3, :3], To[:3, 3] = op.rotation(), op.pos
        G_O = np.linalg.inv(Tg) @ To                     # opener in gripper frame

        def tcp_for(o_rot, o_pos):
            To_t = np.eye(4); To_t[:3, :3], To_t[:3, 3] = o_rot, o_pos
            Tg_t = To_t @ np.linalg.inv(G_O)
            return Tg_t[:3, 3], Tg_t[:3, :3]

        yield from A.move(A.pos + [0, 0, 0.15], A.rot, 1.5)
        seat = np.array([T.CAN_XY[0] - T.R_SEAM, T.CAN_XY[1], T.TOP + T.H_SEAM - T.SHOE_Z])   # shoe on the seam
        p, r = tcp_for(T._USE, seat + [-0.03, 0, 0.06])
        yield from A.move(p, r, 3.0)                     # turn it upright on the way, head toward the can
        if variant == "knock":
            return
        p, r = tcp_for(T._USE, seat + [0, 0, 0.04])
        yield from A.move(p, r, 1.5)
        # open the handles part way so the wheels can go either side of the rim; re-measure the grasp
        yield from A.set_grip(grip_for(math.radians(-23), squeeze=0.003), 0.8)
        yield from wait(0.5)
        op = ctx.states()["opener"]
        Tg = np.eye(4); Tg[:3, :3], Tg[:3, 3] = A.rot, A.pos
        To = np.eye(4); To[:3, :3], To[:3, 3] = op.rotation(), op.pos
        G_O = np.linalg.inv(Tg) @ To
        p, r = tcp_for(T._USE, seat + [0, 0, 0.02])
        yield from A.move(p, r, 1.0)
        p, r = tcp_for(T._USE, seat)
        yield from A.move(p, r, 1.2)
        yield from wait(0.3)
        if variant == "no_squeeze":
            flags["clamped"] = True
            yield from wait_for(flags, "turned")
            flags["off"] = True
            return
        # squeeze shut; the lower finger stays under the lower handle, so the TCP comes down half the closing
        w0 = A.grip * PAD_GAP
        w1 = max(0.0, handle_gap(0.0) - 0.006)
        p0 = A.pos.copy()
        n = round(1.2 * CONTROL_HZ)
        for k in range(1, n + 1):
            s = 0.5 - 0.5 * math.cos(math.pi * k / n)
            w = w0 + s * (w1 - w0)
            A.grip = w / PAD_GAP
            A.aim(p0 + np.array([0.0, 0.0, -(w0 - w) / 2]), A.rot)
            yield
        A.grip = 0.0
        A.aim(A.pos, A.rot)
        yield from wait(0.3)
        yield from A.move(A.pos + [0, 0, -0.003], A.rot, 0.5)      # let the can stand on the table
        if variant == "let_go":                          # let go of the handles before the key is turned
            yield from A.set_grip(1.0, 0.6)
            yield from A.move(A.pos + [-0.08, 0.06, 0.06], A.rot, 1.5)
            flags["clamped"] = True
            return
        flags["clamped"] = True
        yield from wait_for(flags, "turned")
        # let go of the squeeze: the handles spring open and the wheels leave the rim; lift the opener off
        yield from A.set_grip(grip_for(math.radians(-23), squeeze=0.003), 0.8)
        yield from wait(0.4)
        yield from A.move(A.pos + [0, 0, 0.06], A.rot, 1.2)
        yield from A.move(A.pos + [-0.06, 0, 0.0], A.rot, 1.0)
        yield from A.set_grip(0.0, 0.8)                   # squeeze again to carry it
        op = ctx.states()["opener"]
        Tg = np.eye(4); Tg[:3, :3], Tg[:3, 3] = A.rot, A.pos
        To = np.eye(4); To[:3, :3], To[:3, 3] = op.rotation(), op.pos
        G_O = np.linalg.inv(Tg) @ To
        start = np.array(T.OPENER_START) + [0, 0, 0.006]
        p, r = tcp_for(T._LYING, start + [0, 0, 0.08])
        yield from A.move(p, r, 3.0)                      # lay it flat again, back where it was
        p, r = tcp_for(T._LYING, start)
        yield from A.move(p, r, 1.5)
        yield from A.set_grip(1.0, 0.8)
        yield from A.move(A.pos + [0, 0, 0.10], A.rot, 1.5)
        yield from A.move(A.pos + [-0.10, 0.05, 0.05], A.rot, 1.5)
        flags["off"] = True

    def right():
        A = ctx.right
        yield from wait_for(flags, "clamped")
        if variant == "no_turn":
            yield from wait(3.0)
            flags["turned"] = True
            return
        sign = -1.0 if variant == "backwards" else 1.0
        can0 = ctx.can_yaw()
        turned, last = 0.0, can0
        lid_z0 = ctx.states()["lid"].pos[2]
        stop = math.pi if variant == "half" else STOP_TURN
        fwd = rot_from((0, 0, 1), (1, 0, 0))
        from yam_arena import ik

        J6_LO, J6_HI = math.radians(-115), math.radians(115)
        first = True
        q_ref = None
        for _ in range(MAX_GRASPS):
            st = ctx.states()
            op = st["opener"]
            key_c = op.to_world(T.KEY_C)
            kappa = op.joints["key"]
            # the wings' thickness axis is the opener's z turned by the key angle about the opener's y (world -x
            # with the opener seated at the front): a roll phi about world x lines the fingers up when
            # phi = -kappa (mod pi). Joint 6 turns the gripper about its own axis, which runs through the grip
            # point along the key's axis, so the half-turn is made by joint 6 alone.
            if q_ref is None:                        # the reference posture, and which way joint 6 rolls the gripper
                phi_ref = _wrap(-kappa)
                q_ref, _, _ = ik.solve(key_c, ik.matrix_to_quat_xyzw(rx(phi_ref) @ fwd), A.q, base_pos=A.base,
                                       iters=400)
                qd, _, _ = ik.solve(key_c, ik.matrix_to_quat_xyzw(rx(phi_ref + 0.1) @ fwd), q_ref, base_pos=A.base,
                                    iters=300)
                j6_dir = 1.0 if qd[5] > q_ref[5] else -1.0
            d = sign * j6_dir                         # joint 6 moves this way while the key is turned
            best = None
            for k in range(-3, 4):
                phi = _wrap(-kappa) + k * math.pi
                seed = q_ref.copy()
                seed[5] += j6_dir * (phi - phi_ref)   # a roll about the key's axis changes joint 6 only
                end6 = J6_HI if d > 0 else J6_LO      # turn as far as joint 6 allows, at least 45 degrees
                if not (J6_LO <= seed[5] <= J6_HI and (end6 - seed[5]) * d >= math.radians(45)):
                    continue
                q, pe, re = ik.solve(key_c, ik.matrix_to_quat_xyzw(rx(phi) @ fwd), seed, base_pos=A.base, iters=300)
                if pe > 0.003 or re > 0.03 or abs(q[5] - seed[5]) > 0.2:
                    continue
                if best is None or (q[5] * d) < (best[5] * d):
                    best = q
            if best is None:
                print("no reachable grasp on the key", flush=True)
                break
            q = best
            if first:
                yield from A.move(key_c + [-0.07, 0, 0.02], ik.tcp_pose(q, A.base)[1], 2.5, grip=1.0)
                yield from A.move(key_c + [-0.07, 0, 0], ik.tcp_pose(q, A.base)[1], 1.0)
                yield from A.move(key_c, ik.tcp_pose(q, A.base)[1], 1.2)
                yield from A.move_q(q, 0.3)
                first = False
            else:
                # roll back with the fingers open (they clear the wings), within joint 6's range
                yield from A.move_q(q, max(0.3, abs(q[5] - A.q[5]) / 4.0))
            yield from A.set_grip(0.0, 0.5)
            target6 = J6_HI if d > 0 else J6_LO
            v, done = 0.0, False
            while (target6 - A.q[5]) * d > 1e-6:
                v = min(ROLL_SPEED, v + ROLL_SPEED / (0.3 * CONTROL_HZ))
                qq = A.q.copy()
                qq[5] += d * min(v / CONTROL_HZ, abs(target6 - qq[5]))
                A.set_q(qq)
                yield
                y = ctx.can_yaw()
                turned += sign * _wrap(y - last)       # the can turns anticlockwise (seen from above) as it is cut
                last = y
                if turned >= stop or ctx.states()["lid"].pos[2] < lid_z0 - 0.003:
                    done = True
                    break
            yield from A.set_grip(1.0, 0.4)
            if done:
                break
        print(f"key stop: step {ctx.steps}, can turned {math.degrees(turned):.1f} deg", flush=True)
        yield from A.move(A.pos + [-0.08, 0, 0.0], A.rot, 1.2)
        yield from A.move(A.pos + [0, -0.06, 0.06], A.rot, 1.2)
        flags["turned"] = True

    def knock():
        A = ctx.right
        c = np.array([T.CAN_XY[0], T.CAN_XY[1], T.TOP + 0.09])
        rot = rot_from((0, 0, 1), (1, 0, 0))
        yield from A.move(c + [-0.12, -0.0, 0.0], rot, 2.5, grip=0.0)
        yield from A.move(c + [0.0, 0.0, 0.0], rot, 1.2)
        yield from A.move(c + [-0.12, 0.0, 0.03], rot, 1.2)

    if variant == "nothing":
        for _ in range(round(148 * CONTROL_HZ)):        # stay still to the run limit
            yield
    elif variant == "knock":
        yield from both(left(), knock())
    else:
        yield from both(left(), right())


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.steps = 0

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def can_yaw(self) -> float:
        """The can's turn about its axis (rad), from its pose (what the stripe on its label shows)."""
        r = self.states()["can"].rotation()
        return math.atan2(r[1, 0], r[0, 0])

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _actual(emb) -> str:
    """Debug: where each arm's TCP really is (world, m)."""
    from yam_arena.runtime import tcp_world

    base = emb._env.unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    return " ".join(f"{arm[0]}_act {np.round(tcp_world(base.scene[arm]) - origin, 3).tolist()}"
                    for arm in ("left_robot", "right_robot"))


def _fingers(emb) -> str:
    base = emb._env.unwrapped
    out = []
    for arm in ("left_robot", "right_robot"):
        r = base.scene[arm]
        q = r.data.joint_pos[0].cpu().numpy() if hasattr(r.data.joint_pos, "cpu") else np.asarray(r.data.joint_pos)[0]
        out.append(round(float(-2 * q[list(r.joint_names).index("joint7")]) * 1000, 1))
    return out


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 2.0) -> dict:
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
    task = emb.task.module
    obs = emb.reset(Scene(id="demo", instruction=task.INSTRUCTION))
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, res = 0, None, None
    debug = bool(os.environ.get("DEMO_DEBUG"))
    with (out / "actions.jsonl").open("w") as log:
        while steps < max_steps:
            try:
                next(gen)
            except StopIteration:
                ended = steps if ended is None else ended
            act = ctx.action()
            res = emb.step(act)
            steps = ctx.steps = steps + 1
            log.write(json.dumps({"step": steps, "action": np.asarray(act.data, float).round(6).tolist()}) + "\n")
            snap(res.observation)
            if steps % 50 == 0 or (debug and steps % 10 == 0):
                s = ctx.states()
                op = s["opener"]
                err, tilt = task.seat_error(op, s["can"])
                print(f"step {steps} score {res.info.get('score')} cut {task.cut_fraction():.3f} "
                      f"clamped {task._S['clamped']} ready {task._S['ready']} open_deg "
                      f"{math.degrees(task.handle_opening(op)):.1f} key_deg {math.degrees(op.joints['key']):.0f} "
                      f"can_yaw {math.degrees(ctx.can_yaw()):.0f} seat_err_mm {np.round(err * 1000, 1).tolist()} "
                      f"tilt {math.degrees(tilt):.1f} op {np.round(op.pos, 3).tolist()} lid_z "
                      f"{s['lid'].pos[2]:.4f} can {np.round(s['can'].pos, 4).tolist()} "
                      f"tcp_l {np.round(ctx.left.pos, 3).tolist()} tcp_r {np.round(ctx.right.pos, 3).tolist()} "
                      f"{_actual(emb) if debug else ''} grip_l {ctx.left.grip:.2f} fingers_mm {_fingers(emb)}",
                      flush=True)
            if res.terminated:
                break
            if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
                break
    scored = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    print(f"task_score: {scored.value:g} ({scored.explanation}) after {steps} steps", flush=True)
    result = {"task_score": scored.value, "explanation": scored.explanation, "steps": steps,
              "variant": os.environ.get("DEMO_VARIANT", ""), "cut_fraction": task.cut_fraction(),
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
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
